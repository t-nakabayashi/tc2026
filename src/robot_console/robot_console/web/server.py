"""HTML遠隔観測UI向けの読み取り専用HTTPサーバ。

`robot_console_gui_screen_function_design.md` 8章・
`robot_console_gui_architecture_design.md` 15章の方針に従い、以下のみを提供する。

* ``GET /snapshot.json`` ``GET /map_state.json`` ``GET /sensor_panels.json``
  ``GET /health.json`` : `web.json_codec` が組み立てたJSON
* ``GET /images/{panel_id}`` : `ImageStore` の最新画像をPNGで返す
* それ以外のGET : `static_root` 配下の閲覧専用ページを配信する

GET以外のメソッド（POST/PUT/DELETE等）は405を返し、操作系APIを一切提供しない。
"""

from __future__ import annotations

import io
import json
import math
import mimetypes
import sys
import threading
import traceback
from http.server import BaseHTTPRequestHandler, ThreadingHTTPServer
from pathlib import Path
from typing import Any, Callable, Optional, Tuple
from urllib.parse import urlparse

from ..core.image_store import ImageStore
from ..core.snapshot_model import ConsoleSnapshot
from .json_codec import (
    build_health_payload,
    build_map_state_payload,
    build_sensor_panels_payload,
    build_snapshot_payload,
)

DEFAULT_HOST = '127.0.0.1'
DEFAULT_PORT = 8765

# listen backlog。既定値(socketserverの5)は、ブラウザが1オリジンに対して同時に
# 張る接続数（一般に6本）と再接続が重なるだけで溢れ、溢れたSYNはLinuxでは破棄
# されて接続が数秒単位で遅延する。観測UIは常時ポーリングするため余裕を持たせる。
REQUEST_QUEUE_SIZE = 64

# keep-aliveコネクションの受信待ちタイムアウト[s]。相手が無通告で消えた場合
# （モバイル回線やVPNのNATタイムアウト、端末のスリープ等）、既定のブロッキング
# 受信ではTCPのキープアライブが効くまでスレッドが解放されない。一定時間受信が
# 無ければコネクションを閉じ、ワーカースレッドを回収する。
KEEP_ALIVE_TIMEOUT_S = 30.0

SnapshotProvider = Callable[[], ConsoleSnapshot]

_JSON_ROUTES = {
    '/snapshot.json': build_snapshot_payload,
    '/map_state.json': build_map_state_payload,
    '/sensor_panels.json': build_sensor_panels_payload,
    '/health.json': build_health_payload,
}


def _default_static_root() -> Path:
    """`web/static/`（本モジュールと同じパッケージ配下）を返す。"""

    return Path(__file__).resolve().parent / 'static'


def _json_safe(value: Any) -> Any:
    """非有限float（NaN/Infinity）を None に落とし、有効なJSONにできる形へ変換する。

    `json.dumps` は既定で NaN / Infinity / -Infinity をそのまま出力するが、これは
    RFC 8259 のJSONとして不正であり、ブラウザの `Response.json()` は SyntaxError で
    失敗する。GPS未測位時の `hdop` や `heading_deg` のようにROSメッセージ由来の値には
    NaNが入り得るため、シリアライズ前に明示的に null へ倒す。

    Args:
        value (Any): 変換対象のペイロード（dict/list/スカラ）.

    Returns:
        Any: 非有限floatを None に置換した同型のオブジェクト.
    """

    if isinstance(value, float):
        return value if math.isfinite(value) else None
    if isinstance(value, dict):
        return {key: _json_safe(item) for key, item in value.items()}
    if isinstance(value, (list, tuple)):
        return [_json_safe(item) for item in value]
    return value


class _ObservationHTTPServer(ThreadingHTTPServer):
    """snapshot_provider / image_store / static_root を保持するHTTPサーバ。"""

    daemon_threads = True
    request_queue_size = REQUEST_QUEUE_SIZE

    def __init__(
        self,
        address: Tuple[str, int],
        handler_cls: type,
        *,
        snapshot_provider: SnapshotProvider,
        image_store: ImageStore,
        static_root: Path,
    ) -> None:
        super().__init__(address, handler_cls)
        self.snapshot_provider = snapshot_provider
        self.image_store = image_store
        self.static_root = static_root

    def handle_error(self, request: object, client_address: object) -> None:
        """クライアント都合の切断は traceback を出さずに無視する。

        ブラウザはkeep-aliveコネクションを予告なく reset / close するため、
        既定実装のままでは正常運用中でも ConnectionResetError の traceback が
        stderr に大量出力され、本当に調査すべき異常が埋もれる。
        """

        exc = sys.exc_info()[1]
        if isinstance(exc, (BrokenPipeError, ConnectionResetError, TimeoutError)):
            return
        print(
            f'[robot_console_web] リクエスト処理で例外が発生しました: {client_address}',
            file=sys.stderr,
        )
        traceback.print_exc()


class _ObservationRequestHandler(BaseHTTPRequestHandler):
    """読み取り専用エンドポイントのみを提供するリクエストハンドラ。"""

    server: _ObservationHTTPServer  # type: ignore[assignment]
    protocol_version = 'HTTP/1.1'
    timeout = KEEP_ALIVE_TIMEOUT_S

    # 応答の書き出しを開始済みかどうか。処理途中で例外が起きたときに、
    # 500応答を二重送信しないための判定に使う。
    _response_started = False

    def send_response(self, code: int, message: Optional[str] = None) -> None:
        self._response_started = True
        super().send_response(code, message)

    def do_GET(self) -> None:  # noqa: N802 - BaseHTTPRequestHandlerの命名規則に合わせる
        # 応答生成中に例外が発生すると、既定では応答を一切返さないまま
        # コネクションが閉じられる。ブラウザ側はこれを `TypeError: Failed to fetch`
        # として観測し、原因がサーバ側にあることが分からなくなるため、
        # 必ず500応答を返し、コネクションを維持できるようにする。
        self._response_started = False
        try:
            path = urlparse(self.path).path
            if path in _JSON_ROUTES:
                payload = _JSON_ROUTES[path](self.server.snapshot_provider())
                self._write_json(payload)
                return
            if path.startswith('/images/'):
                self._write_image(path[len('/images/'):])
                return
            self._write_static(path)
        except (BrokenPipeError, ConnectionResetError):
            # クライアントが既に切断している。応答は書けないため閉じるだけにする。
            self.close_connection = True
        except Exception:  # noqa: BLE001 - 観測UIを落とさないため全例外を500に変換する
            self._write_internal_error()

    def do_POST(self) -> None:  # noqa: N802
        self._write_method_not_allowed()

    def do_PUT(self) -> None:  # noqa: N802
        self._write_method_not_allowed()

    def do_PATCH(self) -> None:  # noqa: N802
        self._write_method_not_allowed()

    def do_DELETE(self) -> None:  # noqa: N802
        self._write_method_not_allowed()

    def _discard_request_body(self) -> None:
        """リクエストボディを読み捨てる。

        HTTP/1.1のkeep-aliveでは、未読のボディが次のリクエスト行として解釈され
        プロトコルが破綻する。GET以外を405で拒否する場合も、ボディだけは
        読み切っておく必要がある。
        """

        try:
            length = int(self.headers.get('Content-Length', '0'))
        except (TypeError, ValueError):
            self.close_connection = True
            return
        if length > 0:
            self.rfile.read(length)

    def _write_method_not_allowed(self) -> None:
        self._discard_request_body()
        body = 'HTML observation UI is read-only.'.encode('utf-8')
        self.send_response(405)
        self.send_header('Content-Type', 'text/plain; charset=utf-8')
        self.send_header('Content-Length', str(len(body)))
        self.send_header('Allow', 'GET')
        self.end_headers()
        self.wfile.write(body)

    def _write_internal_error(self) -> None:
        """処理中の例外を500応答として返す（応答書き出し済みなら閉じるのみ）。"""

        print(f'[robot_console_web] 応答生成に失敗しました: {self.path}', file=sys.stderr)
        traceback.print_exc()
        if self._response_started:
            # ヘッダ送出後は整合した応答を作れないため、コネクションを閉じる。
            self.close_connection = True
            return
        body = 'Internal Server Error'.encode('utf-8')
        try:
            self.send_response(500)
            self.send_header('Content-Type', 'text/plain; charset=utf-8')
            self.send_header('Content-Length', str(len(body)))
            self.end_headers()
            self.wfile.write(body)
        except (BrokenPipeError, ConnectionResetError):
            self.close_connection = True

    def _write_json(self, payload: object) -> None:
        # allow_nan=False で、NaN/Infinity が混入したまま不正なJSONを返すことを防ぐ
        # （_json_safe で None へ変換済みのため、通常ここで例外にはならない）。
        body = json.dumps(_json_safe(payload), ensure_ascii=False, allow_nan=False).encode('utf-8')
        self.send_response(200)
        self.send_header('Content-Type', 'application/json; charset=utf-8')
        self.send_header('Content-Length', str(len(body)))
        self.end_headers()
        self.wfile.write(body)

    def _write_image(self, panel_id: str) -> None:
        image = self.server.image_store.get(panel_id)
        if image is None:
            self._write_not_found()
            return
        buffer = io.BytesIO()
        image.save(buffer, format='PNG')
        body = buffer.getvalue()
        self.send_response(200)
        self.send_header('Content-Type', 'image/png')
        self.send_header('Content-Length', str(len(body)))
        self.end_headers()
        self.wfile.write(body)

    def _write_static(self, path: str) -> None:
        if path == '/':
            path = '/index.html'
        relative = path.lstrip('/')
        static_root = self.server.static_root.resolve()
        candidate = (static_root / relative).resolve()
        if candidate != static_root and static_root not in candidate.parents:
            self._write_forbidden()
            return
        if not candidate.is_file():
            self._write_not_found()
            return
        content_type, _ = mimetypes.guess_type(str(candidate))
        body = candidate.read_bytes()
        self.send_response(200)
        self.send_header('Content-Type', content_type or 'application/octet-stream')
        self.send_header('Content-Length', str(len(body)))
        self.end_headers()
        self.wfile.write(body)

    def _write_not_found(self) -> None:
        body = b'Not Found'
        self.send_response(404)
        self.send_header('Content-Type', 'text/plain; charset=utf-8')
        self.send_header('Content-Length', str(len(body)))
        self.end_headers()
        self.wfile.write(body)

    def _write_forbidden(self) -> None:
        body = b'Forbidden'
        self.send_response(403)
        self.send_header('Content-Type', 'text/plain; charset=utf-8')
        self.send_header('Content-Length', str(len(body)))
        self.end_headers()
        self.wfile.write(body)

    def log_message(self, format: str, *args: object) -> None:  # noqa: A002
        # BaseHTTPRequestHandlerの既定実装はstderrへ逐次出力するため抑制する。
        return


class WebObservationServer:
    """HTML遠隔観測UI用HTTPサーバの起動・停止を管理する。"""

    def __init__(
        self,
        snapshot_provider: SnapshotProvider,
        image_store: Optional[ImageStore] = None,
        *,
        host: str = DEFAULT_HOST,
        port: int = DEFAULT_PORT,
        static_root: Optional[Path] = None,
    ) -> None:
        self._snapshot_provider = snapshot_provider
        self._image_store = image_store if image_store is not None else ImageStore()
        self._host = host
        self._port = port
        self._static_root = static_root or _default_static_root()
        self._httpd: Optional[_ObservationHTTPServer] = None
        self._thread: Optional[threading.Thread] = None

    @property
    def image_store(self) -> ImageStore:
        """本サーバが画像配信に使う `ImageStore` を返す。"""

        return self._image_store

    @property
    def address(self) -> Optional[Tuple[str, int]]:
        """実際にbindされたアドレス（起動前は None）。"""

        if self._httpd is None:
            return None
        return self._httpd.server_address

    def start(self) -> None:
        """バックグラウンドスレッドでサーバを起動する。既に起動中の場合は何もしない。"""

        if self._httpd is not None:
            return
        self._httpd = _ObservationHTTPServer(
            (self._host, self._port),
            _ObservationRequestHandler,
            snapshot_provider=self._snapshot_provider,
            image_store=self._image_store,
            static_root=self._static_root,
        )
        self._thread = threading.Thread(target=self._httpd.serve_forever, daemon=True)
        self._thread.start()

    def stop(self) -> None:
        """サーバを停止する。起動していない場合は何もしない。"""

        if self._httpd is None:
            return
        self._httpd.shutdown()
        self._httpd.server_close()
        if self._thread is not None:
            self._thread.join(timeout=5.0)
        self._httpd = None
        self._thread = None
