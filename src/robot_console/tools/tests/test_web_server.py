"""web/server.py の単体テスト（実HTTPリクエストで検証する）。"""

import http.client
import json
import urllib.error
import urllib.request

import pytest
from PIL import Image

from robot_console.core.image_store import ImageStore
from robot_console.core.snapshot_model import (
    ConsoleSnapshot,
    GpsStateView,
    OperationStateView,
)
from robot_console.web.server import (
    KEEP_ALIVE_TIMEOUT_S,
    REQUEST_QUEUE_SIZE,
    WebObservationServer,
    _ObservationHTTPServer,
    _ObservationRequestHandler,
)


@pytest.fixture
def server():
    snapshot = ConsoleSnapshot(operation_state=OperationStateView(phase='走行中'))
    image_store = ImageStore()
    image_store.set('route_map', Image.new('RGB', (4, 4), color='green'))

    instance = WebObservationServer(lambda: snapshot, image_store, host='127.0.0.1', port=0)
    instance.start()
    yield instance
    instance.stop()


def _get(server, path: str):
    host, port = server.address
    url = f'http://{host}:{port}{path}'
    with urllib.request.urlopen(url, timeout=5) as response:
        return response.status, response.headers.get('Content-Type'), response.read()


def test_snapshot_json_endpoint_returns_expected_phase(server):
    status, content_type, body = _get(server, '/snapshot.json')

    assert status == 200
    assert content_type.startswith('application/json')
    payload = json.loads(body)
    assert payload['operation']['phase'] == '走行中'


def test_map_state_json_endpoint_responds(server):
    status, content_type, body = _get(server, '/map_state.json')

    assert status == 200
    assert content_type.startswith('application/json')
    json.loads(body)  # パース可能であること


def test_sensor_panels_json_endpoint_responds(server):
    status, _content_type, body = _get(server, '/sensor_panels.json')

    assert status == 200
    payload = json.loads(body)
    assert 'panels' in payload


def test_health_json_endpoint_responds(server):
    status, _content_type, body = _get(server, '/health.json')

    assert status == 200
    json.loads(body)


def test_image_endpoint_returns_png_for_known_panel(server):
    status, content_type, body = _get(server, '/images/route_map')

    assert status == 200
    assert content_type == 'image/png'
    assert body[:8] == b'\x89PNG\r\n\x1a\n'  # PNGシグネチャ


def test_image_endpoint_returns_404_for_unknown_panel(server):
    with pytest.raises(urllib.error.HTTPError) as exc_info:
        _get(server, '/images/unknown_panel')
    assert exc_info.value.code == 404


def test_static_index_html_is_served_at_root(server):
    status, content_type, body = _get(server, '/')

    assert status == 200
    assert content_type == 'text/html'
    assert b'robot_console' in body


def test_static_unknown_path_returns_404(server):
    with pytest.raises(urllib.error.HTTPError) as exc_info:
        _get(server, '/no-such-file.html')
    assert exc_info.value.code == 404


def test_path_traversal_is_rejected(server):
    with pytest.raises(urllib.error.HTTPError) as exc_info:
        _get(server, '/../server.py')
    # urllibが送信前にパスを正規化するため、リクエストパスとしては404/403いずれかになり得る。
    assert exc_info.value.code in (403, 404)


@pytest.mark.parametrize('method', ['POST', 'PUT', 'DELETE', 'PATCH'])
def test_write_methods_are_rejected_with_405(server, method):
    host, port = server.address
    url = f'http://{host}:{port}/snapshot.json'
    request = urllib.request.Request(url, method=method, data=b'{}')
    with pytest.raises(urllib.error.HTTPError) as exc_info:
        urllib.request.urlopen(request, timeout=5)
    assert exc_info.value.code == 405
    assert exc_info.value.headers.get('Allow') == 'GET'


def test_start_is_idempotent(server):
    address_before = server.address
    server.start()
    assert server.address == address_before


def _reject_json_constant(name: str):
    """json.loads が NaN/Infinity を読んだ場合に失敗させるフック。"""

    raise AssertionError(f'JSONとして不正な定数が含まれている: {name}')


def test_non_finite_floats_are_serialized_as_null():
    """NaN/Infinityをそのまま出力すると不正なJSONになり、ブラウザ側のパースが失敗する。

    GPS未測位時のROSメッセージにはNaNが入り得るため、null へ落として
    RFC 8259 準拠のJSONを返すことを確認する。
    """

    snapshot = ConsoleSnapshot(
        gps_state=GpsStateView(
            hdop=float('nan'),
            correction_age_s=float('inf'),
            heading_deg=float('-inf'),
        )
    )
    instance = WebObservationServer(lambda: snapshot, host='127.0.0.1', port=0)
    instance.start()
    try:
        _status, _content_type, body = _get(instance, '/snapshot.json')
        assert b'NaN' not in body
        assert b'Infinity' not in body
        # strict なパーサでも読めること（json.loads の既定はNaNを許容するため明示的に禁止する）
        payload = json.loads(body, parse_constant=_reject_json_constant)
        assert payload['gps']['hdop'] is None
        assert payload['gps']['correction_age_s'] is None
        assert payload['gps']['heading_deg'] is None
    finally:
        instance.stop()


def test_provider_exception_returns_500_without_closing_connection():
    """snapshot生成中の例外でも500応答を返し、コネクションを維持することを確認する。

    応答無しで切断されると、ブラウザ側は原因不明の `TypeError: Failed to fetch`
    としてしか観測できず、サーバ側に原因があることが分からなくなる。
    """

    calls = {'count': 0}

    def provider() -> ConsoleSnapshot:
        calls['count'] += 1
        if calls['count'] == 1:
            raise RuntimeError('テスト用の内部エラー')
        return ConsoleSnapshot(operation_state=OperationStateView(phase='走行中'))

    instance = WebObservationServer(provider, host='127.0.0.1', port=0)
    instance.start()
    try:
        host, port = instance.address
        conn = http.client.HTTPConnection(host, port, timeout=5)

        conn.request('GET', '/snapshot.json')
        first = conn.getresponse()
        first.read()
        assert first.status == 500

        # 同一コネクションを使い回して次のリクエストが通ること（切断されていないこと）
        conn.request('GET', '/snapshot.json')
        second = conn.getresponse()
        body = second.read()
        assert second.status == 200
        assert json.loads(body)['operation']['phase'] == '走行中'
        conn.close()
    finally:
        instance.stop()


def test_keep_alive_connection_serves_repeated_requests(server):
    """ブラウザ同様に1本のkeep-aliveコネクションを使い回せることを確認する。"""

    host, port = server.address
    conn = http.client.HTTPConnection(host, port, timeout=5)
    try:
        for _ in range(20):
            conn.request('GET', '/snapshot.json')
            response = conn.getresponse()
            body = response.read()
            assert response.status == 200
            assert json.loads(body)['operation']['phase'] == '走行中'
    finally:
        conn.close()


def test_rejected_method_keeps_connection_usable(server):
    """405応答時にリクエストボディを読み捨てないと、keep-aliveの次要求が壊れる。"""

    host, port = server.address
    conn = http.client.HTTPConnection(host, port, timeout=5)
    try:
        conn.request('POST', '/snapshot.json', body=b'{"payload": "ignored"}')
        rejected = conn.getresponse()
        rejected.read()
        assert rejected.status == 405

        conn.request('GET', '/snapshot.json')
        response = conn.getresponse()
        body = response.read()
        assert response.status == 200
        assert json.loads(body)['operation']['phase'] == '走行中'
    finally:
        conn.close()


def test_request_queue_size_is_larger_than_socketserver_default():
    """既定のlisten backlog(5)ではブラウザの同時接続+再接続で溢れるため拡張している。"""

    assert REQUEST_QUEUE_SIZE > 5
    assert _ObservationHTTPServer.request_queue_size == REQUEST_QUEUE_SIZE


def test_keep_alive_timeout_is_configured():
    """相手が無通告で消えたkeep-aliveコネクションがワーカースレッドを占有し続けないこと。"""

    assert _ObservationRequestHandler.timeout == KEEP_ALIVE_TIMEOUT_S
    assert KEEP_ALIVE_TIMEOUT_S > 0
