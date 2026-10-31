# センサ参照資料

保存日: 2026-09-14。複数パッケージから参照するメーカー原典をまとめる。
[GNSS・FAST-LIO同期提案](../時刻同期設計.md)から参照する。
PDFは再編集・圧縮・透かし除去をせず保存する。メーカー資料の権利表示は原本に従い、
本リポジトリのコードのライセンスをメーカー資料へ適用しない。

MID-360は[公式ダウンロードページ](https://www.livoxtech.com/mid-360/downloads)掲載の資料を取得した。
Wiki原文はコミット`90fd009e40cf31ff1960826676ca2bbfc19e9216`で固定した。
Wikiの相対画像は同梱していないため、図は公式閲覧版を参照する。

- [同期手順の閲覧版](https://livox-wiki-en.readthedocs.io/en/latest/tutorials/new_product/common/time_sync.html)
- [通信プロトコルの閲覧版](https://livox-wiki-en.readthedocs.io/en/latest/tutorials/new_product/mid360/livox_eth_protocol_mid360.html)

## ファイルと取得元

- [UM982_User_Manual_EN_R1.3-stamped.pdf](um982/UM982_User_Manual_EN_R1.3-stamped.pdf) — R1.3 / 26ページ、930,393 bytes。
  取得元: ユーザー提供の添付PDF（原本維持）
- [Unicore_N4_Reference_Commands_EN_R1.2-stamped.pdf](um982/Unicore_N4_Reference_Commands_EN_R1.2-stamped.pdf) — R1.2 / 308ページ、2,425,729 bytes。
  取得元: ユーザー提供の添付PDF（原本維持）
- [Livox_Mid-360_User_Manual_EN.pdf](mid360/Livox_Mid-360_User_Manual_EN.pdf) — v1.2 / 24ページ、1,926,442 bytes。
  取得元: https://terra-1-g.djicdn.com/851d20f7b9f64838a34cd02351370894/Livox/Livox_Mid-360_User_Manual_EN.pdf
- [Livox_Mid-360_Quick_Start_Guide_multi.pdf](mid360/Livox_Mid-360_Quick_Start_Guide_multi.pdf) — v1.8 / 47ページ、日本語あり、4,663,761 bytes。
  取得元: https://dl.djicdn.com/downloads/Livox/Mid-360/QSG/Livox_Mid-360_Quick_Start_Guide_multi.pdf
- [time_sync.rst](mid360/time_sync.rst) — 公式Wikiの原文、14,862 bytes。
  取得元: https://raw.githubusercontent.com/Livox-SDK/livox_wiki_en/90fd009e40cf31ff1960826676ca2bbfc19e9216/source/tutorials/new_product/common/time_sync.rst
- [livox_eth_protocol_mid360.md](mid360/livox_eth_protocol_mid360.md) — 公式Wikiの原文、47,164 bytes。
  取得元: https://raw.githubusercontent.com/Livox-SDK/livox_wiki_en/90fd009e40cf31ff1960826676ca2bbfc19e9216/source/tutorials/new_product/mid360/livox_eth_protocol_mid360.md

- [UM982_carrier_board_ports.jpg](um982/UM982_carrier_board_ports.jpg) — ユーザー提供の基板説明写真、原本維持。
  COM3のPPS/TX3等の表記を確認できる。基板リビジョンと信号電圧は未確認。

改変確認: このディレクトリで `sha256sum -c SHA256SUMS` を実行する。
