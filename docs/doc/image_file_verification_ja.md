# 画像ファイル駆動検証 (`img_file_uvm`)

🌐 **[English](image_file_verification.md)** | **日本語**

**画像をテストパターンとして使う** img_proc スロット RTL のシミュレーション検証。
任意の画像ファイル（または内蔵パターン）を Verilator + cocotb + pyuvm（本物の UVM）上で
選択可能な DUT へ 1 画素/クロックでストリームし、DUT の出力フレームを UVM モニタで
キャプチャして画像として保存し、**同じ入力に同じフィルタを Python ソフトウェアで適用して
生成した期待値画像と全画素を比較**する（ゴールデンモデルは RTL からビット厳密に移植）。

コード: [verification/cocotb/img_file_uvm/](../../verification/cocotb/img_file_uvm/) ·
実行ラッパ: [scripts/run_image_test.ps1](../../scripts/run_image_test.ps1) ·
関連: [cocotb_python_test_guide.md](cocotb_python_test_guide.md)、
[image_processing_principles_ja.md](image_processing_principles_ja.md)

---

## 1. 3 枚の画像: 入力 → 期待値 vs 出力

すべての実行は 3 枚の画像を生成し、テスト判定は文字どおりその比較である。
実例 — 実行 `proc_slot_20260703_052153`（POST スロット、op=invert、内蔵 64×48 パターン）:

| | 画像 | 生成元 | 意味 |
|---|---|---|---|
| <img src="samples/img_file_uvm/pattern_input.png" width="192"> | `input.png` | ホスト側（テストパターン生成器、または `IMG_FILE` 指定画像の変換結果） | DUT へストリームされるフレーム。1 画素/クロック、sof/eol/eof マーカー付き |
| <img src="samples/img_file_uvm/proc_slot_invert_expected.png" width="192"> | `expected.png` | **ソフトウェアゴールデンモデル** — 同じ invert フィルタを Python で `input.png` に適用（[golden.py](../../verification/cocotb/img_file_uvm/golden.py)） | RTL が出力す*べき*画像（ビット厳密） |
| <img src="samples/img_file_uvm/proc_slot_invert_output.png" width="192"> | `output.png` | **RTL シミュレーション** — DUT 出力ストリームを UVM モニタがビートごとにキャプチャ | RTL が*実際に*出力した画像 |

UVM スコアボード（`FrameScoreboard`）は `output` を `expected` と
**全画素 — 3072 画素中 3072 画素、フレーム境界を含む — ピクセル単位で比較**し、さらに
各ビートの sof/eol/eof/err フレーミングマーカーも検証する。`PASS` は 2 枚の画像が
ビット一致していること（上図のとおり）を意味する。1 画素でも異なればテストは FAIL となり、
その (row, col) と got/exp 値が報告され、全差分が `mismatches.txt` に出力される。
`run_info.txt` には構成が記録される:

```text
dut=proc_slot proc_slot op=1 thresh=128
size: 64x48 frames=1
observed beats: 3072 / expected 3072
```

出力画像はチェックフェーズの**前**に書き出されるため、比較が失敗した場合でも必ず
キャプチャ画像を目視確認できる。

## 2. 全 5 DUT の期待値/出力ペア

入力は上と同じ内蔵パターン。各行は Python ゴールデンの期待値と、キャプチャした RTL 出力を
並べたもの（PASS した実行なのでビット一致している）:

| DUT（フィルタ） | expected.png（Python ゴールデン） | output.png（RTL シミュレーション） |
|---|---|---|
| `proc_slot` — invert | <img src="samples/img_file_uvm/proc_slot_invert_expected.png" width="160"> | <img src="samples/img_file_uvm/proc_slot_invert_output.png" width="160"> |
| `conv3x3` — gaussian, shift 4 | <img src="samples/img_file_uvm/conv3x3_gaussian_expected.png" width="160"> | <img src="samples/img_file_uvm/conv3x3_gaussian_output.png" width="160"> |
| `conv5x5` — gaussian5, shift 8 | <img src="samples/img_file_uvm/conv5x5_gaussian5_expected.png" width="160"> | <img src="samples/img_file_uvm/conv5x5_gaussian5_output.png" width="160"> |
| `prefilter` — median 3×3 | <img src="samples/img_file_uvm/prefilter_median_expected.png" width="160"> | <img src="samples/img_file_uvm/prefilter_median_output.png" width="160"> |
| `dither` — ordered Bayer, 2 bit/ch | <img src="samples/img_file_uvm/dither_ordered2_expected.png" width="160"> | <img src="samples/img_file_uvm/dither_ordered2_output.png" width="160"> |

畳み込み出力の上端/左端の暗いフリンジは**実在する検証済みの RTL 境界挙動**である —
ウィンドウの先頭行/列はゼロ初期化されたラインバッファを参照する — ゴールデンモデルは
これを正確に再現しているため、境界はマスクせず比較対象に含めている。

## 3. 実行方法

```powershell
# 任意画像（PNG/JPEG/BMP/... はリポジトリルート Pillow venv でデコード; .ppm/.pgm は直接）
.\scripts\run_image_test.ps1 -Image photo.png -Dut conv3x3 -Kernel sobel_x
.\scripts\run_image_test.ps1 -Image photo.jpg -Dut prefilter -Op median
.\scripts\run_image_test.ps1 -Image photo.png -Dut dither -DitherMode random -DitherBits 2

# 内蔵パターンで全 5 DUT（登録レグレッションが実行する構成）
.\scripts\run_image_test.ps1

# 環境変数形式（全一覧は img_file_uvm/img_config.py に記載）
$env:IMG_FILE='photo.jpg'; $env:IMG_DUT='dither'
.\scripts\pytest_cocotb.ps1 verification/cocotb/img_file_uvm

# 登録スイート
.\scripts\run_cocotb.ps1 -Suite image
```

各実行は `verification/cocotb/_exec/img_file_uvm/<dut>_<timestamp>/` に出力する:

| ファイル | 内容 |
|---|---|
| `input.png` / `input.ppm` | ストリームされたフレーム（デコード/縮小後） |
| `expected.png` | ゴールデン画像（同じフィルタの Python ソフトウェア適用結果） |
| `output.png` / `output.ppm` | キャプチャした DUT 出力フレーム |
| `run_info.txt` | DUT、フィルタ構成、サイズ、観測/期待ビート数 |
| `mismatches.txt` | FAIL 時: 全差分画素を `f<frame> r<row> c<col> got/exp` 形式で列挙 |

補足: `-MaxWidth`/`-MaxHeight` は縮小上限（既定 640×480）。`LINE_PIXELS` は画像幅から
ビルドごとに設定される。スイート実行（`-Suite ...`）は古い `IMG_*` 環境変数を除去するため、
登録レグレッションは常に決定論的な内蔵パターン構成で走る。

## 4. 期待値画像が信頼できる理由

`expected.png` は汎用ライブラリのフィルタではなく、**RTL のストリーミング・ビート順序
そのままの移植**である（[golden.py](../../verification/cocotb/img_file_uvm/golden.py)）:
同じ read-before-write ラインバッファ（ゼロ初期状態）、同じウィンドウシフトレジスタ、
同じ符号付き係数 × 符号なしタップ演算（算術シフト + 飽和）、同じ Bayer テーブルと
Galois LFSR（シード `0xA5`、valid ビートごとに更新、フレーム間で継続）、さらに RTL の
Verilog ビット幅の癖（dither のスミアシフト `n<<1`/`n<<2` は 3 ビット式なので mod 8 で
回り込む — 「修正」せず忠実に再現）まで一致させている。したがって不一致は常に本物の
RTL/モデル乖離であり、丸め方式の違いでは起こらない。ゴールデンモデルは RTL に対する
敵対的マルチエージェント監査を欠陥ゼロで通過している
（[diary_20260703.md](../progress/diary_20260703.md) 参照）。

## 5. UVM アーキテクチャ

```
ImageFrameItem（1 フレーム = 1 トランザクション）
      │  ItemsSequence → sequencer
      ▼
FrameInputAgent ── FramePixelDriver ──► DUT（スロット契約: pixel/valid/sof/eol/eof/err）
                                          │
PixelOutputAgent ── PixelMonitor ◄────────┘  （analysis port、1 ビート = 1 PixelItem）
      │
      ▼
FrameScoreboard ── set_expected(golden) ── check_phase: ビート数 + マーカー + 全画素
```

全コンポーネントは [verification/cocotb/lib/uvm/](../../verification/cocotb/lib/uvm/)
にあり、他のビデオ系テストベンチからも再利用できる。
