# 画像処理サンプルギャラリー

🌐 **[English](image_processing_samples.md)** | **日本語**

ランタイム画像処理フィルタを、実機（OV5640 → Zybo Z7-20 → HDMI/VDMA）でライブ撮影した
ビジュアルカタログ。各サンプルは**実パイプラインから取得した実静止画**であり、ソフトウェアの
模擬画像ではない。各モジュール・op 表・`0xFE` 係数マップは
**[画像処理パイプライン — 原理とアーキテクチャ](image_processing_principles_ja.md)** を参照。

> ギャラリー全体を 1 コマンドで再現（依存をアップロード → 実機実行 → 静止画を取得）:
> `python scripts/deploy_sample_filters.py`。実機側の撮影ロジックは
> [scripts/sample_filters_capture.py](../../scripts/sample_filters_capture.py)、フィルタ
> プリセットは [scripts/camera_repl.py](../../scripts/camera_repl.py)（`PIPELINES` /
> `CONV_KERNELS` / `DOG_PRESETS`）に定義。

---

## 撮影条件（全サンプル共通）

| 項目 | 値 |
| --- | --- |
| センサ / フォーマット | OV5640、RGB565 (`0x4300=0x6F`) → FPGA RGB888 24bit |
| 解像度 / レート | VGA 640×480、30fps、continuous clock (`0x4800=0x14`) |
| 構成 | **全フィルタはランタイム / リビルド不要** — 1 つのビットストリームを AXI-GPIO + SCCB 予約ページ `0xFE` 係数で切替 |
| 撮影経路 | VDMA genlock シングルショット。6 回 grab して最もタイル化の少ない 1 枚を採用。右端 3px アーティファクトと下端フレーム wrap はマスク |

**被写体**は全画像で同一: 円筒型のチューブ（細かい印刷文字 + 滑らかな曲面グラデーション）を、
マイクのショックマウント（黒い **X** フレーム）とターミナルモニタ（高コントラスト文字）の前に
立てたもの。意図的に混合コンテンツにしてある — 細かい文字は *sharpen / edge* 系を、滑らかな
チューブ胴は *blur / dither* 系を、硬い X フレームの線は *binarize / sketch* 系を試すため。

処理チェーンは **PRE → MID → POST → DITHER**（すべてライブ切替可能）:

```text
video → PRE (3×3 デノイズ + 点処理) → MID (畳み込み) → POST (点処理) → DITHER → capture/HDMI
```

---

## 1. ベースライン & 点処理（PRE/POST 点処理）

画素単位の処理 — 空間近傍を使わない。最も軽い段; `proc_op` 0–7。

### `colour` — パススルー基準

<img src="samples/colour.png" width="420">

**フィルタ:** `cam.passthrough()` / `cam.pipeline('colour')` — `proc_op=0`
**内容:** 無加工の真 RGB888 画像。他の全サンプルの比較基準であり、同時にパイプラインの
健全性チェックを兼ねる（正しい色 ⇒ unpack + genlock 正常）。

### `invert` — 写真ネガ

<img src="samples/invert.png" width="420">

**フィルタ:** `cam.proc(1)` / `cam.pipeline('invert')` — `proc_op=1`
**内容:** 各チャネルで `out = 255 − in`。明るいチューブは暗いシアン/緑に、暗い背景は淡色に
反転 — フィルムネガ風。

### `grayscale` — 輝度

<img src="samples/grayscale.png" width="420">

**フィルタ:** `cam.proc(2)` — `proc_op=2`
**内容:** RGB を 1 つの輝度（Y）に潰し全チャネルに複製。色相を捨て明度構造を残す — 多くの
空間/エッジフィルタが土台にする入力。

### `binarize` — 2 値化（しきい値）

<img src="samples/binarize.png" width="420">

**フィルタ:** `cam.proc(4)` / `cam.pipeline('binarize')` — `proc_op=4`、しきい値 128（緑）
**内容:** 各画素をしきい値の上下で純黒/純白に。2 レベル分割で、後段のスケッチ/輪郭系の基礎。

### `r_only` / `g_only` / `b_only` — 単一チャネル

<img src="samples/r_only.png" width="280"> <img src="samples/g_only.png" width="280"> <img src="samples/b_only.png" width="280">

**フィルタ:** `cam.proc(5)` / `cam.proc(6)` / `cam.proc(7)` — `proc_op=5/6/7`
**内容:** R / G / B のいずれか 1 チャネルのみ残す（他は 0）。チャネル別の応答やセンサの
Bayer/カラーマトリクス挙動を確認するのに有用。

---

## 2. 空間デノイズ（PRE — `axis_rgb_prefilter`、3×3 ラインバッファ）

畳み込み段の**前**に適用し信号を整える 3×3 近傍フィルタ。

### `gaussian` — 3×3 ガウシアンブラー

<img src="samples/gaussian.png" width="420">

**フィルタ:** `cam.denoise('gaussian')` / `cam.pipeline('gaussian')` — PRE op 8、カーネル `[1,2,1; 2,4,2; 1,2,1] >> 4`
**内容:** 重み付き平均による平滑化。わずかなシャープさと引き換えにガウシアン/センサノイズを
抑える — 穏やかで対称なブラー。

### `median` — 3×3 メディアンデノイズ

<img src="samples/median.png" width="420">

**フィルタ:** `cam.denoise('median')` / `cam.pipeline('median')` — PRE op 9（19-CAS ソートネットワーク）
**内容:** 各画素を近傍の中央値に置換。エッジをブラーよりはるかに保ったままインパルス/塩胡椒
ノイズを除去 — 印刷文字が読める状態を保っている点に注目。

---

## 3. 畳み込み（MID — 任意 3×3 カーネル、`proc_op=8`）

任意の符号付き 3×3 カーネルを DSP48 アレイで実行。`cam.k(name)`（プリセット）または
`cam.kernel(coeffs, shift)`（カスタム）でロード。

### `sharpen` — アンシャープ（4 近傍）

<img src="samples/sharpen.png" width="420">

**フィルタ:** `cam.k('sharpen')` / `cam.pipeline('sharpen')` — カーネル `[0,-1,0; -1,5,-1; 0,-1,0] >> 0`
**内容:** 中心画素を近傍に対し強調し高周波ディテールを増幅。エッジと印刷文字が鮮明になる
（強すぎるとリンギング/ハロー）。

### `emboss` — 3D レリーフ

<img src="samples/emboss.png" width="420">

**フィルタ:** `cam.k('emboss')` / `cam.pipeline('emboss')` — カーネル `[-2,-1,0; -1,1,1; 0,1,2] >> 0`
**内容:** 中間グレーにバイアスした非対称（斜め）勾配。金属に型押ししたような外観に — 平坦部は
グレー、エッジは一方向から照らされた明暗の稜線になる。

### `sobel_x` — 縦エッジ勾配

<img src="samples/sobel_x.png" width="420">

**フィルタ:** `cam.k('sobel_x')` — カーネル `[-1,0,1; -2,0,2; -1,0,1] >> 0`
**内容:** X 方向 1 次微分 → **縦**エッジ（左右の輝度変化）に反応。単極性でエッジの片側のみ
光る。両極性の `edges` と比較。

### `sobel_y` — 横エッジ勾配

<img src="samples/sobel_y.png" width="420">

**フィルタ:** `cam.k('sobel_y')` — カーネル `[-1,-2,-1; 0,0,0; 1,2,1] >> 0`
**内容:** Y 方向 1 次微分 → **横**エッジに反応。`sobel_x` の直交パートナーで、両者を合成すると
全方向の `edges` マグニチュードになる。

### `laplacian` — 等方 2 次微分

<img src="samples/laplacian.png" width="420">

**フィルタ:** `cam.k('laplacian')` — カーネル `[0,-1,0; -1,4,-1; 0,-1,0] >> 0`
**内容:** 2 次微分の和 — 方向に依らないエッジ応答。全方位の細部とゼロ交差を一度に強調
（Sobel よりノイズに敏感）。

### `outline` — 8 近傍アウトライン

<img src="samples/outline.png" width="420">

**フィルタ:** `cam.k('outline')` — カーネル `[-1,-1,-1; -1,8,-1; -1,-1,-1] >> 0`
**内容:** 8 近傍すべてを使った強いラプラシアン。ほぼ平坦な地に太い輪郭線を生成 — シーン中の
あらゆる境界がトレースされる。

### `edges` — 全方向エッジマグニチュード

<img src="samples/edges.png" width="420">

**フィルタ:** `cam.edges()` / `cam.pipeline('edges')` — `set_edges(2)`、`proc_op=12`、`|Gx| + |Gy|`
**内容:** Sobel-X と Sobel-Y を並列実行し、各々 `|·|`（`cfg_abs`）で整流して加算 — **両**極性・
**全**方向のエッジが黒地に明るく出る。完全等方なエッジ検出で、スケッチ系チェーンの前段。

---

## 4. 可変ブラー（カスケード）& DoG 二重カーネル

多段畳み込み: 一般 5×5 段（S1）+ 分離 5×5 段 2 つ（S2/S3）。出力タップで実効カーネルサイズを
選択（`proc_op` 13/14/15）。DoG コンバイナは 2 つのブラー半径を差分（`proc_op=12`）。

### `blur_5` / `blur_9` / `blur_13` — カスケードガウシアンブラー

<img src="samples/blur_5.png" width="280"> <img src="samples/blur_9.png" width="280"> <img src="samples/blur_13.png" width="280">

**フィルタ:** `cam.blur(5|9|13)` — `proc_op=13/14/15`、実効 **5×5 / 9×9 / 13×13** ガウシアン
**内容:** カスケード段を重ねるほどブラー半径が広がり（段数 = サポート幅）、リビルド不要で
ライブ切替。左から右へチューブ文字と X フレームが順に溶けていく — ランタイム可変の被写界深度
/ 前処理平滑ノブ。

### `dog_blob` — DoG（バンドパス）

<img src="samples/dog_blob.png" width="420">

**フィルタ:** `cam.pipeline('dog_blob')` / `cam.dog('blob')` — `proc_op=12`、`clamp(G3/16 − G5/256 + 128)`
**内容:** 広いブラーから狭いブラーを引くと空間周波数の帯のみ残る → 中間グレーを中心とした
blob / バンドパス応答。平坦部はグレーに打ち消され、テクスチャや中スケール特徴が浮き出る
（古典的な blob / 特徴検出器）。

### `dog_unsharp` — 広半径アンシャープマスク

<img src="samples/dog_unsharp.png" width="420">

**フィルタ:** `cam.dog('unsharp')` — `proc_op=12`、`clamp(2·identity − G5/256)`
**内容:** `2×原画 − 広ブラー` で失われた高周波を足し戻す → 3×3 `sharpen` より広半径のシャープ。
フルカラーのまま局所コントラストとマイクロディテールを回復。

---

## 5. フィルタ組み合わせ（厳選チェーン）

PRE → MID → POST を 1 コマンドで設定（`cam.chain(...)` / `cam.pipeline(name)`）。順序が重要 —
2 値化→エッジは綺麗な輪郭、エッジ→2 値化はしきい値処理されたエッジマップになる。

### `bin_edges` — 2 値化 → Sobel（輪郭）

<img src="samples/bin_edges.png" width="420">

**フィルタ:** `cam.pipeline('bin_edges')` — PRE threshold(128) → MID `edges`
**内容:** 先に 2 値化し、その 2 値領域のエッジマグニチュードを取る → 平坦形状の綺麗な閉じた
**輪郭**（内部テクスチャなし）。

### `edge_binary` — Sobel → 2 値化（エッジマップ）

<img src="samples/edge_binary.png" width="420">

**フィルタ:** `cam.pipeline('edge_binary')` — MID `edges` → POST threshold(64)
**内容:** 先にエッジ、次に低しきい値 → 硬い**2 値エッジマップ**（黒地に白線、≈ Canny 第 1 段）。
しきい値を下げるほどエッジが増える。

### `sketch` — Gray → エッジ → 2 値化（線画）

<img src="samples/sketch.png" width="420">

**フィルタ:** `cam.pipeline('sketch')` — PRE gray → MID `edges` → POST threshold(64)
**内容:** 脱色 → エッジ検出 → 2 値化 → シーンの綺麗な鉛筆スケッチ / 線画。X フレームとチューブ
輪郭が鮮明な白ストロークに還元される。

### `gray_edges` — Gray → エッジ（モノクロマグニチュード）

<img src="samples/gray_edges.png" width="420">

**フィルタ:** `cam.pipeline('gray_edges')` — PRE gray → MID `edges`
**内容:** 輝度のみでエッジマグニチュードを計算 — `edges` のグレースケール版で、色境界に残る
クロマフリンジが出ない。

### `denoise_edges` — メディアン → Sobel（クリーンエッジ）

<img src="samples/denoise_edges.png" width="420">

**フィルタ:** `cam.pipeline('denoise_edges')` — PRE median → MID `edges`
**内容:** エッジ検出の**前**にメディアンデノイズを掛け、偽エッジを発火させるインパルスノイズを
除去 → 素の `edges` より明確に綺麗なエッジマップ。

### `median_sketch` / `smooth_sketch` — デノイズ付きスケッチ

<img src="samples/median_sketch.png" width="300"> <img src="samples/smooth_sketch.png" width="300">

**フィルタ:** `cam.pipeline('median_sketch')`（median → edges → binarize）/
`cam.pipeline('smooth_sketch')`（gaussian → edges → binarize）
**内容:** `sketch` と同じ gray→edge→binarize に、デノイズ前処理を加えたもの。`median_sketch` は
細線を残しつつ斑点を落とし、`smooth_sketch`（ガウシアン前ブラー）はより少なく太く滑らかな
ストロークになる。

---

## 6. ディザ / ハーフトーン（DITHER — `axis_rgb_dither`、最終段）

POST の後にビット深度を量子化。ordered（Bayer 4×4）または random（LFSR）のディザパターンを使用。

### `halftone` — Gray → 1bit ordered（新聞印刷）

<img src="samples/halftone.png" width="420">

**フィルタ:** `cam.halftone()` / `cam.pipeline('halftone')` — PRE gray → DITHER 1bit ordered
**内容:** チャネル 1bit + ordered Bayer マトリクス → 古典的な新聞ハーフトーン。見かけのグレーは
黒/白ドットの局所密度だけで再現される。

### `poster` — 2bit ordered ポスタライズ

<img src="samples/poster.png" width="420">

**フィルタ:** `cam.poster()` / `cam.pipeline('poster')` — DITHER 2bit/ch ordered
**内容:** 各チャネルを 4 階調に量子化 + ordered ディザ → レトロなポスター調のバンドカラー。
Bayer パターンが硬いバンディングを隠す。

### `edge_halftone` — Gray → エッジ → 1bit

<img src="samples/edge_halftone.png" width="420">

**フィルタ:** `cam.pipeline('edge_halftone')` — PRE gray → MID `edges` → DITHER 1bit ordered
**内容:** エッジマグニチュードをハーフトーン化 → シーンのエッジを点描/彫刻線で描画。
`gray_edges` の線画と `halftone` のスティップルテクスチャを合成したもの。

### `dither_random` — 2bit random（LFSR）

<img src="samples/dither_random.png" width="420">

**フィルタ:** `cam.pipeline('dither_random')` — DITHER 2bit/ch random
**内容:** `poster` と同じ 2bit 量子化だが、しきい値を Bayer でなく LFSR ノイズでディザ →
固定パターンなしでバンディングを崩す、フィルムグレイン / TV スタティック調のテクスチャ。

---

## 関連

- **[画像処理パイプライン — 原理とアーキテクチャ](image_processing_principles_ja.md)** — モジュール別設計、op 表、`0xFE` 係数マップ、SW API、FPGA リソース。
- **[RTL 設計仕様](rtl_design_spec_ja.md)** — RTL の完全詳細。
- リポジトリ: **[README](../../README_ja.md)**。
