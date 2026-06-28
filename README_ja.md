# MIPI2HDML

🌐 **[English](README.md)** | **日本語**

MIPI CSI-2 受信から HDMI 出力までのフルパイプライン FPGA 実装（自作 RTL + PYNQ Python）。

- **ターゲット**: Digilent Zybo Z7-20 (Zynq xc7z020clg400-1)
- **カメラ**: OV5640 搭載 Pcam 5C (2-lane MIPI CSI-2)
- **画像フォーマット**: RGB565 (chip `0x4300=0x6F`, FPGA `expected_dt=0x22`)
- **解像度 / フレームレート**: VGA 640×480 **30fps**（PLL mult=96 → link 384MHz/768Mbps → byte_clk 96MHz）
- **出力**: HDMI (AXI4-Stream Video → AXI VDMA → HDMI TX)

> **動作状態**: カメラ → HDMI モニタまでの **フルパイプラインがライブ動作**。
> **本物の 30fps**（VTS 削減でなく PLL で PCLK を上げる正攻法、フル vblank/露光保持）を
> フル 480 行・帯なし・CRC 0% で常時表示。`fs=fe=30 / std~76`。「30fps は Z-7020 で
> unroutable」は誤りで、XDC を 384MHz に再制約しリビルドすると byte_clk 96MHz で
> **WNS=+0.112** で収束した（追加リソースなし）。設計の全詳細は
> [RTL 設計仕様](docs/doc/rtl_design_spec_ja.md)。
>
> **30fps ロックの注意**: HW 決定論ロック FSM は byte_clk 84MHz(17fps) 用に調整されており、
> 96MHz では bogus ロック（fs=0 / 白画面）するため、30fps では **software lock_mode
> (`--hw-lock 0`)** を使う（`camera_hdmi_demo` / `camera_repl` のデフォルト）。帯 fix は
> byte_clk 比例で **K=14**（17fps は K=8）。FSM の 96MHz 再調整は TODO。

> **再現手順**: ビルド → DSim 検証 → 実機デプロイ → 撮影 / HDMI の完全手順は
> [REPRODUCE.md](REPRODUCE.md)。

---

## パイプライン構成

```text
Pcam 5C (OV5640)  ─ PLL mult=96 → link 384MHz (768Mbps/lane) → 30fps
  └─ MIPI 2-lane HS (continuous clock 0x4800=0x14)
       └─ dphy_hs_byte_probe         IBUFDS/BUFIO/BUFR(÷4 → byte_clk 96MHz)/ISERDES/IDELAY, SoT(0xB8)+bitslip
       │    + dphy_hwlock_fsm         HW 決定論ロック (8x8 bitslip sweep + /4 re-roll, refclk_200) ※96MHz では bogus → software lock 使用
       │    + dphy_lane_supervisor    Digilent 由来のクロックレーン管理 (opt-in)
       │    + settle-blank K=14       バースト頭 settle ゴミを SoT 窓から除外 (帯 fix, byte_clk 比例: 30fps=14 / 17fps=8)
       └─ byte_to_core_cdc            byte_clk → core_clk (Gray FIFO)
            └─ csi2_packet_parser     ヘッダ/ペイロード/CRC 分離
                 └─ csi2_header_ecc / csi2_payload_crc
                      └─ csi2_vcdt_filter        VC/DT フィルタ (expected_dt=0x22)
                           └─ csi2_frame_state    SOF/EOF 管理 + SOF 合成 + force-480
                                └─ rgb565_gray_unpack  byte → RGB888 pixel
                                     └─ axis_video_bridge   AXI4-Stream out
                                          └─ AXI VDMA → HDMI TX
```

ビットストリーム内 SCCB シーケンサ (`ov5640_sccb_init_probe`) が OV5640 を初期化。
ランタイム制御は PYNQ Python から AXI GPIO 経由で行う。モジュール単位の完全な仕様は
[RTL 設計仕様](docs/doc/rtl_design_spec_ja.md) を参照。

> **カラー化 + 3 段画像処理パイプライン**: 上図の `rgb565_gray_unpack` を **真 RGB888 カラー**化
> （`RGB_OUT` で `{R,G,B}`）し、`axis_video_bridge` の前段に **3 段ランタイム画像処理パイプライン**
> を挿入済み。**全部リビルド不要のライブ切替**（SCCB 予約ページ `0xFE` 係数 + idelay GPIO op で制御）:
>
> ```text
> video → PRE (3×3 空間デノイズ + 点処理) → MID (畳み込み) → POST (点処理) → DITHER → capture/HDMI
> ```
>
> **PRE — `axis_rgb_prefilter`（3×3, ラインバッファ）**
>
> - passthrough / invert / grayscale / threshold(2値化) / R/G/B（点処理）
> - **median 3×3**（インパルス/塩胡椒ノイズ除去）, **gaussian 3×3**（ブラー）= 空間デノイズ
>   — `cam.denoise('median'|'gaussian')` / `cam.pre_op(n)`（メディアンは検証済み 19-CAS network）
>
> **MID — 畳み込み**
>
> - **任意 3×3** edge / emboss / sharpen / 任意 — `cam.k(name)` / `cam.kernel(c,s)`
> - **DoG 二重カーネル** 並列 3×3 + 一般 5×5 + 差分 = バンドパス / 特徴検出 — `cam.dog('blob')`
> - **全方向エッジ** `|Gx|+|Gy|`（Sobel マグニチュード, 両極性・全方向）— `cam.edges()`
> - **可変サイズぼかし** 3 段カスケード（実効 5×5 / 9×9 / 13×13）— `cam.blur(5|9|13)` / `cam.cascade(...)`
>
> **POST — 点処理** invert / grayscale / threshold / channel — `cam.post_op(n)`
>
> **DITHER（POST後段・最終段）** `axis_rgb_dither`：ordered(Bayer 4×4)／random(LFSR) でビット深度量子化。
> 1bit=ハーフトーン（gray→`cam.halftone()`）／2–4bit=ポスタライズ／6bit=バンディング抑制 — `cam.dither(bits, mode)`
>
> **組み合わせ（順序自由・1 コマンド）**: `cam.chain(pre, mid, post, …)` で PRE→MID→POST を一括設定。
> 名前付きプリセット `cam.pipeline(name)`（一覧 `cam.pipelines()`）:
> `bin_edges`(2値化→Sobel) / `edge_binary`(Sobel→2値化) / `denoise_edges`(median→Sobel) /
> `median_sketch`(median→Sobel→2値化) / `smooth_sketch`(gaussian→Sobel→2値化) / `sketch` / `sharpen` / `dog_blob` …。
> 対話 REPL `scripts/camera_repl.ps1 -Go`（メニュー Live HDMI → Filter combinations →
> named preset / build custom chain、または `>>>` で `cam.*`）からも設定可。
> 出力は VGA 640×480 真 RGB888 **30fps ライブ HDMI**（CRC0）。
>
> 全モジュール・原理・op 表・0xFE 係数マップ・SW API・リソースは
> **[画像処理パイプライン — 原理とアーキテクチャ](docs/doc/image_processing_principles_ja.md)**。

---

## ハードウェア構成（リソース使用率 / PS-PL 分担）

### FPGA リソース使用率

実ビルド（画像処理スロット最大構成=3段カスケード込み, `xc7z020clg400-1`,
**WNS = +0.017 ns** @ sysclk 100MHz）:

| リソース | 使用 | 全体 (Z-7020) | 使用率 |
| --- | ---: | ---: | ---: |
| LUT | 18,606 | 53,200 | **35.0 %** |
| FF (レジスタ) | 17,705 | 106,400 | 16.6 % |
| BRAM (36Kb) | 9 | 140 | 6.4 % |
| DSP48E1 | 170 | 220 | **77.3 %** |

**DSP は処理スロットの構成で増減**（全モジュール常駐・op で出力選択。各段の詳細は
[画像処理パイプライン](docs/doc/image_processing_principles_ja.md)）:

| スロット構成 | DSP | WNS |
| --- | ---: | ---: |
| 点処理 + 任意3×3 | 29 / 220 (13%) | +0.125 |
| + DoG 二重カーネル (op12) | 110 / 220 (50%) | +0.156 |
| + 3段カスケード可変ブラー (op13-15)（現行） | **170 / 220 (77%)** | +0.017 |

- **乗算は全て DSP48 に逃がす**（混雑エッジの sysclk/AXI 領域に LUT 乗算を足すと WNS が
  −1.6〜−2.6 まで割れる → DSP に逃がし、かつ「和→シフト→飽和」を段分割して収束）。
  ★教訓: timing 失敗を「混雑」と決めつける前に**新規ロジックのパイプライン段不足**を疑え。
- DSP 77% でも LUT 35% / BRAM 6% と余裕。さらに段を積むには分離 line buffer の BRAM 化
  （`xpm_memory_sdpram`、Vivado 推論は 8-6849 で不可）か上位デバイス（Kria 等）。

### Zynq PS の使い方（PS-PL 分担）

Zynq-7020 は **PS（デュアル Cortex-A9 + DDR3 コントローラ + 周辺）** と **PL（FPGA ファブリック）**
の 1 チップ統合。本設計の役割分担:

```text
            ┌─────────────── PS (ARM Cortex-A9, Linux + PYNQ) ───────────────┐
  bitstream │  ブート時に PL へ .bit ロード                                   │
   ─────────┤  制御プレーン:  M_AXI_GP0 (AXI-Lite 32bit) ─► AXI interconnect  │
            │     └─► 6×AXI-GPIO で RTL ノブを R/W:                           │
            │         bitslip / IDELAY / frame_lines / SCCB engine /          │
            │         conv 係数(0xFE0i) / proc_op / debug ページ              │
            │  データプレーン: DDR3 = VDMA フレームバッファ (PYNQ CMA 確保)   │
            └───────────────┬──────────────────────────▲───────────────────┘
       FCLK_CLK0 100MHz     │ S_AXI_HP0 (高速)          │ S_AXI_HP0
       (PL sysclk/AXI 域)   ▼ S2MM: PL→DDR 書込         │ MM2S: DDR→PL 読出
            ┌─────────────── PL (自作 RTL = リアルタイム画素処理) ───────────┐
            │  D-PHY RX → CSI-2 デコード → RGB unpack → 処理スロット →        │
            │  AXI4-Stream → AXI VDMA ─(S2MM)→ DDR / (MM2S)→ rgb2dvi → HDMI   │
            │  ※ byte_clk/core_clk/pixel_clk は PL 内で D-PHY クロックレーン  │
            │     から MMCM/BUFR 生成（PS の FCLK には依存しない）            │
            └────────────────────────────────────────────────────────────────┘
```

- **PS = Linux + 制御 + DRAM フレームバッファ**。ピクセル処理は一切 PS に乗らない
  （PYNQ の Python は「ノブを回す」制御役で、画素は触らない）。
- **制御**: `M_AXI_GP0`（AXI-Lite マスタ）→ 6×AXI-GPIO。PYNQ の MMIO write が RTL の
  全ノブ（ロック・IDELAY・SCCB・conv 係数・proc op）に届き、debug ページを read。
- **データ**: `S_AXI_HP0`（AXI 高速スレーブ）経由で VDMA が **PL→DDR（カメラ書込, S2MM）**
  と **DDR→PL（HDMI 読出, MM2S）**。フレームバッファは PS の DDR3。
- **クロック**: `FCLK_CLK0 = 100MHz` が PL の sysclk / AXI ドメイン（GPIO・VDMA 制御・
  capture bridge）。byte/core/pixel クロックは PL 内生成で PS 非依存。
- これは Zynq の定石分担（PS = ソフト制御 + メモリ、PL = 確定的リアルタイム処理）。
  自作 RTL は **Xilinx MIPI/CSI-2 IP を使わず** D-PHY ロックから画素処理まで PL 内で完結。

---

## 主要機能 (このパイプラインで解決した課題)

| 機能 | 内容 | 関連モジュール |
| ---- | ---- | ---- |
| **本物の 30fps (PLL mult=96)** | VTS 削減（時間前借り）でなく PLL で PCLK を 27→48MHz に上げる正攻法。`0x3036=0x60` で VCO=768 → PCLK=48MHz(30fps) / link=384MHz(768Mbps) → byte_clk 96MHz。XDC `dphy_hs` を 384MHz 再制約しリビルドで WNS=+0.112（追加リソース 0）。フル vblank/露光保持で帯/暗化なし | `ov5640_sccb_init_probe.sv`, `mipi_to_hdmi_probe.xdc` |
| **帯 fix (settle-blank K)** | バースト頭の HS-settle ゴミで per-line SoT を取り逃す問題を byte 域で K byte_clk の SoT 窓ブランクで解決 → フル 480 行。K は byte_clk 比例（30fps=14 / 17fps=8） | `dphy_hs_byte_probe.sv` |
| **HW 決定論ロック FSM (E2)** | ソフト lock_mode (8x8 bitslip sweep + /4 BUFR.CLR re-roll + hold) を RTL FSM 化。電源投入で自動ロック。`HWLOCK_DEFAULT_ON` 焼込み + bit26 inhibit | `dphy_hwlock_fsm.sv` |
| **boot-init NACK 修正** | `frame_lines_gpio` の `C_DOUT_DEFAULT=0x02000000` で RESETB を boot から High に → bitstream-init SCCB が ACK | BD GPIO config |
| **zero-PYNQ RX** | 上記 + continuous/RGB565 焼込みで、電源投入だけで chip 自己設定 + FSM 自動ロック + crc0% 480 行（HDMI 表示は別途 VDMA 起動が必要） | `zero_pynq_test.py` |
| **VDMA genlock (TUSER/FSYNC)** | `C_USE_S2MM_FSYNC=2` + `genlock_mode=2` でタイル化（free-run）を解消 | BD VDMA config |
| **SOF 合成 / force-480** | `csi2_frame_state` が FS 欠落時に最初の LS でフレームを開く + 480 行固定で VTC genlock 安定（ローリング解消） | `csi2_frame_state.sv` |

---

## ディレクトリ構成

```text
rtl/
  mipi_rx/       CSI-2 プロトコル層 + D-PHY フロントエンド + ロック/監視 FSM
  img_proc/      unpack: RGB565=現行 (yuv422/raw8/raw10 は IMAGE_FORMAT 切替の代替で
                 現ビルド未使用→合成で刈取); 画像処理スロット (prefilter/conv3x3/conv5x5/
                 DoG/cascade/proc_slot/dither); VDMA bridge; frame normalizer
  hdmi/          HDMI 出力 / TPG
  prototype/     ハードウェアトップ (mipi_to_hdmi_probe_top), SCCB init FSM, probe
verification/tb/  各ブロック個別テストベンチ (.sv) + DSim filelist (.f)
scripts/          PYNQ Python (ブリングアップ / キャプチャ / デプロイ / 診断) + DSim ランナー (.ps1)
vivado/           ビルド TCL (rebuild_*.tcl, pre_synth_tpg.tcl) + XDC
vloop_probes2/    現行デプロイ Vivado プロジェクト (BD ベース、※gitignore)
docs/doc/         設計仕様（RTL / 画像処理, 英・日）
```

> **注意**: 現行デプロイプロジェクト `vloop_probes2/` は `.gitignore` 対象（BD 含む）。
> RTL/scripts/TCL は git 管理されるが、BD 設計（`C_DOUT_DEFAULT`, core0 CONFIG,
> VDMA fsync=2 等）はローカルのみ。バックアップ推奨。

---

## ビルド / デプロイ

### RTL ビルド (Vivado 2024.2)

```powershell
# core0 (mipi_to_hdmi_probe_top) を OOC 再合成 + impl + bitstream
& "$VIVADO\bin\vivado.bat" -mode batch -source vivado/rebuild_fe_min.tcl
# bitstream: vloop_probes2/vloop.runs/impl_1/bd_wrapper.bit
```

- `rebuild_fe_min.tcl`: 標準の core0 OOC 再合成（VDMA fsync=2 OOC は保持）。
- `rebuild_zeropynq.tcl`: zero-PYNQ RX 構成（GPIO `C_DOUT_DEFAULT` + core0 BD CONFIG を焼く）。
- RTL の param 束縛規則は [RTL 設計仕様](docs/doc/rtl_design_spec_ja.md) 参照（core0 BD CONFIG > RTL default > fileset generic[cosmetic]）。

### DSim 検証 (DSim 2026)

```powershell
& "$DSIM_HOME\shell_activate.ps1"
& "$DSIM_HOME\bin\dsim.exe" -timescale 1ns/1ps -f verification/tb/<block>.f -top tb_<block>
```

### 実機デプロイ / ライブ HDMI (PYNQ)

```powershell
# ライブ HDMI（30fps: software lock + 帯 fix K=14、verified クリーン構成）
# camera_hdmi_demo / oneshot のデフォルトは 30fps 構成（--hw-lock 0 / --settle-blank 14）。
python scripts/deploy_banding_test.py --script camera_hdmi_demo.py `
    --download 1 --full-init 1 `
    --upload-bit vloop_probes2/vloop.runs/impl_1/bd_wrapper.bit `
    --extra-args "--vcm-sweep 0 --total 90"

# 静止画キャプチャ
python scripts/deploy_banding_test.py --script oneshot_capture.py ...
```

> 新ビルドを焼く初回のみ `--upload-bit` で .bit/.hwh をボードへ送る。以降は省略可
> （`--download 1` がボード上の `bd_wrapper.bit` を再ロード）。17fps ビルドに戻した場合は
> `--extra-args "--hw-lock 1 --settle-blank 8 ..."` を付ける。

### 対話制御 REPL（カメラ制御の統合ツール）

散在する一発スクリプトの代わりに、ブリングアップ・ロック・レジスタ R/W・ライブ HDMI・
静止画・フォーカス等を 1 つの対話オブジェクト `cam` に統合した REPL。**起動するとメニューが
開き、番号で操作を選ぶ。任意値（秒数・レジスタ・係数等）は `[既定値]` 付きで対話入力**
（Enter で既定値）。`q` でメニューを抜けると `>>>` プロンプトに落ち、従来の `cam.*` API が
そのまま使える（再入は `Menu(cam).run()`）。

```powershell
.\scripts\camera_repl.ps1            # アップロード + メニュー REPL を開く（SSH パスワード: xilinx）
.\scripts\camera_repl.ps1 -Go        # 起動時に cam.go()（フルブリングアップ）後にメニュー
```

**メニュー操作**（数字を入力 → Enter。`0`/空/`b` で戻る、最上位 `q` で `>>>` へ）:

```text
   1) Bring-up / status      (go / status / diagnostics)
   2) Live HDMI / processing (hdmi / proc / kernel / dog / blur / edges / capture)
   3) Registers / debug      (read / write / dbg / regs / accounting / eye)
   4) Knobs                  (vcm / idelay / settle / window / gain / sharpen / testpattern)
   h) Command help (raw cam.* API)
   q) Quit menu -> >>> python プロンプト（cam は生存）
```

`>>>` プロンプト側の直接 API（`q` で抜けた後、または `--menu 0` 起動時）:

```python
>>> cam.go()          # init + RGB565 arm + software lock + 帯 fix K=14（30fps 構成）
>>> cam.hdmi(60)      # ライブ HDMI 60s（VDMA 自動停止）
>>> cam.capture()     # 静止画 -> _capture/
>>> cam.read(0x300A)  # SCCB R/W | cam.dbg(0x18) | cam.link() | cam.status()
>>> cam.vcm(280)      # フォーカス | cam.settle(14) | cam.idelay(16,16)
>>> cam.kernel([-1,0,1,-2,0,2,-1,0,1], 0)   # 任意 3x3 カーネルのアドホック投入
```

> 30fps では `cam.go()`（メニュー「Full bring-up」）は software lock + K=14 が既定。17fps
> ビルドは bring-up プロンプトで「HW-lock FSM」に y、または `cam.go(hw=True)`。ライブが
> gakugaku なら settle-blank K=14 を再適用（Knobs → Settle-blank、または `cam.settle(14)`）。
> メニューを使わず直接 `>>>` に落とすには `--menu 0`（非対話パイプ実行時）。
>
> **重要**: VDMA 稼働中のジョブを kill/TaskStop しないこと（sshd ハング → 物理電源サイクル）。
> REPL の `cam.hdmi()`/`cam.capture()` は return/exit 時に VDMA を自動停止する（atexit + signal）。
> `--total` で自然終了させる。PYNQ 側スクリプトは必ず `pynq_bringup.setup_session()` を使う。

---

## ツール

| ツール | バージョン | パス例 |
| ------ | --------- | ------ |
| Vivado | 2024.2 | `E:\...\xilinx\Vivado\2024.2\bin\vivado.bat` |
| DSim | 2026 | `C:\Program Files\Altair\DSim\2026` |
| Python (PYNQ) | 3.8+ | ボード上 |

---

## ドキュメント

設計の正本は以下の 4 文書（英・日）。

| 内容 | English | 日本語 |
| ---- | ------- | ------ |
| **RTL 設計仕様**（全モジュールの完全詳細） | [rtl_design_spec.md](docs/doc/rtl_design_spec.md) | [rtl_design_spec_ja.md](docs/doc/rtl_design_spec_ja.md) |
| **画像処理パイプライン**（原理とアーキテクチャ） | [image_processing_principles.md](docs/doc/image_processing_principles.md) | [image_processing_principles_ja.md](docs/doc/image_processing_principles_ja.md) |
| **画像処理サンプルギャラリー**（全フィルタの実機撮影例 + 設定） | [image_processing_samples.md](docs/doc/image_processing_samples.md) | [image_processing_samples_ja.md](docs/doc/image_processing_samples_ja.md) |

- **再現手順**: [REPRODUCE.md](REPRODUCE.md)
- **ライセンス / 帰属**: [LICENSE](LICENSE) / [THIRD_PARTY_NOTICES.md](THIRD_PARTY_NOTICES.md)

---

## ライセンス

本プロジェクトは MIT License（[LICENSE](LICENSE)）。一部の D-PHY RTL
（`rtl/mipi_rx/dphy_lane_supervisor.sv`, `dphy_cdc_prims.sv`）は Digilent MIPI D-PHY
Receiver IP（MIT, Copyright (c) 2016 Digilent, Author: Elod Gyorgy）の派生で、帰属は
[THIRD_PARTY_NOTICES.md](THIRD_PARTY_NOTICES.md) に集約しています。
