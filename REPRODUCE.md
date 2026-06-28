# REPRODUCE — MIPI CSI-2 → HDMI live image (supervisor + SOF synth + FE-resync)

このマイルストーン (commit `3dd252d` 系列) を再現する手順。自作 RTL D-PHY
フロントエンドで OV5640/Pcam 5C のパケットロスを修正し、フル 480 行の合焦実
画像を HDMI ライブ出力するところまでを対象とする。

達成内容の詳細・経緯は `docs/progress/diary_20260612.md`
/ `diary_20260613.md` を参照。

---

## 0. 前提環境

| 項目 | 値 |
|---|---|
| FPGA ボード | Digilent Zybo Z7-20 (`xc7z020clg400-1`) |
| カメラ | OV5640 / Pcam 5C (2-lane MIPI CSI-2、電動 VCM 固定焦点) |
| 合成 | Vivado 2024.2 (`<VIVADO>\bin\vivado.bat`) |
| シミュレータ | DSim 2026 (システム環境変数の `DSIM_HOME`/`DSIM_LICENSE` をそのまま使う) |
| PYNQ ボード | `192.168.2.99` (xilinx/xilinx)、`paramiko` 経由 SSH |
| ローカル Python | リポジトリ直下 `.venv` (paramiko, numpy, PIL) |

> **重要 (DSim ライセンス)**: run スクリプトで `DSIM_LICENSE` を `metrics-ca` の
> 古いキーに上書きすると "License revoked" になる。**システム env
> (`DSIM_HOME=...\DSim\2026`, `DSIM_LICENSE=...\2026\dsim-license.json`) を
> そのまま使い、PATH に deps を足すだけ**にする。`scripts/run_dsim.ps1`
> はこのパターンに修正済み。

---

## 0.5 Vivado プロジェクト再生成 (vloop_probes2)

デプロイプロジェクト `vloop_probes2/` は build ディレクトリとして `.gitignore` 対象。
**設計は `vivado/` 内の再生成 TCL として版管理**されている（gitignore 戦略、2026-06-20）。
`vloop_probes2/` を失った場合の再生成:

```powershell
# 1. プロジェクト + BD を再生成（C_DOUT_DEFAULT/core0 CONFIG/VDMA fsync=2 を含む）
& "$VIVADO\bin\vivado.bat" -mode batch -source vivado/vloop_probes2_recreate.tcl
# 2. bitstream をビルド
& "$VIVADO\bin\vivado.bat" -mode batch -source vivado/rebuild_fe_min.tcl
```

BD を編集（`rebuild_zeropynq.tcl` 等で CONFIG を変更）した後は、再生成 TCL を更新:

```powershell
& "$VIVADO\bin\vivado.bat" -mode batch -source vivado/export_vloop_probes2_tcl.tcl
git add vivado/vloop_probes2_bd.tcl vivado/vloop_probes2_recreate.tcl
```

> 再生成 TCL は初回利用時に実機ビルドまで通すことで検証すること（IP バージョン依存あり）。

---

## 1. Vivado プロジェクトは tcl から生成する (gitignore は意図的)

Vivado プロジェクト一式 (`vloop_probes2/` などの作業ディレクトリ、`.runs/`
`.cache/` `.gen/` `.xpr` 等) は **意図的に `.gitignore` 対象**。プロジェクトは
**tcl から再生成する設計**のため、巨大なバイナリ/中間生成物を git に入れない。
git には RTL / TB / XDC / BD tcl / build スクリプトという「ソース」のみを置く。

- **ゼロから生成**: `vivado/<target>/build_bitstream.tcl` が `create_project`
  + BD 構築 (`bd_design.tcl`) + synth / impl / write_bitstream を一括実行する。
  例: `vivado/mipi_to_hdmi_vdma_loop/build_bitstream.tcl` は BD ベースの
  プロジェクトを `create_project` してビットストリームまで通す。
- **増分リビルド (推奨)**: `vivado/rebuild_fe_min.tcl` は既存の BD-based
  プロジェクト (`vloop_probes2/vloop.xpr`) を `open_project` し、**core0 OOC reset**
  で再合成する (フル create より高速)。`vloop_probes2` は BD-based MIPI パイプライン
  用の作業プロジェクト名。

→ RTL/TB/XDC/tcl が揃っているので、**シミュレーション (§3) と (プロジェクト生成
後の) ビルド (§2)・実機運用 (§4) はすべて再現可能**。

---

## 2. ビットストリームのビルド

ビルド手順は §0.5（`vivado/rebuild_fe_min.tcl`）: 既存 `vloop_probes2/vloop.xpr` を
`open_project` → **core0 OOC (`bd_core0_0_synth_1`) を `reset_run`** → synth_1/impl_1。
RTL は OOC BD セル内で合成されるため core0 reset 必須（memory
`feedback_rtl_edits_need_core0_ooc_resynth`）。所要 ~40 分、期待 **WNS ≈ +0.2 ns /
violations ゼロ**（"All user specified timing constraints are met"）。

出力:
- `vloop_probes2/vloop.runs/impl_1/bd_wrapper.bit`
- `.hwh` は `vloop_probes2/vloop.gen/sources_1/bd/bd/hw_handoff/bd.hwh`
  (デプロイ前に `bd_wrapper.hwh` として impl_1 へコピー、§4 参照)

### XDC の要点 (`vivado/mipi_to_hdmi_probe/mipi_to_hdmi_probe.xdc`)

supervisor は `refclk_200_unbuf` (PLLE2 CLKOUT0、sysclk 派生) で動く。タイミング
を閉じるのに 2 点が必須:
- `set_clock_groups -asynchronous -include_generated_clocks sys_clk_pin ↔ phy_byte_clk`
  (refclk_200↔byte_clk CDC、これがないと WNS -7.5)
- `set_false_path sys_clk_pin ↔ refclk_200_unbuf` (rst_n→supervisor reset CDC、
  同一 PLL ソースで 1ns 要求になり WNS -2.6)
- RTL 側でも `dphy_lane_supervisor.sv` の `ctl_rst` を `dphy_reset_bridge` で
  ctl_clk 同期解除 (なしだと FSM が起動しない)

---

## 3. シミュレーション検証 (DSim)

PowerShell から。各 run スクリプトはシステム env をそのまま使う。

```powershell
cd <repo>
.\scripts\run_dsim.ps1 dphy_lane_supervisor     # supervisor 単体 (T1-T7)
.\scripts\run_dsim.ps1 dphy_probe_supervised    # probe+supervisor 統合 (16/16)
```

frame_state 系 (SOF 合成 + FE-resync + 回帰) は `.f` を直接 DSim へ。期待は
`TEST PASSED`:

| top | .f | 検証 |
|---|---|---|
| `tb_csi2_frame_state_sofsynth` | `csi2_frame_state_sofsynth.f` | SOF 合成 + FE-resync (シナリオ A-E) |
| `tb_csi2_frame_state` ほか 8 本 | `csi2_frame_state*.f` | legacy 回帰 (cfg_sof_synth=0 無変更) |
| `tb_dphy_*` 5 本 | `dphy_*.f` | probe 回帰 (sup_* tie-0 で互換) |

DSim 直接呼び出し例 (システム env 前提):

```powershell
$dsim = $env:DSIM_HOME
$env:PATH = "$dsim\bin;$dsim\mingw\bin;$dsim\dsim_deps\bin;$dsim\lib;$env:PATH"
Push-Location verification\tb
& "$dsim\bin\dsim.exe" -timescale 1ns/1ps -f csi2_frame_state_sofsynth.f `
    -top tb_csi2_frame_state_sofsynth -sv_seed 1 -l ..\..\_dsim\logs\sofsynth.log
Pop-Location
```

`port default (input logic cfg_sof_synth = 1'b0)` は DSim/Vivado 両対応 (既存 TB
が未接続のまま PASS することで実証済み)。

---

## 4. 実機: デプロイ + 撮影

デプロイは `scripts/deploy_banding_test.py` (paramiko)。bitstream は
`/home/xilinx/mipi2hdml/bd_wrapper.bit` (+ `.hwh`) へ送る。

### 4.1 新 bitstream の配置

```powershell
# .hwh を impl_1 に staging (BD 不変なら既存でも可)
Copy-Item vloop_probes2\vloop.gen\sources_1\bd\bd\hw_handoff\bd.hwh `
          vloop_probes2\vloop.runs\impl_1\bd_wrapper.hwh -Force
```

### 4.2 ワンショット撮影 → `picture/`

```powershell
.\scripts\oneshot.ps1                 # 1 枚撮影 → picture\pic_<ts>.png/npy
.\scripts\oneshot.ps1 -Vcm 280        # VCM フォーカス code 指定
.\scripts\oneshot.ps1 -Reboot         # chip 劣化時 (long=0) にリセットしてから
```

初回の bitstream 更新を含めるなら deploy を直接:

```powershell
.\.venv\Scripts\python.exe scripts\deploy_banding_test.py --host 192.168.2.99 `
    --upload-bit vloop_probes2\vloop.runs\impl_1\bd_wrapper.bit `
    --script oneshot_capture.py --download 1 --full-init 1 --pull-dir picture
```

期待: `SAVED .../pic_*.png written_rows=480/480 long~7600/s`。
クリーンな単フレーム (タイル化なし)。

### 4.3 HDMI ライブ出力

```powershell
.\scripts\hdmi.ps1                    # 180 s ライブ出力
.\scripts\hdmi.ps1 -Seconds 60 -Vcm 280
.\scripts\hdmi.ps1 -Sweep             # 出力中に VCM を段階送り (ピント探し)
```

> **絶対に実行中に Ctrl+C / kill しないこと。** VDMA (S2MM+MM2S) 稼働中の
> ジョブを止めると VDMA cleanup が間に合わず **sshd ハング → 物理電源サイクル
> 必須** (memory `feedback_never_taskstop_vdma_running_job`)。`-Seconds` で
> 必要な長さを指定し、自然終了を待つ。

---

## 5. 設定モデル (runtime ノブ)

`frame_lines_runtime_word` (AXI GPIO `frame_lines_gpio`、32-bit):

| bit | 意味 |
|---|---|
| [15:0] | frame lines (480) |
| [16] | `cfg_use_lsle` |
| [23:17] | `expected_long_dt` (RGB565=0x22) |
| [24] | apply strobe |
| [25] | `cam_gpio` (OV5640 RESETB) |
| [26] | `use_tpg_rt` |
| [28:27] | TPG pattern sel |
| **[29]** | **`sup_enable`** (D-PHY lane supervisor、opt-in) |
| **[30]** | **`cfg_sof_synth`** (FS 不在時に LS でフレームを開く、opt-in) |

撮影時の chip 側: `0x4800=0x34` (gated clock)、RGB565 (`0x4300=0x6F`,
`0x501F=0x01`、stream cycle 必須)、`expected_dt=0x22`、bitslip 0/6、idelay 8/8。
**`sup_enable=1` + `cfg_sof_synth=1` がフル 480 行画像化の鍵**。

VCM フォーカス (OV5640、10-bit DAC): `0x3603[5:0]=D[9:4]` (bit7=PD),
`0x3602[7:4]=D[3:0]`。既定 code≈21。ボケは主に被写体距離 (固定焦点)。

---

## 6. 残課題 (再現性とは別、後続)

- VDMA フリーラン (`C_USE_S2MM_FSYNC=0`) のタイル化 → BD で `=2`(s2mm_tuser)
  + genlock 化 + rebuild (oneshot は grab 側で暫定回避)
- continuous クロック (`0x4800=0x14`) の cold-attach escape が実機未 engage
- supervisor / SOF 合成の恒久 default 化 (bit29/30 opt-in → 既定 ON)
- frontend での FS 根本捕捉 (SOF 合成は core 側の回避策)
