# experiments/

One-off measurement scripts. **Dev-only, not part of the sim toolchain** — they need
`numpy` + `opencv-python-headless` + `Pillow` in the **repo-root CPython venv**
(`.venv/`, not the MSYS2 ucrt64 sim interpreter, which is stdlib-only). They import the
stdlib `golden.py` / `image_io.py` from `../img_file_uvm/`.

```powershell
.\.venv\Scripts\python.exe -m pip install opencv-python-headless   # numpy + Pillow already present
.\.venv\Scripts\python.exe verification\cocotb\experiments\opencv_compare.py
.\.venv\Scripts\python.exe verification\cocotb\experiments\viz_compose.py
```

| script | what it does |
|--------|--------------|
| `opencv_compare.py` | Diffs OpenCV output vs the RTL-exact golden and decomposes the divergence into shift / rounding / border. Prints the counts behind the "OpenCV as the oracle" table in [`image_file_verification.md`](../../../docs/doc/image_file_verification.md). Output PNGs → gitignored `_exec/opencv_exp/`. |
| `viz_compose.py` | Regenerates the committed composite figures `docs/doc/samples/img_file_uvm/opencv_vs_rtl_{conv3x3,median}.png` (input / golden / OpenCV / diff-heatmap). |
