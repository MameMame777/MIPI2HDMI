# Third-Party Notices

This project incorporates material from the third-party projects listed below.
Their copyright and license notices are reproduced here as required by their
licenses. The project's own code is under the MIT License (see [LICENSE](LICENSE)).

---

## Digilent MIPI D-PHY Receiver IP

The following files are **derivative works** — functional SystemVerilog
re-implementations of mechanisms from Digilent's MIPI D-PHY Receiver IP
(`DPHY_Pkg.vhd` / `HS_Clocking.vhd` / `DPHY_LaneSCNN` / `DPHY_LaneSFEN` and the
`SyncAsync` / `GlitchFilter` / `ResetBridge` CDC primitives), originally authored
by Elod Gyorgy:

- `rtl/mipi_rx/dphy_lane_supervisor.sv`
- `rtl/mipi_rx/dphy_cdc_prims.sv`

Both are MIT-licensed, the same license as this project, so redistribution and
modification are permitted provided the notice below is retained.

Original copyright and permission notice:

```
MIT License

Copyright (c) 2016 Digilent

Permission is hereby granted, free of charge, to any person obtaining a copy
of this software and associated documentation files (the "Software"), to deal
in the Software without restriction, including without limitation the rights
to use, copy, modify, merge, publish, distribute, sublicense, and/or sell
copies of the Software, and to permit persons to whom the Software is
furnished to do so, subject to the following conditions:

The above copyright notice and this permission notice shall be included in all
copies or substantial portions of the Software.

THE SOFTWARE IS PROVIDED "AS IS", WITHOUT WARRANTY OF ANY KIND, EXPRESS OR
IMPLIED, INCLUDING BUT NOT LIMITED TO THE WARRANTIES OF MERCHANTABILITY,
FITNESS FOR A PARTICULAR PURPOSE AND NONINFRINGEMENT. IN NO EVENT SHALL THE
AUTHORS OR COPYRIGHT HOLDERS BE LIABLE FOR ANY CLAIM, DAMAGES OR OTHER
LIABILITY, WHETHER IN AN ACTION OF CONTRACT, TORT OR OTHERWISE, ARISING FROM,
OUT OF OR IN CONNECTION WITH THE SOFTWARE OR THE USE OR OTHER DEALINGS IN THE
SOFTWARE.
```

Upstream: Digilent `vivado-library` — MIPI D-PHY Receiver IP.
