# tools/

Standalone analysis and validation tools that are not part of the Tunki app.

Empty for now: these tools still live next to Tunki in
[`software/tunki/`](../software/tunki/) and are run from there; several of them
(e.g. `analyze_recording.py`, `measure_rate.py`) import Tunki's own modules, so
moving them needs an import path. They are planned to move here.

| Script (in `software/tunki/`) | What it does |
|---|---|
| `analyze_recording.py` | sample-level QA of a recording |
| `compare_recordings.py` | before/after diff of two recordings |
| `measure_rate.py` | per-channel sample rate measured from the packet headers |
| `correlate_stalls.py` | lines up dropped-sample events with a UART stall log |
| `test_validator.py` | headless live-stream packet validator (`--port`) |
| `mosi_eye_scan.py` | maps each RHD2164's MOSI timing window |
| `chip0_first_err.py` | captures the first wrong MISO answer from a chip |
| `ambient_logger.py` | room temperature / humidity log for bench trials |
| `plot_bench.py`, `plot_gpio_bench.py`, `plot_template.py` | bench and recording plots |

Board-level helpers for the FPGA projects are in [`fpga/scripts/`](../fpga/scripts/).
