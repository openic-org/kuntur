# Kuntur

**Kuntur** (Quechua for *condor*) is an open neural-recording headstage
built around Intan RHD2164 amplifiers, a Lattice CrossLink-NX FPGA and an
STM32WB09 Bluetooth LE microcontroller, together with the firmware, FPGA
designs and desktop software that go with it.

> **Status: pre-release, research use only.** This repository is under
> active development ahead of a first public release. Interfaces, file
> formats and hardware revisions may still change. Not for clinical use.

Kuntur has two data paths:

- **Wireless:** the headstage streams over BLE (2M PHY) to a NUCLEO-WB09KE
  running the **BLE–USB bridge** firmware, which forwards packets over USB
  serial to **Tunki** (Quechua for the Andean cock-of-the-rock, Peru's
  national bird), the desktop app for live display and recording.
- **Wired:** the headstage sends its samples over an LVDS link to the
  **RHD adapter**, a CrossLink-NX FPGA design that appears to an RHD-protocol
  recording controller as two RHD2164 chips.

## Repository layout

```
hardware/      KiCad designs, fab packages, BOMs and pick-and-place files
  kuntur144-ecl/            headstage digital board: FPGA, BLE MCU, power
  kuntur144-nil/            headstage front-end board: recording and stimulation sheets
  kuntur144-omnetics/       Omnetics connector board
  kuntur144-prog/           programming board
  kuntur144-prog-adapter/   programming adapter
fpga/          Lattice Radiant projects
  headstage/                headstage FPGA (LIFCL-17)
  rhd-adapter/              RHD adapter (LIFCL-40)
  scripts/                  build and testbench helpers
firmware/
  headstage/                STM32WB09 headstage firmware (STM32CubeIDE project)
  ble-usb-bridge/           NUCLEO-WB09KE BLE central → USB serial bridge (Makefile)
software/
  tunki/                    PyQt6 desktop app: live display, recording, analysis
tools/                      (reserved)
```

## Building

**Tunki** (Python ≥ 3.11):

```bash
cd software/tunki
pip install -r requirements.txt
python main.py
```

**Firmware** needs the Arm GNU toolchain shipped with STM32CubeIDE and the
STM32Cube WB0 firmware package. Both projects currently locate that package
through absolute paths set by STM32CubeIDE (headstage) or at the top of the
`Makefile` (bridge, `CUBE_DRV_ROOT` / `CUBE_MW_ROOT`); adjust them to your
install. Making these paths portable is planned.

```bash
cd firmware/headstage/Debug && make all -j8     # → kuntur-mcu.elf / .hex
cd firmware/ble-usb-bridge && make -j8          # → build/ble_usb_bridge.bin
```

**FPGA** projects open in Lattice Radiant 2025.2 (`fpga/headstage/kuntur_fpga.rdf`,
`fpga/rhd-adapter/intan_rec_adapter.rdf`). Released bitstreams are tracked
next to each implementation (`impl_*/*.bit`).

## Licences

- `hardware/` and `fpga/`: **CERN Open Hardware Licence v2 – Permissive**
  (`LICENSES/CERN-OHL-P-2.0.txt`)
- everything else: **Apache License 2.0** (`LICENSES/Apache-2.0.txt`)

Third-party files (ST and Lattice generated code, the vendored bridge base in
`firmware/ble-usb-bridge/base/`) keep their own notices. See `LICENSE`.

Intan, RHD2164 and RHX are trademarks of Intan Technologies; this project is
not affiliated with or endorsed by Intan Technologies.
