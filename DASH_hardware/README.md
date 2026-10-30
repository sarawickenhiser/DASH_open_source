# DASH hardware

CAD models for the DASH instrument adapter. The software that drives it, the
firmware for its board, and the list of electronics it needs are in
[`DASH_software/`](../DASH_software).

Every file here is a SolidWorks part (`.SLDPRT`) or assembly (`.SLDASM`).

## `DASH Classic/`

The adapter for da Vinci Classic instruments. **Open `DASH_C.SLDASM`**: it is
the full adapter assembly and references every other file in this folder, so
keep them together in one directory.

| File | What it is |
|---|---|
| `DASH_C.SLDASM` | The full adapter assembly. Start here. |
| `Assem1.SLDASM` | An assembly that `DASH_C.SLDASM` references. |
| `Base.SLDPRT`, `Plate.SLDPRT`, `Track.SLDPRT`, `SpringBase.SLDPRT`, `SpringTop.SLDPRT` | The adapter's own parts. |
| `XL,XC-330.SLDASM` | ROBOTIS DYNAMIXEL XC330 servo. Each adapter uses four XC330-T181-T. |
| `OpenRB-150.SLDASM` | ROBOTIS OpenRB-150 board, which runs the servo bus and the RFID reader. |
| `RFID SCANNER CAD.SLDPRT` | The PN532 RFID reader that identifies the inserted instrument. |

**The servo and board models are incomplete.** `XL,XC-330.SLDASM` and
`OpenRB-150.SLDASM` are imported models of the purchased ROBOTIS parts, and
the part files inside them are not in this repo (11 for the servo, 58 for the
board). SolidWorks lists those as missing when you open `DASH_C.SLDASM`; the
adapter's own parts load normally. If you need the full servo or board
geometry, use ROBOTIS's own CAD models of the XC330 and OpenRB-150.

## `DASH_misc/`

Standalone parts. No assembly in this folder references them.

| File | What it is |
|---|---|
| `huamnoid_mount.SLDPRT` | Connects to surgie_wrist_T and to the underside of any DASH adapter. |
| `surgie_wrist_T.SLDPRT` | What actually connects to Unitree G1 wrist. |

## Building one

Each adapter also needs the electronics listed under
[What you need](../DASH_software/README.md#what-you-need) in the software
README: one OpenRB-150, four XC330-T181-T servos and a PN532 reader. That
README then covers flashing the board, setting the servo IDs and registering
your instruments' RFID tags.

The MuJoCo simulation uses STL meshes of the adapter, which are in
[`DASH_software/assets/dvrk_adapter/meshes/`](../DASH_software/assets/dvrk_adapter/meshes).

## Disclaimers

The [disclaimers in the software README](../DASH_software/README.md#disclaimers)
cover these designs too. In particular, DASH is not FDA approved, must never be
used on patients, and comes with no warranty.

## Credit / Citation

If you use DASH in your research, please cite the following paper:

[DASH: A da Vinci Adapter for Serial-link and Humanoid Robots as an Accessible Platform for Surgical Robotics Research](https://arxiv.org/abs/2610.05792)

Sara Wickenhiser and Junrong Zhou and Zekai Liang and Lizzie Peiros and Liu
Shanglei and Michael C. Yip

arXiv preprint arXiv:2610.05792 [cs.RO], October 2026.

[bibtex](../DASH_software/DASH.bib)

## License

The DASH designs in this folder are © 2026 Sara Wickenhiser and are licensed
under the [Creative Commons Attribution 4.0 International License](LICENSE)
(CC BY 4.0). You may share and adapt them for any purpose, including
commercially, as long as you give credit.

**Giving credit means citing the paper.** The attribution this license
requires is a citation of the paper under
[Credit / Citation](#credit--citation), in any publication, presentation,
product documentation or design files that use these designs or work based on
them. Also link to this repository, state that the designs are licensed under
CC BY 4.0, and say whether you changed them.

**Not covered.** These are models of purchased parts, included so that the
assembly opens. They are not licensed under CC BY 4.0 and belong to their
makers:

- `XL,XC-330.SLDASM`, the ROBOTIS DYNAMIXEL XC330 servo
- `OpenRB-150.SLDASM`, the ROBOTIS OpenRB-150 board
- `RFID SCANNER CAD.SLDPRT`, the PN532 NFC module
