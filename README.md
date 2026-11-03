# DASH

**A da Vinci Adapter for Serial-link and Humanoid Robots as an Accessible
Platform for Surgical Robotics Research**
([arXiv:2610.05792](https://arxiv.org/abs/2610.05792))

DASH mounts da Vinci instruments on general-purpose robots instead. Its adapters need no
modification to the instruments: each one engages the instrument's native
latch, drives its disks with DYNAMIXEL servos, and identifies the inserted
instrument over RFID to load that instrument's kinematics and coupling matrix.

This repository holds the software that teleoperates both arms of a Unitree
G1 humanoid from a da Vinci Research Kit (dVRK) console through two DASH
adapters, and the adapter CAD.

## Contents

| Folder | What is in it |
|---|---|
| [`DASH_software/`](DASH_software) | The ROS 2 teleoperation stack: the RCM controller for the G1 arms, the DYNAMIXEL instrument driver, RFID instrument loading, the OpenRB-150 board firmware, configs for 46 instruments, and MuJoCo models to run it all in simulation. Start with [its README](DASH_software/README.md). |
| [`DASH_hardware/`](DASH_hardware) | SolidWorks CAD for the DASH Classic adapter. See [its README](DASH_hardware/README.md). |

## Disclaimers

DASH is research software and hardware. It is not FDA approved, must never be
used on patients, and comes with no warranty. Read the full
[disclaimers](DASH_software/README.md#disclaimers), including the safety
requirements for running it on a robot, before you start.

## Credit / Citation

If you use DASH in your research, please cite the following paper:

[DASH: A da Vinci Adapter for Serial-link and Humanoid Robots as an Accessible Platform for Surgical Robotics Research](https://arxiv.org/abs/2610.05792)

Sara Wickenhiser and Junrong Zhou and Zekai Liang and Lizzie Peiros and Liu
Shanglei and Michael C. Yip

arXiv preprint arXiv:2610.05792 [cs.RO], October 2026.

[bibtex](DASH_software/DASH.bib)

## License

| Folder | License |
|---|---|
| `DASH_software/` | [MIT License](DASH_software/LICENSE). The third-party robot and instrument models in `DASH_software/assets/` keep their own licenses, listed in that folder. |
| `DASH_hardware/` | [CC BY 4.0](DASH_hardware/LICENSE), with citation of the paper above as the required credit. The models of purchased parts are not covered; see [its License section](DASH_hardware/README.md#license). |
