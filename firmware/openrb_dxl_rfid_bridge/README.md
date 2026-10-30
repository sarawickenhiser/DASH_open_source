# openrb_dxl_rfid_bridge

Firmware for the OpenRB-150 carrying the instrument adapter: one board doing
both the DYNAMIXEL bus and the PN532 RFID reader, because they share a single
USB serial handle.

Flash it to **both** adapters' boards. (In the lab it was written for the left
adapter; the right one ran an older, unpublished sketch that is permanently
transparent and sniffs the byte stream for a `SCAN` line. The host nodes
support both, so a board running either works.)

## Before you flash

You need:

- the **OpenRB-150** board package (ROBOTIS board manager URL → *OpenRB-150*),
- the **Adafruit PN532** library, and its **Adafruit BusIO** dependency.

Board: *OpenRB-150*. Port: whichever `/dev/serial/by-id/usb-ROBOTIS_OpenRB-150_*`
the board enumerates as.

The sketch expects the PN532 on **I2C**, with IRQ on pin 2 and RESET on pin 3
(`PN532_IRQ` / `PN532_RESET`; set both to `-1` if they are not wired), and the
servos at **1 Mbps** (`dxl_baud`), which must match the servos' Baud Rate
register and `dynamixel_controller_node`'s `baudrate` parameter.

## Why v2 exists

v1 powered up in `RFID_MODE`. `dynamixel_controller_node` opens the port and
immediately writes DYNAMIXEL packets; in RFID mode those get parsed as text,
match no command, and come back as `ERROR:UNKNOWN_CMD:<bytes>`. The SDK reads
that ASCII where a status packet belongs and reports **`Incorrect status
packet!`** on every transaction — including the torque-off writes on shutdown,
so the tool could not even be stood down cleanly. It reads exactly like a
wiring or baud fault and is neither.

v2 boots into `BRIDGE_MODE`, so the motors work the instant the board powers
up, with no host cooperation at all.

## Protocol

**BRIDGE_MODE** (LED on, power-on default) — transparent USB ↔ DYNAMIXEL. The
only thing not passed through is the 9-byte escape:

```
FF FF FD 00 FE 'R' 'F' 'I' 'D'
```

It borrows the protocol-2.0 header and broadcast ID, so no genuine packet can
contain it (one would have to declare a length of `0x4652`). This is
deliberately safer than the right board's habit of sniffing for the ASCII word
`SCAN`, which misfires if a real packet ever contains those five bytes.

**RFID_MODE** (LED off) — one ASCII command per line:

| command | reply |
|---|---|
| `BRIDGE` | `STATUS:BRIDGE_MODE` |
| `SCAN` | `UID:<hex>` or `UID:NONE` |
| `PING` | `STATUS:RFID_MODE` |
| `STATUS` | mode, DXL baud, reader presence, DXL power pin state |
| `VERSION` | `VERSION:openrb-dxl-rfid-bridge/2.0` |
| `DXLSCAN [maxid]` | `DXL:<id>:<model>` per hit, then `DXLDONE:<n>` |
| `DXLBAUD <rate>` | `STATUS:DXLBAUD:<rate>` |

An idle watchdog returns the board to bridge mode after 5 s without a command,
so a host that dies mid-scan can never leave the motors deaf.

## Bring-up, in order

With nothing else holding the port:

```bash
python3 - <<'PY'
import serial, time
p = "/dev/serial/by-id/usb-ROBOTIS_OpenRB-150_<your board>-if00"
s = serial.Serial(p, 57600, timeout=1.0); s.dtr = False; time.sleep(0.3)
s.reset_input_buffer()
s.write(bytes([0xFF,0xFF,0xFD,0x00,0xFE])+b"RFID"); time.sleep(0.3)   # -> RFID mode
for cmd in (b"VERSION\n", b"STATUS\n", b"DXLSCAN 10\n", b"SCAN\n"):
    s.write(cmd); time.sleep(1.0)
    print(cmd.strip().decode(), "->", s.read(s.in_waiting or 1))
s.write(b"BRIDGE\n"); time.sleep(0.2); print(s.read(s.in_waiting or 1))
PY
```

`DXLSCAN` pings the bus **from the board**, which separates two failures that
otherwise look identical from the host: *the bridge is not bridging* versus
*there are no live servos*. If it reports `DXLDONE:0`, the bridge is fine and
the problem is downstream — check `DXLPWR` in the `STATUS` line first, since
`BDPIN_DXL_PWR_EN` (31) is a guess inherited from v1 and an unpowered bus is
indistinguishable from an empty one.

Then bring the pipeline up normally:

```bash
bash humanoid_rcm_hardware_bundle/start_hw.sh
```

## Host side

Keep these in step with this file — they encode the protocol above:

- `dynamixel_controller_node.claim_bridge_mode()` — writes `BRIDGE\n` after
  open and after every reopen, then drains. Unconditional on purpose: it is 7
  inert ASCII bytes on transparent firmware. It must **not** send the escape
  instead, which would make a servo buffer 18002 bytes it never receives.
- `rfid_scan_node.do_scan()` — plain `SCAN` first (so the right board is
  untouched), then escape → `SCAN` → `BRIDGE`, caching which firmware it found.
