/*
  OpenRB-150: DYNAMIXEL bridge + PN532 RFID on one board.
  Firmware for the LEFT-side instrument adapter.  v2, 2026-09-05.

  ---------------------------------------------------------------------------
  WHAT THE HOST EXPECTS OF THIS BOARD
  ---------------------------------------------------------------------------
  Two nodes on the ROS side talk to this firmware, and both were written
  against its behaviour -- keep them in step if you change anything here:

    dynamixel_controller_node.claim_bridge_mode()
        writes "BRIDGE\n" after opening and after every reopen, then drains.

    rfid_scan_node.do_scan()
        writes "SCAN\n" first (the older right-hand firmware answers that
        directly); on silence, writes the 9-byte escape, expects
        STATUS:RFID_MODE, writes "SCAN\n", reads UID:, then writes "BRIDGE\n"
        and expects STATUS:BRIDGE_MODE.

  BRIDGE_MODE (LED on) -- the power-on default
    Transparent USB <-> DYNAMIXEL pass-through.  Anything the host writes goes
    out on Serial1 and anything the bus answers goes back to USB, byte for
    byte, with one exception: the escape sequence returns to RFID mode.

      escape = FF FF FD 00 FE 'R' 'F' 'I' 'D'

    It borrows the DYNAMIXEL 2.0 header + broadcast ID so it can never be
    produced by a real packet (that would have to declare a length of 0x4652).
    This is deliberately better than the older board's habit of sniffing the
    binary stream for the ASCII word "SCAN", which misfires the moment a real
    packet happens to contain those five bytes.

  RFID_MODE (LED off)
    Line-based ASCII, one command per line:
      BRIDGE          -> enter bridge mode,    replies STATUS:BRIDGE_MODE
      SCAN            -> single PN532 read,    replies UID:<hex> or UID:NONE
      PING            -> replies STATUS:RFID_MODE
      STATUS          -> one line of mode / baud / reader / power state
      VERSION         -> replies VERSION:<string>
      DXLSCAN [maxid] -> ping DYNAMIXEL IDs 1..maxid (default 10) FROM THE
                         BOARD, replies DXL:<id>:<model> per hit then DXLDONE:n
      DXLBAUD <rate>  -> re-open Serial1 at <rate> (9600..3000000),
                         replies STATUS:DXLBAUD:<rate>

  Nothing here blocks forever: a missing PN532 is reported per-SCAN rather
  than locked on, so a dead reader can never take the motors down with it.

  ---------------------------------------------------------------------------
  WHY THE BOOT MODE IS BRIDGE, NOT RFID
  ---------------------------------------------------------------------------
  v1 powered up in RFID_MODE.  That is the wrong default and it produced a
  failure that reads as a wiring or baud fault when it is neither:

    dynamixel_controller_node opens the port and immediately writes DYNAMIXEL
    packets.  In RFID_MODE those land in rfid_mode()'s readStringUntil('\n'),
    match no command, and come back as ERROR:UNKNOWN_CMD:<packet bytes>.  The
    SDK reads that ASCII where a status packet belongs and reports "Incorrect
    status packet!" on every single transaction -- including the torque-off
    writes on shutdown, so the tool cannot even be stood down cleanly.

  Booting into BRIDGE_MODE makes the motors work the instant the board powers
  up, with no host cooperation at all.  The mode protocol is still the real
  interface and the host still drives it explicitly; the boot mode only
  decides who wins when the host says nothing, and "motors work" is a far
  better answer to that than "nothing works".
*/

#include <string.h>
#include <Wire.h>
#include <Adafruit_PN532.h>

#define FW_VERSION "openrb-dxl-rfid-bridge/2.0"

#define DXL_BUS Serial1
#define HOST    Serial   // not "USB": that name is the SAMD21 peripheral macro

// The DYNAMIXEL bus is fed through a FET on this pin.  Serial1.begin() does not
// touch it -- only Dynamixel2Arduino::begin() does (see SerialPortHandler::begin,
// ARDUINO_OpenRB branch).  A raw-Serial1 sketch that skips this leaves every
// motor unpowered and silent at any baud rate, which looks identical to having
// no motors on the bus.  If DXLSCAN finds nothing after flashing, suspect this
// pin number before you suspect the servos.
#ifndef BDPIN_DXL_PWR_EN
#define BDPIN_DXL_PWR_EN (31u)
#endif
#define DXL_PWR_SETTLE_MS 500

#define BUF_LEN      1024
#define HOST_TIMEOUT  30    // ms, bounds readStringUntil()
#define SCAN_TIMEOUT 200    // ms the PN532 waits for a tag
#define ESC_FLUSH_MS   5    // ms before a partial escape match is released
#define MAX_DXL_BAUD 3000000UL  // 48 MHz / 16x oversampling; above this Serial1 stalls

// How long the board will sit in RFID_MODE with the host saying nothing before
// it puts itself back into bridge mode.
//
// This exists because RFID_MODE is the state in which the motors do not work.
// A host that dies between the escape and its closing BRIDGE -- rfid_scan_node
// killed mid-scan, a USB hiccup, an operator's Ctrl+C landing in the wrong
// pane -- would otherwise leave this board deaf to DYNAMIXEL traffic until
// somebody power-cycled it, and the symptom would be the same "Incorrect
// status packet!" storm that v1's boot mode caused.
//
// 5 s is far longer than a real scan (escape -> SCAN -> BRIDGE is a few
// hundred ms, bounded by SCAN_TIMEOUT) and far shorter than a human noticing.
// The timer is reset by every command received, so a busy reader never trips it.
#define RFID_IDLE_WATCHDOG_MS 5000

// Per-ID budget for DXLSCAN.  A servo answers a ping in well under a
// millisecond at any sane baud; this is the give-up time for one that is not
// there, and it multiplies by the ID count, so keep it small.
#define DXL_PING_TIMEOUT_MS 20

// PN532 wired for I2C.  Change to (-1, -1) if IRQ/RESET are not connected.
#define PN532_IRQ   2
#define PN532_RESET 3
Adafruit_PN532 nfc(PN532_IRQ, PN532_RESET);

uint8_t buf[BUF_LEN];

enum Mode { RFID_MODE, BRIDGE_MODE };

// Power-on default.  See the header comment.
Mode mode = BRIDGE_MODE;

const uint8_t ESC[] = {0xFF, 0xFF, 0xFD, 0x00, 0xFE, 'R', 'F', 'I', 'D'};
const int ESC_LEN = sizeof(ESC);

// Bytes held back because they are still a prefix of ESC.  hold[] always holds
// exactly hold_len bytes and always equals ESC[0..hold_len-1].
uint8_t hold[ESC_LEN];
int hold_len = 0;
unsigned long hold_since_ms = 0;

// Last time a command was accepted in RFID mode; drives the watchdog above.
unsigned long rfid_since_ms = 0;

bool pn532_ok = false;
// Must match the servos' Baud Rate register (3 = 1 Mbps, set in DYNAMIXEL
// Wizard). Not persisted: DXLBAUD changes it only until the next power-up.
uint32_t dxl_baud = 1000000;

// ---------------------------------------------------------------- helpers

void drain_dxl() {
  while (DXL_BUS.available()) DXL_BUS.read();
}

void flush_hold() {
  for (int i = 0; i < hold_len; i++) DXL_BUS.write(hold[i]);
  hold_len = 0;
}

bool init_pn532() {
  nfc.begin();
  if (!nfc.getFirmwareVersion()) return false;
  nfc.SAMConfig();
  return true;
}

void enter_rfid_mode() {
  hold_len = 0;            // the held bytes are the escape itself
  drain_dxl();
  mode = RFID_MODE;
  rfid_since_ms = millis();
  digitalWrite(LED_BUILTIN, LOW);
  HOST.println("STATUS:RFID_MODE");
}

void enter_bridge_mode() {
  drain_dxl();
  hold_len = 0;
  hold_since_ms = millis();
  mode = BRIDGE_MODE;
  digitalWrite(LED_BUILTIN, HIGH);
  HOST.println("STATUS:BRIDGE_MODE");
}

// ------------------------------------------------------ DYNAMIXEL protocol
//
// Only enough of protocol 2.0 to ping.  This is a bring-up aid, not a second
// driver: it answers "is there a live DYNAMIXEL bus behind this board at all",
// which is otherwise indistinguishable from a bridge that is not bridging.

// CRC-16/IBM, MSB-first, poly 0x8005, init 0 -- the same function ROBOTIS
// ships as a 256-entry table, done bitwise because flash is cheaper than the
// table is readable.
uint16_t dxl_crc16(const uint8_t *p, uint16_t n) {
  uint16_t crc = 0;
  for (uint16_t i = 0; i < n; i++) {
    crc ^= (uint16_t)p[i] << 8;
    for (uint8_t b = 0; b < 8; b++)
      crc = (crc & 0x8000) ? (uint16_t)((crc << 1) ^ 0x8005) : (uint16_t)(crc << 1);
  }
  return crc;
}

// Returns true and fills model_out if <id> answers.  No byte stuffing is
// needed: a ping carries no parameters, so no FF FF FD can occur in the body.
bool dxl_ping(uint8_t id, uint16_t *model_out) {
  //            header        id  len_l len_h inst  crc_l crc_h
  uint8_t pkt[10] = {0xFF, 0xFF, 0xFD, 0x00, id, 0x03, 0x00, 0x01, 0x00, 0x00};
  uint16_t crc = dxl_crc16(pkt, 8);
  pkt[8] = (uint8_t)(crc & 0xFF);
  pkt[9] = (uint8_t)((crc >> 8) & 0xFF);

  drain_dxl();
  DXL_BUS.write(pkt, sizeof(pkt));
  DXL_BUS.flush();          // half-duplex: finish sending before listening

  // A ping status packet is 14 bytes:
  //   FF FF FD 00 ID 07 00 55 ERR MODEL_L MODEL_H FW CRC_L CRC_H
  uint8_t rx[32];
  int n = 0;
  unsigned long t0 = millis();
  while ((millis() - t0) < DXL_PING_TIMEOUT_MS && n < (int)sizeof(rx)) {
    if (DXL_BUS.available()) rx[n++] = (uint8_t)DXL_BUS.read();
  }

  for (int i = 0; i + 13 < n; i++) {
    if (rx[i] == 0xFF && rx[i+1] == 0xFF && rx[i+2] == 0xFD && rx[i+3] == 0x00
        && rx[i+4] == id && rx[i+7] == 0x55) {
      *model_out = (uint16_t)(rx[i+9] | ((uint16_t)rx[i+10] << 8));
      return true;
    }
  }
  return false;
}

// ------------------------------------------------------------------ setup

void setup() {
  pinMode(LED_BUILTIN, OUTPUT);
  digitalWrite(LED_BUILTIN, LOW);   // off while booting; set to the mode below

  // No `while (!USB)` here on purpose: it would hang the board -- and with it
  // the motor bus -- whenever it is powered without a host attached.
  HOST.begin(57600);
  HOST.setTimeout(HOST_TIMEOUT);

  pinMode(BDPIN_DXL_PWR_EN, OUTPUT);
  digitalWrite(BDPIN_DXL_PWR_EN, HIGH);
  delay(DXL_PWR_SETTLE_MS);           // let the FET turn on and the motors boot

  DXL_BUS.begin(dxl_baud);

  Wire.begin();
  pn532_ok = init_pn532();

  // Derive the LED and the announced state from the boot mode rather than
  // hardcoding the RFID side, which would quietly lie the moment the default
  // changed -- as it did.
  digitalWrite(LED_BUILTIN, mode == BRIDGE_MODE ? HIGH : LOW);
  hold_since_ms = rfid_since_ms = millis();
  HOST.println(mode == BRIDGE_MODE ? "STATUS:BRIDGE_MODE" : "STATUS:RFID_MODE");
  if (!pn532_ok) HOST.println("ERROR:NO_PN532");
}

void loop() {
  if (mode == RFID_MODE) rfid_mode();
  else                   bridge_mode();
}

// -------------------------------------------------------------- RFID mode

void print_status_line() {
  HOST.print("STATUS:RFID_MODE DXLBAUD:");
  HOST.print(dxl_baud);
  HOST.print(" PN532:");
  HOST.print(pn532_ok ? "OK" : "MISSING");
  HOST.print(" DXLPWR:");
  HOST.println(digitalRead(BDPIN_DXL_PWR_EN) ? "ON" : "OFF");
}

void do_scan_command() {
  if (!pn532_ok) {
    pn532_ok = init_pn532();          // retry, in case it came back
    if (!pn532_ok) { HOST.println("ERROR:NO_PN532"); return; }
  }

  uint8_t uid[7], len;
  if (nfc.readPassiveTargetID(PN532_MIFARE_ISO14443A, uid, &len, SCAN_TIMEOUT)) {
    HOST.print("UID:");
    for (int i = 0; i < len; i++) {
      if (uid[i] < 0x10) HOST.print("0");
      HOST.print(uid[i], HEX);
    }
    HOST.println();
  } else {
    HOST.println("UID:NONE");
  }
}

void do_dxlscan_command(const String &cmd) {
  int max_id = 10;
  int sp = cmd.indexOf(' ');
  if (sp >= 0) {
    int v = cmd.substring(sp + 1).toInt();
    if (v >= 1 && v <= 252) max_id = v;
  }

  int found = 0;
  for (int id = 1; id <= max_id; id++) {
    uint16_t model = 0;
    if (dxl_ping((uint8_t)id, &model)) {
      found++;
      HOST.print("DXL:"); HOST.print(id);
      HOST.print(":");    HOST.println(model);
    }
  }
  HOST.print("DXLDONE:"); HOST.println(found);
}

void rfid_mode() {
  // Never stay deaf to the motor bus indefinitely.  See RFID_IDLE_WATCHDOG_MS.
  //
  // Safe to announce: this only fires once the host has stopped talking, and
  // both host nodes drain the port before their next DYNAMIXEL transaction
  // (rfid_scan_node resets the input buffer on open, and the driver's
  // claim_bridge_mode does the same on reclaim), so the STATUS line cannot be
  // mistaken for a status packet.
  if ((millis() - rfid_since_ms) >= RFID_IDLE_WATCHDOG_MS) {
    HOST.println("STATUS:RFID_TIMEOUT");
    enter_bridge_mode();
    return;
  }

  if (!HOST.available()) return;

  String cmd = HOST.readStringUntil('\n');
  cmd.trim();
  if (cmd.length() == 0) return;

  rfid_since_ms = millis();          // any traffic keeps the watchdog back

  if (cmd == "BRIDGE") {
    enter_bridge_mode();
  }
  else if (cmd == "PING") {
    HOST.println("STATUS:RFID_MODE");
  }
  else if (cmd == "STATUS") {
    print_status_line();
  }
  else if (cmd == "VERSION") {
    HOST.print("VERSION:"); HOST.println(FW_VERSION);
  }
  else if (cmd == "SCAN") {
    do_scan_command();
    rfid_since_ms = millis();        // a scan can take SCAN_TIMEOUT by itself
  }
  else if (cmd == "DXLSCAN" || cmd.startsWith("DXLSCAN ")) {
    do_dxlscan_command(cmd);
    rfid_since_ms = millis();        // ditto, and it scales with the ID count
  }
  else if (cmd.startsWith("DXLBAUD ")) {
    uint32_t rate = (uint32_t) cmd.substring(8).toInt();
    if (rate >= 9600 && rate <= MAX_DXL_BAUD) {
      dxl_baud = rate;
      DXL_BUS.end();
      DXL_BUS.begin(dxl_baud);
      drain_dxl();
      HOST.print("STATUS:DXLBAUD:"); HOST.println(dxl_baud);
    } else {
      HOST.println("ERROR:BAD_BAUD");
    }
  }
  else {
    // Echo back printable characters only.  An unrecognised "command" here is
    // usually a DYNAMIXEL packet that arrived while the board was in the wrong
    // mode, and echoing it raw would put the bytes FF FF FD 00 back onto the
    // host stream -- a convincing fake packet header for whatever reads next.
    HOST.print("ERROR:UNKNOWN_CMD:");
    for (int i = 0; i < cmd.length(); i++) {
      char c = cmd.charAt(i);
      HOST.print((c >= 0x20 && c < 0x7F) ? c : '.');
    }
    HOST.println();
  }
}

// ------------------------------------------------------------ bridge mode

void bridge_mode() {
  // HOST -> DYNAMIXEL, watching for the escape sequence.
  //
  // Only take a byte when the bus can absorb a whole escape's worth, so the
  // shedding loop below can always finish.  Writing into a full TX buffer
  // blocks, and a blocked bridge stops draining USB, which wedges the board
  // until it is power-cycled -- exactly what a bad DXLBAUD used to cause.
  while (HOST.available() && DXL_BUS.availableForWrite() >= ESC_LEN) {
    uint8_t c = HOST.read();

    // Append, then shed leading bytes until what we hold is a prefix of ESC
    // again.  Every byte shed goes out on the bus in order, so the stream is
    // preserved byte for byte and no partial match can hide a later one
    // (FF FF FF FD 00 FE R F I D still escapes, on the second FF).
    hold[hold_len++] = c;

    while (hold_len > 0 && memcmp(hold, ESC, hold_len) != 0) {
      DXL_BUS.write(hold[0]);
      hold_len--;
      memmove(hold, hold + 1, hold_len);
    }

    hold_since_ms = millis();

    if (hold_len == ESC_LEN) {
      enter_rfid_mode();
      return;
    }
  }

  // A packet whose tail happens to match an escape prefix must not sit in
  // limbo waiting for a byte that never comes.
  if (hold_len > 0
      && (millis() - hold_since_ms) >= ESC_FLUSH_MS
      && DXL_BUS.availableForWrite() >= hold_len) {
    flush_hold();
  }

  // DYNAMIXEL -> HOST.  Never hand USB more than it can take: if the host
  // stops reading, the bridge must keep running rather than block here.
  int n = DXL_BUS.available();
  int room = HOST.availableForWrite();
  if (n > room) n = room;
  if (n > 0) {
    if (n > BUF_LEN) n = BUF_LEN;
    for (int i = 0; i < n; i++) buf[i] = DXL_BUS.read();
    HOST.write(buf, n);
  }
}
