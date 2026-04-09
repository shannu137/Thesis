"""
Jetson CCSDS Motor Controller — Safety-Enhanced
================================================
Mirrors the Teensy CCSDS protocol. Sends Telecommands (TC) to the Teensy
and receives/parses Telemetry (TLM) packets in return.

Packet layout:
  SYNC (2B) || Primary Header (6B) || Secondary Header (5B) || Payload (NB) || CRC (2B)

Requirements handled on Jetson side:
  Req  1 : Periodic heartbeat monitoring + watchdog for comms loss
  Req  2 : CRC / seq-no / ACK protocol — retransmit on NACK
  Req  3 : CRC-16 CCITT — discard corrupted, request retransmit
  Req  4 : Range check on received current / velocity / encoder
  Req 17 : No HB / telemetry → flag ESTOP condition
  Req 20 : Track no-command timeout (Teensy will ESTOP)
  Req 21 : Bounds check on all outgoing command values
"""

import serial
import struct
import time
import threading
import logging
from dataclasses import dataclass, field
from typing import Optional, Dict, Any, Callable

logging.basicConfig(level=logging.INFO, format="%(asctime)s [%(levelname)s] %(message)s")
log = logging.getLogger("JetsonCCSDS")

# ============================================================
#  PROTOCOL CONSTANTS  (must match Teensy)
# ============================================================
SYNC_WORD        = 0xAA55
SYNC_SIZE_B      = 2
PRIMARY_SIZE_B   = 6
SECONDARY_SIZE_B = 5
CRC_SIZE_B       = 2
MAX_PACKET_SIZE  = 512

CCSDS_VERSION    = 0
CCSDS_TYPE_TLM   = 0
CCSDS_TYPE_TC    = 1
CCSDS_SECHDR_FLAG = 1
CCSDS_SEQ_FLAG   = 3   # 0b11 — unsegmented

# Teensy ID (must match #define TEENSY_ID on firmware)
TEENSY_ID        = 1
NUM_MOTORS       = 3

# ============================================================
#  APID TABLE
# ============================================================
# Telemetry  (Teensy → Jetson)
APID_DATA_TLM    = 0x001   # encoder, velocity, current, aux_flags
APID_HEARTBEAT   = 0x002   # system_state, hb_count, aux_flags
APID_ACK         = 0x003   # ack_seq (2B), status (1B)
APID_NACK        = 0x004   # nack_seq (2B), status (1B)
APID_FAULT       = 0x005   # aux_flags (1B), motor_id (2B)
APID_TIME_SYNC   = 0x020   # t0(4B) req  /  t0+t1+t2(12B) resp

# Telecommand (Jetson → Teensy)
APID_VEL_SERVO   = 0x010   # float vel[3], int32 servo_pos[3]
APID_SYS_CMD     = 0x011   # command_id (1B)
# 0x020 also used for TC (time-sync request, 4B payload)

# ============================================================
#  AUX FLAGS
# ============================================================
AUX_CPU_OVERLOAD       = 0x01
AUX_ENCODER_FAULT      = 0x02
AUX_OVERCURRENT        = 0x04
AUX_STALL_DETECTED     = 0x08
AUX_CURRSENSOR_FAULT   = 0x10
AUX_DRIVER_FAULT       = 0x20
AUX_SENSOR_OOR         = 0x40
AUX_ESTOP_ACTIVE       = 0x80

AUX_FLAG_NAMES = {
    AUX_CPU_OVERLOAD:     "CPU_OVERLOAD",
    AUX_ENCODER_FAULT:    "ENCODER_FAULT",
    AUX_OVERCURRENT:      "OVERCURRENT",
    AUX_STALL_DETECTED:   "STALL_DETECTED",
    AUX_CURRSENSOR_FAULT: "CURRSENSOR_FAULT",
    AUX_DRIVER_FAULT:     "DRIVER_FAULT",
    AUX_SENSOR_OOR:       "SENSOR_OUT_OF_RANGE",
    AUX_ESTOP_ACTIVE:     "ESTOP_ACTIVE",
}

# ============================================================
#  NACK STATUS CODES
# ============================================================
NACK_CODES = {
    0x01: "INVALID_VERSION",
    0x02: "INVALID_TYPE",
    0x03: "UNKNOWN_APID",
    0x04: "SEQ_STALE",
    0x05: "INVALID_TEENSY_ID",
    0x06: "CRC_MISMATCH",
    0x20: "VALUE_OUT_OF_RANGE",
    0x30: "ESTOP_ACTIVE",
    0x31: "SAFE_ACTIVE",
    0x32: "INHIBIT_ACTIVE",
    0x33: "WRONG_MODE",
    0x34: "FAULTS_REMAIN",
}

# ============================================================
#  SYSTEM COMMANDS  (APID 0x011)
# ============================================================
CMD_STOP_ALL_MOTORS = 0x01
CMD_ENABLE_MOTORS   = 0x02
CMD_DISABLE_MOTORS  = 0x03
CMD_RESET_ENCODERS  = 0x04
CMD_ESTOP           = 0x05
CMD_RELEASE_ESTOP   = 0x06
CMD_REBOOT_TEENSY   = 0x08

# ============================================================
#  SYSTEM MODES  (received in heartbeat)
# ============================================================
MODE_NAMES = {
    0: "INIT", 1: "IDLE", 2: "NOMINAL",
    3: "DEGRADED", 4: "INHIBIT", 5: "SAFE", 6: "ESTOP",
}

# ============================================================
#  SAFETY THRESHOLDS  (Req 4 / 21)
# ============================================================
VEL_CMD_MAX    =  50.0    # rpm
VEL_CMD_MIN    = -50.0
SERVO_CMD_MAX  =  180
SERVO_CMD_MIN  =  0
POS_CMD_MAX    =  1_000_000
POS_CMD_MIN    = -1_000_000

CURR_MAX_VALID = 1.2 * 3500.0   # 20 % above stall threshold
VEL_MAX_VALID  = 1.5 *  50.0

# ============================================================
#  TIMING
# ============================================================
HEARTBEAT_TIMEOUT_S   = 3.0    # Req 17 — no HB → warn
TELEMETRY_TIMEOUT_S   = 0.5    # Req 17 — no TLM → warn
RETRANSMIT_TIMEOUT_S  = 0.2    # Req  2 — wait for ACK/NACK
MAX_RETRANSMIT        = 3      # Req  2

# ============================================================
#  HELPERS
# ============================================================
def crc16_ccitt(data: bytes) -> int:
    crc = 0xFFFF
    for byte in data:
        crc ^= byte << 8
        for _ in range(8):
            crc = (crc << 1) ^ 0x1021 if (crc & 0x8000) else crc << 1
            crc &= 0xFFFF
    return crc

def le16(b: bytes) -> int:
    return b[0] | (b[1] << 8)

def le32(b: bytes) -> int:
    return b[0] | (b[1] << 8) | (b[2] << 16) | (b[3] << 24)

def le_float(b: bytes) -> float:
    return struct.unpack('<f', bytes(b[:4]))[0]

def le_int32(b: bytes) -> int:
    return struct.unpack('<i', bytes(b[:4]))[0]

def pack_le16(v: int) -> bytes:
    return bytes([v & 0xFF, (v >> 8) & 0xFF])

def pack_le32(v: int) -> bytes:
    return bytes([v & 0xFF, (v >> 8) & 0xFF, (v >> 16) & 0xFF, (v >> 24) & 0xFF])

def pack_le_float(v: float) -> bytes:
    return struct.pack('<f', v)

def pack_le_int32(v: int) -> bytes:
    return struct.pack('<i', v)

def decode_aux_flags(flags: int) -> list:
    return [name for bit, name in AUX_FLAG_NAMES.items() if flags & bit]

def read_exact(ser: serial.Serial, n: int) -> Optional[bytes]:
    buf = b''
    deadline = time.monotonic() + 1.0
    while len(buf) < n:
        if time.monotonic() > deadline:
            return None
        chunk = ser.read(n - len(buf))
        if chunk:
            buf += chunk
    return buf

def find_sync(ser: serial.Serial) -> bool:
    """Scan the byte stream until SYNC_WORD 0xAA 0x55 is found."""
    state = 0
    deadline = time.monotonic() + 2.0
    while time.monotonic() < deadline:
        b = ser.read(1)
        if not b:
            continue
        v = b[0]
        if state == 0:
            if v == (SYNC_WORD & 0xFF):      # 0xAA
                state = 1
        else:
            if v == ((SYNC_WORD >> 8) & 0xFF):   # 0x55
                return True
            elif v == (SYNC_WORD & 0xFF):
                state = 1
            else:
                state = 0
    return False

# ============================================================
#  DATA CLASSES
# ============================================================
@dataclass
class DataTelemetry:
    encoder:   list   # [int32 x3]
    velocity:  list   # [float x3]
    current:   list   # [float x3]
    aux_flags: int
    flags:     list   = field(default_factory=list)

    def __post_init__(self):
        self.flags = decode_aux_flags(self.aux_flags)

@dataclass
class Heartbeat:
    system_mode:     int
    mode_name:       str
    heartbeat_count: int
    aux_flags:       int
    flags:           list = field(default_factory=list)

    def __post_init__(self):
        self.flags    = decode_aux_flags(self.aux_flags)
        self.mode_name = MODE_NAMES.get(self.system_mode, f"UNKNOWN({self.system_mode})")

@dataclass
class AckResponse:
    ack_seq: int
    status:  int

@dataclass
class NackResponse:
    nack_seq:    int
    status:      int
    status_name: str = ""

    def __post_init__(self):
        self.status_name = NACK_CODES.get(self.status, f"UNKNOWN(0x{self.status:02X})")

@dataclass
class FaultReport:
    aux_flags: int
    motor_id:  int
    flags:     list = field(default_factory=list)
    motors:    list = field(default_factory=list)

    def __post_init__(self):
        self.flags  = decode_aux_flags(self.aux_flags)
        self.motors = [i for i in range(NUM_MOTORS) if self.motor_id & (1 << i)]

@dataclass
class TimeSyncResponse:
    t0: int   # echo of Jetson's t0
    t1: int   # Teensy rx time
    t2: int   # Teensy tx time

@dataclass
class ParsedPacket:
    apid:       int
    seq_count:  int
    timestamp:  int   # Teensy millis()
    teensy_id:  int
    payload:    Any

# ============================================================
#  PACKET BUILDER
# ============================================================
class PacketBuilder:
    def __init__(self):
        self._seq_tx = 0

    def _make_packet_info(self, pkt_type: int, apid: int) -> int:
        return ((CCSDS_VERSION & 0x7) << 13) | \
               ((pkt_type & 0x1) << 12) | \
               ((CCSDS_SECHDR_FLAG & 0x1) << 11) | \
               (apid & 0x7FF)

    def _make_seq_ctrl(self) -> int:
        ctrl = ((CCSDS_SEQ_FLAG & 0x3) << 14) | (self._seq_tx & 0x3FFF)
        self._seq_tx = (self._seq_tx + 1) & 0x3FFF
        return ctrl

    def build(self, apid: int, pkt_type: int, payload: bytes) -> bytes:
        ts = int(time.monotonic() * 1000) & 0xFFFFFFFF

        primary_info = self._make_packet_info(pkt_type, apid)
        seq_ctrl     = self._make_seq_ctrl()
        data_length  = (SECONDARY_SIZE_B + len(payload) + CRC_SIZE_B) - 1

        primary   = pack_le16(primary_info) + pack_le16(seq_ctrl) + pack_le16(data_length)
        secondary = pack_le32(ts) + bytes([TEENSY_ID])

        crc_data  = primary + secondary + payload
        crc       = crc16_ccitt(crc_data)

        return pack_le16(SYNC_WORD) + primary + secondary + payload + pack_le16(crc)

    def last_seq(self) -> int:
        return (self._seq_tx - 1) & 0x3FFF

# ============================================================
#  PAYLOAD PARSERS  (TLM → Jetson)
# ============================================================
def parse_data_telemetry(payload: bytes) -> Optional[DataTelemetry]:
    if len(payload) < 37:
        return None
    i = 0
    enc  = [le_int32(payload[i+j*4:i+j*4+4]) for j in range(3)]; i += 12
    vel  = [le_float(payload[i+j*4:i+j*4+4]) for j in range(3)]; i += 12
    curr = [le_float(payload[i+j*4:i+j*4+4]) for j in range(3)]; i += 12
    aux  = payload[i]
    return DataTelemetry(encoder=enc, velocity=vel, current=curr, aux_flags=aux)

def parse_heartbeat(payload: bytes) -> Optional[Heartbeat]:
    if len(payload) < 3:
        return None
    return Heartbeat(system_mode=payload[0], heartbeat_count=payload[1],
                     aux_flags=payload[2])

def parse_ack(payload: bytes) -> Optional[AckResponse]:
    if len(payload) < 3:
        return None
    return AckResponse(ack_seq=le16(payload[0:2]), status=payload[2])

def parse_nack(payload: bytes) -> Optional[NackResponse]:
    if len(payload) < 3:
        return None
    return NackResponse(nack_seq=le16(payload[0:2]), status=payload[2])

def parse_fault_report(payload: bytes) -> Optional[FaultReport]:
    if len(payload) < 3:
        return None
    return FaultReport(aux_flags=payload[0], motor_id=le16(payload[1:3]))

def parse_time_sync_resp(payload: bytes) -> Optional[TimeSyncResponse]:
    if len(payload) < 12:
        return None
    return TimeSyncResponse(
        t0=le32(payload[0:4]),
        t1=le32(payload[4:8]),
        t2=le32(payload[8:12]),
    )

# ============================================================
#  PACKET PARSER  (reads one packet from serial)
# ============================================================
def parse_packet(ser: serial.Serial) -> Optional[ParsedPacket]:
    # --- SYNC ---
    if not find_sync(ser):
        return None

    # --- Primary header ---
    primary = read_exact(ser, PRIMARY_SIZE_B)
    if not primary:
        return None

    pkt_info   = le16(primary[0:2])
    seq_ctrl   = le16(primary[2:4])
    data_length = le16(primary[4:6]) + 1

    version      = (pkt_info >> 13) & 0x7
    pkt_type     = (pkt_info >> 12) & 0x1
    sec_hdr_flag = (pkt_info >> 11) & 0x1
    apid         =  pkt_info & 0x7FF
    seq_count    =  seq_ctrl & 0x3FFF

    # Req 3 — basic header sanity
    if version != CCSDS_VERSION or sec_hdr_flag != CCSDS_SECHDR_FLAG:
        log.warning(f"Bad header: version={version} sec_hdr={sec_hdr_flag}")
        return None
    if pkt_type != CCSDS_TYPE_TLM:
        log.warning(f"Expected TLM packet type, got {pkt_type}")
        return None

    # --- Secondary header ---
    secondary = read_exact(ser, SECONDARY_SIZE_B)
    if not secondary:
        return None

    ts        = le32(secondary[0:4])
    teensy_id = secondary[4]

    # --- Payload ---
    payload_size = data_length - SECONDARY_SIZE_B - CRC_SIZE_B
    if payload_size < 0 or payload_size > MAX_PACKET_SIZE:
        log.warning(f"Implausible payload size: {payload_size}")
        return None

    payload = read_exact(ser, payload_size)
    if payload is None:
        return None

    # --- CRC (Req 3) ---
    crc_raw = read_exact(ser, CRC_SIZE_B)
    if not crc_raw:
        return None

    crc_rx   = le16(crc_raw)
    crc_calc = crc16_ccitt(primary + secondary + payload)

    if crc_rx != crc_calc:
        log.warning(f"CRC mismatch: rx=0x{crc_rx:04X} calc=0x{crc_calc:04X}")
        return None

    # --- Dispatch payload parser ---
    parsed_payload = None
    if apid == APID_DATA_TLM:
        parsed_payload = parse_data_telemetry(payload)
        if parsed_payload:
            # Req 4 — range check received sensor data
            for i in range(NUM_MOTORS):
                if abs(parsed_payload.velocity[i]) > VEL_MAX_VALID:
                    log.warning(f"Velocity OOR motor {i}: {parsed_payload.velocity[i]}")
                if abs(parsed_payload.current[i]) > CURR_MAX_VALID:
                    log.warning(f"Current OOR motor {i}: {parsed_payload.current[i]}")
    elif apid == APID_HEARTBEAT:
        parsed_payload = parse_heartbeat(payload)
    elif apid == APID_ACK:
        parsed_payload = parse_ack(payload)
    elif apid == APID_NACK:
        parsed_payload = parse_nack(payload)
    elif apid == APID_FAULT:
        parsed_payload = parse_fault_report(payload)
    elif apid == APID_TIME_SYNC:
        parsed_payload = parse_time_sync_resp(payload)
    else:
        log.warning(f"Unknown APID: 0x{apid:03X}")

    return ParsedPacket(apid=apid, seq_count=seq_count, timestamp=ts,
                        teensy_id=teensy_id, payload=parsed_payload)

# ============================================================
#  JETSON CCSDS CONTROLLER
# ============================================================
class JetsonController:
    """
    Thread-safe controller that:
      - Listens for incoming TLM in a background thread
      - Sends TC packets with retransmit logic (Req 2)
      - Monitors heartbeat / telemetry timeouts (Req 1, 17)
      - Validates all outgoing commands (Req 21)
    """

    def __init__(self, port: str = '/dev/ttyUSB0', baud: int = 115_200):
        self._ser     = serial.Serial(port, baud, timeout=0.1)
        self._builder = PacketBuilder()
        self._lock    = threading.Lock()

        # Watchdog timestamps
        self._last_hb_time   = time.monotonic()
        self._last_tlm_time  = time.monotonic()

        # Latest received state
        self.latest_telemetry: Optional[DataTelemetry] = None
        self.latest_heartbeat: Optional[Heartbeat]     = None
        self.system_mode: int = 0

        # ACK/NACK tracking (Req 2)
        self._pending_ack_seq:  Optional[int] = None
        self._pending_ack_event = threading.Event()
        self._last_ack:  Optional[AckResponse]  = None
        self._last_nack: Optional[NackResponse] = None

        # Optional callbacks
        self.on_telemetry: Optional[Callable[[DataTelemetry], None]] = None
        self.on_heartbeat: Optional[Callable[[Heartbeat], None]]     = None
        self.on_fault:     Optional[Callable[[FaultReport], None]]   = None

        # Time-sync
        self._time_offset_ms: Optional[float] = None

        # Background receiver
        self._running = True
        self._rx_thread = threading.Thread(target=self._rx_loop, daemon=True)
        self._rx_thread.start()

        # Watchdog thread (Req 1, 17)
        self._wd_thread = threading.Thread(target=self._watchdog_loop, daemon=True)
        self._wd_thread.start()

    # ----------------------------------------------------------
    #  RECEIVE LOOP
    # ----------------------------------------------------------
    def _rx_loop(self):
        while self._running:
            try:
                pkt = parse_packet(self._ser)
                if pkt is None:
                    continue
                self._dispatch(pkt)
            except serial.SerialException as e:
                log.error(f"Serial error: {e}")
                time.sleep(0.1)

    def _dispatch(self, pkt: ParsedPacket):
        apid    = pkt.apid
        payload = pkt.payload

        if apid == APID_DATA_TLM and isinstance(payload, DataTelemetry):
            self.latest_telemetry = payload
            self._last_tlm_time   = time.monotonic()
            if self.on_telemetry:
                self.on_telemetry(payload)

        elif apid == APID_HEARTBEAT and isinstance(payload, Heartbeat):
            self.latest_heartbeat = payload
            self.system_mode      = payload.system_mode
            self._last_hb_time    = time.monotonic()
            if self.on_heartbeat:
                self.on_heartbeat(payload)

        elif apid == APID_ACK and isinstance(payload, AckResponse):
            self._last_ack = payload
            log.info(f"ACK seq={payload.ack_seq}")
            if self._pending_ack_seq == payload.ack_seq:
                self._pending_ack_event.set()

        elif apid == APID_NACK and isinstance(payload, NackResponse):
            self._last_nack = payload
            log.warning(f"NACK seq={payload.nack_seq} reason={payload.status_name}")
            if self._pending_ack_seq == payload.nack_seq:
                self._pending_ack_event.set()

        elif apid == APID_FAULT and isinstance(payload, FaultReport):
            log.error(f"Fault report: flags={payload.flags} motors={payload.motors}")
            if self.on_fault:
                self.on_fault(payload)

        elif apid == APID_TIME_SYNC and isinstance(payload, TimeSyncResponse):
            self._handle_time_sync_resp(payload)

    # ----------------------------------------------------------
    #  WATCHDOG  (Req 1, 17)
    # ----------------------------------------------------------
    def _watchdog_loop(self):
        while self._running:
            now = time.monotonic()
            if (now - self._last_hb_time) > HEARTBEAT_TIMEOUT_S:
                log.error("WATCHDOG: No heartbeat — comms loss, Teensy may ESTOP")
            if (now - self._last_tlm_time) > TELEMETRY_TIMEOUT_S:
                log.warning("WATCHDOG: No telemetry")
            time.sleep(0.5)

    # ----------------------------------------------------------
    #  SEND WITH ACK/RETRANSMIT  (Req 2)
    # ----------------------------------------------------------
    def _send_with_ack(self, apid: int, payload: bytes) -> bool:
        """Transmit a TC packet and wait for ACK; retransmit on timeout (Req 2)."""
        for attempt in range(1, MAX_RETRANSMIT + 1):
            with self._lock:
                pkt = self._builder.build(apid, CCSDS_TYPE_TC, payload)
                seq = self._builder.last_seq()
                self._pending_ack_seq   = seq
                self._pending_ack_event.clear()
                self._last_ack  = None
                self._last_nack = None
                self._ser.write(pkt)

            log.debug(f"TX apid=0x{apid:03X} seq={seq} attempt={attempt}")

            if self._pending_ack_event.wait(timeout=RETRANSMIT_TIMEOUT_S):
                with self._lock:
                    if self._last_ack and self._last_ack.ack_seq == seq:
                        return True
                    if self._last_nack:
                        log.warning(f"NACK on attempt {attempt}: {self._last_nack.status_name}")
                        return False  # NACKs are definitive — no point retransmitting
            else:
                log.warning(f"ACK timeout for seq={seq}, attempt {attempt}/{MAX_RETRANSMIT}")

        log.error(f"Max retransmits reached for apid=0x{apid:03X}")
        return False

    # ----------------------------------------------------------
    #  PUBLIC API — MOTOR COMMANDS
    # ----------------------------------------------------------
    def send_velocity_cmd(self, vel: list, servo_pos: list) -> bool:
        """
        Send APID 0x010 velocity + servo command.
        Req 21 — full bounds check before transmitting.
        vel:       list of 3 floats (rpm, clamped to ±50)
        servo_pos: list of 3 ints  (degrees, 0–180)
        """
        if len(vel) != NUM_MOTORS or len(servo_pos) != NUM_MOTORS:
            raise ValueError(f"Expected {NUM_MOTORS} motors")

        # Req 21 — bounds check
        for i, v in enumerate(vel):
            if not (VEL_CMD_MIN <= v <= VEL_CMD_MAX):
                raise ValueError(f"vel[{i}]={v} out of range [{VEL_CMD_MIN}, {VEL_CMD_MAX}]")
        for i, s in enumerate(servo_pos):
            if not (SERVO_CMD_MIN <= s <= SERVO_CMD_MAX):
                raise ValueError(f"servo_pos[{i}]={s} out of range [{SERVO_CMD_MIN}, {SERVO_CMD_MAX}]")

        payload = b''
        for v in vel:
            payload += pack_le_float(float(v))
        for s in servo_pos:
            payload += pack_le_int32(int(s))

        return self._send_with_ack(APID_VEL_SERVO, payload)

    def send_system_command(self, cmd_id: int) -> bool:
        """Send APID 0x011 system command."""
        valid = {CMD_STOP_ALL_MOTORS, CMD_ENABLE_MOTORS, CMD_DISABLE_MOTORS,
                 CMD_RESET_ENCODERS, CMD_ESTOP, CMD_RELEASE_ESTOP, CMD_REBOOT_TEENSY}
        if cmd_id not in valid:
            raise ValueError(f"Unknown system command 0x{cmd_id:02X}")
        payload = bytes([cmd_id])
        return self._send_with_ack(APID_SYS_CMD, payload)

    # Convenience wrappers
    def estop(self)          -> bool: return self.send_system_command(CMD_ESTOP)
    def release_estop(self)  -> bool: return self.send_system_command(CMD_RELEASE_ESTOP)
    def enable_motors(self)  -> bool: return self.send_system_command(CMD_ENABLE_MOTORS)
    def disable_motors(self) -> bool: return self.send_system_command(CMD_DISABLE_MOTORS)
    def stop_all(self)       -> bool: return self.send_system_command(CMD_STOP_ALL_MOTORS)
    def reset_encoders(self) -> bool: return self.send_system_command(CMD_RESET_ENCODERS)
    def reboot_teensy(self)  -> bool: return self.send_system_command(CMD_REBOOT_TEENSY)

    # ----------------------------------------------------------
    #  TIME SYNC  (APID 0x020)
    # ----------------------------------------------------------
    def send_time_sync_request(self):
        """Send a time-sync request; Teensy echoes t0, adds t1/t2."""
        t0 = int(time.monotonic() * 1000) & 0xFFFFFFFF
        payload = pack_le32(t0)
        with self._lock:
            pkt = self._builder.build(APID_TIME_SYNC, CCSDS_TYPE_TC, payload)
            self._ser.write(pkt)
        log.info(f"Time sync request sent, t0={t0}")

    def _handle_time_sync_resp(self, resp: TimeSyncResponse):
        t3 = int(time.monotonic() * 1000) & 0xFFFFFFFF
        # NTP-style offset: ((t1-t0) + (t2-t3)) / 2
        offset = ((resp.t1 - resp.t0) + (resp.t2 - t3)) / 2
        self._time_offset_ms = offset
        log.info(f"Time sync: offset={offset:.1f}ms t0={resp.t0} t1={resp.t1} t2={resp.t2} t3={t3}")

    # ----------------------------------------------------------
    #  DIAGNOSTICS
    # ----------------------------------------------------------
    def get_status(self) -> Dict[str, Any]:
        hb  = self.latest_heartbeat
        tlm = self.latest_telemetry
        return {
            "system_mode":  MODE_NAMES.get(self.system_mode, "UNKNOWN"),
            "heartbeat": {
                "count":    hb.heartbeat_count if hb else None,
                "flags":    hb.flags           if hb else [],
            },
            "telemetry": {
                "encoder":  tlm.encoder   if tlm else None,
                "velocity": tlm.velocity  if tlm else None,
                "current":  tlm.current   if tlm else None,
                "flags":    tlm.flags     if tlm else [],
            },
            "hb_age_s":   round(time.monotonic() - self._last_hb_time, 2),
            "tlm_age_s":  round(time.monotonic() - self._last_tlm_time, 2),
            "time_offset_ms": self._time_offset_ms,
        }

    def close(self):
        self._running = False
        self._ser.close()


# ============================================================
#  EXAMPLE USAGE
# ============================================================
if __name__ == "__main__":
    ctrl = JetsonController(port='/dev/ttyUSB0', baud=115_200)

    # Register callbacks
    def on_tlm(t: DataTelemetry):
        log.info(f"TLM enc={t.encoder} vel={[f'{v:.2f}' for v in t.velocity]} "
                 f"curr={[f'{c:.2f}' for c in t.current]} flags={t.flags}")

    def on_hb(h: Heartbeat):
        log.info(f"HB mode={h.mode_name} count={h.heartbeat_count} flags={h.flags}")

    def on_fault(f: FaultReport):
        log.error(f"FAULT flags={f.flags} motors={f.motors}")

    ctrl.on_telemetry = on_tlm
    ctrl.on_heartbeat = on_hb
    ctrl.on_fault     = on_fault

    try:
        time.sleep(2)  # Wait for INIT → IDLE on Teensy

        ctrl.send_time_sync_request()
        time.sleep(0.2)

        log.info("Enabling motors...")
        ctrl.enable_motors()
        time.sleep(0.5)

        log.info("Sending velocity command...")
        ctrl.send_velocity_cmd(vel=[10.0, 10.0, 10.0], servo_pos=[90, 90, 90])
        time.sleep(3)

        log.info("Stopping motors...")
        ctrl.stop_all()
        time.sleep(0.5)

        print("\n--- Status ---")
        import json
        print(json.dumps(ctrl.get_status(), indent=2))

    except KeyboardInterrupt:
        log.info("Interrupted")
    finally:
        ctrl.estop()
        ctrl.close()