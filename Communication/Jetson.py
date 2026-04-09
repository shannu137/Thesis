import serial
import struct
import time
from dataclasses import dataclass

# ============================================================
#  PROTOCOL CONSTANTS  (must match Teensy)
# ============================================================
SYNC_WORD           = 0xAA55
CCSDS_VERSION       = 0
CCSDS_TYPE_TLM      = 0
CCSDS_TYPE_TC       = 1
CCSDS_SECHDR_FLAG   = 1
CCSDS_SEQ_FLAG      = 3

SYNC_SIZE_B         = 2
PRIMARY_SIZE_B      = 6
SECONDARY_SIZE_B    = 5
CRC_SIZE_B          = 2
MAX_PACKET_SIZE     = 512

# ============================================================
#  APIDs
# ============================================================
APID_DATA_TELEMETRY   =  0x001
APID_HEARTBEAT        =  0x002
APID_ACK_RESPONSE     =  0x003
APID_NACK_RESPONSE    =  0x004
APID_FAULT_REPORT     =  0x005

APID_TELECOMMAND      =  0x010
APID_SYSTEM_COMMANDS  =  0x011

APID_TIME_SYNC        =  0x020

# ============================================================
#  AUX FLAGS
# ============================================================
AUX_FLAGS = {
    0x01: "CPU_OVERLOAD",
    0x02: "ENCODER_FAULT",
    0x04: "OVERCURRENT",
    0x08: "STALL_DETECTED",
    0x10: "CURRSENSOR_FAULT",
    0x20: "DRIVER_FAULT",
    0x40: "SENSOR_OUT_OF_RANGE",
    0x80: "ESTOP_ACTIVE",
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
}

# ============================================================
#  COMMAND CODES FOR APID 0x011
# ============================================================
CMD_STOP_ALL_MOTORS    =  0x01
CMD_ENABLE_MOTORS      =  0x02
CMD_DISABLE_MOTORS     =  0x03
CMD_RESET_ENCODERS     =  0x04
CMD_ESTOP              =  0x05
CMD_RELEASE_ESTOP      =  0x06
CMD_REBOOT_TEENSY      =  0x08

# ============================================================
#  SYSTEM MODES
# ============================================================
MODE_NAMES = {
    0: "INIT", 1: "IDLE", 2: "NOMINAL",
    3: "DEGRADED", 4: "INHIBIT", 5: "SAFE", 6: "ESTOP",
}

# ============================================================
#  COMMAND BOUNDS 
# ============================================================
VEL_CMD_MAX        =   30.0
VEL_CMD_MIN        =  -30.0
POS_CMD_MAX        =   1000000
POS_CMD_MIN        =  -1000000
SERVO_CMD_MAX      =   180
SERVO_CMD_MIN      =   0

MAX_CURRENT_VALID  = 1.2 * 3500.0
MAX_VELOCITY_VALID = 1.5 * VEL_CMD_MAX

# ============================================================
#  TIMING 
# ============================================================
TELEMETRY_TIMEOUT_S   = 0.5    # Req 17 — no TLM → warn
RETRANSMIT_TIMEOUT_S  = 0.2    # Req  2 — wait for ACK/NACK
MAX_RETRANSMIT        = 3      # Req  2

@dataclass
class DataTelemetry:
    encoder:   list
    velocity:  list
    current:   list
    aux_flags: list

# ============================================================
#  HELPERS
# ============================================================
def crc16_ccitt(data):
    crc = 0xFFFF
    for byte in data:
        crc ^= byte << 8
        for _ in range(8):
            if crc & 0x8000:
                crc = (crc << 1) ^ 0x1021
            else:
                crc <<= 1
            crc &= 0xFFFF
    return crc

def from_le16(b):
    temp = b[0]
    temp |= (b[1] << 8)
    return temp

def from_le32(b):
    temp = b[0]
    temp |= (b[1] << 8)
    temp |= (b[2] << 16)
    temp |= (b[3] << 24)
    return temp

def from_le_float(b):
    return struct.unpack('<f', bytes(b))[0]

def from_le_int32(b):
    return struct.unpack('<i', bytes(b))[0]

def to_le16(value):
    return bytes([value & 0xFF, (value >> 8) & 0xFF])

def to_le32(value):
    return bytes([value & 0xFF, (value >> 8) & 0xFF, (value >> 16) & 0xFF, (value >> 24) & 0xFF])

def to_le_float(value):
    return struct.pack('<f', value)

def to_le_int32(value):
    return struct.pack('<i', value)

def decode_aux_flags(flags):
    return [name for bit, name in AUX_FLAGS.items() if flags & bit]

def read_exact(ser, n):
    buf = b''
    while len(buf) < n:
        chunk = ser.read(n - len(buf))
        if chunk:
            buf += chunk
    return buf
            
def find_sync(ser):
    state = 0
    while True:
        b = ser.read(1)
        if not b:
            return None
        v = b[0]

        if state == 0:
            if v == (SYNC_WORD & 0xFF):
                state = 1
        else: 
            if v == ((SYNC_WORD >> 8) & 0xFF):
                return True
            elif v == (SYNC_WORD & 0xFF):
                state = 1
            else:
                state = 0
    return False


        
def parse_data_telemetry(payload):
    if len(payload) < 37:
        return None
    enc = [0, 0, 0]
    curr    = [0, 0, 0]
    vel     = [0, 0, 0]

    i = 0
    enc[0] = from_le_int32(payload[i:i+4]); i+=4;
    enc[1] = from_le_int32(payload[i:i+4]); i+=4;
    enc[2] = from_le_int32(payload[i:i+4]); i+=4;

    vel[0] = from_le_float(payload[i:i+4]); i+=4;
    vel[1] = from_le_float(payload[i:i+4]); i+=4;
    vel[2] = from_le_float(payload[i:i+4]); i+=4;

    curr[0] = from_le_float(payload[i:i+4]); i+=4;
    curr[1] = from_le_float(payload[i:i+4]); i+=4;
    curr[2] = from_le_float(payload[i:i+4]); i+=4;

    aux_flags = payload[i]; i+=1;
    aux = decode_aux_flags(aux_flags)

    return DataTelemetry(encoder=enc, velocity=vel, current=curr, aux_flags=aux)

def parse_heartbeat(payload):
    if len(payload) < 3:
        return None
    return {'system_mode': payload[0], 'hb_count': payload[1],
            'aux_flags': decode_aux_flags(payload[2])}

def parse_ack(payload):
    if len(payload) < 3:
        return None
    return {'ack_seq': from_le16(payload[0:2]), 'status': payload[2]}

def parse_nack(payload):
    if len(payload) < 3:
        return None
    return {'nack_seq': from_le16(payload[0:2]), 'status': payload[2]}

def parse_fault_report(payload):
    if len(payload) < 3:
        return None
    return {'aux_flags': decode_aux_flags(payload[0]), 'motor_id': from_le16(payload[1:3])}

def parse_time_sync_response(payload):
    if len(payload) < 12:
        return None
    return {'t0': from_le32(payload[0:4]), 't1': from_le32(payload[4:8]), 't2': from_le32(payload[8:12])}

def parse_packet(ser):
    # SYNC
    if not find_sync(ser):
        return None
    
    # Primary
    primary = read_exact(ser, PRIMARY_SIZE_B)
    if not primary:
        return None
    
    packet_info = from_le16(primary[0:2])
    seq_ctrl = from_le16(primary[2:4])
    data_length = from_le16(primary[4:6]) + 1

    version = (packet_info >> 13) & 0x7
    packet_type = (packet_info >> 12) & 0x1
    secHdrFlag = (packet_info >> 11) & 0x1
    apid = (packet_info) & 0x7FF
    print(hex(apid))

    seq_flags = (seq_ctrl >> 14) & 0x3
    seq_count = (seq_ctrl) & 0x3FFF

    # Secondary
    secondary = read_exact(ser, SECONDARY_SIZE_B)
    if not secondary:
        return None
    
    ts = from_le32(secondary[0:4])
    teensy_id = secondary[4]

    # Payload
    payload_size = data_length - SECONDARY_SIZE_B - CRC_SIZE_B
    payload = read_exact(ser, payload_size)
    if not payload:
        return None
    
    if apid == APID_DATA_TELEMETRY:
        payload_parsed = parse_data_telemetry(payload)
    elif apid == APID_HEARTBEAT:
        payload_parsed = parse_heartbeat(payload)
    elif apid == APID_ACK_RESPONSE:
        payload_parsed = parse_ack(payload)
    elif apid == APID_NACK_RESPONSE:
        payload_parsed = parse_nack(payload)
    elif apid == APID_FAULT_REPORT:
        payload_parsed = parse_fault_report(payload)
    elif apid == APID_TIME_SYNC:
        payload_parsed = parse_time_sync_response(payload)
    else:
        return None

    
    # CRC
    crc_rx = read_exact(ser, CRC_SIZE_B)
    if not crc_rx:
        return None
    crc_rx = from_le16(crc_rx)

    crc_calc = crc16_ccitt(primary + secondary + payload)

    if crc_rx != crc_calc:
        print("CRC Error")
        return None
    
    # print(payload_parsed)
    
    return {"packet_type": packet_type, "apid": apid, "timestamp": ts, "teensy_id": teensy_id,
            "seq_count": seq_count, "payload": payload_parsed, "primary": primary,
            "secondary": secondary, "crc_ok": True}



ser = serial.Serial('/dev/ttyUSB0', 115200)
while True:
    packet = parse_packet(ser)
    if packet is None:
        continue

    # print(packet["payload"])
