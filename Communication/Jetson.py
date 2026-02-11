import serial
import struct

SYNC_WORD        = 0xAA55
SYNC_SIZE_B      = 2
PRIMARY_SIZE_B   = 6
SECONDARY_SIZE_B = 5
CRC_SIZE_B       = 2

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


def le16(b):
    temp = b[0]
    temp |= (b[1] << 8)
    return temp

def le32(b):
    temp = b[0]
    temp |= (b[1] << 8)
    temp |= (b[2] << 16)
    temp |= (b[3] << 24)
    return temp

def le_float(b):
    return struct.unpack('<f', bytes(b))[0]

def le_int32(b):
    return struct.unpack('<i', bytes(b))[0]


def read_exact(ser, n):
    buf = b''
    while len(buf) < n:
        chunk = ser.read(n - len(buf))
        if not chunk:
            return None
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

            

def parse_packet(ser):
    # SYNC
    if not find_sync(ser):
        return None
    
    # Primary
    primary = read_exact(ser, PRIMARY_SIZE_B)
    if not primary:
        return None
    
    packet_info = le16(primary[0:2])
    seq_ctrl = le16(primary[2:4])
    data_length = le16(primary[4:6]) + 1

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
    
    ts = le32(secondary[0:4])
    teensy_id = secondary[4]

    # Payload
    payload_size = data_length - SECONDARY_SIZE_B - CRC_SIZE_B
    payload = read_exact(ser, payload_size)
    if not payload:
        return None
    
    if apid == 0x001:
        payload_parsed = parse_telemetry_payload(payload)
    else:
        return {}
    
    # CRC
    crc_rx = read_exact(ser, CRC_SIZE_B)
    if not crc_rx:
        return None
    crc_rx = le16(crc_rx)

    crc_calc = crc16_ccitt(primary + secondary + payload)

    if crc_rx != crc_calc:
        print("CRC Error")
        return None
    
    # print(payload_parsed)
    
    return {"packet_type": packet_type, "apid": apid, "timestamp": ts, "teensy_id": teensy_id,
            "seq_count": seq_count, "payload": payload_parsed, "primary": primary,
            "secondary": secondary, "crc_ok": True}


def parse_telemetry_payload(payload):
    encoder = [0, 0, 0]
    curr = [0, 0, 0]
    vel = [0, 0, 0]

    i = 0
    encoder[0] = le_int32(payload[i:i+4]); i+=4;
    encoder[1] = le_int32(payload[i:i+4]); i+=4;
    encoder[2] = le_int32(payload[i:i+4]); i+=4;

    vel[0] = le_float(payload[i:i+4]); i+=4;
    vel[1] = le_float(payload[i:i+4]); i+=4;
    vel[2] = le_float(payload[i:i+4]); i+=4;

    curr[0] = le_float(payload[i:i+4]); i+=4;
    curr[1] = le_float(payload[i:i+4]); i+=4;
    curr[2] = le_float(payload[i:i+4]); i+=4;

    aux_flags = payload[i]; i+=1;

    return {"encoder": encoder, "velocity": vel, "current": curr, "aux_flags": aux_flags}


ser = serial.Serial('/dev/ttyUSB0', 115200)
while True:
    packet = parse_packet(ser)
    if packet is None:
        continue

    # print(packet["payload"])
