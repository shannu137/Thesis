# ACK and Parallelization for 2 teensys

import serial
import struct
import time
from dataclasses import dataclass
import sys
import termios
import tty
import math
import threading

# ============================================================
#  Constants for Keyboard Ops
# ============================================================
LATERAL_ROVER_LEN      = 0.2564    # m
LONGITUDINAL_FRONT_LEN = 0.3957    # m
LONGITUDINAL_REAR_LEN  = 0.3653    # m

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

NUM_MOTORS          = 3
LEFT_TEENSY         = 0
RIGHT_TEENSY        = 1

cmd_lock = threading.Lock()

shared_cmd = {
    "left":  ([0,0,0], [90,90,90]),
    "right": ([0,0,0], [90,90,90])
}

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
#  GLOBAL VARIABLES
# ============================================================
seq_tx        = 0
lin_speed_x   = 0.1         # m/s
lin_speed_y   = 0.1         # m/s
angular_speed = 0.3         # rad/s

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


# ============================================================
#  CCSDS HEADER BUILDERS
# ============================================================
def make_packet_info(pkt_type, apid):
  pkt  = ((CCSDS_VERSION     & 0x7) << 13)
  pkt |= ((pkt_type          & 0x1) << 12)
  pkt |= ((CCSDS_SECHDR_FLAG & 0x1) << 11)
  pkt |= (apid & 0x7FF)

  return pkt

def make_seq_ctrl():
  global seq_tx
  ctrl = ((CCSDS_SEQ_FLAG & 0x3) << 14) | (seq_tx & 0x3FFF)
  seq_tx = seq_tx + 1
  return ctrl

# ============================================================
#  PAYLOAD BUILDER
# ============================================================
def send_vel_servo_cmd(ser, vel, servo_pos, teensy_id):
    if len(vel) != NUM_MOTORS or len(servo_pos) != NUM_MOTORS:
        print(f"Expected {NUM_MOTORS} motors")
        return None
    
    payload = b''

    for i, v in enumerate(vel):
        if not (VEL_CMD_MIN <= v <= VEL_CMD_MAX):
            print("Velocity out of range")
            return None
        payload += to_le_float(v)
        
    for i, s in enumerate(servo_pos):
        if not (SERVO_CMD_MIN <= s <= SERVO_CMD_MAX):
            print("Servo position out of range")
            return None
        payload += to_le_int32(s)

    send_packet(ser, APID_TELECOMMAND, CCSDS_TYPE_TC, payload, teensy_id)

def send_system_cmd(ser, cmd_id, teensy_id):
    valid = {CMD_STOP_ALL_MOTORS, CMD_ENABLE_MOTORS, CMD_DISABLE_MOTORS, CMD_RESET_ENCODERS, 
             CMD_ESTOP, CMD_RELEASE_ESTOP, CMD_REBOOT_TEENSY}
    if cmd_id not in valid:
        print("Command ID not valid")
        return None
    payload = bytes([cmd_id])

    send_packet(ser, APID_SYSTEM_COMMANDS, CCSDS_TYPE_TC, payload, teensy_id)

def send_time_sync_request(ser, teensy_id):
    t0 = int(time.monotonic_ns * 1e6) & 0xFFFFFFFF
    payload = to_le32(t0)
    send_packet(ser, APID_TIME_SYNC, CCSDS_TYPE_TC, payload, teensy_id)

# ============================================================
#  PACKET BUILDER
# ============================================================
def send_packet(ser, apid, pkt_type, payload, teensy_id):
    ts = int(time.monotonic_ns() * 1e6) & 0xFFFFFFFF


    primary_info = make_packet_info(pkt_type, apid)
    seq_ctrl     = make_seq_ctrl()
    data_length  = (SECONDARY_SIZE_B + len(payload) + CRC_SIZE_B) - 1

    primary      = to_le16(primary_info) + to_le16(seq_ctrl) + to_le16(data_length)
    secondary    = to_le32(ts) + bytes([teensy_id])

    crc_data     = primary + secondary + payload
    crc          = crc16_ccitt(crc_data)

    pkt = to_le16(SYNC_WORD) + primary + secondary + payload + to_le16(crc)
    ser.write(pkt)
    ser.flush()

# ============================================================
#  HELPERS
# ============================================================
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

# ============================================================
#  PAYLOAD PARSER
# ============================================================
def parse_data_telemetry(payload):
    if len(payload) < 37:
        return None
    enc     = [0, 0, 0]
    curr    = [0, 0, 0]
    vel     = [0, 0, 0]

    for i in range(NUM_MOTORS):
        enc[i]  = from_le_int32(payload[4*i:4*(i+1)])
        vel[i]  = from_le_float(payload[4*(i+NUM_MOTORS):4*(i+NUM_MOTORS+1)])
        if abs(vel[i]) > MAX_VELOCITY_VALID:
            print(f"Velocity out of range for motor {i}")
        curr[i] = from_le_float(payload[4*(i+2*NUM_MOTORS):4*(i+2*NUM_MOTORS+1)])
        if abs(curr[i]) > MAX_CURRENT_VALID:
            print(f"Current out of range for motor {i}")

    aux_flags = payload[12*NUM_MOTORS]
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

# ============================================================
#  PACKET PARSER
# ============================================================
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

    if version != CCSDS_VERSION or secHdrFlag != CCSDS_SECHDR_FLAG:
        print(f"Bad header: version={version} sec_hdr={secHdrFlag}")
        return None
    if packet_type != CCSDS_TYPE_TLM:
        print(f"Expected TLM packet type, got {packet_type}")
        return None

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
       
    # CRC
    crc_rx = read_exact(ser, CRC_SIZE_B)
    if not crc_rx:
        return None
    crc_rx = from_le16(crc_rx)

    crc_calc = crc16_ccitt(primary + secondary + payload)

    if crc_rx != crc_calc:
        print("CRC Error")
        return None
    
    payload_parsed = None
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
        print("Unknown APID")
        return None
    
    return {"apid": apid, "seq_count": seq_count, "timestamp": ts,
            "teensy_id": teensy_id, "payload": payload_parsed}

# ============================================================
#  Keyboard Ops
# ============================================================
def get_key():
    """Read a single character from keyboard (non-blocking)."""
    fd = sys.stdin.fileno()
    old_settings = termios.tcgetattr(fd)
    try:
        tty.setraw(fd)
        key = sys.stdin.read(1)  # read one character
    finally:
        termios.tcsetattr(fd, termios.TCSADRAIN, old_settings)
    return key

def commands():
    key = get_key()
    
    global lin_speed_x
    global lin_speed_y
    global angular_speed

    if key == 's':
        factors = [0.0, 0.0, 0.0]
    elif key == 'w':
        factors = [1.0, 0.0, 0.0]
    elif key == 'a':
        factors = [0.0, 1.0, 0.0]
    elif key == 'd':
        factors = [0.0, -1.0, 0.0]
    elif key == 'x':
        factors = [-1.0, 0.0, 0.0]
    elif key == 'q':
        factors = [1.0, 1.0, 0.0]
    elif key == 'e':
        factors = [1.0, -1.0, 0.0]
    elif key == 'z':
        factors = [-1.0, 1.0, 0.0]
    elif key == 'c':
        factors = [-1.0, -1.0, 0.0]

    elif key == 'm':
        lin_speed_x += 0.01
        print("Increased linear speed x to ", lin_speed_x)
    elif key == ',':
        lin_speed_x -= 0.01
        print("Decreased linear speed x to ", lin_speed_x)
    elif key == 'j':
        lin_speed_y += 0.01
        print("Increased linear speed y to ", lin_speed_y)
    elif key == 'k':
        lin_speed_y -= 0.01
        print("Decreased linear speed y to ", lin_speed_y)
    elif key == 'u':
        angular_speed += 0.01
        print("Increased angular speed z to ", angular_speed)
    elif key == 'i':
        angular_speed -= 0.01
        print("Decreased angular speed z to ", angular_speed)
    
    elif key == 'Q':
        print('indies Q')
        factors = [1.0, 1.0, 1.0]
    elif key == 'E':
        factors = [1.0, -1.0, -1.0]
    elif key == 'Z':
        factors = [-1.0, 1.0, 1.0]
    elif key == 'C':
        factors = [-1.0, -1.0, -1.0]
    elif key == 'r':
        factors = [1.0, 1.0, -1.0]
    elif key == 'y':
        factors = [1.0, -1.0, 1.0]
    elif key == 'v':
        factors = [-1.0, 1.0, -1.0]
    elif key == 'n':
        factors = [-1.0, -1.0, 1.0]
    elif key == 'f':
        factors = [0.0, 0.0, 1.0]            
    elif key == 'h':
        factors = [0.0, 0.0, -1.0]

    elif key == 'p':
        exit()

    if key in ['w', 'a', 's', 'd', 'x', 'q', 'e', 'z', 'c', 'Q', 'E', 'Z', 'C', 'r', 'v', 'y', 'n', 'f', 'h']:
        actual = [lin_speed_x, lin_speed_y, angular_speed]
        command = [x*y for x,y in zip(factors,actual)]
        actuation_commands(command)
        # data = ser_left.readline()
        # print("Arduino says:", data)

def actuation_commands(command):

    vx, vy, omega = command

    # Steer angles
    # angle1
    gamma1 = math.atan2((vy + (math.sqrt((math.pow(LATERAL_ROVER_LEN,2)+math.pow(LONGITUDINAL_FRONT_LEN,2)))*omega*math.cos(math.atan2(LATERAL_ROVER_LEN,LONGITUDINAL_FRONT_LEN)))), (vx - ((math.pow(LATERAL_ROVER_LEN,2)+math.pow(LONGITUDINAL_FRONT_LEN,2))*omega*math.sin(math.atan2(LATERAL_ROVER_LEN,LONGITUDINAL_FRONT_LEN)))))

    # angle2
    gamma2 = math.atan2((vy + (math.sqrt((math.pow(LATERAL_ROVER_LEN,2)+math.pow(LONGITUDINAL_FRONT_LEN,2)))*omega*math.cos(math.atan2(LATERAL_ROVER_LEN,LONGITUDINAL_FRONT_LEN)))), (vx + ((math.pow(LATERAL_ROVER_LEN,2)+math.pow(LONGITUDINAL_FRONT_LEN,2))*omega*math.sin(math.atan2(LATERAL_ROVER_LEN,LONGITUDINAL_FRONT_LEN)))))

    # angle3
    gamma3 = math.atan2(vy, (vx - ((LATERAL_ROVER_LEN)*omega)))

    # angle4
    gamma4 = math.atan2(vy, (vx + ((LATERAL_ROVER_LEN)*omega)))

    # angle5    
    gamma5 = math.atan2((vy - (math.sqrt((math.pow(LATERAL_ROVER_LEN,2)+math.pow(LONGITUDINAL_REAR_LEN,2)))*omega*math.cos(math.atan2(LATERAL_ROVER_LEN,LONGITUDINAL_REAR_LEN)))), (vx - ((math.pow(LATERAL_ROVER_LEN,2)+math.pow(LONGITUDINAL_REAR_LEN,2))*omega*math.sin(math.atan2(LATERAL_ROVER_LEN,LONGITUDINAL_REAR_LEN)))))

    # angle6
    gamma6 = math.atan2((vy - (math.sqrt((math.pow(LATERAL_ROVER_LEN,2)+math.pow(LONGITUDINAL_REAR_LEN,2)))*omega*math.cos(math.atan2(LATERAL_ROVER_LEN,LONGITUDINAL_REAR_LEN)))), (vx + ((math.pow(LATERAL_ROVER_LEN,2)+math.pow(LONGITUDINAL_REAR_LEN,2))*omega*math.sin(math.atan2(LATERAL_ROVER_LEN,LONGITUDINAL_REAR_LEN)))))
    print("before addinf n50", gamma1, gamma2, gamma3, gamma4, gamma5, gamma6)
    
    # The math assumes the range of steering is from -90 to +90 but the servo operates in 0 to 180  and the conversion happens below (90-theta)
    gammas = [gamma1, gamma2, gamma3, gamma4, gamma5, gamma6]
    gamma = [int(90-math.degrees(g)) for g in gammas]
    
    # Wheel velocities
    
    # w1
    w1 =  (math.sqrt(math.pow((vy + (math.sqrt((math.pow(LATERAL_ROVER_LEN,2)+math.pow(LONGITUDINAL_FRONT_LEN,2)))*omega*math.cos(math.atan2(LATERAL_ROVER_LEN,LONGITUDINAL_FRONT_LEN)))),2) + math.pow((vx - ((math.pow(LATERAL_ROVER_LEN,2)+math.pow(LONGITUDINAL_FRONT_LEN,2))*omega*math.sin(math.atan2(LATERAL_ROVER_LEN,LONGITUDINAL_FRONT_LEN)))),2)))   /  0.1 #math.sqrt((math.pow(LATERAL_ROVER_LEN,2)+math.pow(LONGITUDINAL_FRONT_LEN,2)))

    # w2
    w2 =  (math.sqrt(math.pow((vy + (math.sqrt((math.pow(LATERAL_ROVER_LEN,2)+math.pow(LONGITUDINAL_FRONT_LEN,2)))*omega*math.cos(math.atan2(LATERAL_ROVER_LEN,LONGITUDINAL_FRONT_LEN)))),2) + math.pow((vx - ((math.pow(LATERAL_ROVER_LEN,2)+math.pow(LONGITUDINAL_FRONT_LEN,2))*omega*math.sin(math.atan2(LATERAL_ROVER_LEN,LONGITUDINAL_FRONT_LEN)))),2)))   /  0.1 #math.sqrt((math.pow(LATERAL_ROVER_LEN,2)+math.pow(LONGITUDINAL_FRONT_LEN,2)))

    # w3
    w3 = (math.sqrt(math.pow(vy,2) + math.pow((vx - ((LATERAL_ROVER_LEN)*omega)),2)))  /   0.1 #LATERAL_ROVER_LEN 

    # w4
    w4 = (math.sqrt(math.pow(vy,2) + math.pow((vx + ((LATERAL_ROVER_LEN)*omega)),2)))  /   0.1 #LATERAL_ROVER_LEN 

    # w5
    w5 = (math.sqrt(math.pow((vy + (math.sqrt((math.pow(LATERAL_ROVER_LEN,2)+math.pow(LONGITUDINAL_REAR_LEN,2)))*omega*math.cos(math.atan2(LATERAL_ROVER_LEN,LONGITUDINAL_REAR_LEN)))),2) +  math.pow((vx - ((math.pow(LATERAL_ROVER_LEN,2)+math.pow(LONGITUDINAL_REAR_LEN,2))*omega*math.sin(math.atan2(LATERAL_ROVER_LEN,LONGITUDINAL_REAR_LEN)))),2)))    /    0.1 #math.sqrt((math.pow(LATERAL_ROVER_LEN,2)+math.pow(LONGITUDINAL_REAR_LEN,2)))

    #w6
    w6 = (math.sqrt(math.pow((vy - (math.sqrt((math.pow(LATERAL_ROVER_LEN,2)+math.pow(LONGITUDINAL_REAR_LEN,2)))*omega*math.cos(math.atan2(LATERAL_ROVER_LEN,LONGITUDINAL_REAR_LEN)))),2) + math.pow((vx + ((math.pow(LATERAL_ROVER_LEN,2)+math.pow(LONGITUDINAL_REAR_LEN,2))*omega*math.sin(math.atan2(LATERAL_ROVER_LEN,LONGITUDINAL_REAR_LEN)))),2)))    /    0.1 #math.sqrt((math.pow(LATERAL_ROVER_LEN,2)+math.pow(LONGITUDINAL_REAR_LEN,2)))

    
    # Coversion of rad/s of wheel to RPM (60/2pi)
    w = [w1, w2, w3, w4, w5, w6]
    RPM = [ws*60/(2*math.pi) for ws in w]
    
    for i,g in enumerate(gamma):
        if g < 0:
            gamma[i] = 180 + g
            RPM[i]     = -RPM[i]
        elif g >= 180:
            gamma[i] = g - 180
            RPM[i]     = -RPM[i]
           
    print("gamma: ", gamma) 
    print("RPM: ", RPM)

    vel_cmd_left    = [RPM[0], RPM[2], RPM[4]]
    servo_cmd_left  = [gamma[0], gamma[2], gamma[4]]
    vel_cmd_right   = [RPM[1], RPM[3], RPM[5]]
    servo_cmd_right = [gamma[1], gamma[3], gamma[5]]

    with cmd_lock:
        #shared_cmd["left"]  = (vel_cmd_left,  servo_cmd_left)
        #shared_cmd["right"] = (vel_cmd_right, servo_cmd_right)
        send_vel_servo_cmd(ser_left,  vel_cmd_left,  servo_cmd_left,  LEFT_TEENSY)
        send_vel_servo_cmd(ser_right, vel_cmd_right, servo_cmd_right, RIGHT_TEENSY)

# ============================================================
#  Threads
# ============================================================
#def sender_thread(side):
    #while True:
        #with cmd_lock:
            #vel, servo = shared_cmd[side]

        #send_vel_servo_cmd(ser_left if side == 'left' else ser_right, vel, servo, LEFT_TEENSY if side == "left" else RIGHT_TEENSY)

# ============================================================
#  Main
# ============================================================
ser_left  = serial.Serial('/dev/ttyUSB0', 115200)
ser_right = serial.Serial('/dev/ttyUSB1', 115200)

#time.sleep(0.01)

#t1 = threading.Thread(target=sender_thread, args=('left',),  daemon=True)
#t2 = threading.Thread(target=sender_thread, args=('right',), daemon=True)

#t1.start()
#t2.start()

while True:
    # packet = parse_packet(ser)
    # if packet is None:
    #     continue
    commands()
    

    # print(packet["payload"])
