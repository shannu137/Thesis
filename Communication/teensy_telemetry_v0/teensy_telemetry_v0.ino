/*
 * Standard CCSDS (Consultative Committee for Space Data Systems) type header
 * Packet: SYNC (2B) || Primary Header (6B) || Secondary Header (5B) || Payload (NB) || CRC (2B)
 * 
 * Primary Header: (6B)
 *      PACKET_INFO (2B): VERSION (3) || TYPE (1) || SEC_HDR_FLAG (1) || APID (11)
 *      SEQ_CTRL (2B): SEQ_FLAGS (2) || SEQ_COUNT (14)
 *      DATA_LENGTH (2B) (including secondary header, payload, crc)
 * Secondary Header: (5B)
 *      TIMESTAMP (4B)
 *      TEENSY_ID (1B)      
 *     
 * PACCKET_INFO:
 *      VERSION is 000
 *      TYPE: 0 for Telemetry
 *            1 for Telecommand
 *      SEC_HDR_FLAG is always 1
 *      For Telemetry (packet_type: 0)
 *         APID 0x001 - data telemetry
 *         APID 0x002 - heartbeat
 *         APID 0x003 - ACK response (confirms reception + execution)
 *         APID 0x004 - NACK response (confirms rejection + reason)
 *         APID 0x005 - fault report
 *         APID 0x020 - time sync response     
 *      For Telecommand (packet type: 1)
 *         APID 0x010 - motor (vel ctrl) + servo commands
 *         APID 0x011 - motor (pos ctrl) + servo commands
 *         APID 0x012 - system commands
 *         
 * SEQ_CTRL:
 *      SEQ_FLAGS: 11 always (unsegmented data)
 *      SEQ_COUNT: Counter for packets
 *      
 * CRC: (2B) is CRC16_CCITT
 * 
 * PAYLOAD:
 *      APID 0x001 (data telemetry): 37B
 *         long encoder[3], float velocity[3], float current[3] (9 x 4B = 36B)
 *         aux_flags: 1B (bit masked - compacr way to tell Jetson, the state of Teensy)
 *              0x00 - NO_FAULT
 *              0x01 - CPU_OVERLOAD
 *              0x02 - ENCODER_FAULT
 *              0x04 - OVERCURRENT
 *              0x08 - MOTOR_STALL
 *              0x10, 0x20, 0x40 - Can be used later
 *              0x80 - E_STOP_ACTIVE
 *      APID 0x002 (heartbeat): 3B
 *         system_state: 1B 
 *              0: IDLE, 1: ACTIVE, 2: SAFE, 3: ESTOP
 *         heartbeat_count: 1B
 *         aux_flags: 1B
 *      APID 0x003 (ACK response): 3B   
 *         ack_seq: 2B
 *         status: 1B - 0x00 only (command executed succesfully)
 *      APID 0x004 (NACK response): 3B
 *         nack_seq: 2B
 *         status: 1B
 *              0x01 - INVALID_VERSION
 *              0x02 - INVALID_TYPE
 *              0x03 - UNKNOWN_APID
 *              0x04 - SEQ_STALE
 *              0x05 - INVALID_TEENSY_ID
 *              0x06 - CRC_MISMATCH
 *              0x20 - VALUE_OUT_OF_RANGE
 *              0x30 - E_STOP_ACTIVE
 *              0x31 - STALL_DETECTED
 *              0x32 - OVERCURRENT
 *              0x33 - ENCODER_FAULT
 *              0x40 - INTERNAL_ERROR
 *      APID 0x005 (fault report): 3B 
 *         aux_flags: 1B
 *         motor_id: 2B
 *      APID 0x020 (time sync request/response): 4/12B
 *         From Jetson to Teensy (request): 4B
 *              t0 - timestamp when sending request
 *         From Teensy to Jetson (response): 12B
 *              t0 - echo of Jetson's original t0
 *              t1 - time when teensy received request
 *              t2 - time when teensy sends reply
 *              
 *      APID 0x010 (motor (vel ctrl) + servo commands): 24B
 *         float vel[3], int servo_pos[3] (6 x 4B = 24B)
 *      APID 0x011 (system commands): 1B
 *         command_id: 1B
 *              0x01 - STOP_ALL_MOTORS
 *              0x02 - ENABLE_MOTORS
 *              0x03 - DISABLE_MOTORS
 *              0x04 - RESET_ENCODERS
 *              0x05 - ESTOP
 *              0x06 - RELEASE_ESTOP
 *              0x08 - REBOOT_TEENSY           
 */

/*
 * Teensy CCSDS Motor Controller — Safety-Enhanced
 * ================================================
 * Req  1 : Periodic heartbeat + watchdog for communication loss
 * Req  2 : CRC / seq-no / ACK protocol — retransmit on packet loss
 * Req  3 : CRC-16 CCITT — discard corrupted, request retransmit
 * Req  4 : Range check on current / velocity / encoder
 * Req  5 : Encoder failure → DEGRADED + model-based velocity estimate
 * Req  6 : Encoder stops, wheel moves (current ≠ 0) → ESTOP
 * Req  7 : Motor off + nonzero current → AUTO ZERO CALIBRATION               // Not performed
 * Req  8 : Zero current with PWM → SAFE mode
 * Req  9 : Current ≠ 0 with PWM = 0 + velocity mismatch → ESTOP
 * Req 10 : vel vs commanded, high current → reduce speed cmd
 * Req 11 : Zero current + zero velocity (during run) → SAFE
 * Req 12 : Velocity opposite to cmd + high current → ESTOP
 * Req 13 : Nonzero cmd, zero current and velocity → DEGRADED
 * Req 14 : High current, low velocity → ESTOP
 * Req 15 : High current, zero velocity → ESTOP
 * Req 16 : High velocity, no rover movement → ESTOP
 * Req 17 : No HB / telemetry → ESTOP
 * Req 18 : Software watchdog — deadlock detection + forced reboot
 * Req 19 : Teensy self-check via watchdog (loop must kick on every cycle)
 * Req 20 : No commands from Jetson → ESTOP
 * Req 21 : Bounds check on all incoming command values
 * 
 * System Modes:
 *   0  INIT       — system startup and initialization
 *   1  IDLE       — system powered but inactive
 *   2  NOMINAL    — fully functional
 *   3  DEGRADED   — operating with reduced capability
 *   4  SAFE       — significant failure detected, safe operations only
 *   5  ESTOP      — critical, immediate shutdown of actuators/power
 *   6  INHIBIT    — motors disabled by command; controller remains operational but
                      rejects motion commands until explicitly re-enabled



Think about state transition from high to low
calibration
initialization
degraded mode
 */
 
#include <Arduino.h>
//#include <MotorController_Sim.h>
//#include <WDT_T4.h>                 // Teensy 4.x hardware watchdog (tonton81/WDT_T4)


// ============================================================
//  CCSDS PROTOCOL CONSTANTS
// ============================================================
#define SYNC_WORD            0xAA55
#define CCSDS_VERSION        0
#define CCSDS_TYPE_TLM       0
#define CCSDS_TYPE_TC        1
#define CCSDS_SECHDR_FLAG    1
#define CCSDS_SEQ_FLAG       3           // 0b11 (unsegmented data)

#define SYNC_SIZE_B          2
#define PRIMARY_SIZE_B       6
#define SECONDARY_SIZE_B     5
#define CRC_SIZE_B           2
#define MAX_PACKET_SIZE      512

// ============================================================
//  AUX FLAGS
// ============================================================
#define AUX_CPU_OVERLOAD         0x01
#define AUX_ENCODER_FAULT        0x02
#define AUX_OVERCURRENT          0x04
#define AUX_STALL_DETECTED       0x08
#define AUX_CURRSENSOR_FAULT     0x10
#define AUX_DRIVER_FAULT         0x20
#define AUX_SENSOR_OUT_OF_RANGE  0x30
#define AUX_ESTOP_ACTIVE         0x80

// ============================================================
//  NACK STATUS CODES
// ============================================================
#define NACK_INVALID_VERSION     0x01
#define NACK_INVALID_TYPE        0x02
#define NACK_UNKNOWN_APID        0x03
#define NACK_SEQ_STALE           0x04
#define NACK_INVALID_TEENSY_ID   0x05
#define NACK_CRC_MISMATCH        0x06
#define NACK_VALUE_OUT_OF_RANGE  0x20
#define NACK_ESTOP_ACTIVE        0x30
#define NACK_SAFE_ACTIVE         0X31
#define NACK_INHIBIT_ACTIVE      0x32

// ============================================================
//  SYSTEM MODES
// ============================================================
#define MODE_INIT            0
#define MODE_IDLE            1
#define MODE_NOMINAL         2
#define MODE_DEGRADED        3
#define MODE_SAFE            4
#define MODE_ESTOP           5
#define MODE_INHIBIT         6

// ============================================================
//  HARDWARE CONFIG
// ============================================================
#define TEENSY_ID            1
#define NUM_MOTORS           3

#define ENCA                 18
#define ENCB                 19
#define PWM_PIN              7
#define IN1                  5
#define IN2                  6

// ============================================================
//  TIMING (ms)
// ============================================================
#define TELEMETRY_PERIOD_MS    10
#define HEARTBEAT_PERIOD_MS    1000
#define MAX_LOOP_TIME_MS       1000   
#define INIT_TIMEOUT_MS        5000    // Max time allowed in INIT
#define JETSON_CMD_TIMEOUT_MS  5000    // no commands → ESTOP after this
#define READ_TIMEOUT_MS        50      // To read commands from serial

// Watchdog kick period (ms) — must be < hardware WDT period
#define SW_WDT_KICK_PERIOD_MS  500

// ============================================================
//  CURRENT THRESHOLDS  (mA)
// ============================================================
#define CURR_ZERO_THR        50.0f      // below → treat as zero current
#define CURR_HIGH_THR        1000.0f    // overcurrent warning
#define CURR_STALL_THR       3500.0f    // stall current

// ============================================================
//  VELOCITY THRESHOLDS  (rpm)
// ============================================================
#define VEL_ZERO_THR           0.1f       // below → treat as zero velocity
#define VEL_LOW_THR            20.0f      // low velocity
#define VEL_MISMATCH_THR       0.2f       // between commanded and actual
#define SPEED_REDUCTION_FACTOR 0.6f       // w.r.t VEL_LOW_THR in SAFE mode

// ============================================================
//  COMMAND BOUNDS 
// ============================================================
#define VEL_CMD_MAX          50.0f
#define VEL_CMD_MIN         -50.0f
#define POS_CMD_MAX          1000000L
#define POS_CMD_MIN         -1000000L
#define SERVO_CMD_MAX        180
#define SERVO_CMD_MIN        0

// ============================================================
//  MODEL-BASED VELOCITY
// ============================================================
#define MOTOR_KT             0.01f     // Nm/A
#define MOTOR_R              1.0f      // Ohm

// ============================================================
//  DATA STRUCTURES
// ============================================================
typedef struct {
    uint8_t  aux_flags;
    uint8_t  mode;
    uint16_t motor_id;      // Bitmask of affected motors
} FaultResult;

typedef struct {
    float   vel[NUM_MOTORS];
    int32_t servo_pos[NUM_MOTORS];
} MotorVelServoCmd;

// ============================================================
//  GLOBAL STATE
// ============================================================

// Packet counters
static uint16_t   seq_tx          = 0;
static uint8_t    heartbeat_count = 0;
static uint16_t   prev_rx_seq     = 0xFFFF;     // invalid initial value

// System state machine
static uint8_t     system_mode   = MODE_INIT;   // current FSM state
static uint8_t     prev_mode     = MODE_INIT;
static FaultResult g_fault      = {0};
static uint32_t    init_entry_ms = 0;           // When INIT was entered

static uint32_t   last_jetson_cmd_ms = 0;   
static uint32_t   last_telem_ms      = 0;
static uint32_t   last_hb_ms         = 0;
static uint32_t   last_wdt_kick_ms   = 0; 
static uint32_t   loop_start         = 0;  

// Hardware watchdog instance (WDT_T4 library)
WDT_T4<WDT1> wdt;

MotorController_Sim motor[NUM_MOTORS];

// ============================================================
//  CRC-16 / CCITT 
// ============================================================
uint16_t crc16_ccitt(const uint8_t* data, uint16_t len) {
  uint16_t crc = 0xFFFF;
  for (uint16_t i = 0; i < len; i++) {
      crc ^= (uint16_t)data[i] << 8;
      for (uint8_t j = 0; j < 8; j++)
          crc = (crc & 0x8000) ? (crc << 1) ^ 0x1021 : (crc << 1);
  }
  return crc;
}

// ============================================================
//  ENDIAN HELPERS
// ============================================================
inline void to_le16(uint8_t* dst, uint16_t value){
  dst[0] = value & 0xFF;
  dst[1] = (value >> 8) & 0xFF;
}

inline void to_le32(uint8_t* dst, uint32_t value){
  dst[0] = value & 0xFF;
  dst[1] = (value >> 8) & 0xFF;
  dst[2] = (value >> 16) & 0xFF;
  dst[3] = (value >> 24) & 0xFF;
}

inline void to_le_float(uint8_t* dst, float value){
  uint32_t temp;
  memcpy(&temp, &value, sizeof(float));
  to_le32(dst, temp);
}

inline void to_le_int32(uint8_t* dst, int32_t value){
  to_le32(dst, (uint32_t)value);
}

inline uint16_t from_le16(const uint8_t* src) {
  return (uint16_t)src[0] | ((uint16_t)src[1] << 8);
}

inline uint32_t from_le32(const uint8_t* src) {
  return (uint32_t)src[0]
       | ((uint32_t)src[1] << 8)
       | ((uint32_t)src[2] << 16)
       | ((uint32_t)src[3] << 24);
}

inline float from_le_float(const uint8_t* src)
{
  float v;
  memcpy(&v, src, sizeof(float));
  return v;
}

inline int32_t from_le_int32(const uint8_t* src)
{
  return (int32_t)(from_le32(src));
}

// ============================================================
//  CCSDS HEADER BUILDERS
// ============================================================
uint16_t make_packet_info(uint8_t pkt_type, uint16_t apid) {
  uint16_t pkt  = ((CCSDS_VERSION     & 0x7) << 13);
           pkt |= ((pkt_type          & 0x1) << 12);
           pkt |= ((CCSDS_SECHDR_FLAG & 0x1) << 11);
           pkt |= (apid & 0x7FF);

  return pkt;
}

uint16_t make_seq_ctrl() {
  return ((CCSDS_SEQ_FLAG & 0x3) << 14) | (seq_tx++ & 0x3FFF);
}

// ============================================================
//  PACKET BUILDER
// ============================================================
void send_packet(uint16_t apid, uint8_t pkt_type, uint32_t ts,
                 const uint8_t* payload, uint16_t payload_len)
{
  uint8_t pkt[MAX_PACKET_SIZE];
  uint16_t i=0;

  // Sync
  to_le16(pkt + i, SYNC_WORD);   i+=2;

  // Primary Header
  uint16_t packet_info = make_packet_info(pkt_type, apid);
  uint16_t seq_ctrl    = make_seq_ctrl();
  uint16_t data_length = (SECONDARY_SIZE_B + payload_len + CRC_SIZE_B) - 1; 
  
  to_le16(pkt + i, packet_info); i+=2;
  to_le16(pkt + i, seq_ctrl);    i+=2;
  to_le16(pkt + i, data_length); i+=2;

  // Secondary Header
  to_le32(pkt + i, ts);          i+=4;
  pkt[i] = TEENSY_ID;            i+=1;

  // Payload
  memcpy(pkt + i, payload, payload_len); i+= payload_len;

  // CRC
  uint16_t crc = crc16_ccitt(pkt + SYNC_SIZE_B, PRIMARY_SIZE_B + SECONDARY_SIZE_B + payload_len);
  to_le16(pkt + i, crc); i+=2;

  // Serial.write(pkt, i);
}

// ============================================================
//  FAULT ANALYSIS AND MODE TRANSITION
// ============================================================
FaultResult evaluate_faults(float vel[NUM_MOTORS], float curr[NUM_MOTORS],
                            int pwm[NUM_MOTORS], float cmd_vel[NUM_MOTORS],
                            uint32_t loop_ms)
{
  FaultResult result = {0};

  bool cpu_overload     = (loop_ms > MAX_LOOP_TIME_MS);
  bool encoder_fault    = false;
  bool over_current     = false;
  bool motor_stall      = false;
  bool currsensor_fault = false;
  bool driver_fault     = false;
  bool any_pwm          = false;
  bool do_degraded      = false;
  bool do_estop         = false;
  bool do_safe          = false;

  for (int i = 0; i < NUM_MOTORS; i++)
{
    bool  pwm0   = (pwm[i] == 0);
    float v      = fabsf(vel[i]);
    float c      = fabsf(curr[i]);
    float cv     = fabsf(cmd_vel[i]);
    float sign_v = (vel[i] >= 0) ? 1.0f : -1.0f;
    float sign_c = (cmd_vel[i] >= 0) ? 1.0f : -1.0f;

    bool vel_zero   = (v  < VEL_ZERO_THR);
    bool vel_low    = (v  < VEL_LOW_THR);
    bool cmd_nz     = (cv > VEL_ZERO_THR);
    bool curr_zero  = (c  < CURR_ZERO_THR);
    bool curr_high  = (c  > CURR_HIGH_THR);
    bool curr_stall = (c  > CURR_STALL_THR);

    if (!pwm0) any_pwm = true;

    // ------------------ zero pwm cases ---------------------------
    if (pwm0){
      if (curr_zero && vel_zero){
        // IDLE → no fault
      }
      else if (curr_zero && !vel_zero)
      {
        // ENCODER_FAULT (free wheel) or mechanical issue
        do_safe       = true;
        encoder_fault = true;
        result.motor_id  |= (1 << i);
      }
      else if (!curr_zero && vel_zero)
      {
        // do calibration
      }
      else
      {
        // Any current and velocity while PWM=0 → DRIVER_FAULT
        do_estop     = true;
        driver_fault = true;
        result.motor_id  |= (1 << i);
      }
      continue;
    }

    // ------------------ non-zero pwm cases ---------------------------
    if (curr_zero && vel_zero)
    {
      // DRIVER_FAULT
      do_safe      = true;
      driver_fault = true;
      result.motor_id  |= (1 << i);
      continue;
    }

    if (curr_zero && !vel_zero)
    {
      do_degraded      = true;
      currsensor_fault = true;
      result.motor_id  |= (1 << i);
      continue;
    }

    if (vel_zero && !curr_zero) {
      if (curr_stall || curr_high) 
      {
        do_estop    = true;
        motor_stall = true;
        result.motor_id  |= (1 << i);
      } 
      else {
        do_degraded   = true;
        encoder_fault = true;
        result.motor_id  |= (1 << i);
      }
      continue;
    }

    if (cmd_nz && !vel_zero && (sign_v != sign_c)) {
      do_estop = true;
      result.motor_id  |= (1 << i);
      continue;
    }

    if (curr_high && vel_low && !vel_zero) {
      do_safe      = true;
      over_current = true;
      result.motor_id  |= (1 << i);
      continue;
    }

    if ((fabs(cv - v) > VEL_MISMATCH_THR) && curr_high){
      do_safe = true;
      continue;
    }

    if (curr_high){
      do_safe      = true;
      over_current = true;
      result.motor_id  |= (1 << i);
    }
  }

  // ------------------ build aux_flags ---------------------------
  if (cpu_overload)                  result.aux_flags |= AUX_CPU_OVERLOAD;
  if (encoder_fault)                 result.aux_flags |= AUX_ENCODER_FAULT;
  if (over_current)                  result.aux_flags |= AUX_OVERCURRENT;
  if (motor_stall)                   result.aux_flags |= AUX_STALL_DETECTED;
  if (currsensor_fault)              result.aux_flags |= AUX_CURRSENSOR_FAULT;
  if (driver_fault)                  result.aux_flags |= AUX_DRIVER_FAULT;
  if (do_estop)                      result.aux_flags |= AUX_ESTOP_ACTIVE;

  // ------------------ Mode ---------------------------
  if      (do_estop)    result.mode = MODE_ESTOP;
  else if (do_safe)     result.mode = MODE_SAFE;
  else if (do_degraded) result.mode = MODE_DEGRADED;
  else if (any_pwm)     result.mode = MODE_NOMINAL;
  else                  result.mode = MODE_IDLE;

  return result;
}

// ============================================================
//  PAYLOAD BUILDER
// ============================================================
void send_telemetry(uint32_t ts, long enc[3], float vel[3], float curr[3], const FaultResult* fault){
  uint8_t  payload[37];
  uint16_t i=0;

  to_le_int32(payload+i, (int32_t)enc[0]);  i+=4;
  to_le_int32(payload+i, (int32_t)enc[1]);  i+=4;
  to_le_int32(payload+i, (int32_t)enc[2]);  i+=4;

  to_le_float(payload+i, vel[0]);  i+=4;
  to_le_float(payload+i, vel[1]);  i+=4;
  to_le_float(payload+i, vel[2]);  i+=4;

  to_le_float(payload+i, curr[0]); i+=4;
  to_le_float(payload+i, curr[1]); i+=4;
  to_le_float(payload+i, curr[2]); i+=4;

  payload[i++] = fault->aux_flags;

  send_packet(0x001, CCSDS_TYPE_TLM, ts, payload, i);
}

void send_heartbeat(const FaultResult* fault)
{
  uint8_t  payload[3];
  uint16_t i = 0;

  payload[i++] = fault->mode;
  payload[i++] = heartbeat_count++;
  payload[i++] = fault->aux_flags;

  send_packet(0x002, CCSDS_TYPE_TLM, millis(), payload, i);
}

void send_ack(uint16_t ack_seq)
{
  uint8_t  payload[3];
  uint16_t i = 0;

  to_le16(payload, ack_seq); i += 2;
  payload[i++] = 0x00;

  send_packet(0x003, CCSDS_TYPE_TLM, millis(), payload, i);
}

void send_nack(uint16_t nack_seq, uint8_t nack_status)
{
  uint8_t  payload[3];
  uint16_t i = 0;

  to_le16(payload, nack_seq); i += 2;
  payload[i++] = nack_status;

  send_packet(0x004, CCSDS_TYPE_TLM, millis(), payload, i);
}

void send_fault_report(const FaultResult* fault)
{
  uint8_t  payload[3];
  uint16_t i = 0;
  
  payload[i++] = fault->aux_flags;
  to_le16(payload + i, fault->motor_id); i += 2;
  
  send_packet(0x005, CCSDS_TYPE_TLM, millis(), payload, i);
}

void send_time_sync_response(uint32_t t0, uint32_t t1)
{
  uint8_t  payload[12];
  uint16_t i = 0;

  to_le32(payload + i, t0);       i+= 4;
  to_le32(payload + i, t1);       i+= 4;
  to_le32(payload + i, millis()); i+= 4;

  send_packet(0x020, CCSDS_TYPE_TLM, millis(), payload, i);
}

// ============================================================
//  ESTOP - cuts all PWM immediately
// ============================================================
void engage_estop()
{
  for (int i = 0; i < NUM_MOTORS; i++)
    motor[i].setPWM(0);      
  system_mode          = MODE_ESTOP;
  g_fault.mode         = MODE_ESTOP;
  g_fault.aux_flags   |= AUX_ESTOP_ACTIVE;
  send_fault_report(&g_fault);
}

// ============================================================
// COMMAND VALIDATION
// ============================================================
bool validate_vel_cmd(const MotorVelServoCmd* cmd, uint16_t seq)
{
  for (int i = 0; i < NUM_MOTORS; i++) {
    if (cmd->vel[i] < VEL_CMD_MIN || cmd->vel[i] > VEL_CMD_MAX) {
      send_nack(seq, NACK_VALUE_OUT_OF_RANGE); return false;
    }
    if (cmd->servo_pos[i] < SERVO_CMD_MIN || cmd->servo_pos[i] > SERVO_CMD_MAX) {
      send_nack(seq, NACK_VALUE_OUT_OF_RANGE); return false;
    }
  }
  return true;
}

// ============================================================
// SENSOR RANGE CHECKS
// ============================================================
bool sensor_range_ok(float vel[3], float curr[3], long enc[3])
{
    for (int i = 0; i < NUM_MOTORS; i++) {
        if (fabsf(vel[i])  > 1.5f * VEL_CMD_MAX)          return false;
        if (fabsf(curr[i]) > 1.2f * CURR_STALL_THR) return false;
        if (enc[i] > POS_CMD_MAX || enc[i] < POS_CMD_MIN) return false;
    }
    return true;
}

// ============================================================
// PACKET PARSER
// ============================================================
bool find_sync(Stream& ser)
{
  bool state = 0;
  while (ser.available()){
    uint8_t v = ser.read();
    if (!state){
      if (v == (SYNC_WORD & 0xFF))
        state = 1;
    }
    else{
      if (v == ((SYNC_WORD >> 8) & 0xFF))
        return true;
      else if (v == (SYNC_WORD & 0xFF))
        state = 1;
      else
        state = 0;
    }
  }
  return false;
}

void read_exact(Stream& ser, uint8_t* buf, uint16_t n)
{
  uint32_t t0 = millis();
  uint16_t i = 0;
  while (i < n && (millis() - t0) < READ_TIMEOUT_MS)
  {
    if (ser.available())
      buf[i++] = ser.read();
  }
}

void parse_packet(Stream& ser)
{
  // SYNC
  if (!find_sync(ser))
    return;
      
  uint8_t primary[PRIMARY_SIZE_B];
  uint8_t secondary[SECONDARY_SIZE_B];
  uint8_t crc_buf[CRC_SIZE_B];
  uint8_t payload[MAX_PACKET_SIZE];
    
  // Primary header
  read_exact(ser, primary, PRIMARY_SIZE_B);

  uint16_t pkt_info = from_le16(primary);
  uint16_t seq_ctrl = from_le16(primary + 2);
  uint16_t data_len = from_le16(primary + 4) + 1;

  uint8_t  pkt_version  = (pkt_info >> 13) & 0x7;
  uint8_t  pkt_type     = (pkt_info >> 12) & 0x1;
  uint8_t  sec_hdr_flag = (pkt_info >> 11) & 0x1;
  uint16_t apid         =  pkt_info & 0x7FF;
  uint16_t seq_count    =  seq_ctrl & 0x3FFF;

  if (pkt_version != CCSDS_VERSION || sec_hdr_flag != 1){
    send_nack(seq_count, NACK_INVALID_VERSION);
    return;
  }
  if (pkt_type != CCSDS_TYPE_TC){
    send_nack(seq_count, NACK_INVALID_TYPE);
    return;
  }

  if (prev_rx_seq == 0xFFFF){
    prev_rx_seq = seq_count;
  }
  else{
    uint16_t diff = (seq_count - prev_rx_seq) & 0x3FFF;
    if (diff == 0 || diff >= 8192){
      send_nack(seq_count, NACK_SEQ_STALE); 
      return;
    }
    prev_rx_seq = seq_count;
  }

  // Secondary header
  read_exact(ser, secondary, SECONDARY_SIZE_B);
  uint32_t ts            = from_le32(secondary);
  uint8_t  src_teensy_id = secondary[4];

  if (src_teensy_id != TEENSY_ID){
    send_nack(seq_count, NACK_INVALID_TEENSY_ID);
  }

  // Payload
  uint16_t payload_len = data_len - SECONDARY_SIZE_B - CRC_SIZE_B;
  read_exact(ser, payload, payload_len);

  // CRC
  read_exact(ser, crc_buf, CRC_SIZE_B);

  uint8_t temp[PRIMARY_SIZE_B + SECONDARY_SIZE_B + payload_len];
  memcpy(temp, primary, PRIMARY_SIZE_B);
  memcpy(temp + PRIMARY_SIZE_B, secondary, SECONDARY_SIZE_B);
  memcpy(temp + PRIMARY_SIZE_B + SECONDARY_SIZE_B, payload, payload_len);
  
  uint16_t crc_calc = crc16_ccitt(temp, PRIMARY_SIZE_B + SECONDARY_SIZE_B + payload_len);
  uint16_t crc_rx = from_le16(crc_buf);

  if (crc_rx != crc_calc){
    send_nack(seq_count, NACK_CRC_MISMATCH);
    return;
  }

  last_jetson_cmd_ms = millis();

  // Parse payload
  switch (apid){
    case 0x010:{
      if (system_mode == MODE_ESTOP){
        send_nack(seq_count, NACK_ESTOP_ACTIVE);
        return;
      }
      else if (system_mode == MODE_INHIBIT){
        send_nack(seq_count, NACK_INHIBIT_ACTIVE);
        return;
      }
      MotorVelServoCmd cmd;
      parse_vel_servo_cmd(payload, &cmd, seq_count);
      break;
    }
    case 0x011:{
      uint8_t cmd = payload[0];
      switch (cmd) {
        case 0x01: // STOP_ALL_MOTORS
          for (int m = 0; m < NUM_MOTORS; m++){
            motor[m].setPWM(0);
          }
          system_mode = MODE_IDLE;
          send_ack(seq_count);
          break;
        case 0x02: // ENABLE_MOTORS
          if (system_mode == MODE_ESTOP){
            send_nack(seq_count, NACK_ESTOP_ACTIVE); 
            break;
          }
          system_mode = MODE_NOMINAL;
          send_ack(seq_count);
          break;
        case 0x03: // DISABLE_MOTORS
          for (int m = 0; m < NUM_MOTORS; m++){
            motor[m].setPWM(0);
          }
          system_mode = MODE_INHIBIT;
          send_ack(seq_count);
          break;
        case 0x04: // RESET_ENCODERS
          for (int m = 0; m < NUM_MOTORS; m++){
            motor[m].resetEncoder();
          }
          send_ack(seq_count);
          break;
        case 0x05: // ESTOP
          engage_estop();
          send_ack(seq_count);
          break;
        case 0x06: // RELEASE_ESTOP — only if faults cleared
          if (!(g_fault.aux_flags & ~AUX_ESTOP_ACTIVE)){
              system_mode = MODE_IDLE;
              g_fault.aux_flags &= ~AUX_ESTOP_ACTIVE;
              send_ack(seq_count);
          } 
          else{
              send_nack(seq_count, NACK_ESTOP_ACTIVE);
          }
          break;
        case 0x08: // REBOOT_TEENSY  
          send_ack(seq_count);
          delay(50);
          SCB_AIRCR = 0x05FA0004;  // Cortex-M software reset
          break;
        default:
          send_nack(seq_count, NACK_VALUE_OUT_OF_RANGE);
      }
      break;
    }
    case 0x020:{
      uint32_t t0_echo = from_le32(payload);
      send_time_sync_response(t0_echo, millis());
      break;
    }
    default:{
      send_nack(seq_count, NACK_UNKNOWN_APID);
      return;
    }
  }
}

void parse_vel_servo_cmd(const uint8_t* payload, MotorVelServoCmd* cmd, uint16_t seq)
{
  uint16_t i = 0;
  for (int m = 0; m < NUM_MOTORS; m++){
    cmd->vel[m] = from_le_float(payload + i); i+= 4;
    if (system_mode == MODE_SAFE && cmd->vel[m] > SPEED_REDUCTION_FACTOR * VEL_LOW_THR){
      send_nack(seq, NACK_SAFE_ACTIVE);
    }
  }
  for (int m = 0; m < NUM_MOTORS; m++){
    cmd->servo_pos[m] = from_le_int32(payload + i); i+=4;
  }

  if(!validate_vel_cmd(cmd, seq))
    return;

  if (system_mode == MODE_NOMINAL || system_mode == MODE_DEGRADED){
    for (int m = 0; m < NUM_MOTORS; m++){
      motor[m].setTargetVelocity(cmd->vel[m]);
//      servo[m].setAngle(cmd.servo_pos[m]);
    }
  }
  send_ack(seq);
}


// ============================================================
// SETUP
// ============================================================
void setup(){
  Serial.begin(115200);
  
  // Hardware watchdog: 2-second timeout  (Req 18, 19)
  WDT_timings_t cfg;
  cfg.trigger = 1;   // interrupt at 1 s (warning)
  cfg.timeout = 2;   // reset at 2 s
  wdt.begin(cfg);

  for (int i = 0; i < NUM_MOTORS; i++) {
    motor[i].begin();
  }

  system_mode = MODE_IDLE;
  g_fault     = {0};
  g_fault.mode = MODE_IDLE;
}

// ============================================================
// MAIN LOOP
// ============================================================
void loop(){
  uint32_t now        = millis();
  uint32_t loop_time  = now - loop_start;
  loop_start          = now;

  // ------------------ kick watchdog each cycle ---------------
  if ((now - last_wdt_kick_ms) >= SW_WDT_KICK_PERIOD_MS) {
      wdt.feed();
      last_wdt_kick_ms = now;
  }

  // ------------------ read sensors ---------------
  long  enc[NUM_MOTORS];
  float vel[NUM_MOTORS];
  float curr[NUM_MOTORS];
  int   pwm[NUM_MOTORS];
  float cmd_vel[NUM_MOTORS];

  for (int i = 0; i < NUM_MOTORS; i++){
    enc[i]     = motor[i].getEncoder();
    vel[i]     = motor[i].getVelocity()
    curr[i]    = motor[i].getCurrent();
    pwm[i]     = motor[i].getPWM();
    cmd_vel[i] = motor[i].getCmdVel();
  }

  // ------------------ sensor range check ---------------
  if (!sensor_range_ok(vel, curr, enc)) {
    g_fault.aux_flags |= AUX_SENSOR_OUT_OF_RANGE;
    engage_estop();
  }

  // ------------------ fault evaluation ---------------
  FaultResult fault = evaluate_faults(vel, curr, pwm, cmd_vel, loop_time);
  g_fault.aux_flags = fault.aux_flags;
  g_fault.motor_id  = fault.motor_id;
  g_fault.mode      = fault.mode;
  
  prev_mode = g_fault.mode;

  if(g_fault.mode == MODE_ESTOP){
    engage_estop();
  }

  // ------------------ no commands watchdog ---------------
  if ((now - last_jetson_cmd_ms) > JETSON_CMD_TIMEOUT_MS && system_mode == MODE_NOMINAL) {
    engage_estop();
  }

  // ------------------ parse incoming packets ---------------
  if (Serial.available()){
    parse_packet(Serial);
    last_jetson_cmd_ms = now; 
  }

  // ------------------ periodic telemetry ---------------
  if ((now - last_telem_ms) >= TELEMETRY_PERIOD_MS){
    send_telemetry(now, enc, vel, curr, &g_fault);
    last_telem_ms = now;
  }

  // ------------------ periodic heartbeat ---------------
  if ((now - last_hb_ms) >= HEARTBEAT_PERIOD_MS){
    send_heartbeat(&fault);
    if (g_fault.aux_flags)
      send_fault_report(&g_fault);
    last_hb_ms = now;
  }
}
