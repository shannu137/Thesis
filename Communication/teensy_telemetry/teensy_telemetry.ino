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
 *      APID 0x011 (motor (pos ctrl) + servo commands): 24B
 *         long pos[3], int servo_pos[3] (6 x 4B = 24B)
 *      APID 0x012 (system commands): 1B
 *         command_id: 1B
 *              0x01 - STOP_ALL_MOTORS
 *              0x02 - ENABLE_MOTORS
 *              0x03 - DISABLE_MOTORS
 *              0x04 - RESET_ENCODERS
 *              0x05 - ESTOP
 *              0x06 - RELEASE_ESTOP
 *              0x08 - REBOOT_TEENSY           
 */

#include <Arduino.h>
#include <MotorController.h>

#define SYNC_WORD            0xAA55
#define CCSDS_VERSION        0
#define CCSDS_TYPE           0  // Telemetry
#define CCSDS_SECHDR_FLAG    1
#define CCSDS_SEQ_FLAG       3  // 0b11 (unsegmented data)

#define SYNC_SIZE_B          2
#define PRIMARY_SIZE_B       6
#define SECONDARY_SIZE_B     5
#define CRC_SIZE_B           2

#define AUX_CPU_OVERLOAD         0x01
#define AUX_ENCODER_FAULT        0x02
#define AUX_OVERCURRENT          0x04
#define AUX_STALL_DETECTED       0x08
#define AUX_DRIVER_FAULT         0x10
#define AUX_ESTOP_ACTIVE         0x80

#define NACK_INVALID_VERSION     0x01
#define NACK_INVALID_TYPE        0x02
#define NACK_UNKNOWN_APID        0x03
#define NACK_SEQ_STALE           0x04
#define NACK_INVALID_TEENSY_ID   0x05
#define NACK_CRC_MISMATCH        0x06
#define NACK_VALUE_OUT_OF_RANGE  0x20
#define NACK_ESTOP_ACTIVE        0x30
#define NACK_STALL_DETECTED      0x31
#define NACK_OVERCURRENT         0x32
#define NACK_ENCODER_FAULT       0x33
#define NACK_INTERNAL_ERROR      0x40

#define TEENSY_ID                1
#define MAX_LOOP_TIME            0
#define VEL_MIN_THRESHOLD        0
#define CURR_ZERO_THRESHOLD      0 
#define CURR_THRESHOLD           0
#define CURR_STALL_CURRENT       0

#define TELEMETRY_TIMEPERIOD 10;   // in millis
#define HEARTBEAT_TIMEPERIOD 1000; // in millis

#define ENCA  18
#define ENCB  19
#define PWM   7
#define IN1   5
#define IN2   6

static uint16_t seq_counter = 0;
static uint8_t heartbeat_count = 0;
static uint16_t prev_rx_seq_count = 0xFFFF;  // invalid initial value

typedef struct {
    uint8_t  aux_flags;
    uint8_t  state;
    uint16_t motor_id;
} FaultResult;

typedef struct {
    float    vel[3];
    int32_t  servo_pos[3];
} MotorVelServoCmd;

typedef struct {
    int32_t  pos[3];
    int32_t  servo_pos[3];
} MotorPosServoCmd;

uint16_t crc16_ccitt(const uint8_t* data, uint16_t data_length){
  uint16_t crc = 0xFFFF;
  for(uint16_t i=0; i<data_length; i++){
    crc ^= (uint16_t)data[i] << 8;
    for(uint16_t j=0; j<8; j++){
      if(crc & 0x8000){
        crc = (crc << 1) ^ 0x1021;
      }
      else{
        crc <<= 1;
      }
    }
  }
  return crc;
}

uint16_t create_packet_info(uint16_t apid){
  uint16_t packet_info = (CCSDS_VERSION & 0x7) << 13;
  packet_info |= (CCSDS_TYPE & 0x1) << 12;
  packet_info |= (CCSDS_SECHDR_FLAG & 0x1) << 11;
  packet_info |= (apid & 0x7FF);

  return packet_info;
}

uint16_t create_seq_ctrl(){
  uint16_t seq_ctrl = (CCSDS_SEQ_FLAG & 0x3) << 14;
  seq_ctrl |= (seq_counter++ & 0x3FFF);

  return seq_ctrl;
}

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

void send_packet(uint16_t apid, uint32_t ts, const uint8_t* payload, uint16_t payload_length){
  uint8_t packet[512];
  
  uint16_t i=0;

  // Sync
  to_le16(packet+i, SYNC_WORD); i+=2;

  // Primary Header
  uint16_t packet_info = create_packet_info(apid);
  uint16_t seq_ctrl = create_seq_ctrl();
  uint16_t data_length = (SECONDARY_SIZE_B + payload_length + CRC_SIZE_B) - 1; 
  
  to_le16(packet+i, packet_info); i+=2;
  to_le16(packet+i, seq_ctrl); i+=2;
  to_le16(packet+i, data_length); i+=2;

  // Secondary Header
  to_le32(packet+i, ts); i+=4;
  packet[i] = TEENSY_ID; i+=1;

  // Payload
  memcpy(packet+i, payload, payload_length); i+= payload_length;

  // CRC
  uint16_t crc = crc16_ccitt(packet+SYNC_SIZE_B, PRIMARY_SIZE_B + SECONDARY_SIZE_B + payload_length);
  to_le16(packet+i, crc); i+=2;

  Serial.write(packet, i);
}

FaultResult get_fault_report(float vel[3], float curr[3], int pwm[3], uint32_t loop_time)
{
    FaultResult result = {0};

    bool cpu_overload  = (loop_time > MAX_LOOP_TIME);
    bool encoder_fault = false;
    bool over_current  = false;
    bool motor_stall   = false;
    bool driver_fault  = false;
    bool any_pwm_on    = false;

    for (int i = 0; i < 3; i++)
    {
        bool pwm0 = (pwm[i] == 0);
        bool enc0 = (fabsf(vel[i]) <= VEL_MIN_THRESHOLD);

        float cur = curr[i];

        bool curr0        = (cur <= CURR_ZERO_THRESHOLD);
        bool curr_lt      = (cur > CURR_ZERO_THRESHOLD &&
                             cur <= CURR_THRESHOLD);
        bool curr_gt      = (cur > CURR_THRESHOLD &&
                             cur <= CURR_STALL_CURRENT);
        bool curr_stall   = (cur > CURR_STALL_CURRENT);

        if (!pwm0) any_pwm_on = true;

        if (pwm0)
        {
            if (curr0 && enc0)
            {
                // IDLE → no fault
            }
            else if (curr0 && !enc0)
            {
                // ENCODER_FAULT (free wheel)
                encoder_fault = true;
                result.motor_id  |= (1 << i);
            }
            else
            {
                // Any current while PWM=0 → DRIVER_FAULT
                driver_fault = true;
                result.motor_id  |= (1 << i);
            }
            continue;
        }
        
        if (!pwm0)
        {
            if (curr0 && enc0)
            {
                // DRIVER_FAULT
                driver_fault = true;
                result.motor_id  |= (1 << i);
            }
            else if (curr_lt && enc0)
            {
                // ENCODER_FAULT
                encoder_fault = true;
                result.motor_id  |= (1 << i);
            }
            else if (curr_gt && enc0)
            {
                // MOTOR_STALL + DRIVER_FAULT
                motor_stall = true;
                driver_fault = true;
                result.motor_id  |= (1 << i);
            }
            else if (curr_stall && enc0)
            {
                // MOTOR_STALL
                motor_stall = true;
                result.motor_id  |= (1 << i);
            }
            else if (curr0 && !enc0)
            {
                // ENCODER_FAULT + DRIVER_FAULT
                encoder_fault = true;
                driver_fault = true;
                result.motor_id  |= (1 << i);
            }
            else if (curr_lt && !enc0)
            {
                // ACTIVE → no fault
            }
            else if (curr_gt && !enc0)
            {
                // OVER_CURRENT
                over_current = true;
                result.motor_id  |= (1 << i);
            }
            else if (curr_stall && !enc0)
            {
                // OVER_CURRENT
                over_current = true;
                result.motor_id  |= (1 << i);
            }
        }
    }

    if (cpu_overload)                  result.aux_flags |= AUX_CPU_OVERLOAD;
    if (encoder_fault)                 result.aux_flags |= AUX_ENCODER_FAULT;
    if (over_current)                  result.aux_flags |= AUX_OVERCURRENT;
    if (motor_stall)                   result.aux_flags |= AUX_STALL_DETECTED;
    if (driver_fault)                  result.aux_flags |= AUX_DRIVER_FAULT;
    if (motor_stall || driver_fault)   result.aux_flags |= AUX_ESTOP_ACTIVE;

    if (driver_fault || motor_stall)
        result.state = 3;                      // ESTOP
    else if (encoder_fault || over_current)
        result.state = 2;                      // SAFE_MODE
    else if (any_pwm_on)
        result.state = 1;                      // ACTIVE
    else
        result.state = 0;                      // IDLE

    return result;
}

void send_telemetry(uint32_t ts, long encoder[3], float vel[3], float curr[3], const FaultResult* fault){
  uint8_t payload[37];
  uint16_t i=0;

  to_le_int32(payload+i, encoder[0]); i+=4;
  to_le_int32(payload+i, encoder[1]); i+=4;
  to_le_int32(payload+i, encoder[2]); i+=4;

  to_le_float(payload+i, vel[0]); i+=4;
  to_le_float(payload+i, vel[1]); i+=4;
  to_le_float(payload+i, vel[2]); i+=4;

  to_le_float(payload+i, curr[0]); i+=4;
  to_le_float(payload+i, curr[1]); i+=4;
  to_le_float(payload+i, curr[2]); i+=4;

  payload[i] = fault->aux_flags; i++;

  send_packet(0x001, ts, payload, i);
}

void send_heartbeat(const FaultResult* fault)
{
    uint8_t payload[3];
    uint16_t i = 0;

    payload[i++] = fault->state;
    payload[i++] = heartbeat_count++;
    payload[i++] = fault->aux_flags;

    send_packet(0x002, millis(), payload, i);
}

void send_ack_response(uint16_t ack_seq)
{
    uint8_t payload[3];
    uint16_t i = 0;

    to_le16(payload, ack_seq); i += 2;
    payload[i++] = 0x00;

    send_packet(0x003, millis(), payload, i);
}

void send_nack_response(uint16_t nack_seq, uint8_t nack_status)
{
    uint8_t payload[3];
    uint16_t i = 0;

    to_le16(payload, nack_seq); i += 2;
    payload[i++] = nack_status;

    send_packet(0x004, millis(), payload, i);
}

void send_fault_report(const FaultResult* fault)
{
    uint8_t payload[3];
    uint16_t i = 0;
    
    payload[i++] = fault->aux_flags;
    to_le16(payload + i, fault->motor_id); i += 2;
    
    send_packet(0x005, millis(), payload, i);
}

void send_time_sync_response(uint32_t t0, uint32_t t1)
{
    uint8_t payload[12];
    uint16_t i = 0;

    to_le32(payload + i, t0); i+= 4;
    to_le32(payload + i, t1); i+= 4;
    to_le32(payload + i, millis()); i+= 4;

    send_packet(0x020, millis(), payload, i);
}



bool find_sync(Stream& ser)
{
    bool state = 0;

    while (ser.available())
    {
      uint8_t v = ser.read();

      if (!state)
      {
        if (v == (SYNC_WORD & 0xFF))
          state = 1;
      }
      else
      {
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
    uint16_t i = 0;
    while (i < n)
    {
      if (ser.available())
        buf[i++] = ser.read();
    }
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

void parse_packet(Stream& ser, const FaultResult* fault)
{
    uint8_t primary[PRIMARY_SIZE_B];
    uint8_t secondary[SECONDARY_SIZE_B];
    uint8_t crc_buf[CRC_SIZE_B];
    uint8_t payload[512];

    // SYNC
    if (!find_sync(ser))
        return;
    
    // Primary header
    read_exact(ser, primary, PRIMARY_SIZE_B);

    uint16_t packet_info = from_le16(primary);
    uint16_t seq_ctrl    = from_le16(primary + 2);
    uint16_t data_length = from_le16(primary + 4) + 1;

    uint8_t pkt_version  = (packet_info >> 13) & 0x7;
    uint8_t packet_type  = (packet_info >> 12) & 0x1;
    uint8_t sec_hdr_flag = (packet_info >> 11) & 0x1;
    uint16_t apid        = packet_info & 0x7FF;

    uint16_t seq_count   = seq_ctrl & 0x3FFF;

    if (pkt_version != CCSDS_VERSION || sec_hdr_flag != 1)
    {
      send_nack_response(seq_count, NACK_INVALID_VERSION);
      return;
    }
      
    if (packet_type != 1)
    {
      send_nack_response(seq_count, NACK_INVALID_TYPE);
      return;
    }

    if (prev_rx_seq_count == 0xFFFF){
      prev_rx_seq_count = seq_count;
    }
    else{
      uint16_t diff = (seq_count - prev_rx_seq_count) & 0x3FFF;
      if (diff == 0 || diff >= 8192){
        send_nack_response(seq_count, NACK_SEQ_STALE); 
        return;
      }
      prev_rx_seq_count = seq_count;
    }

    // Secondary header
    read_exact(ser, secondary, SECONDARY_SIZE_B);

    uint32_t ts = from_le32(secondary);
    uint8_t src_teensy_id = secondary[4];

    if (src_teensy_id != TEENSY_ID)
    {
      send_nack_response(seq_count, NACK_INVALID_TEENSY_ID);
    }

    // Payload
    uint16_t payload_len = data_length - SECONDARY_SIZE_B - CRC_SIZE_B;

    read_exact(ser, payload, payload_len);

    // CRC
    read_exact(ser, crc_buf, CRC_SIZE_B);
    
    uint16_t crc_rx = from_le16(crc_buf);

    uint8_t temp[PRIMARY_SIZE_B + SECONDARY_SIZE_B + payload_len];
    memcpy(temp, primary, PRIMARY_SIZE_B);
    memcpy(temp + PRIMARY_SIZE_B, secondary, SECONDARY_SIZE_B);
    memcpy(temp + PRIMARY_SIZE_B + SECONDARY_SIZE_B, payload, payload_len);
    
    uint16_t crc_calc = crc16_ccitt(temp, PRIMARY_SIZE_B + SECONDARY_SIZE_B + payload_len);

    if (crc_rx != crc_calc)
    {
      send_nack_response(seq_count, NACK_CRC_MISMATCH);
      return;
    }

    // Parse payload
    switch (apid){
      case 0x010:{
        if (!check_state(fault, seq_count))
          return;
        MotorVelServoCmd cmd;
        parse_vel_servo_cmd(payload, &cmd);
        break;
      }
      case 0x011:{
        if (!check_state(fault, seq_count))
          return;
        MotorPosServoCmd cmd;
        parse_pos_servo_cmd(payload, &cmd);
        break;
      } 
      case 0x012:{
        uint8_t cmd = payload[0];
        break;
      }
      case 0x020:{
        uint32_t t0 = from_le32(payload);
        send_time_sync_response(t0, millis());
      }
      default:{
        send_nack_response(seq_count, NACK_UNKNOWN_APID);
        return;
        break;
      }
    }
}

void parse_vel_servo_cmd(const uint8_t* payload, MotorVelServoCmd* cmd)
{
  uint16_t i = 0;
  cmd->vel[0] = from_le_float(payload + i); i+=4;
  cmd->vel[1] = from_le_float(payload + i); i+=4;
  cmd->vel[2] = from_le_float(payload + i); i+=4;

  cmd->servo_pos[0] = from_le_int32(payload + i); i+=4;
  cmd->servo_pos[1] = from_le_int32(payload + i); i+=4;
  cmd->servo_pos[2] = from_le_int32(payload + i); i+=4;

  //  add out of range conditions, cmd execution, and ack response
}

void parse_pos_servo_cmd(const uint8_t* payload, MotorPosServoCmd* cmd)
{
  uint16_t i = 0;
  cmd->pos[0] = from_le_int32(payload + i); i+=4;
  cmd->pos[1] = from_le_int32(payload + i); i+=4;
  cmd->pos[2] = from_le_int32(payload + i); i+=4;

  cmd->servo_pos[0] = from_le_int32(payload + i); i+=4;
  cmd->servo_pos[1] = from_le_int32(payload + i); i+=4;
  cmd->servo_pos[2] = from_le_int32(payload + i); i+=4;

  //  add out of range conditions, cmd execution, and ack response
}

bool check_state(const FaultResult* fault, const uint8_t seq_count){
  uint8_t aux_flags = fault->aux_flags;

  if ((aux_flags & AUX_ESTOP_ACTIVE) >> 7){
    send_nack_response(seq_count, NACK_ESTOP_ACTIVE);
    return false;
    }
  if ((aux_flags & AUX_STALL_DETECTED) >> 3){
    send_nack_response(seq_count, NACK_STALL_DETECTED);
    return false;
  }
  if ((aux_flags & AUX_OVERCURRENT) >> 2){
    send_nack_response(seq_count, NACK_OVERCURRENT);
    return false;
  }
  if ((aux_flags & AUX_ENCODER_FAULT) >> 3){
    send_nack_response(seq_count, NACK_ENCODER_FAULT);
    return false;
  }
  return true;
}





MotorController motor(ENCA, ENCB, PWM, IN1, IN2, 0x40);

uint32_t loop_start_time = millis();
uint32_t prev_tdaq_time = 0;
uint32_t prev_hb_time = 0;
uint8_t prev_state = 0;

void setup(){
  Serial.begin(115200);
  motor.begin();
  motor.setTargetVelocity(-10);
}

void loop(){
  uint32_t loop_time = millis() - loop_start_time;
  loop_start_time = millis();
  
  //  daq_time = millis();
  
  //  get 3 encoder reading
  //  get 3 current reading
  //  calculate 3 velocity
  //  get 3 pwm
  
  FaultResult fault = get_fault_report(vel, curr, pwm, loop_time);
  if (((fault->state == 2) && (prev_state != 2)) || ((fault->state == 3) && (prev_state != 3))){
    send_fault_report(&fault);
  }

  if(fault->state == 2){
  //  stop_all_motors
  }

  if (daq_time - prev_tdaq_time >= TELEMETRY_TIMEPERIOD){
    send_telemetry(ts, encoder, vel, curr, &fault);
    prev_tdaq_time = daq_time;
  }
  if (millis() - prev_hb_time >= HEARTBEAT_TIMEPERIOD){
    send_heartbeat(&fault);
    prev_hb_time = millis();
  }

  if (ser.available()){
    parse_packet(ser, &fault);
  }
}