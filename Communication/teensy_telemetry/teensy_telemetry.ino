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
 *              0x04 - OVERCURRENT_DETECTED
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
 *              0x10 - UNKNOWN_APID
 *              0x11 - INVALID_LENGTH
 *              0x12 - CRC_MISMATCH
 *              0x13 - SEQ_STALE
 *              0x20 - VALUE_OUT_OF_RANGE
 *              0x30 - MOTORS_DISABLED
 *              0x31 - E_STOP_ACTIVE
 *              0x32 - STALL_DETECTED
 *              0x33 - OVERCURRENT
 *              0x40 - ENCODER_FAULT
 *              0x50 - INTERNAL_ERROR
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

#define SYNC_WORD            0xAA55
#define CCSDS_VERSION        0
#define CCSDS_TYPE           0  // Telemetry
#define CCSDS_SECHDR_FLAG    1
#define CCSDS_SEQ_FLAG       3  // 0b11 (unsegmented data)

#define SYNC_SIZE_B          2
#define PRIMARY_SIZE_B       6
#define SECONDARY_SIZE_B     5
#define CRC_SIZE_B           2

static uint16_t seq_counter = 0;

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



void send_packet(uint16_t apid, uint8_t teensy_id, uint32_t ts, const uint8_t* payload, uint16_t payload_length){
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
  packet[i] = teensy_id; i+=1;

  // Payload
  memcpy(packet+i, payload, payload_length); i+= payload_length;

  // CRC
  uint16_t crc = crc16_ccitt(packet+SYNC_SIZE_B, PRIMARY_SIZE_B + SECONDARY_SIZE_B + payload_length);
  to_le16(packet+i, crc); i+=2;

  Serial.write(packet, i);
}



void send_telemetry_packet(uint8_t teensy_id, uint32_t ts, long encoder[3], float vel[3], float curr[3], uint8_t aux_flags){
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

  payload[i] = aux_flags; i++;

  send_packet(0x001, teensy_id, ts, payload, i);
}



void setup(){
  
}

void loop(){
  
}
