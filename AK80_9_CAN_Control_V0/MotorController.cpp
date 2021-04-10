#include "MotorController.h"

//Public interface
void MotorController::init() {
  sender = Serial_CAN();
  sender.begin(CAN_RATE);
}

void MotorController::enableMotor(unsigned long int id, bool enable) {
  if (enable) {
  unsigned char enable_data[8] = {0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFC};
  sender.send(id, 0, 0, 8, enable_data);
  } else {
    unsigned char disable_data[8] = {0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFD};
    sender.send(id, 0, 0, 8, disable_data);
  }
}

void  MotorController::setZero(unsigned long int id) {
  unsigned char zero_data[8] = {0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFE};
  sender.send(id, 0, 0, 8, zero_data);
}

void MotorController::send_command(unsigned long int id, float p_des, float v_des, float kp, float kd, float t_des) {
  Serial.println("Sending command with: ");
  Serial.println(p_des);
  Serial.println(v_des);
  Serial.println(kp);
  Serial.println(kd);
  Serial.println(t_des);
  
  byte buf[8];
  float p_sat = constrain(p_des, P_MIN, P_MAX);
  float v_sat = constrain(v_des, V_MIN, V_MAX);
  float kp_sat = constrain(kp, KP_MIN, KP_MAX);
  float kd_sat = constrain(kd, KD_MIN, KD_MAX);
  float t_sat = constrain(t_des, T_MIN, T_MAX);
  //Convert floats to unsigned ints
  unsigned int p_int = float_to_uint(p_sat, P_MIN, P_MAX, 16);
  unsigned int v_int = float_to_uint(v_sat, V_MIN, V_MAX, 12);
  unsigned int kp_int = float_to_uint(kp_sat, KP_MIN, KP_MAX, 12);
  unsigned int kd_int = float_to_uint(kd_sat, KD_MIN, KD_MAX, 12);
  unsigned int t_int = float_to_uint(t_sat, T_MIN, T_MAX, 12);
  //pack ints into the can buffer
  buf[0] = p_int >> 8;
  buf[1] = p_int & 0xFF;
  buf[2] = v_int >> 4;
  buf[3] = ((v_int & 0xF) << 4) | (kp_int >> 8);
  buf[4] = kp_int &0xFF;
  buf[5] = kd_int >> 4;
  buf[6] = ((kd_int & 0xF) << 4) | (t_int >> 8);
  buf[7] = t_int & 0xFF;
  //Send CAN message
  sender.send(id, 0, 0, 8, buf);
}

void MotorController::recv_command(unsigned long int id, float* buffer) {
  unsigned long int recv_id;
  bool found{false};
  unsigned char data[8];
  for (int i=0;i<=N_MOTORS;i++) {
    if (!found && (sender.recv(&recv_id, data)=='1')) {
      if (recv_id == id) {
        found = true;
      }
    }
  }
  if (sender.recv(&recv_id, data) == '1') {
    if (recv_id == id) {
      //Unpack buffer
      long unsigned int ID = data[0];
      unsigned int p_int = (data[1] << 8) | data[2];
      unsigned int v_int = (data[3] << 4) | (data[4] >> 4);
      unsigned int i_int = ((data[4] & 0xF) << 8) | data[5];
    
      //Convert to floats
      *(buffer) = uint_to_float(p_int, P_MIN, P_MAX, 16);
      *(buffer + 1) = uint_to_float(v_int, V_MIN, V_MAX, 12);
      *(buffer + 2) = uint_to_float(i_int, -T_MAX, T_MAX, 12);
    }
  }
}

//===================================
//Private
float MotorController::float_to_uint(float x, float x_min, float x_max, int bits) {
  float span = x_max - x_min;
  float offset = x_min;
  unsigned int pgg = 0;
  if (bits == 12) {
    pgg = (unsigned int) ((x-offset)*4095.0/span); 
  }
  if (bits == 16) {
    pgg = (unsigned int) ((x-offset)*65535.0/span);
  }
  return pgg;
}

float MotorController::uint_to_float(unsigned int x_int, float x_min, float x_max, int bits) {
  float span = x_max - x_min;
  float offset = x_min;
  float pgg = 0;
  if (bits == 12) {
    pgg = ((float)x_int)*span/4095.0 + offset;
  }
  if (bits == 16) {
    pgg = ((float)x_int)*span/65535.0 + offset;
  }
  return pgg;
 }
//==================================
