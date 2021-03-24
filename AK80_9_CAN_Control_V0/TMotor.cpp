#include "Serial_CAN_Nano.h"
#include "TMotor.h"

TMotor::TMotor(int id) {
  this->id = id;
  sender = Serial_CAN();
  sender.begin(57600);
}

float TMotor::uint_to_float(unsigned int x_int, float x_min, float x_max, int bits) {
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

void TMotor::unpack_reply() {
  //Unpack buffer
  unsigned int ID = data[0];
  unsigned int p_int = (data[1] << 8) | data[2];
  unsigned int v_int = (data[3] << 4) | (data[4] >> 4);
  unsigned int i_int = ((data[4] & 0xF) << 8) | data[5];
  //Convert to floats
  this->posFrom = uint_to_float(p_int, P_MIN, P_MAX, 16);
  this->velFrom = uint_to_float(v_int, V_MIN, V_MAX, 12);
  this->torFrom = uint_to_float(i_int, -T_MAX, T_MAX, 12);
}

void TMotor::handleReply() {
  if (sender.recv(id, data) == '1') {
    unpack_reply();
  }
}

void TMotor::enableMotor(bool zero = false) {
  if (!zero) {
    //Update the current position with the actual motor position
    unpack_reply();
    posTo = posFrom
  }
  unsigned char enable_data[8] = {0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFC};
  sender.send(id, 0, 0, 8, enable_data);
  enabled = true;
}

void TMotor::disableMotor() {
  unsigned char disable_data[8] = {0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFD};
    can.send(can_id, 0, 0, 8, disable_data);
    enabled = false;
}
