#include "TMotor.h"

//Public interface
void TMotor::init(unsigned long int id, bool zero) {
  this->id = id;
  sender = Serial_CAN();
  sender.begin(CAN_RATE);
  if (zero) {
    this->enableMotor(true);
    this->setZero();
  }
  this->disableMotor();
}
void TMotor::handleReply(bool prin) {
  if (sender.recv(&id, data) == 1) {
    //Serial.println("Unpacking Reply!");
    this->unpack_reply(prin);
  }
}

void TMotor::enableMotor(bool zero = false) {
  if (!zero) {
    //Update the current position with the actual motor position
    unpack_reply(false);
    posTo = posFrom;
  }
  unsigned char enable_data[8] = {0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFC};
  sender.send(id, 0, 0, 8, enable_data);
  enabled = true;
}

void TMotor::disableMotor() {
  unsigned char disable_data[8] = {0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFD};
  sender.send(id, 0, 0, 8, disable_data);
  enabled = false;
}

void TMotor::changeMode(int modeL) {
  this->mode = modeL;
  switch (mode) { //Change kp, kd
    case (0): //Pos
      kp = KP_DEF;
      kd = KD_DEF;
      break;
    case (1): //Vel
      kp = 0.0001;
      kd = KD_DEF;
      break;
    case (2): //Tor
      kp = 0.0001;
      kd = KD_DEF;
      break;
    default:
      //Serial.println("Unknown Mode!");
      break;
  }
}

void TMotor::setpoint(float point) {
  switch (this->mode) { //Check the mode then assign a setpoint
    case (0):
      posTo = point;
      break;
    case (1):
      velTo = point;
      break;
    case (2):
      torTo = point;
      break;
    default:
      break;
      //Serial.println("Unknown Mode!");
  }
  pack_cmd();
}

void  TMotor::setZero() {
  unsigned char zero_data[8] = {0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFE};
  sender.send(id, 0, 0, 8, zero_data);
}

void  TMotor::tick() {
  
}

void  TMotor::update() {
  pack_cmd();
  unpack_reply(false);
}


float TMotor::pos() {
  return this->posTo;
}

float TMotor::vel() {
  return this->velTo;
}

float TMotor::tor() {
  return this->torTo;
}

bool TMotor::isEnabled() {
  return this->enabled;
}
//===================================
//Private
void TMotor::pack_cmd() {
  byte buf[8];
  //Bound input data
  float p_des = constrain(posTo, P_MIN, P_MAX);
  float v_des = constrain(velTo, V_MIN, V_MAX);
  float kp = constrain(kp, KP_MIN, KP_MAX);
  float kd = constrain(kd, KD_MIN, KD_MAX);
  float t_ff = constrain(torTo, T_MIN, T_MAX);
  //Convert floats to unsigned ints
  unsigned int p_int = float_to_uint(p_des, P_MIN, P_MAX, 16);
  unsigned int v_int = float_to_uint(v_des, V_MIN, V_MAX, 12);
  unsigned int kp_int = float_to_uint(kp, KP_MIN, KP_MAX, 12);
  unsigned int kd_int = float_to_uint(kd, KD_MIN, KD_MAX, 12);
  unsigned int t_int = float_to_uint(t_ff, T_MIN, T_MAX, 12);
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

float TMotor::float_to_uint(float x, float x_min, float x_max, int bits) {
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

void TMotor::unpack_reply(bool prin) {
  //Unpack buffer
  unsigned int ID = data[0];
  unsigned int p_int = (data[1] << 8) | data[2];
  unsigned int v_int = (data[3] << 4) | (data[4] >> 4);
  unsigned int i_int = ((data[4] & 0xF) << 8) | data[5];
  //Serial.println("Got data!");
  //Convert to floats
  this->posFrom = uint_to_float(p_int, P_MIN, P_MAX, 16);
  this->velFrom = uint_to_float(v_int, V_MIN, V_MAX, 12);
  this->torFrom = uint_to_float(i_int, -T_MAX, T_MAX, 12);
  //Serial.println("Converted data!");
  if (prin) {
    Serial.print("Position: ");
    Serial.println(this->posFrom);
    Serial.print("Velocity: ");
    Serial.println(this->velFrom);
    Serial.print("Torque: ");
    Serial.println(this->torFrom);
    //Serial.println("Done Printing!");
  }
}
//==================================
