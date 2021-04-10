/* Sketch to control the AK80-9 by Tmotor. Code from Ben Katz (HKC_MiniCheetah)
 * Uses an Arduino Nano 33 BLE Sense and Seeed studios Serial-CAN transciever
 * t_ret = kp(p_des-p_real) + kd(V_des-V_real) + t_in
 */
#include"BTFuncs.h"
#include"MotorController.h"
#include"Globals.h"

//Declare Global Motor Instance
MotorController motor = MotorController();

//Time keeping
unsigned long int startMillis;
unsigned long int currentMillis;

float kp = KP_DEF;
float kd = KD_DEF;
float pos = 0;
float vel = 0;
float tor = 0;
unsigned long int using_id = LEFT_MOTOR_ID;
bool enable = false;

void modify_pos();
void modify_vel();
void modify_tor();

void setup() {
  Serial.begin(115200);
  delay(500);
  motor.init();
  motor.enableMotor(using_id,enable);
  //setupBLE();
  startMillis = millis();
}


void loop() {
  //BLE.poll();
  currentMillis = millis();
  if (currentMillis - startMillis > WAIT_TIME) {
    //Serial.println("Getting data from motor!");
    //motor.handleReply(true);
    startMillis = currentMillis;
  }
  unsigned int command = 0;
  if (Serial.available()) {
    command = Serial.read();
    switch (command) {
      case 'k':
        collect_constants();
        break;
      case 'p':
        Serial.println("Changing Postion!");
        modify_pos();
        break;
      case 'v':
        Serial.println("Changing Velocity!");
        modify_vel();
        break;
      case 't':
        Serial.println("Changing Torque!");
        modify_tor();
        break;
      case 'o':
        Serial.println("Setting Zero...");
        motor.setZero(using_id);
        break;
      case 'M':
        Serial.println("Changing enable state!");
        enable = !enable;
        motor.enableMotor(using_id, enable);
        break;
      case 'u':
        Serial.println("Updating...");
        motor.send_command(using_id, pos, vel, kp, kd, tor);
        break;
       
    }
    Serial.flush(); 
  }
}

void collect_constants() {
  Serial.println("Collecting Constants.");
  bool done = false;
  bool got_kp = false;
  Serial.flush();
  
  while (!done) {
    while (Serial.available()) {
      float data = Serial.read();
      if (got_kp) {
        kd = data;
        Serial.print("Kd: ");
        Serial.println(kd);
        done = true;
        motor.send_command(using_id, pos, vel, kp, kd, tor);
      } else {
        kp = data;
        got_kp = true;
        Serial.print("Kp: ");
        Serial.println(kp);
      }
    } 
  }
}

void modify_pos() {
  bool done = false;
  while (!done) {
    if (Serial.available()) {
      int data = Serial.read();
      if (char(data) == '~') {
        Serial.println("Exiting...");
        done = true;
        return; 
      }
      if (char(data) == '+') {
        Serial.print("Adding...");
        pos += 1;
        motor.send_command(using_id, pos, vel, kp, kd, tor);
        Serial.println("Command sent!");
      }
      if (char(data) == '-') {
        Serial.println("Subtracting...");
        pos -= 1;
        motor.send_command(using_id, pos, vel, kp, kd, tor);
        Serial.println("Command sent!");
      }
    }
  }
}

void modify_vel() {
  bool done = false;
  while (!done) {
    if (Serial.available()) {
      int data = Serial.read();
      if (char(data) == '~') {
        Serial.println("Exiting...");
        done = true;
        return; 
      }
      if (char(data) == '+') {
        Serial.print("Adding...");
        vel += 10;
        motor.send_command(using_id, pos, vel, kp, kd, tor);
        Serial.println("Command sent!");
      }
      if (char(data) == '-') {
        Serial.println("Subtracting...");
        vel -= 10;
        motor.send_command(using_id, pos, vel, kp, kd, tor);
        Serial.println("Command sent!");
      }
    }
  }
}

void modify_tor() {
  bool done = false;
  while (!done) {
    if (Serial.available()) {
      int data = Serial.read();
      if (char(data) == '~') {
        Serial.println("Exiting...");
        done = true;
        return; 
      }
      if (char(data) == '+') {
        Serial.print("Adding...");
        tor += 1;
        motor.send_command(using_id, pos, vel, kp, kd, tor);
        Serial.println("Command sent!");
      }
      if (char(data) == '-') {
        Serial.println("Subtracting...");
        tor -= 1;
        motor.send_command(using_id, pos, vel, kp, kd, tor);
        Serial.println("Command sent!");
      }
    }
  }
}

void modify_x(int x) {
  bool done = false;
  while (!done) {
    if (Serial.available()) {
      int data = Serial.read();
      if (char(data) == '~') {
        Serial.println("Exiting...");
        done = true;
        return; 
      }
      if (char(data) == '+') {
        Serial.print("Adding...");
        switch (x) {
        case 1:
          pos += 1;
        case 2:
          vel += 1;
        case 3:
          tor += 1;
        }
        motor.send_command(using_id, pos, vel, kp, kd, tor);
        Serial.println("Command sent!");
      }
      if (char(data) == '-') {
        Serial.println("Subtracting...");
        switch (x) {
        case 1:
          pos -= 1;
        case 2:
          vel -= 1;
        case 3:
          tor -= 1;
        }
        motor.send_command(using_id, pos, vel, kp, kd, tor);
        Serial.println("Command sent!");
      }
    }
  }
}


  /*
  void EnterMotorMode()
  {
    unsigned char enable_data[8] = {0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFC};
    can.send(can_id, 0, 0, 8, enable_data);
    motor_on = true;
  }
  void ExitMotorMode()
  {
    unsigned char disable_data[8] = {0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFD};
    can.send(can_id, 0, 0, 8, disable_data);
    motor_on = false;
  }
  void setZero()
  {
    unsigned char zero_data[8] = {0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFE};
    can.send(can_id, 0, 0, 8, zero_data);
  }
  void sendZero()
  {
    if (!motor_on) 
    {
      EnterMotorMode();
    }
    p_in = 0;
    pack_cmd();
  }
  
  bool pos_ctl(float pos, float kp, float kd)
  {
    if (kp<=0 || kd<0)
    {
      return false;
    }
    p_in = pos;
    v_in = 0;
    kp_in = kp;
    kd_in = kd;
    t_in = 0;
    pack_cmd();
    return true;
  }
  bool vel_ctrl(float vel, float kd)
  {
    if (kd<=0)
    {
      return false;
    }
    v_in = vel;
    kp_in = 0;
    kd_in = kd;
    t_in = 0;
    pack_cmd();
    return true;
  }
  bool trq_ctrl(float tor, float kd)
  {
    if (kd<=0)
    {
      return false;
    }
    kp_in = 0;
    kd_in = kd;
    t_in = tor;
    pack_cmd();
  }












  
 void pack_cmd() {
  byte buf[8];
  //Bound input data
  float p_des = constrain(p_in, P_MIN, P_MAX);
  float v_des = constrain(v_in, V_MIN, V_MAX);
  float kp = constrain(kp_in, KP_MIN, KP_MAX);
  float kd = constrain(kd_in, KD_MIN, KD_MAX);
  float t_ff = constrain(t_in, T_MIN, T_MAX);
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
  can.send(can_id, 0, 0, 8, buf);
 }
 void unpack_reply() {
  //Unpack global buffer
  unsigned int id = data[0];
  unsigned int p_int = (data[1] << 8) | data[2];
  unsigned int v_int = (data[3] << 4) | (data[4] >> 4);
  unsigned int i_int = ((data[4] & 0xF) << 8) | data[5];
  //Convert to floats
  p_out = uint_to_float(p_int, P_MIN, P_MAX, 16);
  v_out = uint_to_float(v_int, V_MIN, V_MAX, 12);
  t_out = uint_to_float(i_int, -T_MAX, T_MAX, 12);
 }

//Type conversion funcs
 float float_to_uint(float x, float x_min, float x_max, int bits){
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
 float uint_to_float(unsigned int x_int, float x_min, float x_max, int bits) {
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
 */
