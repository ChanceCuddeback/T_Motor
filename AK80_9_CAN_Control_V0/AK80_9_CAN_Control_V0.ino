/* Sketch to control the AK80-9 by Tmotor. Code from Ben Katz (HKC_MiniCheetah)
 * Uses an Arduino Nano 33 BLE Sense and Seeed studios Serial-CAN transciever
 * t_ret = kp(p_des-p_real) + kd(V_des-V_real) + t_in
 */
 
#include"BTFuncs.h"
#include"TMotor.h"

#define WAIT_TIME 50
const long unsigned int LEFT_MOTOR_ID = 1;
const long unsigned int RIGHT_MOTOR_ID = 2;

//Global Vars to interface bluetooth and motor instances
int bt_mode =       0;    //0: Position, 1: Velocity, 2: Torque
int bt_motor =      0;    //0: Left, 1: Right 
bool bt_enable =    0;
float bt_setpoint = 0;


//Time keeping
unsigned long int startMillis;
unsigned long int currentMillis;

//Declare Global Motor Instance(s)
TMotor left_motor = TMotor();
//TMotor right_motor = TMotor();


void setup() {
  Serial.begin(115200);
  while (!Serial) {};
  delay(1000);
  //Serial.println("Begin!");
  left_motor.init(LEFT_MOTOR_ID, true);
  setupBLE();
  //Serial.println("Done!");
  startMillis = millis();
}


void loop() {
  BLE.poll();
  currentMillis = millis();
  if (currentMillis - startMillis > WAIT_TIME) {
    //Serial.println("Getting data from motor!");
    left_motor.handleReply(true);
    startMillis = currentMillis;
    //Serial.println("Done getting Data!");
  }
  /*
  while (Serial.available()) {
    command = Serial.read();
    float p_step = 0.01;
    switch (command) {
      case '1':
        p_in = p_in + p_step;
        Serial.println("Increase Step.");
        break;
      case '2':
        Serial.println("Beware of Torque!");
        delay(1000);
        t_in = 5;
        pack_cmd();
        delay(1000);
        t_in = 0;
        pack_cmd();
        break;
      case '3':
        EnterMotorMode();
        Serial.println("Began Motor Mode!");
        break;
      case '4':
        ExitMotorMode();
        Serial.println("Stopped Motor Mode!");
        break;
      case '5':
        setZero();
        Serial.println("Zero Motor");
        break;
      case '6':
        pack_cmd();
        Serial.println("Sent Command");
        break;
      case '7':
        if (motor_on) 
        {
          Serial.println("Start Pos Ramp Sequence!");
          for (int i=0;i<85;i++)
          {
            p_in=p_in + p_step;
            pack_cmd();
            Serial.print("p_in: ");
            Serial.println(p_in);
            Serial.print("i: ");
            Serial.println(i);
            delay(1);
          }
        }
        else 
        {
          Serial.println("Motor must be on!");
        }
        break;
      case '8':
        sendZero();
        break;
    } 
  }
  if (motor_on) 
  {
    pack_cmd();
  }
  
  
  if (can.recv(&can_id, data) == '1') {
    unpack_reply();
  }
  */
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
