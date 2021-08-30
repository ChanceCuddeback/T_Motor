#ifndef MOTOR_H
#define MOTOR_H

#include "Serial_CAN_Nano.h"
#include "SPI_CAN_Nano.h"

//Motor max/mins
#define P_MIN -12.5f
#define P_MAX 12.5f
#define V_MIN -25.0f //-65.0f
#define V_MAX 500.0f  //500.0f
#define KP_MIN 0.0f
#define KP_MAX 500.0f
#define KD_MAX 5.0f
#define KD_MIN 0.0f
#define T_MIN -18.0f
#define T_MAX 18.0f

//Weigth Defaults
#define KP_DEF 0.01f
#define KD_DEF 0.01f


#define CAN_RATE 57600
#define N_MOTORS 2

class MotorController {
  public:
    void    init();//Starts CAN Comms
    void    enableMotor(unsigned long int, bool);
    void    setZero(unsigned long int id);
    void    send_command(unsigned long int id, float p_des, float v_des, float kp, float kd, float t_des);
    void    recv_command(unsigned long int id, float* buffer);  //Filters recv search for the id, packs buffer with returns
    
  private:
    void    unpack_reply(bool);
    float   float_to_uint(float, float, float, int);
    float   uint_to_float(unsigned int, float, float, int);
    Serial_CAN sender;
};

#endif
