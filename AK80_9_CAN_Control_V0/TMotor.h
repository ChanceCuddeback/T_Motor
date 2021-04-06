#ifndef TMOTORHEADER
#define TMOTORHEADER

#include"Serial_CAN_Nano.h"

//Motor max/mins
#define P_MIN -12.5f
#define P_MAX 12.5f
#define V_MIN -25.0f //-65.0f
#define V_MAX 25.0f  //500.0f
#define KP_MIN 0.0f
#define KP_MAX 500.0f
#define KD_MAX 5.0f
#define KD_MIN 0.0f
#define T_MIN -18.0f
#define T_MAX 18.0f

//Weigth Defaults
#define KP_DEF 100.0f
#define KD_DEF 10.0f


#define CAN_RATE 57600

class TMotor {
  public:
    //Constructor
    TMotor(long unsigned int);
    //Methods
    void    enableMotor(bool);
    void    disableMotor();
    void    changeMode(int);
    void    setpoint(float);
    void    setZero();
    void    tick();
    void    update();
    void    handleReply(bool);
    //Getters/Setters
    float   pos();
    float   vel();
    float   tor();
    bool    isEnabled();
  private:
    //Methods
    void    pack_cmd();
    void    unpack_reply(bool);
    float   float_to_uint(float, float, float, int);
    float   uint_to_float(unsigned int, float, float, int);
    //Properties
    long unsigned int id;
    float kp{KP_DEF};
    float kd{KD_DEF};
    bool enabled = false;
    int mode;
    Serial_CAN sender;
    //Buffer for CAN
    unsigned char data[8] = {1,2,3,4,5,6,7,8};
    float posTo, velTo, torTo;
    float posFrom, velFrom, torFrom;
};

#endif
