#ifndef TMOTORHEADER
#define TMOTORHEADER
#include "Serial_CAN_Nano.h"

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

class TMotor {
  public:
    //Constructor
    TMotor(int);
    //Methods
    void    enableMotor(bool);
    void    disableMotor();
    void    changeMode(int);
    void    setpoint(double);
    void    setZero();
    void    tick();
    void    update();
    void    handleReply();
    //Getters/Setters
    double  pos();
    double  vel();
    double  tor();
    bool    isEnabled();
  private:
    //Methods
    void    pack_cmd();
    void    unpack_reply();
    float   float_to_uint(float, float, float, int);
    float   uint_to_float(unsigned int, float, float, int);
    //Properties
    int id;
    int kp, kd;
    bool enabled = false;
    Serial_CAN sender;
    //Buffer for CAN
    unsigned char data[8] = {1,2,3,4,5,6,7,8};
    double posTo, velTo, torTo;
    double posFrom, velFrom, torFrom;
};

#endif
