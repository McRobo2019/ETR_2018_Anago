#ifndef MY_UNTL_H_
#define MY_UNTL_H_

#include "ev3api.h"

extern void init_f(const char *str);
extern void msg_f(const char *str, int32_t line);
extern int emergencyStop(float velocity);

class PID{
public:
  explicit PID();
  void init_pid(float kp, float ki, float kd, float dT);
  int  calc_pid(float command, float contrvalue);
  int  sat(int max, int min, int inputvalue);

private:
  float KP=0;
  float TI=0;
  float TD=0;
  float DT=1;

  int error_old=0;
  int error_P_old=0;		//過去の偏差
};

class Average_125_Data{
public:
  explicit Average_125_Data();
  void init(float init_dat);
  float average_125(float indata);



private:
  int   cap_cnt = 0;
  float dat_array[125];
  float sum_dat  = 0.0;
  float ave_dat  = 0.0;
  
};

class Average_500_Data{
public:
  explicit Average_500_Data();
  void init();
  float average_500(float indata);



private:
  int   cap_cnt = 0;
  bool  full_cnt = false;
  float dat_array[500];
  float sum_dat  = 0.0;
  float ave_dat  = 0.0;
  
};


#endif  // MY_UNTL_H_
