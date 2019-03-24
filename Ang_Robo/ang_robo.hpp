/******************************************************************************
 *  ang_robo.hpp (for LEGO Mindstorms EV3)
 *****************************************************************************/

#ifndef EV3_UNIT_BALANCINGWALKER_H_
#define EV3_UNIT_BALANCINGWALKER_H_

#include "util.hpp"
#include "parameter.h"
#include "GyroSensor.h"
#include "Motor.h"
#include "BalancerCpp.h"


using namespace std;

//17.07.28 k-ota copy from 3-apex
#define P_GAIN             1 /* 完全停止用モータ制御比例係数 */
#define PWM_ABS_MAX       60 /* 完全停止用モータ制御PWM絶対最大値 */

class Ang_Robo {
public:
  static const int LOW;
  static const int NORMAL;
  static const int HIGH;

  Ang_Robo(const ev3api::GyroSensor& gyroSensor,
	   ev3api::Motor& leftWheel,
	   ev3api::Motor& rightWheel,
	   ev3api::Motor& tail_motor,
	   Balancer* balancer);

  void init();
  void setCommand(float velocity, int forward, float yawratecmd, signed int tail_ang_req, float yawrate, bool forward_curve_mode, bool tail_stand_mode, bool tail_lug_mode, bool rising_seesaw, bool falling_seesaw);
  void set_robo_mode_launch();

  void run();

  void tail_control(signed int angle); //2017.07.28 kota copy from 3-apex
  //170816 ota add tail control
  void tail_reset();
  void tail_stand_up(); //tail for gyro reset and color sensor calibration
  void tail_lug();
  
  void tail_stand_from_balance();
  
  int   offset;    
  bool  balance_mode;
  bool  lug_mode;

  int   log_forward;
  int   log_turn;
  int   log_gyro;
  int   log_left_wheel_enc;
  int   log_right_wheel_enc;
  int   log_battery;
  int   log_left_pwm;
  int   log_right_pwm;
  int   log_robo_mode;
  int   log_stand_mode;

private:
  const ev3api::GyroSensor& mGyroSensor;
  ev3api::Motor& mLeftWheel;
  ev3api::Motor& mRightWheel;
  ev3api::Motor& mTail_Motor;
  Balancer* mBalancer;
  PID *gTail_pwm = new PID();
  PID *gForward  = new PID();

  void forward_curve_gen();
  void velocity_ctl();

  enum Robo_Mode{
    SET,
    READY,
    LAUNCH,
    LAUNCH_BALANCE,
    BALANCE_RUN,
    BALANCE_TO_TAIL,
    TAIL_RUN,
    LUG_RUN,
    TAIL_TO_BALANCE,
    ROBO_DEBUG
  };

  enum Stand_Mode{
    BALANCE,
    TAIL_DOWN,
    TAIL_ON,
    TAIL_STAND,
    TAIL_LUG,
    LUG_TO_STAND,
    STAND_VERT,
    STAND_TO_BALANCE,
    TAIL_FOR_RUN,
    DEBUG
  };

  Robo_Mode  ROBO_MODE;
  Stand_Mode STAND_MODE;

  float mVelocity        = 0.0;
  int   mForward;
  float mTurn;
  float mYawratecmd;//目標Yawrate
  float mYawrate;

  int   mTarget_forward  = 0;
  float mCurved_forward  = 0;
  int   mCurrent_forward = 0;

  int   mRef_velocity  = 0;


  bool  mForward_curve_mode;
  bool  mTail_stand_mode;
  bool  mTail_lug_mode;
  bool  mRising_seesaw;
  bool  mFalling_seesaw;
   
  int omega                  = 0;
  int right_wheel_enc        = 0;
  int left_wheel_enc         = 0;
  signed int mTail_ang_req;
  float      tail_motor_pwm;

  float YawrateController(float yawrate, float yawrate_cmd);
  float yaw_ctl_dt = 0.004;

  float turn_tmp   = 0.0;
  float r_yaw_rate = 0.0;
  float r_yaw_rate_ud = 0.0;
  float I_gain1 = 1.0+1.0/0.8;
  float I_gain2 = yaw_ctl_dt/0.06/0.8;
  float I_gain3 = 1.0-yaw_ctl_dt/0.06;

  float F_controller(float r_yaw_rate);
  float F_in = 0.0;
  float F_out = 0.0;
  //    float F_gain = 1/0.062;
  float F_gain = 1/0.062; //0818 tada
    
  float E_controller(float r_yaw_rate);
  float E_in;
  float E_in_d;
  float E_in_dd;
  float E_in_ddd;
  float E_in_dddd;
  float E_in_ddddd = 0.0;
  float E_in_dddddd = 0.0;

  float E_out = 0.0;
  float E_gain1 = yaw_ctl_dt/0.1;
  float E_gain2 = 1.0-yaw_ctl_dt/0.1*1.0;
  float E_ud1 = 0.0;

  float C_controller(float E_out, float yawrate, float S_out);
  float C_in = 0.0;
  float C_out = 0.0;
  //float C_gain = yaw_ctl_dt*10.0;
  float C_gain = yaw_ctl_dt*50.0;
  float C_ud1 = 0.0;

  float S_controller(float C_out);
  float S_gain1 = 1.0;
  float S_in;
  float S_in_d;
  float S_in_dd;
  float S_in_ddd;
  float S_in_dddd;
  float S_in_ddddd = 0.0;
  float S_in_dddddd = 0.0;
  float S_out = 0.0;
  float Pn_gain1 = yaw_ctl_dt/0.1;
  float Pn_gain2 = 1.0-yaw_ctl_dt/0.1*1.0;
  float Pn_ud1 = 0.0;
  float Pd_gain1 = yaw_ctl_dt/0.1;
  float Pd_gain2 = 1.0-yaw_ctl_dt/0.1*1.0;
  float Pd_ud1 = 0.0;

  void TailMode(int mForward, float mTurn); //PWM Gen. without Balancer task 0814
  int mtail_mode_pwm_l;
  int mtail_mode_pwm_r;

  bool balance_off_en;
  //  bool pre_balancer_on;
};

#endif  // EV3_UNIT_BALANCINGWALKER_H_
