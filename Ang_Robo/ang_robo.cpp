/******************************************************************************
 *  Ang_Robo.cpp (for LEGO Mindstorms EV3)
 *  Created on: 2018/04/22
 *  Implementation of the Class Ang_Robo
 *  Author: Kaoru Ota
 *  https://github.com/KaoruOta/anago2018_beta
 *****************************************************************************/

#include "ang_robo.hpp"
#include "Clock.h"

using ev3api::Clock;

Clock*       robo_Clock;


// 定数宣言
const int Ang_Robo::LOW    = 30;    // 低速
const int Ang_Robo::NORMAL = 50;    // 通常
const int Ang_Robo::HIGH   = 70;    // 高速

/**
 * コンストラクタ
 * @param gyroSensor ジャイロセンサ
 * @param leftWheel  左モータ
 * @param rightWheel 右モータ
 * @param balancer   バランサ
 */
Ang_Robo::Ang_Robo(const ev3api::GyroSensor& gyroSensor,
                                 ev3api::Motor& leftWheel,
                                 ev3api::Motor& rightWheel,
                                 ev3api::Motor& tail_motor,
                                 Balancer* balancer)
    : mGyroSensor(gyroSensor),
      mLeftWheel(leftWheel),
      mRightWheel(rightWheel),
      mTail_Motor(tail_motor),
      mBalancer(balancer),
      mForward(LOW),
      mTurn(LOW),

     tail_motor_pwm(0)
 {
}


void Ang_Robo::init() {

  offset = mGyroSensor.getAnglerVelocity();  // ジャイロセンサ値
    
  robo_Clock       = new Clock();        

  mLeftWheel.reset();
  mRightWheel.reset();


  mBalancer->init(offset);
  balance_mode = true; 
  lug_mode     = false;

  //  mCurved_forward  = 0.0; //180716 kota
  mCurrent_forward = 0; //180716 kota
  mCurved_forward  = 10.0;

  gTail_pwm->init_pid(0, 0, 0, 1); //it would be chanbed bellow 
  //  gTail_pwm->init_pid(1, 0.005, 0.001, dT_4ms);    
  gForward->init_pid(0.05,0.01,0.001,dT_4ms);

  ROBO_MODE  = SET;
  //  STAND_MODE = BALANCE;
  STAND_MODE = TAIL_STAND;
}

/**
 * PWM値を設定する
 * @param forward 前進値
 * @param turn    旋回値
 */
void Ang_Robo::setCommand(float velocity, int forward, float yawratecmd, signed int tail_ang_req, float yawrate,  bool forward_curve_mode, bool tail_stand_mode, bool tail_lug_mode, bool rising_seesaw, bool falling_seesaw) {

  mVelocity           = velocity;
  mForward            = forward;
  mTarget_forward     = forward;

  mRef_velocity       = forward;

  mYawratecmd         = yawratecmd;
  mTail_ang_req       = tail_ang_req;
  mYawrate            = yawrate;
  mForward_curve_mode = forward_curve_mode,
  mTail_stand_mode    = tail_stand_mode;
  mTail_lug_mode      = tail_lug_mode;
  mRising_seesaw      = rising_seesaw;
  mFalling_seesaw     = falling_seesaw;

}

void Ang_Robo::set_robo_mode_launch(){
  ROBO_MODE = LAUNCH;
}

void Ang_Robo::run() {
    int     battery       = ev3_battery_voltage_mV();

    switch(ROBO_MODE){

    case SET:

      break;

    case READY:

      break;


    case LAUNCH:
      log_robo_mode = 0;

      if(mTail_Motor.getCount() <= TAIL_ANGLE_BALANCE_START){
	tail_control(TAIL_ANGLE_LAUNCH);
      }else{
	ROBO_MODE = LAUNCH_BALANCE;
      }
      break;

      //---0902
    case LAUNCH_BALANCE:


      log_robo_mode   = 1;
      omega           = mGyroSensor.getAnglerVelocity();
      //      omega = omega + 5;

      right_wheel_enc = mRightWheel.getCount();
      left_wheel_enc  = mLeftWheel.getCount();             // 左モータ回転角度

      STAND_MODE = BALANCE;
      balance_off_en = false;
      balance_mode = true; 

      if(mForward_curve_mode){
	forward_curve_gen();
	mForward = mCurrent_forward;
      }else{
	mForward = mTarget_forward;
      }

      mTurn = YawrateController(mYawrate, mYawratecmd);
      mBalancer->setCommand(mForward, mTurn);
      mBalancer->update(omega, right_wheel_enc, left_wheel_enc, battery);
      mLeftWheel.setPWM(mBalancer->getPwmLeft());
      mRightWheel.setPWM(mBalancer->getPwmRight());
      tail_control(TAIL_ANGLE_RUN); //180602 kota it will be modified


      if(mVelocity > 30){
	ROBO_MODE = BALANCE_RUN;
	}


      break;

    case BALANCE_RUN:
      log_robo_mode   = 1;
      omega           = mGyroSensor.getAnglerVelocity();
      right_wheel_enc = mRightWheel.getCount();
      left_wheel_enc  = mLeftWheel.getCount();             // 左モータ回転角度

      if(mTail_stand_mode == true){
	ROBO_MODE = BALANCE_TO_TAIL;
      }

      STAND_MODE = BALANCE;
      balance_off_en = false;
      balance_mode = true; 

      if(mForward_curve_mode){
	forward_curve_gen();
	mForward = mCurrent_forward;
      }else{
	mForward = mTarget_forward;
      }


      mTurn = YawrateController(mYawrate, mYawratecmd);
      
      mBalancer->setCommand(mForward, mTurn);

      if(mRising_seesaw){
	omega = omega - 10;
      }else if(mFalling_seesaw){
	omega = omega + 25;
      }
      mBalancer->update(omega, right_wheel_enc, left_wheel_enc, battery);

      mLeftWheel.setPWM(mBalancer->getPwmLeft());
      mRightWheel.setPWM(mBalancer->getPwmRight());

      //      tail_control(TAIL_ANGLE_RUN); //180602 kota it will be modified
      tail_control(mTail_ang_req);
      break;

    case BALANCE_TO_TAIL:
      log_robo_mode = 2;
      tail_stand_from_balance();
      
      if((balance_off_en == true) && (mTail_Motor.getCount() >  70)){

	//	mLeftWheel.setPWM(100); //too much
	mLeftWheel.setPWM(10);
	//	mRightWheel.setPWM(100); //too much
	mRightWheel.setPWM(10);

	balance_mode = false; 
      }else{
	omega           = mGyroSensor.getAnglerVelocity();
	right_wheel_enc = mRightWheel.getCount();
	left_wheel_enc  = mLeftWheel.getCount();             // 左モータ回転角度

	mBalancer->setCommand(mForward, mTurn);
	mBalancer->update(omega, right_wheel_enc, left_wheel_enc, battery);

	mLeftWheel.setPWM(mBalancer->getPwmLeft());
	mRightWheel.setPWM(mBalancer->getPwmRight());
	balance_mode = true; 

      }
      break;

    case TAIL_RUN:
      log_robo_mode = 3;

      tail_stand_from_balance();

      balance_mode = false;       

      if(mTail_stand_mode == false){
	ROBO_MODE = TAIL_TO_BALANCE;
      }else if(mTail_lug_mode == true){
	STAND_MODE = TAIL_LUG;
      }else{
	STAND_MODE = TAIL_STAND;
      }

      //      tail_stand_from_balance();

      mTurn = YawrateController(mYawrate, mYawratecmd);

      TailMode(mForward, mTurn);
      mLeftWheel.setPWM(mtail_mode_pwm_l);
      mRightWheel.setPWM(mtail_mode_pwm_r);
      break;      

    case LUG_RUN:
      log_robo_mode = 4;
      
      balance_mode = false; 

      tail_stand_from_balance();

      //bellow functions will be changed later 180602 kota
      mTurn = YawrateController(mYawrate, mYawratecmd);

      TailMode(mForward, mTurn);
      mLeftWheel.setPWM(mtail_mode_pwm_l);
      mRightWheel.setPWM(mtail_mode_pwm_r);
      break;      

    case TAIL_TO_BALANCE:
      log_robo_mode = 5;


      if(STAND_MODE == TAIL_STAND){
	STAND_MODE = STAND_VERT;
      }
      tail_stand_from_balance();

      break;

    case ROBO_DEBUG:
      log_robo_mode = 6;
      break;
    }

    log_forward         = mForward;
    log_turn            = mTurn;
    log_gyro            = omega;
    log_left_wheel_enc  = left_wheel_enc;
    log_right_wheel_enc = right_wheel_enc;
    log_battery         = battery;
    log_left_pwm        = mBalancer->getPwmLeft();
    log_right_pwm       = mBalancer->getPwmRight();
}


//2017.07.28 k-ota copy from 3-apex
//*****************************************************************************
// 関数名 : tail_control
// 引数 : angle (モータ目標角度[度])
// 返り値 : 無し
// 概要 : 走行体完全停止用モータの角度制御
//*****************************************************************************
void Ang_Robo::tail_control(signed int angle)
{

  //  tail_motor_pwm = (float)(angle - mTail_Motor.getCount()*P_GAIN); /* 比例制御 */
  tail_motor_pwm = gTail_pwm->calc_pid(angle, mTail_Motor.getCount());
  tail_motor_pwm = tail_motor_pwm*0.1;
  /* PWM出力飽和処理 */
  if (tail_motor_pwm > PWM_ABS_MAX)
    {
      tail_motor_pwm = PWM_ABS_MAX;
    }
  else if (tail_motor_pwm < -PWM_ABS_MAX)
    {
      tail_motor_pwm = -PWM_ABS_MAX;
    }

  if (tail_motor_pwm == 0)
    {
      //17.07.28 kota modify//        ev3_motor_stop(tail_motor, true);
      mTail_Motor.stop();
    }
  else
    {
      //17.07.28 kota modify//        ev3_motor_set_power(tail_motor, (signed char)pwm);
      mTail_Motor.setPWM((signed int)tail_motor_pwm);
    }


}

//170816 ota add tail control

void Ang_Robo::tail_reset(){
  int32_t angle    = 0;
  int32_t angle_1d = 0;

  mTail_Motor.setPWM(-10);
  angle = 0;
  angle_1d = 1;

  while(1){
    if(angle == angle_1d){
      mTail_Motor.stop();
      mTail_Motor.reset();
      break;
    }
    else{
      angle_1d = angle;
      tslp_tsk(1000);
      angle = mTail_Motor.getCount();
    }
  }
  mTail_Motor.stop();
  mTail_Motor.reset();
}

void Ang_Robo::tail_stand_up(){
    while(1){
      if(mTail_Motor.getCount() == TAIL_ANGLE_STAND_UP){
	mTail_Motor.stop();
	break;
      }
      else{
	mTail_Motor.setPWM(5);
      }
    }
    mTail_Motor.stop();
} //tail for gyro reset and color sensor calibration

void Ang_Robo::tail_lug(){
    while(1){
      if(mTail_Motor.getCount() == TAIL_ANGLE_LUG){
	mTail_Motor.stop();
	break;
      }
      else{
	mTail_Motor.setPWM(5);
      }
    }
    mTail_Motor.stop();
} //tail for gyro reset and color sensor calibration


void Ang_Robo::tail_stand_from_balance(){
  static float   target_tail_angle;
  static int32_t clock_start;

  switch(STAND_MODE){
  case BALANCE:
    log_stand_mode = 0;
    mForward = 0;
    mTurn = 0;
    target_tail_angle =  TAIL_ANGLE_RUN;

    balance_off_en = false;
    if(mTail_stand_mode == true){
      STAND_MODE = TAIL_DOWN;
    }
    //    pre_balancer_on = false;
    break;

  case TAIL_DOWN:
    log_stand_mode = 1;
    mForward = 0;
    mTurn = 0;
    if(target_tail_angle <= TAIL_ON_ANGLE){
      target_tail_angle = target_tail_angle + 0.5;
    }
    if(mTail_Motor.getCount() >= TAIL_ON_ANGLE){
      STAND_MODE = TAIL_ON;
      clock_start = robo_Clock->now();
    }
    tail_control(target_tail_angle);
    balance_off_en = false;
    break;

  case TAIL_ON:
    log_stand_mode = 2;

    mForward = 0;
    mTurn = 0;

    /*2017 version
    if((robo_Clock->now() - clock_start) < 3500){
      mForward = 0;
      mTurn = 0;
      balance_off_en = false;
    }
    else if((robo_Clock->now() - clock_start) > 5000){
      mForward = 0;
      mTurn = 0;
      balance_off_en = true;
      STAND_MODE = TAIL_STAND;
      clock_start = robo_Clock->now();
    }else{
      mForward = -20;
      mTurn = 0;
      balance_off_en = false;
    }
    */

    /*tail on 2018 version*/


    mForward = 0;
    mTurn = 0;
    balance_off_en = true;
    STAND_MODE = TAIL_STAND;
    clock_start = robo_Clock->now();
    break;

  case TAIL_STAND:
    log_stand_mode = 3;
    lug_mode     = false;
    balance_off_en = true;

    // if((robo_Clock->now() - clock_start) < 500){
    if((robo_Clock->now() - clock_start) < 700){
    //    if((robo_Clock->now() - clock_start) < 1000){
      mForward = 0;
      mTurn = 0;
    }else{
      ROBO_MODE = TAIL_RUN;

      if(mTail_lug_mode == true){

	STAND_MODE = TAIL_LUG;
      }
    }


    break;

  case TAIL_LUG:
    log_stand_mode = 4;
    balance_off_en = true;
    ROBO_MODE = LUG_RUN;

    if(mTail_Motor.getCount() <= TAIL_ANGLE_LUG){
      clock_start = robo_Clock->now();
      tail_control(TAIL_ANGLE_LUG);
      mTail_Motor.setBrake(true);
      lug_mode     = true;
    }else{
      target_tail_angle = target_tail_angle - 0.05;
      tail_control(target_tail_angle);
    }

    if(mTail_lug_mode == false){
      STAND_MODE = LUG_TO_STAND;
      mTail_Motor.setBrake(false);
    }

    break;

  case LUG_TO_STAND:
    log_stand_mode = 5;
    balance_off_en = true;
    mTurn = 0;

    if(mTail_Motor.getCount() < 75){
      mForward = -100;
      mTail_Motor.setPWM(100);
    }else{
      mForward = 0;
      tail_control(TAIL_ON_ANGLE);
      target_tail_angle = TAIL_ON_ANGLE;
      if(mTail_Motor.getCount() >= TAIL_ON_ANGLE){
	mTail_Motor.setBrake(true);
	mTail_Motor.setPWM(0);
	STAND_MODE = TAIL_STAND;
	clock_start = robo_Clock->now();
      }
    }

    break;


  case STAND_VERT:
    log_stand_mode = 6;
    mForward = 0;
    mTurn    = 0; 
    balance_off_en = true;
    if(mTail_Motor.getCount() >= 95){
      tail_control(96);
      clock_start = robo_Clock->now();
      STAND_MODE = STAND_TO_BALANCE;
    }

    if(mTail_Motor.getCount() < 96){
      target_tail_angle = target_tail_angle + 0.02;
      tail_control(target_tail_angle);
    }else{
      tail_control(96);
      clock_start = robo_Clock->now();
      STAND_MODE = STAND_TO_BALANCE;
    }
    break;
      
  case STAND_TO_BALANCE:
    log_stand_mode = 7;
    mForward = 0;
    mTurn    = 0; 
    balance_off_en = true;
    tail_control(96);

    if((robo_Clock->now() - clock_start) > 1000){
      STAND_MODE     = TAIL_FOR_RUN;
    }
    break;

  case TAIL_FOR_RUN:
    log_stand_mode = 8;
    mForward = 0;
    mTurn    = 0; 
    tail_control(98);
    balance_off_en = true;

    if(mTail_Motor.getCount() >=  97){
      STAND_MODE     = BALANCE;
      ROBO_MODE      = BALANCE_RUN;
      balance_off_en = false;

      //0828---
	omega = mGyroSensor.getAnglerVelocity();  // ジャイロセンサ値
	mLeftWheel.reset();
	mRightWheel.reset();
	mBalancer->init(omega);
      //---0828

    }
    break;



  default:
    mForward = 0;
    mTurn = 0;
    balance_off_en = false;
    break;
  }
}

void Ang_Robo::forward_curve_gen(){
  int dif;
  bool slow_down;

  dif = mTarget_forward - mCurrent_forward;

  //  if(mVelocity > 500){
  if(mVelocity > MAX_VELOCITY){
    slow_down = true;
  }else{
    slow_down = false;
  }
 
  if(dif == 0){
    if(slow_down){
      //      mCurved_forward  = mCurved_forward - 0.05;
      mCurved_forward  = mCurved_forward - 0.01;
      mCurrent_forward = (int)( mCurved_forward + 0.5);
    }else{
      mCurrent_forward = mCurrent_forward;
    }

  }else if(dif > 0){
    if(slow_down){
      mCurved_forward  = mCurved_forward - 0.2;
      mCurrent_forward = (int)( mCurved_forward + 0.5);
    }else{
      mCurved_forward  = mCurved_forward + 0.2;
      mCurrent_forward = (int)( mCurved_forward + 0.5);
    }
  }else{
    mCurved_forward  = mCurved_forward - 0.2;
    mCurrent_forward = (int)( mCurved_forward + 0.5);
  }

}

void Ang_Robo::velocity_ctl(){
  
  float k;
  float val;

  //  k = mRef_velocity/100.0;
  //  k = 0.001;
  k = 0.002;

  //  val = gForward->calc_pid(mRef_velocity,   mVelocity);
  val = gForward->calc_pid(500,   mVelocity);
  val = k*val;

  mCurved_forward  = mCurved_forward + val;

  if(mCurved_forward > 200){
    mCurved_forward  = 200.0;
  }

  mCurrent_forward = (int)( mCurved_forward + 0.5);

  //  mCurrent_forward = 180;


}


//2017/08/06多田さんヨーレートコントローラー
float Ang_Robo::YawrateController(float yawrate, float yawrate_cmd)
{
  //r_yaw_rate = yawrate_cmd*I_gain1 - r_yaw_rate_ud; //20180530 kota
  r_yaw_rate = (-1.0*yawrate_cmd*I_gain1) - r_yaw_rate_ud; //20180530 kota

  
  r_yaw_rate_ud =(r_yaw_rate*I_gain2 + r_yaw_rate_ud*I_gain3);
  F_out = F_controller((float)r_yaw_rate);
  E_out = E_controller((float)r_yaw_rate);
  C_out = C_controller(E_out, (float)yawrate, S_out);
  S_out = S_controller(C_out);
  turn_tmp = F_out + C_out;
  if(turn_tmp > 100) turn_tmp = 100;
  if(turn_tmp < -100) turn_tmp = -100;

  
  return turn_tmp;//制御出力

}

float Ang_Robo::F_controller(float r_yaw_rate)
{
	F_in = r_yaw_rate;

	F_out = F_in * F_gain;

	return F_out;
}

float Ang_Robo::E_controller(float r_yaw_rate)
{
	E_in_dddddd = E_in_ddddd;
	E_in_ddddd = E_in_dddd;
	E_in_dddd = E_in_ddd;
	E_in_ddd = E_in_dd;
	E_in_dd = E_in_d;
	E_in_d = E_in;
	E_in = (r_yaw_rate);
	E_out = E_ud1;
	E_ud1 = (E_in_dddddd * E_gain1) + E_ud1 * E_gain2;

	return E_out;
}

float Ang_Robo::C_controller(float E_out, float yawrate, float S_out)
{
//	C_in = (E_out - yawrate - S_out);
	C_in = (E_out + yawrate - S_out);//0816
//	C_in = (E_out - yawrate);
	C_out = C_ud1;
	C_ud1 = C_in * C_gain + (C_out * 1.0);

	if(C_ud1 > 20){
	  C_ud1 = 20;
	}else if(C_ud1 < -20){
	  C_ud1 = -20;
	}

	return C_out;
}

float Ang_Robo::S_controller(float C_out)
{
	S_in_dddddd = S_in_ddddd;
	S_in_ddddd = S_in_dddd;
	S_in_dddd = S_in_ddd;
	S_in_ddd = S_in_dd;
	S_in_dd = S_in_d;
	S_in_d = S_in;
	S_in = C_out*S_gain1;
	S_out = Pn_ud1 - Pd_ud1;
	Pn_ud1 = S_in*Pn_gain1 + Pn_ud1*Pn_gain2;
	Pd_ud1 = S_in_dddddd*Pd_gain1 + Pd_ud1*Pd_gain2;

	return S_out;
}

void Ang_Robo::TailMode(int mForward, float mTurn){
	mtail_mode_pwm_l = 0.5*mForward + 1.0*mTurn ;
	mtail_mode_pwm_r = 0.5*mForward - 1.0*mTurn;
}

