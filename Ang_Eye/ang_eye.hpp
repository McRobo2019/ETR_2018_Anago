#ifndef ANAGO_EYE_H_
#define ANAGO_EYE_H_

#include "ev3api.h"
#include "parameter.h"
#include "ColorSensor.h"
#include "Motor.h"
#include "GyroSensor.h"
#include "SonarSensor.h"
#include "TouchSensor.h"
#include "util.hpp"

class Ang_Eye {
public:
    explicit Ang_Eye(const ev3api::ColorSensor& colorSensor,
		     ev3api::Motor& leftWheel,
		     ev3api::Motor& rightWheel,
		     ev3api::GyroSensor& gyro,
		     ev3api::SonarSensor& sonar,
		     ev3api::TouchSensor& touchSensor);

  void  init(); //17.0.28 k-ota add
  void  run();
  void  color_sensor_calib();
  void  det_line_rgb();

  void  wheel_odometry(float dT);
  void  average_dat(float dT); //20181008
  void  correct_odometry( );
  void  det_Movement();
  void  det_Dansa();
  void  setSonarDistance(void);

  bool  stop_sys   = 0;
  int   linevalue  = 0;
  bool  green_flag = 0;

  float xvalue    = 0.0;//x座標推定値
  float yvalue    = 0.0;//y座標推定値

  float pre_50mm_x = 0.0;//50mm saki 20180512 kota
  float pre_50mm_y = 0.0;//50mm saki 20180512 kota

  float ave_x     = 0.0; //average_x 20181008
  float ave_y     = 0.0; //average_y 20181008

  float ave_vel_x = 0.0; //20181008
  float ave_vel_y = 0.0; //20181008


  float odo              = 0.0;//odometry
  float velocity         = 0.0;//Velocity
  float ave_velo         = 0.0;//Average_Velocity
  float ave_accel        = 0.0;//Average_Acceleration
  float pre_velo_0p5sec  = 0.0;//prediction vekicity 0.5 sec latar


  int   encR      = 0;//右側タイヤ角度
  int   encL      = 0;
  int   old_encR  = 0;//右側タイヤ角度
  int   old_encL  = 0;

  float wheel_rotational_speed = 0;
  float ave_wheel_rot_speed    = 0;
  float abs_ave_wheel_speed    = 0;
  float wheel_load             = 0;
  float ave_wheel_load         = 0;

  float yawrate   = 0;
  float abs_angle = 0;
  float ave_angle = 0;
  float ave_angle_500 = 0;
  
  bool  dansa     = 0;
  //signals for robo movement
  bool  robo_stop       = 0;
  bool  robo_forward    = 0;
  bool  robo_back       = 0;
  bool  robo_turn_left  = 0;
  bool  robo_turn_right = 0;

  int    sonarDistance = 0; // 距離 [cm]
  int    pre_sonar_dis = 0; // 距離 [cm]
  bool   sonar_stop  = false;

  float correction_angle = 0.0;
  
private:

  static const int8_t INITIAL_WHITE_THRESHOLD;
  static const int8_t INITIAL_BLACK_THRESHOLD;

  const ev3api::ColorSensor& mColorSensor;
  ev3api::Motor& mLeftWheel;
  ev3api::Motor& mRightWheel;
  ev3api::GyroSensor& mGyro;
  ev3api::SonarSensor& mSonar;
  ev3api::TouchSensor& mTouch;

  Average_125_Data *gAve_angle_dat      = new Average_125_Data();
  Average_125_Data *gAve_x_dat          = new Average_125_Data();
  Average_125_Data *gAve_y_dat          = new Average_125_Data();
  Average_125_Data *gAve_vel_x_dat      = new Average_125_Data();
  Average_125_Data *gAve_vel_y_dat      = new Average_125_Data();
  Average_125_Data *gAve_velo_dat       = new Average_125_Data();
  Average_125_Data *gAve_accel_dat      = new Average_125_Data();
  Average_125_Data *gAve_wheel_rot_dat  = new Average_125_Data();
  Average_125_Data *gAve_wheel_load_dat = new Average_125_Data();

  Average_500_Data *gAve_angle_500_dat  = new Average_500_Data();
  
  enum   Sensor_Calib_Mode{
    LINE_100,
    LINE_100_ERROR,
    LINE_50,
    LINE_50_ERROR,
    LINE_0,
    LINE_0_ERROR,
    SET_DEFAULT,
    SET_GAIN_OFFSET,
    CALIB_DONE
  };

  enum Correct_Mode{
    ST_1ST,
    CN_1ST,
    ST_2ND,
    ST_3RD,
    CN_4TH,
    ST_4TH,
    LUG,
    DONE
  };

  Sensor_Calib_Mode SENSOR_CALIB_MODE;
  Correct_Mode      CORRECT_MODE;


  int    line_100_val;
  int    line_50_val;
  int    line_0_val;

  int8_t dColor_val[5]; //170814 ota signals for filter of color sensor value.
  
  int   line_val_offset = 0;
  float line_val_gain   = 0.0;

  float real_wheel;
  float relative_angle;
  //  float correction_angle = 0.0;
  
  //signals for detection robo's direction
  //  int   cap_size = 1000;
  //int   cap_cnt = 0;

  //20181008 ---- for CS
  //  float x_sum_dat     = 0.0;
  //  float x_ave_dat     = 0.0;
  float dif_x_ave_dat = 0.0;
  float old_x_ave_dat = 0.0;
  //  float x_dat_500ms[125]; //data array during 500ms @ 4ms task term
  //----20181001 for CS

  //20181008 ---- for CS
  //  float y_sum_dat     = 0.0;
  //  float y_ave_dat     = 0.0;
  float dif_y_ave_dat = 0.0;
  float old_y_ave_dat = 0.0;
  //  float y_dat_500ms[125]; //data array during 500ms @ 4ms task term
  //----20181001 for CS


  //  float angle_sum_dat     = 0.0;
  //  float angle_ave_dat     = 0.0;
  float dif_angle_ave_dat = 0.0;
  float old_angle_ave_dat = 0.0;
  //  float angle_dat_500ms[125]; //data array during 500ms @ 4ms task term

  //  float velocity_sum_dat     = 0.0;
  //  float velocity_ave_dat     = 0.0;
  float dif_velocity_ave_dat = 0.0;
  float old_velocity_ave_dat = 0.0;
  //  float velocity_dat_500ms[125]; //data array during 500ms @ 4ms task term

  float WheelAngVLt = 0;
  float WheelAngVRt = 0;
  float RoboVt      = 0;
  float RoboAngVt   = 0;
  int   sonar_counter = 0;
};

#endif  // ANAGO_EYE_H_
