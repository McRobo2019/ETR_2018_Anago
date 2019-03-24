#include "ang_eye.hpp"
#include "math.h"

#define CALIB_FONT        (EV3_FONT_MEDIUM)
#define CALIB_FONT_WIDTH  (6/*TODO: magic number*/)
#define CALIB_FONT_HEIGHT (20/*TODO: magic number*/)


// 定数宣言
const int8_t Ang_Eye::INITIAL_WHITE_THRESHOLD = 40;  // 黒色の光センサ値
const int8_t Ang_Eye::INITIAL_BLACK_THRESHOLD = 20;  // 黒色の光センサ値

/**
 * コンストラクタ
 * @param colorSensor カラーセンサ
 */
Ang_Eye::Ang_Eye(const ev3api::ColorSensor& colorSensor,
		 ev3api::Motor& leftWheel,
		 ev3api::Motor& rightWheel,
		 ev3api::GyroSensor& gyro,
		 ev3api::SonarSensor& sonar,
		 ev3api::TouchSensor& touchSensor)
  : 
    mColorSensor(colorSensor),
    mLeftWheel(leftWheel),
    mRightWheel(rightWheel),
    mGyro(gyro),
    mSonar(sonar),
    mTouch(touchSensor)
{
}

void Ang_Eye::init(){
  //  int i;

  mGyro.reset();

  //  real_wheel = WHEEL_R * cos(RAD_6_DEG);
 
  //real_wheel = WHEEL_R * 1.05; /*180630 temporary value*/
  real_wheel = WHEEL_R;
  xvalue    = 360.0;
  yvalue    = 160.0;

  SENSOR_CALIB_MODE = LINE_100;
  CORRECT_MODE = ST_1ST;
  pre_sonar_dis = 0;


  pre_50mm_x = 0.0;
  pre_50mm_y = 0.0;

  ave_x     = 360.0; //average_x 20181008
  ave_y     = 160.0; //average_y 20181008

  ave_vel_x = 0;    //20181008
  ave_vel_y = 0;    //20181008


  odo             = 0.0;
  velocity        = 0.0;
  ave_velo        = 0.0; //20181008
  ave_accel       = 0.0; //20181008
  pre_velo_0p5sec = 0.0;//20181008

  encR      = 0;
  encL      = 0;

  old_encR  = 0;
  old_encL  = 0;

  wheel_rotational_speed = 0;
  ave_wheel_rot_speed    = 0;
  abs_ave_wheel_speed    = 0;
  wheel_load             = 0;

  yawrate   = 0;
  abs_angle = 0;
  ave_angle = 0;

  /*
 for (i=0; i<125; i++){
    angle_dat_500ms[i]    = 0.0;
    velocity_dat_500ms[i] = 0.0;
    x_dat_500ms[i]        = 360.0;
    y_dat_500ms[i]        = 160.0;
  }
  */
  
  robo_stop       = 1;
  robo_forward    = 0;
  robo_back       = 0;
  robo_turn_left  = 0;
  robo_turn_right = 0;

    gAve_angle_dat->init(0.0);          

  gAve_x_dat->init(360.0);          
  gAve_y_dat->init(160.0);

  gAve_vel_x_dat->init(0.0);
  gAve_vel_y_dat ->init(0.0);

  gAve_velo_dat->init(0.0);
  gAve_accel_dat->init(0.0);

  gAve_wheel_rot_dat->init(0.0);
  gAve_wheel_load_dat->init(0.0);

  gAve_angle_500_dat->init();

}

void Ang_Eye::run( ) {
  det_line_rgb();
  wheel_odometry(dT_4ms);
  average_dat(dT_4ms);
  det_Movement(); //20180501
}


void Ang_Eye::color_sensor_calib( ) {

  rgb_raw_t rgb_val;
  int       set_value;
  char s[20]={'\0'};

  ev3_lcd_fill_rect(0, 0, EV3_LCD_WIDTH, EV3_LCD_HEIGHT, EV3_LCD_WHITE);
  //  ev3_lcd_set_font(EV3_FONT_SMALL);
  ev3_lcd_set_font(EV3_FONT_MEDIUM);

  while(1){

    if(SENSOR_CALIB_MODE == CALIB_DONE){
      break;
    }

    switch(SENSOR_CALIB_MODE){
    case LINE_100:

      ev3_lcd_draw_string("Set Line 100%", 0, 20);

      mColorSensor.getRawColor(rgb_val);
      set_value =  rgb_val.b;

      sprintf(s,"BLUE : %d ", rgb_val.b);
      ev3_lcd_draw_string(s, 0, 40);
      ev3_lcd_draw_string("SET    : E",0, 60);
      ev3_lcd_draw_string("CANCEL : TOUCH",0, 80);

      if (mTouch.isPressed()){
	SENSOR_CALIB_MODE = SET_DEFAULT;
	ev3_lcd_fill_rect(0, 0, EV3_LCD_WIDTH, EV3_LCD_HEIGHT, EV3_LCD_WHITE);
      }

      if (ev3_button_is_pressed(ENTER_BUTTON)){
	ev3_lcd_fill_rect(0, 0, EV3_LCD_WIDTH, EV3_LCD_HEIGHT, EV3_LCD_WHITE);

	if(set_value > CALIB_LINE_100_MAX_THRS ){
	  SENSOR_CALIB_MODE = LINE_100_ERROR;
	}else{
	  line_0_val=set_value;
	  sprintf(s,"SET_VAL : %d ", line_0_val);
	  ev3_lcd_draw_string(s, 0, 40);
	  //	  SENSOR_CALIB_MODE = LINE_50;
	  SENSOR_CALIB_MODE = LINE_0;
	  tslp_tsk(1000);
	  ev3_lcd_fill_rect(0, 0, EV3_LCD_WIDTH, EV3_LCD_HEIGHT, EV3_LCD_WHITE);
	}
      }

      tslp_tsk(100);
  
      break;

    case LINE_100_ERROR:
      ev3_lcd_draw_string("ERROR Too Big", 0, 20);
      SENSOR_CALIB_MODE = LINE_100;
      tslp_tsk(1000);
      ev3_lcd_fill_rect(0, 0, EV3_LCD_WIDTH, EV3_LCD_HEIGHT, EV3_LCD_WHITE);
      break;

    case LINE_50:

      ev3_lcd_draw_string("Set Line 50%", 0, 20);
      mColorSensor.getRawColor(rgb_val);
      set_value =  rgb_val.b;

      sprintf(s,"BLUE : %d ", rgb_val.b);
      ev3_lcd_draw_string(s, 0, 40);
      ev3_lcd_draw_string("SET    : E",0, 60);
      ev3_lcd_draw_string("CANCEL : TOUCH",0, 80);

      if (mTouch.isPressed()){
	SENSOR_CALIB_MODE = SET_DEFAULT;
	ev3_lcd_fill_rect(0, 0, EV3_LCD_WIDTH, EV3_LCD_HEIGHT, EV3_LCD_WHITE);
      }

      if (ev3_button_is_pressed(ENTER_BUTTON)){
	ev3_lcd_fill_rect(0, 0, EV3_LCD_WIDTH, EV3_LCD_HEIGHT, EV3_LCD_WHITE);

	if((set_value > CALIB_LINE_50_MAX_THRS)||(set_value < CALIB_LINE_50_MIN_THRS )){
	  SENSOR_CALIB_MODE = LINE_50_ERROR;
	}else{
	  line_50_val = set_value;
	  sprintf(s,"SET_VAL : %d ", line_50_val);
	  ev3_lcd_draw_string(s, 0, 40);
	  SENSOR_CALIB_MODE = LINE_0;
	  tslp_tsk(1000);
	  ev3_lcd_fill_rect(0, 0, EV3_LCD_WIDTH, EV3_LCD_HEIGHT, EV3_LCD_WHITE);
	}
      }

      tslp_tsk(100);

      break;
      
    case LINE_50_ERROR:
      ev3_lcd_draw_string("ERROR Too Big or Small", 0, 20);
      SENSOR_CALIB_MODE = LINE_50;
      tslp_tsk(1000);
      ev3_lcd_fill_rect(0, 0, EV3_LCD_WIDTH, EV3_LCD_HEIGHT, EV3_LCD_WHITE);
      break;
    
    case LINE_0:
      ev3_lcd_draw_string("Set Line 0%", 0, 20);

      mColorSensor.getRawColor(rgb_val);
      set_value =  rgb_val.b;

      sprintf(s,"BLUE : %d ", rgb_val.b);
      ev3_lcd_draw_string(s, 0, 40);
      ev3_lcd_draw_string("SET    : E",0, 60);
      ev3_lcd_draw_string("CANCEL : TOUCH",0, 80);

      if (mTouch.isPressed()){
	SENSOR_CALIB_MODE = SET_DEFAULT;
	ev3_lcd_fill_rect(0, 0, EV3_LCD_WIDTH, EV3_LCD_HEIGHT, EV3_LCD_WHITE);
      }

      if (ev3_button_is_pressed(ENTER_BUTTON)){
	ev3_lcd_fill_rect(0, 0, EV3_LCD_WIDTH, EV3_LCD_HEIGHT, EV3_LCD_WHITE);

	if(set_value < CALIB_LINE_0_MIN_THRS ){
	  SENSOR_CALIB_MODE = LINE_0_ERROR;
	}else{
	  line_100_val = set_value;
	  sprintf(s,"SET_VAL : %d ", line_100_val);
	  ev3_lcd_draw_string(s, 0, 40);
	  SENSOR_CALIB_MODE = SET_GAIN_OFFSET;
	  tslp_tsk(1000);
	  ev3_lcd_fill_rect(0, 0, EV3_LCD_WIDTH, EV3_LCD_HEIGHT, EV3_LCD_WHITE);
	}
      }

      tslp_tsk(100);
      break;

    case LINE_0_ERROR:
      ev3_lcd_draw_string("ERROR Too Small", 0, 20);
      SENSOR_CALIB_MODE = LINE_0;
      tslp_tsk(1000);
      ev3_lcd_fill_rect(0, 0, EV3_LCD_WIDTH, EV3_LCD_HEIGHT, EV3_LCD_WHITE);
      break;

    case SET_DEFAULT:
      ev3_lcd_draw_string("Set Defalut", 0, 20);

      line_100_val = 180;
      line_50_val  = 90;
      line_0_val = 10;

      line_val_offset = COLOR_SENSOR_OFFSET;
      line_val_gain   = COLOR_SENSOR_GAIN;

      SENSOR_CALIB_MODE = CALIB_DONE;

      break;

    case SET_GAIN_OFFSET:
      line_val_offset = line_0_val;
      line_val_gain   = line_100_val - line_0_val;
      line_val_gain   = line_val_gain/100.0;
      sprintf(s,"offset: %d ", line_val_offset);
      ev3_lcd_draw_string(s, 0, 40);

      sprintf(s,"gain: %f ", line_val_gain);
      ev3_lcd_draw_string(s, 0, 60);
      SENSOR_CALIB_MODE = CALIB_DONE;
      tslp_tsk(1000);
      break;

    case CALIB_DONE:

      break;

    default:
      break;
    }
  }
}

void Ang_Eye::det_line_rgb(){

  rgb_raw_t rgb_val;
  float adj_brightness;

  mColorSensor.getRawColor(rgb_val);

  /*
  adj_brightness = rgb_val.b - COLOR_SENSOR_OFFSET;
  adj_brightness = adj_brightness/COLOR_SENSOR_GAIN;
  */
  adj_brightness = rgb_val.b - line_val_offset;
  adj_brightness = adj_brightness/line_val_gain;

  if(adj_brightness < 0){
    adj_brightness = 0;
  }
  else if(adj_brightness > 100){
    adj_brightness = 100;
  }
  linevalue = 100-adj_brightness;

  if((rgb_val.r < 10) && (rgb_val.g > rgb_val.b)){
    green_flag = 1;
  }else{
    green_flag = 0;
  }

}

void Ang_Eye::wheel_odometry(float dT) {

  static float odo_prev;
  static float velocity_input;
  static float velocity_prev;
  //LPF 10[rad/s]/////////////////////////////////////
  static float Alpfd = 0.9391; // LPF
  static float Blpfd = 1; // LPF
  static float Clpfd = 0.0609; // LPF
  static float Dlpfd = 0; // LPF
  //////////////////////////////////////////
  static float old_rel_angle;     //過去のYaw角[rad]

  int   WheelAngRdeg = mRightWheel.getCount();  //右モータ回転角度[deg]
  int   WheelAngLdeg = mLeftWheel.getCount();   //右モータ回転角度[deg]

  encR =  WheelAngRdeg;
  encL =  WheelAngLdeg;

  odo            = ((float)WheelAngLdeg + (float)WheelAngRdeg)/2.0 * RAD_1_DEG * real_wheel; //[mm]

  velocity_input = (odo - odo_prev)/dT;
  velocity       = Clpfd * velocity_prev + Dlpfd * velocity_input;
  velocity_prev  = Alpfd * velocity_prev + Blpfd * velocity_input;
  
  relative_angle =  ((float)WheelAngRdeg - (float)WheelAngLdeg) * RAD_1_DEG * real_wheel / RoboTread; //ロボのYaw角[rad]
  //  relative_angle = relative_angle + correction_angle; //20180701 kota

  abs_angle      = relative_angle + correction_angle;

  xvalue = xvalue+(odo-odo_prev)*cos(abs_angle);
  yvalue = yvalue+(odo-odo_prev)*sin(abs_angle);

  pre_50mm_x = xvalue + 50.0*cos(abs_angle);//20180512 kota
  pre_50mm_y = yvalue + 50.0*sin(abs_angle);


  yawrate  =(relative_angle-old_rel_angle)/dT;           //ロボのYawレート[rad/s]

  old_rel_angle=relative_angle;         //過去のYaw角[rad]
  odo_prev = odo;

  //  average_dat(dT);
  //  det_Movement(); //20180501

  //  pre_velo_0p5sec  =  ave_velo + (ave_accel * 0.5);
}

void Ang_Eye::average_dat(float dT) {
  ave_angle           = gAve_angle_dat->average_125(abs_angle);
  ave_velo            = gAve_velo_dat->average_125(velocity);
  ave_wheel_rot_speed = gAve_wheel_rot_dat->average_125(wheel_rotational_speed);
  ave_wheel_load      = gAve_wheel_load_dat->average_125(wheel_load);
  ave_x               = gAve_x_dat->average_125(xvalue);
  ave_y               = gAve_y_dat->average_125(yvalue);

  dif_angle_ave_dat    = ave_angle    - old_angle_ave_dat;
  old_angle_ave_dat    = ave_angle;
 
  dif_velocity_ave_dat = ave_velo - old_velocity_ave_dat;
  old_velocity_ave_dat = ave_velo;
 
  dif_x_ave_dat        = ave_x - old_x_ave_dat;
  old_x_ave_dat        = ave_x;
 
  dif_y_ave_dat        = ave_y - old_y_ave_dat;
  old_y_ave_dat        = ave_y;
  
  ave_accel = dif_velocity_ave_dat/dT;
 
  ave_vel_x = dif_x_ave_dat/dT;
  ave_vel_y = dif_y_ave_dat/dT;
 
  pre_velo_0p5sec  =  ave_velo + (ave_accel * 0.5);
  
}

void Ang_Eye::correct_odometry( ) {

  /* 2018 LEFT */
  int   dif_sonar_dis;
  float correct_x;
  float correct_y;

  static float x_min;
  static float x_max;
  static float ref_y;


  switch(CORRECT_MODE){
  case ST_1ST:
    if((xvalue > (FIRST_STRAIGHT_AREA[2] - 300))&&(robo_forward == 1)){
      correction_angle = 0.0 - ave_angle;
      yvalue = 160.0;
      CORRECT_MODE = ST_2ND;
    }
    
    break;
    
  case ST_2ND:
    if((yvalue > (SECOND_STRAIGHT_AREA[3] - 300))&&(robo_forward == 1)){
      correction_angle = RAD_90_DEG - ave_angle;
      xvalue = 3770.0;
      CORRECT_MODE = ST_3RD;
    }

    x_max = 0;
    break;

  case ST_3RD:
    if((xvalue > (THIRD_STRAIGHT_AREA[0] + 200))&&(robo_forward == 1)){
      correction_angle = 0.0 - ave_angle;
      yvalue = 3270;
      x_min  = xvalue;
  //	 CORRECT_MODE = DONE;
      //      CORRECT_MODE = CN_4TH;
      CORRECT_MODE = ST_4TH;
    }
    
    if(xvalue > x_max){
      x_max = xvalue;
    }

    break;

  case ST_4TH:

    if(xvalue > x_max){
      x_max = xvalue;
    }

    if( yvalue < 2000 ){
      correct_x = 5020 - x_max;
      xvalue = xvalue + correct_x;
      CORRECT_MODE = CN_4TH;
    }

    break;



  case CN_4TH:

    if(xvalue < x_min){
      x_min = xvalue;
      ref_y = yvalue;
    }

    if( yvalue < 280 ){
      correct_x = 3580 - x_min;
      xvalue = xvalue + correct_x;

      correct_y = 450 - ref_y;
      yvalue = yvalue + correct_y;
      CORRECT_MODE = LUG;
      pre_sonar_dis = sonarDistance;
    }

    /*
    if((yvalue < FIRST_GRAY_AREA[3]) && (ave_angle > MINUS_RAD_5_DEG)){

    if( (xvalue > 3770 )&&( xvalue < 3970) ){
    xvalue = 3870;
    yvalue = 160;
    }

    CORRECT_MODE = LUG;
    pre_sonar_dis = sonarDistance;

    }*/



    break;

  case LUG:
    dif_sonar_dis = sonarDistance - pre_sonar_dis;
   
    if( (dif_sonar_dis > 100) && (pre_sonar_dis < 20) && (xvalue > 4100) ){
      xvalue       = BACK_LUG_AREA[0] - (pre_sonar_dis * 10);
      CORRECT_MODE = DONE;
    }

    pre_sonar_dis = sonarDistance;

    break;
    
  case DONE:

    break;

  default:
    break;
    }
}



//!!!! Not Done Any Verification Yet 20180501 !!!!
/****************************************************************************************/
//2018 0501 Kaoru Ota
//Move DET_MOVEMENT from :wheel_odometry(float dT)
//!!!! Not Done Any Verification Yet 20180501 !!!!
/****************************************************************************************/
void Ang_Eye::det_Movement( ) {

  if (dif_angle_ave_dat > -0.001 && dif_angle_ave_dat < 0.001){

    if(ave_velo > 0){
      robo_stop       = 0;
      robo_forward    = 1;
      robo_back       = 0;
      robo_turn_left  = 0;
      robo_turn_right = 0;
    }else if (ave_velo < 0){
      robo_stop       = 0;
      robo_forward    = 0;
      robo_back       = 1;
      robo_turn_left  = 0;
      robo_turn_right = 0;
    }else{
      robo_stop       = 1;
      robo_forward    = 0;
      robo_back       = 0;
      robo_turn_left  = 0;
      robo_turn_right = 0;
    }

  }else if(dif_angle_ave_dat >= 0.001){
    robo_stop       = 0;
    robo_forward    = 0;
    robo_back       = 0;
    robo_turn_left  = 1;
    robo_turn_right = 0;
  }else if(dif_angle_ave_dat <= -0.001){
    robo_stop       = 0;
    robo_forward    = 0;
    robo_back       = 0;
    robo_turn_left  = 0;
    robo_turn_right = 1;
  }else {
    robo_stop       = 0;
    robo_forward    = 0;
    robo_back       = 0;
    robo_turn_left  = 0;
    robo_turn_right = 0;
  }
}


void Ang_Eye::det_Dansa( ) {
  static int gyro_250d[250];
  int cnt;
  gyro_250d[0] = mGyro.getAnglerVelocity();

  for(cnt =9; cnt > 0; cnt--){
    gyro_250d[cnt] = gyro_250d[cnt-1];
  }
 
  for(cnt = 0; cnt < 250; cnt++){
    if(gyro_250d[cnt] < -100 || gyro_250d[cnt] > 100){
      if(velocity < 0){
	dansa = 1;
	cnt = 250;
      }else{
	dansa = 0;
      }
    }else{
      dansa = 0;
    }
  }
}


void Ang_Eye::setSonarDistance(void) {

  if(sonar_counter%50 == 0){
    sonarDistance = mSonar.getDistance();
  }
  sonar_counter++;
  if(sonar_counter>10000) sonar_counter = 0;
 
}



