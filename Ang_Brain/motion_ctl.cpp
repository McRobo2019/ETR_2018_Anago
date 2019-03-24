#include "motion_ctl.hpp"
#include "ev3api.h"
#include "Clock.h"
#include "math.h"

using ev3api::Clock;

#define liting_radius 10; // liting spot radius [mm]

Clock*       gClock;

Motion_Ctl::Motion_Ctl(){

}

void Motion_Ctl::init( ){
  gClock            = new Clock();
  MOTION_MODE       = LINE_TRACE;


  LUG_MODE          = LUG_START;

  SEESAW_MODE       = SEESAW_START;
  ZONE              = FIRST_STRAIGHT_ZONE;

  mYaw_angle_offset = 0.0;
  left_line_edge    = true;
  tail_stand_mode   = false;
  tail_lug_mode     = false;
  rising_seesaw     = false;
  falling_seesaw    = false;
  ref_forward       = 0.0;
  bat_mv            = ev3_battery_voltage_mV();


}

void Motion_Ctl::SetCurrentData(int     linevalue,
				bool    green_flag,
				float   xvalue,
				float   yvalue,
				float   pre_50mm_x, //20180512 kota
				float   pre_50mm_y,
				float   odo,                     
				float   velocity,
				float   yawrate,
				float   abs_angle,
				int     robo_tail_angle,
				bool    robo_stop,
				bool    robo_forward,
				bool    robo_back,
				bool    robo_turn_left,
				bool    robo_turn_right,
				bool    dansa,
				int16_t sonar_dis,
				bool    robo_balance_mode,
				bool    robo_lug_mode,
				int     max_forward,
				float   ref_yawrate,
				float   max_yawrate,
				float   min_yawrate
				 ) {

  mLinevalue         = linevalue;
  mGreen_flag      = green_flag;
  mXvalue            = xvalue;
  mYvalue            = yvalue;
  mPre_50mm_x        = pre_50mm_x;//50mm saki 20180512 kota
  mPre_50mm_y        = pre_50mm_y;//50mm saki 20180512 kota
  mOdo               = odo;
  mVelocity             = velocity;
  mYawrate           = yawrate;
  mYawangle          = abs_angle + mYaw_angle_offset;
  mTail_angle        = robo_tail_angle;
  mRobo_stop         = robo_stop;
  mRobo_forward      = robo_forward;
  mRobo_back         = robo_back;
  mRobo_turn_left    = robo_turn_left;
  mRobo_turn_right   = robo_turn_right;
  mDansa             = dansa;
  mSonar_dis         = sonar_dis;
  mRobo_balance_mode = robo_balance_mode;
  mRobo_lug_mode     = robo_lug_mode;

  mMax_Forward       = max_forward;
  mRef_Yawrate       = ref_yawrate;
  mMax_Yawrate       = max_yawrate;
  mMin_Yawrate       = min_yawrate;
}



void Motion_Ctl::set_mode_LT() {
  MOTION_MODE = LINE_TRACE;
}

void Motion_Ctl::set_mode_map_trace() {
  MOTION_MODE = MAP_TRACE;
}

void Motion_Ctl::set_mode_LUG() {
  MOTION_MODE = LUG;
}

void Motion_Ctl::set_mode_seesaw() {
  MOTION_MODE = SEESAW;
}



void Motion_Ctl::set_mode_tail_std_debug() {
  MOTION_MODE = TAIL_STAND_DEBUG;
}

void Motion_Ctl::set_mode_debug() {
  MOTION_MODE = DEBUG;
}

void Motion_Ctl::set_zone_start(){           ZONE = START_ZONE;}
void Motion_Ctl::set_zone_1st_straight(){    ZONE = FIRST_STRAIGHT_ZONE;}
void Motion_Ctl::set_zone_enter_1st_corner(){ZONE = ENTER_1ST_CORNER_ZONE;}
void Motion_Ctl::set_zone_1st_corner(){      ZONE = FIRST_CORNER_ZONE;}
void Motion_Ctl::set_zone_2nd_straight(){    ZONE = SECOND_STRAIGHT_ZONE;}
void Motion_Ctl::set_zone_enter_2nd_corner(){ZONE = ENTER_2ND_CORNER_ZONE;}
void Motion_Ctl::set_zone_2nd_corner(){      ZONE = SECOND_CORNER_ZONE;}
void Motion_Ctl::set_zone_3rd_straight(){    ZONE = THIRD_STRAIGHT_ZONE;}
void Motion_Ctl::set_zone_3rd_corner(){      ZONE = THIRD_CORNER_ZONE;}
void Motion_Ctl::set_zone_4th_straight(){    ZONE = FOURTH_STRAIGHT_ZONE;}
void Motion_Ctl::set_zone_4th_corner(){      ZONE = FOURTH_CORNER_ZONE;}
void Motion_Ctl::set_zone_5th_corner(){      ZONE = FIFTH_CORNER_ZONE;}
void Motion_Ctl::set_zone_1st_gray(){        ZONE = FIRST_GRAY_ZONE;}
void Motion_Ctl::set_zone_lug(){             ZONE = LUG_ZONE;}
void Motion_Ctl::set_zone_back_lug(){        ZONE = BACK_LUG_ZONE;}
void Motion_Ctl::set_zone_2nd_gray(){        ZONE = SECOND_GRAY_ZONE;}
void Motion_Ctl::set_zone_garage(){          ZONE = GARAGE_ZONE;}
void Motion_Ctl::set_zone_lost(){            ZONE = LOST;}


void Motion_Ctl::run(float xvalue, float yvalue, float yawangle) {

  switch(MOTION_MODE){

  case LINE_TRACE:
    forward =  mMax_Forward;
    LineTracerYawrate(mLinevalue);
    ref_tail_angle = TAIL_ANGLE_RUN;
    tail_stand_mode = false;
    break;

  case MAP_TRACE:

    forward =  mMax_Forward;

    if( mVelocity > 0){
      MapTracer(mXvalue, mYvalue, mYawangle);
    }else{
      yawratecmd      = 0.0;
    }
    //    ref_tail_angle = TAIL_ANGLE_RUN;
    ref_tail_angle = 30;
    tail_stand_mode = false;
    break;

  case LUG:
    forward =  mMax_Forward;
    LUG_run(mOdo, mYawangle,mLinevalue);
    break;

  case SEESAW:
    forward =  mMax_Forward;

    break;



  case  TAIL_STAND_DEBUG:

    forward =  mMax_Forward;
    yawratecmd      = 0.0;
    tail_stand_mode = true;

    //    if(mRobo_balance_mode == false){
    //      tail_stand_mode = false;	  
    //    }

    break;

  case DEBUG:

    forward    = mMax_Forward;
    yawratecmd = LUG_YAW_GAIN*(0 -   mYawangle);

    ref_tail_angle    = TAIL_ANGLE_RUN;
    tail_stand_mode = false;

    break;

  case  DEBUG_1:
    forward         = 0;
    yawratecmd      = 0.0;

    if(mRobo_balance_mode == false){
      tail_stand_mode = false;	  
      }

    break;

  default:
    forward = 0;
    break;
  }

}

/*void Motion_Ctl::LineTracerYawrate(int line_value) {

  y_t = (((float)line_value-50.0)/50.0) * (float)liting_radius;//20180530 kota

  if(y_t > 10.0) y_t = 10.0;
  if(y_t < -10.0) y_t = -10.0;
   y_t = y_t + 7.0*(y_t/8.0)*(y_t/8.0)*(y_t/8.0);

  //    yawratecmd = y_t/4.0;
  yawratecmd = (y_t/4.0)*(pg + df*(y_t-y_t_prev));
  
  if(yawratecmd > mMax_Yawrate){
    yawratecmd =  mMax_Yawrate;
    
  }else if (yawratecmd < mMin_Yawrate){
    yawratecmd = mMin_Yawrate;
  }else{
    yawratecmd = yawratecmd;
    }

    y_t_prev = y_t;

}
*/

void Motion_Ctl::LineTracerYawrate(int line_value) {

  float pos_yaw_step;
  float neg_yaw_step;

  pos_yaw_step = mMax_Yawrate - mRef_Yawrate;
  pos_yaw_step = pos_yaw_step/50.0;

  neg_yaw_step = mMin_Yawrate - mRef_Yawrate;
  neg_yaw_step = neg_yaw_step/50.0;

  /*
  y_t = (float)line_value-50.0;
  
  y_t = y_t * YAW_STEP;
  
  if(y_t > YAW_LIMIT)        y_t = YAW_LIMIT;
  if(y_t < -1.0 * YAW_LIMIT) y_t = -1.0 * YAW_LIMIT;
  */
  y_t = (float)line_value-50.0;

  if(y_t >= 0){
    yawratecmd = mRef_Yawrate + (y_t * pos_yaw_step);
  }else{
    yawratecmd = mRef_Yawrate - (y_t * neg_yaw_step);
  }
}


void Motion_Ctl::MapTracer(float mXvalue, float mYvalue, float mYawangle) {

  //	float extend_gain = 1.0;
  float Virtual_point_dist = 50.0;
	
  float x0,x1,x2,y0,y1,y2,a,a2,b,b2,r2,x10,y10,x12,y12;

/** LEFT 2018 *************************************************************** LEFT **/
  switch(ZONE){
  case FIRST_STRAIGHT_ZONE:
    
    x0 = mXvalue+Virtual_point_dist*cos(mYawangle);
    y0 = mYvalue+Virtual_point_dist*sin(mYawangle);

    x1 = STRAIGT_01[0];
    y1 = STRAIGT_01[1];
    x2 = STRAIGT_01[2];
    y2 = STRAIGT_01[3];

    x12 = x2-x1;
    y12 = y2-y1;
    x10 = x0-x1;
    y10 = y0-y1;
    a   = (x12*y10)-(y12*x10);

    y_t = a/sqrt(pow(x12,2.0) + pow(y12,2.0));
    y_t = -1.0 * y_t; //20180530
		
    if(y_t > 20.0) y_t = 20.0;
    if(y_t < -20.0) y_t = -20.0;
		
    yawratecmd = (y_t/4.0)*(pg + df*(y_t-y_t_prev));
    y_t_prev = y_t;
    
    break;

  /** LEFT 2018 *************************************************************** LEFT **/
  case FIRST_CORNER_ZONE:
		
    x0  = mXvalue+Virtual_point_dist*cos(mYawangle);
    y0  = mYvalue+Virtual_point_dist*sin(mYawangle);

    x1  = CIRCLE_01[0];
    y1  = CIRCLE_01[1];

    a   = x1 - x0;
    b   = y1 - y0;
    a2  = a * a;
    b2  = b * b;
    r2  = a2 + b2;
    y_t = sqrt(r2) - CIRCLE_01[2];
    //	  y_t = -1.0 * y_t; //20180530

    if(y_t > 20.0) y_t = 20.0;
    if(y_t < -20.0) y_t = -20.0;
		
    yawratecmd = (y_t/4.0)*(pg + df*(y_t-y_t_prev));
    y_t_prev = y_t;

    break;

  /** LEFT 2018 *************************************************************** LEFT **/
  case SECOND_STRAIGHT_ZONE:

    x0 = mXvalue+Virtual_point_dist*cos(mYawangle);
    y0 = mYvalue+Virtual_point_dist*sin(mYawangle);

    x1 = STRAIGT_02[0];
    y1 = STRAIGT_02[1];
    x2 = STRAIGT_02[2];
    y2 = STRAIGT_02[3];

    x12 = x2-x1;
    y12 = y2-y1;
    x10 = x0-x1;
    y10 = y0-y1;
    a   = (x12*y10)-(y12*x10);

    y_t = a/sqrt(pow(x12,2.0) + pow(y12,2.0));
    y_t = -1.0 * y_t; //20180530
		
    if(y_t > 20.0) y_t = 20.0;
    if(y_t < -20.0) y_t = -20.0;
		
    yawratecmd = (y_t/4.0)*(pg + df*(y_t-y_t_prev));
    y_t_prev = y_t;
		
    break;

  /** LEFT 2018 *************************************************************** LEFT **/
  case SECOND_CORNER_ZONE:
    x0 = mXvalue+Virtual_point_dist*cos(mYawangle);
    y0 = mYvalue+Virtual_point_dist*sin(mYawangle);

    x1 = CIRCLE_02[0];
    y1 = CIRCLE_02[1];

    a   = x1 - x0;
    b   = y1 - y0;
    a2  = a * a;
    b2  = b * b;
    r2  = a2 + b2;
    y_t = sqrt(r2) - CIRCLE_02[2];
    y_t = -1.0 * y_t; //20180530

    if(y_t > 20.0) y_t = 20.0;
    if(y_t < -20.0) y_t = -20.0;
		
    yawratecmd = (y_t/4.0)*(pg + df*(y_t-y_t_prev));
    y_t_prev = y_t;
    
    break;

  /** LEFT 2018 *************************************************************** LEFT **/
  case THIRD_STRAIGHT_ZONE:

    x0 = mXvalue+Virtual_point_dist*cos(mYawangle);
    y0 = mYvalue+Virtual_point_dist*sin(mYawangle);

    x1 = STRAIGT_03[0];
    y1 = STRAIGT_03[1];
    x2 = STRAIGT_03[2];
    y2 = STRAIGT_03[3];

    x12 = x2-x1;
    y12 = y2-y1;
    x10 = x0-x1;
    y10 = y0-y1;
    a   = (x12*y10)-(y12*x10);

    y_t = a/sqrt(pow(x12,2.0) + pow(y12,2.0));
    y_t = -1.0 * y_t; //20180530
		
    if(y_t > 20.0) y_t = 20.0;
    if(y_t < -20.0) y_t = -20.0;
		
    yawratecmd = (y_t/4.0)*(pg + df*(y_t-y_t_prev));
    y_t_prev = y_t;

    break;

  /** LEFT 2018 *************************************************************** LEFT **/
  case THIRD_CORNER_ZONE:

    x0  = mXvalue+Virtual_point_dist*cos(mYawangle);
    y0  = mYvalue+Virtual_point_dist*sin(mYawangle);

    x1  = CIRCLE_03[0];
    y1  = CIRCLE_03[1];

    a   = x1 - x0;
    b   = y1 - y0;
    a2  = a * a;
    b2  = b * b;
    r2  = a2 + b2;
    y_t = sqrt(r2) - CIRCLE_03[2];
    y_t = -1.0 * y_t; //20180530

    if(y_t > 20.0) y_t = 20.0;
    if(y_t < -20.0) y_t = -20.0;
		
    yawratecmd = (y_t/4.0)*(pg + df*(y_t-y_t_prev));
    y_t_prev = y_t;
    break;

  /** LEFT 2018 *************************************************************** LEFT **/
  case FOURTH_STRAIGHT_ZONE:

    x0  = mXvalue+Virtual_point_dist*cos(mYawangle);
    y0  = mYvalue+Virtual_point_dist*sin(mYawangle);

    x1  = STRAIGT_04[0];
    y1  = STRAIGT_04[1];
    x2  = STRAIGT_04[2];
    y2  = STRAIGT_04[3];

    x12 = x2-x1;
    y12 = y2-y1;
    x10 = x0-x1;
    y10 = y0-y1;
    a   = (x12*y10)-(y12*x10);

    y_t = a/sqrt(pow(x12,2.0) + pow(y12,2.0));
    y_t = -1.0 * y_t; //20180530
		
    if(y_t > 20.0) y_t = 20.0;
    if(y_t < -20.0) y_t = -20.0;
		
    yawratecmd = (y_t/4.0)*(pg + df*(y_t-y_t_prev));
    y_t_prev = y_t;
    break;

  /** LEFT 2018 *************************************************************** LEFT **/	
  case FOURTH_CORNER_ZONE:

    x0  = mXvalue+Virtual_point_dist*cos(mYawangle);
    y0  = mYvalue+Virtual_point_dist*sin(mYawangle);

    x1  = CIRCLE_04[0];
    y1  = CIRCLE_04[1];

    a   = x1 - x0;
    b   = y1 - y0;
    a2  = a * a;
    b2  = b * b;
    r2  = a2 + b2;
    y_t = sqrt(r2) - CIRCLE_04[2];
    //	  y_t = -1.0 * y_t; //20180530

    if(y_t > 20.0) y_t = 20.0;
    if(y_t < -20.0) y_t = -20.0;
		
    yawratecmd = (y_t/4.0)*(pg + df*(y_t-y_t_prev));
    y_t_prev = y_t;
    break;
    
    /** LEFT 2018 *************************************************************** LEFT **/	
  case GARAGE_ZONE:

    x0 = mXvalue+Virtual_point_dist*cos(mYawangle);
    y0 = mYvalue+Virtual_point_dist*sin(mYawangle);
    
    x1 = STRAIGT_05[0];
    y1 = STRAIGT_05[1];
    x2 = STRAIGT_05[2];
    y2 = STRAIGT_05[3];

    x12 = x2-x1;
    y12 = y2-y1;
    x10 = x0-x1;
    y10 = y0-y1;
    a   = (x12*y10)-(y12*x10);

    y_t = a/sqrt(pow(x12,2.0) + pow(y12,2.0));
    y_t = -1.0 * y_t; //20180530
		
    if(y_t > 20.0) y_t = 20.0;
    if(y_t < -20.0) y_t = -20.0;
		
    yawratecmd = (y_t/4.0)*(pg + df*(y_t-y_t_prev));
    y_t_prev = y_t;

    break;
    /** LEFT 2018 *************************************************************** LEFT **/	
  default:
    yawratecmd = 0.0;
    break;
  }
}

/** LEFT 2018 *************************************************************** LEFT **/	
/** LEFT 2018 *************************************************************** LEFT **/	
void Motion_Ctl::LUG_run(float odo, float angle,int line_value){
  static float   ref_forward;
  static float   ref_y;

  switch(LUG_MODE){
/** LEFT 2018 *************************************************************** LEFT **/	
  case LUG_START:
    forward    = 40;
    yawratecmd = 0;
    LUG_MODE    = APPROACH_TO_LUG;
    ref_forward = 40;
    break;

/** LEFT 2018 *************************************************************** LEFT **/	

  case APPROACH_TO_LUG:
    ref_forward = ref_forward-0.1; //modify later
    forward     = (int)(ref_forward + 0.5);

    if(forward < 0){
      forward = 0;
    }
    yawratecmd = 0;

  //    if(forward < 10){
  if((forward < 10)&&(mVelocity < 10)){
      LUG_MODE = TAIL_ON_1ST;
    }
    break;

/** LEFT 2018 *************************************************************** LEFT **/	
  case TAIL_ON_1ST:
    tail_stand_mode = true;
    tail_lug_mode  = false;

    forward    = 0;
    yawratecmd = 0;

    if(mRobo_balance_mode == false){
      forward    = 0;
      yawratecmd = 0;
      LUG_MODE   = LUG_MODE_1ST;

    }
    break;

/** LEFT 2018 *************************************************************** LEFT **/	
  case LUG_MODE_1ST:
    forward      = 0;
    ref_forward  = 0.0;
    yawratecmd   = 0;

    tail_lug_mode  = true;

    if(mRobo_lug_mode == true){
      LUG_MODE      = LUG_1ST;
      }
    break;

/** LEFT 2018 *************************************************************** LEFT **/	
  case LUG_1ST:

    ref_forward = ref_forward+0.1; //modify later
    forward     = (int)(ref_forward + 0.5);

    if(forward >= MAX_LUG_FORWARD){
      forward = MAX_LUG_FORWARD;
    }

    yawratecmd = LUG_YAW_GAIN*(0 - angle);//180701 kota

    if(mXvalue > LUG_1st_STOP_X){
      LUG_MODE    = LUG_MODE_2ND;
    }

    break;

/** LEFT 2018 *************************************************************** LEFT **/	
  case LUG_MODE_2ND:
    forward       = 0;
    yawratecmd    = 0;
    tail_lug_mode = true;

    if(mRobo_lug_mode == true){
      ref_forward  = 0.0;
      LUG_MODE    = LUG_2ND;
    }
    break;

/** LEFT 2018 *************************************************************** LEFT **/	
  case LUG_2ND:

    ref_forward = ref_forward-0.1; //180801
    forward     = (int)(ref_forward + 0.5);
    if(forward <= (-1.0*MAX_LUG_FORWARD)){
      forward = -1.0*MAX_LUG_FORWARD;
    }
    yawratecmd = 0; //180801

    if(mXvalue < LUG_2nd_STOP_X){
      LUG_MODE    = LUG_MODE_3RD; //180801
    }
    break;

/** LEFT 2018 *************************************************************** LEFT **/	
  case LUG_MODE_3RD:
    forward      = 0;
    yawratecmd   = 0;
    tail_lug_mode  = true;

    if(mRobo_lug_mode == true){
      ref_forward  = 0.0;
      LUG_MODE     = LUG_3RD;
    }
    break;

/** LEFT 2018 *************************************************************** LEFT **/	
  case LUG_3RD:

    ref_forward = ref_forward+0.1; //modify later
    forward     = (int)(ref_forward + 0.5);

    if(forward >= MAX_LUG_FORWARD){
      forward = MAX_LUG_FORWARD;
    }
    yawratecmd = LUG_YAW_GAIN*(0 - angle);

    if(mXvalue > LUG_3rd_STOP_X){
      LUG_MODE    = TAIL_STAND_UP;
    }
    break;

/** LEFT 2018 *************************************************************** LEFT **/	
  case TAIL_STAND_UP:
    forward       = 0;
    yawratecmd    = 0;
    tail_lug_mode = false;

    if(mRobo_lug_mode == false){
      ref_forward = 0.0;
      LUG_MODE    =  TURN_90;
    }
    break;

/** LEFT 2018 *************************************************************** LEFT **/	
  case TURN_90:
    forward    = 0;
    yawratecmd = LUG_YAW_GAIN*(RAD_90_DEG - angle);
    if(angle >  (RAD_90_DEG - RAD_1_DEG)){
      LUG_MODE   =  FIND_GREEN;
      ref_forward = 0.0;
    }
    break;

/** LEFT 2018 *************************************************************** LEFT **/	
  case FIND_GREEN:

    if(mGreen_flag &&(mYvalue > 200)){
      forward = 0;
      LUG_MODE   =  TURN_M90;
    }else{
      forward = 10;
    }
    yawratecmd = LUG_YAW_GAIN*(RAD_90_DEG - angle);
    break;

/** LEFT 2018 *************************************************************** LEFT **/	
  case TURN_M90:
    forward    = 0;
    yawratecmd = LUG_YAW_GAIN*(MINUS_RAD_90_DEG - angle);
    if(angle <  (MINUS_RAD_90_DEG + RAD_1_DEG)){
      LUG_MODE   =  FIND_LEFT_EDGE;
      ref_forward = 0.0;
      ref_y = mYvalue - 50;
    }
    break;

/** LEFT 2018 *************************************************************** LEFT **/	
  case FIND_LEFT_EDGE:

    //    if(mGreen_flag && (mYvalue < 200)){
    
    //    if((mLinevalue > 50) && (mYvalue < 200)){
    //    if((mLinevalue > TAIL_STD_LINE_DET) && (mYvalue < 200)){
    if((mLinevalue > TAIL_STD_LINE_DET) && (mYvalue < ref_y)){
      ref_y = mYvalue - 50;
      forward = 0;
      LUG_MODE   =  Y_POS_ADJ;
    }else if(mYvalue < (ref_y - 150) ){
      ref_forward = 0.0;
      LUG_MODE   =  TURN_ZERO;
      forward = 0;
    }else{
      forward = 10;
    }
    yawratecmd = LUG_YAW_GAIN*(MINUS_RAD_90_DEG - angle);
    break;

/** LEFT 2018 *************************************************************** LEFT **/	
  case Y_POS_ADJ:
    forward    = 10;
    yawratecmd = LUG_YAW_GAIN*(MINUS_RAD_90_DEG - angle);
    if(mYvalue < ref_y){
      ref_forward = 0.0;
      LUG_MODE   =  TURN_ZERO;
    }
    break;

/** LEFT 2018 *************************************************************** LEFT **/	
  case TURN_ZERO:
    forward    = 0;
    yawratecmd = LUG_YAW_GAIN*(0 - angle);

    if(angle >  (-1.0*RAD_1_DEG)){
      LUG_MODE   =  GRAY_GARAGE;
      ref_forward = 0.0;
    }
    break;

/** LEFT 2018 *************************************************************** LEFT **/	
  case GRAY_GARAGE:

    ref_forward = ref_forward+0.1; //modify later
    forward     = (int)(ref_forward + 0.5);

    if(forward >= MAX_LUG_FORWARD){
      forward = MAX_LUG_FORWARD;
    }

    if(mXvalue > GARAGE_X_POS){
      forward = 0;
    }

    //    yawratecmd = LUG_YAW_GAIN*(0 - angle);
    yawratecmd = LUG_YAW_GAIN*((-1.0*RAD_1_DEG)- angle);
    break;


/** LEFT 2018 *************************************************************** LEFT **/	
  default:
    forward      = 0;
    yawratecmd    = 0;
    ref_tail_angle = TAIL_ANGLE_RUN; //0817 tada
    tail_stand_mode = false;
    break;

    }
}


