/******************************************************************************
 *  ang_brain.cpp (for LEGO Mindstorms EV3)
 *  Created on: 2018/04/22
 *  Implementation of the Class Ang_Robo
 *  Author: Koaru Ota
 *  https://github.com/KaoruOta/anago2018_beta
 *  https://drive.google.com/open?id=1Ih-fZX68pgRuQWVPbCW-5Oj-Bgrn3pgo

--- basic structure ---

void Ang_Brain::run()

void Ang_Brain::det_navigation() {

  if(DRIVE_MODE == LINE_TRACE){

    switch(ZONE){

#ifdef LEFT_2018
    case START_ZONE:
    case FIRST_STRAIGHT_ZONE:
    case ENTER_1ST_CORNER_ZONE:
    case FIRST_CORNER_ZONE:
    case SECOND_STRAIGHT_ZONE:
    case ENTER_2ND_CORNER_ZONE:
    case SECOND_CORNER_ZONE:
    case THIRD_STRAIGHT_ZONE:
    case THIRD_CORNER_ZONE:
    case FOURTH_STRAIGHT_ZONE:
    case FOURTH_CORNER_ZONE:
    case FIRST_GRAY_ZONE:
    case LUG_ZONE:
    case SECOND_GRAY_ZONE:
    case GARAGE_ZONE:
    case LOST:

#ifdef ET_2018_RIGHT
    case START_ZONE:
    ....same as LEFT
}
else if(DRIVE_MODE == TRACK){

#ifdef LEFT_2018
    switch(ZONE){
    case START_ZONE:
    case FIRST_STRAIGHT_ZONE:
    case ENTER_1ST_CORNER_ZONE:
    case FIRST_CORNER_ZONE:
    case SECOND_STRAIGHT_ZONE:
    case ENTER_2ND_CORNER_ZONE:
    case SECOND_CORNER_ZONE:
    case THIRD_STRAIGHT_ZONE:
    case THIRD_CORNER_ZONE:
    case FOURTH_STRAIGHT_ZONE:
    case FOURTH_CORNER_ZONE:
    case FIRST_GRAY_ZONE:
    case LUG_ZONE:
    case SECOND_GRAY_ZONE:
    case GARAGE_ZONE:
    case LOST:

#ifdef ET_2018_RIGHT
    switch(ZONE){
    case START_ZONE:
    ....same as LEFT
}

else if(DRIVE_MODE == DEBUG){}

*****************************************************************************/

#include <stdlib.h>
#include "ev3api.h"
#include "ang_brain.hpp"
#define liting_radius 10; // liting spot radius [mm]


Ang_Brain::Ang_Brain() {

}

void Ang_Brain::init() {
  ZONE               = START_ZONE;
  DRIVE_MODE         = LINE_TRACE;
  ON_LINE_MODE       = ON_THE_LEFT_EDGE;
  PRE_ON_LINE_MODE   = ON_THE_LEFT_EDGE;
  
  on_line            = true;
  left_line          = false;
  right_line         = false;
  lost_line          = false;
  line_to_map        = false; //181108

  line_trace_mode    = true;
  forward_curve_mode = false;

  gAve_line_val->init();
  gAve_yaw_angle_500->init(); //20181108

  gMotion_Ctl->init();

  mMax_Forward = 10;

  det_navi_log = 0;
}


void Ang_Brain::set_drive_mode_LT(){
  DRIVE_MODE = LINE_TRACE;
  line_trace_mode = true;
}

void Ang_Brain::set_drive_mode_TK(){
  DRIVE_MODE = TRACK;
  line_trace_mode = false;
}

void Ang_Brain::set_drive_mode_DB(){
  DRIVE_MODE = DEBUG;
  line_trace_mode = false;
}



void Ang_Brain::run() {

  ave_line_val = gAve_line_val->average_500(mLinevalue);
  ave_yaw_angle_500 = gAve_yaw_angle_500->average_500(mYawangle);

  //  det_on_line();

  det_navigation();

  gMotion_Ctl->SetCurrentData(mLinevalue,
			      mGreen_flag,
			      mXvalue,
			      mYvalue,
			      mPre_50mm_x,
			      mPre_50mm_y,
			      mOdo,
			      mVelocity,
			      mYawrate,
			      mYawangle,
			      mTail_angle,
			      mRobo_stop,
			      mRobo_forward,
			      mRobo_back,
			      mRobo_turn_left,
			      mRobo_turn_right,
			      mDansa,
			      mSonar_dis,
			      mRobo_balance_mode,
			      mRobo_lug_mode,
			      mMax_Forward,
			      mRef_Yawrate,
			      mMax_Yawrate,
			      mMin_Yawrate
			      );

  gMotion_Ctl->run(mXvalue,mYvalue,mYawangle);


  GetCalcResult(gMotion_Ctl->forward,
		gMotion_Ctl->yawratecmd,
		gMotion_Ctl->ref_tail_angle,
		gMotion_Ctl->tail_stand_mode,
		gMotion_Ctl->tail_lug_mode,
		gMotion_Ctl->rising_seesaw,
		gMotion_Ctl->falling_seesaw);
}

/****************************************************************************************/
//2018 04 21 Kaoru Ota
//    //https://drive.google.com/open?id=1Ih-fZX68pgRuQWVPbCW-5Oj-Bgrn3pgo
/****************************************************************************************/
void Ang_Brain::det_navigation() {

  float yaw_time;
  static float ref_odo;
  static float dif_odo;

  static int   ref_forward;
  static float acl_forward;

    //https://drive.google.com/open?id=1Ih-fZX68pgRuQWVPbCW-5Oj-Bgrn3pgo
  if(DRIVE_MODE == LINE_TRACE){
    //    forward_curve_mode = true; //20181008 for CS

    switch(ZONE){

/** LEFT 2018 **********************************************************LINE TRACE **/      
    case START_ZONE:
      det_navi_log = 1000;
      gMotion_Ctl->set_mode_LT();

      //MAX_FORWARD GEN--------------------------------------------------------------
      ref_odo = mOdo;

      if(ref_odo < 0){
	ref_odo = 0;
      }

      if(mPre_velo_0p5sec > MAX_VELOCITY){
	mMax_Forward = 	mMax_Forward;
      }else{
	acl_forward = 10 + (ref_odo * ACCEL_GAIN);
	mMax_Forward = (int)acl_forward;
      }

      if(mMax_Forward > START_FORWARD_VAL){
	mMax_Forward = START_FORWARD_VAL;
      }
      //--------------------------------------------------------------MAX_FORWARD GEN

      //REF YAW RATE GEN-------------------------------------------------------------
      if(mVelocity > 10){
	mMin_Yawrate = MINUS_RAD_22P5_DEG;
	mRef_Yawrate = 0;
	mMax_Yawrate = RAD_22P5_DEG;
      }else{
	mMin_Yawrate = 0;
	mRef_Yawrate = 0;
	mMax_Yawrate = 0;
      }
      //-------------------------------------------------------------REF YAW RATE GEN


      //DET RUNNING ARE-------------------------------------------------------------
      if (mPre_50mm_x > FIRST_STRAIGHT_AREA[0]){

	ZONE = FIRST_STRAIGHT_ZONE;
	gMotion_Ctl->set_mode_LT();
	gMotion_Ctl->set_zone_1st_straight();

	//FOR MAX_FORWARD GEN--------------------------------------------------------
	ref_forward = mMax_Forward;
	ref_odo     = mOdo;
	//--------------------------------------------------------FOR MAX_FORWARD GEN


      //DET START FAIL
      }else if(mXvalue < START_AREA[0]){
	ZONE = START_BACK;

      }else if ((mAve_yaw_angle < MINUS_RAD_22P5_DEG)||(mAve_yaw_angle > RAD_22P5_DEG)){ //LOST THE LINE
	/*
	DRIVE_MODE = TRACK;
	ZONE = FIRST_STRAIGHT_ZONE;
	gMotion_Ctl->set_mode_map_trace();
	gMotion_Ctl->set_zone_1st_straight();

	mMax_Forward = 50;

	mMin_Yawrate = -1.0 * RAD_6_DEG;
	mMax_Yawrate = RAD_6_DEG;
	*/

      }
      break;


/** LEFT 2018 **********************************************************LINE TRACE **/
    case START_BACK:
      det_navi_log = 1010;
      gMotion_Ctl->set_mode_map_trace();
      gMotion_Ctl->set_zone_1st_straight();
      //      mMax_Forward = 10;
      //      mMax_Forward = 30;
      mMax_Forward = 70;

      //DET RE-START
      if(mXvalue > 360){
	ZONE = START_ZONE;
      }
      break;


/** LEFT 2018 **********************************************************LINE TRACE **/
    case FIRST_STRAIGHT_ZONE:
      det_navi_log = 1020;

      //MAX_FORWARD GEN--------------------------------------------------------------
      dif_odo = mOdo - ref_odo;

      if(dif_odo < 0){
	dif_odo = 0;
      }

      if(mPre_velo_0p5sec > MAX_VELOCITY){
	mMax_Forward = 	mMax_Forward;
      }else{
	acl_forward  = ref_forward + (dif_odo * ACCEL_GAIN);
	mMax_Forward = (int)acl_forward;
      }

      if(mMax_Forward > FIRST_STRAIGHT_FORWARD_VAL){
	mMax_Forward =  FIRST_STRAIGHT_FORWARD_VAL;
      }
      //--------------------------------------------------------------MAX_FORWARD GEN


      //REF YAW RATE GEN-------------------------------------------------------------
      mMin_Yawrate = MINUS_RAD_22P5_DEG;
      mRef_Yawrate = 0;
      mMax_Yawrate = RAD_22P5_DEG;
      //-------------------------------------------------------------REF YAW RATE GEN


      //DET LINE_LOST ARE-------------------------------------------------------------
      //      ave_yaw_angle_500 = gAve_yaw_angle_500->average_500(mYawangle);

      //DET RUNNING ARE-------------------------------------------------------------
      if (mPre_50mm_x > ENTER_1ST_CORNER_AREA[0]){

	ZONE = ENTER_1ST_CORNER_ZONE;

	//FOR MAX_FORWARD GEN--------------------------------------------------------
	ref_forward = mMax_Forward;
	ref_odo     = mOdo;
	//--------------------------------------------------------FOR MAX_FORWARD GEN

      //DET LINE_LOST ARE-------------------------------------------------------------
      }else if( (ave_yaw_angle_500 < MINUS_RAD_22P5_DEG)|| (mYvalue < (STRAIGT_01[1]- 150)) ){
	//      }else if( (ave_yaw_angle_500 < -0.05) ){
	//      }else if( (ave_yaw_angle_500 < MINUS_RAD_22P5_DEG)|| (mYvalue < (STRAIGT_01[1]- 150)) ){
	//      }else if ((mAve_yaw_angle < MINUS_RAD_22P5_DEG)||(mAve_yaw_angle > RAD_22P5_DEG)){ //LOST THE LINE
	/*
	line_to_map = true;
	DRIVE_MODE  = TRACK;
	ZONE = FIRST_STRAIGHT_ZONE;
	gMotion_Ctl->set_mode_map_trace();
	gMotion_Ctl->set_zone_1st_straight();
	*/

      }
      break;


/** LEFT 2018 **********************************************************LINE TRACE **/
    case ENTER_1ST_CORNER_ZONE:
      det_navi_log = 1030;

      //20181008----- for CS
      /*
	mMax_Forward = ENTER_1ST_CORNER_FORWARD_VAL;
      */

      dif_odo = mOdo - ref_odo;

      if(dif_odo < 0){
	dif_odo = 0;
      }

      if(mPre_velo_0p5sec > MAX_VELOCITY){
	mMax_Forward = 	mMax_Forward;
      }else{
	acl_forward  = ref_forward + (dif_odo * ACCEL_GAIN);
	mMax_Forward = (int)acl_forward;
      }

      if(mMax_Forward > ENTER_1ST_CORNER_FORWARD_VAL){
	mMax_Forward =  ENTER_1ST_CORNER_FORWARD_VAL;
      }

      //----20181008----- for CS

      mMin_Yawrate = -1.0 * RAD_6_DEG;
      mRef_Yawrate = 0.0;
      yaw_time     = CIRCLE_01_LENGTH/mVelocity;
      mMax_Yawrate = (CIRCLE_01_ANGLE/yaw_time) + RAD_22P5_DEG;

      if (mPre_50mm_x >FIRST_CORNER_AREA[0]){

	ZONE = FIRST_CORNER_ZONE;
	gMotion_Ctl->set_zone_1st_corner();

	//20181008----- for CS
	ref_forward = mMax_Forward;
	ref_odo     = mOdo;
	//----20181008----- for CS

      }else if ((mAve_yaw_angle < MINUS_RAD_22P5_DEG)||(mAve_yaw_angle > RAD_45_DEG)){ //LOST THE LINE
	/*
	DRIVE_MODE = TRACK;
	ZONE       = FIRST_CORNER_ZONE;
	gMotion_Ctl->set_mode_map_trace();
	gMotion_Ctl->set_zone_1st_corner();
	*/
      }
      break;

/** LEFT 2018 **********************************************************LINE TRACE **/
    case FIRST_CORNER_ZONE:
      det_navi_log = 1040;
      //20181008----- for CS
      /*
      mMax_Forward = FIRST_CORNER_FORWARD_VAL;
      */

      dif_odo = mOdo - ref_odo;
      if(dif_odo < 0){
	dif_odo = 0;
      }

      if(mPre_velo_0p5sec > MAX_VELOCITY){
	mMax_Forward = 	mMax_Forward;
      }else{
	//	mMax_Forward = ref_forward + (dif_odo * ACCEL_GAIN);
	acl_forward  = ref_forward + (dif_odo * ACCEL_GAIN);
	mMax_Forward = (int)acl_forward;
      }

      if(mMax_Forward > FIRST_CORNER_FORWARD_VAL){
	mMax_Forward =  FIRST_CORNER_FORWARD_VAL;
      }

      //----20181008----- for CS



      yaw_time     = CIRCLE_01_LENGTH/mVelocity;
      mRef_Yawrate = CIRCLE_01_ANGLE/yaw_time;
      mMin_Yawrate = mRef_Yawrate - RAD_22P5_DEG;
      mMax_Yawrate = mRef_Yawrate + RAD_45_DEG;

      if (mPre_50mm_y > SECOND_STRAIGHT_AREA[1]){
	ZONE = SECOND_STRAIGHT_ZONE;
	gMotion_Ctl->set_zone_2nd_straight();

	//20181008----- for CS
	ref_forward = mMax_Forward;
	ref_odo     = mOdo;
	//----20181008----- for CS

      }else if ((mAve_yaw_angle < MINUS_RAD_22P5_DEG)||(mAve_yaw_angle > RAD_120_DEG)){ //LOST THE LINE -> MAP TRACE MODE
	/*
	DRIVE_MODE = TRACK;
	gMotion_Ctl->set_mode_map_trace();

	ZONE       = FIRST_CORNER_ZONE;
	gMotion_Ctl->set_zone_1st_corner();
	*/
      }
      break;

/** LEFT 2018 **********************************************************LINE TRACE **/
    case SECOND_STRAIGHT_ZONE:
      det_navi_log = 1050;

      //20181008----- for CS
      /*
      mMax_Forward = SECOND_STRAIGHT_FORWARD_VAL;
      */
      dif_odo = mOdo - ref_odo;
      if(dif_odo < 0){
	dif_odo = 0;
      }

      if(mPre_velo_0p5sec > MAX_VELOCITY){
	mMax_Forward = 	mMax_Forward;
      }else{
	//	mMax_Forward = ref_forward + (dif_odo * ACCEL_GAIN);
	acl_forward  = ref_forward + (dif_odo * ACCEL_GAIN);
	mMax_Forward = (int)acl_forward;
      }

      if(mMax_Forward > SECOND_STRAIGHT_FORWARD_VAL){
	mMax_Forward = SECOND_STRAIGHT_FORWARD_VAL;
      }



      //----20181008----- for CS

      mMin_Yawrate = MINUS_RAD_22P5_DEG;
      mRef_Yawrate = 0;
      mMax_Yawrate = RAD_22P5_DEG;


      if (mPre_50mm_y > ENTER_2ND_CORNER_AREA[1]){

      	ZONE = ENTER_2ND_CORNER_ZONE;
      	gMotion_Ctl->set_zone_enter_2nd_corner();

	//20181008----- for CS
	ref_forward = mMax_Forward;
	ref_odo     = mOdo;
	//----20181008----- for CS


      }else if ((mAve_yaw_angle <  RAD_45_DEG) || (mAve_yaw_angle > RAD_120_DEG)){ //LOST THE LINE -> MAP TRACE MODE
	/*
	DRIVE_MODE = TRACK;
	gMotion_Ctl->set_mode_map_trace();

	ZONE = SECOND_STRAIGHT_ZONE;
	gMotion_Ctl->set_zone_2nd_straight();
	*/

      }
      break;
      
/** LEFT 2018 **********************************************************LINE TRACE **/
    case ENTER_2ND_CORNER_ZONE:
      det_navi_log = 1060;


      //20181008----- for CS
      /*
	mMax_Forward = ENTER_2ND_CORNER_FORWARD_VAL;
      */
      dif_odo = mOdo - ref_odo;
      if(dif_odo < 0){
	dif_odo = 0;
      }

      if(mPre_velo_0p5sec > MAX_VELOCITY){
	mMax_Forward = 	mMax_Forward;
      }else{
	//	mMax_Forward = ref_forward + (dif_odo * ACCEL_GAIN);
	acl_forward  = ref_forward + (dif_odo * ACCEL_GAIN);
	mMax_Forward = (int)acl_forward;
      }

      if(mMax_Forward > ENTER_2ND_CORNER_FORWARD_VAL){
	mMax_Forward = ENTER_2ND_CORNER_FORWARD_VAL;
      }

      //----20181008----- for CS


      yaw_time     = CIRCLE_02_LENGTH/mVelocity;
      mMin_Yawrate = (CIRCLE_02_ANGLE/yaw_time) - RAD_22P5_DEG;
      mRef_Yawrate = 0.0;
      mMax_Yawrate = RAD_22P5_DEG;

      if (mPre_50mm_y > SECOND_CORNER_AREA[1]){

	ZONE = SECOND_CORNER_ZONE;
	gMotion_Ctl->set_zone_2nd_corner();

	//20181008----- for CS
	ref_forward = mMax_Forward;
	ref_odo     = mOdo;
	//----20181008----- for CS

      }else if ((mAve_yaw_angle <  RAD_45_DEG) || (mAve_yaw_angle > RAD_120_DEG)){ //LOST THE LINE -> MAP TRACE MODE
	/*
	DRIVE_MODE = TRACK;
	gMotion_Ctl->set_mode_map_trace();

	ZONE = SECOND_CORNER_ZONE;
	gMotion_Ctl->set_zone_2nd_corner();
	*/

      }
      break;

/** LEFT 2018 **********************************************************LINE TRACE **/
    case SECOND_CORNER_ZONE:
      det_navi_log = 1070;


      //20181008----- for CS
      /*
	mMax_Forward = SECOND_CORNER_FORWARD_VAL;
      */

      dif_odo = mOdo - ref_odo;
      if(dif_odo < 0){
	dif_odo = 0;
      }

      if(mPre_velo_0p5sec > MAX_VELOCITY){
	mMax_Forward = 	mMax_Forward;
      }else{
	//	mMax_Forward = ref_forward + (dif_odo * ACCEL_GAIN);
	acl_forward  = ref_forward + (dif_odo * ACCEL_GAIN);
	mMax_Forward = (int)acl_forward;
      }

      if(mMax_Forward > SECOND_CORNER_FORWARD_VAL){
	mMax_Forward = SECOND_CORNER_FORWARD_VAL;
      }

      //----20181008----- for CS


      yaw_time     = CIRCLE_02_LENGTH/mVelocity;
      mRef_Yawrate = CIRCLE_02_ANGLE/yaw_time;

      mMin_Yawrate = mRef_Yawrate - RAD_22P5_DEG;
      mRef_Yawrate = 0;
      mMax_Yawrate = mRef_Yawrate + RAD_22P5_DEG;

      if (mPre_50mm_x > THIRD_STRAIGHT_AREA[0]){

	ZONE = THIRD_STRAIGHT_ZONE;
	gMotion_Ctl->set_zone_3rd_straight();

	//20181008----- for CS
	ref_forward = mMax_Forward;
	ref_odo     = mOdo;
	//----20181008----- for CS

      }else if ((mAve_yaw_angle < MINUS_RAD_22P5_DEG) || (mAve_yaw_angle > RAD_120_DEG)){ //LOST THE LINE -> MAP TRACE MODE
	mMax_Forward = 0;
	/*
	DRIVE_MODE = TRACK;
	gMotion_Ctl->set_mode_map_trace();

	ZONE = SECOND_CORNER_ZONE;
	gMotion_Ctl->set_zone_2nd_corner();
	*/
      }
      break;

/** LEFT 2018 **********************************************************LINE TRACE **/
    case THIRD_STRAIGHT_ZONE:
      det_navi_log = 1080;
      //20181008----- for CS
      /*
	mMax_Forward = THIRD_STRAIGHT_FORWARD_VAL;
      */
      dif_odo = mOdo - ref_odo;

      if(dif_odo < 0){
	dif_odo = 0;
      }

      if(mPre_velo_0p5sec > MAX_VELOCITY){
	mMax_Forward = 	mMax_Forward;
      }else{
	//	mMax_Forward = ref_forward + (dif_odo * ACCEL_GAIN);
	acl_forward  = ref_forward + (dif_odo * ACCEL_GAIN);
	mMax_Forward = (int)acl_forward;
      }

      if(mMax_Forward > THIRD_STRAIGHT_FORWARD_VAL){
	mMax_Forward = THIRD_STRAIGHT_FORWARD_VAL;
      }

      //----20181008----- for CS




      yaw_time     = CIRCLE_02_LENGTH/mVelocity;
      mMin_Yawrate = CIRCLE_02_ANGLE/yaw_time;

      mRef_Yawrate = 0;
      mMax_Yawrate = RAD_22P5_DEG;

      if (mPre_50mm_x > THIRD_CORNER_AREA[0]){

	ZONE = THIRD_CORNER_ZONE;
	gMotion_Ctl->set_zone_3rd_corner();

	//20181008----- for CS
	ref_forward = mMax_Forward;
	ref_odo     = mOdo;
	//----20181008----- for CS

      }else if ((mAve_yaw_angle < MINUS_RAD_22P5_DEG) || (mAve_yaw_angle > RAD_22P5_DEG)){ //LOST THE LINE -> MAP TRACE MODE
	mMax_Forward = 0;
	/*
	DRIVE_MODE = TRACK;
	gMotion_Ctl->set_mode_map_trace();

	ZONE = THIRD_STRAIGHT_ZONE;
	gMotion_Ctl->set_zone_3rd_straight();
	*/
      }
      break;


/** LEFT 2018 **********************************************************LINE TRACE **/
    case THIRD_CORNER_ZONE:
      det_navi_log = 1090;

      //20181008----- for CS
      /*
	mMax_Forward = THIRD_CORNER_FORWARD_VAL;
      */

      dif_odo = mOdo - ref_odo;
      if(dif_odo < 0){
	dif_odo = 0;
      }

      if(mPre_velo_0p5sec > MAX_VELOCITY){
	mMax_Forward = 	mMax_Forward;
      }else{
	//	mMax_Forward = ref_forward + (dif_odo * ACCEL_GAIN);
	acl_forward  = ref_forward + (dif_odo * ACCEL_GAIN);
	mMax_Forward = (int)acl_forward;
      }

      if(mMax_Forward > THIRD_CORNER_FORWARD_VAL){
	mMax_Forward = THIRD_CORNER_FORWARD_VAL;
      }


      //----20181008----- for CS



      yaw_time     = CIRCLE_03_LENGTH/mVelocity;
      mRef_Yawrate = CIRCLE_03_ANGLE/yaw_time;
      
      mMin_Yawrate = mRef_Yawrate - RAD_22P5_DEG;
      mRef_Yawrate = 0;
      mMax_Yawrate = mRef_Yawrate + RAD_22P5_DEG;

      if (mPre_50mm_y < (FOURTH_STRAIGHT_AREA[3] - 110 )){
	ZONE = FOURTH_STRAIGHT_ZONE;
	gMotion_Ctl->set_zone_4th_straight();
	//	ref_odo = mOdo;

	//20181008----- for CS
	ref_forward = mMax_Forward;
	ref_odo     = mOdo;
	//----20181008----- for CS

      }else if ((mAve_yaw_angle < MINUS_RAD_145_DEG ) || (mAve_yaw_angle > RAD_22P5_DEG)){ //LOST THE LINE -> MAP TRACE MODE
	/*
	DRIVE_MODE = TRACK;
	gMotion_Ctl->set_mode_map_trace();

	ZONE = THIRD_CORNER_ZONE;
	gMotion_Ctl->set_zone_3rd_corner();
	*/

      }
      break;


/** LEFT 2018 **********************************************************LINE TRACE **/
    case FOURTH_STRAIGHT_ZONE:
      det_navi_log = 1100;


      //20181008----- for CS
      /*
      if(mPre_50mm_y < 1500){
	mMax_Forward = FOURTH_CORNER_FORWARD_VAL;
      }else if(mPre_50mm_y < 2500){
	mMax_Forward = FOURTH_STRAIGHT_FORWARD_VAL;
      }else{
	mMax_Forward = 100;
      }
      */

      dif_odo = mOdo - ref_odo;
      if(dif_odo < 0){
	dif_odo = 0;
      }

      if(mPre_velo_0p5sec > MAX_VELOCITY){
	mMax_Forward = 	mMax_Forward;
      }else{
	//	mMax_Forward = ref_forward + (dif_odo * ACCEL_GAIN);
	acl_forward  = ref_forward + (dif_odo * ACCEL_GAIN);
	mMax_Forward = (int)acl_forward;
      }

      if(mMax_Forward > FOURTH_STRAIGHT_FORWARD_VAL){
	mMax_Forward = FOURTH_STRAIGHT_FORWARD_VAL;
      }
      

      if (mPre_50mm_y < FOURTH_CORNER_AREA[3] + 300){
	mMax_Forward = GOAL_VAL;
      }

      //----20181008----- for CS


      mMin_Yawrate = MINUS_RAD_22P5_DEG;
      mRef_Yawrate = 0;
      mMax_Yawrate = RAD_22P5_DEG;

      if (mPre_50mm_y < FOURTH_CORNER_AREA[3]){

	//***********************************************************/
	//Change from line trace to Map trace  20180701 kota
	//***********************************************************/
	/*
	ZONE = FOURTH_CORNER_ZONE;
	gMotion_Ctl->set_zone_4th_corner();
	
	DRIVE_MODE = TRACK;
	gMotion_Ctl->set_mode_map_trace();

	ZONE = FOURTH_CORNER_ZONE;
	gMotion_Ctl->set_zone_4th_corner();
	*/
	//***********************************************************/
	//Change from line trace to Map trace  20180701 kota
	//***********************************************************/

	ZONE = FOURTH_CORNER_ZONE;
	gMotion_Ctl->set_zone_4th_corner();

	//20181008----- for CS
	ref_forward = mMax_Forward;
	ref_odo     = mOdo;
	//----20181008----- for CS


      }else if ((mAve_yaw_angle < MINUS_RAD_145_DEG ) || (mAve_yaw_angle > MINUS_RAD_90_DEG)){ //LOST THE LINE -> MAP TRACE MODE

	DRIVE_MODE = TRACK;
	gMotion_Ctl->set_mode_map_trace();

	ZONE = FOURTH_STRAIGHT_ZONE;
	gMotion_Ctl->set_zone_4th_straight();

      }
      break;


/** LEFT 2018 **********************************************************LINE TRACE **/
      case FOURTH_CORNER_ZONE:
	det_navi_log = 1110;

	mMax_Forward = FOURTH_CORNER_FORWARD_VAL;

	//20180714 kota
	yaw_time     = CIRCLE_04_LENGTH/300;
	mRef_Yawrate = CIRCLE_04_ANGLE/yaw_time;
	mMax_Yawrate = mRef_Yawrate + RAD_22P5_DEG;
	mRef_Yawrate = 0;

	if(mAve_yaw_angle > 0){
	  mMin_Yawrate = MINUS_RAD_5_DEG;
	}else{
	  mMin_Yawrate = 0;
	}

	if (  (mXvalue > FIRST_GRAY_AREA[0])&&(mYvalue < FIRST_GRAY_AREA[3])){
	  gMotion_Ctl->set_mode_LUG();
	  ZONE = LUG_ZONE;
	  gMotion_Ctl->set_zone_garage();
	  //20181008----- for CS
	  ref_forward = mMax_Forward;
	  ref_odo     = mOdo;
	  //----20181008----- for CS

	}else if ((mAve_yaw_angle < MINUS_RAD_145_DEG ) || (mAve_yaw_angle > RAD_22P5_DEG)){ //LOST THE LINE -> MAP TRACE MODE

	  DRIVE_MODE = TRACK;
	  gMotion_Ctl->set_mode_map_trace();

	  ZONE = FOURTH_CORNER_ZONE;
	  gMotion_Ctl->set_zone_4th_corner();

	}
	break;


/** LEFT 2018 **********************************************************LINE TRACE **/
    case FIRST_GRAY_ZONE:
      det_navi_log = 1120;
      forward_curve_mode = true;
      mMax_Forward = FIRST_GRAY_FORWARD_VAL;

      mMin_Yawrate = MINUS_RAD_5_DEG;
      mRef_Yawrate = 0;
      mMax_Yawrate = RAD_5_DEG;

      if (mPre_50mm_x > LUG_AREA[0]){
	ZONE = LUG_ZONE;
	//	gMotion_Ctl->set_mode_lug(); //it will be added later 180624 kota
	//	gMotion_Ctl->set_zone_lug();

      }else if ((mAve_yaw_angle < MINUS_RAD_22P5_DEG)||(mAve_yaw_angle > RAD_22P5_DEG)){ //LOST THE LINE
	mMax_Forward = 0;
      }
      break;


/** LEFT 2018 **********************************************************LINE TRACE **/
    case LUG_ZONE:
      det_navi_log = 1130;
      mMax_Forward = 0;

      break;


/** LEFT 2018 **********************************************************LINE TRACE **/
    case SECOND_GRAY_ZONE:
      det_navi_log = 1140;
      break;



/** LEFT 2018 **********************************************************LINE TRACE **/
    case GARAGE_ZONE:
      det_navi_log = 1150;
      break;

/** LEFT 2018 **********************************************************LINE TRACE **/
    case LOST:
      det_navi_log = 1160;
	mMax_Forward = 0;
	mRef_Yawrate = 0;

	gMotion_Ctl->set_zone_lost();

      break;


    default:
      break;
    }
/**2018 L course ***************************/

/**************************************************************************************************************/
/**  TRACK MODE                                                                                              **/
/**  TRACK MODE                                                                                              **/
/**  TRACK MODE                                                                                              **/
/**  TRACK MODE                                                                                              **/
/**************************************************************************************************************/
  }else if(DRIVE_MODE == TRACK){
    //    forward_curve_mode = true;
    forward_curve_mode = false;
    line_trace_mode    = false;

/********************************************/
/* 2018 Left course*/
/********************************************/

    switch(ZONE){

/** LEFT 2018 *************************************************************** TRACK **/
    case START_ZONE:
      det_navi_log = 2000;
      gMotion_Ctl->set_mode_LT();

      ref_odo = mOdo;

      if(ref_odo < 0){
	ref_odo = 0;
      }

      if(mPre_velo_0p5sec > MAX_VELOCITY){
	mMax_Forward = 	mMax_Forward;
      }else{
	acl_forward = 10 + (ref_odo * ACCEL_GAIN);
	mMax_Forward = (int)acl_forward;
      }

      if(mMax_Forward > START_FORWARD_VAL){
	mMax_Forward = START_FORWARD_VAL;
      }

      if(mVelocity > 10){
	mMin_Yawrate = MINUS_RAD_22P5_DEG;
	mRef_Yawrate = 0;
	mMax_Yawrate = RAD_22P5_DEG;
      }else{
	mMin_Yawrate = 0;
	mRef_Yawrate = 0;
	mMax_Yawrate = 0;
      }

      if (mPre_50mm_x > FIRST_STRAIGHT_AREA[0]){

	ref_forward = mMax_Forward;
	ref_odo     = mOdo;

	ZONE = FIRST_STRAIGHT_ZONE;
	gMotion_Ctl->set_mode_map_trace();
	gMotion_Ctl->set_zone_1st_straight();

	//	mMax_Forward = 100;
	//	mMax_Yawrate =  RAD_6_DEG;
	//	mMin_Yawrate = -1.0 * RAD_6_DEG;

      }else if(mXvalue < START_AREA[0]){
	ZONE = START_BACK;

      }else if ((mAve_yaw_angle < MINUS_RAD_22P5_DEG)||(mAve_yaw_angle > RAD_22P5_DEG)){

      }

      //---- 1108 ----///////////////////////////////////
      /*
      gMotion_Ctl->set_mode_LT();

      mMax_Forward = START_FORWARD_VAL;

      mMax_Yawrate = RAD_6_DEG;
      mMin_Yawrate = -1.0 * RAD_6_DEG;

      if (det_area(FIRST_STRAIGHT_AREA[0],FIRST_STRAIGHT_AREA[1],FIRST_STRAIGHT_AREA[2],FIRST_STRAIGHT_AREA[3],mPre_50mm_x,mPre_50mm_y)){
	ZONE = FIRST_STRAIGHT_ZONE;
	gMotion_Ctl->set_mode_map_trace();
	gMotion_Ctl->set_zone_1st_straight();
	mMax_Forward = 100;
	mMax_Yawrate =  RAD_6_DEG;
	mMin_Yawrate = -1.0 * RAD_6_DEG;
      }else if (det_area(START_AREA[0],START_AREA[1],START_AREA[2],START_AREA[3],mXvalue,mYvalue) == false){
	//ZONE = LOST; //debug 20180625 kota
      }
      */

      /////////////////////////////////////---- 1108 ----

      break;

    case START_BACK:
      det_navi_log = 2010;
      gMotion_Ctl->set_mode_map_trace();
      gMotion_Ctl->set_zone_1st_straight();
      //      mMax_Forward = 10;
      mMax_Forward = 30;

      if(mXvalue > 360){
	ZONE = START_ZONE;
      }
      break;


/** LEFT 2018 *************************************************************** TRACK **/
    case FIRST_STRAIGHT_ZONE:
      det_navi_log = 2020;

      dif_odo = mOdo - ref_odo;

      if(dif_odo < 0){
	dif_odo = 0;
      }

      if(mPre_velo_0p5sec > MAX_VELOCITY){
	mMax_Forward = 	mMax_Forward;
      }else{
	acl_forward  = ref_forward + (dif_odo * ACCEL_GAIN);
	mMax_Forward = (int)acl_forward;
      }

      if(mMax_Forward > FIRST_STRAIGHT_FORWARD_VAL){
	mMax_Forward =  FIRST_STRAIGHT_FORWARD_VAL;
      }

      if(line_to_map){
	mMax_Forward =  20;
      }


      if (mPre_50mm_x > ENTER_1ST_CORNER_AREA[0]){


	ZONE = ENTER_1ST_CORNER_ZONE;
	//	gMotion_Ctl->set_zone_enter_1st_corner(); //may not be used 180624 kota

	//20181008----- for CS
	ref_forward = mMax_Forward;
	ref_odo     = mOdo;
	//----20181008----- for CS

	line_to_map = false;

      }else if (det_area(FIRST_STRAIGHT_AREA[0],FIRST_STRAIGHT_AREA[1],FIRST_STRAIGHT_AREA[2],FIRST_STRAIGHT_AREA[3],mXvalue,mYvalue) == false){
	//ZONE = LOST; //debug 20180625 kota
      }
      break;

/** LEFT 2018 *************************************************************** TRACK **/
    case ENTER_1ST_CORNER_ZONE:
      det_navi_log = 2030;
	
      //mMax_Forward = ENTER_1ST_CORNER_FORWARD_VAL;
      //mMax_Yawrate = RAD_45_DEG;
      //mMin_Yawrate = -1.0 * RAD_45_DEG;

      dif_odo = mOdo - ref_odo;
      
      if(dif_odo < 0){
	dif_odo = 0;
      }

      if(mPre_velo_0p5sec > MAX_VELOCITY){
	mMax_Forward = 	mMax_Forward;
      }else{
	acl_forward  = ref_forward + (dif_odo * ACCEL_GAIN);
	mMax_Forward = (int)acl_forward;
      }
      
      if(mMax_Forward > ENTER_1ST_CORNER_FORWARD_VAL){
	mMax_Forward =  ENTER_1ST_CORNER_FORWARD_VAL;
      }

      if (mPre_50mm_x >FIRST_CORNER_AREA[0]){
	//      if (det_area(FIRST_CORNER_AREA[0],FIRST_CORNER_AREA[1],FIRST_CORNER_AREA[2],FIRST_CORNER_AREA[3],mPre_50mm_x,mPre_50mm_y)){
	ZONE = FIRST_CORNER_ZONE;
	gMotion_Ctl->set_zone_1st_corner();

	//20181008----- for CS
	ref_forward = mMax_Forward;
	ref_odo     = mOdo;
	//----20181008----- for CS

      }else if (det_area(ENTER_1ST_CORNER_AREA[0],ENTER_1ST_CORNER_AREA[1],ENTER_1ST_CORNER_AREA[2],ENTER_1ST_CORNER_AREA[3],mXvalue,mYvalue) == false){
	//ZONE = LOST; //debug 20180625 kota
      }
      break;

/** LEFT 2018 *************************************************************** TRACK **/
    case FIRST_CORNER_ZONE:
      det_navi_log = 2040;
      //	mMax_Forward = FIRST_CORNER_FORWARD_VAL;
      //	mMax_Yawrate = RAD_45_DEG;
      //	mMin_Yawrate = -1.0 * RAD_45_DEG;

      dif_odo = mOdo - ref_odo;
      if(dif_odo < 0){
	dif_odo = 0;
      }
      
      if(mPre_velo_0p5sec > MAX_VELOCITY){
	mMax_Forward = 	mMax_Forward;
      }else{
	//	mMax_Forward = ref_forward + (dif_odo * ACCEL_GAIN);
	acl_forward  = ref_forward + (dif_odo * ACCEL_GAIN);
	mMax_Forward = (int)acl_forward;
      }

      if(mMax_Forward > FIRST_CORNER_FORWARD_VAL){
	mMax_Forward =  FIRST_CORNER_FORWARD_VAL;
      }

      //----20181008----- for CS


      //      if (det_area(SECOND_STRAIGHT_AREA[0],SECOND_STRAIGHT_AREA[1],SECOND_STRAIGHT_AREA[2],SECOND_STRAIGHT_AREA[3],mPre_50mm_x,mPre_50mm_y)){
      if (mPre_50mm_y > SECOND_STRAIGHT_AREA[1]){

	ZONE = SECOND_STRAIGHT_ZONE;
	gMotion_Ctl->set_zone_2nd_straight();

	//20181008----- for CS
	ref_forward = mMax_Forward;
	ref_odo     = mOdo;
	//----20181008----- for CS

      }else if (det_area(FIRST_CORNER_AREA[0],FIRST_CORNER_AREA[1],FIRST_CORNER_AREA[2],FIRST_CORNER_AREA[3],mXvalue,mYvalue) == false){
	//ZONE = LOST; //debug 20180625 kota
      }
      break;

/** LEFT 2018 *************************************************************** TRACK **/
      case SECOND_STRAIGHT_ZONE:
	det_navi_log = 2050;
	//20181008----- for CS
	//	mMax_Forward = SECOND_STRAIGHT_FORWARD_VAL;
	//	mMax_Yawrate =  RAD_6_DEG;
	//	mMin_Yawrate = -1.0 * RAD_6_DEG;

      dif_odo = mOdo - ref_odo;
      if(dif_odo < 0){
	dif_odo = 0;
      }

      if(mPre_velo_0p5sec > MAX_VELOCITY){
	mMax_Forward = 	mMax_Forward;
      }else{
	acl_forward  = ref_forward + (dif_odo * ACCEL_GAIN);
	mMax_Forward = (int)acl_forward;
      }

      if(mMax_Forward > SECOND_STRAIGHT_FORWARD_VAL){
	mMax_Forward = SECOND_STRAIGHT_FORWARD_VAL;
      }
      //----20181008----- for CS

      //      if (det_area(ENTER_2ND_CORNER_AREA[0],ENTER_2ND_CORNER_AREA[1],ENTER_2ND_CORNER_AREA[2],ENTER_2ND_CORNER_AREA[3],mPre_50mm_x,mPre_50mm_y)){
      if (mPre_50mm_y > ENTER_2ND_CORNER_AREA[1]){

	ZONE = ENTER_2ND_CORNER_ZONE;
	gMotion_Ctl->set_zone_enter_2nd_corner();

	//20181008----- for CS
	ref_forward = mMax_Forward;
	ref_odo     = mOdo;
	//----20181008----- for CS

      }else if (det_area(SECOND_STRAIGHT_AREA[0],SECOND_STRAIGHT_AREA[1],SECOND_STRAIGHT_AREA[2],SECOND_STRAIGHT_AREA[3],mXvalue,mYvalue) == false){
	//ZONE = LOST; //debug 20180625 kota
      }
      break;

/** LEFT 2018 *************************************************************** TRACK **/
      case ENTER_2ND_CORNER_ZONE:
	det_navi_log = 2060;
	//	mMax_Forward = ENTER_2ND_CORNER_FORWARD_VAL;
	//	mMax_Yawrate =  RAD_90_DEG;
	//	mMin_Yawrate = -1.0 * RAD_90_DEG;

      dif_odo = mOdo - ref_odo;
      if(dif_odo < 0){
	dif_odo = 0;
      }

      if(mPre_velo_0p5sec > MAX_VELOCITY){
	mMax_Forward = 	mMax_Forward;
      }else{
	//	mMax_Forward = ref_forward + (dif_odo * ACCEL_GAIN);
	acl_forward  = ref_forward + (dif_odo * ACCEL_GAIN);
	mMax_Forward = (int)acl_forward;
      }

      if(mMax_Forward > ENTER_2ND_CORNER_FORWARD_VAL){
	mMax_Forward = ENTER_2ND_CORNER_FORWARD_VAL;
      }

      //----20181008----- for CS


      //      if (det_area(SECOND_CORNER_AREA[0],SECOND_CORNER_AREA[1],SECOND_CORNER_AREA[2],SECOND_CORNER_AREA[3],mPre_50mm_x,mPre_50mm_y)){
      if (mPre_50mm_y > SECOND_CORNER_AREA[1]){
	ZONE = SECOND_CORNER_ZONE;
	gMotion_Ctl->set_zone_2nd_corner();

	//20181008----- for CS
	ref_forward = mMax_Forward;
	ref_odo     = mOdo;
	//----20181008----- for CS

      }else if (det_area(ENTER_2ND_CORNER_AREA[0],ENTER_2ND_CORNER_AREA[1],ENTER_2ND_CORNER_AREA[2],ENTER_2ND_CORNER_AREA[3],mXvalue,mYvalue) == false){
	//ZONE = LOST; //debug 20180625 kota
      }
      break;

/** LEFT 2018 *************************************************************** TRACK **/
    case SECOND_CORNER_ZONE:
      det_navi_log = 2070;
      //	mMax_Forward = SECOND_CORNER_FORWARD_VAL;
      //	mMax_Yawrate =  RAD_90_DEG;
      //	mMin_Yawrate = -1.0 * RAD_90_DEG;
      
      dif_odo = mOdo - ref_odo;
      if(dif_odo < 0){
	dif_odo = 0;
      }

      if(mPre_velo_0p5sec > MAX_VELOCITY){
	mMax_Forward = 	mMax_Forward;
      }else{
	//	mMax_Forward = ref_forward + (dif_odo * ACCEL_GAIN);
	acl_forward  = ref_forward + (dif_odo * ACCEL_GAIN);
	mMax_Forward = (int)acl_forward;
      }
      
      if(mMax_Forward > SECOND_CORNER_FORWARD_VAL){
	mMax_Forward = SECOND_CORNER_FORWARD_VAL;
      }

      //----20181008----- for CS

      //      if (det_area(THIRD_STRAIGHT_AREA[0],THIRD_STRAIGHT_AREA[1],THIRD_STRAIGHT_AREA[2],THIRD_STRAIGHT_AREA[3],mPre_50mm_x,mPre_50mm_y)){
      if (mPre_50mm_x > THIRD_STRAIGHT_AREA[0]){
	ZONE = THIRD_STRAIGHT_ZONE;
	gMotion_Ctl->set_zone_3rd_straight();

	//20181008----- for CS
	ref_forward = mMax_Forward;
	ref_odo     = mOdo;
	//----20181008----- for CS

      }else if (det_area(SECOND_CORNER_AREA[0],SECOND_CORNER_AREA[1],SECOND_CORNER_AREA[2],SECOND_CORNER_AREA[3],mXvalue,mYvalue) == false){
	//ZONE = LOST; //debug 20180625 kota
      }
      break;

/** LEFT 2018 *************************************************************** TRACK **/
      case THIRD_STRAIGHT_ZONE:
	det_navi_log = 2080;
	//	mMax_Forward = THIRD_STRAIGHT_FORWARD_VAL;
	//	mMax_Yawrate =  RAD_6_DEG;
	//	mMin_Yawrate = -1.0 * RAD_6_DEG;

      dif_odo = mOdo - ref_odo;

      if(dif_odo < 0){
	dif_odo = 0;
      }

      if(mPre_velo_0p5sec > MAX_VELOCITY){
	mMax_Forward = 	mMax_Forward;
      }else{
	//	mMax_Forward = ref_forward + (dif_odo * ACCEL_GAIN);
	acl_forward  = ref_forward + (dif_odo * ACCEL_GAIN);
	mMax_Forward = (int)acl_forward;
      }

      if(mMax_Forward > THIRD_STRAIGHT_FORWARD_VAL){
	mMax_Forward = THIRD_STRAIGHT_FORWARD_VAL;
      }

      //----20181008----- for CS


      //      if (det_area(THIRD_CORNER_AREA[0],THIRD_CORNER_AREA[1],THIRD_CORNER_AREA[2],THIRD_CORNER_AREA[3],mPre_50mm_x,mPre_50mm_y)){
      if (mPre_50mm_x > THIRD_CORNER_AREA[0]){
	ZONE = THIRD_CORNER_ZONE;
	gMotion_Ctl->set_zone_3rd_corner();

	//20181008----- for CS
	ref_forward = mMax_Forward;
	ref_odo     = mOdo;
	//----20181008----- for CS

      }else if (det_area(THIRD_STRAIGHT_AREA[0],THIRD_STRAIGHT_AREA[1],THIRD_STRAIGHT_AREA[2],THIRD_STRAIGHT_AREA[3],mXvalue,mYvalue) == false){
	//ZONE = LOST; //debug 20180625 kota
      }
      break;


/** LEFT 2018 *************************************************************** TRACK **/
    case THIRD_CORNER_ZONE:
      det_navi_log = 2090;
      //	mMax_Forward = THIRD_CORNER_FORWARD_VAL;
      //	mMax_Yawrate =  RAD_90_DEG;
      //	mMin_Yawrate = -1.0 * RAD_90_DEG;

      dif_odo = mOdo - ref_odo;
      if(dif_odo < 0){
	dif_odo = 0;
      }

      if(mPre_velo_0p5sec > MAX_VELOCITY){
	mMax_Forward = 	mMax_Forward;
      }else{
	//	mMax_Forward = ref_forward + (dif_odo * ACCEL_GAIN);
	acl_forward  = ref_forward + (dif_odo * ACCEL_GAIN);
	mMax_Forward = (int)acl_forward;
      }

      if(mMax_Forward > THIRD_CORNER_FORWARD_VAL){
	mMax_Forward = THIRD_CORNER_FORWARD_VAL;
      }
      //----20181008----- for CS
      if (mPre_50mm_y < (FOURTH_STRAIGHT_AREA[3] - 110 )){
	//      if (det_area(FOURTH_STRAIGHT_AREA[0],FOURTH_STRAIGHT_AREA[1],FOURTH_STRAIGHT_AREA[2],FOURTH_STRAIGHT_AREA[3],mPre_50mm_x,mPre_50mm_y)){
	ZONE = FOURTH_STRAIGHT_ZONE;
	gMotion_Ctl->set_zone_4th_straight();

	//20181008----- for CS
	ref_forward = mMax_Forward;
	ref_odo     = mOdo;
	//----20181008----- for CS

      }else if (det_area(THIRD_CORNER_AREA[0],THIRD_CORNER_AREA[1],THIRD_CORNER_AREA[2],THIRD_CORNER_AREA[3],mXvalue,mYvalue) == false){
	//ZONE = LOST; //debug 20180625 kota
      }
      break;


/** LEFT 2018 *************************************************************** TRACK **/
      case FOURTH_STRAIGHT_ZONE:
	det_navi_log = 2100;

      dif_odo = mOdo - ref_odo;
      if(dif_odo < 0){
	dif_odo = 0;
      }

      if(mPre_velo_0p5sec > MAX_VELOCITY){
	mMax_Forward = 	mMax_Forward;
      }else{
	//	mMax_Forward = ref_forward + (dif_odo * ACCEL_GAIN);
	acl_forward  = ref_forward + (dif_odo * ACCEL_GAIN);
	mMax_Forward = (int)acl_forward;
      }

      if(mMax_Forward > FOURTH_STRAIGHT_FORWARD_VAL){
	mMax_Forward = FOURTH_STRAIGHT_FORWARD_VAL;
      }

      //      if (det_area(FOURTH_CORNER_AREA[0],FOURTH_CORNER_AREA[1],FOURTH_CORNER_AREA[2],FOURTH_CORNER_AREA[3],mPre_50mm_x,mPre_50mm_y)){
      if (mPre_50mm_y < FOURTH_CORNER_AREA[3]){

	ZONE = FOURTH_CORNER_ZONE;
	gMotion_Ctl->set_zone_4th_corner();

	//20181008----- for CS
	ref_forward = mMax_Forward;
	ref_odo     = mOdo;
	//----20181008----- for CS


      }else if (det_area(FOURTH_STRAIGHT_AREA[0],FOURTH_STRAIGHT_AREA[1],FOURTH_STRAIGHT_AREA[2],FOURTH_STRAIGHT_AREA[3],mXvalue,mYvalue) == false){
	//ZONE = LOST; //debug 20180625 kota
      }
      break;


/** LEFT 2018 *************************************************************** TRACK **/
    case FOURTH_CORNER_ZONE:
      det_navi_log = 2110;

      mMax_Forward = FOURTH_CORNER_FORWARD_VAL;

      if (  (mXvalue > (FIRST_GRAY_AREA[0] - 200 ))&&(mYvalue < FIRST_GRAY_AREA[3])){
	gMotion_Ctl->set_mode_LUG();
	gMotion_Ctl->set_zone_garage();
	ZONE = LUG_ZONE;


      }else if (det_area(FOURTH_CORNER_AREA[0],FOURTH_CORNER_AREA[1],FOURTH_CORNER_AREA[2],FOURTH_CORNER_AREA[3],mXvalue,mYvalue) == false){
	//ZONE = LOST; //debug 20180625 kota
      }
      break;


/** LEFT 2018 *************************************************************** TRACK **/
      case FIRST_GRAY_ZONE:
	det_navi_log = 2120;
	mMax_Forward = 30;
	mMax_Yawrate =  RAD_6_DEG;
	mMin_Yawrate = -1.0 * RAD_6_DEG;

	//      if (det_area(LUG_AREA[0],LUG_AREA[1],LUG_AREA[2],LUG_AREA[3],mPre_50mm_x,mPre_50mm_y)){
	if(mPre_50mm_x > LUG_AREA[0]){
	ZONE = LUG_ZONE;
	gMotion_Ctl->set_mode_LUG();
	forward_curve_mode = false;

      }else if (det_area(FIRST_GRAY_AREA[0],FIRST_GRAY_AREA[1],FIRST_GRAY_AREA[2],FIRST_GRAY_AREA[3],mXvalue,mYvalue) == false){
	//ZONE = LOST; //debug 20180625 kota
      }
      break;


/** LEFT 2018 *************************************************************** TRACK **/
      case LUG_ZONE:
	det_navi_log = 2130;
      if (det_area(SECOND_GRAY_AREA[0],SECOND_GRAY_AREA[1],SECOND_GRAY_AREA[2],SECOND_GRAY_AREA[3],mPre_50mm_x,mPre_50mm_y)){
	ZONE = SECOND_GRAY_ZONE;
	//	gMotion_Ctl->set_zone_2nd_gray();
      }else if (det_area(LUG_AREA[0],LUG_AREA[1],LUG_AREA[2],LUG_AREA[3],mXvalue,mYvalue) == false){
	//ZONE = LOST; //debug 20180625 kota
      }
      break;


/** LEFT 2018 *************************************************************** TRACK **/
      case SECOND_GRAY_ZONE:
	det_navi_log = 2140;
	mMax_Forward = 100;
	mMax_Yawrate =  RAD_6_DEG;
	mMin_Yawrate = -1.0 * RAD_6_DEG;

      if (det_area(GARAGE_AREA[0],GARAGE_AREA[1],GARAGE_AREA[2],GARAGE_AREA[3],mPre_50mm_x,mPre_50mm_y)){
	//	gMotion_Ctl->set_mode_garage(); //it will be added later 180624 kota
	ZONE = GARAGE_ZONE;
	//	gMotion_Ctl->set_zone_garage();
      }else if (det_area(SECOND_GRAY_AREA[0],SECOND_GRAY_AREA[1],SECOND_GRAY_AREA[2],SECOND_GRAY_AREA[3],mXvalue,mYvalue) == false){
	//ZONE = LOST; //debug 20180625 kota
      }
      break;



/** LEFT 2018 *************************************************************** TRACK **/
      case GARAGE_ZONE:
	det_navi_log = 2150;
      if (det_area(GARAGE_AREA[0],GARAGE_AREA[1],GARAGE_AREA[2],GARAGE_AREA[3],mXvalue,mYvalue) == false){
	//ZONE = LOST; //debug 20180625 kota
      }
      break;

/** LEFT 2018 *************************************************************** TRACK **/
    case LOST:
      det_navi_log = 2160;
      mMax_Forward =    0;
      mMax_Yawrate =  0.0;
      mMin_Yawrate = -0.0;
      gMotion_Ctl->set_zone_lost();

      break;


    default:
      break;
    }
/**2018 L course ***************************/

/********************************************************************************/
/**  Bluetooth Mode  it has not been complited yet                             **/
/********************************************************************************/
  }else if(DRIVE_MODE == DEBUG){
    //    forward_curve_mode = true;
    forward_curve_mode = false;
    line_trace_mode    = false;

    gMotion_Ctl->set_mode_debug();    

    //   mMax_Forward = mOdo/20;
    //    mMax_Forward = mOdo/40;
    //    mMax_Forward = mOdo/20;
    //    mMax_Forward = mOdo/10; OK
    mMax_Forward = mOdo;
      
    if(mMax_Forward < 30){
      mMax_Forward = 30;
    }

    /*    if(mMax_Forward > 200){
      mMax_Forward = 200;
      }*/

    if(mMax_Forward > 180){
      mMax_Forward = 180;
    }


  }else{
    mMax_Forward = 200;
    mMax_Yawrate = 0;
    mMin_Yawrate = 0;
  }

}

/****************************************************************************************/
//2018 04 21 Kaoru Ota
//
/****************************************************************************************/

bool Ang_Brain::det_area(float x_left, float y_under, float x_right, float y_top, float x_value, float y_value){
  if(x_left < x_value && x_value <= x_right && y_under < y_value && y_value <= y_top){
    return true;
  }else{
    return false;
  }
}

/****************************************************************************************/
//2018 11 07 Kaoru Ota
//
/****************************************************************************************/
void Ang_Brain::det_on_line(){
  switch(ON_LINE_MODE){

  case ON_THE_LEFT_EDGE:
    on_line            = true;
    left_line          = false;
    right_line         = false;
    lost_line          = false;

    if(ave_line_val < 10){
      ON_LINE_MODE     = LEFT_THE_LINE;
      PRE_ON_LINE_MODE = ON_THE_LEFT_EDGE;
    }

    if(mLinevalue > 80){
      ON_LINE_MODE     = CENTER_LINE;
      PRE_ON_LINE_MODE = ON_THE_LEFT_EDGE;
    }

    break;

  case LEFT_THE_LINE:
    on_line            = false;
    left_line          = true;
    right_line         = false;
    lost_line          = false;

    break;

  case CENTER_LINE:
    on_line            = true;
    left_line          = false;
    right_line         = false;
    lost_line          = false;
    if(ave_line_val < 10){
      if(PRE_ON_LINE_MODE == ON_THE_LEFT_EDGE){
	ON_LINE_MODE = RIGHT_THE_LINE;
      }
    }
    break;

  case RIGHT_THE_LINE:
    on_line            = false;
    left_line          = false;
    right_line         = true;
    lost_line          = false;

    break;

  case UNKOWN:
    break;

  default:
    break;

  }
}



void Ang_Brain::setEyeCommand(int     linevalue,
			      bool    green_flag,
			      float   xvalue,
			      float   yvalue,
			      float   pre_50mm_x, //20180512 kota
			      float   pre_50mm_y,
			      float   odo,
			      float   velocity,
                              float   pre_velo_0p5sec,
			      float   yawrate,
			      float   abs_angle,
			      float   ave_angle,
			      int     robo_tail_angle,
			      bool    robo_stop,
			      bool    robo_forward,
			      bool    robo_back,
			      bool    robo_turn_left,
			      bool    robo_turn_right,
			      bool    dansa,
                              int16_t sonar_dis){

  mLinevalue       = linevalue;
  mGreen_flag      = green_flag;
  mXvalue          = xvalue;
  mYvalue          = yvalue;
  mPre_50mm_x      = pre_50mm_x;//50mm saki 20180512 kota
  mPre_50mm_y      = pre_50mm_y;//50mm saki 20180512 kota
  mOdo             = odo; 
  mVelocity        = velocity;
  mPre_velo_0p5sec = pre_velo_0p5sec;
  mYawrate         = yawrate;
  mYawangle        = abs_angle;
  mAve_yaw_angle   = ave_angle;

  mTail_angle      = robo_tail_angle;
  mRobo_stop       = robo_stop;
  mRobo_forward    = robo_forward;
  mRobo_back       = robo_back;
  mRobo_turn_left  = robo_turn_left;
  mRobo_turn_right = robo_turn_right;
  mDansa           = dansa;
  mSonar_dis       = sonar_dis;
}


void Ang_Brain::setRoboCommand(bool robo_balance_mode, bool robo_lug_mode){
  mRobo_balance_mode = robo_balance_mode;
  mRobo_lug_mode     = robo_lug_mode;
}

void Ang_Brain::GetCalcResult(int   forward_calc,
			      float yawratecmd_calc,
			      float ref_tail_angle_calc,
			      bool  tail_stand_mode_calc,
			      bool  tail_lug_mode_calc,
			      bool  rising_seesaw_calc,
			      bool  falling_seesaw_calc){
  
  forward         = forward_calc;
  yawratecmd      = yawratecmd_calc;
  ref_tail_angle  = ref_tail_angle_calc;
  tail_stand_mode = tail_stand_mode_calc;
  tail_lug_mode   = tail_lug_mode_calc;
  rising_seesaw   = rising_seesaw_calc;
  falling_seesaw  = falling_seesaw_calc;

}

