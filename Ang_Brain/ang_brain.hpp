/******************************************************************************
 *  ang_brain.h (for LEGO Mindstorms EV3)
 *  Created on: 2018/04/21
 *  Implementation of the Class ang_brain
 *  Author: Kaoru Ota
 *****************************************************************************/

#ifndef EV3_APP_ANAGOBRAIN_H_
#define EV3_APP_ANAGOBRAIN_H_
#include "motion_ctl.hpp"
#include "util.hpp"

using namespace std;

class Ang_Brain {
public:
	explicit Ang_Brain();//コンストラクタ
	void init();
	void set_drive_mode_LT();
	void set_drive_mode_TK();
	void set_drive_mode_DB();
	void run();
	void setEyeCommand(int     linevalue,
			   bool    green_flag,
			   float   xvalue,
			   float   yvalue,
			   float   pre_50mm_x,
			   float   pre_50mm_y,
			   float   odo,
                           float   velocity,
                           float   pre_velo_0p5sec,
			   float   yawrate,
//			   float   yawangle,
			   float   abs_angle,
			   float   ave_angle,
			   int     robo_tail_angle,
			   bool    robo_stop,
			   bool    robo_forward,
			   bool    robo_back,
			   bool    robo_turn_left,
			   bool    robo_turn_right,
                           bool    dansa,
			   int16_t sonar_dis);

  void setRoboCommand(bool robo_balance_mode, bool robo_lug_mode);

  Average_500_Data *gAve_line_val = new Average_500_Data();
  Average_500_Data *gAve_yaw_angle_500 = new Average_500_Data(); //20181108

  float ave_line_val;
  float ave_yaw_angle_500; //20181108

  bool  line_trace_mode;
  //あなご手足に渡す変数
  int   forward;         //前進目標値
  float yawratecmd;      //目標ヨーレート
  float ref_tail_angle;    //尻尾角度
  bool  forward_curve_mode;
  bool  tail_stand_mode; //倒立走行フラグ
  bool  tail_lug_mode;
  bool  rising_seesaw;
  bool  falling_seesaw;
  bool  on_line;
  bool  left_line;
  bool  right_line;
  bool  lost_line;
  bool  line_to_map;  //181108

  int   det_navi_log;

private:
  void det_navigation();

  bool det_area(float x_left, float y_under, float x_right, float y_top, float x_value, float y_value);

  void det_on_line();
  
  void GetCalcResult(int forward_calc,
		     float yawratecmd_calc,
		     float ref_tail_angle_calc,
		     bool  tail_stand_mode_calc,
		     bool  tail_stand_lug_calc,
		     bool  rising_seesaw_calc,
		     bool  falling_seesaw_calc);

  //    StrategyDet *gStrategyDet = new StrategyDet();
  Motion_Ctl *gMotion_Ctl   = new Motion_Ctl();

  int   Mmode;
  int   mLinevalue;  //ライン検出値
  bool  mGreen_flag;  //ライン検出値

  float mXvalue;     //x座標
  float mYvalue;     //y座標

  float mPre_50mm_x;//50mm saki 20180512 kota
  float mPre_50mm_y;//50mm saki 20180512 kota


  float mOdo;             //Total Distance from Start point
  float mVelocity;        //速度
  float mPre_velo_0p5sec; //prediction vekicity 0.5 sec latar
    float mYawrate;         //ヨーレート
  float mYawangle;        //ヨー角
  float mAve_yaw_angle;

  int   mTail_angle;
  //signals for robo movement
  bool  mRobo_stop       = false;
  bool  mRobo_forward    = false;
  bool  mRobo_back       = false;
  bool  mRobo_turn_left  = false;
  bool  mRobo_turn_right = false;
  bool  mDansa;      //段差検出値

  int16_t mSonar_dis;
  bool  mRobo_balance_mode;
  bool  mRobo_lug_mode;
  
  int   mMax_Forward;

  float mRef_Yawrate;
  float mMax_Yawrate;
  float mMin_Yawrate;

  enum Zone{
    START_ZONE,
    START_BACK,
    FIRST_STRAIGHT_ZONE,
    ENTER_1ST_CORNER_ZONE,
    FIRST_CORNER_ZONE,
    SECOND_STRAIGHT_ZONE,
    ENTER_2ND_CORNER_ZONE,
    SECOND_CORNER_ZONE,
    THIRD_STRAIGHT_ZONE,
    THIRD_CORNER_ZONE,
    S_CORNER_ZONE,
    FOURTH_STRAIGHT_ZONE,
    FOURTH_CORNER_ZONE,
    ENTER_5TH_CORNER_ZONE,
    FIFTH_CORNER_ZONE,
    FIRST_GRAY_ZONE,
    LUG_ZONE,
    BACK_LUG_ZONE,
    SECOND_GRAY_ZONE,
    SEESAW_ZONE,
    GARAGE_ZONE,
    LOST
  };

  enum Drive_Mode{
    LINE_TRACE,
    TRACK,
    DEBUG
  };

  enum On_Line_Mode{
    ON_THE_LEFT_EDGE,
    CENTER_LINE,
    LEFT_THE_LINE,
    RIGHT_THE_LINE,
    UNKOWN,
  };

  Zone         ZONE;
  Drive_Mode   DRIVE_MODE;
  On_Line_Mode ON_LINE_MODE;
  On_Line_Mode PRE_ON_LINE_MODE;

};

#endif  // EV3_APP_ANAGOBRAIN_H_
