#include "parameter.h"
#include "util.hpp"
//#include "brain_calc_lib.hpp"

class Motion_Ctl {
public:
	explicit Motion_Ctl();//コンストラクタ

	void init ();
	void SetCurrentData(int     linevalue,
			    bool    green_flag,
                            float   xvalue,
                            float   yvalue,
			    float   pre_50mm_x,
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
			    );

	void set_mode_LT();
	void set_mode_map_trace();
	void set_mode_LUG();
	void set_mode_seesaw();

	void set_mode_tail_std_debug();
	void set_mode_debug();

	void set_zone_start();
	void set_zone_1st_straight();
	void set_zone_enter_1st_corner();
	void set_zone_1st_corner();
	void set_zone_2nd_straight();
	void set_zone_enter_2nd_corner();
	void set_zone_2nd_corner();
	void set_zone_3rd_straight();
	void set_zone_3rd_corner();
	void set_zone_4th_straight();
	void set_zone_4th_corner();
	void set_zone_5th_corner();
	void set_zone_1st_gray();
	void set_zone_lug();
	void set_zone_back_lug();
	void set_zone_2nd_gray();
	void set_zone_garage();
	void set_zone_lost();

	void run(float xvalue, float yvalue, float yawangle);//走行戦略を計算


	bool  left_line_edge = true; //non used now, 
	int   forward;
	float yawratecmd;
	float ref_tail_angle;
	bool  tail_stand_mode;
	bool  tail_lug_mode;
        bool  rising_seesaw;
        bool  falling_seesaw;

private:
    void LineTracerYawrate(int line_value);              //ライントレース（ヨーレート）
    void MapTracer(float mXvalue, float mYvalue, float mYawangle);  //仮想ゲート走行 0827 tada
    void LUG_run(float odo, float angle,int line_value);

    PID *gForward = new PID();

    enum Motion_Mode{
      MAP_TRACE,
      LINE_TRACE,
      LUG,
      SEESAW,
      TAIL_STAND_DEBUG,
      DEBUG,
      DEBUG_1
    };
    
    enum Zone{
      START_ZONE,
      FIRST_STRAIGHT_ZONE,
      ENTER_1ST_CORNER_ZONE,
      FIRST_CORNER_ZONE,
      SECOND_STRAIGHT_ZONE,
      ENTER_2ND_CORNER_ZONE,
      SECOND_CORNER_ZONE,
      THIRD_STRAIGHT_ZONE,
      THIRD_CORNER_ZONE,
      FOURTH_STRAIGHT_ZONE,
      FOURTH_CORNER_ZONE,
      FIFTH_CORNER_ZONE,
      FIRST_GRAY_ZONE,
      LUG_ZONE,
      BACK_LUG_ZONE,
      SECOND_GRAY_ZONE,
      GARAGE_ZONE,
      LOST
    };

    enum Lug_Mode{
      LUG_START,
      APPROACH_TO_LUG,
      TAIL_ON_1ST,
      POS_ADJ_1ST,
      LUG_MODE_1ST,
      LUG_1ST,
      PRE_1ST_TURN,
      TURN_1ST,

      APPROACH_TO_2ND_LUG,
      LUG_MODE_2ND,
      LUG_2ND,
      PRE_2ND_TURN,
      TURN_2ND,

      APPROACH_TO_3RD_LUG,
      LUG_MODE_3RD,
      LUG_3RD,

      TAIL_STAND_UP,
      TURN_90,
      FIND_GREEN,
      TURN_M90,
      FIND_LEFT_EDGE,
      Y_POS_ADJ,
      TURN_ZERO,
      GRAY_GARAGE,

      LUG_DEBUG_00
    };

  enum Seesaw_Mode{
    SEESAW_START,
    APPROACH_TO_SEESAW,
    TAIL_ON,
    TURN_TO_180,
    FIND_GREEN_SS,
    TURN_TO_ZERO,
    FIND_LINE,
    X_POS_ADJ,
    TURN_TO_90,
    FIND_SEESAW,
    BACK_FOR_BALANCE,
    BALANCE,
    GET_ON,
    FIRST_SEESAW,
    SEESAW_OFF,
    SEESAW_DEBUG_00,
    SEESAW_DEBUG_01,
    SEESAW_DEBUG_02,
    SEESAW_DEBUG_03,
    DONE


  };



  Motion_Mode MOTION_MODE;
  Zone        ZONE;
  Lug_Mode    LUG_MODE;
  Seesaw_Mode SEESAW_MODE;

  int   Mmode;
  int   mLinevalue; //ライン検出値
  bool  mGreen_flag;

  float mXvalue;    //x座標
  float mYvalue;    //y座標

  float mPre_50mm_x;//50mm saki 20180512 kota
  float mPre_50mm_y;//50mm saki 20180512 kota


  float mOdo;       //Total distance [mm] from start point
  float mVelocity;  //速度
  float mYawrate;   //ヨーレート
  float mYawangle;  //ヨー角
  int   mTail_angle;
  float mYaw_angle_offset;

  float ref_x;


  //signals for robo movement
  bool  mRobo_stop       = false;
  bool  mRobo_forward    = false;
  bool  mRobo_back       = false;
  bool  mRobo_turn_left  = false;
  bool  mRobo_turn_right = false;

  bool    mDansa;      //段差検出値
  int16_t mSonar_dis;

  bool  mRobo_balance_mode;
  bool  mRobo_lug_mode;

  int   mMax_Forward;
  float mRef_Yawrate;
  float mMax_Yawrate;
  float mMin_Yawrate;
  float ref_forward;

  //	bool tail_mode_lflag_calc; // 倒立走行フラグ 0817
  
  float y_t;
  float y_t_prev; //0818 tada. passed y_t
  float pg = 0.35;//暫定0.9 //0818 tada
  float df = 0.024;//暫定-0.1 //0818 tada
  int   bat_mv;
};
