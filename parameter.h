/*
#ifdef __cplusplus
extern "C" {
#endif

#define dT_100ms             0.1 //タスク周期[s]

#ifdef __cplusplus
}
#endif
*/
//Parameter of Robo
extern int TAIL_ANGLE_STAND_UP;  /* 完全停止時の角度[度] */
extern int TAIL_ANGLE_BALANCE_START;
extern int TAIL_ANGLE_LAUNCH;
extern int TAIL_ANGLE_RUN;     /* バランス走行時の角度[度] */
extern int TAIL_ON_ANGLE; /* 完全停止時の角度[度] */
extern int TAIL_ANGLE_LUG; /* 3点移動時の角度[度] */
extern int TAIL_ANGLE_GARAGE; /* 完全停止時の角度[度] */

extern float WHEEL_R;       //radius of wheel[mm]
extern int   RoboTread;      //トレッド長さ[mm]

extern float MAX_VELOCITY;


//Parameter of time length unit
extern float dT_4ms;

extern float PAI;

extern float RAD_315_DEG;
extern float RAD_1_DEG; //deg@1rad 
extern float RAD_5_DEG; //
extern float RAD_6_DEG;
extern float RAD_15_DEG; //deg@1rad 
extern float RAD_22P5_DEG; //
extern float RAD_30_DEG; //
extern float RAD_45_DEG; //
extern float RAD_89_DEG;
extern float RAD_88p5_DEG;
extern float RAD_87_DEG;
extern float RAD_90_DEG;
extern float RAD_120_DEG;
extern float RAD_135_DEG;
extern float RAD_150_DEG;
extern float RAD_180_DEG;
extern float RAD_225_DEG;
extern float RAD_270_DEG;
extern float RAD_315_DEG;
extern float RAD_345_DEG;
extern float RAD_360_DEG;
extern float RAD_450_DEG;

extern float MINUS_RAD_5_DEG;    //
extern float MINUS_RAD_15_DEG;   //
extern float MINUS_RAD_22P5_DEG; //
extern float MINUS_RAD_30_DEG;   //
extern float MINUS_RAD_45_DEG;   //
extern float MINUS_RAD_60_DEG;   //
extern float MINUS_RAD_90_DEG;   //
extern float MINUS_RAD_135_DEG;   //
extern float MINUS_RAD_145_DEG;  //
extern float MINUS_RAD_180_DEG;  //
extern float MINUS_RAD_225_DEG;  //
extern float MINUS_RAD_270_DEG;  //

extern float YAW_LIMIT;
extern float YAW_STEP;

//LUG

extern int   MAX_LUG_FORWARD;
extern float LUG_1st_STOP_X;
extern float LUG_2nd_STOP_X;
extern float LUG_3rd_STOP_X;

extern float LUG_YAW_GAIN;
extern int   TAIL_STD_LINE_DET;

extern int   CALIB_LINE_100_MAX_THRS;
extern int   CALIB_LINE_50_MAX_THRS;
extern int   CALIB_LINE_50_MIN_THRS;
extern int   CALIB_LINE_0_MIN_THRS;

extern int   COLOR_SENSOR_OFFSET;
extern float COLOR_SENSOR_GAIN;

//Parameter of Garage
extern float GARAGE_X_POS;


extern float ACCEL_GAIN;
extern float DECEL_GAIN;

//Parameter of Area
extern int START_FORWARD_VAL;
extern int FIRST_STRAIGHT_FORWARD_VAL;
extern int ENTER_1ST_CORNER_FORWARD_VAL;
extern int FIRST_CORNER_FORWARD_VAL;
extern int SECOND_STRAIGHT_FORWARD_VAL;
extern int ENTER_2ND_CORNER_FORWARD_VAL;
extern int SECOND_CORNER_FORWARD_VAL;
extern int THIRD_STRAIGHT_FORWARD_VAL;
extern int THIRD_CORNER_FORWARD_VAL;
extern int S_CORNER_FORWARD_VAL;
extern int FOURTH_STRAIGHT_FORWARD_VAL;
extern int FOURTH_CORNER_FORWARD_VAL;
extern int ENTER_5TH_CORNER_FORWARD_VAL;
extern int FIFTH_CORNER_FORWARD_VAL;
extern int FIRST_GRAY_FORWARD_VAL;
extern int LUG_FORWARD_VAL;
extern int BACK_LUG_FORWARD_VAL;
extern int SECOND_GRAY_FORWARD_VAL;
extern int SEESAW_FORWARD_VAL;
extern int GARAGE_FORWARD_VAL;
extern int GOAL_VAL;

extern float START_AREA[4];
extern float FIRST_STRAIGHT_AREA[4];
extern float ENTER_1ST_CORNER_AREA[4];
extern float FIRST_CORNER_AREA[4];
extern float SECOND_STRAIGHT_AREA[4];
extern float ENTER_2ND_CORNER_AREA[4];
extern float SECOND_CORNER_AREA[4];
extern float THIRD_STRAIGHT_AREA[4];
extern float THIRD_CORNER_AREA[4];
extern float S_CORNER_AREA[4];
extern float FOURTH_STRAIGHT_AREA[4];
extern float FOURTH_CORNER_AREA[4];
extern float ENTER_5TH_CORNER_AREA[4];
extern float FIFTH_CORNER_AREA[4];
extern float FIRST_GRAY_AREA[4];
extern float LUG_AREA[4];
extern float BACK_LUG_AREA[4];
extern float SECOND_GRAY_AREA[4];
extern float SEESAW_AREA[4];
extern float GARAGE_AREA[4];

extern float STRAIGT_01[4];
extern float CIRCLE_01[3];
extern float STRAIGT_02[4];
extern float CIRCLE_02[3];
extern float STRAIGT_03[4];
extern float CIRCLE_03[3];
extern float STRAIGT_04[4];
extern float CIRCLE_04[3];
extern float STRAIGT_05[4];
extern float CIRCLE_05[3];

extern float CIRCLE_01_LENGTH;
extern float CIRCLE_01_ANGLE;

extern float CIRCLE_02_LENGTH;
extern float CIRCLE_02_ANGLE;

extern float CIRCLE_03_LENGTH;
extern float CIRCLE_03_ANGLE;

extern float CIRCLE_04_LENGTH;
extern float CIRCLE_04_ANGLE;

extern float CIRCLE_05_LENGTH;
extern float CIRCLE_05_ANGLE;
