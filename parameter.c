/**** SEE           ****/
/**** ADJ_PARAMETER ****/

//Parameter of Robo
int TAIL_ANGLE_STAND_UP       = 97; /* 完全停止時の角度[度]     */
int TAIL_ANGLE_BALANCE_START  = 100;
int TAIL_ANGLE_LAUNCH         = 105;


int TAIL_ANGLE_RUN      =  3; /* バランス走行時の角度[度] */
int TAIL_ON_ANGLE       = 85; /* 完全停止時の角度[度]     */
//int TAIL_ANGLE_LUG      = 75; /* 3点移動時の角度[度]      */
//int TAIL_ANGLE_LUG      = 70; /* 3点移動時の角度[度]      */
//int TAIL_ANGLE_LUG      = 65; /* 3点移動時の角度[度]      */
//int TAIL_ANGLE_LUG      = 68; /* 3点移動時の角度[度]      */
int TAIL_ANGLE_LUG      = 67; /* 3点移動時の角度[度]      */
int TAIL_ANGLE_GARAGE   = 94; /* 完全停止時の角度[度]     */

float WHEEL_R       = 49.75; //Wheel radius 2018
int   RoboTread     = 184; //トレッド長さ[mm]

float MAX_VELOCITY      = 400; /**** ADJ_PARAMETER ****/
//float MAX_VELOCITY      = 450; /**** ADJ_PARAMETER ****/
//float MAX_VELOCITY      = 500; /**** ADJ_PARAMETER ****/

//Parameter of time length unit
float dT_4ms   = 0.004;

float PAI         =  3.1472;

float RAD_1_DEG    = 0.0175; //deg@1rad 
float RAD_5_DEG    = 0.0873; //
float RAD_6_DEG    = 0.1047; //
float RAD_15_DEG   = 0.2618; //
float RAD_22P5_DEG = 0.3927; //
float RAD_30_DEG   = 0.5236; //
float RAD_45_DEG   = 0.7854; //
float RAD_89_DEG   = 1.5533; //
float RAD_88p5_DEG = 1.5446; //
float RAD_87_DEG   = 1.5184; //
float RAD_90_DEG   = 1.5708; //
float RAD_120_DEG  = 2.0944; //
float RAD_135_DEG  = 2.3562; //
float RAD_150_DEG  = 2.6180; //
float RAD_180_DEG  = 3.1472; //
float RAD_225_DEG  = 3.9270; //
float RAD_270_DEG  = 4.7124;
float RAD_315_DEG  = 5.4978; //
float RAD_345_DEG  = 6.0214; //
float RAD_360_DEG  = 6.2832; //
float RAD_450_DEG  = 7.8540;

float MINUS_RAD_5_DEG    = -0.0873; //
float MINUS_RAD_15_DEG   = -0.2618; //
float MINUS_RAD_22P5_DEG = -0.3927; //
float MINUS_RAD_30_DEG   = -0.5236; //
float MINUS_RAD_45_DEG   = -0.7854; //
float MINUS_RAD_60_DEG   = -1.0472; //
float MINUS_RAD_90_DEG   = -1.5708; //
float MINUS_RAD_135_DEG  = -2.3562; //
float MINUS_RAD_145_DEG  = -2.5307; //
float MINUS_RAD_180_DEG  = -3.1472; //
float MINUS_RAD_225_DEG  = -3.9270; //
float MINUS_RAD_270_DEG  = -4.7124;

float YAW_LIMIT    = 0.393;   //PI/8 see anago synario
float YAW_STEP     = 0.00786; //YAW_LIMIT/50 see anago synario

//--LUG--LUG--LUG--LUG--LUG--LUG--LUG--LUG--LUG--LUG--LUG--LUG--LUG--LUG--LUG--LUG--
//--LUG--LUG--LUG--LUG--LUG--LUG--LUG--LUG--LUG--LUG--LUG--LUG--LUG--LUG--LUG--LUG--

int   MAX_LUG_FORWARD        = 30;

//map data
//https://drive.google.com/open?id=1Ih-fZX68pgRuQWVPbCW-5Oj-Bgrn3pgo

float LUG_1st_STOP_X         = 4590; /**** ADJ_PARAMETER ****/
//float LUG_2nd_STOP_X         = 4190; /**** ADJ_PARAMETER ****/
float LUG_2nd_STOP_X         = 4150; /**** ADJ_PARAMETER ****/
float LUG_3rd_STOP_X         = 4590; /**** ADJ_PARAMETER ****/

float LUG_YAW_GAIN           = 2.0;

//Parameter of Garage
//float GARAGE_X_POS          = 4980;
float GARAGE_X_POS          = 4990; /**** ADJ_PARAMETER   case GRAY_GARAGE:****/

//int   TAIL_STD_LINE_DET      = 49; /**** ADJ_PARAMETER ****/
int   TAIL_STD_LINE_DET      = 70;  /**** ADJ_PARAMETER ****/

int   CALIB_LINE_100_MAX_THRS = 20;
int   CALIB_LINE_50_MAX_THRS  = 100;
int   CALIB_LINE_50_MIN_THRS  = 40;
int   CALIB_LINE_0_MIN_THRS   = 150;


//HONBAN
int   COLOR_SENSOR_OFFSET    = 18;  /**** ADJ_PARAMETER ****/
//int   COLOR_SENSOR_OFFSET    = 10;  /**** ADJ_PARAMETER ****/
//float COLOR_SENSOR_GAIN      = 2.0; /**** ADJ_PARAMETER ****/
float COLOR_SENSOR_GAIN      = 3.0; /**** ADJ_PARAMETER ****/

//YOBI
//int   COLOR_SENSOR_OFFSET    = 10;  /**** ADJ_PARAMETER ****/
//float COLOR_SENSOR_GAIN      = 1.5; /**** ADJ_PARAMETER ****/


//map data
//https://drive.google.com/open?id=1Ih-fZX68pgRuQWVPbCW-5Oj-Bgrn3pgo
//(x0,y0) - (x1,y1)

float ACCEL_GAIN                = 1.0;
float DECEL_GAIN                = 1.0;

int START_FORWARD_VAL            = 180; //koko
int FIRST_STRAIGHT_FORWARD_VAL   = 180;
int ENTER_1ST_CORNER_FORWARD_VAL = 140;
int FIRST_CORNER_FORWARD_VAL     = 140;
int SECOND_STRAIGHT_FORWARD_VAL  = 180;
int ENTER_2ND_CORNER_FORWARD_VAL = 100;
int SECOND_CORNER_FORWARD_VAL    =  80;
int THIRD_STRAIGHT_FORWARD_VAL   = 100;
int THIRD_CORNER_FORWARD_VAL     =  80;
int S_CORNER_FORWARD_VAL         = 100;
int FOURTH_STRAIGHT_FORWARD_VAL  = 180;
int GOAL_VAL                     = 120;

int FOURTH_CORNER_FORWARD_VAL    =  40;

int ENTER_5TH_CORNER_FORWARD_VAL = 100;
int FIFTH_CORNER_FORWARD_VAL     = 100;
int FIRST_GRAY_FORWARD_VAL       =  40;
int LUG_FORWARD_VAL              = 100;
int BACK_LUG_FORWARD_VAL         = 100;
int SECOND_GRAY_FORWARD_VAL      = 100;
int SEESAW_FORWARD_VAL           = 100;
int GARAGE_FORWARD_VAL           = 100;


float START_AREA[4]            = { 300,    0,  500,  320};
float FIRST_STRAIGHT_AREA[4]   = { 500,    0, 2770,  320};
float ENTER_1ST_CORNER_AREA[4] = {2470,    0, 2770,  320};
float FIRST_CORNER_AREA[4]     = {2770,    0, 3780, 1160};
float SECOND_STRAIGHT_AREA[4]  = {3500, 1160, 4000, 2820};
float ENTER_2ND_CORNER_AREA[4] = {3500, 2520, 4000, 2820};
float SECOND_CORNER_AREA[4]    = {3500, 2820, 4220, 4000};
float THIRD_STRAIGHT_AREA[4]   = {4220, 3000, 4660, 4000};
float THIRD_CORNER_AREA[4]     = {4660, 2790, 5500, 4000};
float FOURTH_STRAIGHT_AREA[4]  = {3000,  610, 5500, 2790};

float FOURTH_CORNER_AREA[4]    = {3000,    0, 4000,  610};
//float FOURTH_CORNER_AREA[4]    = {3000,    0, 4000,  800}; //koko

float ENTER_5TH_CORNER_AREA[4] = {   0,    0,    0,    0};
float FIFTH_CORNER_AREA[4]     = {   0,    0,    0,    0};
//float FIRST_GRAY_AREA[4]     = {3870,    0, 4020,  320};
float FIRST_GRAY_AREA[4]       = {3920,    0, 4020,  270};

float LUG_AREA[4]              = {4020,    0, 4765,  320}; //lug + back lug area 180624 kota
float BACK_LUG_AREA[4]         = {4390,    0, 4765,  320}; // may not be used
float SECOND_GRAY_AREA[4]      = {4765,    0, 4915,  320};
float SEESAW_AREA[4]           = {   0,    0,    0,    0};
float GARAGE_AREA[4]           = {4915,    0, 5200,  320};


//straigt (x0,y0) - (x1,y1)
//circle (x0,y0,r)
float STRAIGT_01[4] = {   0,  160, 2770,  160};
float CIRCLE_01[3]  = {2770, 1160, 1000};
float STRAIGT_02[4] = {3770, 1160, 3770, 2820};
float CIRCLE_02[3]  = {4220, 2820,  450};
float STRAIGT_03[4] = {4220, 3270, 4660, 3270};
float CIRCLE_03[3]  = {4660, 2910,  360};
float STRAIGT_04[4] = {5000, 2790, 3630,  610};
//float CIRCLE_04[3]  = {3870,  450,  290};

float CIRCLE_04[3]  = {3970,  450,  300};

float STRAIGT_05[4] = {3870,  160, 5200,  160};
float CIRCLE_05[3]  = {   0,    0,    0};

float CIRCLE_01_LENGTH = 1571;
float CIRCLE_01_ANGLE  = 1.570796;

float CIRCLE_02_LENGTH = 707;
float CIRCLE_02_ANGLE  = -1.570796;

float CIRCLE_03_LENGTH = 691;
float CIRCLE_03_ANGLE  = -1.919862;

float CIRCLE_04_LENGTH = 623;
float CIRCLE_04_ANGLE  = 2.146755;

float CIRCLE_05_LENGTH = 0;
float CIRCLE_05_ANGLE  = 0;


