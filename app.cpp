//Anago System
//Date:2018.4.27
//Author:Kaoru Ota
//https://github.com/KaoruOta/left_anago2018
#include "stdio.h"
#include "string.h"
#include "stdlib.h"

#include "app.hpp"
#include "util.hpp"
#include "ev3api.h"

#include "TouchSensor.h"
#include "SonarSensor.h"
#include "ColorSensor.h"
#include "GyroSensor.h"
#include "Motor.h"
#include "Clock.h"

//anagoサブシステム
#include "ang_eye.hpp"
#include "ang_brain.hpp"
#include "ang_robo.hpp"

// デストラクタ問題の回避
// https://github.com/ETrobocon/etroboEV3/wiki/problem_and_coping
void *__dso_handle=0;

// using宣言
//using namespace ev3api;
using ev3api::ColorSensor;
using ev3api::GyroSensor;
using ev3api::TouchSensor;
using ev3api::SonarSensor;
using ev3api::Motor;
using ev3api::Clock;

/* LCDフォントサイズ */
#define CALIB_FONT        (EV3_FONT_MEDIUM)
#define CALIB_FONT_WIDTH  (6/*TODO: magic number*/)
#define CALIB_FONT_HEIGHT (20/*TODO: magic number*/)

#define LOG_RECORD
#define LOG_SHORT
//#define LOG_LONG

// Device objects
// オブジェクトを静的に確保する
TouchSensor gTouchSensor (PORT_1);
SonarSensor gSonarSensor (PORT_2);
ColorSensor gColorSensor (PORT_3);
GyroSensor  gGyroSensor  (PORT_4);
Motor       gTailMotor   (PORT_A);
Motor       gRightWheel  (PORT_B);
Motor       gLeftWheel   (PORT_C);

enum Sys_Mode{
  LINE_TRACE,
  TRACK,
  DEBUG,
};

Sys_Mode SYS_MODE;


static int32_t   bt_cmd = 0;      /* Bluetoothコマンド 1:リモートスタート */
static FILE     *bt     = NULL;   /* Bluetoothファイルハンドル */

static Ang_Eye   *gAng_Eye;
static Ang_Brain *gAng_Brain;
static Ang_Robo  *gAng_Robo;
static Balancer  *gBalancer;

//0729 kota Color Sensor Calibration
unsigned char white       = 60;
unsigned char black       = 2;
unsigned char white_slant = 12;
unsigned char black_slant = 2;

#ifdef LOG_RECORD

#ifdef LOG_SHORT
static int   log_size = 10000;
static int   log_cnt  = 0;
static int   log_dat_00[10000];
static int   log_dat_01[10000];
static int   log_dat_02[10000];
static int   log_dat_03[10000];
static int   log_dat_04[10000];
static int   log_dat_05[10000];
static int   log_dat_06[10000];
static int   log_dat_07[10000];
static int   log_dat_08[10000];
#endif

#ifdef LOG_LONG
static int   log_size = 20000;
static int   log_cnt  = 0;
static int   log_dat_00[20000];
static int   log_dat_01[20000];
static int   log_dat_02[20000];
static int   log_dat_03[20000];
#endif

#endif

/****************************************************************************************/
//System Initialize
//
/****************************************************************************************/

static void sys_initialize() {
  
  int  battery;
  char battery_str[32];
  int  gyro;
  char gyro_str[32];
  bool set_mode;

  //**********************************************************************************//
  //Display Course mode on LCD
  //initialize LCD and display Program mode
  //**********************************************************************************//
  ev3_lcd_fill_rect(0, 0, EV3_LCD_WIDTH, EV3_LCD_HEIGHT, EV3_LCD_WHITE);
  ev3_lcd_set_font(EV3_FONT_MEDIUM);
  ev3_lcd_draw_string("LEFT_1108_00",0, 40);
  //**********************************************************************************//
  //New Object of Sub System(Class)
  //
  //**********************************************************************************//
  // [TODO] タッチセンサの初期化に2msのdelayがあるため、ここで待つ
  tslp_tsk(2);

  
  // オブジェクトの作成
  gBalancer  = new Balancer();
  gAng_Eye   = new Ang_Eye(gColorSensor,
			   gLeftWheel,
			   gRightWheel,
			   gGyroSensor,
			   gSonarSensor,
                           gTouchSensor);

  gAng_Brain = new Ang_Brain();
  gAng_Robo  = new Ang_Robo(gGyroSensor,
			    gLeftWheel,
			    gRightWheel,
			    gTailMotor,
			    gBalancer);

  //**********************************************************************************//
  //Set Tail Initial position
  //
  //**********************************************************************************//
  gAng_Robo->tail_reset();
  gAng_Robo->tail_stand_up();

  //**********************************************************************************//
  //Display Robot Status
  //State of Battery, mA,mV, Gyro_offset
  //**********************************************************************************//
  ev3_speaker_set_volume(1);
  ev3_speaker_play_tone(NOTE_C4,200);
  
  ev3_lcd_fill_rect(0, 0, EV3_LCD_WIDTH, EV3_LCD_HEIGHT, EV3_LCD_WHITE);
  ev3_lcd_set_font(EV3_FONT_MEDIUM);
  ev3_lcd_draw_string("LEFT 2018",0, 0);
  battery = ev3_battery_voltage_mV();
  sprintf(battery_str, "V:%d", battery);
  ev3_lcd_draw_string(battery_str,0, 20);

  battery = ev3_battery_current_mA();
  sprintf(battery_str, "A:%d", battery);
  ev3_lcd_draw_string(battery_str,0, 40);

  //**********************************************************************************//
  //Set Gyro offset
  //
  //**********************************************************************************//
  ev3_lcd_draw_string("Set ANG on GND",0, 60);
  ev3_lcd_draw_string("PUSH TS 4 RESET",0, 80);

  while(1){
    if (gTouchSensor.isPressed()){
      gAng_Eye->init();   //reset gyro
      gAng_Robo->init();  //
      gAng_Brain->init(); //initialize mode
      break; /* タッチセンサが押された */
    }
    gyro = gGyroSensor.getAnglerVelocity();  // ジャイロセンサ値
    sprintf(gyro_str, "Gyro:%d", gyro);
    ev3_lcd_draw_string(gyro_str,0, 100);
    tslp_tsk(10); //What dose it mean? kota 170812
  }
  ev3_speaker_play_tone(NOTE_E4,200);
  ev3_lcd_fill_rect(0, 0, EV3_LCD_WIDTH, EV3_LCD_HEIGHT, EV3_LCD_WHITE);

  tslp_tsk(500);

  while(1){
    if (gTouchSensor.isPressed()){
      break; /* タッチセンサが押された */
    }
    gyro = gGyroSensor.getAnglerVelocity();  // ジャイロセンサ値
    sprintf(gyro_str, "Gyro:%d", gyro);
    ev3_lcd_draw_string(gyro_str,0, 100);
    tslp_tsk(20); //What dose it mean? kota 170812
    ev3_lcd_fill_rect(0, 0, EV3_LCD_WIDTH, EV3_LCD_HEIGHT, EV3_LCD_WHITE);
  }
  ev3_speaker_play_tone(NOTE_E4,200);
  ev3_lcd_fill_rect(0, 0, EV3_LCD_WIDTH, EV3_LCD_HEIGHT, EV3_LCD_WHITE);

  //**********************************************************************************//
  //Connect Bluetooh
  //
  //**********************************************************************************//
  /* Open Bluetooth file */
  bt = ev3_serial_open_file(EV3_SERIAL_BT);
  assert(bt != NULL);

  /* Bluetooth通信タスクの起動 */
  act_tsk(BT_TASK);

  //**********************************************************************************//
  //Select Mode
  //
  //**********************************************************************************//
  ev3_lcd_fill_rect(0, 0, EV3_LCD_WIDTH, EV3_LCD_HEIGHT, EV3_LCD_WHITE);
  ev3_lcd_set_font(EV3_FONT_MEDIUM);
  ev3_lcd_draw_string("Select Mode",0, 0);

  SYS_MODE = LINE_TRACE;

  while(1){
    set_mode = false;

    switch(SYS_MODE){

    case LINE_TRACE:
      ev3_lcd_draw_string("->LINE_TRACE",0, 20);
      ev3_lcd_draw_string("TRACK_MODE  ",0, 40);
      ev3_lcd_draw_string("DEBUG_MODE  ",0, 80);

      if (ev3_button_is_pressed(ENTER_BUTTON)){
	ev3_speaker_play_tone(NOTE_E4,200);
	SYS_MODE = LINE_TRACE;
	set_mode = true;
      }else if (ev3_button_is_pressed(DOWN_BUTTON)){
	SYS_MODE = TRACK;
	set_mode = false;
      }else{
	SYS_MODE = LINE_TRACE;
	set_mode = false;
      }
      break;


    case TRACK:
      ev3_lcd_draw_string("LINE_TRACE  ",0, 20);
      ev3_lcd_draw_string("->TRACK_MODE",0, 40);
      ev3_lcd_draw_string("DEBUG_MODE  ",0, 80);

      if (ev3_button_is_pressed(ENTER_BUTTON)){
	ev3_speaker_play_tone(NOTE_E4,200);
	SYS_MODE = TRACK;
	set_mode = true;
      }else if (ev3_button_is_pressed(DOWN_BUTTON)){
	SYS_MODE = DEBUG;
	set_mode = false;
      }else{
	SYS_MODE = TRACK;
	set_mode = false;
      }
      break;

    case DEBUG:
      ev3_lcd_draw_string("LINE_TRACE  ",0, 20);
      ev3_lcd_draw_string("TRACK_MODE  ",0, 40);
      ev3_lcd_draw_string("->DEBUG_MODE",0, 80);

      if (ev3_button_is_pressed(ENTER_BUTTON)){
	ev3_speaker_play_tone(NOTE_E4,200);
	SYS_MODE = DEBUG;
	set_mode = true;
      }else if (ev3_button_is_pressed(DOWN_BUTTON)){
	SYS_MODE = LINE_TRACE;
	set_mode = false;
      }else{
	SYS_MODE = DEBUG;
	set_mode = false;
      }
      break;

    default:
      SYS_MODE = LINE_TRACE;
      set_mode = false;
      break;
    }

    tslp_tsk(100);

    
    if (set_mode == true){

      if(SYS_MODE == LINE_TRACE){
	ev3_lcd_draw_string("SET_LINE_TRACE            ",0, 100);
	gAng_Brain->set_drive_mode_LT();
      }else if(SYS_MODE == TRACK){
	ev3_lcd_draw_string("SET_TRACK_MODE            ",0, 100);
	gAng_Brain->set_drive_mode_TK();
      }else{
	ev3_lcd_draw_string("SET_DEBUG_MODE            ",0, 100);
	gAng_Brain->set_drive_mode_DB();
      }
      tslp_tsk(1000);
      break;
    }else{
      ev3_lcd_draw_string("select mode down and enter",0, 100);
    }
    
  }

  //**********************************************************************************//
  //Completed Intitialize
  //**********************************************************************************//
  // 初期化完了通知
  ev3_led_set_color(LED_OFF);

  ev3_lcd_fill_rect(0, 0, EV3_LCD_WIDTH, EV3_LCD_HEIGHT, EV3_LCD_WHITE);
  ev3_speaker_play_tone(NOTE_C4,200);

}


//Systen Destroy
static void sys_destroy(){
  delete gAng_Eye;
  delete gAng_Brain;
  delete gAng_Robo;
  delete gBalancer;
}

#ifdef LOG_RECORD
static void log_dat( ){
  
  float float_to_int_x1000;

  switch(SYS_MODE){
    case LINE_TRACE:
#ifdef LOG_SHORT
      /*
      log_dat_00[log_cnt]  = gAng_Eye->odo;
      log_dat_01[log_cnt]  = gAng_Eye->linevalue;
      log_dat_02[log_cnt]  = (int)gAng_Eye->xvalue;
      log_dat_03[log_cnt]  = (int)gAng_Eye->yvalue;
      log_dat_04[log_cnt]  = (int)gAng_Brain->ave_line_val;

      float_to_int_x1000   = gAng_Brain->ave_yaw_angle_500 * 1000.0;
      log_dat_05[log_cnt]  =  (int)float_to_int_x1000;




      log_dat_07[log_cnt]  = gAng_Robo->log_forward;
      
      float_to_int_x1000   = gAng_Brain->yawratecmd * 1000.0;
      log_dat_08[log_cnt]  =  (int)float_to_int_x1000;
      */

      log_dat_00[log_cnt]  = gAng_Eye->odo;
      log_dat_01[log_cnt]  = gAng_Eye->linevalue;
      log_dat_02[log_cnt]  = (int)gAng_Eye->xvalue;
      log_dat_03[log_cnt]  = (int)gAng_Eye->yvalue;
      log_dat_04[log_cnt]  = (int)gAng_Eye->velocity;
      log_dat_05[log_cnt]  = (int)gAng_Eye->pre_velo_0p5sec;

      float_to_int_x1000   =  gAng_Eye->abs_angle*1000.0;
      log_dat_06[log_cnt]  =  (int)float_to_int_x1000;

      log_dat_07[log_cnt]  = gAng_Robo->log_forward;
      log_dat_08[log_cnt]  = gAng_Brain->det_navi_log;      

      /*

      log_dat_00[log_cnt]  = (int)gAng_Eye->xvalue;
      log_dat_01[log_cnt]  = (int)gAng_Eye->yvalue;
      log_dat_02[log_cnt]  = (int)gAng_Eye->velocity;
      log_dat_03[log_cnt]  = (int)gAng_Eye->pre_velo_0p5sec;
      log_dat_04[log_cnt]  = gAng_Brain->forward;
      log_dat_05[log_cnt]  = gAng_Robo->log_forward;
      log_dat_06[log_cnt]  = gAng_Eye->linevalue;
      log_dat_07[log_cnt]  = gAng_Eye->odo;
      log_dat_08[log_cnt]  = (int)gAng_Eye->ave_y;
      */
      
      /*

      log_dat_00[log_cnt]  = (int)gAng_Eye->xvalue;
      log_dat_01[log_cnt]  = (int)gAng_Eye->ave_x;
      log_dat_02[log_cnt]  = (int)gAng_Eye->ave_y;
      log_dat_03[log_cnt]  = (int)gAng_Eye->ave_vel_x;
      log_dat_04[log_cnt]  = (int)gAng_Eye->ave_vel_y;
      log_dat_05[log_cnt]  = (int)gAng_Eye->velocity;
      log_dat_06[log_cnt]  = (int)gAng_Eye->ave_velo;
      log_dat_07[log_cnt]  = (int)gAng_Eye->ave_accel;
      log_dat_08[log_cnt]  = (int)gAng_Eye->pre_velo_0p5sec;

       */

      /*
      log_dat_00[log_cnt]  = gAng_Eye->linevalue;
      log_dat_01[log_cnt]  = gAng_Eye->odo;
      log_dat_02[log_cnt]  = (int)gAng_Eye->xvalue;
      log_dat_03[log_cnt]  = (int)gAng_Eye->yvalue;
      //      log_dat_04[log_cnt]  = gTailMotor.getCount();
      log_dat_04[log_cnt]  = gAng_Eye->sonarDistance;
      log_dat_05[log_cnt]  = gAng_Robo->log_forward;
      log_dat_06[log_cnt]  = gAng_Eye->velocity;

      float_to_int_x1000   = gAng_Brain->yawratecmd * 1000.0;
      log_dat_07[log_cnt]  =  (int)float_to_int_x1000;


      float_to_int_x1000   =  gAng_Eye->abs_angle*1000.0;
      log_dat_08[log_cnt]  =  (int)float_to_int_x1000;
       */

      //      log_dat_08[log_cnt]  = gAng_Robo->log_gyro;
      //---- 20181008----



      /*
      //0828-- 
      log_dat_00[log_cnt]  = gTailMotor.getCount();
      log_dat_01[log_cnt]  = gAng_Robo->log_gyro;
      //      log_dat_02[log_cnt]  = gGyroSensor.getAngle();
      log_dat_02[log_cnt]  = 0;
      log_dat_03[log_cnt]  = gAng_Robo->log_forward;
      log_dat_04[log_cnt]  = gAng_Eye->velocity;
      log_dat_05[log_cnt]  = gAng_Robo->log_left_pwm; 
      log_dat_06[log_cnt]  = gAng_Robo->log_right_pwm; 
      log_dat_07[log_cnt]  = ev3_battery_voltage_mV();
      log_dat_08[log_cnt]  = ev3_battery_current_mA();

      //--0828
      */
#endif

#ifdef LOG_LONG
      /*
      log_dat_00[log_cnt]  = gAng_Eye->linevalue;
      log_dat_01[log_cnt]  = gAng_Eye->odo;
      log_dat_02[log_cnt]  = (int)gAng_Eye->xvalue;
      log_dat_03[log_cnt]  = (int)gAng_Eye->yvalue;
      */

      log_dat_00[log_cnt]  = (int)gAng_Eye->xvalue;
      log_dat_01[log_cnt]  = gAng_Brain->forward;
      log_dat_02[log_cnt]  = gAng_Robo->log_forward;
      log_dat_03[log_cnt]  = gAng_Eye->velocity;
#endif
      break;


    case TRACK:
#ifdef LOG_SHORT

      /*
      log_dat_00[log_cnt]  = gAng_Eye->linevalue;
      log_dat_01[log_cnt]  = gAng_Brain->ave_line_val;
      log_dat_02[log_cnt]  = gAng_Brain->on_line * 100;
      log_dat_03[log_cnt]  = gAng_Brain->left_line * 100 ;
      log_dat_04[log_cnt]  = gAng_Brain->right_line * 100;
      log_dat_05[log_cnt]  = gAng_Brain->lost_line * 100;
      log_dat_06[log_cnt]  = gAng_Robo->log_left_pwm; 
      log_dat_07[log_cnt]  = gAng_Robo->log_gyro;
      log_dat_08[log_cnt]  = (int)gAng_Eye->yvalue;
      */

      log_dat_00[log_cnt]  = gAng_Eye->linevalue;
      log_dat_01[log_cnt]  = gAng_Eye->odo;
      log_dat_02[log_cnt]  = (int)gAng_Eye->xvalue;
      log_dat_03[log_cnt]  = (int)gAng_Eye->yvalue;

      float_to_int_x1000   =  gAng_Eye->abs_angle*1000.0;
      log_dat_04[log_cnt]  =  (int)float_to_int_x1000;

      log_dat_05[log_cnt]  = gAng_Robo->log_forward;
      log_dat_06[log_cnt]  = gAng_Eye->velocity;

      float_to_int_x1000   = gAng_Brain->yawratecmd * 1000.0;
      log_dat_07[log_cnt]  =  (int)float_to_int_x1000;

      log_dat_08[log_cnt]  = gAng_Brain->det_navi_log;



      /*
      //0828--
      log_dat_00[log_cnt]  = gTailMotor.getCount();
      log_dat_01[log_cnt]  = gAng_Robo->log_gyro;
      log_dat_02[log_cnt]  = 0;
      log_dat_03[log_cnt]  = gAng_Robo->log_forward;
      log_dat_04[log_cnt]  = gAng_Eye->velocity;
      log_dat_05[log_cnt]  = gAng_Robo->log_left_pwm; 
      log_dat_06[log_cnt]  = gAng_Robo->log_right_pwm; 
      log_dat_07[log_cnt]  = ev3_battery_voltage_mV();
      log_dat_08[log_cnt]  = ev3_battery_current_mA();

      //--0828
      */

#endif

#ifdef LOG_LONG
      log_dat_00[log_cnt]  = (int)gAng_Eye->xvalue;
      log_dat_01[log_cnt]  = gAng_Brain->forward;
      log_dat_02[log_cnt]  = gAng_Robo->log_forward;
      log_dat_03[log_cnt]  = gAng_Eye->velocity;
#endif

      break;

    case DEBUG:

#ifdef LOG_SHORT

      log_dat_00[log_cnt]  = gAng_Eye->linevalue;
      log_dat_01[log_cnt]  = gAng_Brain->ave_line_val;
      log_dat_02[log_cnt]  = gAng_Brain->on_line * 100;
      log_dat_03[log_cnt]  = gAng_Brain->left_line * 100 ;
      log_dat_04[log_cnt]  = gAng_Brain->right_line * 100;
      log_dat_05[log_cnt]  = gAng_Brain->lost_line * 100;
      log_dat_06[log_cnt]  = gAng_Robo->log_left_pwm; 
      log_dat_07[log_cnt]  = gAng_Robo->log_gyro;
      log_dat_08[log_cnt]  = (int)gAng_Eye->yvalue;

#endif

#ifdef LOG_LONG
      log_dat_00[log_cnt]  = (int)gAng_Eye->xvalue;
      log_dat_01[log_cnt]  = gAng_Brain->forward;
      log_dat_02[log_cnt]  = gAng_Robo->log_forward;
      log_dat_03[log_cnt]  = gAng_Eye->velocity;
#endif

      break;

  default:

      break;
  }


  log_cnt++;
  if (log_cnt == log_size){
    log_cnt  = 0;
  }
}

static void export_log_dat( ){

#define MAX_CHAR_NUM 100
#define CHAR_ARRAY_NUM 2

  //  FILE* file_id;
  //    int battery = ev3_battery_voltage_mV();

  FILE  *fp_rd;
  FILE  *fp_wr;

  char word_array[CHAR_ARRAY_NUM][MAX_CHAR_NUM];
	
  int i;
		
  //file name gen ----
  char file_name[MAX_CHAR_NUM] = "log_000_"; //koko
  char file_cnt[MAX_CHAR_NUM] = "000";
  char file_format[MAX_CHAR_NUM] = ".csv";
  int  str_to_num;
  //---- file name gen


	//READ FILE NUMBER
  fp_rd = fopen("sys_dat.csv", "r");


  i = 0;
  while (fgets(&word_array[i][0], MAX_CHAR_NUM, fp_rd) != NULL){
    i++;
  }
  fclose(fp_rd);
  
  //incremant number of dat file ----
  str_to_num = atoi(&word_array[0][0]);
  str_to_num = str_to_num + 10;
  sprintf(file_cnt, "%d", str_to_num);
  strcat(file_name, file_cnt);
  strcat(file_name, file_format);
	
  //UPDATA NUM in sys_dat
  fp_wr = fopen("sys_dat.csv", "w");
  fprintf(fp_wr, "%d\n", str_to_num);
  fclose(fp_wr);

  //LOG DATA WRITE
  fp_wr = fopen(file_name, "w");
  



  switch(SYS_MODE){
#ifdef LOG_SHORT
    case LINE_TRACE:
      //      fprintf(fp_wr, "odo,line,x,y,ave_line,ave_yaw_anglex1000,det_nabi,robo_forward,yawrate_cmd\n");   
      fprintf(fp_wr, "odo,line,x,y,velo,pre_velo,angle,robo_forward,det_navi_log\n");   
      //      fprintf(fp_wr, "x,y,velo,pre_velo,brain_forward, robo_forward,line,odo,ave_y\n");   
      //      fprintf(fp_wr, "x,ave_x,ave_y,ave_vel_x,ave_vel_y,velo,ave_velo,ave_accel,pre_velo_3sec\n");   
      //      fprintf(fp_wr, "line,odo,x,y,sonar,log_forward,velocity,yawratecmd,angle\n");   
      //fprintf(fp_wr, "tail_angle,gyro_omega,gyro_angle,robo_forward,velo,left_pwm,right_pwm,mV,mA\n");   
      break;

    case TRACK:

      //      fprintf(fp_wr, "line, ave_line,on_line,left_line,right_line,lost_line,pwm,gyro,y \n");   
      fprintf(fp_wr, "line,odo,x,y,abs_angle,forward,velocity,yaw_cmd,det_navi_log\n");   
      //      fprintf(fp_wr, "tail_angle,gyro_omega,gyro_angle,robo_forward,velo,left_pwm,right_pwm,mV,mA\n");   
      break;

    case DEBUG:
      fprintf(fp_wr, "line, ave_line,on_line,left_line,right_line,lost_line,pwm,gyro,y \n");   
      break;
#endif

#ifdef LOG_LONG
    case LINE_TRACE:
      fprintf(fp_wr, "x,ref_speed,forward,velocity\n");   
      break;

    case TRACK:
      fprintf(fp_wr, "x,ref_speed,forward,velocity\n");   
      break;

    case DEBUG:
      fprintf(fp_wr, "x,ref_speed,forward,velocity\n");   

#endif

  default:

      break;
  }




    int cnt;

    for(cnt = 0; cnt < log_size ; cnt++){
#ifdef LOG_SHORT
      fprintf(fp_wr, "%d,%d,%d,%d,%d,%d,%d,%d,%d\n",log_dat_00[cnt],log_dat_01[cnt], log_dat_02[cnt],log_dat_03[cnt],log_dat_04[cnt],log_dat_05[cnt],log_dat_06[cnt],log_dat_07[cnt],log_dat_08[cnt]);
#endif

#ifdef LOG_LONG
      fprintf(fp_wr, "%d,%d,%d,%d\n",log_dat_00[cnt],log_dat_01[cnt], log_dat_02[cnt],log_dat_03[cnt]);
#endif

    }
    fclose(fp_wr);
}
#endif



//*****************************************************************************
// 関数名 : bt_task
// 引数 : unused
// 返り値 : なし
// 概要 : Bluetooth通信によるリモートスタート。 Tera Termなどのターミナルソフトから、
//       ASCIIコードで1を送信すると、リモートスタートする。
//*****************************************************************************
void bt_task(intptr_t unused)
{
  while(1){
    uint8_t c = fgetc(bt); /* 受信 */


    switch(c){
    case '1':
      bt_cmd = 1;
      break;

    case '0':
      ev3_speaker_play_tone(NOTE_C4,200);
      ev3_led_set_color(LED_GREEN);
      break;

    default:
      ev3_led_set_color(LED_OFF);
      break;
    }
    fputc(c, bt); /* エコーバック */

  }
}

//Anago Eye Task
void eye_cyc(intptr_t exinf) {
    act_tsk(EYE_TASK);//0817 tada
}

void eye_task(intptr_t exinf) {

    gAng_Eye->run();

    if(gAng_Brain->line_trace_mode){
      gAng_Eye->correct_odometry();
    }

    gAng_Eye->setSonarDistance();

    gAng_Brain->setEyeCommand(gAng_Eye->linevalue,
                              gAng_Eye->green_flag,
                              gAng_Eye->xvalue,
                              gAng_Eye->yvalue,
			      gAng_Eye->pre_50mm_x,
			      gAng_Eye->pre_50mm_y,
                              gAng_Eye->odo,
                              gAng_Eye->velocity,
                              gAng_Eye->pre_velo_0p5sec, 
                              gAng_Eye->yawrate,
                              gAng_Eye->abs_angle,
                              gAng_Eye->ave_angle,
			      gTailMotor.getCount(),
			      gAng_Eye->robo_stop,
			      gAng_Eye->robo_forward,
			      gAng_Eye->robo_back,
			      gAng_Eye->robo_turn_left,
			      gAng_Eye->robo_turn_right,
                              gAng_Eye->dansa,
			      gAng_Eye->sonarDistance);

    
    gAng_Brain->setRoboCommand(gAng_Robo->balance_mode, gAng_Robo->lug_mode);

  ext_tsk();
}

//Anago Brain Task
void brain_cyc(intptr_t exinf) {
    act_tsk(BRAIN_TASK); //0817 tada
}

void brain_task(intptr_t exinf) {

    if (ev3_button_is_pressed(BACK_BUTTON)) {

    //    wup_tsk(MAIN_TASK);  // バックボタン押下

    }
    else {
      gAng_Brain->run();
      gAng_Robo->setCommand(gAng_Eye->ave_velo,//gAng_Eye->velocity,
			    gAng_Brain->forward,
                            gAng_Brain->yawratecmd,
                            gAng_Brain->ref_tail_angle,
                            gAng_Eye->yawrate,
                            gAng_Brain->forward_curve_mode,
                            gAng_Brain->tail_stand_mode,
			    gAng_Brain->tail_lug_mode,
			    gAng_Brain->rising_seesaw,
			    gAng_Brain->falling_seesaw);
    }


    ext_tsk();
}

//Anago Robo(Teashi) Task
void robo_cyc(intptr_t exinf) {
    act_tsk(ROBO_TASK);
}

void robo_task(intptr_t exinf) {

#ifdef LOG_RECORD
  log_dat();
#endif

#ifdef LOG_RECORD  
  if (ev3_button_is_pressed(DOWN_BUTTON)){
    wup_tsk(MAIN_TASK);
  }
#endif

  if (ev3_button_is_pressed(BACK_BUTTON)) {
    wup_tsk(MAIN_TASK);  // バックボタン押下
  } else {
    gAng_Robo->run();
  }
  ext_tsk();
}


//Main Task
void main_task(intptr_t unused) {
  //**********************************************************************************//
  //System Intialize
  //**********************************************************************************//
  sys_initialize();

  //**********************************************************************************//
  //Color Sensor calibration
  //**********************************************************************************//
  gAng_Eye->color_sensor_calib(); //20180930 kota
  //**********************************************************************************//
  //Reset angle of tail
  //**********************************************************************************//
  //REDAY for START
  gAng_Robo->tail_reset();
  gAng_Robo->tail_stand_up();

  ev3_sta_cyc(EYE_CYC);
  //  ev3_sta_cyc(BRAIN_CYC);

  ev3_lcd_set_font(EV3_FONT_MEDIUM);
  ev3_lcd_draw_string("Set ANG on Start Line",0, 40);
  ev3_lcd_draw_string("PRESS TS or 1",0, 80);

  while(1){

    if(ev3_bluetooth_is_connected()){
      ev3_lcd_draw_string("BT connected",0, 60);
    }else{
      ev3_lcd_draw_string("BT unconnected",0, 60);
    }

    if (bt_cmd == 1){

      break; /* リモートスタート */
    }
    if (gTouchSensor.isPressed()){
      tslp_tsk(100);
      break; /* タッチセンサが押された */
    }
    tslp_tsk(10); //What dose it mean? kota 170812
  }


  ev3_lcd_fill_rect(0, 0, EV3_LCD_WIDTH, EV3_LCD_HEIGHT, EV3_LCD_WHITE);
  ev3_led_set_color(LED_OFF);

  ev3_sta_cyc(BRAIN_CYC);
  gAng_Robo->set_robo_mode_launch();
  ev3_sta_cyc(ROBO_CYC);
  ter_tsk(BT_TASK);

  slp_tsk();  // バックボタンが押されるまで待つ

  ev3_stp_cyc(ROBO_CYC);

  gLeftWheel.~Motor();
  gRightWheel.~Motor();
  gTailMotor.~Motor();

  ev3_led_set_color(LED_ORANGE);
  ev3_lcd_set_font(EV3_FONT_SMALL);
  ev3_lcd_fill_rect(0, 0, EV3_LCD_WIDTH, EV3_LCD_HEIGHT, EV3_LCD_WHITE);
  ev3_lcd_draw_string("Stop",0, CALIB_FONT_HEIGHT*1);

#ifdef LOG_RECORD
  ev3_lcd_draw_string("Saving Log Data",0, CALIB_FONT_HEIGHT*2);
  export_log_dat( );
  ev3_lcd_draw_string("Saving Log Data is done",0, CALIB_FONT_HEIGHT*3);
#endif

  ev3_led_set_color(LED_OFF);



  sys_destroy();
  ext_tsk();
}// end::main_task

