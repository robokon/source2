#include<float.h>

#include "ev3api.h"
#include "app.h"
#include "balancer.h"
#include "common.h"
#include "stair.h"
#include "module_cfg.h"
#include "Distance.h" 

signed char forward;            /* 前後進命令 */
signed char turn;               /* 旋回命令 */
signed char pwm_L, pwm_R;       /* 左右モータPWM出力 */

int Floor_Search_Flag;          /* フロア検知フラグ  */
#define FLAG_OFF          0     /* フロア検知オフ */
#define FLAG_ON           1     /* フロア検知オン */

int Floor_Status;               /* フロアステータス */
#define STAGE_ZERO        0     /* 階段 0 階 */
#define STAGE_ONE         1     /* 階段 1 階 */
#define STAGE_ONE         2     /* 階段 2 階 */

int Run_Mode;                   /* 走行モード */
#define RUN_MODE_STAY     0     /* 走行モード（停止）*/
#define RUN_MODE_RUN      1     /* 走行モード（走行中）*/
#define RUN_MODE_SPIN     2     /* 走行モード（回転中）*/

int Stage_One_Spin_Status;      /* 1階の回転ステータス */
int Stage_Tow_Spin_Status;      /* 2階の回転ステータス */
#define SPIN_FINISH       0     /* 回転完了 */
#define SPIN_NOT_FINISH   1     /* 回転未完了 */

int Stair_Count = 0;            /* 階段の関数に入った回数 */
int Gyro_Store[100];            /* ジャイロセンサー値の平均値の格納先 */


//*****************************************************************************
// 関数名 : stair_main
// 引数 : 
// 返り値 : なし
// 概要 : 
//       
//*****************************************************************************
void stair_main()
{
    int gyro_Ave_Arg; /* ジャイロ平均値の格納するときの引数 */
    
    if(Stair_Count == 0)
    {
        /* 各ステータスの初期化 */
        Floor_Status = STAGE_ZERO;
        Run_Mode = RUN_MODE_RUN;
        Stage_One_Spin_Status = SPIN_NOT_FINISH;
        Stage_Tow_Spin_Status = SPIN_NOT_FINISH;
        Floor_Search_Flag = FLAG_OFF;
    }

    
    /* こっから下、間違ってる 明日の夕方から直す */
    /* ジャイロ角速度にて階段を検知する */ 
   if((Stair_Count % 10) == 0) 
   { 
        gyro_Ave_Arg = ( Stair_Count / 10 ); /* 格納先の指定 */
        /* 40 msごとにgyro_strにジャイロ角速度センサー値を入手 (4 ms周期と仮定) */ 
         Gyro_Store[gyro_Ave_Arg] = ev3_gyro_sensor_get_rate(gyro_sensor); 
   }

   /* 40 ms前のセンサー値と今のセンサー値の差分が規定値より大きい場合階段にぶつかったと判断する */ 
   if((gyro_str - ev3_gyro_sensor_get_rate(gyro_sensor)) > 140 || (gyro_str - ev3_gyro_sensor_get_rate(gyro_sensor)) < (140 * (-1)) ) 
   { 
      /* 階段検知を音で示す */ 
       ev3_speaker_set_volume(10);  
       ev3_speaker_play_tone(NOTE_C4, 100); 
       Floor_Search_Flag = FLAG_ON; 
   }

    Stair_Count++;
}


