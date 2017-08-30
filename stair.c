#include<float.h>

#include "ev3api.h"
#include "app.h"
#include "balancer.h"
#include "common.h"
#include "stair.h"
#include "module_cfg.h"
#include "Distance.h"

signed char forward;                /* 前後進命令 */
signed char turn;                   /* 旋回命令 */
signed char pwm_L, pwm_R;           /* 左右モータPWM出力 */


/* ~~~~~~~~~~~~~~~~~~~~~~~要調整の定義~~~~~~~~~~~~~~~~~~~~~~~~ */
#define GYRO_AVE_OK_BORDER    40    /* ジャイロの安定したの基準値  */
#define STAGE_ZERO_BORDEAR    140   /* 0階 -> 1階のジャイロ境界線 */
#define STAGE_ONE_BORDEAR     160   /* 1階 -> 2階のジャイロ境界線 */
#define STAGE_TWO_BORDEAR     140   /* 2階 -> 0階のジャイロ境界線 */
#define GYRO_AVE_STR_MAX      20    /* 40 ms間隔のセンサ値いくつで平均値出すか　*/
#define FLOOR_UP_OK_COUNT     200    /* どのくらい連続してジャイロが安定していたら登ったと判定するか <- これでぶつかってから真ん中までを調整 */

#define FLOOR_ZERO_RUN_SPEED  60    /* 0階の走行速度 */
#define FLOOR_ONE_RUN_SPEED   60    /* 1階の走行速度 */
#define FLOOR_TWO_RUN_SPEED   60    /* 2階の走行速度 */

#define FLOOR_ONE_SPIN_VALUE  700   /* 360度回転 */
#define FLOOR_TWO_SPIN_VALUE  900   /* 450度回転 */

#define FLOOR_ZERO_WAITING_COUNT 1000
#define FLOOR_ONE_WAITING_COUNT  1000
#define FLOOR_TWO_WAITING_COUNT  1000

/* ~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~ */


int Stair_Count = 0;                /* 階段の関数に入った回数 */

int Floor_Search_Flag;              /* フロア検知フラグ  */
#define FLAG_OFF              0     /* フロア検知オフ */
#define FLAG_ON               1     /* フロア検知オン */

int Floor_Status;                   /* フロアステータス */
#define STAGE_ZERO            0     /* 階段 0 階 */
#define STAGE_ONE             1     /* 階段 1 階 */
#define STAGE_TWO             2     /* 階段 2 階 */
#define STAGE_LAST            3     /* 階段 2 階最後の直線 */

int Run_Mode;                       /* 走行モード */
#define RUN_MODE_STAY         0     /* 走行モード（停止）*/
#define RUN_MODE_RUN          1     /* 走行モード（走行中）*/
#define RUN_MODE_SPIN         2     /* 走行モード（回転中）*/

int Stage_One_Spin_Status;          /* 1階の回転ステータス */
int Stage_Two_Spin_Status;          /* 2階の回転ステータス */
#define SPIN_FINISH           0     /* 回転完了 */
#define SPIN_NOT_FINISH       1     /* 回転未完了 */

int Sipn_Func_Count = 0;            /* Spin_Funcのカウント値 */
int RIGHT_info = 0;                 /* 右モータの情報 */
int RIGHT_info_first = 0;           /* 回転直前の角位置 */


int Gyro_Pre_40ms = 0;              /* 40 ms前のジャイロセンサ値 */
int Ave_Count;                      /* 格納先のカウンタ */
int Gyro_Store[GYRO_AVE_STR_MAX];   /* ジャイロセンサー値の格納先 */
int Gyro_Ave = 0;                   /* 平均値 */
int Gyro_Ave_OK_COUNT = 0;          /* フロア検知時のセンサー値がOKのカウント数　*/

int Waiting_Flag;                   /* 倒立制御だけ行うモードのフラグ */
#define WAITING_FLAG_ON       1     /* 倒立制御だけ行うモードのフラグ ON */
#define WAITING_FLAG_OFF      0     /* 倒立制御だけ行うモードのフラグ OFF */

int Waiting_Count_Flag;
#define WAITING_COUNT_FLAG_ON       1     /* フラグ ON */
#define WAITING_COUNT_FLAG_OFF      0     /* フラグ OFF */
int Waiting_Count_Value;

extern int main_status;             /* メインステータスの引用 */

/* プロトタイプ宣言 */
void stair_Run(int forward_value, int turn_value);
int spin_func(int spin_end_value);
void stair_Stop(int Floor_Status);

//*****************************************************************************
// 関数名 : stair_main
// 引数 :
// 返り値 : なし
// 概要 :
//
//*****************************************************************************
void stair_main()
{
    int i;    /* index */
    int ave_sum = 0;

    if(Stair_Count == 0)
    {
        /* 各ステータスの初期化 */
        Floor_Status = STAGE_ZERO;
        Run_Mode = RUN_MODE_RUN;
        Stage_One_Spin_Status = SPIN_NOT_FINISH;
        Stage_Two_Spin_Status = SPIN_NOT_FINISH;
        Floor_Search_Flag = FLAG_OFF;
        Ave_Count = 0;
        Waiting_Flag = WAITING_FLAG_OFF;
        Waiting_Count_Flag = WAITING_COUNT_FLAG_OFF;
    }

    /* ジャイロセンサ値を定期的に保存、平均値を算出する */
   if((Stair_Count % 10) == 0)
   {
        /* 40 msごとにgyro_strにジャイロ角速度センサー値を入手 (4 ms周期と仮定) */
         Gyro_Pre_40ms = ev3_gyro_sensor_get_rate(gyro_sensor);
         /* 平均値用にストアする */
         Gyro_Store[Ave_Count] = Gyro_Pre_40ms;
         /* ストア数をカウント */
         Ave_Count++;
         /* ストア数が上限であったときにクリアして0からカウント */
         if(Ave_Count == GYRO_AVE_STR_MAX)
         {
             Ave_Count = 0;
         }

         /* ストア数が上限数溜まったら、平均値を算出する */
         if( Gyro_Store[GYRO_AVE_STR_MAX - 1 ] != 0)
         {
             for(i = 0 ; i < GYRO_AVE_STR_MAX - 1 ; i++)
             {
                 /* 平均値は絶対値で計算 */
                 if (Gyro_Store[i] < 0)
                 {
                     Gyro_Store[i] = (Gyro_Store[i] * (-1) );
                 }
                 ave_sum = ave_sum + Gyro_Store[i];
             }
             Gyro_Ave = (ave_sum / GYRO_AVE_STR_MAX);
         }
   }

   if(Waiting_Flag == WAITING_FLAG_OFF)
   {
       /* ~~~~~~~~~~~~~~~~~~~~~~~~~~0階の動作~~~~~~~~~~~~~~~~~~~~~~~~~~ */
       if(Floor_Status == STAGE_ZERO)
       {
           /* 40 ms前のセンサー値と今のセンサー値の差分が規定値より大きい場合階段にぶつかったと判断する */
           if((Gyro_Pre_40ms - ev3_gyro_sensor_get_rate(gyro_sensor)) > STAGE_ZERO_BORDEAR
             || (Gyro_Pre_40ms - ev3_gyro_sensor_get_rate(gyro_sensor)) < (STAGE_ZERO_BORDEAR * (-1)) )
           {
               /* 階段検知を音で示す */
               ev3_speaker_set_volume(10);
               ev3_speaker_play_tone(NOTE_C4, 100);
               /* フロア検知フラグをONにする */
               Floor_Search_Flag = FLAG_ON;
           }

           /* フロア検知　*/
           if(Floor_Search_Flag == FLAG_ON)
           {
               /* ジャイロの平均値が規定値以下か */
               if(Gyro_Ave < GYRO_AVE_OK_BORDER)
               {
                   /* 規定値以下の数をカウントする */
                   Gyro_Ave_OK_COUNT++;
               }
               else
               {
               /* 規定値以上がいたらリセット */
               Gyro_Ave_OK_COUNT = 0;
               }
               /* ジャイロセンサー値が安定したら　上ったと判定　*/
               if(Gyro_Ave_OK_COUNT > FLOOR_UP_OK_COUNT)
               {
                   /* 一度停止してフロアステータスを上げる　*/
                   Waiting_Flag = WAITING_FLAG_ON;
                   /* フラグ下げ、カウント値の初期化 */
                   Floor_Search_Flag = FLAG_OFF;
                   Gyro_Ave_OK_COUNT = 0;
               }
           }
           /* 倒立制御プログラム呼び出し */
           stair_Run(FLOOR_ZERO_RUN_SPEED, 0);
       }
       /* ~~~~~~~~~~~~~~~~~~~~~~~~~~1階の動作~~~~~~~~~~~~~~~~~~~~~~~~~~ */
       else if(Floor_Status == STAGE_ONE)
       {
           if(Stage_One_Spin_Status == SPIN_NOT_FINISH)
           {
             /* 360度回転したらステータス更新 */
             Stage_One_Spin_Status = spin_func(FLOOR_ONE_SPIN_VALUE);
           }
           else
           {
               /* 40 ms前のセンサー値と今のセンサー値の差分が規定値より大きい場合階段にぶつかったと判断する */
               if((Gyro_Pre_40ms - ev3_gyro_sensor_get_rate(gyro_sensor)) > STAGE_ONE_BORDEAR
                 || (Gyro_Pre_40ms - ev3_gyro_sensor_get_rate(gyro_sensor)) < (STAGE_ONE_BORDEAR * (-1)) )
               {
               /* 階段検知を音で示す */
                   ev3_speaker_set_volume(10);
                   ev3_speaker_play_tone(NOTE_C4, 100);
                   /* フロア検知フラグをONにする */
                   Floor_Search_Flag = FLAG_ON;
               }

               /* フロア検知　*/
               if(Floor_Search_Flag == FLAG_ON)
               {
                   /* ジャイロの平均値が規定値以下か */
                   if(Gyro_Ave < GYRO_AVE_OK_BORDER)
                   {
                       /* 規定値以下の数をカウントする */
                       Gyro_Ave_OK_COUNT++;
                   }
                   else
                   {
                       /* 規定値以上がいたらリセット */
                       Gyro_Ave_OK_COUNT = 0;
                   }
                   /* 規定値以上ジャイロセンサー値が安定したら　上ったと判定　*/
                   if(Gyro_Ave_OK_COUNT > FLOOR_UP_OK_COUNT)
                   {
                       /* 一度停止してフロアステータスを上げる　*/
                       Waiting_Flag = WAITING_FLAG_ON;
                       /* フラグ下げ、カウント値の初期化 */
                       Floor_Search_Flag = FLAG_OFF;
                       Gyro_Ave_OK_COUNT = 0;
                   }
               }
               stair_Run(FLOOR_ONE_RUN_SPEED, 0); /* 倒立制御プログラム呼び出し */
           }
       }
       /* ~~~~~~~~~~~~~~~~~~~~~~~~~~2階の動作~~~~~~~~~~~~~~~~~~~~~~~~~~ */
       else if(Floor_Status == STAGE_TWO)
       {
           if(Stage_Two_Spin_Status == SPIN_NOT_FINISH)
           {
               /* 450度回転したらステータス更新 */
               Stage_Two_Spin_Status = spin_func(FLOOR_TWO_SPIN_VALUE);
           }
           else
           {
               /* 一度停止してフロアステータスを上げる　*/
               Waiting_Flag = WAITING_FLAG_ON;
           }
       }
       else if(Floor_Status == STAGE_LAST)
       {
               /* ちょこっと進んでライントレースに引き渡し */
               stair_Run(FLOOR_TWO_RUN_SPEED, 0); /* 倒立制御プログラム呼び出し */
               /* メインステータスをガレージに引き渡す */
               main_status = STAT_GAREGE;

       } /* end of (Floor_Status == STAGE_ZERO) */
   }
   else /* (Waiting_Flag == WAITING_FLAG_ON) */
   {
       /* 段位ごとの停止とフロアステータスの更新 */
       stair_Stop(Floor_Status);
   }
    Stair_Count++;
}

//*****************************************************************************
// 関数名 : stair_Run
// 引数  : int forward_value, int turn_value
// 返り値 : なし
// 概要 :
//
//*****************************************************************************

void stair_Run(int forward_value, int turn_value)
{
    int32_t motor_ang_l, motor_ang_r;
    int gyro, volt;

    forward = forward_value; /* 前進命令 */
    turn =  turn_value; /* 左旋回命令 */

    tail_control(TAIL_ANGLE_DRIVE); /* バランス走行用角度に制御 */

    /* 倒立振子制御API に渡すパラメータを取得する */
    motor_ang_l = ev3_motor_get_counts(left_motor);
    motor_ang_r = ev3_motor_get_counts(right_motor);
    gyro = ev3_gyro_sensor_get_rate(gyro_sensor);
    volt = ev3_battery_voltage_mV();

    /* 倒立振子制御APIを呼び出し、倒立走行するための */
    /* 左右モータ出力値を得る */
    balance_control(
        (float)forward,
        (float)turn,
        (float)gyro,
        (float)GYRO_OFFSET,
        (float)motor_ang_l,
        (float)motor_ang_r,
        (float)volt,
        (signed char*)&pwm_L,
        (signed char*)&pwm_R);

    /* EV3ではモーター停止時のブレーキ設定が事前にできないため */
    /* 出力0時に、その都度設定する */
    if (pwm_L == 0)
    {
        ev3_motor_stop(left_motor, true);
    }
    else
    {
        ev3_motor_set_power(left_motor, (int)pwm_L);
    }

    if (pwm_R == 0)
    {
         ev3_motor_stop(right_motor, true);
    }
    else
    {
         ev3_motor_set_power(right_motor, (int)pwm_R);
    }
}

//*****************************************************************************
// 関数名 : spin_func
// 引数  : int spin_end_value
// 返り値 : Sipn status
// 概要 :
//*****************************************************************************
int spin_func(int spin_end_value)
{
    int spin_status = SPIN_NOT_FINISH;
    int32_t motor_ang_l;
    int32_t motor_ang_r;
    int gyro;
    int volt;

    /* Runモードのステータス変更 */
    Run_Mode = RUN_MODE_SPIN;

     /* 回転直前の角位置を取得(初回のみ実行) */
     if(Sipn_Func_Count == 0)
     {
        RIGHT_info_first = ev3_motor_get_counts(right_motor);
     }

    /* RIGHT_info値が"700"で走行体が360度回転 */
    /* RIGHT_info値が"875(700 × 1.25)で走行体が約450度回転(900の方がより450度に近い)" */
    /* ライントレースで走行向きを補正できれば875で問題なしと判断 */
    if ((RIGHT_info - RIGHT_info_first) >= spin_end_value )
    {
        forward = turn = 0; /* 前途運動や旋回は一旦ストップ */

        tail_control(TAIL_ANGLE_DRIVE); /* バランス走行用角度に制御 */

        /* 倒立振子制御API に渡すパラメータを取得する */
        motor_ang_l = ev3_motor_get_counts(left_motor);
        motor_ang_r = ev3_motor_get_counts(right_motor);
        gyro = ev3_gyro_sensor_get_rate(gyro_sensor);
        volt = ev3_battery_voltage_mV();

        /* 倒立振子制御APIを呼び出し*/
        /* 左右モータ出力値を得る */
        balance_control(
            (float)forward,
            (float)turn,
            (float)gyro,
            (float)GYRO_OFFSET,
            (float)motor_ang_l,
            (float)motor_ang_r,
            (float)volt,
            (signed char*)&pwm_L,
            (signed char*)&pwm_R);

         ev3_motor_set_power(left_motor, (int)pwm_L);
         ev3_motor_set_power(right_motor, (int)pwm_R);

         /* 回転完了 */
         Sipn_Func_Count = 0;
         spin_status = SPIN_FINISH;
         Run_Mode = RUN_MODE_RUN;
    }
    else
    {
        /* モータのフルパワーのパーセント値を設定 */
        pwm_L = 30;
        pwm_R = 30;

        ev3_motor_set_power(left_motor, ( -1 * (int)pwm_L));
        ev3_motor_set_power(right_motor, (int)pwm_R);
        /* 右モータの角位置を取得する */
        RIGHT_info = ev3_motor_get_counts(right_motor);
    }

    Sipn_Func_Count++;
    return(spin_status);
}


//*****************************************************************************
// 関数名 : stair_Stop
// 引数  :
// 返り値 : なし
// 概要 :
//
//*****************************************************************************

void stair_Stop(int Floor_Status)
{
    int32_t motor_ang_l, motor_ang_r;
    int gyro, volt;

    if(Floor_Status == STAGE_ZERO)
    {
        if(Waiting_Count_Flag == WAITING_COUNT_FLAG_OFF)
        {
            Waiting_Count_Value = FLOOR_ZERO_WAITING_COUNT;
            Waiting_Count_Flag = WAITING_COUNT_FLAG_ON;
        }

        Waiting_Count_Value--;

        if(Waiting_Count_Value < 0)
        {
            Waiting_Flag = WAITING_FLAG_OFF;
            Floor_Status = STAGE_ONE;
        }
    }
    else if(Floor_Status == STAGE_ONE)
    {
        if(Waiting_Count_Flag == WAITING_COUNT_FLAG_OFF)
        {
            Waiting_Count_Value = FLOOR_ONE_WAITING_COUNT;
            Waiting_Count_Flag = WAITING_COUNT_FLAG_ON;
        }

        Waiting_Count_Value--;

        if(Waiting_Count_Value < 0)
        {
            Waiting_Flag = WAITING_FLAG_OFF;
            Floor_Status = STAGE_TWO;
        }
    }
    else
    {
        if(Waiting_Count_Flag == WAITING_COUNT_FLAG_OFF)
        {
            Waiting_Count_Value = FLOOR_TWO_WAITING_COUNT;
            Waiting_Count_Flag = WAITING_COUNT_FLAG_ON;
        }

        Waiting_Count_Value--;

        if(Waiting_Count_Value < 0)
        {
            Waiting_Flag = WAITING_FLAG_OFF;
            Floor_Status = STAGE_LAST;
        }
    }

    forward = 0; /* 前進命令 */
    turn =  0; /* 左旋回命令 */

    tail_control(TAIL_ANGLE_DRIVE); /* バランス走行用角度に制御 */

    /* 倒立振子制御API に渡すパラメータを取得する */
    motor_ang_l = ev3_motor_get_counts(left_motor);
    motor_ang_r = ev3_motor_get_counts(right_motor);
    gyro = ev3_gyro_sensor_get_rate(gyro_sensor);
    volt = ev3_battery_voltage_mV();

    /* 倒立振子制御APIを呼び出し、倒立走行するための */
    /* 左右モータ出力値を得る */
    balance_control(
        (float)forward,
        (float)turn,
        (float)gyro,
        (float)GYRO_OFFSET,
        (float)motor_ang_l,
        (float)motor_ang_r,
        (float)volt,
        (signed char*)&pwm_L,
        (signed char*)&pwm_R);

    /* EV3ではモーター停止時のブレーキ設定が事前にできないため */
    /* 出力0時に、その都度設定する */
    if (pwm_L == 0)
    {
        ev3_motor_stop(left_motor, true);
    }
    else
    {
        ev3_motor_set_power(left_motor, (int)pwm_L);
    }

    if (pwm_R == 0)
    {
         ev3_motor_stop(right_motor, true);
    }
    else
    {
         ev3_motor_set_power(right_motor, (int)pwm_R);
    }
}
