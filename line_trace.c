/*
 * line_tarce.h
 * 
 */

#include "line_trace.h"

#define DELTA_T 0.004
signed char forward = 100;              /* 前後進命令 */
signed char turn;                 /* 旋回命令 */
signed char pwm_L, pwm_R;         /* 左右モータPWM出力 */
static float integral=0;          /* I制御 */
static int diff [2];              /* カラーセンサの差分 */ 
int count  = 0;                   /* ログ出力 */
int blackcount = 0;
signed char pwm_L=0, pwm_R=0; /* 左右モータPWM出力 */
static int curveCount = 0;
static int curve = 0;

/* PIDパラメータ */
#define KP 0.8
#define KI 0.58
#define KD 0.04

//*****************************************************************************
// 関数名 : line_tarce_main
// 引数 : 
// 返り値 : なし
// 概要 : 
//       
//*****************************************************************************
void line_tarce_main(int gray_color)
{
    signed char turn;         /* 旋回命令 */

    int32_t motor_ang_l, motor_ang_r;
    int gyro, volt;
    uint8_t color_sensor_reflect;
    
    tail_control(TAIL_ANGLE_DRIVE); /* バランス走行用角度に制御 */

    color_sensor_reflect= ev3_color_sensor_get_reflect(color_sensor);

    int temp_p=1000;
    int temp_d=1000;


    /* PID制御 */
    float p,i,d;
    diff[0] = diff[1];
    diff[1] = color_sensor_reflect - ((gray_color)/2);
    integral += (diff[1] + diff[0]) / 2.0 * DELTA_T;
    
    p = KP * diff[1];
    i = KI * integral;
    d = KD * (diff[1]-diff[0]) / DELTA_T;
    
    turn = p + i + d;
    temp_p = p;
    temp_d = d;
    
    /* モータ値調整 */
    if(100 < turn)
    {
        turn = 100;
    }
    else if(turn < -100)
    {
        turn = -100;
    }

    /* 倒立振子制御API に渡すパラメータを取得する */
    motor_ang_l = ev3_motor_get_counts(left_motor);
    motor_ang_r = ev3_motor_get_counts(right_motor);
    gyro = ev3_gyro_sensor_get_rate(gyro_sensor);
    volt = ev3_battery_voltage_mV();

    // カーブ検知
    int L = pwm_L-'0';
    int R = pwm_R-'0';
    if((L-R>15)||(R-L>15))
    {
        curve++;
        if(curve == 25)
        {
            forward = 80;
        }
    }
    else
    {
        curve = 0;
    }



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
    
    /* 戻るボタンor転んだら終了 */
    if(ev3_button_is_pressed(BACK_BUTTON))
    {
        wup_tsk(MAIN_TASK);
    }
    if(gyro < -150 || 150 < gyro)
    {
        wup_tsk(MAIN_TASK);
    }

    /* センサ値が目標値＋15以上をblackcount回数
       連続検知したらグレーとみなす処理 */
    // グレーの値　50～60くらい
    if( color_sensor_reflect > ((gray_color)/2))
    {
        blackcount++;
        if(blackcount==100)
        {
            // 10回連続白を検知 
            ev3_speaker_set_volume(30); 
            ev3_speaker_play_tone(NOTE_C4, 100);
        }
    }
    else
    {
        blackcount=0;
    }
    
    /* ログ出力 */
    count++;
    log_Str(color_sensor_reflect,(int16_t)forward, (int16_t)L, (int16_t)R, (int16_t)count);

}

/* end of file */
