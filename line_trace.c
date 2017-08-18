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
static float diff [2] = {0,0};      /* カラーセンサの差分 */ 
static int black_count = 0;
static int curve_count = 0;
signed char pwm_L=0, pwm_R=0; /* 左右モータPWM出力 */

/* PIDパラメータ */
#define KP 0.8
#define KI 0
#define KD 0.04

static float normalize_color_sensor_reflect(uint8_t color_sensor_refelect, signed char light_white, signed char light_black);

//*****************************************************************************
// 関数名 : line_tarce_main
// 引数 :   signed char light_white 白のセンサ値
//          signed char light_black 黒のセンサ値
// 返り値 : なし
// 概要 : 
//       
//*****************************************************************************
void line_tarce_main(signed char light_white, signed char light_black)
{
    signed char turn;         /* 旋回命令 */

    uint8_t color_sensor_reflect;
    
    tail_control(TAIL_ANGLE_DRIVE); /* バランス走行用角度に制御 */

    /* 光センサ値取得 */
    color_sensor_reflect= ev3_color_sensor_get_reflect(color_sensor);
    
    /* PID制御によりターン値求める */
    turn = pid_control(color_sensor_reflect, light_white, light_black);

    /* カーブ検知(作成途中) */
    if((pwm_L-pwm_L>15)||(pwm_L-pwm_L>15))
    {
        curve_count++;
        if(curve_count == 25)
        {
            forward = 80;
        }
    }
    else
    {
        curve_count = 0;
    }
    
    /* 倒立振子制御処理 */
    balanceControl(forward, turn);
    
    /* センサ値が目標値＋15以上をblack_count回数
       連続検知したらグレーとみなす処理 */
    // グレーの値　50～60くらい
    if( color_sensor_reflect > (light_white+light_black)/2)
    {
        black_count++;
        if(black_count==100)
        {
            // 10回連続白を検知 
            ev3_speaker_set_volume(30); 
            ev3_speaker_play_tone(NOTE_C4, 100);
        }
    }
    else
    {
        black_count=0;
    }
}

//*****************************************************************************
// 関数名 : pid_control
// 引数 : uint8_t color_sensor_reflect 光センサ値
//        signed char light_white      白のセンサ値
//        signed char light_black      黒のセンサ値
// 返り値 : signed char                ターン値
// 概要 : 
//
//*****************************************************************************
signed char pid_control(uint8_t color_sensor_reflect, signed char light_white, signed char light_black)
{
    /* PID制御によりターン値を求める */
    float p,i,d;
    float normalize_reflect_value;
    float target = 0.5;

    normalize_reflect_value = normalize_color_sensor_reflect(color_sensor_reflect, light_white, light_black);

    diff[0] = diff[1];
    diff[1] = normalize_reflect_value - target;
    integral += (diff[1] + diff[0]) / 2.0 * DELTA_T;
    
    p = KP * diff[1];
    i = KI * integral;
    d = KD * (diff[1]-diff[0]) / DELTA_T;
    
    turn = (p + i + d) * 100; //正規化で出した値を0-100にするため
    
    /* モータ値調整 */
    if(100 < turn)
    {
        turn = 100;
    }
    else if(turn < -100)
    {
        turn = -100;
    }
    
    /* ログ出力 */
    log_Str(color_sensor_reflect, normalize_reflect_value, p, i, d);
    
    return turn;
}

//*****************************************************************************
// 関数名 : normalize_color_sensor_reflect
// 引数 : signed char color_sensor_reflect 光センサ値
//        signed char light_white          キャリブレーションした白の値
//        signed char light_black          キャリブレーションした黒の値
// 返り値 : float                    正規化した光センサ値
// 概要 : 光センサ値をキャリブレーションしたデータをもとに0~1で正規化する
//
//*****************************************************************************
static float normalize_color_sensor_reflect(uint8_t color_sensor_reflect, signed char light_white, signed char light_black)
{
    return (float)(color_sensor_reflect - light_black) / (float)(light_white - light_black);
}


//*****************************************************************************
// 関数名 : balanceControl
// 引数 : signed char forward 前進命令
//        , signed char turn  ターン値
// 返り値 : 
// 概要 : 
//
//*****************************************************************************
void balanceControl(signed char forward, signed char turn)
{
    int32_t motor_ang_l, motor_ang_r;
    int gyro, volt;
    
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

    /* 戻るボタンor転んだら終了 */
    if(ev3_button_is_pressed(BACK_BUTTON))
    {
        wup_tsk(MAIN_TASK);
    }
    if(gyro < -150 || 150 < gyro)
    {
        wup_tsk(MAIN_TASK);
    }
}

/* end of file */
