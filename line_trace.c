/*
 * line_tarce.h
 * 
 */

#include "line_trace.h"

#define DELTA_T 0.004
signed char forward = DEFAULT_SPEED;              /* 前後進命令 */
signed char turn;                 /* 旋回命令 */
static float integral=0;          /* I制御 */
static float diff [2] = {0,0};      /* カラーセンサの差分 */ 
static int black_count = 0;
signed char pwm_L=0, pwm_R=0; /* 左右モータPWM出力 */

static float kp = KP;
static float kd = KD;
static float target = TARGET;


static float normalize_color_sensor_reflect(uint8_t color_sensor_refelect, signed char light_white, signed char light_black);

unsigned char detect_curve(signed char turn){
    static int old_turn[TURN_MAX];
    static int turnIndex = 0;
    static int plus_turn_num = 0;
    static int minus_turn_num = 0;
    static int neutral_turn_num = TURN_MAX;

    float minus_per, plus_per;
    int remove_turn = old_turn[turnIndex];
//    float turn_plus_threshold = TURN_THRESHOLD * ((1 - target) * 2);
//    float turn_minus_threshold = TURN_THRESHOLD * (target * 2);
    float turn_plus_threshold = TURN_THRESHOLD;
    float turn_minus_threshold = TURN_THRESHOLD;

    old_turn[turnIndex++] = turn;
    turnIndex %= TURN_MAX;

    if(remove_turn > turn_plus_threshold)
    {
       plus_turn_num--; 
    }
    else if ((remove_turn * -1) > turn_minus_threshold)
    {
       minus_turn_num--; 
    }
    else
    {
        neutral_turn_num--;
    }

    if(turn > turn_plus_threshold)
    {
       plus_turn_num++; 
    }
    else if ((turn * -1) > turn_minus_threshold)
    {
       minus_turn_num++; 
    }
    else
    {
        neutral_turn_num++;
    }

    minus_per = (float)minus_turn_num / TURN_MAX;
    plus_per = (float)plus_turn_num / TURN_MAX;
    log_Str(144,plus_turn_num,minus_turn_num,neutral_turn_num, (minus_per > TURN_PER_THRESHOLD || plus_per > TURN_PER_THRESHOLD));

    if (minus_per > TURN_PER_THRESHOLD) {
        return -1;
    } else if (plus_per > TURN_PER_THRESHOLD) {
        return 1;
    }
    return 0;
}

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
    unsigned char curve;

    uint8_t color_sensor_reflect;
    
    tail_control(TAIL_ANGLE_DRIVE); /* バランス走行用角度に制御 */

    /* 光センサ値取得 */
    color_sensor_reflect= ev3_color_sensor_get_reflect(color_sensor);
    
    /* PID制御によりターン値求める */
    turn = pid_control(color_sensor_reflect, light_white, light_black);

    /* カーブ検知(作成途中) */
    curve = detect_curve(turn);
    if (curve) {
        if (curve == 1) {
            target = TARGET - CURVE_TARGET_OFFSET;
        } else if (curve == -1) {
            target = TARGET + CURVE_TARGET_OFFSET;
        }
        kp = CURVE_KP;
        kd = CURVE_KD;
        ev3_speaker_set_volume(15); 
        ev3_speaker_play_tone(NOTE_C4, 100);
        forward = CURVE_SPEED;
    } else {
        target = TARGET;
        kp = KP;
        kd = KD;
        forward = DEFAULT_SPEED;
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
//            ev3_speaker_set_volume(30); 
//            ev3_speaker_play_tone(NOTE_C4, 100);
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

    normalize_reflect_value = normalize_color_sensor_reflect(color_sensor_reflect, light_white, light_black);

    diff[0] = diff[1];
    diff[1] = normalize_reflect_value - target;
    integral += (diff[1] + diff[0]) / 2.0 * DELTA_T;
    
    p = kp * diff[1];
    i = KI * integral;
    d = kd * (diff[1]-diff[0]) / DELTA_T;
    
    turn = (p + i + d) * KTURN; //正規化で出した値を0-100にするため

    //targetの値によるturn値の補正
    if (turn > 0) {
        turn = TURN_PLUS_CORRECT_EXP;
    } else {
        turn = TURN_MINUS_CORRECT_EXP;
    }
    
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
    log_Str(color_sensor_reflect, normalize_reflect_value, p, d, turn);
    
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
