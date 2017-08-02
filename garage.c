#include "garage.h"
#include "app.h"
#include "Distance.h"

/* nakagawa Add_STA */
#define SLOW_DISTANCE (100) /* 低速距離暫定 TBD */
#define STOP_DISTANCE (150) /* ガレージイン距離暫定 TBD*/
/* nakagawa Add_END */

signed char forward;      /* 前後進命令 */
signed char turn;         /* 旋回命令 */
signed char pwm_L, pwm_R; /* 左右モータPWM出力 */

/* nakagawa Add_STA */
unsigned char slow_flag = 0;
float left_zankyori = 0.0;
float right_zankyori = 0.0;
extern int LIGHT_WHITE;         /* 白色の光センサ値 */
extern int LIGHT_BLACK;
/* nakagawa Add_END */

//*****************************************************************************
// 関数名 : garage_main
// 引数 : 
// 返り値 : なし
// 概要 : 
//       
//*****************************************************************************
void garage_main()
{
    int32_t motor_ang_l, motor_ang_r;
    int gyro, volt;

/* nakagawa Add_STA */
    float distance4msL = 0.0; //左タイヤの4ms間の距離
    float distance4msR = 0.0; //右タイヤの4ms間の距離
    float distance = 0.0;     //走行距離
/* nakagawa Add_END */

     if (ev3_button_is_pressed(BACK_BUTTON)) return;

     tail_control(TAIL_ANGLE_DRIVE); /* バランス走行用角度に制御 */

     if (sonar_alert() == 1) /* 障害物検知 */
     {
         forward = turn = 0; /* 障害物を検知したら停止 */
     }
     else
     {
         forward = 30; /* 前進命令 */
         if (ev3_color_sensor_get_reflect(color_sensor) >= (LIGHT_WHITE + LIGHT_BLACK)/2)
         {
             turn =  20; /* 左旋回命令 */
         }
         else
         {
             turn = -20; /* 右旋回命令 */
         }
     }

     /* 倒立振子制御API に渡すパラメータを取得する */
     motor_ang_l = ev3_motor_get_counts(left_motor);/*左モーター*/
     motor_ang_r = ev3_motor_get_counts(right_motor);/*右モーター*/
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

/* nakagawa Add_STA */
     /* ガレージイン起動後走行距離が○○進んだら速度を低下させる */
    if(distance > SLOW_DISTANCE && slow_flag == 0){
        slow_flag = 1;
        pwm_L = pwm_L /2;
        pwm_R = pwm_R /2;
    }
/* nakagawa Add_END */

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

/* nakagawa Add_STA */
    Distance_update();/* 距離更新（4ms間の移動距離を毎回加算している） */

    distance4msL= Distance_getDistance4msLeft();    /* 左距離取得 */
    distance4msR= Distance_getDistance4msRight();   /* 右距離取得 */

    left_zankyori  += distance4msL;
    right_zankyori += distance4msR;

    distance += (left_zankyori + right_zankyori) / 2.0; //左右タイヤの走行距離を足して割る

    /* ガレージイン起動後走行距離が○○進んだら停止させる */
    if (distance > STOP_DISTANCE){
        ev3_motor_stop(left_motor, true);
        ev3_motor_stop(right_motor, true);
        
    }
/* nakagawa Add_END */
}

static void garage_touritu_stop(void){
    /* T B D */
}
/* end of file */
