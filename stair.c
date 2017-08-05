#include "stair.h"

signed char forward;      /* 前後進命令 */
signed char turn;         /* 旋回命令 */
signed char pwm_L , pwm_R; /* 左右モータPWM出力 */
static int RIGHT_info = 0;
#if 1 /* ikeda */
static int  RIGHT_info_first = 0;      /* 回転直前の角位置 */
static int  first_time = 1;            /* 回転直前の角位置ステータス(初回のみ) */
#endif /* ikeda */

//*****************************************************************************
// 関数名 : stair_main
// 引数 : 
// 返り値 : なし
// 概要 : 
//       
//*****************************************************************************
void stair_main()
{
	int32_t motor_ang_l, motor_ang_r;
    int gyro, volt;

     /* 回転直前の角位置を取得(初回のみ実行) */
	 if(first_time != 0)
	 {
	    RIGHT_info_first = ev3_motor_get_counts(right_motor);
	    first_time = 0;
	 }
    /* EV3ではモーター停止時のブレーキ設定が事前にできないため */
    /* 出力0時に、その都度設定する */
//    if (RIGHT_info >= 1000 ) 

    /* RIGHT_info値が"700"で走行体が360度回転 */
    /* RIGHT_info値が"875(700 × 1.25)で走行体が約450度回転(900の方がより450度に近い)" */
    /* ライントレースで走行向きを補正できれば875で問題なしと判断 */
    if ((RIGHT_info - RIGHT_info_first) >= 875 )
    {
         tail_control(TAIL_ANGLE_DRIVE); /* バランス走行用角度に制御 */
         forward = turn = 0; /* 前途運動や旋回は一旦ストップ */
    	
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
    	
    }
    else
    {
    	/* 尻尾を下す */
		tail_control(TAIL_ANGLE_STAND_UP);

    	/* モータのフルパワーのパーセント値を設定 */
	    pwm_L = 30;
	    pwm_R = 30;
	
        ev3_motor_set_power(left_motor, -1 * (int)pwm_L);
        ev3_motor_set_power(right_motor, (int)pwm_R);    	
    	/* 右モータの角位置を取得する */
    	RIGHT_info = ev3_motor_get_counts(right_motor);

    }
    
}

/* end of file */
