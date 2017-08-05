#include "line_trace.h"
#include "Distance.h"

#define DISTANCE_NOTIFY (500.0)

int grade_test_cnt = 0;     /*  音カウント */
int grade_test_flg = 0;     /*  huragu */
int grade_test_touritu = 0;     /*  音カウント */
int touritu_flg = 0;/* 倒立状態 */
int end_flag = 0;

//*****************************************************************************
// 関数名 : garage_main
// 引数 : 
// 返り値 : なし
// 概要 : 
//       
//*****************************************************************************

signed char forward;      /* 前後進命令 */
signed char turn;         /* 旋回命令 */
signed char pwm_L, pwm_R; /* 左右モータPWM出力 */

void garage_main()
{
    int32_t motor_ang_l, motor_ang_r;
    int gyro, volt;

    if (ev3_button_is_pressed(BACK_BUTTON)) return;
    if(grade_test_flg == 0){
        tail_control(TAIL_ANGLE_DRIVE); /* バランス走行用角度に制御 */
    }
    forward = 30;
    turn = 0;
    if(grade_test_flg == 1)
    {
        forward = turn = 0;
    }
    if(grade_test_touritu >= 2900){
        forward = -100;
    }
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

    if(grade_test_flg == 1)
    {
        grade_test_touritu++;
        if(grade_test_touritu >= 1000)
        {
            if(grade_test_touritu >= 2500){
                tail_control(TAIL_ANGLE_STAND_UP); /* 倒立制御を削除 尻尾を下す */
            }else if(grade_test_touritu >= 2000){
                tail_control((TAIL_ANGLE_STAND_UP-20)); /* 倒立制御を削除 尻尾を下す */
            }else if(grade_test_touritu >= 1500){
                tail_control((TAIL_ANGLE_STAND_UP-30)); /* 倒立制御を削除 尻尾を下す */
            }
#if 0
            }else if(grade_test_touritu >= 2000){
                tail_control((TAIL_ANGLE_STAND_UP-40)); /* 倒立制御を削除 尻尾を下す */
            }else if(grade_test_touritu >= 1000){
                tail_control((TAIL_ANGLE_STAND_UP-50)); /* 倒立制御を削除 尻尾を下す */
            }
#endif            
            if(grade_test_touritu >= 3000)
            {
                if(grade_test_touritu < 3010){
                    ev3_speaker_set_volume(10); 
                    ev3_speaker_play_tone(NOTE_C4, 100);
                }
                if(touritu_flg == 0){
                    ev3_motor_stop(right_motor, true);
                    ev3_motor_stop(left_motor, true);
                    touritu_flg = 1;
                }
            }
        }else{
            tail_control(TAIL_ANGLE_STAND_UP-60); /* バランス走行用角度に制御 */
        }
    }
    if(touritu_flg == 1) {
        end_flag = 1;
        return;
    }
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
	
	Distance_update(); /* 移動距離加算 */
	
	if( Distance_getDistance() > DISTANCE_NOTIFY )
	{
		/* DISTANCE_NOTIFY以上進んだら音を出す */
		ev3_speaker_set_volume(100); 
		ev3_speaker_play_tone(NOTE_C4, 100);
	    grade_test_cnt++;
	    if(grade_test_cnt >= 1)
	    {
	        grade_test_flg  =1;


	    }
		/* 距離計測変数初期化 */
		Distance_init();
	}
	
}
int garage_end(void){
    return end_flag;
}
/* end of file */
