/*
 * look_up_gate.c
 */

#include "look_up_gate.h"
#include "Distance.h"

#define LOOK_UP_GATE_PASSING_ANGLE  90  /* ルックアップゲート通過時の角度 */
#define SLEEP_TIME                  1000 /* 暫定であるスリープ制御の時間 */
#define FORWARD_DISTANCE            30   /* 前進距離 */
#define TAIL_ANGLE_LUG 68        /* LUG尻尾角度[度] */
#define MOTOR_ANGLE   720        /* motorの角度[度] */
#define PWM_FALLDOWN   30        /* LUGの上体倒しのパワー */
#define PWM_AHEAD      30        /* LUGの前進・後退パワー */

volatile static LOOK_UP_GATE_STATUS  look_up_gate_status_    = LOOK_UP_GATE_STAT_UNKNOWN;    /* ルックアップゲートの攻略状態 */

//*****************************************************************************
// 関数名 : look_up_gate_main
// 引数 : 
// 返り値 : なし
// 概要 : 
//       
//*****************************************************************************
unsigned int look_up_gate_main(void)
{
    static unsigned int  rev_t          = 0;    // (サーボモータ回転角度)現在値_尻尾
    static unsigned int  phase          = 0;    // 状態
    static unsigned int  run_count_0    = 0;    // 時間経過監視カウンタ
    static unsigned int  run_count_1    = 0;    // 時間経過監視カウンタ
    static unsigned int  run_count_4    = 0;    // 時間経過監視カウンタ 
    static unsigned int  ret            = 0;    // 戻り値 
    static unsigned int  flg            = 0;    // 通過フラグ(一度通過したらフラグを立てる)

    /* 関数初回呼び出し時、鳴動してルックアップゲートの処理開始を通知 */
    if(run_count_0 == 0) {
	    ev3_speaker_set_volume(100); 
	    ev3_speaker_play_tone(NOTE_E4, 100);
    }

    switch (phase) {

    // 停止→後傾(phase1)
    case 0:
		/*** インクリメント ***/
	    run_count_0++;

    	/* 0~2秒間 */
        if(run_count_0 < 500)
        {
        	/* 0 */
        	if (0 <= run_count_0 && 125 > run_count_0) {
        		printf("1\r\n");
			    rev_t = LOOK_UP_GATE_PASSING_ANGLE -60;
			    look_up_gate_tail_control(rev_t);
        	}
        	else if (125 <= run_count_0 && 250 > run_count_0) {
			    rev_t = LOOK_UP_GATE_PASSING_ANGLE -40;
			    look_up_gate_tail_control(rev_t);
        	}
        	else if (250 <= run_count_0 && 500 > run_count_0) {
			    rev_t = LOOK_UP_GATE_PASSING_ANGLE -20;
			    look_up_gate_tail_control(rev_t);
        	}
        	else {
			    rev_t = LOOK_UP_GATE_PASSING_ANGLE -5;
			    look_up_gate_tail_control(rev_t);
        	}

        	/* 前進0、転回0で倒立振り子制御する */
            do_balance(0,0);

            return;
        }

    	/* 2~2.5秒間 */
        if(run_count_0 < 625)
        {
        	printf("4\r\n");
            /* 以下の角度に尻尾を制御 */
		    rev_t = LOOK_UP_GATE_PASSING_ANGLE -10;
		    look_up_gate_tail_control(rev_t);

            ev3_motor_set_power(left_motor, (int)20);
            ev3_motor_set_power(right_motor, (int)20);
        	/* 車輪を完全停止させる */
//			ev3_motor_stop(left_motor, true);
//        	ev3_motor_stop(right_motor, true);
        	return;
        }

    	/* 2.5秒後 */
        if(run_count_0 >= 625)
        {
        	printf("5\r\n");
        	/* 車輪を完全停止させる */
			ev3_motor_stop(left_motor, true);
        	ev3_motor_stop(right_motor, true);

            /* 以下の角度に尻尾を制御 */
		    rev_t = LOOK_UP_GATE_PASSING_ANGLE -10;
		    look_up_gate_tail_control(rev_t);
        }

    	/* phase1に移行 */
    	phase++;

		break;

    // 後傾→前進→停止
    case 1:
		/*** インクリメント ***/
    	run_count_1++;

    	/* phase 1移行後、1~5秒 */
        if(run_count_1 >= 0 && run_count_1 < 1000)
        {
		    ev3_motor_set_power(left_motor, (int)10);
		    ev3_motor_set_power(right_motor, (int)10);
		    rev_t = LOOK_UP_GATE_PASSING_ANGLE -10;
		    look_up_gate_tail_control(rev_t);
        }
    	/* phase 1移行後、5秒後 */
        else if(run_count_1 >= 1000 && run_count_1 < 1250)
        {
        	/* 車輪を完全停止させる */
			ev3_motor_stop(left_motor, true);
        	ev3_motor_stop(right_motor, true);
		    rev_t = LOOK_UP_GATE_PASSING_ANGLE -10;
		    look_up_gate_tail_control(rev_t);
        	printf("5\r\n");
        }
    	else {
		    rev_t = LOOK_UP_GATE_PASSING_ANGLE -10;
		    look_up_gate_tail_control(rev_t);

			/* phase2に移行 */
//    		phase = 2;
			
	    	/* phase4に移行 */
	    	phase = 4;
    	}


        break;

    // 後退
    case 2:
        break;

    // 前進→停止
    case 3:
        break;

    // 直立
    case 4:
		/*** インクリメント ***/
    	run_count_4++;
    	
//    	look_up_gate_tail_control(LOOK_UP_GATE_PASSING_ANGLE +10);
    	
    	rev_t = ev3_motor_get_counts(tail_motor);
//    	if (rev_t == LOOK_UP_GATE_PASSING_ANGLE +13) {
    	if (rev_t == TAIL_ANGLE_STAND_UP) {
    		ev3_motor_stop(tail_motor, true);
    		do_balance(0,0);
   		}
    	else {
    		ev3_motor_set_power(tail_motor, 10);  
    	}
/* 0.5s毎に2度尻尾を下げるのではパワー不足のため、以下は使用しない */
#if 0
    	/* 0~6秒間 */
        if(run_count_4 < 1500)
        {
        	if (0 <= run_count_4 && 125 > run_count_4) {
        		printf("1\r\n");
			    ev3_motor_set_power(left_motor, (int)-2);
			    ev3_motor_set_power(right_motor, (int)-2);
			    rev_t = LOOK_UP_GATE_PASSING_ANGLE -5;
			    look_up_gate_tail_control(rev_t);
        	}
        	else if (125 <= run_count_4 && 250 > run_count_4) {
			    ev3_motor_set_power(left_motor, (int)-2);
			    ev3_motor_set_power(right_motor, (int)-2);
			    rev_t = LOOK_UP_GATE_PASSING_ANGLE;
			    look_up_gate_tail_control(rev_t);
        	}
        	else if (250 <= run_count_4 && 375 > run_count_4) {
			    ev3_motor_set_power(left_motor, (int)-2);
			    ev3_motor_set_power(right_motor, (int)-2);
			    rev_t = LOOK_UP_GATE_PASSING_ANGLE +2;
			    look_up_gate_tail_control(rev_t);
        	}
        	else if (375 <= run_count_4 && 500 > run_count_4) {
			    ev3_motor_set_power(left_motor, (int)-2);
			    ev3_motor_set_power(right_motor, (int)-2);
			    rev_t = LOOK_UP_GATE_PASSING_ANGLE +4;
			    look_up_gate_tail_control(rev_t);
        	}
        	else if (500 <= run_count_4 && 625 > run_count_4) {
			    ev3_motor_set_power(left_motor, (int)-2);
			    ev3_motor_set_power(right_motor, (int)-2);
			    rev_t = LOOK_UP_GATE_PASSING_ANGLE +6;
			    look_up_gate_tail_control(rev_t);
        	}
        	else if (625 <= run_count_4 && 750 > run_count_4) {
			    ev3_motor_set_power(left_motor, (int)-2);
			    ev3_motor_set_power(right_motor, (int)-2);
			    rev_t = LOOK_UP_GATE_PASSING_ANGLE +8;
			    look_up_gate_tail_control(rev_t);
        	}
        	else if (825 <= run_count_4 && 1000 > run_count_4) {
			    ev3_motor_set_power(left_motor, (int)-2);
			    ev3_motor_set_power(right_motor, (int)-2);
			    rev_t = LOOK_UP_GATE_PASSING_ANGLE +10;
			    look_up_gate_tail_control(rev_t);
        	}
        	else if (1125 <= run_count_4 && 1250 > run_count_4) {
			    ev3_motor_set_power(left_motor, (int)-2);
			    ev3_motor_set_power(right_motor, (int)-2);
			    rev_t = LOOK_UP_GATE_PASSING_ANGLE +12;
			    look_up_gate_tail_control(rev_t);
        	}
        	else if (1250 <= run_count_4 && 1500 > run_count_4) {
			    ev3_motor_set_power(left_motor, (int)-2);
			    ev3_motor_set_power(right_motor, (int)-2);
			    rev_t = LOOK_UP_GATE_PASSING_ANGLE +14;
			    look_up_gate_tail_control(rev_t);
        	}
        	else {
			    rev_t = LOOK_UP_GATE_PASSING_ANGLE +16;
			    look_up_gate_tail_control(rev_t);
	        	/* 車輪を完全停止させる */
				ev3_motor_stop(left_motor, true);
		    	ev3_motor_stop(right_motor, true);
	            do_balance(0,0);
        	}
        }
    	else {
        	/* 前進0、転回0で倒立振り子制御する */
            do_balance(0,0);
    	}

    	/* 8秒後 */
        if(run_count_4 > 2000)
        {
        	/* 前進0、転回0で倒立振り子制御する */
            do_balance(0,0);
        	/* 10秒後 */
        	if (run_count_4 > 2500) {
			    rev_t = TAIL_ANGLE_DRIVE;
			    look_up_gate_tail_control(rev_t);
        	}
        }
#endif
        break;
    case 5:
        if (flg == 0) {
            ret = 1;
            flg = 1;
        }
        break;

    }

    /* 戻るボタンor転んだら終了 */
    if(ev3_button_is_pressed(BACK_BUTTON))
    {
        wup_tsk(MAIN_TASK);
    }

        return (ret);

}

//*****************************************************************************
// 関数名 : do_balance
// 引数 : ロボ制御情報、速度、回転
// 返り値 : なし
// 概要 : 与えられた情報を用いて、倒立振子制御を実施し、左右モータへ反映する。
//*****************************************************************************
void do_balance( signed char forward, signed char turn)
{
	signed char pwm_L = 0, pwm_R = 0; /* 左右モータPWM出力 */
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
}

//*****************************************************************************
// 関数名 : look_up_gate_tail_control
// 引数 : angle (モータ目標角度[度])
// 返り値 : 無し
// 概要 : 走行体完全停止用モータの角度制御
//*****************************************************************************
int look_up_gate_tail_control(signed int angle)
{
    int motor_stop = 0;
    float pwm = (float)(angle - ev3_motor_get_counts(tail_motor))*P_GAIN; /* 比例制御 */
    /* PWM出力飽和処理 */
    if (pwm > PWM_ABS_MAX)
    {
        pwm = PWM_ABS_MAX;
    }
    else if (pwm < -PWM_ABS_MAX)
    {
        pwm = -PWM_ABS_MAX;
    }

    if (pwm == 0)
    {
        ev3_motor_stop(tail_motor, true);
        motor_stop = 1;
    }
    else
    {
        ev3_motor_set_power(tail_motor, (signed char)pwm);
        motor_stop = 0;
    }
    
    return    motor_stop;
}

