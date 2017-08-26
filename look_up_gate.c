/*
 * look_up_gate.c
 */

#include "look_up_gate.h"
#include "Distance.h"

#define LOOK_UP_GATE_PASSING_ANGLE  120  /* ルックアップゲート通過時の角度 */
#define SLEEP_TIME                  1000 /* 暫定であるスリープ制御の時間 */
#define FORWARD_DISTANCE            30   /* 前進距離 */

volatile static LOOK_UP_GATE_STATUS  look_up_gate_status_    = LOOK_UP_GATE_STAT_UNKNOWN;    /* ルックアップゲートの攻略状態 */
static int                  tail_angle_             = 0;                            /* 角度 */
// static float                process_start_location_ = 0;                            /* ゲート通過開始位置 */
static int                  motor_stop              = 0;

//*****************************************************************************
// 関数名 : look_up_gate_main
// 引数 : 
// 返り値 : なし
// 概要 : 
//       
//*****************************************************************************
void look_up_gate_main(void)
{
    static unsigned int  base_rev_l      = 0;        // (サーボモータ回転角度)基準値_左
    static unsigned int  base_rev_r      = 0;        // (サーボモータ回転角度)基準値_右
    static unsigned int   pwm_l           = 40;       // (サーボモータ回転角度)設定値_左
    static unsigned int   pwm_r           = 40;       // (サーボモータ回転角度)設定値_右
    static unsigned int   pwm_t           = 40;       // (サーボモータ回転角度)設定値_尻尾
    static unsigned int   rev_l           = 0;        // (サーボモータ回転角度)現在値_左
    static unsigned int   rev_r           = 0;        // (サーボモータ回転角度)現在値_左
    static unsigned int   rev_t           = 0;        // (サーボモータ回転角度)現在値_尻尾
    static unsigned int   phase           = 0;        // 状態
    unsigned int          forward         = 25;       // 前後進命令
    static unsigned int  run_count       = 0;        // 時間経過監視カウンタ
    static unsigned int  run_count_endofphase0= 0;   // 時間経過監視カウンタ(phase0終了値)
    static unsigned int  run_count_endofphase1= 0;   // 時間経過監視カウンタ(phase1終了値)
    static unsigned int  run_count_endofphase2= 0;   // 時間経過監視カウンタ(phase2終了値)
    static unsigned int  run_count_endofphase3= 0;   // 時間経過監視カウンタ(phase3終了値)


    if(run_count == 1)
    {
        base_rev_l = ev3_motor_get_counts(left_motor);
        base_rev_r = ev3_motor_get_counts(right_motor);
//        base_rev_l = ecrobot_get_motor_rev(PORT_MOTOR_L);
//        base_rev_r = ecrobot_get_motor_rev(PORT_MOTOR_R);
    }

    if(phase <= 4)
    {
        rev_l = ev3_motor_get_counts(left_motor) - base_rev_l;
        rev_r = ev3_motor_get_counts(right_motor) - base_rev_r;
//        rev_l = ecrobot_get_motor_rev(PORT_MOTOR_L) - base_rev_l;
//        rev_r = ecrobot_get_motor_rev(PORT_MOTOR_R) - base_rev_r;
#if 0
        if (rev_l > rev_r) {
            pwm_r = pwm_l * PWM_ADD;
        } else if (rev_r > rev_l) {
            pwm_l = pwm_r * PWM_ADD;
        }
#endif
        pwm_r *= 1.058;
        //ecrobot_set_motor_speed(PORT_MOTOR_L, pwm_l);
        //ecrobot_set_motor_speed(PORT_MOTOR_R, pwm_r);
        ev3_motor_set_power(left_motor, (int)50);
        ev3_motor_set_power(right_motor, (int)50);
//        nxt_motor_set_speed(PORT_MOTOR_L, pwm_l,1);
//        nxt_motor_set_speed(PORT_MOTOR_R, pwm_r,1);
    }

    run_count++;
    switch (phase) {

    // 停止→後傾
    case 0:
        rev_t = LOOK_UP_GATE_PASSING_ANGLE -10;
//        rev_t = TAIL_ANGLE_STAND_UP -10;
        look_up_gate_tail_control(rev_t);
//        figureL_tail_control(rev_t);
        if(run_count < 400)
        {
            do_balance(0,0);
            return 0;
        }

        if(run_count < 500)
        {
            ev3_motor_set_power(left_motor, (int)50);
            ev3_motor_set_power(right_motor, (int)50);
//            ecrobot_set_motor_mode_speed(PORT_MOTOR_L, 1, 50);  /* 左モータPWM出力セット(-100～100) */
//            ecrobot_set_motor_mode_speed(PORT_MOTOR_R, 1, 50);  /* 右モータPWM出力セット(-100～100) */
            return 0;
        }
        run_count_endofphase0 = run_count;
        phase++;

    // 後傾→前進→停止
    case 1:
        break;

    // 後退
    case 2:
        break;

    // 前進→停止
    case 3:
        break;

    // 直立
    case 4:
        break;

    return 0;

    }
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

/* 新たに作成するため使用しない！*/
#if 0
void look_up_gate_main(void)
{
    /* ローカル変数の定義・初期化 */
	signed char forward = 0;      /* 前後進命令 */
	signed char turn    = 0;         /* 旋回命令 */
	signed char pwm_L, pwm_R; /* 左右モータPWM出力 */

	static int 				tail_status 			= 0;
	static unsigned int		is_balance_control_     = true;	/* 倒立振り子制御有無(true: 有効，false: 無効) */ 

	/* ★0で初期化すると振り子制御が上手くいかなくなる・・・ */
    int32_t motor_ang_l, motor_ang_r;
    int gyro, volt;

    /* 本関数初回時の処理 */
    if (look_up_gate_status_ == LOOK_UP_GATE_STAT_UNKNOWN) {

        /* ゲート検知を音で示す */
        ev3_speaker_set_volume(100); 
        ev3_speaker_play_tone(NOTE_C4, 100);

        /* 障害物を検知したので速度無し */
        forward = turn = 0;      
        /* 次の状態に遷移 */
        look_up_gate_status_ = LOOK_UP_GATE_STAT_PREPARE;        
    }

    /* 攻略状態を判定する */
    switch (look_up_gate_status_) {

    /* ゲート通過前処理状態 */
    case LOOK_UP_GATE_STAT_PREPARE:

//	        ev3_speaker_set_volume(100); 
//          ev3_speaker_play_tone(NOTE_D4, 100);  
        
            /* スタート時の倒立角度から15度に上げる */
            tail_angle_ = LOOK_UP_GATE_PASSING_ANGLE - 30;

			motor_stop = look_up_gate_tail_control(tail_angle_);
		if (1 == motor_stop
				&& 0 == tail_status) {
			tslp_tsk(10);

		    tail_status = 1;
		    motor_stop = 0;
		}

		if (1 == tail_status) {
            /* ゲート通過角度に設定 */
            tail_angle_ = LOOK_UP_GATE_PASSING_ANGLE - 20;

			motor_stop = look_up_gate_tail_control(tail_angle_);

			if (1 == motor_stop
					&& 1 == tail_status) { 
				tslp_tsk(10);
                /* 1秒停止 */
        		static int stop_cycle = 0;
                stop_cycle++;
                if (stop_cycle >= 250) {
                    /* 次の状態に遷移 */
                    tail_status = 2;
                	motor_stop = 0;
                }
            }
		}

		if (2 == tail_status) {
            /* ゲート通過角度に設定 */
            tail_angle_ = LOOK_UP_GATE_PASSING_ANGLE - 15;

			motor_stop = look_up_gate_tail_control(tail_angle_);

			if (1 == motor_stop
					&& 2 == tail_status) { 
				tslp_tsk(10);
                /* 1秒停止 */
        		static int stop_cycle = 0;
                stop_cycle++;
                if (stop_cycle >= 250) {
                    /* 次の状態に遷移 */
                    tail_status = 3;
                	motor_stop = 0;
                }
            }
		}

		if (3 == tail_status) {
            /* ゲート通過角度に設定 */
            tail_angle_ = LOOK_UP_GATE_PASSING_ANGLE - 10;

			motor_stop = look_up_gate_tail_control(tail_angle_);

			if (1 == motor_stop
					&& 3 == tail_status) { 
				tslp_tsk(10);
                /* 1秒停止 */
        		static int stop_cycle = 0;
                stop_cycle++;
                if (stop_cycle >= 250) {
                    /* 次の状態に遷移 */
                    tail_status = 4;
                	motor_stop = 0;
                }
            }
		}

        /* 以下の処理で重心を後方に傾けるようにする */
		if (4 == tail_status) {
            /* ゲート通過角度に設定 */
            tail_angle_ = LOOK_UP_GATE_PASSING_ANGLE - 5;
			motor_stop = look_up_gate_tail_control(tail_angle_);

		    static int balance_back = 0;
		    /* 初回 */
		    if (0 == balance_back) {
		    	/* 前に転倒する可能性があるため */
		    	/* 各角度が０の状態で重心を後ろにする */
		    	if (0 == gyro) {
			        ev3_speaker_set_volume(100); 
		            ev3_speaker_play_tone(NOTE_E4, 100);
			        /* 倒立振り子を無効化 */
					is_balance_control_ = false; 
			        balance_back = 1;
		    	}
		    }
			else if (1 == balance_back) {
		    	if (0 == gyro) {
			        ev3_speaker_set_volume(100); 
		            ev3_speaker_play_tone(NOTE_F4, 100);
			        /* 後ろに傾けるため、前進させる */
	                ev3_motor_set_power(left_motor, (int)5);
	                ev3_motor_set_power(right_motor, (int)5);
			        balance_back = 2;
		    	}
		    }
            else {
                ev3_motor_set_power(left_motor, (int)0);
                ev3_motor_set_power(right_motor, (int)0);
            }
        }

        break;

    /* ゲート通過処理状態 */
    case LOOK_UP_GATE_STAT_PROCESSING:
#if 0
        /* 距離を更新 */
        Distance_update();

        /* 前進 */
        if ((process_start_location_ + FORWARD_DISTANCE) > Distance_getDistance())
//            look_up_gate_gate_passing(0);

        /* 後進 */
        /* TODO 後進すると距離が減算されるか不明の為，未実装で */
        if ((process_start_location_ + FORWARD_DISTANCE) < Distance_getDistance())
//            look_up_gate_gate_passing(1);

        /* 停止 */
        if ((process_start_location_ + FORWARD_DISTANCE) == Distance_getDistance()) {
            look_up_gate_gate_passing(2);
            tslp_tsk(1000 * 10);
        }

        break;
#endif
    /* ゲート通過後処理状態 */
    case LOOK_UP_GATE_STAT_FINISH:
        /* T.B.D. */
        break;

    /* エラー状態(例外) */
    case LOOK_UP_GATE_STAT_ERROR:
    default:
        /* 通常のライントレース状態に戻す */
        /* T.B.D. */
        break;
    }


    /* 倒立振子制御API に渡すパラメータを取得する */
    motor_ang_l = ev3_motor_get_counts(left_motor);
    motor_ang_r = ev3_motor_get_counts(right_motor);
    gyro = ev3_gyro_sensor_get_rate(gyro_sensor);
    volt = ev3_battery_voltage_mV();

	log_Str(100, gyro, 0, 0, 0);

    /* 倒立振り子制御有無の判定 */
    if (true == is_balance_control_) {
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
#endif
//*****************************************************************************
// 関数名 : look_up_gate_get_distance
// 引数 : なし
// 返り値 : 障害物までの距離(cm)
// 概要 : スタート地点からの距離を測定
//        障害物までの距離を取得する
//*****************************************************************************
signed int look_up_gate_get_distance(void)
{
    signed int distance = -1;

    /* スタートからの距離測定をなくしてみる */
    /* 超音波センサから障害物までの距離を取得する */
    distance = ev3_ultrasonic_sensor_get_distance(sonar_sensor);
    
    return (distance);
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

//*****************************************************************************
// 関数名 : look_up_gate_sonar_distance
// 引数 : 無し
// 返り値 : 1(障害物あり)/0(障害物無し)
// 概要 : 超音波センサによる障害物検知
//*****************************************************************************
int look_up_gate_sonar_distance(void)
{
    static unsigned int counter = 0;
    static int alert = 0;

    signed int distance;

    if (++counter == 40/4) /* 約40msec周期毎に障害物検知  */
    {
        /*
         * 超音波センサによる距離測定周期は、超音波の減衰特性に依存します。
         * NXTの場合は、40msec周期程度が経験上の最短測定周期です。
         * EV3の場合は、要確認
         */
        distance = ev3_ultrasonic_sensor_get_distance(sonar_sensor);
        if ((distance <= LOOK_UP_GATE_DISTANCE) && (distance >= 0))
        {
            alert = 1; /* 障害物を検知 */
        }
        else
        {
            alert = 0; /* 障害物無し */
        }
        counter = 0;
    }

    return alert;
}

//*****************************************************************************
// 関数名 : look_up_gate_gate_passing
// 引数 : direction(0:前進、1:後進、2:停止)
// 返り値 : 無し
// 概要 : 倒立振子制御無効化状態でのモータ制御
//*****************************************************************************
void look_up_gate_gate_passing(unsigned int direction)
{
    /* 左右モータPWM出力 */
    signed char local_pwm_L = 0, local_pwm_R = 0;

    switch (direction) {
    /* 前進 */
    case 0:
        local_pwm_L = 10, local_pwm_R = 10; 
        ev3_motor_set_power(left_motor, (int)local_pwm_L);
        ev3_motor_set_power(right_motor, (int)local_pwm_R);
        break;

    /* 後進 */
    case 1:
        local_pwm_L = -10, local_pwm_R = -10;
        ev3_motor_set_power(left_motor, (int)local_pwm_L);
        ev3_motor_set_power(right_motor, (int)local_pwm_R);
        break;

    /* 停止 */
    case 2:
        ev3_motor_set_power(left_motor, (int)local_pwm_L);
        ev3_motor_set_power(right_motor, (int)local_pwm_R);
        break;

    /* 例外 */
    default:
        /* T.B.D. */
        break;
    }
}

//*****************************************************************************
// 関数名 : balance_task
// 引数 : unused
// 返り値 : なし
// 概要 : 異常検知タスク。
//        現状、転倒によるジャイロセンサが異常値の検出した際の処理を行う
//       
//*****************************************************************************
void balance_task(intptr_t unused)
{
    /* ローカル変数の定義・初期化 */
	signed char local_forward = 0;      /* 前後進命令 */
	signed char local_turn    = 0;         /* 旋回命令 */
	signed char local_pwm_L, local_pwm_R; /* 左右モータPWM出力 */
    int32_t motor_ang_l, motor_ang_r;
    int gyro, volt;

//   ev3_speaker_set_volume(100); 
//    ev3_speaker_play_tone(NOTE_E4, 100);

	while (1) {
        /* 倒立振子制御API に渡すパラメータを取得する */
        motor_ang_l = ev3_motor_get_counts(left_motor);
        motor_ang_r = ev3_motor_get_counts(right_motor);
        gyro = ev3_gyro_sensor_get_rate(gyro_sensor);
        volt = ev3_battery_voltage_mV();

        /* 倒立振子制御APIを呼び出し、倒立走行するための */
        /* 左右モータ出力値を得る */
        balance_control(
            (float)local_forward,
            (float)local_turn,
            (float)gyro,
            (float)GYRO_OFFSET,
            (float)motor_ang_l,
            (float)motor_ang_r,
            (float)volt,
            (signed char*)&local_pwm_L,
            (signed char*)&local_pwm_R);

        /* EV3ではモーター停止時のブレーキ設定が事前にできないため */
        /* 出力0時に、その都度設定する */
        if (local_pwm_L == 0)
        {
             ev3_motor_stop(left_motor, true);
        }
        else
        {
            ev3_motor_set_power(left_motor, (int)local_pwm_L);
        }
        
        if (local_pwm_R == 0)
        {
             ev3_motor_stop(right_motor, true);
        }
        else
        {
            ev3_motor_set_power(right_motor, (int)local_pwm_R);
        }

	    tslp_tsk(4);
	}
}

