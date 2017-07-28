/*
 * look_up_gate.c
 */

#include "look_up_gate.h"
#include "Distance.h"

#define LOOK_UP_GATE_STAND_UP       TAIL_ANGLE_STAND_UP       /* ルックアップゲート通過時の角度 */
#define LOOK_UP_GATE_PASSING_ANGLE  TAIL_ANGLE_STAND_UP - 5   /* ルックアップゲート通過時の角度 */
#define SLEEP_TIME                  1000 /* 暫定であるスリープ制御の時間 */
#define FORWARD_DISTANCE            30   /* 前進距離 */

static LOOK_UP_GATE_STATUS  look_up_gate_status_    = LOOK_UP_GATE_STAT_UNKNOWN;    /* ルックアップゲートの攻略状態 */
static int                  tail_angle_             = 0;                            /* 角度 */
static float                process_start_location_ = 0;                            /* ゲート通過開始位置 */
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
    /* ローカル変数の定義・初期化 */
	signed char forward = 0;      /* 前後進命令 */
	signed char turn    = 0;         /* 旋回命令 */
	signed char pwm_L, pwm_R; /* 左右モータPWM出力 */

	static int 				tail_status 			= TAIL_STAT_UNKNOWN;
	static unsigned int		is_balance_control_     = true;	/* 倒立振り子制御有無(true: 有効，false: 無効) */ 

	/* ★0で初期化すると振り子制御が上手くいかなくなる・・・ */
    int32_t motor_ang_l, motor_ang_r;
    int gyro, volt;

    /* 本関数初回時の処理 */
    if (look_up_gate_status_ == LOOK_UP_GATE_STAT_UNKNOWN) {
        /* ゲート検知を音で示す */
//        ev3_speaker_set_volume(100); 
//        ev3_speaker_play_tone(NOTE_C4, 100);

        /* 障害物を検知したので速度無し */
        forward = turn = 0;

        /* 攻略開始に遷移 */
        look_up_gate_status_ = LOOK_UP_GATE_STAT_PREPARE;
    }

    /* 攻略状態を判定する */
    switch (look_up_gate_status_) {

    /* ゲート通過前処理状態 */
    case LOOK_UP_GATE_STAT_PREPARE:
#if 1
            tail_angle_ = LOOK_UP_GATE_PASSING_ANGLE - 25;

            motor_stop = look_up_gate_tail_control(tail_angle_);

            /* テイルモータ停止、かつ尻尾が初期(ライントレース時)状態 */
			if ((1 == motor_stop)
			        && (TAIL_STAT_UNKNOWN == tail_status)) {
				tslp_tsk(10);

		        ev3_speaker_set_volume(100); 
	            ev3_speaker_play_tone(NOTE_D4, 100);
			    tail_status = TAIL_STAT_GATE_PASS_1;
			    motor_stop = 0;
			}

			if (TAIL_STAT_GATE_PASS_1 == tail_status) {
	            /* ゲート通過角度に設定 */
	            tail_angle_ = LOOK_UP_GATE_PASSING_ANGLE - 15;

				motor_stop = look_up_gate_tail_control(tail_angle_);

				if (1 == motor_stop
						&& TAIL_STAT_GATE_PASS_1 == tail_status) { 
					tslp_tsk(10);
				    tail_status = TAIL_STAT_GATE_PASS_2;
				}
			}

			if (TAIL_STAT_GATE_PASS_2 == tail_status) {
	            /* ゲート通過角度に設定 */
	            tail_angle_ = LOOK_UP_GATE_PASSING_ANGLE;

				motor_stop = look_up_gate_tail_control(tail_angle_);
				is_balance_control_ = false;
			}
//        }
#endif
#if 0
        /* 完全停止角度になった */
        else {
            /* 倒立振り子制御が有効 */
            if (is_balance_control_) {
                /* ★4msec周期起動なのでモータ制御が早すぎて上手くいかないと思うので，暫定で1000msのスリープを入れ緩やかにする */
                tslp_tsk(SLEEP_TIME);

		        ev3_speaker_set_volume(100); 
	            ev3_speaker_play_tone(NOTE_E4, 100);
                /* ゲート通過角度に設定 */
                tail_angle_ = LOOK_UP_GATE_PASSING_ANGLE;

                /* 目的の角度にモータ制御されるまでループ */
	            act_tsk(BALANCE_TASK);

                while (1 != motor_stop) {
                    motor_stop = look_up_gate_tail_control(tail_angle_);
                    tslp_tsk(500);
                }
	            ter_tsk(BALANCE_TASK);
                /* 倒立振り子制御は無効にする */
//                is_balance_control_ = false;
            }
            else {
                /* 機体がゲート通過可能な角度か判定 */
                if (LOOK_UP_GATE_PASSING_ANGLE == tail_angle_) {
                    /* ゲート通過中に遷移を音で示す */
                    ev3_speaker_set_volume(100); 
                    ev3_speaker_play_tone(NOTE_F4, 100);

                    /* ゲート通過可能な角度である為，状態をゲート通過中に移行する */
                    look_up_gate_status_ = LOOK_UP_GATE_STAT_PROCESSING;

                    /* 距離を更新 */
                    Distance_update();

                    /* ゲート通過処理開始時の位置を取得する */
                    process_start_location_ = Distance_getDistance();
                }
            }
        }
#endif

        break;

    /* ゲート通過処理状態 */
    case LOOK_UP_GATE_STAT_PROCESSING:
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
    /* 倒立振り子制御有無の判定 */
    if (true == is_balance_control_) {

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

}

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

