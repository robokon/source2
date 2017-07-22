/*
 * look_up_gate.c
 */

#include "look_up_gate.h"

#define LOOK_UP_GATE_PASSING_ANGLE  115  /* ルックアップゲート通過時の角度 */
#define SLEEP_TIME                  1000 /* 暫定であるスリープ制御の時間 */
#define FORWARD_DISTANCE            30   /* 前進距離 */
#define BACKWARD_DISTANCE           -30  /* 後進距離 */
#define START_DISTANCE              200

signed char forward;      /* 前後進命令 */
signed char turn;         /* 旋回命令 */
signed char pwm_L, pwm_R; /* 左右モータPWM出力 */

static LOOK_UP_GATE_STATUS  look_up_gate_status_    = LOOK_UP_GATE_STAT_UNKNOWN;    /* ルックアップゲートの攻略状態 */
static unsigned int         is_balance_control_     = true;                         /* 倒立振り子制御有無(true: 有効，false: 無効) */ 
static int                  tail_angle_             = 0;                            /* 角度 */
static float                process_start_location_ = 0;                            /* ゲート通過開始位置 */

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
    int32_t motor_ang_l, motor_ang_r    = 0;
    int gyro, volt                      = 0;

    /* 障害物を検知した */
    if (look_up_gate_sonar_distance()) {
        /* 攻略開始に遷移 */
        look_up_gate_status_ = LOOK_UP_GATE_STAT_PREPARE;
    }
    /* 障害物を検知しない場合，処理を終了する */
    else {
        return;
    }

    /* 攻略状態を判定する */
    switch (look_up_gate_status_) {

    /* ゲート通過前処理状態 */
    case LOOK_UP_GATE_STAT_PREPARE:
        /* ルックアップゲートを攻略するため，完全停止用角度にモータ制御 */
        /* 完全停止角度になるまでモータを制御する */
        if (TAIL_ANGLE_STAND_UP >= tail_angle_) {
            /* ★4msec周期起動なのでモータ制御が早すぎて上手くいかないと思うので，暫定で200msのスリープを入れ緩やかにする */
            tslp_tsk(SLEEP_TIME);
            look_up_gate_tail_control(tail_angle_++);
        }
        /* 完全停止角度になった */
        else {
            /* 倒立振り子制御が有効 */
            if (is_balance_control_) {
                tail_angle_ = TAIL_ANGLE_STAND_UP;
                /* 倒立振り子制御は無効にする */
                is_balance_control_ = false;
            }
            else {
                /* 機体がゲート通過可能な角度か判定 */
                if (LOOK_UP_GATE_PASSING_ANGLE >= tail_angle_) {
                    /* ★4msec周期起動なのでモータ制御が早すぎて上手くいかないと思うので，暫定で200msのスリープを入れ緩やかにする */
                    tslp_tsk(SLEEP_TIME);
                    look_up_gate_tail_control(tail_angle_++);
                }
                else {
                    /* ゲート通過可能な角度である為，状態をゲート通過中に移行する */
                    look_up_gate_status_ = LOOK_UP_GATE_STAT_PROCESSING;

                    /* ゲート通過処理開始時の位置を取得する */
                    /* 一定距離前進したら、後進させ、取得した位置まで戻る為 */
                    process_start_location_ = Distance_getDistance();
                }
            }
        }

        break;

    /* ゲート通過処理状態 */
    case LOOK_UP_GATE_STAT_PROCESSING:
        /* 前進 */
        if ((process_start_location_ + FORWARD_DISTANCE) > Distance_getDistance())
            look_up_gate_gate_passing(1);
        /* 後進 */
        /* TODO 後進すると距離が減算されるか不明の為，未実装で */
//        if ((process_start_location_ + FORWARD_DISTANCE) < Distance_getDistance())
            
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
    if (is_balance_control_) {
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
    
    /* 走行距離更新 */
    Distance_update();
    /* スタート地点からの距離が2m以上 */
    if (Distance_getDistance() >= START_DISTANCE) {
        /* 超音波センサから障害物までの距離を取得する */
        distance = ev3_ultrasonic_sensor_get_distance(sonar_sensor);
    }
    
    return (distance);
}

//*****************************************************************************
// 関数名 : look_up_gate_tail_control
// 引数 : angle (モータ目標角度[度])
// 返り値 : 無し
// 概要 : 走行体完全停止用モータの角度制御
//*****************************************************************************
void look_up_gate_tail_control(signed int angle)
{
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
    }
    else
    {
        ev3_motor_set_power(tail_motor, (signed char)pwm);
    }
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
        if ((distance <= SONAR_ALERT_DISTANCE) && (distance >= 0))
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
// 引数 : 無し
// 返り値 : 無し
// 概要 : 
//*****************************************************************************
void look_up_gate_gate_passing(unsigned int direction)
{
    signed char local_pwm_L = 30, local_pwm_R = 30; /* 左右モータPWM出力 */

    switch (direction) {
    /* 前進 */
    case 0:
        ev3_motor_set_power(left_motor, (int)local_pwm_L);
        ev3_motor_set_power(right_motor, (int)local_pwm_R);

        break;
    /* 後進 */
    case 1:
        /* T.B.D. */
    default:
        /* T.B.D. */
        break;
    }
}

/* end of file */
