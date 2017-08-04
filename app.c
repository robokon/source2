/**
 ******************************************************************************
 ** ファイル名 : app.c
 **
 ** 概要 : 2輪倒立振子ライントレースロボットのTOPPERS/HRP2用Cサンプルプログラム
 **
 ** 注記 : sample_c4 (sample_c3にBluetooth通信リモートスタート機能を追加)
 ******************************************************************************
 **/

#include "ev3api.h"
#include "app.h"
#include "balancer.h"
#include "common.h"
#include "line_trace.h"
#include "Distance.h"
#include "stair.h"
#include "look_up_gate.h"
#include "log.h"

#if defined(BUILD_MODULE)
#include "module_cfg.h"
#else
#include "kernel_cfg.h"
#endif

#define DEBUG

#ifdef DEBUG
#define _debug(x) (x)
#else
#define _debug(x)
#endif

int bt_cmd = 0;     /* Bluetoothコマンド 1:リモートスタート */
FILE *bt = NULL;     /* Bluetoothファイルハンドル */
int LIGHT_WHITE=0;         /* 白色の光センサ値 */
int LIGHT_BLACK=100;       /* 黒色の光センサ値 */
int mode_flg = 0;          /* モード変更のフラグ */

/* 各難所制御状態 */
STATUS main_status = STAT_UNKNOWN;

/* ルックアップゲートモード移行距離(cm) */
#define START_TO_LOOKUP 5


/* メインタスク */
void main_task(intptr_t unused)
{

    /* LCD画面表示 */
    ev3_lcd_fill_rect(0, 0, EV3_LCD_WIDTH, EV3_LCD_HEIGHT, EV3_LCD_WHITE);
    ev3_lcd_draw_string("EV3way-ET sample_c4", 0, CALIB_FONT_HEIGHT*1);

    /* センサー入力ポートの設定 */
    ev3_sensor_config(sonar_sensor, ULTRASONIC_SENSOR);
    ev3_sensor_config(color_sensor, COLOR_SENSOR);
    ev3_color_sensor_get_reflect(color_sensor); /* 反射率モード */
    ev3_sensor_config(touch_sensor, TOUCH_SENSOR);
    ev3_sensor_config(gyro_sensor, GYRO_SENSOR);
    /* モーター出力ポートの設定 */
    ev3_motor_config(left_motor, LARGE_MOTOR);
    ev3_motor_config(right_motor, LARGE_MOTOR);
    ev3_motor_config(tail_motor, LARGE_MOTOR);
    ev3_motor_reset_counts(tail_motor);

    /* Open Bluetooth file */
    bt = ev3_serial_open_file(EV3_SERIAL_BT);
    assert(bt != NULL);

    /* Bluetooth通信タスクの起動 */
    act_tsk(BT_TASK);
	Distance_init(); /* 距離計測変数初期化 */
	
    /* 尻尾の位置を初期値 */
    while(1)
    {
        tail_control(-100);
        if (ev3_touch_sensor_is_pressed(touch_sensor) == 1)
        {
            break;/* タッチセンサが押された */
        }
    }
    ev3_motor_reset_counts(tail_motor);
    tslp_tsk(1000); /* 1000msecウェイト */
    
    ev3_led_set_color(LED_ORANGE); /* 初期化完了通知 */

    int cal_mode = 0;
    while(1)
    {
        tail_control(TAIL_ANGLE_STAND_UP); /* 完全停止用角度に制御 */
        switch(cal_mode)
        {
        case 0:
            /*白色の光センサ値取得*/
            if (ev3_touch_sensor_is_pressed(touch_sensor) == 1)
            {
                LIGHT_WHITE = ev3_color_sensor_get_reflect(color_sensor);
                log_Str(LIGHT_WHITE,0,0,0,0);
                cal_mode = 1; /* タッチセンサが押された */
                tslp_tsk(300); /* 300msecウェイト */
            }
            break;
        case 1:
            /*黒色の光センサ値*/
            if (ev3_touch_sensor_is_pressed(touch_sensor) == 1)
            {
                LIGHT_BLACK = ev3_color_sensor_get_reflect(color_sensor);
                log_Str(LIGHT_BLACK,0,0,0,0);
                cal_mode = 2; /* タッチセンサが押された */
                tslp_tsk(300); /* 1000msecウェイト */
            }
            break; 
        case 2:
            /* スタート待機 */
            if (bt_cmd == 1)
            {
                ev3_speaker_play_tone(NOTE_C4, 100);
                cal_mode = 3; /* リモートスタート */
            }
            else
            if(bt_cmd == 2)
            {
                ev3_speaker_play_tone(NOTE_G4, 50);
                cal_mode = 3; /* リモートスタート */
            }

            if (ev3_touch_sensor_is_pressed(touch_sensor) == 1)
            {
                cal_mode = 3; /* タッチセンサが押された */
            }
            break;
        }
        if(cal_mode == 3)
        {
            break; /* 白と黒の値を設定したら処理を抜ける */
        }
    }

    /* 走行モーターエンコーダーリセット */
    ev3_motor_reset_counts(left_motor);
    ev3_motor_reset_counts(right_motor);

    /* ジャイロセンサーリセット */
    ev3_gyro_sensor_reset(gyro_sensor);
    balance_init(); /* 倒立振子API初期化 */

    ev3_led_set_color(LED_GREEN); /* スタート通知 */
    
    /* スタート通知後、通常のライントレースに移行するように設定 */
    main_status = STAT_NORMAL; 

    /* 超音波センサタスクの起動 */
    act_tsk(SONAR_TASK);
    
    /*スタート処理*/
    while(1)
    {
        float tail = 0;
        tail = tail_control(TAIL_ANGLE_START);
        if(tail==0)
        {
            break;
        }
    }
    
    /**
    * Main loop for the self-balance control algorithm
    */
    // 周期ハンドラ開始
    ev3_sta_cyc(MAIN_CYC1);
    //ev3_sta_cyc(TEST_EV3_CYC2);

    // バックボタンが押されるまで待つ
    slp_tsk();
    // 周期ハンドラ停止
    ev3_stp_cyc(MAIN_CYC1); 
    

    ev3_motor_stop(left_motor, false);
    ev3_motor_stop(right_motor, false);

    log_Commit();
    ter_tsk(BT_TASK);
    fclose(bt);

    ext_tsk();
}

//*****************************************************************************
// 関数名 : main_cyc1
// 引数 : 無し
// 返り値 :
// 概要 : メイン周期タスク
//*****************************************************************************
void main_cyc1(intptr_t idx) 
{
    /* とりあえず，処理なし */
    /* sonar_taskで行う */
#if 0
        /* ルックアップゲート からの距離を測定 */
        if (Distance_getDistance() >= START_TO_LOOKUP) {
            main_status = STAT_LOOK_UP_GATE;
        }
#endif
    
    switch (main_status) {
        /* 通常制御中 */
        case STAT_NORMAL:
            /* 通常のライントレース制御 */
            line_tarce_main(LIGHT_WHITE + LIGHT_BLACK);
            break;

        /* 階段制御中 */
        case STAT_STAIR:
            stair_main();
            break;

        /* ルックアップゲート制御中 */
        case STAT_LOOK_UP_GATE:
            look_up_gate_main();
            break;

        /* ガレージ制御中 */
        case STAT_GAREGE:
            /* T.B.D */
            break;

        /* その他 */
        default:
            /* T.B.D */
            break;
    }

    Distance_update(); /* 移動距離加算 */
    
    if( mode_flg == 0 )
    {
        /* Lコースモードの場合 */
        if( bt_cmd == 1 )
        {
            if( Distance_getDistance() > L_GOAL_DISTANCE )
            {
                /* DISTANCE_NOTIFY以上進んだら音を出す */
                ev3_speaker_set_volume(100); 
                ev3_speaker_play_tone(NOTE_C4, 100);
                
                /* 距離計測変数初期化 */
                Distance_init();
                
                /* 階段モードへ切り替え */
                main_status = STAT_STAIR;
                mode_flg = 1;
            } 
        }
        /* Rコースモードの場合 */
        else
        if( bt_cmd == 2 )
        {
            if( Distance_getDistance() > R_GOAL_DISTANCE )
            {
                /* DISTANCE_NOTIFY以上進んだら音を出す */
                ev3_speaker_set_volume(100); 
                ev3_speaker_play_tone(NOTE_G4, 50);
                
                /* 距離計測変数初期化 */
                Distance_init();
                
                /* ルックアップゲートモードへ切り替え */
                main_status = STAT_LOOK_UP_GATE;
                mode_flg = 1;
            }
        }
    }
}

//*****************************************************************************
// 関数名 : sonar_alert
// 引数 : 無し
// 返り値 : 1(障害物あり)/0(障害物無し)
// 概要 : 超音波センサによる障害物検知
//*****************************************************************************
int sonar_alert(void)
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
// 関数名 : tail_control
// 引数 : angle (モータ目標角度[度])
// 返り値 : 無し
// 概要 : 走行体完全停止用モータの角度制御
//*****************************************************************************
float tail_control(signed int angle)
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
    return pwm;
}

//*****************************************************************************
// 関数名 : bt_task
// 引数 : unused
// 返り値 : なし
// 概要 : Bluetooth通信によるリモートスタート。 Tera Termなどのターミナルソフトから、
//       ASCIIコードで1を送信すると、リモートスタートする。
//*****************************************************************************
void bt_task(intptr_t unused)
{
    while(1)
    {
        uint8_t c = fgetc(bt); /* 受信 */
        switch(c)
        {
        case 'l':
            bt_cmd = 1;
            break;  
        case 'r':
            bt_cmd = 2;
            break;
        default:
            break;
        }
        fputc(c, bt); /* エコーバック */
    }
}

//*****************************************************************************
// 関数名 : sonar_task
// 引数 : unused
// 返り値 : なし
// 概要 : ルックアップゲートを検知する為、超音波センサの値をタスク処理で取得する
//       
//*****************************************************************************
void sonar_task(intptr_t unused)
{
    int         alert       = 0;
    signed int  distance    = -1;

    while (1) {
        /* 障害物検知を行う */
        alert = sonar_alert();
        /* 検知した */
        if (1 == alert) {
            /* 障害物までの距離を計測，減速開始距離である */
            if (LOOK_UP_GATE_BRAKE_DISTANCE == look_up_gate_get_distance()) {
                /* 障害物を検知したらすぐに停止するのではなく */
                /* 徐々に減速し、ゲート5㎝手前で停止させるようにする */
                /* 減速開始は、検知した障害物までの距離が15㎝手前から(妥当か微妙) */
                /* 本番のコースはポールが設置してある為、誤検知等も考慮する */
                
                /* TODO */

                /* 障害物までの距離を計測，ゲート攻略開始距離である */
                if (LOOK_UP_GATE_DISTANCE == look_up_gate_get_distance()) {
                    /* ルックアップゲート攻略状態 */
                    main_status = STAT_LOOK_UP_GATE;

                    /* 走行体を停止 */
                    ev3_motor_stop(left_motor, true);
                    ev3_motor_stop(right_motor, true);
                }
            }
        }
        tslp_tsk(4); /* 4msec周期起動 */
    }
}

/* end of file */
