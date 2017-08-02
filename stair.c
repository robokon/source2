#include<float.h>

#include "ev3api.h"
#include "app.h"
#include "balancer.h"
#include "common.h"
#include "stair.h"
#include "module_cfg.h"
#include "Distance.h" 

signed char forward;      /* 前後進命令 */
signed char turn;         /* 旋回命令 */
signed char pwm_L, pwm_R; /* 左右モータPWM出力 */

#define LOG_MAX 9000
static int counter = 0;
static int gyro_str = 0;
static int gyro_log[LOG_MAX];
static int stair_distance_log[LOG_MAX];
static int stair_floor_status_log[LOG_MAX];

static int gyro_Ave = 0;

/* T.Mochizuki 20170726 */
static unsigned int gi_Stage; 	/* 階段の段位ステータス */
static int gi_total = 0;
static int gi_Ave = 0;		/* 平均値が規定値以下の回数 */
static int gi_AveOkCount = 0;
static int gi_AveCount = 0;			/* 平均値が規定値以下の回数かの測定（ON/OFF）*/
static int stair_distance = 0;
static int ONE_spin_timecounter = 0;
//static int gi_LineStatus = 0;		/* ライン上にいるか検知 */
static int STAGE_INIT = 0;         /* 会談ステータス初期化呼び出し */
static int FLOOR_SEACH;             /* フロア検知ステータス(ON/OFF) */
static int ONE_spin_status;				/* 1階時の回転ステータス */
static int TWO_spin_status;				/* 2階時の回転ステータス */

/* プロトタイプ宣言 */
void stair_A(int gi_Stage);
void log_commit(void);
static float stair_search(int counter); /* 段差検知関数*/
void FLOOR_status(int gyro_Average);        /* フロア検知 */

/* フロア検知ステータス */
#define OFF 0
#define ON 1

//*****************************************************************************
// 関数名 : stair_main
// 引数 : 
// 返り値 : なし
// 概要 : 
//       
//*****************************************************************************
void stair_main()
{
    if( STAGE_INIT == 0)
    {
        gi_Stage = 0;
        ONE_spin_status = 0;
        TWO_spin_status = 0;
        FLOOR_SEACH = OFF;
    }
    
    if (ev3_touch_sensor_is_pressed(touch_sensor) == 1)
    {
      log_commit(); /* タッチセンサが押された */
    }
	

    /* T.Mochizuki 2017729*/
    gyro_Ave = stair_search( counter );
    
    ///* ジャイロ角速度にて階段を検知する */
    //if((counter % 10) == 0)
    //{
    //    /* 40 msごとにgyro_strにジャイロ角速度センサー値を入手 (4 ms周期と仮定) */
    //    gyro_str = ev3_gyro_sensor_get_rate(gyro_sensor);
    // }
    
     /* 40 ms前のセンサー値と今のセンサー値の差分が規定値より大きい場合階段にぶつかったと判断する */
    if((gyro_str - ev3_gyro_sensor_get_rate(gyro_sensor)) > 140 ||
      (gyro_str - ev3_gyro_sensor_get_rate(gyro_sensor)) < (140 * (-1)) )
    {
         /* 階段検知を音で示す */
        ev3_speaker_set_volume(2); 
        ev3_speaker_play_tone(NOTE_C4, 100);
        FLOOR_SEACH = ON;
	} 
	
    gyro_log[counter] = ev3_gyro_sensor_get_rate(gyro_sensor);
    stair_distance_log[counter] = Distance_getDistance();
    stair_floor_status_log[counter] =gi_Stage;
    
    if( FLOOR_SEACH == ON )
    {
        FLOOR_status(gyro_Ave);
    }
    
    stair_A(gi_Stage);
    
    counter++;
    
    STAGE_INIT = 1;
}


void stair_A(int gi_Stage)
{
    int32_t motor_ang_l, motor_ang_r;
    int gyro, volt;

        tail_control(TAIL_ANGLE_DRIVE); /* バランス走行用角度に制御 */

    switch(gi_Stage)
    {
        case 1:
        stair_distance = 147;
        break;
        
        case 2:
        stair_distance = 143;
        break;
        
        default:
        break;
        
    }
    if (gi_Stage == 1 && Distance_getDistance() >= stair_distance ) /* 検知 */
    {
            if(ONE_spin_status == 0)
            {
                forward = turn = 0; /* 障害物を検知したら停止 */
                ev3_speaker_set_volume(1); 
                ev3_lcd_draw_string("dayoooon", 0, CALIB_FONT_HEIGHT*3);
            }
            else
            {
                forward = 50;   /* 前進命令 */
                turn = 0;
            }

    }
    else if (gi_Stage == 2 && Distance_getDistance() >= stair_distance ) {
            forward = turn = 0; /* 障害物を検知したら停止 */
            ev3_speaker_set_volume(1); 
            ev3_speaker_play_tone(NOTE_FS4, 100);
            ev3_lcd_draw_string("dayoooon222", 0, CALIB_FONT_HEIGHT*5);
    }
    else
    {
            forward = 60; /* 前進命令 */
            turn =  0; /* 左旋回命令 */
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
       
    
    Distance_update();
    ONE_spin_timecounter++;
    
    if(ONE_spin_timecounter >= 10000)
    {
        ONE_spin_status = 1;
        ev3_lcd_draw_string("ONE_spin_status OK!!", 0, CALIB_FONT_HEIGHT*4);
    }

}


//*****************************************************************************
// 関数名 : stair_search
// 引数 : 
// 返り値 : float型 段差なしのジャイロ平均値
// 概要 : 段差検知関数
//       
//*****************************************************************************
static float stair_search(int counter)
{
    if((counter % 10) == 0)
    {
        /* 40 msごとにgyro_strにジャイロ角速度センサー値を入手 (4 ms周期と仮定) */
        gyro_str = ev3_gyro_sensor_get_rate(gyro_sensor);
        gi_total += gyro_str;
        
        if((gi_total % 10) == 0)
        {
            gi_Ave = (float)gi_total / 10;
            ev3_speaker_set_volume(1); 
            ev3_lcd_draw_string("stair_seach OK", 0, CALIB_FONT_HEIGHT*1);

        }
    }
    
    return gi_Ave;
}
//*****************************************************************************
// 関数名 : FLOOR_status(float gyro_Average)
// 引数 : 
// 返り値 : なし
// 概要 : フロア検知用の関数
//       
//*****************************************************************************
void FLOOR_status(int gyro_Average)
{
    if(gyro_Average <= 140 || gyro_Average >= (140 * (-1)) )
    {
        gi_AveOkCount += 1;
    }
    
    if(gi_AveOkCount >= 15 )
    {
        gi_Stage += 1;
        FLOOR_SEACH = OFF;
        
        Distance_init();
        ev3_speaker_set_volume(30); 
        ev3_speaker_play_tone(NOTE_B6, 100);
        ev3_lcd_draw_string("FLOOR_status OK", 0, CALIB_FONT_HEIGHT*2);
        gi_AveOkCount = 0;
    }
}


void log_commit(void)
{
    FILE *fp; /* ファイルポインタ */
	int  i;   /* インクリメント */

    /* Logファイル作成 */
	fp=fopen("170729_Stair_Log.csv","a");
	/* 列タイトル挿入 */
	fprintf(fp,"ジャイロセンサ角速度, 走行距離, フロア検知ステータス \n");
	
	/* Logの出力 */
	for(i = 0 ; i < LOG_MAX; i++)
	{
		fprintf(fp,"%d,%lf,%d\n", gyro_log[i], (float)stair_distance_log[i], stair_floor_status_log[i]);
	}
	
	fclose(fp);
}

/* end of file */
