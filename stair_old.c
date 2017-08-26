#include<float.h>

#include "ev3api.h"
#include "app.h"
#include "balancer.h"
#include "common.h"
#include "stair.h"
#include "module_cfg.h"
#include "Distance.h" 

signed char forward;      /* �O��i���� */
signed char turn;         /* ���񖽗� */
signed char pwm_L, pwm_R; /* ���E���[�^PWM�o�� */

#define LOG_MAX 200000
static int counter = 0;
static int gyro_str = 0;
static int gyro_log[LOG_MAX];
static float stair_distance_log[LOG_MAX];
static int stair_floor_status_log[LOG_MAX];

static int gyro_Ave = 0;

/* T.Mochizuki 20170726 */
static unsigned int gi_Stage; 	/* �K�i�̒i�ʃX�e�[�^�X */
static int gi_total = 0;
static int gi_Ave = 0;		/* ���ϒl���K��l�ȉ��̉� */
static int gi_AveOkCount = 0;
//static int gi_AveCount = 0;			/* ���ϒl���K��l�ȉ��̉񐔂��̑���iON/OFF�j*/
static float stair_distance = 0;
static int ONE_spin_timecounter = 0;
static int TWO_spin_timecounter = 0;


//static int gi_LineStatus = 0;		/* ���C����ɂ��邩���m */
static int STAGE_INIT = 0;         /* ��k�X�e�[�^�X�������Ăяo�� */
static int FLOOR_SEACH;             /* �t���A���m�X�e�[�^�X(ON/OFF) */
static int ONE_spin_status;				/* 1�K���̉�]�X�e�[�^�X */
static int TWO_spin_status;				/* 2�K���̉�]�X�e�[�^�X */

/* T.Mochizuki 20170805 */

/* �v���g�^�C�v�錾 */
void stair_A(void);
void stair_B(int gi_Stage);
void log_commit(void);
static float stair_search(int counter); /* �i�����m�֐�*/
void FLOOR_status(int gyro_Average);        /* �t���A���m */


/* T.Mochizuki 20170805 */
void Direction_init(void);
float Direction_getDirection(void);
void Dirction_update(void);


/* �t���A���m�X�e�[�^�X */
#define OFF 0
#define ON 1

//*****************************************************************************
// �֐��� : stair_main
// ���� : 
// �Ԃ�l : �Ȃ�
// �T�v : 
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
      log_commit(); /* �^�b�`�Z���T�������ꂽ */
    }
	

    /* T.Mochizuki 2017729*/
    gyro_Ave = stair_search( counter );
    
    ///* �W���C���p���x�ɂĊK�i�����m���� */
    //if((counter % 10) == 0)
    //{
    //    /* 40 ms���Ƃ�gyro_str�ɃW���C���p���x�Z���T�[�l����� (4 ms�����Ɖ���) */
    //    gyro_str = ev3_gyro_sensor_get_rate(gyro_sensor);
    // }
    
     /* 40 ms�O�̃Z���T�[�l�ƍ��̃Z���T�[�l�̍������K��l���傫���ꍇ�K�i�ɂԂ������Ɣ��f���� */
//    if((gyro_str - ev3_gyro_sensor_get_rate(gyro_sensor)) > 140 ||
//      (gyro_str - ev3_gyro_sensor_get_rate(gyro_sensor)) < (140 * (-1)) )
//    {
         /* �K�i���m�����Ŏ��� */
//        ev3_speaker_set_volume(10); 
//        ev3_speaker_play_tone(NOTE_C4, 100);
//        FLOOR_SEACH = ON;
//	} 
	
    gyro_log[counter] = ev3_gyro_sensor_get_rate(gyro_sensor);
    stair_distance_log[counter] = Distance_getDistance();
    stair_floor_status_log[counter] =gi_Stage;
    
	switch(gi_Stage){
	case 0:     
#if 1
	    stair_B(gi_Stage);
#else
		stair_A();
#endif
	    FLOOR_status(gyro_Ave);
		break;
		
	case 1:
		stair_B(gi_Stage);
	    FLOOR_status(gyro_Ave);
		break;
		
	case 2:
		stair_B(gi_Stage);
		break;
		
	default:
		break;
	}
    counter++;
    
    STAGE_INIT = 1;
}
 #if 1   
 void stair_B(int gi_Stage)
{
    int32_t motor_ang_l, motor_ang_r;
    int gyro, volt;

        tail_control(TAIL_ANGLE_DRIVE); /* �o�����X���s�p�p�x�ɐ��� */

    switch(gi_Stage)
    {
        case 1:
        stair_distance = 140.0;
        break;
        
        case 2:
        stair_distance = 135.0;
        break;
        
        default:
        break;
        
    }
    if (gi_Stage == 1 && Distance_getDistance() >= stair_distance ) /* ���m */
    {
            if(ONE_spin_status == 0)
            {
                forward = turn = 0; /* ��Q�������m�������~ */
                ev3_lcd_draw_string("dayoooon", 0, CALIB_FONT_HEIGHT*3);
            }
            else
            {
                forward = 60;   /* �O�i���� */
                turn = 0;
            }
    	ONE_spin_timecounter++;

    }
	/* T.Mochizuki 20170805 */
	else if(gi_Stage == 2 && Distance_getDistance() >= stair_distance )
	{
		if(TWO_spin_status == 0)
		{
			forward = turn = 0; /* ��Q�������m�������~ */
            ev3_speaker_set_volume(10); 
            ev3_speaker_play_tone(NOTE_FS4, 100);
            ev3_lcd_draw_string("dayoooon222", 0, CALIB_FONT_HEIGHT*5);

		}
		TWO_spin_timecounter++;
	}
//    else if (gi_Stage == 2 && Distance_getDistance() >= stair_distance ) {
//            forward = turn = 0; /* ��Q�������m�������~ */
//            ev3_speaker_set_volume(10); 
//            ev3_speaker_play_tone(NOTE_FS4, 100);
//            ev3_lcd_draw_string("dayoooon222", 0, CALIB_FONT_HEIGHT*5);
//    }
	
    else
    {
            forward = 30; /* �O�i���� */
            turn =  0; /* �����񖽗� */
    }

        /* �|���U�q����API �ɓn���p�����[�^���擾���� */
        motor_ang_l = ev3_motor_get_counts(left_motor);
        motor_ang_r = ev3_motor_get_counts(right_motor);
        gyro = ev3_gyro_sensor_get_rate(gyro_sensor);
        volt = ev3_battery_voltage_mV();

        /* �|���U�q����API���Ăяo���A�|�����s���邽�߂� */
        /* ���E���[�^�o�͒l�𓾂� */
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

        /* EV3�ł̓��[�^�[��~���̃u���[�L�ݒ肪���O�ɂł��Ȃ����� */
        /* �o��0���ɁA���̓s�x�ݒ肷�� */
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
       
    
	if(ONE_spin_timecounter >= 10000)
    {
        ONE_spin_status = 1;
        ev3_lcd_draw_string("ONE_spin_status OK!!", 0, CALIB_FONT_HEIGHT*4);
    }
	else if(TWO_spin_timecounter >= 10000){
		//TWO_spin_status = 1;
        ev3_lcd_draw_string("TWO_spin_status OK!!", 0, CALIB_FONT_HEIGHT*4);
	}
	
    Distance_update();
    
}

#else
void stair_A(void)
{
    int32_t motor_ang_l, motor_ang_r;
    int gyro, volt;

        tail_control(TAIL_ANGLE_DRIVE); /* �o�����X���s�p�p�x�ɐ��� */

        if (sonar_alert() == 1) /* ��Q�����m */
        {
            forward = turn = 0; /* ��Q�������m�������~ */
        }
        else
        {
            forward = 60; /* �O�i���� */
            turn =  0; /* �����񖽗� */
        }

        /* �|���U�q����API �ɓn���p�����[�^���擾���� */
        motor_ang_l = ev3_motor_get_counts(left_motor);
        motor_ang_r = ev3_motor_get_counts(right_motor);
        gyro = ev3_gyro_sensor_get_rate(gyro_sensor);
        volt = ev3_battery_voltage_mV();

        /* �|���U�q����API���Ăяo���A�|�����s���邽�߂� */
        /* ���E���[�^�o�͒l�𓾂� */
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

        /* EV3�ł̓��[�^�[��~���̃u���[�L�ݒ肪���O�ɂł��Ȃ����� */
        /* �o��0���ɁA���̓s�x�ݒ肷�� */
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
}
     
#endif

//*****************************************************************************
// �֐��� : stair_search
// ���� : 
// �Ԃ�l : float�^ �i���Ȃ��̃W���C�����ϒl
// �T�v : �i�����m�֐�
//       
//*****************************************************************************
static float stair_search(int counter)
{
    if((counter % 10) == 0)
    {
        /* 40 ms���Ƃ�gyro_str�ɃW���C���p���x�Z���T�[�l����� (4 ms�����Ɖ���) */
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
// �֐��� : FLOOR_status(float gyro_Average)
// ���� : 
// �Ԃ�l : �Ȃ�
// �T�v : �t���A���m�p�̊֐�
//       
//*****************************************************************************
void FLOOR_status(int gyro_Average)
{
    if((gyro_str - ev3_gyro_sensor_get_rate(gyro_sensor)) > 200 ||
      (gyro_str - ev3_gyro_sensor_get_rate(gyro_sensor)) < 200 * (-1))
    {
         /* �K�i���m�����Ŏ��� */
        ev3_speaker_set_volume(10); 
        ev3_speaker_play_tone(NOTE_C4, 100);
        FLOOR_SEACH = ON;
	}
	
	if( FLOOR_SEACH == ON){
		if(gyro_Average <= 200 || gyro_Average >= (200 * (-1)) )
		{
			gi_AveOkCount += 1;
		}
		if(gi_AveOkCount >= 100 )
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
}


void log_commit(void)
{
    FILE *fp; /* �t�@�C���|�C���^ */
	int  i;   /* �C���N�������g */

    /* Log�t�@�C���쐬 */
	fp=fopen("170826_Stair_Log_7.csv","a");
	/* ��^�C�g���}�� */
	fprintf(fp,"�W���C���Z���T�p���x, ���s����, �t���A���m�X�e�[�^�X�@\n");
	
	/* Log�̏o�� */
	for(i = 0 ; i < LOG_MAX; i++)
	{
		fprintf(fp,"%d,%f,%d\n", gyro_log[i], stair_distance_log[i], stair_floor_status_log[i]);
	}
	
	fclose(fp);
}

/* end of file */
