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


static int counter = 0;
static int gyro_str = 0;
static int gyro_log[3000];
static int status = 0;

/* �v���g�^�C�v�錾 */
void stair_A(void);
void log_commit(void);

//*****************************************************************************
// �֐��� : stair_main
// ���� : 
// �Ԃ�l : �Ȃ�
// �T�v : 
//       
//*****************************************************************************
void stair_main()
{
    if (ev3_touch_sensor_is_pressed(touch_sensor) == 1)
    {
      log_commit(); /* �^�b�`�Z���T�������ꂽ */
    }
	
    /* �W���C���p���x�ɂĊK�i�����m���� */
    if((counter % 10) == 0)
    {
        /* 40 ms���Ƃ�gyro_str�ɃW���C���p���x�Z���T�[�l����� (4 ms�����Ɖ���) */
        gyro_str = ev3_gyro_sensor_get_rate(gyro_sensor);
     }
    
     /* 40 ms�O�̃Z���T�[�l�ƍ��̃Z���T�[�l�̍������K��l���傫���ꍇ�K�i�ɂԂ������Ɣ��f���� */
    if((gyro_str - ev3_gyro_sensor_get_rate(gyro_sensor)) > 250 ||
      (gyro_str - ev3_gyro_sensor_get_rate(gyro_sensor)) < (250 * (-1)) )
    {
         /* �K�i���m�����Ŏ��� */
       ev3_speaker_set_volume(100); 
        ev3_speaker_play_tone(NOTE_C4, 100);
	} 
	

	status = 0;
    gyro_log[counter] = ev3_gyro_sensor_get_rate(gyro_sensor);
    
    switch(status){
       case 0:
         stair_A();
         break;
           
       default:
         break;
    }

    counter++;
    
}


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
        

}

void log_commit(void)
{
    FILE *fp; /* �t�@�C���|�C���^ */
	int  i;   /* �C���N�������g */

    /* Log�t�@�C���쐬 */
	fp=fopen("170708_Stair_Log.csv","a");
	/* ��^�C�g���}�� */
	fprintf(fp,"�W���C���Z���T�p���x�@\n");
	
	/* Log�̏o�� */
	for(i = 0 ; i < 3000; i++)
	{
		fprintf(fp,"%d\n", gyro_log[i]);
	}
	
	fclose(fp);
}

/* end of file */
