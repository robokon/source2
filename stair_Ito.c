#include<float.h>

#include "ev3api.h"
#include "app.h"
#include "balancer.h"
#include "common.h"
#include "stair.h"
#include "module_cfg.h"
#include "Distance.h" 

signed char forward;            /* �O��i���� */
signed char turn;               /* ���񖽗� */
signed char pwm_L, pwm_R;       /* ���E���[�^PWM�o�� */

int Floor_Search_Flag;          /* �t���A���m�t���O  */
#define FLAG_OFF          0     /* �t���A���m�I�t */
#define FLAG_ON           1     /* �t���A���m�I�� */

int Floor_Status;               /* �t���A�X�e�[�^�X */
#define STAGE_ZERO        0     /* �K�i 0 �K */
#define STAGE_ONE         1     /* �K�i 1 �K */
#define STAGE_ONE         2     /* �K�i 2 �K */

int Run_Mode;                   /* ���s���[�h */
#define RUN_MODE_STAY     0     /* ���s���[�h�i��~�j*/
#define RUN_MODE_RUN      1     /* ���s���[�h�i���s���j*/
#define RUN_MODE_SPIN     2     /* ���s���[�h�i��]���j*/

int Stage_One_Spin_Status;      /* 1�K�̉�]�X�e�[�^�X */
int Stage_Tow_Spin_Status;      /* 2�K�̉�]�X�e�[�^�X */
#define SPIN_FINISH       0     /* ��]���� */
#define SPIN_NOT_FINISH   1     /* ��]������ */

int Stair_Count = 0;            /* �K�i�̊֐��ɓ������� */
int Gyro_Store[100];            /* �W���C���Z���T�[�l�̕��ϒl�̊i�[�� */


//*****************************************************************************
// �֐��� : stair_main
// ���� : 
// �Ԃ�l : �Ȃ�
// �T�v : 
//       
//*****************************************************************************
void stair_main()
{
    int gyro_Ave_Arg; /* �W���C�����ϒl�̊i�[����Ƃ��̈��� */
    
    if(Stair_Count == 0)
    {
        /* �e�X�e�[�^�X�̏����� */
        Floor_Status = STAGE_ZERO;
        Run_Mode = RUN_MODE_RUN;
        Stage_One_Spin_Status = SPIN_NOT_FINISH;
        Stage_Tow_Spin_Status = SPIN_NOT_FINISH;
        Floor_Search_Flag = FLAG_OFF;
    }

    
    /* �������牺�A�Ԉ���Ă� �����̗[�����璼�� */
    /* �W���C���p���x�ɂĊK�i�����m���� */ 
   if((Stair_Count % 10) == 0) 
   { 
        gyro_Ave_Arg = ( Stair_Count / 10 ); /* �i�[��̎w�� */
        /* 40 ms���Ƃ�gyro_str�ɃW���C���p���x�Z���T�[�l����� (4 ms�����Ɖ���) */ 
         Gyro_Store[gyro_Ave_Arg] = ev3_gyro_sensor_get_rate(gyro_sensor); 
   }

   /* 40 ms�O�̃Z���T�[�l�ƍ��̃Z���T�[�l�̍������K��l���傫���ꍇ�K�i�ɂԂ������Ɣ��f���� */ 
   if((gyro_str - ev3_gyro_sensor_get_rate(gyro_sensor)) > 140 || (gyro_str - ev3_gyro_sensor_get_rate(gyro_sensor)) < (140 * (-1)) ) 
   { 
      /* �K�i���m�����Ŏ��� */ 
       ev3_speaker_set_volume(10);  
       ev3_speaker_play_tone(NOTE_C4, 100); 
       Floor_Search_Flag = FLAG_ON; 
   }

    Stair_Count++;
}


