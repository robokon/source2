#include<float.h>

#include "ev3api.h"
#include "app.h"
#include "balancer.h"
#include "common.h"
#include "stair.h"
#include "module_cfg.h"
#include "Distance.h"

signed char forward;                /* �O��i���� */
signed char turn;                   /* ���񖽗� */
signed char pwm_L, pwm_R;           /* ���E���[�^PWM�o�� */


/* ~~~~~~~~~~~~~~~~~~~~~~~�v�����̒�`~~~~~~~~~~~~~~~~~~~~~~~~ */
#define GYRO_AVE_OK_BORDER    40    /* �W���C���̈��肵���̊�l  */
#define STAGE_ZERO_BORDEAR    140   /* 0�K -> 1�K�̃W���C�����E�� */
#define STAGE_ONE_BORDEAR     160   /* 1�K -> 2�K�̃W���C�����E�� */
#define STAGE_TWO_BORDEAR     140   /* 2�K -> 0�K�̃W���C�����E�� */
#define GYRO_AVE_STR_MAX      20    /* 40 ms�Ԋu�̃Z���T�l�����ŕ��ϒl�o�����@*/
#define FLOOR_UP_OK_COUNT     200    /* �ǂ̂��炢�A�����ăW���C�������肵�Ă�����o�����Ɣ��肷�邩 <- ����łԂ����Ă���^�񒆂܂ł𒲐� */

#define FLOOR_ZERO_RUN_SPEED  60    /* 0�K�̑��s���x */
#define FLOOR_ONE_RUN_SPEED   60    /* 1�K�̑��s���x */
#define FLOOR_TWO_RUN_SPEED   60    /* 2�K�̑��s���x */

#define FLOOR_ONE_SPIN_VALUE  700   /* 360�x��] */
#define FLOOR_TWO_SPIN_VALUE  900   /* 450�x��] */

#define FLOOR_ZERO_WAITING_COUNT 1000
#define FLOOR_ONE_WAITING_COUNT  1000
#define FLOOR_TWO_WAITING_COUNT  1000

/* ~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~ */


int Stair_Count = 0;                /* �K�i�̊֐��ɓ������� */

int Floor_Search_Flag;              /* �t���A���m�t���O  */
#define FLAG_OFF              0     /* �t���A���m�I�t */
#define FLAG_ON               1     /* �t���A���m�I�� */

int Floor_Status;                   /* �t���A�X�e�[�^�X */
#define STAGE_ZERO            0     /* �K�i 0 �K */
#define STAGE_ONE             1     /* �K�i 1 �K */
#define STAGE_TWO             2     /* �K�i 2 �K */
#define STAGE_LAST            3     /* �K�i 2 �K�Ō�̒��� */

int Run_Mode;                       /* ���s���[�h */
#define RUN_MODE_STAY         0     /* ���s���[�h�i��~�j*/
#define RUN_MODE_RUN          1     /* ���s���[�h�i���s���j*/
#define RUN_MODE_SPIN         2     /* ���s���[�h�i��]���j*/

int Stage_One_Spin_Status;          /* 1�K�̉�]�X�e�[�^�X */
int Stage_Two_Spin_Status;          /* 2�K�̉�]�X�e�[�^�X */
#define SPIN_FINISH           0     /* ��]���� */
#define SPIN_NOT_FINISH       1     /* ��]������ */

int Sipn_Func_Count = 0;            /* Spin_Func�̃J�E���g�l */
int RIGHT_info = 0;                 /* �E���[�^�̏�� */
int RIGHT_info_first = 0;           /* ��]���O�̊p�ʒu */


int Gyro_Pre_40ms = 0;              /* 40 ms�O�̃W���C���Z���T�l */
int Ave_Count;                      /* �i�[��̃J�E���^ */
int Gyro_Store[GYRO_AVE_STR_MAX];   /* �W���C���Z���T�[�l�̊i�[�� */
int Gyro_Ave = 0;                   /* ���ϒl */
int Gyro_Ave_OK_COUNT = 0;          /* �t���A���m���̃Z���T�[�l��OK�̃J�E���g���@*/

int Waiting_Flag;                   /* �|�����䂾���s�����[�h�̃t���O */
#define WAITING_FLAG_ON       1     /* �|�����䂾���s�����[�h�̃t���O ON */
#define WAITING_FLAG_OFF      0     /* �|�����䂾���s�����[�h�̃t���O OFF */

int Waiting_Count_Flag;
#define WAITING_COUNT_FLAG_ON       1     /* �t���O ON */
#define WAITING_COUNT_FLAG_OFF      0     /* �t���O OFF */
int Waiting_Count_Value;

extern int main_status;             /* ���C���X�e�[�^�X�̈��p */

/* �v���g�^�C�v�錾 */
void stair_Run(int forward_value, int turn_value);
int spin_func(int spin_end_value);
void stair_Stop(int Floor_Status);

//*****************************************************************************
// �֐��� : stair_main
// ���� :
// �Ԃ�l : �Ȃ�
// �T�v :
//
//*****************************************************************************
void stair_main()
{
    int i;    /* index */
    int ave_sum = 0;

    if(Stair_Count == 0)
    {
        /* �e�X�e�[�^�X�̏����� */
        Floor_Status = STAGE_ZERO;
        Run_Mode = RUN_MODE_RUN;
        Stage_One_Spin_Status = SPIN_NOT_FINISH;
        Stage_Two_Spin_Status = SPIN_NOT_FINISH;
        Floor_Search_Flag = FLAG_OFF;
        Ave_Count = 0;
        Waiting_Flag = WAITING_FLAG_OFF;
        Waiting_Count_Flag = WAITING_COUNT_FLAG_OFF;
    }

    /* �W���C���Z���T�l�����I�ɕۑ��A���ϒl���Z�o���� */
   if((Stair_Count % 10) == 0)
   {
        /* 40 ms���Ƃ�gyro_str�ɃW���C���p���x�Z���T�[�l����� (4 ms�����Ɖ���) */
         Gyro_Pre_40ms = ev3_gyro_sensor_get_rate(gyro_sensor);
         /* ���ϒl�p�ɃX�g�A���� */
         Gyro_Store[Ave_Count] = Gyro_Pre_40ms;
         /* �X�g�A�����J�E���g */
         Ave_Count++;
         /* �X�g�A��������ł������Ƃ��ɃN���A����0����J�E���g */
         if(Ave_Count == GYRO_AVE_STR_MAX)
         {
             Ave_Count = 0;
         }

         /* �X�g�A������������܂�����A���ϒl���Z�o���� */
         if( Gyro_Store[GYRO_AVE_STR_MAX - 1 ] != 0)
         {
             for(i = 0 ; i < GYRO_AVE_STR_MAX - 1 ; i++)
             {
                 /* ���ϒl�͐�Βl�Ōv�Z */
                 if (Gyro_Store[i] < 0)
                 {
                     Gyro_Store[i] = (Gyro_Store[i] * (-1) );
                 }
                 ave_sum = ave_sum + Gyro_Store[i];
             }
             Gyro_Ave = (ave_sum / GYRO_AVE_STR_MAX);
         }
   }

   if(Waiting_Flag == WAITING_FLAG_OFF)
   {
       /* ~~~~~~~~~~~~~~~~~~~~~~~~~~0�K�̓���~~~~~~~~~~~~~~~~~~~~~~~~~~ */
       if(Floor_Status == STAGE_ZERO)
       {
           /* 40 ms�O�̃Z���T�[�l�ƍ��̃Z���T�[�l�̍������K��l���傫���ꍇ�K�i�ɂԂ������Ɣ��f���� */
           if((Gyro_Pre_40ms - ev3_gyro_sensor_get_rate(gyro_sensor)) > STAGE_ZERO_BORDEAR
             || (Gyro_Pre_40ms - ev3_gyro_sensor_get_rate(gyro_sensor)) < (STAGE_ZERO_BORDEAR * (-1)) )
           {
               /* �K�i���m�����Ŏ��� */
               ev3_speaker_set_volume(10);
               ev3_speaker_play_tone(NOTE_C4, 100);
               /* �t���A���m�t���O��ON�ɂ��� */
               Floor_Search_Flag = FLAG_ON;
           }

           /* �t���A���m�@*/
           if(Floor_Search_Flag == FLAG_ON)
           {
               /* �W���C���̕��ϒl���K��l�ȉ��� */
               if(Gyro_Ave < GYRO_AVE_OK_BORDER)
               {
                   /* �K��l�ȉ��̐����J�E���g���� */
                   Gyro_Ave_OK_COUNT++;
               }
               else
               {
               /* �K��l�ȏオ�����烊�Z�b�g */
               Gyro_Ave_OK_COUNT = 0;
               }
               /* �W���C���Z���T�[�l�����肵����@������Ɣ���@*/
               if(Gyro_Ave_OK_COUNT > FLOOR_UP_OK_COUNT)
               {
                   /* ��x��~���ăt���A�X�e�[�^�X���グ��@*/
                   Waiting_Flag = WAITING_FLAG_ON;
                   /* �t���O�����A�J�E���g�l�̏����� */
                   Floor_Search_Flag = FLAG_OFF;
                   Gyro_Ave_OK_COUNT = 0;
               }
           }
           /* �|������v���O�����Ăяo�� */
           stair_Run(FLOOR_ZERO_RUN_SPEED, 0);
       }
       /* ~~~~~~~~~~~~~~~~~~~~~~~~~~1�K�̓���~~~~~~~~~~~~~~~~~~~~~~~~~~ */
       else if(Floor_Status == STAGE_ONE)
       {
           if(Stage_One_Spin_Status == SPIN_NOT_FINISH)
           {
             /* 360�x��]������X�e�[�^�X�X�V */
             Stage_One_Spin_Status = spin_func(FLOOR_ONE_SPIN_VALUE);
           }
           else
           {
               /* 40 ms�O�̃Z���T�[�l�ƍ��̃Z���T�[�l�̍������K��l���傫���ꍇ�K�i�ɂԂ������Ɣ��f���� */
               if((Gyro_Pre_40ms - ev3_gyro_sensor_get_rate(gyro_sensor)) > STAGE_ONE_BORDEAR
                 || (Gyro_Pre_40ms - ev3_gyro_sensor_get_rate(gyro_sensor)) < (STAGE_ONE_BORDEAR * (-1)) )
               {
               /* �K�i���m�����Ŏ��� */
                   ev3_speaker_set_volume(10);
                   ev3_speaker_play_tone(NOTE_C4, 100);
                   /* �t���A���m�t���O��ON�ɂ��� */
                   Floor_Search_Flag = FLAG_ON;
               }

               /* �t���A���m�@*/
               if(Floor_Search_Flag == FLAG_ON)
               {
                   /* �W���C���̕��ϒl���K��l�ȉ��� */
                   if(Gyro_Ave < GYRO_AVE_OK_BORDER)
                   {
                       /* �K��l�ȉ��̐����J�E���g���� */
                       Gyro_Ave_OK_COUNT++;
                   }
                   else
                   {
                       /* �K��l�ȏオ�����烊�Z�b�g */
                       Gyro_Ave_OK_COUNT = 0;
                   }
                   /* �K��l�ȏ�W���C���Z���T�[�l�����肵����@������Ɣ���@*/
                   if(Gyro_Ave_OK_COUNT > FLOOR_UP_OK_COUNT)
                   {
                       /* ��x��~���ăt���A�X�e�[�^�X���グ��@*/
                       Waiting_Flag = WAITING_FLAG_ON;
                       /* �t���O�����A�J�E���g�l�̏����� */
                       Floor_Search_Flag = FLAG_OFF;
                       Gyro_Ave_OK_COUNT = 0;
                   }
               }
               stair_Run(FLOOR_ONE_RUN_SPEED, 0); /* �|������v���O�����Ăяo�� */
           }
       }
       /* ~~~~~~~~~~~~~~~~~~~~~~~~~~2�K�̓���~~~~~~~~~~~~~~~~~~~~~~~~~~ */
       else if(Floor_Status == STAGE_TWO)
       {
           if(Stage_Two_Spin_Status == SPIN_NOT_FINISH)
           {
               /* 450�x��]������X�e�[�^�X�X�V */
               Stage_Two_Spin_Status = spin_func(FLOOR_TWO_SPIN_VALUE);
           }
           else
           {
               /* ��x��~���ăt���A�X�e�[�^�X���グ��@*/
               Waiting_Flag = WAITING_FLAG_ON;
           }
       }
       else if(Floor_Status == STAGE_LAST)
       {
               /* ���傱���Ɛi��Ń��C���g���[�X�Ɉ����n�� */
               stair_Run(FLOOR_TWO_RUN_SPEED, 0); /* �|������v���O�����Ăяo�� */
               /* ���C���X�e�[�^�X���K���[�W�Ɉ����n�� */
               main_status = STAT_GAREGE;

       } /* end of (Floor_Status == STAGE_ZERO) */
   }
   else /* (Waiting_Flag == WAITING_FLAG_ON) */
   {
       /* �i�ʂ��Ƃ̒�~�ƃt���A�X�e�[�^�X�̍X�V */
       stair_Stop(Floor_Status);
   }
    Stair_Count++;
}

//*****************************************************************************
// �֐��� : stair_Run
// ����  : int forward_value, int turn_value
// �Ԃ�l : �Ȃ�
// �T�v :
//
//*****************************************************************************

void stair_Run(int forward_value, int turn_value)
{
    int32_t motor_ang_l, motor_ang_r;
    int gyro, volt;

    forward = forward_value; /* �O�i���� */
    turn =  turn_value; /* �����񖽗� */

    tail_control(TAIL_ANGLE_DRIVE); /* �o�����X���s�p�p�x�ɐ��� */

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

//*****************************************************************************
// �֐��� : spin_func
// ����  : int spin_end_value
// �Ԃ�l : Sipn status
// �T�v :
//*****************************************************************************
int spin_func(int spin_end_value)
{
    int spin_status = SPIN_NOT_FINISH;
    int32_t motor_ang_l;
    int32_t motor_ang_r;
    int gyro;
    int volt;

    /* Run���[�h�̃X�e�[�^�X�ύX */
    Run_Mode = RUN_MODE_SPIN;

     /* ��]���O�̊p�ʒu���擾(����̂ݎ��s) */
     if(Sipn_Func_Count == 0)
     {
        RIGHT_info_first = ev3_motor_get_counts(right_motor);
     }

    /* RIGHT_info�l��"700"�ő��s�̂�360�x��] */
    /* RIGHT_info�l��"875(700 �~ 1.25)�ő��s�̂���450�x��](900�̕������450�x�ɋ߂�)" */
    /* ���C���g���[�X�ő��s������␳�ł����875�Ŗ��Ȃ��Ɣ��f */
    if ((RIGHT_info - RIGHT_info_first) >= spin_end_value )
    {
        forward = turn = 0; /* �O�r�^�������͈�U�X�g�b�v */

        tail_control(TAIL_ANGLE_DRIVE); /* �o�����X���s�p�p�x�ɐ��� */

        /* �|���U�q����API �ɓn���p�����[�^���擾���� */
        motor_ang_l = ev3_motor_get_counts(left_motor);
        motor_ang_r = ev3_motor_get_counts(right_motor);
        gyro = ev3_gyro_sensor_get_rate(gyro_sensor);
        volt = ev3_battery_voltage_mV();

        /* �|���U�q����API���Ăяo��*/
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

         ev3_motor_set_power(left_motor, (int)pwm_L);
         ev3_motor_set_power(right_motor, (int)pwm_R);

         /* ��]���� */
         Sipn_Func_Count = 0;
         spin_status = SPIN_FINISH;
         Run_Mode = RUN_MODE_RUN;
    }
    else
    {
        /* ���[�^�̃t���p���[�̃p�[�Z���g�l��ݒ� */
        pwm_L = 30;
        pwm_R = 30;

        ev3_motor_set_power(left_motor, ( -1 * (int)pwm_L));
        ev3_motor_set_power(right_motor, (int)pwm_R);
        /* �E���[�^�̊p�ʒu���擾���� */
        RIGHT_info = ev3_motor_get_counts(right_motor);
    }

    Sipn_Func_Count++;
    return(spin_status);
}


//*****************************************************************************
// �֐��� : stair_Stop
// ����  :
// �Ԃ�l : �Ȃ�
// �T�v :
//
//*****************************************************************************

void stair_Stop(int Floor_Status)
{
    int32_t motor_ang_l, motor_ang_r;
    int gyro, volt;

    if(Floor_Status == STAGE_ZERO)
    {
        if(Waiting_Count_Flag == WAITING_COUNT_FLAG_OFF)
        {
            Waiting_Count_Value = FLOOR_ZERO_WAITING_COUNT;
            Waiting_Count_Flag = WAITING_COUNT_FLAG_ON;
        }

        Waiting_Count_Value--;

        if(Waiting_Count_Value < 0)
        {
            Waiting_Flag = WAITING_FLAG_OFF;
            Floor_Status = STAGE_ONE;
        }
    }
    else if(Floor_Status == STAGE_ONE)
    {
        if(Waiting_Count_Flag == WAITING_COUNT_FLAG_OFF)
        {
            Waiting_Count_Value = FLOOR_ONE_WAITING_COUNT;
            Waiting_Count_Flag = WAITING_COUNT_FLAG_ON;
        }

        Waiting_Count_Value--;

        if(Waiting_Count_Value < 0)
        {
            Waiting_Flag = WAITING_FLAG_OFF;
            Floor_Status = STAGE_TWO;
        }
    }
    else
    {
        if(Waiting_Count_Flag == WAITING_COUNT_FLAG_OFF)
        {
            Waiting_Count_Value = FLOOR_TWO_WAITING_COUNT;
            Waiting_Count_Flag = WAITING_COUNT_FLAG_ON;
        }

        Waiting_Count_Value--;

        if(Waiting_Count_Value < 0)
        {
            Waiting_Flag = WAITING_FLAG_OFF;
            Floor_Status = STAGE_LAST;
        }
    }

    forward = 0; /* �O�i���� */
    turn =  0; /* �����񖽗� */

    tail_control(TAIL_ANGLE_DRIVE); /* �o�����X���s�p�p�x�ɐ��� */

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
