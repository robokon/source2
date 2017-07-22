#include "garage.h"
#include "app.h"
#include "Distance.h"

/* nakagawa Add_STA */
#define SLOW_DISTANCE (100) /* �ᑬ�����b�� TBD */
#define STOP_DISTANCE (150) /* �K���[�W�C�������b�� TBD*/
/* nakagawa Add_END */

signed char forward;      /* �O��i���� */
signed char turn;         /* ���񖽗� */
signed char pwm_L, pwm_R; /* ���E���[�^PWM�o�� */

/* nakagawa Add_STA */
unsigned char slow_flag = 0;
float left_zankyori = 0.0;
float right_zankyori = 0.0;
extern int LIGHT_WHITE;         /* ���F�̌��Z���T�l */
extern int LIGHT_BLACK;
/* nakagawa Add_END */

//*****************************************************************************
// �֐��� : garage_main
// ���� : 
// �Ԃ�l : �Ȃ�
// �T�v : 
//       
//*****************************************************************************
void garage_main()
{
    int32_t motor_ang_l, motor_ang_r;
    int gyro, volt;

/* nakagawa Add_STA */
    float distance4msL = 0.0; //���^�C����4ms�Ԃ̋���
    float distance4msR = 0.0; //�E�^�C����4ms�Ԃ̋���
    float distance = 0.0;     //���s����
/* nakagawa Add_END */

     if (ev3_button_is_pressed(BACK_BUTTON)) return;

     tail_control(TAIL_ANGLE_DRIVE); /* �o�����X���s�p�p�x�ɐ��� */

     if (sonar_alert() == 1) /* ��Q�����m */
     {
         forward = turn = 0; /* ��Q�������m�������~ */
     }
     else
     {
         forward = 30; /* �O�i���� */
         if (ev3_color_sensor_get_reflect(color_sensor) >= (LIGHT_WHITE + LIGHT_BLACK)/2)
         {
             turn =  20; /* �����񖽗� */
         }
         else
         {
             turn = -20; /* �E���񖽗� */
         }
     }

     /* �|���U�q����API �ɓn���p�����[�^���擾���� */
     motor_ang_l = ev3_motor_get_counts(left_motor);/*�����[�^�[*/
     motor_ang_r = ev3_motor_get_counts(right_motor);/*�E���[�^�[*/
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

/* nakagawa Add_STA */
     /* �K���[�W�C���N���㑖�s�����������i�񂾂瑬�x��ቺ������ */
    if(distance > SLOW_DISTANCE && slow_flag == 0){
        slow_flag = 1;
        pwm_L = pwm_L /2;
        pwm_R = pwm_R /2;
    }
/* nakagawa Add_END */

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

/* nakagawa Add_STA */
    Distance_update();/* �����X�V�i4ms�Ԃ̈ړ������𖈉���Z���Ă���j */

    distance4msL= Distance_getDistance4msLeft();    /* �������擾 */
    distance4msR= Distance_getDistance4msRight();   /* �E�����擾 */

    left_zankyori  += distance4msL;
    right_zankyori += distance4msR;

    distance += (left_zankyori + right_zankyori) / 2.0; //���E�^�C���̑��s�����𑫂��Ċ���

    /* �K���[�W�C���N���㑖�s�����������i�񂾂��~������ */
    if (distance > STOP_DISTANCE){
        ev3_motor_stop(left_motor, true);
        ev3_motor_stop(right_motor, true);
        
    }
/* nakagawa Add_END */
}

static void garage_touritu_stop(void){
    /* T B D */
}
/* end of file */
