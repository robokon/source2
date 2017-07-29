/*
 * line_tarce.h
 * 
 */

#include "line_trace.h"

#define DELTA_T 0.004
signed char forward;              /* �O��i���� */
signed char turn;                 /* ���񖽗� */
signed char pwm_L, pwm_R;         /* ���E���[�^PWM�o�� */
static float integral=0;          /* I���� */
static int diff [2];              /* �J���[�Z���T�̍��� */ 
int count  = 0;                   /* ���O�o�� */
/* PID�p�����[�^ */
#define KP 0.7
#define KI 1.08
#define KD 0.03

//*****************************************************************************
// �֐��� : line_tarce_main
// ���� : 
// �Ԃ�l : �Ȃ�
// �T�v : 
//       
//*****************************************************************************
void line_tarce_main(int gray_color)
{
    signed char forward;      /* �O��i���� */
    signed char turn;         /* ���񖽗� */
    signed char pwm_L, pwm_R; /* ���E���[�^PWM�o�� */

    int32_t motor_ang_l, motor_ang_r;
    int gyro, volt;
    uint8_t color_sensor_reflect;
    
    tail_control(TAIL_ANGLE_DRIVE); /* �o�����X���s�p�p�x�ɐ��� */

    color_sensor_reflect= ev3_color_sensor_get_reflect(color_sensor);

    int temp_p=1000;
    int temp_d=1000;

    if (sonar_alert() == 1) /* ��Q�����m */
    {
        forward = turn = 0; /* ��Q�������m�������~ */
    }
    else
    {
    	/* PID���� */
        forward = 100; /* �O�i���� */
        float p,i,d;
        diff[0] = diff[1];
        diff[1] = color_sensor_reflect - ((gray_color)/2);
        integral += (diff[1] + diff[0]) / 2.0 * DELTA_T;
        
        p = KP * diff[1];
        i = KI * integral;
        d = KD * (diff[1]-diff[0]) / DELTA_T;
        
        turn = p + i + d;
        temp_p = p;
        temp_d = d;
        
        /* ���[�^�l���� */
        if(100 < turn)
        {
            turn = 100;
        }
        else if(turn < -100)
        {
            turn = -100;
        }
    }

    /* �|���U�q����API �ɓn���p�����[�^���擾���� */
    motor_ang_l = ev3_motor_get_counts(left_motor);
    motor_ang_r = ev3_motor_get_counts(right_motor);
    gyro = ev3_gyro_sensor_get_rate(gyro_sensor);
    volt = ev3_battery_voltage_mV();

    /* ���O�o�� */
    count++;
    log_Str(color_sensor_reflect,(int16_t)gyro, (int16_t)temp_p, (int16_t)temp_d, (int16_t)count);

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
    
    /* �߂�{�^��or�]�񂾂�I�� */
    if(ev3_button_is_pressed(BACK_BUTTON))
    {
        wup_tsk(MAIN_TASK);
    }
    if(gyro < -150 || 150 < gyro)
    {
        wup_tsk(MAIN_TASK);
    }
}

/* end of file */
