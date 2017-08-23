/*
 * line_tarce.h
 * 
 */

#include "line_trace.h"

#define DELTA_T 0.004
signed char forward = 100;              /* �O��i���� */
signed char turn;                 /* ���񖽗� */
signed char pwm_L, pwm_R;         /* ���E���[�^PWM�o�� */
static float integral=0;          /* I���� */
static float diff [2] = {0,0};      /* �J���[�Z���T�̍��� */ 
static int black_count = 0;
static int curve_count = 0;
signed char pwm_L=0, pwm_R=0; /* ���E���[�^PWM�o�� */

/* PID�p�����[�^ */
#define KP 0.8
#define KI 0
#define KD 0.04

static float normalize_color_sensor_reflect(uint8_t color_sensor_refelect, signed char light_white, signed char light_black);

//*****************************************************************************
// �֐��� : line_tarce_main
// ���� :   signed char light_white ���̃Z���T�l
//          signed char light_black ���̃Z���T�l
// �Ԃ�l : �Ȃ�
// �T�v : 
//       
//*****************************************************************************
void line_tarce_main(signed char light_white, signed char light_black)
{
    signed char turn;         /* ���񖽗� */

    uint8_t color_sensor_reflect;
    
    tail_control(TAIL_ANGLE_DRIVE); /* �o�����X���s�p�p�x�ɐ��� */

    /* ���Z���T�l�擾 */
    color_sensor_reflect= ev3_color_sensor_get_reflect(color_sensor);
    
    /* PID����ɂ��^�[���l���߂� */
    turn = pid_control(color_sensor_reflect, light_white, light_black);

    /* �J�[�u���m(�쐬�r��) */
    if((pwm_L-pwm_L>15)||(pwm_L-pwm_L>15))
    {
        curve_count++;
        if(curve_count == 25)
        {
            forward = 80;
        }
    }
    else
    {
        curve_count = 0;
    }
    
    /* �|���U�q���䏈�� */
    balanceControl(forward, turn);
    
    /* �Z���T�l���ڕW�l�{15�ȏ��black_count��
       �A�����m������O���[�Ƃ݂Ȃ����� */
    // �O���[�̒l�@50�`60���炢
    if( color_sensor_reflect > (light_white+light_black)/2)
    {
        black_count++;
        if(black_count==100)
        {
            // 10��A���������m 
            ev3_speaker_set_volume(30); 
            ev3_speaker_play_tone(NOTE_C4, 100);
        }
    }
    else
    {
        black_count=0;
    }
}

//*****************************************************************************
// �֐��� : pid_control
// ���� : uint8_t color_sensor_reflect ���Z���T�l
//        signed char light_white      ���̃Z���T�l
//        signed char light_black      ���̃Z���T�l
// �Ԃ�l : signed char                �^�[���l
// �T�v : 
//
//*****************************************************************************
signed char pid_control(uint8_t color_sensor_reflect, signed char light_white, signed char light_black)
{
    /* PID����ɂ��^�[���l�����߂� */
    float p,i,d;
    float normalize_reflect_value;
    float target = 0.5;

    normalize_reflect_value = normalize_color_sensor_reflect(color_sensor_reflect, light_white, light_black);

    diff[0] = diff[1];
    diff[1] = normalize_reflect_value - target;
    integral += (diff[1] + diff[0]) / 2.0 * DELTA_T;
    
    p = KP * diff[1];
    i = KI * integral;
    d = KD * (diff[1]-diff[0]) / DELTA_T;
    
    turn = (p + i + d) * 100; //���K���ŏo�����l��0-100�ɂ��邽��
    
    /* ���[�^�l���� */
    if(100 < turn)
    {
        turn = 100;
    }
    else if(turn < -100)
    {
        turn = -100;
    }
    
    /* ���O�o�� */
    log_Str(color_sensor_reflect, normalize_reflect_value, p, i, d);
    
    return turn;
}

//*****************************************************************************
// �֐��� : normalize_color_sensor_reflect
// ���� : signed char color_sensor_reflect ���Z���T�l
//        signed char light_white          �L�����u���[�V�����������̒l
//        signed char light_black          �L�����u���[�V�����������̒l
// �Ԃ�l : float                    ���K���������Z���T�l
// �T�v : ���Z���T�l���L�����u���[�V���������f�[�^�����Ƃ�0~1�Ő��K������
//
//*****************************************************************************
static float normalize_color_sensor_reflect(uint8_t color_sensor_reflect, signed char light_white, signed char light_black)
{
    return (float)(color_sensor_reflect - light_black) / (float)(light_white - light_black);
}


//*****************************************************************************
// �֐��� : balanceControl
// ���� : signed char forward �O�i����
//        , signed char turn  �^�[���l
// �Ԃ�l : 
// �T�v : 
//
//*****************************************************************************
void balanceControl(signed char forward, signed char turn)
{
    int32_t motor_ang_l, motor_ang_r;
    int gyro, volt;
    
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
