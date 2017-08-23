/*
 * line_tarce.h
 * 
 */

#include "line_trace.h"

#define DELTA_T 0.004
signed char forward = DEFAULT_SPEED;              /* �O��i���� */
signed char turn;                 /* ���񖽗� */
static float integral=0;          /* I���� */
static float diff [2] = {0,0};      /* �J���[�Z���T�̍��� */ 
static int black_count = 0;
signed char pwm_L=0, pwm_R=0; /* ���E���[�^PWM�o�� */

static float kp = KP;
static float kd = KD;
static float target = TARGET;


static float normalize_color_sensor_reflect(uint8_t color_sensor_refelect, signed char light_white, signed char light_black);

unsigned char detect_curve(signed char turn){
    static int old_turn[TURN_MAX];
    static int turnIndex = 0;
    static int plus_turn_num = 0;
    static int minus_turn_num = 0;
    static int neutral_turn_num = TURN_MAX;

    float minus_per, plus_per;
    int remove_turn = old_turn[turnIndex];
//    float turn_plus_threshold = TURN_THRESHOLD * ((1 - target) * 2);
//    float turn_minus_threshold = TURN_THRESHOLD * (target * 2);
    float turn_plus_threshold = TURN_THRESHOLD;
    float turn_minus_threshold = TURN_THRESHOLD;

    old_turn[turnIndex++] = turn;
    turnIndex %= TURN_MAX;

    if(remove_turn > turn_plus_threshold)
    {
       plus_turn_num--; 
    }
    else if ((remove_turn * -1) > turn_minus_threshold)
    {
       minus_turn_num--; 
    }
    else
    {
        neutral_turn_num--;
    }

    if(turn > turn_plus_threshold)
    {
       plus_turn_num++; 
    }
    else if ((turn * -1) > turn_minus_threshold)
    {
       minus_turn_num++; 
    }
    else
    {
        neutral_turn_num++;
    }

    minus_per = (float)minus_turn_num / TURN_MAX;
    plus_per = (float)plus_turn_num / TURN_MAX;
    log_Str(144,plus_turn_num,minus_turn_num,neutral_turn_num, (minus_per > TURN_PER_THRESHOLD || plus_per > TURN_PER_THRESHOLD));

    if (minus_per > TURN_PER_THRESHOLD) {
        return -1;
    } else if (plus_per > TURN_PER_THRESHOLD) {
        return 1;
    }
    return 0;
}

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
    unsigned char curve;

    uint8_t color_sensor_reflect;
    
    tail_control(TAIL_ANGLE_DRIVE); /* �o�����X���s�p�p�x�ɐ��� */

    /* ���Z���T�l�擾 */
    color_sensor_reflect= ev3_color_sensor_get_reflect(color_sensor);
    
    /* PID����ɂ��^�[���l���߂� */
    turn = pid_control(color_sensor_reflect, light_white, light_black);

    /* �J�[�u���m(�쐬�r��) */
    curve = detect_curve(turn);
    if (curve) {
        if (curve == 1) {
            target = TARGET - CURVE_TARGET_OFFSET;
        } else if (curve == -1) {
            target = TARGET + CURVE_TARGET_OFFSET;
        }
        kp = CURVE_KP;
        kd = CURVE_KD;
        ev3_speaker_set_volume(15); 
        ev3_speaker_play_tone(NOTE_C4, 100);
        forward = CURVE_SPEED;
    } else {
        target = TARGET;
        kp = KP;
        kd = KD;
        forward = DEFAULT_SPEED;
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
//            ev3_speaker_set_volume(30); 
//            ev3_speaker_play_tone(NOTE_C4, 100);
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

    normalize_reflect_value = normalize_color_sensor_reflect(color_sensor_reflect, light_white, light_black);

    diff[0] = diff[1];
    diff[1] = normalize_reflect_value - target;
    integral += (diff[1] + diff[0]) / 2.0 * DELTA_T;
    
    p = kp * diff[1];
    i = KI * integral;
    d = kd * (diff[1]-diff[0]) / DELTA_T;
    
    turn = (p + i + d) * KTURN; //���K���ŏo�����l��0-100�ɂ��邽��

    //target�̒l�ɂ��turn�l�̕␳
    if (turn > 0) {
        turn = TURN_PLUS_CORRECT_EXP;
    } else {
        turn = TURN_MINUS_CORRECT_EXP;
    }
    
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
    log_Str(color_sensor_reflect, normalize_reflect_value, p, d, turn);
    
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
