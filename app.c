/**
 ******************************************************************************
 ** �t�@�C���� : app.c
 **
 ** �T�v : 2�֓|���U�q���C���g���[�X���{�b�g��TOPPERS/HRP2�pC�T���v���v���O����
 **
 ** ���L : sample_c4 (sample_c3��Bluetooth�ʐM�����[�g�X�^�[�g�@�\��ǉ�)
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

int bt_cmd = 0;     /* Bluetooth�R�}���h 1:�����[�g�X�^�[�g */
FILE *bt = NULL;     /* Bluetooth�t�@�C���n���h�� */
signed char light_white=0;     /* ���F�̌��Z���T�l */
signed char light_black=100;       /* ���F�̌��Z���T�l */
signed char target_value=0;      /* �ڕW�l */
int mode_flg = 0;          /* ���[�h�ύX�̃t���O */

/* �e������� */
STATUS main_status = STAT_UNKNOWN;

/* ���b�N�A�b�v�Q�[�g���[�h�ڍs����(cm) */
#define START_TO_LOOKUP 5


/* ���C���^�X�N */
void main_task(intptr_t unused)
{

    /* LCD��ʕ\�� */
    ev3_lcd_fill_rect(0, 0, EV3_LCD_WIDTH, EV3_LCD_HEIGHT, EV3_LCD_WHITE);
    ev3_lcd_draw_string("EV3way-ET sample_c4", 0, CALIB_FONT_HEIGHT*1);

    /* �Z���T�[���̓|�[�g�̐ݒ� */
    ev3_sensor_config(sonar_sensor, ULTRASONIC_SENSOR);
    ev3_sensor_config(color_sensor, COLOR_SENSOR);
    ev3_color_sensor_get_reflect(color_sensor); /* ���˗����[�h */
    ev3_sensor_config(touch_sensor, TOUCH_SENSOR);
    ev3_sensor_config(gyro_sensor, GYRO_SENSOR);
    /* ���[�^�[�o�̓|�[�g�̐ݒ� */
    ev3_motor_config(left_motor, LARGE_MOTOR);
    ev3_motor_config(right_motor, LARGE_MOTOR);
    ev3_motor_config(tail_motor, LARGE_MOTOR);
    ev3_motor_reset_counts(tail_motor);

    /* Open Bluetooth file */
    bt = ev3_serial_open_file(EV3_SERIAL_BT);
    assert(bt != NULL);

    /* ���O�o�͏����̏����� */
    initialize_log(bt);


    /* �K���̈ʒu�������l */
    while(1)
    {
        tail_control(-100);
        if (ev3_touch_sensor_is_pressed(touch_sensor) == 1)
        {
            ev3_speaker_set_volume(50); 
            ev3_speaker_play_tone(NOTE_C4, 100);
            break;/* �^�b�`�Z���T�������ꂽ */
            
        }
    }
    ev3_motor_reset_counts(tail_motor);
    tslp_tsk(1000); /* 1000msec�E�F�C�g */
    
    ev3_led_set_color(LED_ORANGE); /* �����������ʒm */

    /* �L�����u���[�V�����X�^�[�g�ҋ@ */
    while(1)
    {
        tail_control(TAIL_ANGLE_STAND_UP); /* ���S��~�p�p�x�ɐ��� */

        if (ev3_touch_sensor_is_pressed(touch_sensor) == 1)
        {
            break; /* �^�b�`�Z���T�������ꂽ */
        }
        tslp_tsk(10); /* 10msec�E�F�C�g */
    }
    /* ���s���[�^�[�G���R�[�_�[���Z�b�g */
    ev3_motor_reset_counts(left_motor);
    ev3_motor_reset_counts(right_motor);

    /* �W���C���Z���T�[���Z�b�g */
    ev3_gyro_sensor_reset(gyro_sensor);
    balance_init(); /* �|���U�qAPI������ */
    Distance_init(); /* �����v���ϐ������� */
    
//    /*�L�����u���[�V�����X�^�[�g����*/
//    while(1)
//    {
//        float tail = 0;
//        tail = tail_control(TAIL_ANGLE_START);
//        if(tail==0)
//        {
//            break;
//        }
//    }
//    
//    // �L�����u���[�V���������n���h���J�n
//    ev3_sta_cyc(CAL_CYC1);
//    // �o�b�N�{�^�����������܂ő҂�
//    slp_tsk();
//    // �����n���h����~
//    ev3_stp_cyc(CAL_CYC1); 
//    ev3_motor_stop(left_motor, false);
//    ev3_motor_stop(right_motor, false);

// �ꎞ�I�ɏ�����
    light_black = 0;
    light_white = 100;

    /* �L�����u���[�V�����Őݒ肵�����Z���T�l�����O�o�� */
    log_Str(light_white,0,0,0,0);
    log_Str(light_black,0,0,0,0);

    /* Bluetooth�ʐM�^�X�N�̋N�� */
    act_tsk(BT_TASK);
    
    /* �X�^�[�g�ҋ@ */
    while(1)
    {
        tail_control(TAIL_ANGLE_STAND_UP); /* ���S��~�p�p�x�ɐ��� */

        if (bt_cmd == 1)
        {
            ev3_speaker_play_tone(NOTE_C4, 100);
            break; /* �����[�g�X�^�[�g */
        }
        else
        if(bt_cmd == 2)
        {
            ev3_speaker_play_tone(NOTE_G4, 50);
            break; /* �����[�g�X�^�[�g */
        }

        if (ev3_touch_sensor_is_pressed(touch_sensor) == 1)
        {
            break; /* �^�b�`�Z���T�������ꂽ */
        }
        tslp_tsk(10); /* 10msec�E�F�C�g */
    }
    
    ter_tsk(BT_TASK);

    /* ���s���[�^�[�G���R�[�_�[���Z�b�g */
    ev3_motor_reset_counts(left_motor);
    ev3_motor_reset_counts(right_motor);

    /* �W���C���Z���T�[���Z�b�g */
    ev3_gyro_sensor_reset(gyro_sensor);
    balance_init(); /* �|���U�qAPI������ */
    Distance_init(); /* �����v���ϐ������� */
    
    ev3_led_set_color(LED_GREEN); /* �X�^�[�g�ʒm */
    
    /* �X�^�[�g�ʒm��A�ʏ�̃��C���g���[�X�Ɉڍs����悤�ɐݒ� */
    main_status = STAT_NORMAL; 
    
    /*�X�^�[�g����*/
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
    // �����n���h���J�n
    ev3_sta_cyc(MAIN_CYC1);
    //ev3_sta_cyc(TEST_EV3_CYC2);

    // �o�b�N�{�^�����������܂ő҂�
    slp_tsk();
    // �����n���h����~
    ev3_stp_cyc(MAIN_CYC1); 
    

    ev3_motor_stop(left_motor, false);
    ev3_motor_stop(right_motor, false);

    close_log();
    fclose(bt);

    ext_tsk();
}

//*****************************************************************************
// �֐��� : cal_cyc1
// ���� : ����
// �Ԃ�l :
// �T�v : �L�����u���[�V���������^�X�N
//*****************************************************************************
void cal_cyc1(intptr_t exinf)
{
    Distance_update(); /* �ړ��������Z */
    signed char forward=50;
    signed char turn=20;
    
    /* ���Z���T�l�擾 */
    uint8_t color_sensor_reflect= ev3_color_sensor_get_reflect(color_sensor);

    /* ���Z���T�l�̍ő�l��light_white
       ���Z���T�l�̍ő�l��light_black�ɐݒ� */
    if(light_white < color_sensor_reflect)
    {
        light_white = color_sensor_reflect;
    }
    if(light_black > color_sensor_reflect)
    {
        light_black = color_sensor_reflect;
    }

    tail_control(TAIL_ANGLE_DRIVE); /* �o�����X���s�p�p�x�ɐ��� */
    
    int32_t motor_ang_l, motor_ang_r;
    int gyro, volt;
    signed char pwm_L=0, pwm_R=0; /* ���E���[�^PWM�o�� */
    
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
//*****************************************************************************
// �֐��� : main_cyc1
// ���� : ����
// �Ԃ�l :
// �T�v : ���C�������^�X�N
//*****************************************************************************
void main_cyc1(intptr_t idx) 
{
    switch (main_status) {
        /* �ʏ퐧�䒆 */
        case STAT_NORMAL:
            /* �ʏ�̃��C���g���[�X���� */
            line_tarce_main(light_white,light_black);
            break;

        /* �K�i���䒆 */
        case STAT_STAIR:
            stair_main();
            break;

        /* ���b�N�A�b�v�Q�[�g���䒆 */
        case STAT_LOOK_UP_GATE:
            look_up_gate_main();
            break;

        /* �K���[�W���䒆 */
        case STAT_GAREGE:
            /* T.B.D */
            break;

        /* ���̑� */
        default:
            /* T.B.D */
            break;
    }

    Distance_update(); /* �ړ��������Z */
    
    if( mode_flg == 0 )
    {
        /* L�R�[�X���[�h�̏ꍇ */
        if( bt_cmd == 1 )
        {
            if( Distance_getDistance() > L_GOAL_DISTANCE )
            {
                /* DISTANCE_NOTIFY�ȏ�i�񂾂特���o�� */
                ev3_speaker_set_volume(100); 
                ev3_speaker_play_tone(NOTE_C4, 100);
                
                /* �����v���ϐ������� */
                Distance_init();
                
                /* �K�i���[�h�֐؂�ւ� */
                main_status = STAT_STAIR;
                mode_flg = 1;
            } 
        }
        /* R�R�[�X���[�h�̏ꍇ */
        else
        if( bt_cmd == 2 )
        {
            if( Distance_getDistance() > R_GOAL_DISTANCE )
            {
                /* DISTANCE_NOTIFY�ȏ�i�񂾂特���o�� */
                ev3_speaker_set_volume(100); 
                ev3_speaker_play_tone(NOTE_G4, 50);
                
                /* �����v���ϐ������� */
                Distance_init();
                
                /* ���b�N�A�b�v�Q�[�g���[�h�֐؂�ւ� */
                main_status = STAT_LOOK_UP_GATE;
                mode_flg = 1;
            }
        }
    }
}

//*****************************************************************************
// �֐��� : sonar_alert
// ���� : ����
// �Ԃ�l : 1(��Q������)/0(��Q������)
// �T�v : �����g�Z���T�ɂ���Q�����m
//*****************************************************************************
int sonar_alert(void)
{
    static unsigned int counter = 0;
    static int alert = 0;

    signed int distance;

    if (++counter == 40/4) /* ��40msec�������ɏ�Q�����m  */
    {
        /*
         * �����g�Z���T�ɂ�鋗����������́A�����g�̌��������Ɉˑ����܂��B
         * NXT�̏ꍇ�́A40msec�������x���o����̍ŒZ��������ł��B
         * EV3�̏ꍇ�́A�v�m�F
         */
        distance = ev3_ultrasonic_sensor_get_distance(sonar_sensor);
        if ((distance <= SONAR_ALERT_DISTANCE) && (distance >= 0))
        {
            alert = 1; /* ��Q�������m */
        }
        else
        {
            alert = 0; /* ��Q������ */
        }
        counter = 0;
    }

    return alert;
}

//*****************************************************************************
// �֐��� : tail_control
// ���� : angle (���[�^�ڕW�p�x[�x])
// �Ԃ�l : ����
// �T�v : ���s�̊��S��~�p���[�^�̊p�x����
//*****************************************************************************
float tail_control(signed int angle)
{
    float pwm = (float)(angle - ev3_motor_get_counts(tail_motor))*P_GAIN; /* ��ᐧ�� */
    /* PWM�o�͖O�a���� */
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
// �֐��� : bt_task
// ���� : unused
// �Ԃ�l : �Ȃ�
// �T�v : Bluetooth�ʐM�ɂ�郊���[�g�X�^�[�g�B Tera Term�Ȃǂ̃^�[�~�i���\�t�g����A
//       ASCII�R�[�h��1�𑗐M����ƁA�����[�g�X�^�[�g����B
//*****************************************************************************
void bt_task(intptr_t unused)
{
    while(1)
    {
        uint8_t c = fgetc(bt); /* ��M */
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
        fputc(c, bt); /* �G�R�[�o�b�N */
    }
}

/* end of file */
