#include "line_trace.h"
#include "Distance.h"

#define DISTANCE_NOTIFY (500.0)

int grade_test_cnt = 0;     /*  ���J�E���g */
int grade_test_flg = 0;     /*  huragu */
int grade_test_touritu = 0;     /*  ���J�E���g */
int touritu_flg = 0;/* �|����� */
int end_flag = 0;
int tail_count = 0;  /* �K������� */

static float integral=0;          /* I���� */
static int diff [2];              /* �J���[�Z���T�̍��� */ 
/* PID�p�����[�^ */
#define KP 0.8
#define KI 0.0
#define KD 0.03

//*****************************************************************************
// �֐��� : garage_main
// ���� : 
// �Ԃ�l : �Ȃ�
// �T�v : 
//       
//*****************************************************************************

signed char forward;      /* �O��i���� */
signed char turn;         /* ���񖽗� */
signed char pwm_L, pwm_R; /* ���E���[�^PWM�o�� */

void garage_main(int gray_color)
{
    int32_t motor_ang_l, motor_ang_r;
    int gyro, volt;
	uint8_t color_sensor_reflect;

	int temp_p=1000;
    int temp_d=1000;

    if (ev3_button_is_pressed(DOWN_BUTTON))
	{

 ext_tsk();
		return;
	}
    if(grade_test_flg == 0){
        tail_control(TAIL_ANGLE_DRIVE); /* �o�����X���s�p�p�x�ɐ��� */
    }
	color_sensor_reflect= ev3_color_sensor_get_reflect(color_sensor);
/* ����@�`2017/8/25�Ή� STA */
/* line_tarace.c�̃R�[�h�𗬗p(forward�̂�30�ɕύX) */
	if (sonar_alert() == 1) /* ��Q�����m */
    {
        forward = turn = 0; /* ��Q�������m�������~ */
    }
    else
    {
    	/* PID���� */
        forward = 30; /* �O�i���� */
        float p,i,d;
        diff[0] = diff[1];
        diff[1] = color_sensor_reflect - ((gray_color)/2);
        integral += (diff[1] + diff[0]) / 2.0 * 0.004;
        
        p = KP * diff[1];
        i = KI * integral;
        d = KD * (diff[1]-diff[0]) / 0.004;
        
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
/* �����܂ŗ��p */
/* ����@�`2017/8/25�Ή� END */
	
	if(grade_test_flg == 1)
    {
        forward = turn = 0;
    }
    if(grade_test_touritu >= 2150){
        forward = -100;
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

    if(grade_test_flg == 1)
    {
        grade_test_touritu++;
        if(grade_test_touritu >= 1000)
        {
/* ����  �`2017/8/25�Ή� STA */
            if(grade_test_touritu >= 1750){
//            	if(tail_count == 2){    /*1500�J�E���g����1��K��������*/
                    tail_control(TAIL_ANGLE_STAND_UP); /* �|��������폜 �K�������� */
//            		tail_count = 3;
//            	}
            }else if(grade_test_touritu >= 1500){
//            	if(tail_count == 1){    /*1500�J�E���g����1��K��������*/
                    tail_control((TAIL_ANGLE_STAND_UP-20)); /* �|��������폜 �K�������� */
//            		tail_count = 2;
//            	}
            }else if(grade_test_touritu >= 1250){
//            	if(tail_count == 0){    /*1500�J�E���g����1��K��������*/
                    tail_control((TAIL_ANGLE_STAND_UP-30)); /* �|��������폜 �K�������� */
//            		tail_count = 1;
//            	}
            }else if(grade_test_touritu >= 1000){
                   tail_control((TAIL_ANGLE_STAND_UP-60)); /* �o�����X���s�p�p�x�ɐ��� */
            }

/* ���� �`2017/8/25�Ή� END */
#if 0
            else if(grade_test_touritu >= 2000){
                tail_control((TAIL_ANGLE_STAND_UP-40)); /* �|��������폜 �K�������� */
            }else if(grade_test_touritu >= 1000){
                tail_control((TAIL_ANGLE_STAND_UP-50)); /* �|��������폜 �K�������� */
            }
#endif            
            if(grade_test_touritu >= 2250)
            {
                if(grade_test_touritu < 2350){
                    ev3_speaker_set_volume(10); 
                    ev3_speaker_play_tone(NOTE_E6, 100);
                }
                if(touritu_flg == 0){
                    ev3_motor_stop(right_motor, true);
                    ev3_motor_stop(left_motor, true);
                    touritu_flg = 1;
                    ev3_speaker_set_volume(10); 
                    ev3_speaker_play_tone(NOTE_D6, 100);

                }
            }
        }else{
            tail_control((TAIL_ANGLE_STAND_UP-60)); /* �o�����X���s�p�p�x�ɐ��� */
        }
    }
    if(touritu_flg == 1) {
        end_flag = 1;
        return;
    }
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
	
	Distance_update(); /* �ړ��������Z */
	
	if( Distance_getDistance() > DISTANCE_NOTIFY )
	{
		/* DISTANCE_NOTIFY�ȏ�i�񂾂特���o�� */
		ev3_speaker_set_volume(100); 
		ev3_speaker_play_tone(NOTE_C4, 100);
	    grade_test_cnt++;
	    if(grade_test_cnt >= 1)
	    {
	        grade_test_flg  =1;


	    }
		/* �����v���ϐ������� */
		Distance_init();
	}
	
}
int garage_end(void){
    return end_flag;
}
/* end of file */
