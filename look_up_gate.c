/*
 * look_up_gate.c
 */

#include "look_up_gate.h"
#include "Distance.h"

#define LOOK_UP_GATE_PASSING_ANGLE  TAIL_ANGLE_STAND_UP - 15  /* ���b�N�A�b�v�Q�[�g�ʉߎ��̊p�x */
#define SLEEP_TIME                  1000 /* �b��ł���X���[�v����̎��� */
#define FORWARD_DISTANCE            30   /* �O�i���� */

volatile static LOOK_UP_GATE_STATUS  look_up_gate_status_    = LOOK_UP_GATE_STAT_UNKNOWN;    /* ���b�N�A�b�v�Q�[�g�̍U����� */
static int                  tail_angle_             = 0;                            /* �p�x */
static float                process_start_location_ = 0;                            /* �Q�[�g�ʉߊJ�n�ʒu */
static int                  motor_stop              = 0;

//*****************************************************************************
// �֐��� : look_up_gate_main
// ���� : 
// �Ԃ�l : �Ȃ�
// �T�v : 
//       
//*****************************************************************************
void look_up_gate_main(void)
{
    /* ���[�J���ϐ��̒�`�E������ */
	signed char forward = 0;      /* �O��i���� */
	signed char turn    = 0;         /* ���񖽗� */
	signed char pwm_L, pwm_R; /* ���E���[�^PWM�o�� */

	static int 				tail_status 			= 0;
	static unsigned int		is_balance_control_     = true;	/* �|���U��q����L��(true: �L���Cfalse: ����) */ 

	/* ��0�ŏ���������ƐU��q���䂪��肭�����Ȃ��Ȃ�E�E�E */
    int32_t motor_ang_l, motor_ang_r;
    int gyro, volt;

    /* �{�֐����񎞂̏��� */
    if (look_up_gate_status_ == LOOK_UP_GATE_STAT_UNKNOWN) {

        /* �Q�[�g���m�����Ŏ��� */
        ev3_speaker_set_volume(100); 
        ev3_speaker_play_tone(NOTE_C4, 100);

        /* ��Q�������m�����̂ő��x���� */
        forward = turn = 0;      
        /* ���̏�ԂɑJ�� */
        look_up_gate_status_ = LOOK_UP_GATE_STAT_PREPARE;        
    }

    /* �U����Ԃ𔻒肷�� */
    switch (look_up_gate_status_) {

    /* �Q�[�g�ʉߑO������� */
    case LOOK_UP_GATE_STAT_PREPARE:

//	        ev3_speaker_set_volume(100); 
//          ev3_speaker_play_tone(NOTE_D4, 100);  
        
            /* �X�^�[�g���̓|���p�x����15�x�ɏグ�� */
            tail_angle_ = LOOK_UP_GATE_PASSING_ANGLE;

			motor_stop = look_up_gate_tail_control(tail_angle_);
		if (1 == motor_stop
				&& 0 == tail_status) {
			tslp_tsk(10);

	        ev3_speaker_set_volume(100); 
            ev3_speaker_play_tone(NOTE_D4, 100);
		    tail_status = 1;
		    motor_stop = 0;
		}

		if (1 == tail_status) {
            /* �Q�[�g�ʉߊp�x�ɐݒ� */
            tail_angle_ = LOOK_UP_GATE_PASSING_ANGLE - 5;

			motor_stop = look_up_gate_tail_control(tail_angle_);

			if (1 == motor_stop
					&& 1 == tail_status) { 
				tslp_tsk(10); 
                /* 1�b��~ */
        		static int stop_cycle = 0;
                stop_cycle++;
                if (stop_cycle >= 250) {
                    /* ���̏�ԂɑJ�� */
                    tail_status = 2;
                }
            }
		}

        /* �ȉ��̏����ŏd�S������ɌX����悤�ɂ��� */
		if (2 == tail_status) {
            /* �Q�[�g�ʉߊp�x�ɐݒ� */
            tail_angle_ = LOOK_UP_GATE_PASSING_ANGLE - 5;
			motor_stop = look_up_gate_tail_control(tail_angle_);

		    static int balance_back = 0;
		    /* ���� */
		    if (0 == balance_back) {
		        /* ��i���� */
		        forward = -5;
                ev3_motor_set_power(left_motor, (int)pwm_L);
                ev3_motor_set_power(right_motor, (int)pwm_R);
		        /* �|���U��q�𖳌��� */
				is_balance_control_ = false; 
		        balance_back = 1;
		    }
            else {
                ev3_motor_stop(left_motor, true);
                ev3_motor_stop(right_motor, true);
            }
        }

        break;

    /* �Q�[�g�ʉߏ������ */
    case LOOK_UP_GATE_STAT_PROCESSING:
#if 0
        /* �������X�V */
        Distance_update();

        /* �O�i */
        if ((process_start_location_ + FORWARD_DISTANCE) > Distance_getDistance())
//            look_up_gate_gate_passing(0);

        /* ��i */
        /* TODO ��i����Ƌ��������Z����邩�s���ׁ̈C�������� */
        if ((process_start_location_ + FORWARD_DISTANCE) < Distance_getDistance())
//            look_up_gate_gate_passing(1);

        /* ��~ */
        if ((process_start_location_ + FORWARD_DISTANCE) == Distance_getDistance()) {
            look_up_gate_gate_passing(2);
            tslp_tsk(1000 * 10);
        }
#endif   
        break;

    /* �Q�[�g�ʉߌ㏈����� */
    case LOOK_UP_GATE_STAT_FINISH:
        /* T.B.D. */
        break;

    /* �G���[���(��O) */
    case LOOK_UP_GATE_STAT_ERROR:
    default:
        /* �ʏ�̃��C���g���[�X��Ԃɖ߂� */
        /* T.B.D. */
        break;
    }

    /* �|���U�q����API �ɓn���p�����[�^���擾���� */
    motor_ang_l = ev3_motor_get_counts(left_motor);
    motor_ang_r = ev3_motor_get_counts(right_motor);
    gyro = ev3_gyro_sensor_get_rate(gyro_sensor);
    volt = ev3_battery_voltage_mV();

    /* �|���U��q����L���̔��� */
    if (true == is_balance_control_) {
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
// �֐��� : look_up_gate_get_distance
// ���� : �Ȃ�
// �Ԃ�l : ��Q���܂ł̋���(cm)
// �T�v : �X�^�[�g�n�_����̋����𑪒�
//        ��Q���܂ł̋������擾����
//*****************************************************************************
signed int look_up_gate_get_distance(void)
{
    signed int distance = -1;

    /* �X�^�[�g����̋���������Ȃ����Ă݂� */
    /* �����g�Z���T�����Q���܂ł̋������擾���� */
    distance = ev3_ultrasonic_sensor_get_distance(sonar_sensor);
    
    return (distance);
}

//*****************************************************************************
// �֐��� : look_up_gate_tail_control
// ���� : angle (���[�^�ڕW�p�x[�x])
// �Ԃ�l : ����
// �T�v : ���s�̊��S��~�p���[�^�̊p�x����
//*****************************************************************************
int look_up_gate_tail_control(signed int angle)
{
    int motor_stop = 0;
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
        motor_stop = 1;
    }
    else
    {
        ev3_motor_set_power(tail_motor, (signed char)pwm);
        motor_stop = 0;
    }
    
    return    motor_stop;
}

//*****************************************************************************
// �֐��� : look_up_gate_sonar_distance
// ���� : ����
// �Ԃ�l : 1(��Q������)/0(��Q������)
// �T�v : �����g�Z���T�ɂ���Q�����m
//*****************************************************************************
int look_up_gate_sonar_distance(void)
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
        if ((distance <= LOOK_UP_GATE_DISTANCE) && (distance >= 0))
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
// �֐��� : look_up_gate_gate_passing
// ���� : direction(0:�O�i�A1:��i�A2:��~)
// �Ԃ�l : ����
// �T�v : �|���U�q���䖳������Ԃł̃��[�^����
//*****************************************************************************
void look_up_gate_gate_passing(unsigned int direction)
{
    /* ���E���[�^PWM�o�� */
    signed char local_pwm_L = 0, local_pwm_R = 0;

    switch (direction) {
    /* �O�i */
    case 0:
        local_pwm_L = 10, local_pwm_R = 10; 
        ev3_motor_set_power(left_motor, (int)local_pwm_L);
        ev3_motor_set_power(right_motor, (int)local_pwm_R);
        break;

    /* ��i */
    case 1:
        local_pwm_L = -10, local_pwm_R = -10;
        ev3_motor_set_power(left_motor, (int)local_pwm_L);
        ev3_motor_set_power(right_motor, (int)local_pwm_R);
        break;

    /* ��~ */
    case 2:
        ev3_motor_set_power(left_motor, (int)local_pwm_L);
        ev3_motor_set_power(right_motor, (int)local_pwm_R);
        break;

    /* ��O */
    default:
        /* T.B.D. */
        break;
    }
}

//*****************************************************************************
// �֐��� : balance_task
// ���� : unused
// �Ԃ�l : �Ȃ�
// �T�v : �ُ팟�m�^�X�N�B
//        ����A�]�|�ɂ��W���C���Z���T���ُ�l�̌��o�����ۂ̏������s��
//       
//*****************************************************************************
void balance_task(intptr_t unused)
{
    /* ���[�J���ϐ��̒�`�E������ */
	signed char local_forward = 0;      /* �O��i���� */
	signed char local_turn    = 0;         /* ���񖽗� */
	signed char local_pwm_L, local_pwm_R; /* ���E���[�^PWM�o�� */
    int32_t motor_ang_l, motor_ang_r;
    int gyro, volt;

//   ev3_speaker_set_volume(100); 
//    ev3_speaker_play_tone(NOTE_E4, 100);

	while (1) {
        /* �|���U�q����API �ɓn���p�����[�^���擾���� */
        motor_ang_l = ev3_motor_get_counts(left_motor);
        motor_ang_r = ev3_motor_get_counts(right_motor);
        gyro = ev3_gyro_sensor_get_rate(gyro_sensor);
        volt = ev3_battery_voltage_mV();

        /* �|���U�q����API���Ăяo���A�|�����s���邽�߂� */
        /* ���E���[�^�o�͒l�𓾂� */
        balance_control(
            (float)local_forward,
            (float)local_turn,
            (float)gyro,
            (float)GYRO_OFFSET,
            (float)motor_ang_l,
            (float)motor_ang_r,
            (float)volt,
            (signed char*)&local_pwm_L,
            (signed char*)&local_pwm_R);

        /* EV3�ł̓��[�^�[��~���̃u���[�L�ݒ肪���O�ɂł��Ȃ����� */
        /* �o��0���ɁA���̓s�x�ݒ肷�� */
        if (local_pwm_L == 0)
        {
             ev3_motor_stop(left_motor, true);
        }
        else
        {
            ev3_motor_set_power(left_motor, (int)local_pwm_L);
        }
        
        if (local_pwm_R == 0)
        {
             ev3_motor_stop(right_motor, true);
        }
        else
        {
            ev3_motor_set_power(right_motor, (int)local_pwm_R);
        }

	    tslp_tsk(4);
	}
}

