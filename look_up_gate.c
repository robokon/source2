/*
 * look_up_gate.c
 */

#include "look_up_gate.h"
#include "Distance.h"

#define LOOK_UP_GATE_PASSING_ANGLE  90  /* ���b�N�A�b�v�Q�[�g�ʉߎ��̊p�x */
#define SLEEP_TIME                  1000 /* �b��ł���X���[�v����̎��� */
#define FORWARD_DISTANCE            30   /* �O�i���� */
#define TAIL_ANGLE_LUG 68        /* LUG�K���p�x[�x] */
#define MOTOR_ANGLE   720        /* motor�̊p�x[�x] */
#define PWM_FALLDOWN   30        /* LUG�̏�̓|���̃p���[ */
#define PWM_AHEAD      30        /* LUG�̑O�i�E��ރp���[ */

volatile static LOOK_UP_GATE_STATUS  look_up_gate_status_    = LOOK_UP_GATE_STAT_UNKNOWN;    /* ���b�N�A�b�v�Q�[�g�̍U����� */

//*****************************************************************************
// �֐��� : look_up_gate_main
// ���� : 
// �Ԃ�l : �Ȃ�
// �T�v : 
//       
//*****************************************************************************
unsigned int look_up_gate_main(void)
{
    static unsigned int  rev_t          = 0;    // (�T�[�{���[�^��]�p�x)���ݒl_�K��
    static unsigned int  phase          = 0;    // ���
    static unsigned int  run_count_0    = 0;    // ���Ԍo�ߊĎ��J�E���^
    static unsigned int  run_count_1    = 0;    // ���Ԍo�ߊĎ��J�E���^
    static unsigned int  run_count_2    = 0;    // ���Ԍo�ߊĎ��J�E���^
    static unsigned int  run_count_3    = 0;    // ���Ԍo�ߊĎ��J�E���^
    static unsigned int  run_count_4    = 0;    // ���Ԍo�ߊĎ��J�E���^
    static unsigned int  run_count_5    = 0;    // ���Ԍo�ߊĎ��J�E���^
    static unsigned int  ret            = 0;    // �߂�l 
//    static unsigned int  flg            = 0;    // �ʉ߃t���O(��x�ʉ߂�����t���O�𗧂Ă�)

    /* �֐�����Ăяo�����A�����ă��b�N�A�b�v�Q�[�g�̏����J�n��ʒm */
    if(run_count_0 == 0) {
	    ev3_speaker_set_volume(100); 
	    ev3_speaker_play_tone(NOTE_E4, 100);
    }

    switch (phase) {

    // ��~����X(phase1)
    case 0:
		/*** �C���N�������g ***/
	    run_count_0++;

    	/* 0~2�b�� */
        if(run_count_0 < 1000)
        {
        	/* �O�i0�A�]��0�œ|���U��q���䂷�� */
            do_balance(0,0);

        	/* 0 */
        	if (0 <= run_count_0 && 125 > run_count_0) {
        		printf("1\r\n");
			    rev_t = LOOK_UP_GATE_PASSING_ANGLE -60;
			    look_up_gate_tail_control(rev_t);
        	}
        	else if (125 <= run_count_0 && 250 > run_count_0) {
			    rev_t = LOOK_UP_GATE_PASSING_ANGLE -40;
			    look_up_gate_tail_control(rev_t);
        	}
        	else if (250 <= run_count_0 && 500 > run_count_0) {
			    rev_t = LOOK_UP_GATE_PASSING_ANGLE -20;
			    look_up_gate_tail_control(rev_t);
        	}
        	else {
			    rev_t = LOOK_UP_GATE_PASSING_ANGLE -5;
			    look_up_gate_tail_control(rev_t);
        	}


            return (ret);
        }

    	/* 2~2.5�b�� */
        if(run_count_0 < 1125)
        {
            /* �ȉ��̊p�x�ɐK���𐧌� */
		    rev_t = LOOK_UP_GATE_PASSING_ANGLE -10;
		    look_up_gate_tail_control(rev_t);

            ev3_motor_set_power(left_motor, (int)15);
            ev3_motor_set_power(right_motor, (int)15);
        	/* �ԗւ����S��~������ */
//			ev3_motor_stop(left_motor, true);
//        	ev3_motor_stop(right_motor, true);
        	return (ret);
        }

    	/* 2.5�b�� */
        if(run_count_0 >= 1125)
        {
        	/* �ԗւ����S��~������ */
			ev3_motor_stop(left_motor, true);
        	ev3_motor_stop(right_motor, true);

            /* �ȉ��̊p�x�ɐK���𐧌� */
		    rev_t = LOOK_UP_GATE_PASSING_ANGLE -20;
		    look_up_gate_tail_control(rev_t);
        }

    	/* phase1�Ɉڍs */
    	phase++;

		break;

    // ��X���O�i����~
    case 1:
		/*** �C���N�������g ***/
    	run_count_1++;

    	/* phase 1�ڍs��A1~7�b */
        if(run_count_1 >= 0 && run_count_1 < 1500)
        {
		    ev3_motor_set_power(left_motor, (int)10);
		    ev3_motor_set_power(right_motor, (int)10);
		    rev_t = LOOK_UP_GATE_PASSING_ANGLE -20;
		    look_up_gate_tail_control(rev_t);
        }
    	/* phase 1�ڍs��A5�b�� */
        else if(run_count_1 >= 1500 && run_count_1 < 1750)
        {
        	/* �ԗւ����S��~������ */
			ev3_motor_stop(left_motor, true);
        	ev3_motor_stop(right_motor, true);
		    rev_t = LOOK_UP_GATE_PASSING_ANGLE -20;
		    look_up_gate_tail_control(rev_t);
        }
    	else {
		    rev_t = LOOK_UP_GATE_PASSING_ANGLE -20;
		    look_up_gate_tail_control(rev_t);

			/* phase2�Ɉڍs */
    		phase = 2;
    	}


        break;

    // ���
    case 2:
		/*** �C���N�������g ***/
    	run_count_2++;

    	/* phase 2�ڍs��A1~7�b */
        if(run_count_2 >= 0 && run_count_2 < 1500)
        {
		    ev3_motor_set_power(left_motor, (int)-10);
		    ev3_motor_set_power(right_motor, (int)-10);
		    rev_t = LOOK_UP_GATE_PASSING_ANGLE -20;
		    look_up_gate_tail_control(rev_t);
        }
    	/* phase 2�ڍs��A7�b�� */
        else if(run_count_2 >= 1500 && run_count_2 < 1750)
        {
        	/* �ԗւ����S��~������ */
			ev3_motor_stop(left_motor, true);
        	ev3_motor_stop(right_motor, true);
		    rev_t = LOOK_UP_GATE_PASSING_ANGLE -20;
		    look_up_gate_tail_control(rev_t);
        }
    	else {
		    rev_t = LOOK_UP_GATE_PASSING_ANGLE -20;
		    look_up_gate_tail_control(rev_t);

			/* phase3�Ɉڍs */
    		phase = 3;
    	}

    	break;

    // �O�i����~
    case 3:
		/*** �C���N�������g ***/
    	run_count_3++;

    	/* phase 3�ڍs��A1~7�b */
        if(run_count_3 >= 0 && run_count_3 < 1500)
        {
        	printf("1\r\n");
		    ev3_motor_set_power(left_motor, (int)10);
		    ev3_motor_set_power(right_motor, (int)10);
		    rev_t = LOOK_UP_GATE_PASSING_ANGLE -20;
		    look_up_gate_tail_control(rev_t);
        }
    	/* phase 3�ڍs��A7�b�� */
        else if(run_count_3 >= 1500 && run_count_3 < 1750)
        {
        	/* �ԗւ����S��~������ */
			ev3_motor_stop(left_motor, true);
        	ev3_motor_stop(right_motor, true);
		    rev_t = LOOK_UP_GATE_PASSING_ANGLE -20;
		    look_up_gate_tail_control(rev_t);
        	printf("2\r\n");
        }
    	else {
        	printf("3\r\n");
		    rev_t = LOOK_UP_GATE_PASSING_ANGLE -20;
		    look_up_gate_tail_control(rev_t);

	    	/* phase4�Ɉڍs */
	    	phase = 4;
    	}

        break;

    // ����
    case 4:
        /*** �C���N�������g ***/
        run_count_4++;

        /* �K���̃��[�^�p�x���擾 */
        rev_t = ev3_motor_get_counts(tail_motor);
        
    	if (0 <= run_count_4 && 90 > run_count_4) {
    		printf("case 4\r\n");
		    ev3_motor_set_power(tail_motor, (int)23);
		    ev3_motor_set_power(left_motor, (int)-10);
		    ev3_motor_set_power(right_motor, (int)-10);
    	}
        else if (90 <= run_count_4 && 500 > run_count_4){
		    look_up_gate_tail_control(90);
            
//            ev3_motor_stop(tail_motor, true);
            ev3_motor_stop(left_motor, true);
            ev3_motor_stop(right_motor, true);
        }
        else if (500 <= run_count_4 && 750 > run_count_4) {
		    look_up_gate_tail_control(94);
            
            ev3_motor_stop(left_motor, true);
            ev3_motor_stop(right_motor, true);
        }
        else if (750 <= run_count_4 && 1000 > run_count_4) {
		    look_up_gate_tail_control(96);
            
            ev3_motor_stop(left_motor, true);
            ev3_motor_stop(right_motor, true);
        }
        else if (1000 <= run_count_4 && 1250 > run_count_4) {
		    look_up_gate_tail_control(97);
            
            ev3_motor_stop(left_motor, false);
            ev3_motor_stop(right_motor, false);
        }
#if 0
        else if (1250 <= run_count_4 && 1500 > run_count_4) {
		    look_up_gate_tail_control(98);
            
            ev3_motor_stop(left_motor, false);
            ev3_motor_stop(right_motor, false);
        }
        else if (1500 <= run_count_4 && 1750 > run_count_4) {
		    look_up_gate_tail_control(99);
            
            ev3_motor_stop(left_motor, false);
            ev3_motor_stop(right_motor, false);

            /* �W���C���Z���T�[���Z�b�g */
            ev3_gyro_sensor_reset(gyro_sensor);
            balance_init(); /* �|���U�qAPI������ */            
        }
#endif
        else {
		    look_up_gate_tail_control(97);

            ev3_motor_stop(left_motor, false);
            ev3_motor_stop(right_motor, false);
	    	/* phase5�Ɉڍs */
	    	phase = 5;
        }

        break;
    case 5:
/* �{���Ȃ璼����A�|���U�q����𕜊����� */
/* �K���[�W�Ɉڍs�����邽�߁A�{�֐����Ă΂Ȃ��悤�ɂ��ׂ����� */
/* �|���U�q������o���Ȃ����� */
/* �����ŃK���[�WIN������ */
#if 0
        if (flg == 0) {
            ret = 1;
            flg = 1;
        }
#endif
        /*** �C���N�������g ***/
        run_count_5++;
        
        /* 8�b */
    	if (0 <= run_count_5 && 2000 > run_count_5) {
		    look_up_gate_tail_control(90);
		    ev3_motor_set_power(left_motor, (int)20);
		    ev3_motor_set_power(right_motor, (int)20);
    	}
    	else if (2000 <= run_count_5 && 3000 > run_count_5) {
		    look_up_gate_tail_control(80);
		    ev3_motor_set_power(left_motor, (int)10);
		    ev3_motor_set_power(right_motor, (int)10);
    	}
    	else if (3000 <= run_count_5 && 3500 > run_count_5) {
		    look_up_gate_tail_control(80);
		    ev3_motor_set_power(left_motor, (int)5);
		    ev3_motor_set_power(right_motor, (int)5);
    	}
        else {
		    look_up_gate_tail_control(80);
            ev3_motor_stop(left_motor, true);
            ev3_motor_stop(right_motor, true);
        }
        
        break;

    }

    /* �߂�{�^��or�]�񂾂�I�� */
    if(ev3_button_is_pressed(BACK_BUTTON))
    {
        wup_tsk(MAIN_TASK);
    }

        return (ret);

}

//*****************************************************************************
// �֐��� : do_balance
// ���� : ���{������A���x�A��]
// �Ԃ�l : �Ȃ�
// �T�v : �^����ꂽ����p���āA�|���U�q��������{���A���E���[�^�֔��f����B
//*****************************************************************************
void do_balance( signed char forward, signed char turn)
{
	signed char pwm_L = 0, pwm_R = 0; /* ���E���[�^PWM�o�� */
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
    
    if(gyro < -150 || 150 < gyro)
    {
        wup_tsk(MAIN_TASK);
    }

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

