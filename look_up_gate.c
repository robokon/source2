/*
 * look_up_gate.c
 */

#include "look_up_gate.h"

#define LOOK_UP_GATE_PASSING_ANGLE  115  /* ���b�N�A�b�v�Q�[�g�ʉߎ��̊p�x */
#define SLEEP_TIME                  1000 /* �b��ł���X���[�v����̎��� */
#define FORWARD_DISTANCE            30   /* �O�i���� */
#define BACKWARD_DISTANCE           -30  /* ��i���� */
#define START_DISTANCE              200

signed char forward;      /* �O��i���� */
signed char turn;         /* ���񖽗� */
signed char pwm_L, pwm_R; /* ���E���[�^PWM�o�� */

static LOOK_UP_GATE_STATUS  look_up_gate_status_    = LOOK_UP_GATE_STAT_UNKNOWN;    /* ���b�N�A�b�v�Q�[�g�̍U����� */
static unsigned int         is_balance_control_     = true;                         /* �|���U��q����L��(true: �L���Cfalse: ����) */ 
static int                  tail_angle_             = 0;                            /* �p�x */
static float                process_start_location_ = 0;                            /* �Q�[�g�ʉߊJ�n�ʒu */

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
    int32_t motor_ang_l, motor_ang_r    = 0;
    int gyro, volt                      = 0;

    /* ��Q�������m���� */
    if (look_up_gate_sonar_distance()) {
        /* �U���J�n�ɑJ�� */
        look_up_gate_status_ = LOOK_UP_GATE_STAT_PREPARE;
    }
    /* ��Q�������m���Ȃ��ꍇ�C�������I������ */
    else {
        return;
    }

    /* �U����Ԃ𔻒肷�� */
    switch (look_up_gate_status_) {

    /* �Q�[�g�ʉߑO������� */
    case LOOK_UP_GATE_STAT_PREPARE:
        /* ���b�N�A�b�v�Q�[�g���U�����邽�߁C���S��~�p�p�x�Ƀ��[�^���� */
        /* ���S��~�p�x�ɂȂ�܂Ń��[�^�𐧌䂷�� */
        if (TAIL_ANGLE_STAND_UP >= tail_angle_) {
            /* ��4msec�����N���Ȃ̂Ń��[�^���䂪�������ď�肭�����Ȃ��Ǝv���̂ŁC�b���200ms�̃X���[�v�����ɂ₩�ɂ��� */
            tslp_tsk(SLEEP_TIME);
            look_up_gate_tail_control(tail_angle_++);
        }
        /* ���S��~�p�x�ɂȂ��� */
        else {
            /* �|���U��q���䂪�L�� */
            if (is_balance_control_) {
                tail_angle_ = TAIL_ANGLE_STAND_UP;
                /* �|���U��q����͖����ɂ��� */
                is_balance_control_ = false;
            }
            else {
                /* �@�̂��Q�[�g�ʉ߉\�Ȋp�x������ */
                if (LOOK_UP_GATE_PASSING_ANGLE >= tail_angle_) {
                    /* ��4msec�����N���Ȃ̂Ń��[�^���䂪�������ď�肭�����Ȃ��Ǝv���̂ŁC�b���200ms�̃X���[�v�����ɂ₩�ɂ��� */
                    tslp_tsk(SLEEP_TIME);
                    look_up_gate_tail_control(tail_angle_++);
                }
                else {
                    /* �Q�[�g�ʉ߉\�Ȋp�x�ł���ׁC��Ԃ��Q�[�g�ʉߒ��Ɉڍs���� */
                    look_up_gate_status_ = LOOK_UP_GATE_STAT_PROCESSING;

                    /* �Q�[�g�ʉߏ����J�n���̈ʒu���擾���� */
                    /* ��苗���O�i������A��i�����A�擾�����ʒu�܂Ŗ߂�� */
                    process_start_location_ = Distance_getDistance();
                }
            }
        }

        break;

    /* �Q�[�g�ʉߏ������ */
    case LOOK_UP_GATE_STAT_PROCESSING:
        /* �O�i */
        if ((process_start_location_ + FORWARD_DISTANCE) > Distance_getDistance())
            look_up_gate_gate_passing(1);
        /* ��i */
        /* TODO ��i����Ƌ��������Z����邩�s���ׁ̈C�������� */
//        if ((process_start_location_ + FORWARD_DISTANCE) < Distance_getDistance())
            
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

    /* �|���U��q����L���̔��� */
    if (is_balance_control_) {
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
    
    /* ���s�����X�V */
    Distance_update();
    /* �X�^�[�g�n�_����̋�����2m�ȏ� */
    if (Distance_getDistance() >= START_DISTANCE) {
        /* �����g�Z���T�����Q���܂ł̋������擾���� */
        distance = ev3_ultrasonic_sensor_get_distance(sonar_sensor);
    }
    
    return (distance);
}

//*****************************************************************************
// �֐��� : look_up_gate_tail_control
// ���� : angle (���[�^�ڕW�p�x[�x])
// �Ԃ�l : ����
// �T�v : ���s�̊��S��~�p���[�^�̊p�x����
//*****************************************************************************
void look_up_gate_tail_control(signed int angle)
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
// �֐��� : look_up_gate_gate_passing
// ���� : ����
// �Ԃ�l : ����
// �T�v : 
//*****************************************************************************
void look_up_gate_gate_passing(unsigned int direction)
{
    signed char local_pwm_L = 30, local_pwm_R = 30; /* ���E���[�^PWM�o�� */

    switch (direction) {
    /* �O�i */
    case 0:
        ev3_motor_set_power(left_motor, (int)local_pwm_L);
        ev3_motor_set_power(right_motor, (int)local_pwm_R);

        break;
    /* ��i */
    case 1:
        /* T.B.D. */
    default:
        /* T.B.D. */
        break;
    }
}

/* end of file */
