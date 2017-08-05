#include "Distance.h"

#define TIRE_DIAMETER 81.0  /* �^�C�����a�i81mm�j*/
#define PI 3.14159265358    /* �~���� */

static float distance = 0.0;     //���s����
static float distance4msL = 0.0; //���^�C����4ms�Ԃ̋���
static float distance4msR = 0.0; //�E�^�C����4ms�Ԃ̋���
static float pre_angleL, pre_angleR; // ���E���[�^��]�p�x�̉ߋ��l

/* �������֐� */
void Distance_init()
{
    //�e�ϐ��̒l�̏�����
    distance = 0.0;
    distance4msR = 0.0;
    distance4msL = 0.0;
    //���[�^�p�x�̉ߋ��l�Ɍ��ݒl����
    pre_angleL = ev3_motor_get_counts(left_motor);
    pre_angleR = ev3_motor_get_counts(right_motor);
}

/* �����X�V�i4ms�Ԃ̈ړ������𖈉���Z���Ă���j */
void Distance_update()
{
    float cur_angleL = ev3_motor_get_counts(left_motor); //�����[�^��]�p�x�̌��ݒl
    float cur_angleR = ev3_motor_get_counts(right_motor);//�E���[�^��]�p�x�̌��ݒl
    float distance4ms = 0.0;        //4ms�̋���

    // 4ms�Ԃ̑��s���� = ((�~���� * �^�C���̒��a) / 360) * (���[�^�p�x�ߋ��l�@- ���[�^�p�x���ݒl)
    distance4msL = ((PI * TIRE_DIAMETER) / 360.0) * (cur_angleL - pre_angleL);  // 4ms�Ԃ̍����[�^����
    distance4msR = ((PI * TIRE_DIAMETER) / 360.0) * (cur_angleR - pre_angleR);  // 4ms�Ԃ̉E���[�^����
    distance4ms = (distance4msL + distance4msR) / 2.0; //���E�^�C���̑��s�����𑫂��Ċ���
    distance += distance4ms;

    //���[�^�̉�]�p�x�̉ߋ��l���X�V
    pre_angleL = cur_angleL;
    pre_angleR = cur_angleR;
}

/* ���s�������擾 */
float Distance_getDistance()
{
    return distance;
}

/* �E�^�C����4ms�Ԃ̋������擾 */
float Distance_getDistance4msRight()
{
    return distance4msR;
}

/* ���^�C����4ms�Ԃ̋������擾 */
float Distance_getDistance4msLeft()
{
    return distance4msL;
}

/* end of file */
