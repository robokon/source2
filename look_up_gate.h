/*
 * look_up_gate.h
 */

#ifndef _LOOK_UP_GATE_H_
#define _LOOK_UP_GATE_H_

#ifdef __cplusplus
extern "C" {
#endif

#include "common.h"

/* ���b�N�A�b�v�Q�[�g�f�o�b�O����p�}�N�� */
#define LOOK_UP_GATE_DEBUG

/* ���b�N�A�b�v�Q�[�g�U���J�n���� */
#define LOOK_UP_GATE_DISTANCE 5

/**
 * ���b�N�A�b�v�Q�[�g�p��Ԓ�`
 *
 * ��O�������ȉ���3��ԂŃQ�[�g�ʉ߂��s���B
 *
 *
 * �E�Q�[�g�ʉߑO�������
 * �����g�Z���T�[�Ń��b�N�A�b�v�Q�[�g�����m���Ă���A
 * �Q�[�g�ʉ߉\��Ԃɑ��s�̂��X����܂ł̏�ԁB
 * 
 * �E�Q�[�g�ʉߏ������
 * ��L��Ԃ��瑖�s�̂��Q�[�g��ʉ߂���܂ł̏�ԁB
 * �{�[�i�X�l������ׂɁA�o�b�N�ő��s�����ꍇ�A
 * �ēx�Q�[�g��ʉ߂���܂ł̏�Ԃ��܂߂�
 * 
 * �E�Q�[�g�ʉߌ㏈�����
 * �Q�[�g�ʉߌ�A���C���g���[�X�ɖ߂�ׂɁA
 * ���s�̂����ɖ߂��A���C���g���[�X���J�n����܂ŁB
 * 
 * �E�G���[���(��O)
 * ���s�̂̌X���߂��ē]�|����������A�Q�[�g��|�����ȂǁA
 * �Q�[�g�U�����s�\�E���s������ԁB
 * �]�|������A�R�[�X�����Ȃ��悤�ɁA
 * �Q�[�g�U���������I�ɏI�����A���C���g���[�X�ɖ߂�B
 * 
 */
typedef enum{
    LOOK_UP_GATE_STAT_UNKNOWN = 0,   /* �����l */
    LOOK_UP_GATE_STAT_PREPARE,       /* �Q�[�g�ʉߑO������� */
    LOOK_UP_GATE_STAT_PROCESSING,    /* �Q�[�g�ʉߏ������ */
    LOOK_UP_GATE_STAT_FINISH,        /* �Q�[�g�ʉߌ㏈����� */
    LOOK_UP_GATE_STAT_ERROR,         /* �G���[���(��O) */
} LOOK_UP_GATE_STATUS;

/*** �O���֐� ***/
/* ���C���֐� */
extern void look_up_gate_main();
extern signed int look_up_gate_get_distance();


/*** �����֐��̗\��(����A�O��������g�p�\) ***/
/* �����g�Z���T�ɂ��v�� */
int look_up_gate_sonar_distance(void);

/* ���s�̊��S��~�p���[�^�̊p�x���� */
int look_up_gate_tail_control(signed int angle);

/* ���s�̃Q�[�g�ʉߎ��̑��s */
void look_up_gate_gate_passing(unsigned int direction);


#ifdef __cplusplus
}
#endif


#endif /* _LOOK_UP_GATE_H_ */

/* end of file */
