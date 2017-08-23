/*
 * line_tarce.h
 * 
 */
#ifndef _LINE_TRACE_H_
#define _LINE_TRACE_H_

#ifdef __cplusplus
extern "C" {
#endif

#include "common.h"
#include "Distance.h"
#include "log.h"
extern void line_tarce_main(signed char light_white,signed char light_black);
extern signed char pid_control(uint8_t color_sensor_reflect, signed char light_white, signed char light_black);
extern void balanceControl(signed char forward, signed char turn);
#ifdef __cplusplus
}
#endif

#define DEFAULT_SPEED 100

/* PID�p�����[�^ */
#define KP 1.2
#define KI 0.0
#define KD 0.025

//���K�����ĎZ�o����PID�l�Ɋ|����W��
#define KTURN 100

#define TARGET 0.5

/* �J�[�u���m�p�����[�^ */
#define TURN_MAX 100
#define TURN_THRESHOLD 20
#define TURN_PER_THRESHOLD 0.25

/* �J�[�u���m���̑��s�p�����[�^ */
#define CURVE_SPEED DEFAULT_SPEED
#define CURVE_TARGET_OFFSET 0
#define CURVE_KP KP
#define CURVE_KD KD

/* TARGET�l�ɂ��TURN�̕␳ */
#define TURN_PLUS_CORRECT_EXP  turn * (target * 2)
#define TURN_MINUS_CORRECT_EXP turn * ((1 - target) * 2)

//�錾�����ꍇ�A�����t�@�C���Ƀ��O���o��
//#define _LOG_OUTPUT_FILE
//�錾�����ꍇ�ABLUETOOTH�Ƀ��O���o��
#define _LOG_OUTPUT_BLUETOOTH
//�錾�����ꍇ�����t�@�C���ɏo�͂��郍�O�̖��O�������I�ɕύX����(���Ȃ��ꍇdefault.csv�ɏo��)
//#define _LOG_RENAME_FILE_NAME


#endif /* _LINE_TRACE_H_ */

/* end of file */
