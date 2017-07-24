/*
 * common.h
 * 
 * ���ʒ�`�t�@�C��
 *
 */

#ifndef _COMMON_H_
#define _COMMON_H_

#ifdef __cplusplus
extern "C" {
#endif

#include "ev3api.h"
#include "balancer.h"

/**
 * �߂�l��`
 */
typedef enum{
    RET_OK = 0,
    RET_ERROR
} RETCODE ;

/**
 * ��Ԓ�`
 */
typedef enum{
    STAT_UNKNOWN = 0,
    STAT_NORMAL,
    STAT_STAIR,
    STAT_LOOK_UP_GATE,
    STAT_GAREGE,
} STATUS ;

/**
 * �Z���T�[�A���[�^�[�̐ڑ����`���܂�
 */
static const sensor_port_t
    touch_sensor    = EV3_PORT_1,
    sonar_sensor    = EV3_PORT_2,
    color_sensor    = EV3_PORT_3,
    gyro_sensor     = EV3_PORT_4;

static const motor_port_t
    left_motor      = EV3_PORT_C,
    right_motor     = EV3_PORT_B,
    tail_motor      = EV3_PORT_A;

/* ���L�̃}�N���͌�/���ɍ��킹�ĕύX����K�v������܂� */
/* sample_c1�}�N�� */
#define GYRO_OFFSET  0          /* �W���C���Z���T�I�t�Z�b�g�l(�p���x0[deg/sec]��) */
/* sample_c2�}�N�� */
#define SONAR_ALERT_DISTANCE 30 /* �����g�Z���T�ɂ���Q�����m����[cm] */
/* sample_c3�}�N�� */
#define TAIL_ANGLE_STAND_UP  85 /* ���S��~���̊p�x[�x] */
#define TAIL_ANGLE_START     96 /* �X�^�[�g���̊p�x[�x] */
#define TAIL_ANGLE_DRIVE      3 /* �o�����X���s���̊p�x[�x] */
#define P_GAIN             2.5F /* ���S��~�p���[�^������W�� */
#define PWM_ABS_MAX          60 /* ���S��~�p���[�^����PWM��΍ő�l */
#define DISTANCE_NOTIFY (10000.0) /* �����鑖�s�����@�P�ʁF�~�����[�g�� */

/* sample_c4�}�N�� */
//#define DEVICE_NAME     "ET0"  /* Bluetooth�� hrp2/target/ev3.h BLUETOOTH_LOCAL_NAME�Őݒ� */
//#define PASS_KEY        "1234" /* �p�X�L�[    hrp2/target/ev3.h BLUETOOTH_PIN_CODE�Őݒ� */
#define CMD_START         '1'    /* �����[�g�X�^�[�g�R�}���h */

/* LCD�t�H���g�T�C�Y */
#define CALIB_FONT (EV3_FONT_SMALL)
#define CALIB_FONT_WIDTH (6/*TODO: magic number*/)
#define CALIB_FONT_HEIGHT (8/*TODO: magic number*/)

/* �e������� */
extern STATUS main_status;

/* �֐��v���g�^�C�v�錾 */
extern float tail_control(signed int angle);
extern int sonar_alert(void);

#ifdef __cplusplus
}
#endif

#endif /* _COMMON_H_ */

/* end of file */
