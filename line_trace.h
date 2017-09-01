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
extern void line_tarce_stair(signed char light_white,signed char light_black);
extern signed char pid_control(uint8_t color_sensor_reflect, signed char light_white, signed char light_black);
extern void balanceControl(signed char forward, signed char turn);
extern void corrent_forword();
#ifdef __cplusplus
}
#endif

#define DEFAULT_SPEED 100

/* PIDパラメータ */
#define LKP 0.6
#define LKI 0.0
#define LKD 0.025

//正規化して算出したPID値に掛ける係数
#define KTURN 80

#define TARGET 0.65

/* カーブ検知パラメータ */
// TURN_MAX*TURN_PER_THRESHOLD ＜ TURN_THRESHOLDを超えるターンの値 ＝ カーブ
#define TURN_MAX 50 // ターン検知履歴数
#define TURN_THRESHOLD 20 // ターンの閾値（+-）閾値を超える数が
                          // TURN_MAXのうちTURN_PER_THRESHOLDの割合を超えると
                          // カーブとみなす。
#define TURN_PER_THRESHOLD 0.20 // カーブとみなす割合

/* カーブ検知中の走行パラメータ */
#define CURVE_SPEED 75
#define CURVE_TARGET_PLUS_OFFSET 0
#define CURVE_TARGET_MINUS_OFFSET 0
#define CURVE_KP 1.1
#define CURVE_KD 0.035

/* TARGET値によるTURNの補正 */
#define TURN_PLUS_CORRECT_EXP  turn * (target * 2)
#define TURN_MINUS_CORRECT_EXP turn * ((1 - target) * 2)



#endif /* _LINE_TRACE_H_ */

/* end of file */
