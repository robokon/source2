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
extern void corrent_forword();
#ifdef __cplusplus
}
#endif

#define DEFAULT_SPEED 100

/* PIDパラメータ */
#define KP 1.0
#define KI 0.0
#define KD 0.035

//正規化して算出したPID値に掛ける係数
#define KTURN 100

#define TARGET 0.6

/* カーブ検知パラメータ */
// TURN_MAX*TURN_PER_THRESHOLD ＜ TURN_THRESHOLDを超えるターンの値 ＝ カーブ
#define TURN_MAX 100 // ターン検知履歴数
#define TURN_THRESHOLD 20 // ターンの閾値（+-）閾値を超える数が
                          // TURN_MAXのうちTURN_PER_THRESHOLDの割合を超えると
                          // カーブとみなす。
#define TURN_PER_THRESHOLD 0.25 // カーブとみなす割合

/* カーブ検知中の走行パラメータ */
#define CURVE_SPEED 80
#define CURVE_TARGET_OFFSET 0
#define CURVE_KP KP
#define CURVE_KD KD

/* TARGET値によるTURNの補正 */
#define TURN_PLUS_CORRECT_EXP  turn * (target * 2)
#define TURN_MINUS_CORRECT_EXP turn * ((1 - target) * 2)

//宣言した場合、内部ファイルにログを出力
#define _LOG_OUTPUT_FILE
//宣言した場合、BLUETOOTHにログを出力
#define _LOG_OUTPUT_BLUETOOTH
//宣言した場合内部ファイルに出力するログの名前を自動的に変更する(しない場合default.csvに出力)
#define _LOG_RENAME_FILE_NAME


#endif /* _LINE_TRACE_H_ */

/* end of file */
