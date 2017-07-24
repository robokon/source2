/*
 * look_up_gate.h
 */

#ifndef _LOOK_UP_GATE_H_
#define _LOOK_UP_GATE_H_

#ifdef __cplusplus
extern "C" {
#endif

#include "common.h"

/* ルックアップゲートデバッグ制御用マクロ */
#define LOOK_UP_GATE_DEBUG

/* ルックアップゲート攻略開始距離 */
#define LOOK_UP_GATE_DISTANCE 5

/**
 * ルックアップゲート用状態定義
 *
 * 例外を除く以下の3状態でゲート通過を行う。
 *
 *
 * ・ゲート通過前処理状態
 * 超音波センサーでルックアップゲートを検知してから、
 * ゲート通過可能状態に走行体を傾けるまでの状態。
 * 
 * ・ゲート通過処理状態
 * 上記状態から走行体がゲートを通過するまでの状態。
 * ボーナス獲得する為に、バックで走行した場合、
 * 再度ゲートを通過するまでの状態も含める
 * 
 * ・ゲート通過後処理状態
 * ゲート通過後、ライントレースに戻る為に、
 * 走行体を元に戻し、ライントレースを開始するまで。
 * 
 * ・エラー状態(例外)
 * 走行体の傾け過ぎて転倒しかけたり、ゲートを倒したなど、
 * ゲート攻略が不可能・失敗した状態。
 * 転倒したり、コースを逸れないように、
 * ゲート攻略を強制的に終了し、ライントレースに戻る。
 * 
 */
typedef enum{
    LOOK_UP_GATE_STAT_UNKNOWN = 0,   /* 無効値 */
    LOOK_UP_GATE_STAT_PREPARE,       /* ゲート通過前処理状態 */
    LOOK_UP_GATE_STAT_PROCESSING,    /* ゲート通過処理状態 */
    LOOK_UP_GATE_STAT_FINISH,        /* ゲート通過後処理状態 */
    LOOK_UP_GATE_STAT_ERROR,         /* エラー状態(例外) */
} LOOK_UP_GATE_STATUS;

/*** 外部関数 ***/
/* メイン関数 */
extern void look_up_gate_main();
extern signed int look_up_gate_get_distance();


/*** 内部関数の予定(現状、外部からも使用可能) ***/
/* 超音波センサによる計測 */
int look_up_gate_sonar_distance(void);

/* 走行体完全停止用モータの角度制御 */
int look_up_gate_tail_control(signed int angle);

/* 走行体ゲート通過時の走行 */
void look_up_gate_gate_passing(unsigned int direction);


#ifdef __cplusplus
}
#endif


#endif /* _LOOK_UP_GATE_H_ */

/* end of file */
