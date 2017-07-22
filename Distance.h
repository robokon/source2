#ifndef _DISTANCE_H_
#define _DISTANCE_H_

#ifdef __cplusplus
extern "C" {
#endif

#include "common.h"
#include "ev3api.h"

/* �������֐� */
void Distance_init();

/* �������X�V */
void Distance_update();

/* ���s�������擾 */
float Distance_getDistance();

/* �E�^�C����4ms�Ԃ̋������擾 */
float Distance_getDistance4msRight();

/* ���^�C����4ms�Ԃ̋������擾 */
float Distance_getDistance4msLeft();

#ifdef __cplusplus
}
#endif

#endif /* _LINE_TRACE_H_ */

/* end of file */
