#ifndef _DISTANCE_H_
#define _DISTANCE_H_

#ifdef __cplusplus
extern "C" {
#endif

#include "common.h"
#include "ev3api.h"

/* ú»Ö */
void Distance_init();

/* £ðXV */
void Distance_update();

/* s£ðæ¾ */
float Distance_getDistance();

/* E^CÌ4msÔÌ£ðæ¾ */
float Distance_getDistance4msRight();

/* ¶^CÌ4msÔÌ£ðæ¾ */
float Distance_getDistance4msLeft();

#ifdef __cplusplus
}
#endif

#endif /* _LINE_TRACE_H_ */

/* end of file */
