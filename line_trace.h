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
extern void line_tarce_main(signed char color_sensor);
extern signed char pid_control(uint8_t color_sensor_reflect, signed char target_value);
extern void balanceControl(signed char forward, signed char turn);
#ifdef __cplusplus
}
#endif

#endif /* _LINE_TRACE_H_ */

/* end of file */
