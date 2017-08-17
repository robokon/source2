/*
 * log.h
 */

#ifndef _LOG_H_
#define _LOG_H_

#ifdef __cplusplus
extern "C" {
#endif

#include "common.h"

extern void log_Str(uint8_t reflect, int16_t p, int16_t i, int16_t d);
extern void log_Commit();
extern void initialize_log(FILE *_bluetooth);
extern void close_log();
extern void log_task(intptr_t unused);
    
#ifdef __cplusplus
}
#endif


#endif /* _LOG_H_ */

/* end of file */
