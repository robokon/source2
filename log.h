/*
 * log.h
 */

#ifndef _LOG_H_
#define _LOG_H_

#ifdef __cplusplus
extern "C" {
#endif

#include "common.h"

extern void log_Str(uint8_t reflect, float normalize_reflect, float p, float d, float turn);
extern void initialize_log(FILE *_bluetooth);
extern void close_log();
extern void log_task(intptr_t unused);

//宣言した場合、内部ファイルにログを出力
#define _LOG_OUTPUT_FILE
//宣言した場合、BLUETOOTHにログを出力
#define _LOG_OUTPUT_BLUETOOTH
//宣言した場合内部ファイルに出力するログの名前を自動的に変更する(しない場合default.csvに出力)
#define _LOG_RENAME_FILE_NAME

#ifdef __cplusplus
}
#endif


#endif /* _LOG_H_ */

/* end of file */
