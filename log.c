#include "log.h"
#include "Distance.h"

#define  LOG_MAX   10000                  /* Log の最大回数 */
#define  LOG_FILE_NAME  "Log_%08d.csv" /* Log fileの名前 */

/* Log用の構造体 */
typedef struct{
    uint8_t Reflect;
    int16_t I;  
    int16_t P;
    int16_t D;
    int16_t Distance;
}Logger;

int LogNum = 0;              /* Log回数の格納変数 */
Logger gst_Log_str[LOG_MAX]; /* Log格納配列 */
void logStr(uint8_t reflect, int16_t p, int16_t i, int16_t d, int16_t);
void logCommit(FILE *fp);

static char logfile_name[21];

//*****************************************************************************
// 関数名 : log_str
// 引数 : なし
// 返り値 : なし
// 概要 : グローバル配列 gst_Log_strに現在のセンサー値を格納
//
//*****************************************************************************
void log_Str(uint8_t reflect, int16_t p, int16_t i, int16_t d)
{
    if(LogNum < LOG_MAX)
    {
        gst_Log_str[LogNum].Reflect = reflect;
        gst_Log_str[LogNum].P = p;
        gst_Log_str[LogNum].I = i;
        gst_Log_str[LogNum].D = d;
        gst_Log_str[LogNum].Distance = Distance_getDistance();
        LogNum++;    
    }
}

//*****************************************************************************
// 関数名 : log_commit
// 引数 : なし
// 返り値 : なし
// 概要 : グローバル配列 gst_Log_strに格納されているデータをファイル出力する
//
//*****************************************************************************
void log_Commit(FILE *fp)
{
    int  i;   /* インクリメント */

    /* Logファイル作成 */
    /* 列タイトル挿入 */
    fprintf(fp,"反射光センサー, P,I,D,走行距離　\n");
    
    /* Logの出力 */
    for(i = 0 ; i < LOG_MAX; i++)
    {
        fprintf(fp,"%d,%d,%d,%d,%d\n",gst_Log_str[i].Reflect, gst_Log_str[i].P, gst_Log_str[i].I, gst_Log_str[i].D, gst_Log_str[i].Distance);
    }
}

//*****************************************************************************
// 関数名 : get_logfile
// 引数 : なし
// 返り値 : なし
// 概要 : 出力するlogfileのファイルポインタを返す
//
//*****************************************************************************
FILE* get_logfile()
{
    return fopen(logfile_name, "a");
}

//*****************************************************************************
// 関数名 : initialize_log
// 引数 : なし
// 返り値 : なし
// 概要 : ログ出力処理の初期化を行う
//
//*****************************************************************************
void initialize_log()
{
    int i = 0;

    do {
        sprintf(logfile_name, LOG_FILE_NAME, i);
        i++;
    } while (fopen(logfile_name, "r") != NULL);
}

/* end of file */
