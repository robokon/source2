#include "log.h"
#include "Distance.h"

#define  LOG_MAX   10000                  /* Log の最大回数 */
#define  LOG_FILE_NAME  "Log_yymmdd.csv" /* Log fileの名前 */

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
void logCommit(void);

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
void log_Commit(void)
{
    FILE *fp; /* ファイルポインタ */
    int  i;   /* インクリメント */

    /* Logファイル作成 */
    fp=fopen(LOG_FILE_NAME,"a");
    /* 列タイトル挿入 */
    fprintf(fp,"反射光センサー, P,I,D,走行距離　\n");
    
    /* Logの出力 */
    for(i = 0 ; i < LOG_MAX; i++)
    {
        fprintf(fp,"%d,%d,%d,%d,%d\n",gst_Log_str[i].Reflect, gst_Log_str[i].P, gst_Log_str[i].I, gst_Log_str[i].D, gst_Log_str[i].Distance);
    }
    
    fclose(fp);
}

/* end of file */
