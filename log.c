#include "log.h"
#include "Distance.h"

#define  LOG_MAX   15000                  /* Log の最大回数 */
#define  LOG_COMMIT_INTERVAL 2500

#define  LOG_FILE_NAME  "Log_%08d.csv" /* Log fileの名前 */

/* Log用の構造体 */
typedef struct{
    uint8_t Reflect;
    float NormalizeReflect;
    float TURN;
    float P;
    float D;
    int16_t Distance;
}Logger;

typedef enum { NON_HEADER, INCLUDE_HEADER } CommitStyle;

static int LogNum = 0;              /* Log格納配列の現在位置 */
static int oldLogNum = 0;           /* Log格納配列の書き込み済位置 */
static int16_t count = 0;               /* Logのカウント */
Logger gst_Log_str[LOG_MAX]; /* Log格納配列 */
void logStr(uint8_t reflect, int16_t p, int16_t i, int16_t d, int16_t);

static char logfile_name[21];
static FILE *bluetooth;

//*****************************************************************************
// 関数名 : log_str
// 引数 : なし
// 返り値 : なし
// 概要 : グローバル配列 gst_Log_strに現在のセンサー値を格納
//
//*****************************************************************************
void log_Str(uint8_t reflect, float normalize_reflect, float p, float d, float turn)
{
    if(LogNum < LOG_MAX)
    {
        gst_Log_str[LogNum].Reflect = reflect;
        gst_Log_str[LogNum].NormalizeReflect = normalize_reflect;
        gst_Log_str[LogNum].P = p;
        gst_Log_str[LogNum].D = d;
        gst_Log_str[LogNum].TURN = turn;
        gst_Log_str[LogNum].Distance = Distance_getDistance();
        LogNum++;
        LogNum %= LOG_MAX;    
    }
}

//*****************************************************************************
// 関数名 : write_data
// 引数 : なし
// 返り値 : なし
// 概要 : グローバル配列 gst_Log_strに格納されているデータをファイル出力する
//
//*****************************************************************************
static void write_data(FILE *fp[], int fpsize, CommitStyle cstyle)
{
    int  i, j;   /* インクリメント */

    /* 列タイトル挿入 */
    if (cstyle == INCLUDE_HEADER) {
        for(j = 0; j < fpsize; j++) {
            fprintf(fp[j],"count,reflectedLight,normalizeReflectedLight,P,D,TURN,distance\n");
        }
    }
    
    /* Logデータの出力 */
    if (oldLogNum == LogNum)
        return;

    for(i = oldLogNum, oldLogNum = LogNum; i != (oldLogNum + 1) % LOG_MAX; i = (i + 1) % LOG_MAX, count++)
    {
        for(j = 0; j < fpsize; j++) {
            fprintf(fp[j], "%d,%d,%f,%f,%f,%f,%d\n", count, gst_Log_str[i].Reflect, gst_Log_str[i].NormalizeReflect, gst_Log_str[i].P, gst_Log_str[i].D, gst_Log_str[i].TURN, gst_Log_str[i].Distance);
        }
    }
}

//*****************************************************************************
// 関数名 : log_Commit
// 引数 : なし
// 返り値 : なし
// 概要 : ローカルファイルおよびbluetooth接続先にログのデータを出力する
//
//*****************************************************************************
void log_Commit(CommitStyle cstyle)
{
    FILE *localfile = NULL; /* ファイルポインタ */
    FILE *fp[2];
    unsigned char fpnum = 1;

    fp[0] = bluetooth;

    localfile = fopen(logfile_name, "a");
    if (localfile == NULL) {
        fprintf(bluetooth, "file open error! (file=%s)\n", logfile_name);
    } else {
        fp[1] = localfile;
        fpnum++;
    }

    write_data(fp, fpnum, cstyle);

    if (localfile != NULL) {
        fclose(localfile);
    }
}

//*****************************************************************************
// 関数名 : initialize_log
// 引数 : なし
// 返り値 : なし
// 概要 : ログ出力処理の初期化を行う
//
//*****************************************************************************
void initialize_log(FILE *_bluetooth)
{
    int i = 0;
    FILE *fp = NULL;
    bluetooth = _bluetooth;

    do {
        if (fp != NULL)
            fclose(fp);
        sprintf(logfile_name, LOG_FILE_NAME, i);
        i++;
    } while ((fp = fopen(logfile_name, "r")) != NULL);

    log_Commit(INCLUDE_HEADER);
    act_tsk(LOG_TASK);
}

//*****************************************************************************
// 関数名 : close_log
// 引数 : なし
// 返り値 : なし
// 概要 : ログ出力の終了処理
//
//*****************************************************************************
void close_log()
{
    ter_tsk(LOG_TASK);
    log_Commit(NON_HEADER);
}

//*****************************************************************************
// 関数名 : log_task
// 引数 : なし
// 返り値 : なし
// 概要 : 定期的なログ出力処理
//
//*****************************************************************************
void log_task(intptr_t unused)
{
    while(1) {
        if (LogNum > LOG_COMMIT_INTERVAL) {
            log_Commit(NON_HEADER);
        }
        tslp_tsk(10); /* 10msecウェイト */
    }
}

/* end of file */
