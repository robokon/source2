#include "log.h"
#include "line_trace.h"
#include "Distance.h"

#define  LOG_MAX   15000                  /* Log �̍ő�� */
#define  LOG_COMMIT_INTERVAL 2500

#define  LOG_FILE_NAME  "Log_%08d.csv" /* Log file�̖��O */

/* Log�p�̍\���� */
typedef struct{
    uint8_t Reflect;
    float NormalizeReflect;
    float TURN;
    float P;
    float D;
    int16_t Distance;
}Logger;

typedef enum { COMMIT_DATA, COMMIT_HEADER } CommitStyle;

static int LogNum = 0;              /* Log�i�[�z��̌��݈ʒu */
static int oldLogNum = 0;           /* Log�i�[�z��̏������ݍψʒu */
static int16_t count = 0;               /* Log�̃J�E���g */
Logger gst_Log_str[LOG_MAX]; /* Log�i�[�z�� */
void logStr(uint8_t reflect, int16_t p, int16_t i, int16_t d, int16_t);

#ifdef _LOG_OUTPUT_FILE
static char logfile_name[21] = "default.csv";
#endif

static FILE *bluetooth;

//*****************************************************************************
// �֐��� : log_str
// ���� : �Ȃ�
// �Ԃ�l : �Ȃ�
// �T�v : �O���[�o���z�� gst_Log_str�Ɍ��݂̃Z���T�[�l���i�[
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
// �֐��� : write_data
// ���� : �Ȃ�
// �Ԃ�l : �Ȃ�
// �T�v : �O���[�o���z�� gst_Log_str�Ɋi�[����Ă���f�[�^���t�@�C���o�͂���
//
//*****************************************************************************
static void write_data(FILE *fp[], int fpsize, CommitStyle cstyle)
{
    int  i, j;   /* �C���N�������g */

    switch(cstyle) {
    case COMMIT_HEADER:
        /* ��^�C�g���}�� */
        for(j = 0; j < fpsize; j++) {
            fprintf(fp[j],"KP,KI,KD,TARGET,TURN_MAX,TURN_THRESHOLD,TURN_PER_THRESHOLD\n");
            fprintf(fp[j],"%f,%f,%f,%f,%d,%d,%f\n", LKP, LKI, LKD, TARGET, TURN_MAX, TURN_THRESHOLD, TURN_PER_THRESHOLD);
            fprintf(fp[j],"count,reflectedLight,normalizeReflectedLight,P,D,TURN,distance\n");
        }

    case COMMIT_DATA:
        /* Log�f�[�^�̏o�� */
        if (oldLogNum == LogNum)
            return;
    
        for(i = oldLogNum, oldLogNum = LogNum; i != oldLogNum; i = (i + 1) % LOG_MAX, count++)
        {
            for(j = 0; j < fpsize; j++) {
                fprintf(fp[j], "%d,%d,%f,%f,%f,%f,%d\n", count, gst_Log_str[i].Reflect, gst_Log_str[i].NormalizeReflect, gst_Log_str[i].P, gst_Log_str[i].D, gst_Log_str[i].TURN, gst_Log_str[i].Distance);
            }
        }
    }
}

//*****************************************************************************
// �֐��� : log_Commit
// ���� : �Ȃ�
// �Ԃ�l : �Ȃ�
// �T�v : ���[�J���t�@�C�������bluetooth�ڑ���Ƀ��O�̃f�[�^���o�͂���
//
//*****************************************************************************
void log_Commit(CommitStyle cstyle)
{
    FILE *fp[2];
    unsigned char fpnum = 0;

#ifdef _LOG_OUTPUT_FILE
    FILE *localfile = NULL; /* �t�@�C���|�C���^ */
#endif

#ifdef _LOG_OUTPUT_BLUETOOTH
    fp[fpnum] = bluetooth;
    fpnum++;
#endif

#ifdef _LOG_OUTPUT_FILE
    localfile = fopen(logfile_name, "a");
    if (localfile == NULL) {
        fprintf(bluetooth, "file open error! (file=%s)\n", logfile_name);
    } else {
        fp[fpnum] = localfile;
        fpnum++;
    }
#endif

    write_data(fp, fpnum, cstyle);

#ifdef _LOG_OUTPUT_FILE
    if (localfile != NULL) {
        fclose(localfile);
    }
#endif
}

//*****************************************************************************
// �֐��� : initialize_log
// ���� : �Ȃ�
// �Ԃ�l : �Ȃ�
// �T�v : ���O�o�͏����̏��������s��
//
//*****************************************************************************
void initialize_log(FILE *_bluetooth)
{
    bluetooth = _bluetooth;

#ifdef _LOG_OUTPUT_FILE
#ifdef _LOG_RENAME_FILE_NAME
    FILE *fp = NULL;
    int i = 0;

    do {
        if (fp != NULL)
            fclose(fp);
        sprintf(logfile_name, LOG_FILE_NAME, i);
        i++;
    } while ((fp = fopen(logfile_name, "r")) != NULL);
#endif
#endif

    log_Commit(COMMIT_HEADER);
    act_tsk(LOG_TASK);
}

//*****************************************************************************
// �֐��� : close_log
// ���� : �Ȃ�
// �Ԃ�l : �Ȃ�
// �T�v : ���O�o�͂̏I������
//
//*****************************************************************************
void close_log()
{
    ter_tsk(LOG_TASK);
    log_Commit(COMMIT_DATA);
    fprintf(bluetooth, "end to send data\n");
}

//*****************************************************************************
// �֐��� : log_task
// ���� : �Ȃ�
// �Ԃ�l : �Ȃ�
// �T�v : ����I�ȃ��O�o�͏���
//
//*****************************************************************************
void log_task(intptr_t unused)
{
    while(1) {
        if (((LogNum + LOG_MAX) - oldLogNum) % LOG_MAX > LOG_COMMIT_INTERVAL) {
            log_Commit(COMMIT_DATA);
        }
        tslp_tsk(10); /* 10msec�E�F�C�g */
    }
}

/* end of file */
