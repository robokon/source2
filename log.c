#include "log.h"
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

typedef enum { NON_HEADER, INCLUDE_HEADER } CommitStyle;

static int LogNum = 0;              /* Log�i�[�z��̌��݈ʒu */
static int oldLogNum = 0;           /* Log�i�[�z��̏������ݍψʒu */
static int16_t count = 0;               /* Log�̃J�E���g */
Logger gst_Log_str[LOG_MAX]; /* Log�i�[�z�� */
void logStr(uint8_t reflect, int16_t p, int16_t i, int16_t d, int16_t);

static char logfile_name[21];
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

    /* ��^�C�g���}�� */
    if (cstyle == INCLUDE_HEADER) {
        for(j = 0; j < fpsize; j++) {
            fprintf(fp[j],"count,reflectedLight,normalizeReflectedLight,P,D,TURN,distance\n");
        }
    }
    
    /* Log�f�[�^�̏o�� */
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
// �֐��� : log_Commit
// ���� : �Ȃ�
// �Ԃ�l : �Ȃ�
// �T�v : ���[�J���t�@�C�������bluetooth�ڑ���Ƀ��O�̃f�[�^���o�͂���
//
//*****************************************************************************
void log_Commit(CommitStyle cstyle)
{
    FILE *localfile = NULL; /* �t�@�C���|�C���^ */
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
// �֐��� : initialize_log
// ���� : �Ȃ�
// �Ԃ�l : �Ȃ�
// �T�v : ���O�o�͏����̏��������s��
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
// �֐��� : close_log
// ���� : �Ȃ�
// �Ԃ�l : �Ȃ�
// �T�v : ���O�o�͂̏I������
//
//*****************************************************************************
void close_log()
{
    ter_tsk(LOG_TASK);
    log_Commit(NON_HEADER);
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
        if (LogNum > LOG_COMMIT_INTERVAL) {
            log_Commit(NON_HEADER);
        }
        tslp_tsk(10); /* 10msec�E�F�C�g */
    }
}

/* end of file */
