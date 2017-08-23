#include "log.h"
#include "Distance.h"

#define  LOG_MAX   10000                  /* Log �̍ő�� */
#define  LOG_FILE_NAME  "Log_%08d.csv" /* Log file�̖��O */

/* Log�p�̍\���� */
typedef struct{
    uint8_t Reflect;
    int16_t I;  
    int16_t P;
    int16_t D;
    int16_t Distance;
}Logger;

int LogNum = 0;              /* Log�񐔂̊i�[�ϐ� */
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
// �֐��� : write_data
// ���� : �Ȃ�
// �Ԃ�l : �Ȃ�
// �T�v : �O���[�o���z�� gst_Log_str�Ɋi�[����Ă���f�[�^���t�@�C���o�͂���
//
//*****************************************************************************
static void write_data(FILE *fp)
{
    int  i;   /* �C���N�������g */

    /* ��^�C�g���}�� */
    fprintf(fp,"���ˌ��Z���T�[, P,I,D,���s�����@\n");
    
    /* Log�f�[�^�̏o�� */
    for(i = 0 ; i < LogNum; i++)
    {
        fprintf(fp,"%d,%d,%d,%d,%d\n",gst_Log_str[i].Reflect, gst_Log_str[i].P, gst_Log_str[i].I, gst_Log_str[i].D, gst_Log_str[i].Distance);
    }
}

//*****************************************************************************
// �֐��� : log_Commit
// ���� : �Ȃ�
// �Ԃ�l : �Ȃ�
// �T�v : ���[�J���t�@�C�������bluetooth�ڑ���Ƀ��O�̃f�[�^���o�͂���
//
//*****************************************************************************
void log_Commit()
{
    FILE *localfile; /* �t�@�C���|�C���^ */
    localfile = fopen(logfile_name, "a");

    write_data(localfile);
    write_data(bluetooth);

    fclose(localfile);
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
    bluetooth = _bluetooth;

    do {
        sprintf(logfile_name, LOG_FILE_NAME, i);
        i++;
    } while (fopen(logfile_name, "r") != NULL);

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
    log_Commit();
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
    if (LogNum > (LOG_MAX / 2)) {
        log_Commit();
        LogNum = 0;
    }
    tslp_tsk(10); /* 10msec�E�F�C�g */
}

/* end of file */
