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
void logCommit(FILE *fp);

static char logfile_name[21];

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
// �֐��� : log_commit
// ���� : �Ȃ�
// �Ԃ�l : �Ȃ�
// �T�v : �O���[�o���z�� gst_Log_str�Ɋi�[����Ă���f�[�^���t�@�C���o�͂���
//
//*****************************************************************************
void log_Commit(FILE *fp)
{
    int  i;   /* �C���N�������g */

    /* Log�t�@�C���쐬 */
    /* ��^�C�g���}�� */
    fprintf(fp,"���ˌ��Z���T�[, P,I,D,���s�����@\n");
    
    /* Log�̏o�� */
    for(i = 0 ; i < LOG_MAX; i++)
    {
        fprintf(fp,"%d,%d,%d,%d,%d\n",gst_Log_str[i].Reflect, gst_Log_str[i].P, gst_Log_str[i].I, gst_Log_str[i].D, gst_Log_str[i].Distance);
    }
}

//*****************************************************************************
// �֐��� : get_logfile
// ���� : �Ȃ�
// �Ԃ�l : �Ȃ�
// �T�v : �o�͂���logfile�̃t�@�C���|�C���^��Ԃ�
//
//*****************************************************************************
FILE* get_logfile()
{
    return fopen(logfile_name, "a");
}

//*****************************************************************************
// �֐��� : initialize_log
// ���� : �Ȃ�
// �Ԃ�l : �Ȃ�
// �T�v : ���O�o�͏����̏��������s��
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
