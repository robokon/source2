#include "log.h"
#include "Distance.h"

#define  LOG_MAX   10000                  /* Log �̍ő�� */
#define  LOG_FILE_NAME  "Log_yymmdd.csv" /* Log file�̖��O */

/* Log�p�̍\���� */
typedef struct{
    uint8_t Reflect;
    int16_t Gyro_angle;
    int16_t Gyro_rate;  
     int16_t P;
     int16_t D;
    float Count;
}Logger;

int LogNum = 0;              /* Log�񐔂̊i�[�ϐ� */
Logger gst_Log_str[LOG_MAX]; /* Log�i�[�z�� */
void logStr(uint8_t reflect, int16_t rate, int16_t p, int16_t d, int16_t count);
void logCommit(void);

//*****************************************************************************
// �֐��� : log_str
// ���� : �Ȃ�
// �Ԃ�l : �Ȃ�
// �T�v : �O���[�o���z�� gst_Log_str�Ɍ��݂̃Z���T�[�l���i�[
//
//*****************************************************************************
void log_Str(uint8_t reflect, int16_t rate, int16_t p, int16_t d, int16_t count)
{
    if(LogNum < LOG_MAX)
    {
        gst_Log_str[LogNum].Reflect = reflect;
        //gst_Log_str[LogNum].Gyro_angle = ev3_gyro_sensor_get_angle(gyro_sensor);
        gst_Log_str[LogNum].Gyro_rate = rate;
        gst_Log_str[LogNum].P = p;
        gst_Log_str[LogNum].D = d;
        gst_Log_str[LogNum].Count = Distance_getDistance();
        
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
void log_Commit(void)
{
    FILE *fp; /* �t�@�C���|�C���^ */
    int  i;   /* �C���N�������g */

    /* Log�t�@�C���쐬 */
    fp=fopen(LOG_FILE_NAME,"a");
    /* ��^�C�g���}�� */
    fprintf(fp,"���ˌ��Z���T�[, �W���C���Z���T�p���x,P,D,���s�����@\n");
    
    /* Log�̏o�� */
    for(i = 0 ; i < LOG_MAX; i++)
    {
        fprintf(fp,"%d,%d,%d,%d,%f\n",gst_Log_str[i].Reflect, gst_Log_str[i].Gyro_rate, gst_Log_str[i].P, gst_Log_str[i].D, gst_Log_str[i].Count);
    }
    
    fclose(fp);
}

/* end of file */
