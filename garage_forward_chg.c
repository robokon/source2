#include "garage.h"
#include "app.h"
#include "Distance.h"

/* nakagawa Add_STA */
#define SLOW_DISTANCE1 (1000) /* �ᑬ�����b�� TBD */
#define SLOW_DISTANCE2 (1100) /* �ᑬ�����b�� TBD */
#define SLOW_DISTANCE3 (1200) /* �ᑬ�����b�� TBD */
#define SLOW_DISTANCE4 (1300) /* �ᑬ�����b�� TBD */
#define SLOW_DISTANCE5 (1400) /* �ᑬ�����b�� TBD */
#define STOP_DISTANCE (2000) /* �K���[�W�C�������b�� TBD*/
#define BACK_DISTANCE (1500) /* �K���[�W�C�������b�� TBD*/
/* nakagawa Add_END */

signed char forward;      /* �O��i���� */
signed char turn;         /* ���񖽗� */
signed char pwm_L, pwm_R; /* ���E���[�^PWM�o�� */

/* nakagawa Add_STA */
unsigned char slow_flag = 0;
float left_zankyori = 0.0;
float right_zankyori = 0.0;
/* nakagawa Add_STA 0729 */
static float distance = 0.0;     //���s����
static int Distance_first = 0;
static int touritu_end =0;
/* nakagawa Add_END 0729 */
extern int LIGHT_WHITE;         /* ���F�̌��Z���T�l */
extern int LIGHT_BLACK;
/* nakagawa Add_END */

static void garage_touritu_stop(void);

//*****************************************************************************
// �֐��� : garage_main
// ���� : 
// �Ԃ�l : �Ȃ�
// �T�v : 
//       
//*****************************************************************************
void garage_main()
{
    int32_t motor_ang_l, motor_ang_r;
    int gyro, volt;

/* nakagawa Add_STA */
    float distance4msL = 0.0; //���^�C����4ms�Ԃ̋���
    float distance4msR = 0.0; //�E�^�C����4ms�Ԃ̋���
/* nakagawa DEL STA 0729*/
//  float distance = 0.0;     //���s����
/* nakagawa DEL END 0729*/
/* nakagawa Add_END */
/* nakagawa add STA 0729*/
    int Distance_tmp = 0;
#if 0
    if(Distance_first == 0){
        Distance_init();
        Distance_first = 1;
    }
#endif
/* nakagawa add END 0729*/
        if (ev3_button_is_pressed(BACK_BUTTON)) return;

//         tail_control(TAIL_ANGLE_DRIVE); /* �o�����X���s�p�p�x�ɐ��� */
/* nakagawa_debug */
#if 0
         if (sonar_alert() == 1) /* ��Q�����m */
         {
             forward = turn = 0; /* ��Q�������m�������~ */
         }
         else
         {
/* nakagawa_debug */
#endif
             forward = 30; /* �O�i���� */
/* nakagawa_debug */
             turn = 0;
#if 0
             if (ev3_color_sensor_get_reflect(color_sensor) >= (LIGHT_WHITE + LIGHT_BLACK)/2)
             {
                 turn =  20; /* �����񖽗� */
             }
             else
             {
                 turn = -20; /* �E���񖽗� */
             }

         }
#endif
        /* nakagawa_debug */
         /* �|���U�q����API �ɓn���p�����[�^���擾���� */
         motor_ang_l = ev3_motor_get_counts(left_motor);/*�����[�^�[*/
         motor_ang_r = ev3_motor_get_counts(right_motor);/*�E���[�^�[*/
         gyro = ev3_gyro_sensor_get_rate(gyro_sensor);
         volt = ev3_battery_voltage_mV();

    
/* nakagawa Add_STA */
         /* �K���[�W�C���N���㑖�s�����������i�񂾂瑬�x��ቺ������ */
        Distance_tmp = Distance_getDistance();
        distance = Distance_tmp;
    if(slow_flag < 5){
        if(distance > SLOW_DISTANCE5 && slow_flag == 4){
            forward = 5;
            slow_flag=5;
            /*�X�^�[�g����*/
           while(1)
          {
              float tail = 0;
              tail = tail_control(TAIL_ANGLE_STAND_UP);
              if(tail==0)
              {
                tslp_tsk(10); /* 10msec�E�F�C�g */
               break;
              }
          }
            ev3_speaker_set_volume(3);
            ev3_speaker_play_tone(NOTE_C4, 100);
        }else if(distance > SLOW_DISTANCE4 && slow_flag == 3){
            forward = 2;
            slow_flag=4;
            ev3_speaker_set_volume(3);
            ev3_speaker_play_tone(NOTE_D4, 100);
        }else if(distance > SLOW_DISTANCE3 && slow_flag == 2){
            forward = 4;
            slow_flag=3;
            ev3_speaker_set_volume(3);
            ev3_speaker_play_tone(NOTE_E4, 100);
        }else if(distance > SLOW_DISTANCE2 && slow_flag == 1){
            forward = 6;
            slow_flag=2;
            ev3_speaker_set_volume(3);
            ev3_speaker_play_tone(NOTE_F4, 100);
        }else if(distance > SLOW_DISTANCE1 && slow_flag == 0){
            forward = 8;
            slow_flag=1;
            ev3_speaker_set_volume(3);
            ev3_speaker_play_tone(NOTE_G4, 100);
        }
    }
        #if 0
        if(distance > SLOW_DISTANCE && slow_flag < 5){
            tail_control(TAIL_ANGLE_STAND_UP); /* �|��������폜 �K�������� */ 
            if(slow_flag == 0){
                ev3_speaker_set_volume(5);
                ev3_speaker_play_tone(NOTE_B6, 1000);
            }
            slow_flag++;
            forward -= 10;
            if(forward <= 0){
                pwm_L = 10;
                pwm_R = 10;
            }
        }
#endif        
        /* �K���[�W�C���N���㑖�s�����������i�񂾂��~������ */
    if (distance > BACK_DISTANCE){
        forward = -10;
        ev3_speaker_set_volume(3);
        ev3_speaker_play_tone(NOTE_AS4, 100);
    }
        if (distance > STOP_DISTANCE && touritu_end == 0){
            forward = 0;
            garage_touritu_stop();
            touritu_end = 1;
            ev3_speaker_set_volume(3);
            ev3_speaker_play_tone(NOTE_B6, 100);
        }
    if(touritu_end == 1){
            forward = 0;
        ev3_speaker_set_volume(3);
        ev3_speaker_play_tone(NOTE_C4, 100);
        ev3_speaker_stop();
    }
    
/* nakagawa Add_END */    

            /* �|���U�q����API���Ăяo���A�|�����s���邽�߂� */
            /* ���E���[�^�o�͒l�𓾂� */
            balance_control(
                (float)forward,
                (float)turn,
                (float)gyro,
                (float)GYRO_OFFSET,
                (float)motor_ang_l,
                (float)motor_ang_r,
                (float)volt,
                (signed char*)&pwm_L,
                (signed char*)&pwm_R);



            /* EV3�ł̓��[�^�[��~���̃u���[�L�ݒ肪���O�ɂł��Ȃ����� */
            /* �o��0���ɁA���̓s�x�ݒ肷�� */
            if (pwm_L == 0)
            {
                 ev3_motor_stop(left_motor, true);
            }
            else
            {
                ev3_motor_set_power(left_motor, (int)pwm_L);
            }
            
            if (pwm_R == 0)
            {
                 ev3_motor_stop(right_motor, true);
            }
            else
            {
                ev3_motor_set_power(right_motor, (int)pwm_R);
            }

/* nakagawa Add_STA */
//        Distance_update();/* �����X�V�i4ms�Ԃ̈ړ������𖈉���Z���Ă���j */
#if 0
        distance4msL= Distance_getDistance4msLeft();    /* �������擾 */
        distance4msR= Distance_getDistance4msRight();   /* �E�����擾 */

        left_zankyori  += distance4msL;
        right_zankyori += distance4msR;

        distance += (left_zankyori + right_zankyori) / 2.0; //���E�^�C���̑��s�����𑫂��Ċ���
#endif


/* nakagawa Add_END */
}

static void garage_touritu_stop(void){
//    ev3_speaker_set_volume(5);
//    ev3_speaker_play_tone(NOTE_C6, 100);
    
//    ev3_motor_stop(left_motor, true);
//    ev3_motor_stop(right_motor, true);
    ev3_motor_set_power(left_motor, (int)pwm_L);
    ev3_motor_set_power(right_motor, (int)pwm_R);
    ev3_speaker_set_volume(3);
    ev3_speaker_play_tone(NOTE_B6, 1000);
    ev3_speaker_stop();
}
/* end of file */
