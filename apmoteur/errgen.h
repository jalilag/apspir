#ifndef _ERRGEN_H
#define _ERRGEN_H
#include "data.h"
#include <gtk/gtk.h>

void errgen_set(UNS16 dat,char* op);
GSourceFunc errgen_set_safe(gpointer data);

static char* errgen_get_title(UNS16 dat);
static char* errgen_get_content(UNS16 dat);

#include "errgen_laser.h"
#define LASER_ERROR(err)                    (0x80 | err)
#define ERR_LASER_SERIAL_CONFIG              0x0501
#define ERR_LASER_STARTCHECKTHREAD           0x0502

#define ERR_SLAVE_CONFIG_ROT_REFPOS          0x0510
#define ERR_MASTER_CONFIG_PDOT2              0x0511
#define ERR_LASER_ASSERV_START               0x0512
#define ERR_LASER_ASSERV_STOP                0x0513
#define ERR_LASER_SIMU_START                 0x0514
#define ERR_LASER_SIMU_STOP                  0x0515
#define ERR_ROT_CALC_ACCEL                   0x0516
#define ERR_ROT_WRITE_ACCEL                  0x0517
#define ERR_ROT_GET_ACCEL                    0x0518
#define ERR_ROT_GET_DECEL                    0x0519

#define ERR_LASER_ASSERV_2_VELMOTROTDEFINED  0x0520
#define ERR_LASER_ASSERV_READMOTROTDATA      0x0521
#define ERR_LASER_ASSERV_GETINDEXROT         0x0522
#define ERR_LASER_ASSERV_GETINDEXTRANS       0x0523
#define ERR_LASER_ASSERV_MOTROT              0x0524
#define ERR_LASER_ASSERV_SENS		         0x0525
#define ERR_LASER_ASSERV_GETSTARTPOS	     0x0526

#define ERR_DRIVER_UP                        0x0001
#define ERR_DRIVER_LOAD                      0x0002
#define ERR_DRIVER_OPEN                      0x0003
#define ERR_INIT_LOOP_RUN                    0x0004
#define ERR_GUI_LOOP_RUN                     0x0005
#define ERR_LASER_LOOP_RUN                   0x0006
#define ERR_VELOCITY_MEAN_LOOP               0x0007
#define ERR_VEL_STOP_LOOP                    0x0008
#define ERR_SUPPORT_STOP                     0x0009

#define ERR_FILE_OPEN                        0x0010
#define ERR_FILE_EMPTY                       0x0011
#define ERR_FILE_GEN                         0x0012
#define ERR_FILE_SLAVE_DEF                   0x0013
#define ERR_FILE_PROFILE                     0x0014
#define ERR_FILE_PROFILE_INVALID_PARAM       0x0015

#define ERR_MASTER_CONFIG                    0x0040
#define ERR_MASTER_STOP_SYNC                 0x0041
#define ERR_MASTER_START_SYNC                0x0042
#define ERR_MASTER_SET_PERIOD                0x0043
#define ERR_MASTER_SET_HB_CONS               0x0044
#define ERR_MASTER_CONFIG_PDOR               0x0045
#define ERR_MASTER_CONFIG_PDOT               0x0046

#define ERR_SLAVE_CONFIG                     0x0060
#define ERR_SLAVE_CONFIG_LSS                 0x0061
#define ERR_SLAVE_CONFIG_HB                  0x0062
#define ERR_SLAVE_CONFIG_PDOT                0x0063
#define ERR_SLAVE_CONFIG_PDOR                0x0064
#define ERR_SLAVE_CONFIG_ACTIVE_PDO          0x0065
#define ERR_SLAVE_CONFIG_CURRENT             0x0066
#define ERR_SLAVE_CONFIG_PROFILE             0x0067
#define ERR_SLAVE_CONFIG_MAX_VELOCITY        0x0068
#define ERR_SLAVE_CONFIG_ACCELERATION        0x0069
#define ERR_SLAVE_CONFIG_DECELERATION        0x006A
#define ERR_SLAVE_CONFIG_DECELERATION_QS     0x006B
#define ERR_SLAVE_CONFIG_TORQUE              0x006C
#define ERR_SLAVE_CONFIG_TORQUE_SLOPE        0x006D
#define ERR_SLAVE_CONFIG_TORQUE_MAX_VELOCITY 0x006E
#define ERR_SLAVE_CONFIG_MOTOR_SON           0x006F
#define ERR_SLAVE_SAVE_CONFIG                0x0070
#define ERR_SLAVE_LOAD_CONFIG                0x0071
#define ERR_SLAVE_LOAD_INTERFACE             0x0072

#define ERR_READ_ACCELERATION                0x00C0
#define ERR_READ_DECELERATION                0x00C1
#define ERR_READ_DECELERATION_QS             0x00C2
#define ERR_READ_TORQUE                      0x00C3
#define ERR_READ_TORQUE_SLOPE                0x00C4
#define ERR_READ_TORQUE_VELOCITY             0x00C5
#define ERR_READ_CURRENT                     0x00C6
#define ERR_READ_PROFILE                     0x00C7
#define ERR_READ_TORQUE_VELOCITY_MAKEUP      0x00C8
#define ERR_READ_HMT_ACTIVATE                0x00C9
#define ERR_READ_HMT_CONTROL                 0x00CA
#define ERR_READ_VELOCITY                    0x00CB
#define ERR_READ_POSITION                    0x00CC
#define ERR_READ_CONTROL                     0x00CD
#define ERR_READ_VELOCITY_MAX                0x00CE

#define ERR_SET_ACCELERATION                 0x0100
#define ERR_SET_DECELERATION                 0x0101
#define ERR_SET_DECELERATION_QS              0x0102
#define ERR_SET_VELOCITY_INC                 0x0103
#define ERR_SET_TORQUE                       0x0104
#define ERR_SET_TORQUE_SLOPE                 0x0105
#define ERR_SET_TORQUE_VELOCITY              0x0106
#define ERR_SET_CURRENT                      0x0107
#define ERR_SET_PROFILE                      0x0108
#define ERR_SET_PDO                          0x0109
#define ERR_SET_TORQUE_VELOCITY_MAKEUP       0x010A
#define ERR_SET_HMT_ACTIVATE                 0x010B
#define ERR_SET_HMT_CONTROL                  0x010C
#define ERR_SET_VELOCITY                     0x010D
#define ERR_SET_POSITION                     0x010E
#define ERR_SET_CONTROL                      0x010F
#define ERR_SET_VELOCITY_MAX                 0x0110

#define ERR_SAVE_ACCELERATION                0x0140
#define ERR_SAVE_DECELERATION                0x0141
#define ERR_SAVE_DECELERATION_QS             0x0142
#define ERR_SAVE_TORQUE                      0x0143
#define ERR_SAVE_TORQUE_SLOPE                0x0144
#define ERR_SAVE_TORQUE_VELOCITY             0x0145
#define ERR_SAVE_CURRENT                     0x0146
#define ERR_SAVE_PROFILE                     0x0147
#define ERR_SAVE_TORQUE_VELOCITY_MAKEUP      0x0148
#define ERR_SAVE_HMT_ACTIVATE                0x0149
#define ERR_SAVE_HMT_CONTROL                 0x014A
#define ERR_SAVE_VELOCITY                    0x014B
#define ERR_SAVE_POSITION                    0x014C
#define ERR_SAVE_CONTROL                     0x014D
#define ERR_SAVE_VELOCITY_MAX                0x014F

#define ERR_MOTOR_PAUSE                      0x0180
#define ERR_MOTOR_RUN                        0x0181
#define ERR_MOTOR_OFF                        0x0182
#define ERR_MOTOR_FORWARD                    0x0183
#define ERR_MOTOR_BACKWARD                   0x0184
#define ERR_MOTOR_LOW_VOLTAGE                0x0185

#define LASER_ERROR(err)                    (0x80 | err)
#define ERR_LASER_REINIT                     0x0500
#define ERR_LASER_SERIAL_CONFIG              0x0501
#define ERR_LASER_STARTCHECKTHREAD           0x0502

#endif // _ERRGEN
