#ifndef _ERRGEN_H
#define _ERRGEN_H
#include "data.h"
#include <gtk/gtk.h>

void errgen_set(UNS16 dat,char* op);
GSourceFunc errgen_set_safe(gpointer data);

static char* errgen_get_title(UNS16 dat);
static char* errgen_get_content(UNS16 dat);

#define ERR_DRIVER_UP                        0x0001
#define ERR_DRIVER_LOAD                      0x0002
#define ERR_DRIVER_OPEN                      0x0003
#define ERR_INIT_LOOP_RUN                    0x0004
#define ERR_GUI_LOOP_RUN                     0x0005

#define ERR_FILE_OPEN                        0x0006
#define ERR_FILE_EMPTY                       0x0007
#define ERR_FILE_GEN                         0x0008
#define ERR_FILE_SLAVE_DEF                   0x0009
#define ERR_FILE_PROFILE                     0x000A
#define ERR_FILE_PROFILE_INVALID_PARAM       0x000B

#define ERR_MASTER_CONFIG                    0x0040
#define ERR_MASTER_STOP_SYNC                 0x0041
#define ERR_MASTER_START_SYNC                0x0042
#define ERR_MASTER_SET_PERIOD                0x0043
#define ERR_MASTER_SET_HB_CONS               0x0044
#define ERR_MASTER_CONFIG_PDOR               0x0045
#define ERR_MASTER_CONFIG_PDOT               0x0046

#define ERR_SLAVE_CONFIG                     0x0080
#define ERR_SLAVE_CONFIG_LSS                 0x0081
#define ERR_SLAVE_CONFIG_HB                  0x0082
#define ERR_SLAVE_CONFIG_PDOT                0x0083
#define ERR_SLAVE_CONFIG_PDOR                0x0086
#define ERR_SLAVE_CONFIG_ACTIVE_PDO          0x0087
#define ERR_SLAVE_CONFIG_CURRENT             0x0088
#define ERR_SLAVE_CONFIG_PROFILE             0x0089
#define ERR_SLAVE_CONFIG_MAX_VELOCITY        0x008A
#define ERR_SLAVE_CONFIG_ACCELERATION        0x008B
#define ERR_SLAVE_CONFIG_DECELERATION        0x008C
#define ERR_SLAVE_CONFIG_DECELERATION_QS     0x008D
#define ERR_SLAVE_CONFIG_TORQUE              0x008E
#define ERR_SLAVE_CONFIG_TORQUE_SLOPE        0x008F
#define ERR_SLAVE_CONFIG_TORQUE_MAX_VELOCITY 0x0090
#define ERR_SLAVE_CONFIG_MOTOR_SON           0x0091
#define ERR_SLAVE_SAVE_CONFIG                0x0092
#define ERR_SLAVE_LOAD_CONFIG                0x0093
#define ERR_SLAVE_LOAD_INTERFACE             0x0094

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
#endif // _ERRGEN
