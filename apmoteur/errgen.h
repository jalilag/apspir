#ifndef _ERRGEN_H
#define _ERRGEN_H
#include "data.h"

extern UNS8 errgen_laserState;

void errgen_set(UNS16 dat,char* op);
static char* errgen_get_title(UNS16 dat);
static char* errgen_get_content(UNS16 dat);

#include "errgen_laser.h"
#define LASER_ERROR(err)                    (0x80 | err)
#define ERR_LASER_REINIT                     0x0500
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

#define ERR_LASER_ASSERV_2_VELMOTROTDEFINED  0x0540
#define ERR_LASER_ASSERV_READMOTROTDATA      0x0541
#define ERR_LASER_ASSERV_GETINDEXROT         0x0542
#define ERR_LASER_ASSERV_GETINDEXTRANS       0x0543
#define ERR_LASER_ASSERV_MOTROT              0x0544

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

#define ERR_MASTER_CONFIG                    0x0010
#define ERR_MASTER_STOP_SYNC                 0x0011
#define ERR_MASTER_START_SYNC                0x0012
#define ERR_MASTER_SET_PERIOD                0x0013
#define ERR_MASTER_SET_HB_CONS               0x0014
#define ERR_MASTER_CONFIG_PDOR               0x0015
#define ERR_MASTER_CONFIG_PDOT               0x0016
#define ERR_MASTER_CONFIG_MAP                0x0017

#define ERR_SLAVE_CONFIG                     0x0020
#define ERR_SLAVE_CONFIG_LSS                 0x0021
#define ERR_SLAVE_CONFIG_HB                  0x0022
#define ERR_SLAVE_CONFIG_PDOT                0x0023
#define ERR_SLAVE_CONFIG_PDOR                0x0026
#define ERR_SLAVE_CONFIG_ACTIVE_PDO          0x0027
#define ERR_SLAVE_CONFIG_CURRENT             0x0028
#define ERR_SLAVE_CONFIG_PROFILE             0x0029
#define ERR_SLAVE_CONFIG_MAX_VELOCITY        0x002A
#define ERR_SLAVE_CONFIG_ACCELERATION        0x002B
#define ERR_SLAVE_CONFIG_DECELERATION        0x002C
#define ERR_SLAVE_CONFIG_DECELERATION_QS     0x002D
#define ERR_SLAVE_CONFIG_TORQUE              0x002E
#define ERR_SLAVE_CONFIG_TORQUE_SLOPE        0x002F
#define ERR_SLAVE_CONFIG_TORQUE_MAX_VELOCITY 0x0030
#define ERR_SLAVE_CONFIG_MOTOR_SON           0x0031
#define ERR_SLAVE_SAVE_CONFIG                0x0032
#define ERR_SLAVE_LOAD_CONFIG                0x0033
#define ERR_SLAVE_LOAD_INTERFACE             0x0034

#define ERR_READ_ACCELERATION                0x0040
#define ERR_READ_DECELERATION                0x0041
#define ERR_READ_DECELERATION_QS             0x0042
#define ERR_READ_TORQUE                      0x0043
#define ERR_READ_TORQUE_SLOPE                0x0044
#define ERR_READ_TORQUE_VELOCITY             0x0045
#define ERR_READ_CURRENT                     0x0046
#define ERR_READ_PROFILE                     0x0047
#define ERR_READ_TORQUE_VELOCITY_MAKEUP      0x0048
#define ERR_READ_HMT_ACTIVATE                0x0049
#define ERR_READ_HMT_CONTROL                 0x004A
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

#define ERR_MOTOR_PAUSE                      0x0180
#define ERR_MOTOR_RUN                        0x0181
#define ERR_MOTOR_OFF                        0x0182
#define ERR_MOTOR_FORWARD                    0x0183
#define ERR_MOTOR_BACKWARD                   0x0184
#define ERR_MOTOR_LOW_VOLTAGE                0x0185

#endif // _ERRGEN
