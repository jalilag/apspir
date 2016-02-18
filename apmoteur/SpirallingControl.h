#ifndef _SPIRALLING_CONTROL_H
#define _SPIRALLING_CONTROL_H
#include "canfestival.h"
// DONNEE PROGRAMME
#define SLAVE_NUMBER_LIMIT 10 // Nombre d'esclave
#define PROFILE_NUMBER 5 // Nombre de profile
#define PARAM_NUMBER 25 // Nombre max de profile
#define VAR_NUMBER 21
#define LOCVAR_NUMBER 10

#define CYCLE_PERIOD 0x000186A0 // Sync tou les 500000us = 500ms
#define HB_CONS_BASE 0x000003E8 // Verification du heartbeat tout les 1000ms
#define HB_CONS_BASE_OFFSET 0x00000032 // Décalage de vérification 50ms
#define HB_PROD 0x01F4 // Hearbeat esclave envoyé tout les 500ms

#define FILE_SLAVE_CONFIG "config/slave_config.txt"
#define FILE_HELIX_CONFIG "config/helix_config.txt"
#define FILE_GEOM_CONFIG "config/geom_config.txt"
#define FILE_HELIX_RECORDED "config/helix_record.txt"
#define FILE_VELOCITY_LASER "config/velocity_laser.txt"
#define FILE_VELOCITY_SYNC "config/velocity_sync.txt"
#define FILE_ERROR "config/step_error.txt"

#define TIME_SET_LIMIT 300
#define STEP_LIMIT 6

#define PIPE_DIAMETER 326
#define WHEEL_PERIMETER 0.817
#define STEP_PER_REV 51200
#define ROT_REDUCTION 500
#define TRANS_REDUCTION 75
// PROFILE
void catch_signal(int sig);
void Exit(int type);

#endif

