#ifndef _SPIRALLING_CONTROL_H
#define _SPIRALLING_CONTROL_H
#include "canfestival.h"
// DONNEE PROGRAMME
#define SLAVE_NUMBER_LIMIT 10 // Nombre d'esclave
#define PROFILE_NUMBER_LIMIT 10 // Nombre max de profile
#define PARAM_NUMBER 18 // Nombre max de profile
#define VAR_NUMBER 18
#define CYCLE_PERIOD 0x0007A120 // Sync tou les 50000us = 50ms
#define HB_CONS_BASE 0x000003E8 // Verification du heartbeat tout les 1000ms
#define HB_CONS_BASE_OFFSET 0x00000032 // Décalage de vérification 50ms
#define HB_PROD 0x01F4 // Hearbeat esclave envoyé tout les 500ms
#define FILE_SLAVE_CONFIG "slave_config.txt"


void catch_signal(int sig);
void Exit(int type);

#endif

