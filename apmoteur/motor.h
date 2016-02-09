#ifndef _MOTOR_H
#define _MOTOR_H
#include "canfestival.h"

typedef struct PARAM PARAM;
struct PARAM {
    char* title;
    char* gui_code;
    UNS16 index;
    UNS8 subindex;
    UNS8 type;
    INTEGER32 valDown;
    INTEGER32 valUp;
    int err_set;
    int err_read;
    int err_save;
};


UNS8 motor_get_state(UNS16 state);
char* motor_get_state_title(UNS8 key);
char* motor_get_error_title(UNS16 data);
int motor_get_target(UNS16 state);
int motor_switch_on(UNS8 node);
int motor_switch_off(UNS8 node);
int motor_start(UNS8 node,int start); // SON -> OP
int motor_forward(UNS8 node,int polar ); // Polarity

int motor_set_param(UNS8 node,char* id,INTEGER32 dat);
int motor_get_param(UNS8 node,char* id,INTEGER32* dat);
char* motor_get_param_title(char* id);
int motor_get_param_index(char* id);
UNS8 motor_get_param_type(char* id);
INTEGER16 motor_get_couple(INTEGER32 vel);
//ajout 220116
void motor_set_MotRot_Accel(void);
//fin ajout 220116
#endif
