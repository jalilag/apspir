#ifndef _ASSERV_H
#define _ASSERV_H

#include "data.h"
#include "glib.h"
typedef struct CONS CONS;
struct CONS {
    char* name;
    double default_val;
    double valmin;
    double valmax;
};
int asserv_init();
int asserv_check();
int asserv_motor_config();
int asserv_motor_start();
gpointer asserv_calc_mean_velocity(gpointer data);

// Traitement des contantes
int asserv_get_const_index_with_name(char* name);
int asserv_get_const_index_with_id(char* id);
char* asserv_get_const_name_with_id(char* id);
int asserv_check_const(char* name,double val);
gpointer asserv_motor_stop (gpointer data);
#endif

