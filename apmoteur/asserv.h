#ifndef _ASSERV_H
#define _ASSERV_H
#define CONST_NUMBER_LIMIT 16
#include "data.h"
#include "glib.h"
typedef struct CONS CONS;
struct CONS {
    char* name;
    INTEGER32 value;
    INTEGER32 valmin;
    INTEGER32 valmax;
};
int asserv_init();
int asserv_check();
gpointer asserv_calc_mean_velocity(gpointer data);

#endif

