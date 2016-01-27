#ifndef _OD_CALLBACK_H
#define _OD_CALLBACK_H
#include "canfestival.h"

#endif
//ajout 260116
//definition des callback
UNS32 On_CaptPos_R_Update(CO_Data* d, const indextable * unsused_indextable, UNS8 unsused_bSubindex);
//setting des callback dynamique
void od_callback_define(void);
//fin ajout 260116
