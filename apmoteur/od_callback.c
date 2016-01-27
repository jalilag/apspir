#include "od_callback.h"
#include "canfestival.h"
#include "SpirallingMaster.h"
//ajout 260116
#include "laser_asserv.h"
//fin ajout 260116

unsigned long CaptPos_R_Time = 0;

UNS32 On_CaptPos_R_Update(CO_Data* d, const indextable * unsused_indextable, UNS8 unsused_bSubindex){
    CaptPos_R_Time = GetDate_us();
}

void od_callback_define(void){
    RegisterSetODentryCallBack(&SpirallingMaster_Data, 0x201D, 0x00, &On_CaptPos_R_Update);
}
//fin ajout 260116

