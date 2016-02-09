#include "od_callback.h"
#include "canfestival.h"
#include "SpirallingMaster.h"
//ajout 260116
#include "laser_asserv.h"
#include "cantools.h"
#include "profile.h"
#include "slave.h"

extern int SLAVE_NUMBER;
unsigned long CaptPos_R_Time = 0;
unsigned long CaptVit_T_Time = 0;
UNS32 On_CaptPos_R_Update(CO_Data* d, const indextable * unsused_indextable, UNS8 unsused_bSubindex){
    CaptPos_R_Time = GetDate_us();
}
UNS32 On_CaptVit_T_Update(CO_Data* d, const indextable * unsused_indextable, UNS8 unsused_bSubindex){
    CaptVit_T_Time = GetDate_us();
    int i;
    for (i=0;i<SLAVE_NUMBER;i++){
        if((slave_get_profile_with_index(i) == PROF_VITTRANS) || slave_get_profile_with_index(i) == PROF_COUPLTRANS)
            slave_set_param("Velocity", i, CaptVit_T);
    }
    return 0;
}
UNS32 On_ConsVit_T_Update(CO_Data* d, const indextable * unsused_indextable, UNS8 unsused_bSubindex){
    int i;
    return 0;
}

void od_callback_define(void){
    RegisterSetODentryCallBack(&SpirallingMaster_Data, 0x201D, 0x00, &On_CaptPos_R_Update);
    RegisterSetODentryCallBack(&SpirallingMaster_Data, 0x201B, 0x00, &On_CaptVit_T_Update);
    RegisterSetODentryCallBack(&SpirallingMaster_Data, 0x2019, 0x00, &On_ConsVit_T_Update);
}
//fin ajout 260116

