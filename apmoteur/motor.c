#include <stdio.h>
#include <string.h>
#include <unistd.h>
#include <stdlib.h>
#include <gtk/gtk.h>
#include "motor.h"
#include "strtools.h"
#include "gui.h"
#include "keyword.h"
#include "SpirallingControl.h"
#include "master.h"
//#include "canfestival.h"
#include "cantools.h"
#include "slave.h"

//ajout 220116
#include "laser_asserv.h"
#include "laser_simulation.h"
#include "profile.h"
#include "errgen.h"
extern UNS16 errgen_state;
extern char * errgen_aux;
//fin ajout 220116

int motor_switch_step = 0;
const int motor_switch_step_max = 10;
extern PARAM pardata[PARAM_NUMBER];
extern INTEGER32 velocity_inc[SLAVE_NUMBER_LIMIT];

UNS8 motor_get_state(UNS16 state) {
    char *bstate = strtools_hex2bin(&state,0x06);
    if (bstate != NULL) {
        if (bstate[0] == '0' && bstate[1] == '0' && bstate[2] == '0' && bstate[3] == '0' &&
            bstate[6] == '0' ) {
            return NR2SON;

        } else if (bstate[0] == '0' && bstate[1] == '0' && bstate[2] == '0' && bstate[3] == '0'  &&
            bstate[6] == '1' ) {
            return SOND;

        } else if (bstate[0] == '1' && bstate[1] == '0' && bstate[2] == '0' && bstate[3] == '0'  &&
            bstate[5] == '1' && bstate[6] == '0' ) {
            return R2SON;

        } else if (bstate[0] == '1' && bstate[1] == '1' && bstate[2] == '0' && bstate[3] == '0'  &&
            bstate[5] == '1' && bstate[6] == '0' ) {
            return SON;

        } else if (bstate[0] == '1' && bstate[1] == '1' && bstate[2] == '1' && bstate[3] == '0'  &&
            bstate[5] == '1' && bstate[6] == '0' ) {
            return OENABLED;

        } else if (bstate[0] == '1' && bstate[1] == '1' && bstate[2] == '1' && bstate[3] == '0'  &&
            bstate[5] == '0' && bstate[6] == '0' ) {
            return QUICK_STOP_A;

        } else if (bstate[0] == '1' && bstate[1] == '1' && bstate[2] == '1' && bstate[3] == '1'  &&
            bstate[6] == '0' ) {
            return FAULT_REACTION_A;

        } else if (bstate[0] == '0' && bstate[1] == '0' && bstate[2] == '0' && bstate[3] == '1'  &&
            bstate[6] == '0' ) {
            return FAULT;
        } else {
            return NR2SON;
        }
    } else {
        return NR2SON;
    }
}

int motor_get_target(UNS16 state) {
    char *bstate = strtools_hex2bin(&state,0x06);
    if (bstate != NULL) {
        if (bstate[10] == '1') return 1;
        else return 0;
    }
}

char* motor_get_state_title(UNS8 key) {
    if (key == 0x00) {
        return NOT_DEFINED_txt;
    } else if (key == 0x01) {
        return NR2SON_txt;
    } else if (key == 0x02) {
        return SOND_txt;
    } else if (key == 0x03) {
        return R2SON_txt;
    } else if (key == 0x04) {
        return SON_txt;
    } else if (key == 0x05) {
        return OENABLED_txt;
    } else if (key == 0x06) {
        return QUICK_STOP_A_txt;
    } else if (key == 0x07) {
        return FAULT_REACTION_A_txt;
    } else if (key == 0x08) {
        return FAULT_txt;
    }
}

char* motor_get_error_title(UNS16 data) {
    if (data == MOTOR_ERROR_CAN_OVERRUN ) return MOTOR_ERROR_CAN_OVERRUN_TXT;
    if (data == MOTOR_ERROR_CAN_PASSIVE ) return MOTOR_ERROR_CAN_PASSIVE_TXT;
    if (data == MOTOR_ERROR_HEARTBEAT ) return MOTOR_ERROR_HEARTBEAT_TXT;
    if (data == MOTOR_ERROR_BUSOFF_RECOVER ) return MOTOR_ERROR_BUSOFF_RECOVER_TXT;
    if (data == MOTOR_ERROR_BUSOFF_PENDING ) return MOTOR_ERROR_BUSOFF_PENDING_TXT;
    if (data == MOTOR_ERROR_BUSOFF_RECOVER ) return MOTOR_ERROR_BUSOFF_RECOVER_TXT;
    if (data == MOTOR_ERROR_PDO ) return MOTOR_ERROR_PDO_TXT;
    if (data == MOTOR_ERROR_TEMP ) return MOTOR_ERROR_TEMP_TXT;
    if (data == MOTOR_ERROR_IDLE ) return MOTOR_ERROR_IDLE_TXT;
}

int motor_switch_on(UNS8 node) {
    SDOR c_word={0x6040,0x00,0x06};
    UNS8 strState = motor_get_state((UNS16)slave_get_param_in_num("Power",slave_get_index_with_node(node)));
    if (strState != NR2SON) {
        if (motor_switch_step >= motor_switch_step_max) {
            motor_switch_step = 0;
            return 0;
        } else {
            if (strState == SOND) {
                UNS16 word = 0x0006;
                if(!cantools_write_sdo(node,c_word,&word)) {
                    motor_switch_step++;
                }
                motor_switch_on(node);
            }
            if (strState == R2SON) {
                UNS16 word = 0x0007;
                if(!cantools_write_sdo(node,c_word,&word)) {
                    motor_switch_step++;
                }
                motor_switch_on(node);
            }
            if (strState == SON) {
                motor_switch_step = 0;
                return 1;
            }
        }
    } else {
        motor_switch_step = 0;
        return 0;
    }
}

/**
* Arret d'un moteur
* Mode SOND
**/
int motor_switch_off(UNS8 node) {
    if (motor_switch_step >= motor_switch_step_max){
        motor_switch_step = 0;
        return 0;
    }
    UNS8 strState = motor_get_state((UNS16)slave_get_param_in_num("Power",slave_get_index_with_node(node)));
    if (strState != NR2SON) {
        if (strState != SOND) {
            SDOR c_word={0x6040,0x00,0x06};
            UNS16 word = 0x0002;
            if (cantools_write_sdo(node,c_word,&word)) {
                motor_switch_step = 0;
                return 1;
            } else {
                motor_switch_step++;
                motor_switch_off(node);
            }
        } else {
            motor_switch_step = 0;
            return 1;
        }
    } else {
        motor_switch_step = 0;
        return 0;
    }
}
/**
* Passage du mode SON à OP
* start : 1
* stop : 0
**/
int motor_start(UNS8 node,int start) {
    SDOR c_word={0x6040,0x00,0x06};
    UNS16 word;
    UNS8 state = motor_get_state((UNS16)slave_get_param_in_num("Power",slave_get_index_with_node(node)));
    if (state != SON && state != OENABLED)
        return 0;
    if (state == SON && start) {
        word = 0x000F;
        if (cantools_write_sdo(node,c_word,&word))
            return 1;
        else
            return 0;
    } else if (state == OENABLED && start == 0) {
        word = 0x0007;
        if (cantools_write_sdo(node,c_word,&word))
            return 1;
        else
            return 0;
    }
    return 1;
}
/**
* Changement de polarité
* 1 : avant
* 0 : arrière
*/
int motor_forward(UNS8 node,int polar) {
    INTEGER8 dat;
    int error = 0;
    SDOR polar_adress = {0x2030,0x00,0x02};
    if (polar)
        dat = 1;
    else
        dat = -1;
    if (!motor_start(node,0))
        return 0;
    if (!cantools_write_sdo(node,polar_adress,&dat))
        return 0;
    return 1;
}

/**
* Permet la modification d'un des paramètres de la liste
* node
* i : indice du paramètre
* dat : valeur en INTEGER32
**/
int motor_set_param(UNS8 node,char* id,INTEGER32 dat) {
    int i;
    for (i=0;i<PARAM_NUMBER;i++) {
        if (strcmp(pardata[i].gui_code,id) == 0 ) {
            if (dat<pardata[i].valDown || dat>pardata[i].valUp) {
		errgen_state = pardata[i].err_set; errgen_aux = slave_get_title_with_node(node);
		g_idle_add(errgen_set_safe(NULL), NULL);
                return 0;
            }
            if (pardata[i].gui_code == "Velinc")
                slave_set_param("Velinc",i,dat);
            else {
                SDOR address = {pardata[i].index,pardata[i].subindex,pardata[i].type};
                if(pardata[i].type == 0x02) {
                    INTEGER8 data = (INTEGER8)dat;
                    if (!cantools_write_sdo(node,address,&data)) {
                        errgen_state = pardata[i].err_read; errgen_aux = slave_get_title_with_node(node);
			g_idle_add(errgen_set_safe(NULL), NULL);
                        return 0;
                    }
                } else if(pardata[i].type == 0x03) {
                    INTEGER16 data = (INTEGER16)dat;
                    if (!cantools_write_sdo(node,address,&data)) {
                        errgen_state = pardata[i].err_read; errgen_aux = slave_get_title_with_node(node);
			g_idle_add(errgen_set_safe(NULL), NULL);
                        return 0;
                    }
                } else if(pardata[i].type == 0x04) {
                    INTEGER32 data = (INTEGER32)dat;
                    if (!cantools_write_sdo(node,address,&data)) {
                        errgen_state = pardata[i].err_read; errgen_aux = slave_get_title_with_node(node);
			g_idle_add(errgen_set_safe(NULL), NULL);
                        return 0;
                    }
                } else if(pardata[i].type == 0x05) {
                    UNS8 data = (UNS8)dat;
                    if (!cantools_write_sdo(node,address,&data)) {
                        errgen_state = pardata[i].err_read; errgen_aux = slave_get_title_with_node(node);
			g_idle_add(errgen_set_safe(NULL), NULL);
                        return 0;
                    }
                } else if(pardata[i].type == 0x06) {
                    UNS16 data = (UNS16)dat;
                    if (!cantools_write_sdo(node,address,&data)) {
                        errgen_state = pardata[i].err_read; errgen_aux = slave_get_title_with_node(node);
			g_idle_add(errgen_set_safe(NULL), NULL);
                        return 0;
                    }
                } else if(pardata[i].type == 0x07) {
                    UNS32 data = (UNS32)dat;
                    if (!cantools_write_sdo(node,address,&data)) {
                        errgen_state = pardata[i].err_read; errgen_aux = slave_get_title_with_node(node);
			g_idle_add(errgen_set_safe(NULL), NULL);
                        return 0;
                    }
                } else {
                    printf("error get param");
                    exit(1);
                }
            }
        }
    }
    return 1;
}
/**
* Permet la lecture d'un des paramètres de la liste
* node
* i : indice du paramètre
* dat : pointeur valeur en INTEGER32
**/
int motor_get_param(UNS8 node,char* id,INTEGER32* dat) {
    int i;
    for (i=0;i<PARAM_NUMBER;i++) {
        if (strcmp(pardata[i].gui_code,id) == 0 ) {
            if (pardata[i].gui_code == "Velinc")
                *dat = (INTEGER32)slave_get_param_in_num("Velinc",i);
            else {
                SDOR address = {pardata[i].index,pardata[i].subindex,pardata[i].type};
                if(pardata[i].type == 0x02) {
                    INTEGER8 data;
                    if (!cantools_read_sdo(node,address,&data)) {
                        errgen_state = pardata[i].err_read; errgen_aux = slave_get_title_with_node(node);
			g_idle_add(errgen_set_safe(NULL), NULL);
                        return 0;
                    }
                    *dat = (INTEGER8)data;
                } else if(pardata[i].type == 0x03) {
                    INTEGER16 data;
                    if (!cantools_read_sdo(node,address,&data)) {
                        errgen_state = pardata[i].err_read; errgen_aux = slave_get_title_with_node(node);
			g_idle_add(errgen_set_safe(NULL), NULL);
                        return 0;
                    }
                    *dat = (INTEGER16)data;
                } else if(pardata[i].type == 0x04) {
                    INTEGER32 data;
                    if (!cantools_read_sdo(node,address,&data)) {
                        errgen_state = pardata[i].err_read; errgen_aux = slave_get_title_with_node(node);
			g_idle_add(errgen_set_safe(NULL), NULL);
                        return 0;
                    }
                    *dat = (INTEGER32)data;
                } else if(pardata[i].type == 0x05) {
                    UNS8 data;
                    if (!cantools_read_sdo(node,address,&data)) {
                        errgen_state = pardata[i].err_read; errgen_aux = slave_get_title_with_node(node);
			g_idle_add(errgen_set_safe(NULL), NULL);
                        return 0;
                    }
                    *dat = (UNS8)data;
                } else if(pardata[i].type == 0x06) {
                    UNS16 data;
                    if (!cantools_read_sdo(node,address,&data)) {
                        errgen_state = pardata[i].err_read; errgen_aux = slave_get_title_with_node(node);
			g_idle_add(errgen_set_safe(NULL), NULL);
                        return 0;
                    }
                    *dat = (UNS16)data;
                } else if(pardata[i].type == 0x07) {
                    UNS32 data;
                    if (!cantools_read_sdo(node,address,&data)) {
                        errgen_state = pardata[i].err_read; errgen_aux = slave_get_title_with_node(node);
			g_idle_add(errgen_set_safe(NULL), NULL);
                        return 0;
                    }
                    *dat = (UNS32)data;
                } else {
                    printf("error get param");
                    exit(1);
                }
            }
        }
    }
    return 1;
}
char* motor_get_param_title(char* id) {
    int i;
    for (i=0;i<PARAM_NUMBER;i++) {
        if (strcmp(pardata[i].gui_code,id) == 0 )
            return pardata[i].title;
    }
    printf("id : %s non trouvé\n",id);
    exit(1);
}
int motor_get_param_index(char* id) {
    int i;
    for (i=0;i<PARAM_NUMBER;i++) {
        if (strcmp(pardata[i].gui_code,id) == 0 )
            return i;
    }
    printf("id : %s non trouvé\n",id);
    exit(1);
}
UNS8 motor_get_param_type(char* id) {
    int i;
    for (i=0;i<PARAM_NUMBER;i++) {
        if (strcmp(pardata[i].gui_code,id) == 0 )
            return pardata[i].type;
    }
    printf("id : %s non trouvé\n",id);
    exit(1);
}

INTEGER16 motor_get_couple(INTEGER32 vel) {
    //cas positif
    if (vel == 0 && vel <= 1) return 0;
    else if (vel >= 2 && vel < 200000) return 150;
    else if (vel >= 200000 && vel < 280000) return 1000;
    else if (vel >= 280000 && vel < 300000) return 800;
    else if (vel >= 300000 && vel < 320000) return 600;
    else if (vel >= 320000 && vel < 341000) return 500;
    else if (vel >= 341000 ) return 400;
    //cas negatif
    else if (vel == -1) return 0;
    else if (vel <= -2 && vel > -200000) return -150;
    else if (vel <= -200000 && vel > -280000) return -1000;
    else if (vel <= -280000 && vel > -300000) return -800;
    else if (vel <= -300000 && vel > -320000) return -600;
    else if (vel <= -320000 && vel > -341000) return -500;
    else if (vel <= -341000 ) return -400;
    else return 0;
}

void motor_set_MotRot_Accel(void)
{
    int index_rot, index_trans;
    //printf("1\n");
    if(slave_get_indexList_from_Profile(PROF_VITROT, &index_rot)){
	errgen_state = ERR_LASER_ASSERV_GETINDEXROT;
	g_idle_add(errgen_set_safe(NULL), NULL);
        return;
    }

    UNS32 accel_T, decel_T;
    UNS32 accel_R, decel_R;
    SDOR Accel = {0x6083, 0x00, 0x07};
    SDOR Decel = {0x6084, 0x00, 0x07};
    //printf("2\n");
    slave_get_indexList_from_Profile(PROF_VITTRANS, &index_trans);
    //printf("3\n");
    if(index_trans >=0)
    {
        //lecture des accelerations
        if (!cantools_read_sdo(slave_get_node_with_index(index_trans),Accel,&accel_T)) {
            errgen_state = ERR_ROT_GET_ACCEL;
	    g_idle_add(errgen_set_safe(NULL), NULL);
            return;
        }
        if (!cantools_read_sdo(slave_get_node_with_index(index_trans),Decel,&decel_T)) {
            errgen_state = ERR_ROT_GET_DECEL;
	    g_idle_add(errgen_set_safe(NULL), NULL);
            return;
        }
    } else {//aucun moteurs vitesse translation
        accel_T = MOTOR_DEFAULT_MOTTRANSACCEL;
        decel_T = MOTOR_DEFAULT_MOTTRANSACCEL;
    }
    CaptAccel_T = accel_T;
    CaptDecel_T = decel_T;
    ConsAccel_T = accel_T;
    ConsDecel_T = decel_T;
    //calcul des acceleration rotation correspondante
    //printf("4\n");
    if(!laser_simu){
        printf("cas pas laser simu\n");
        if(laser_asserv_CalcRotAccel(&ml, &sl, &accel_T, &accel_R)){
            errgen_state = ERR_ROT_CALC_ACCEL;
	    g_idle_add(errgen_set_safe(NULL), NULL);
            return;
        }
        if(laser_asserv_CalcRotAccel(&ml, &sl, &decel_T, &decel_R)){
            errgen_state = ERR_ROT_CALC_ACCEL;
	    g_idle_add(errgen_set_safe(NULL), NULL);
            return;
        }
    } else {
        printf("cas laser simu\n");
        if(laser_asserv_CalcRotAccel(&lsim, NULL, &accel_T, &accel_R)){
            errgen_state = ERR_ROT_CALC_ACCEL;
	    g_idle_add(errgen_set_safe(NULL), NULL);
            return;
        }
        if(laser_asserv_CalcRotAccel(&lsim, NULL, &decel_T, &decel_R)){
            errgen_state = ERR_ROT_CALC_ACCEL;
	    g_idle_add(errgen_set_safe(NULL), NULL);
            return;
        }

    }

    //printf("5\n");
    //ecriture des accelerations
    if (accel_R > 1000){
        if (!cantools_write_sdo(slave_get_node_with_index(index_rot), Accel, &accel_R)){
            errgen_state = ERR_ROT_WRITE_ACCEL;
	    g_idle_add(errgen_set_safe(NULL), NULL);
            return;
        }
    }
    if (decel_R > 1000){
        if (!cantools_write_sdo(slave_get_node_with_index(index_rot), Decel, &decel_R)){
            errgen_state = ERR_ROT_WRITE_ACCEL;
	    g_idle_add(errgen_set_safe(NULL), NULL);
            return;
        }
    }
    //printf("6\n");
    CaptAccel_R = accel_R;
    CaptDecel_R = decel_R;
    ConsAccel_R = accel_R;
    ConsDecel_R = decel_R;
    //printf("A1 = %u, A2 = %u, D1 = %u, D2 = %u\n", accel_T, decel_T, accel_R, decel_R);

}
