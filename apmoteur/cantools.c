#include <time.h>
#include <stdio.h>
#include <stdlib.h>
#include "cantools.h"
#include "strtools.h"
#include "canfestival.h"
#include "gui.h"
#include "keyword.h"
#include "master.h"
#include "slave.h"
#include "SpirallingControl.h"
#include "motor.h"
#include "od_callback.h"
#include "errgen.h"

//ajout 22/01/16
#include "profile.h"
#include "lsstools.h" //!!!
#include "serialtools.h"
//fin ajout 220116
// CONSTANTS
#define MAX_SDO_ERROR 2
#define SDO_TIMEOUT 3

static int SDO_step_error = 0;

extern SLAVES_conf slaves[SLAVE_NUMBER_LIMIT];
extern pthread_mutex_t lock_slave;
extern int run_init, SLAVE_NUMBER;
extern INTEGER32 old_voltage[SLAVE_NUMBER_LIMIT];

extern UNS16 errgen_state;
extern char* errgen_aux;
/**
* Lecture d'une SDO
**/
int cantools_read_sdo(UNS8 nodeid,SDOR sdo, void* data) {
	UNS32 abortCode;
    UNS32 size;

	UNS8 res;
    UNS8 datatype = 0;
    if (sdo.type == 0x01) {
        size = 1;
    } else if (sdo.type == 0x02) {
        size = 1;
    } else if (sdo.type == 0x03) {
        size = 2;
    } else if (sdo.type == 0x04) {
        size = 4;
    } else if (sdo.type == 0x05) {
        size = 1;
    } else if (sdo.type == 0x06) {
		size = 2;
    } else if (sdo.type == 0x07) {
		size = 4;
    } else if (sdo.type == 0x09) {
        size = 4;
    }
	readNetworkDict(&SpirallingMaster_Data, nodeid, sdo.index, sdo.subindex,datatype, 0);
	res = getReadResultNetworkDict(&SpirallingMaster_Data, nodeid, data, &size, &abortCode);

	if (res == SDO_UPLOAD_IN_PROGRESS || res == SDO_DOWNLOAD_IN_PROGRESS ) {
        time_t deb = time (NULL);
        time_t fin;
 		while (res == SDO_UPLOAD_IN_PROGRESS || res == SDO_DOWNLOAD_IN_PROGRESS) {
            usleep(10000);
            fin = time (NULL);
			res = getReadResultNetworkDict(&SpirallingMaster_Data, nodeid, data, &size, &abortCode);
			if (difftime(fin,deb) > SDO_TIMEOUT) {
                res = SDO_ABORTED_INTERNAL;
                gui_push_state(strtools_concat(SDO_READING_TIMEOUT, " (",strtools_gnum2str(&sdo.index,0x06)," | ",strtools_gnum2str(&sdo.subindex,0x05),")",NULL));
            }
		}
    }
    closeSDOtransfer(&SpirallingMaster_Data, nodeid, SDO_CLIENT);
    if (res == SDO_FINISHED) {
        SDO_step_error=0;
        return 1;
	} else {
        SDO_step_error++;
        if (SDO_step_error < MAX_SDO_ERROR) {
            cantools_read_sdo(nodeid,sdo,data);
        } else {
            SDO_step_error=0;
            gui_push_state(strtools_concat(SDO_READING_ERROR, " (",strtools_gnum2str(&sdo.index,0x06)," | ",strtools_gnum2str(&sdo.subindex,0x05),")",NULL));
            return 0;
        }
	}
}

/**
* Ecriture d'une SDO
**/
int cantools_write_sdo(UNS8 nodeid,SDOR sdo, void* data) {
	UNS32 abortCode;
	UNS32 size;
	UNS8 res;
	UNS8 datatype = 0;

	if (sdo.type == 0x04 || sdo.type == 0x07) {
		size = 4;
 	} else if (sdo.type == 0x03 || sdo.type == 0x06 ) {
		size = 2;
 	} else if (sdo.type == 0x01 || sdo.type == 0x02 || sdo.type == 0x05 ) {
		size = 1;
    } else if (sdo.type == 0X09) {
		size = 8;
        datatype = visible_string;
	} else {
        printf("Type error"); exit(EXIT_FAILURE);
	}

	writeNetworkDict(&SpirallingMaster_Data, nodeid, sdo.index, sdo.subindex, size, datatype, data, 0);
	res = getWriteResultNetworkDict(&SpirallingMaster_Data, nodeid, &abortCode);
	if (res == SDO_DOWNLOAD_IN_PROGRESS || res == SDO_UPLOAD_IN_PROGRESS) {
        time_t deb = time (NULL);
        time_t fin;
		while (res == SDO_DOWNLOAD_IN_PROGRESS || res == SDO_UPLOAD_IN_PROGRESS) {
            usleep(10000);
            fin = time (NULL);
 			res = getWriteResultNetworkDict(&SpirallingMaster_Data, nodeid, &abortCode);
			if (difftime(fin,deb) > SDO_TIMEOUT) {
                res = SDO_ABORTED_INTERNAL;
                gui_push_state(SDO_WRITING_TIMEOUT);
            }
		}
    }
	closeSDOtransfer(&SpirallingMaster_Data, nodeid, SDO_CLIENT);
    if (res == SDO_FINISHED) {
        SDO_step_error=0;
        return 1;
	} else {
        SDO_step_error++;
        if (SDO_step_error < MAX_SDO_ERROR) {
            cantools_write_sdo(nodeid,sdo,data);
        } else {
            SDO_step_error=0;
            gui_push_state(strtools_concat(SDO_WRITING_ERROR, " (",strtools_gnum2str(&sdo.index,0x06)," | ",strtools_gnum2str(&sdo.subindex,0x05),")",NULL));
        return 0;
        }
    }
}

/**
* Ecriture dans le dictionnaire local
**/
int cantools_write_local(UNS16 Mindex, UNS8 Msubindex, void* data, UNS32 datsize) {
	UNS32 res;
	res = writeLocalDict(&SpirallingMaster_Data, Mindex, Msubindex, data, &datsize,1);
    if (res == OD_SUCCESSFUL) {
        return 1;
    } else {
        gui_push_state(strtools_concat(LOCAL_WRITING_ERROR, " -> ",strtools_gnum2str(&Mindex,0x06)," (", " | ",
        strtools_gnum2str(&Msubindex,0x05),")",NULL));
        return 0;
    }
}

/**
* Configuration du type de transmission des PDOs
**/
int cantools_PDO_trans(UNS8 nodeID, UNS16 index, UNS8 trans, UNS16 inhibit, UNS16 event) {
    UNS16 cobID;
    if (index == 0x1400) cobID = 0x0200;
    if (index == 0x1401) cobID = 0x0300;
    if (index == 0x1402) cobID = 0x0400;
    if (index == 0x1403) cobID = 0x0500;
    if (index == 0x1800) cobID = 0x0180;
    if (index == 0x1801) cobID = 0x0280;
    if (index == 0x1802) cobID = 0x0380;
    if (index == 0x1803) cobID = 0x0480;
    UNS32 EnabledCobid = 0x40000000 + cobID + nodeID;
    UNS32 DisabledCobid = 0x80000000 + cobID + nodeID;
    SDOR SDO_trans = {index,0x02,0x05};
    SDOR SDO_cobid = {index,0x01,0x07};
    SDOR SDO_inhib = {index,0x03,0x06};
    SDOR SDO_event = {index,0x05,0x06};

    // Désactivation pour modification
    if(!cantools_write_sdo(nodeID,SDO_cobid,&DisabledCobid)) {
        printf("Erreur : desactivation\n");
        return 0;
    }
    // Modification transmission
    if(!cantools_write_sdo(nodeID,SDO_trans,&trans)) {
        printf("Erreur : trans\n");
        return 0;
    }
    // Modification de l'inhibit
    if(!cantools_write_sdo(nodeID,SDO_inhib,&inhibit)) {
        printf("Erreur : inhib\n");
        return 0;
    }
    // Modification de l'event timer
    if(!cantools_write_sdo(nodeID,SDO_event,&event)) {
        printf("Erreur : event\n");
        return 0;
    }
    // Activation transmission
    if (!cantools_write_sdo(nodeID,SDO_cobid,&EnabledCobid)) {
        printf("Erreur : activation %#.8x\n", EnabledCobid);
        return 0;
    }
    return 1;
}


/**
* Configuration des PDO map d'un esclave
* UNS32 PDOMapData : index + subindex + size : 60400020
* UNS16 PDOParamData : 0x0180 / 0x0280 ...
**/
int cantools_PDO_map_config(UNS8 nodeID, UNS16 PDOMapIndex,...) {
    UNS16 PDOParamData;
    if (PDOMapIndex == 0x1A00) PDOParamData = 0x0180;
    if (PDOMapIndex == 0x1A01) PDOParamData = 0x0280;
    if (PDOMapIndex == 0x1A02) PDOParamData = 0x0380;
    if (PDOMapIndex == 0x1A03) PDOParamData = 0x0480;

    if (PDOMapIndex == 0x1600) PDOParamData = 0x0200;
    if (PDOMapIndex == 0x1601) PDOParamData = 0x0300;
    if (PDOMapIndex == 0x1602) PDOParamData = 0x0400;
    if (PDOMapIndex == 0x1603) PDOParamData = 0x0500;

    va_list ap;
    va_start(ap,PDOMapIndex);
    UNS32 arg = va_arg(ap,UNS32);

    SDOR PDOMapAddress = {PDOMapIndex,0x00,0x05};
    // Désactivation de la transmission pour modification
    SDOR PDOParamAddress = {PDOMapIndex-0x0200,0x01,0x07};
    UNS32 PDOParamVal = 0x80000000 + PDOParamData + nodeID;
    if(!cantools_write_sdo(nodeID,PDOParamAddress,&PDOParamVal)) {
        printf("Erreur : désactivation\n");
        return 0;
    }
    // Mise à 0 du subindex pour activer le mode edition
    UNS8 i =0x00;
    if(!cantools_write_sdo(nodeID,PDOMapAddress,&i)) {
        printf("Erreur : mise a 0 subindex\n");
        return 0;
    }
    // Ecriture de tout les index en paramètres

    while (arg != 0) {
        i++;
        SDOR PDOMapAddress2 = {PDOMapIndex,i,0x07};
        if(!cantools_write_sdo(nodeID,PDOMapAddress2,&arg)) {
            printf("ERREUR ecriture index : %#.8x\n",arg);
            return 0;
        }
        arg = va_arg(ap,UNS32);
    }
    va_end(ap);

    // Sortie du mode édition, Ecriture nombre d'index
    if(!cantools_write_sdo(nodeID,PDOMapAddress,&i)) {
        printf("Erreur : Ecriture nombre subindex\n");
        return 0;
    }

    // Activation PDO trans
    PDOParamVal = 0x40000000 + PDOParamData + nodeID;
    if(!cantools_write_sdo(nodeID,PDOParamAddress,&PDOParamVal)) {
        printf("Erreur : Acivation\n");
        return 0;
    }
    return 1;
}
//debut ajout 220116
int cantools_PDO_map_local_config(UNS16 PDOMapIndex, ...){

    va_list ap;
    va_start(ap,PDOMapIndex);
    UNS32 arg = va_arg(ap,UNS32);
    UNS8 i=0;

    // Ecriture de tout les index en paramètres

    while (arg != 0) {
        i++;
        if(!cantools_write_local(PDOMapIndex, i, &arg, sizeof(UNS32) )) {
            printf("ERREUR ecriture index : %#.8x\n",arg);
            return 0;
        }
        arg = va_arg(ap,UNS32);
    }
    va_end(ap);

    return 1;

}

int cantools_desactive_PDO_reseau(UNS8 nodeID, UNS16 index){
    UNS16 cobID;
    if (index == 0x1400) cobID = 0x0200;
    if (index == 0x1401) cobID = 0x0300;
    if (index == 0x1402) cobID = 0x0400;
    if (index == 0x1403) cobID = 0x0500;
    if (index == 0x1800) cobID = 0x0180;
    if (index == 0x1801) cobID = 0x0280;
    if (index == 0x1802) cobID = 0x0380;
    if (index == 0x1803) cobID = 0x0480;

    UNS32 DisabledCobid = 0x80000000 + cobID + nodeID;
    SDOR SDO_cobid = {index,0x01,0x07};
    // Désactivation
    if(!cantools_write_sdo(nodeID,SDO_cobid,&DisabledCobid)) {
        printf("Erreur : desactivation\n");
        return 0;
    }
    UNS16 PDOMapIndex = index + 0x200;
    SDOR PDOMapAddress = {PDOMapIndex,0x00,0x05};
    UNS8 i =0x00;
    if(!cantools_write_sdo(nodeID,PDOMapAddress,&i)) {
        printf("Erreur : mise a 0 subindex\n");
        return 0;
    }
    return 1;

}

int rot_motor_index;
gpointer cantools_waitStopRot_Thread (gpointer arg)
{
    int apply0 = 1;
    while (abs(CaptVit_R) > 3){
        usleep(100);
        if(abs(ConsVit_R) > 1){
            apply0 = 0;
            break;
        }
    }
    if(apply0){
        ConsVit_R = 0;
        EnterMutex();
        PDOEventTimerAlarm(&SpirallingMaster_Data, slave_get_param_in_num("GetPDOT0Num", *(int*)arg));
        LeaveMutex();

    }

    g_thread_exit(NULL);
}

int trans_motor_index;
gpointer cantools_waitStopTrans_Thread (gpointer arg)
{
    int apply0 = 1;
    while (abs(CaptVit_T) > 3){
        usleep(100);
        if(abs(ConsVit_T) > 1){
            apply0 = 0;
            break;
        }
    }
    if(apply0){
        ConsVit_T = 0;
        EnterMutex();
        PDOEventTimerAlarm(&SpirallingMaster_Data, slave_get_param_in_num("GetPDOT0Num", *(int*)arg));
        LeaveMutex();
    }

    g_thread_exit(NULL);
}

//title = "Rotation" ou "Translation"
void cantools_ApplyVelRot(INTEGER32 vit)
{
    int i;
    UNS32 pdonum;
    INTEGER32 vtm, vtc;
    INTEGER32 data;
    //printf("7\n");
    for (i=0;i<SLAVE_NUMBER; i++){
        if(slave_get_profile_with_index(i) == PROF_VITROT){
            //printf("8\n");

            EnterMutex();
            vtm = CaptVit_R;
            vtc = ConsVit_R;
            if(vtc!=vit){
                vtc = vit;
                ConsVit_R = vit;
            }
            LeaveMutex();

            pdonum = slave_get_param_in_num("GetPDOT0Num",i);
            //gerer les problèmes de non-application de l'acceleration au demarrage et a l'arrêt
            if(vtm == 0 || vtc == 0){
                if((vtm == 0) && (vtc != 0)){
                    data = vtc/abs(vtc);

                    EnterMutex();
                    ConsVit_R = data;
                    PDOEventTimerAlarm(&SpirallingMaster_Data, pdonum);
                    ConsVit_R = vtc;
                    LeaveMutex();

                    usleep(1000);
                } else if ((vtc == 0) && (vtm != 0)) {
                    data = vtm/abs(vtm);

                    EnterMutex();
                    ConsVit_R = data;
                    PDOEventTimerAlarm(&SpirallingMaster_Data, pdonum);
                    LeaveMutex();
                    rot_motor_index = i;
                    if(g_thread_try_new(NULL, cantools_waitStopRot_Thread, &rot_motor_index, NULL) == NULL){
                        //bloquer le moteur direct
                        EnterMutex();
                        ConsVit_R = vtc;
                        PDOEventTimerAlarm(&SpirallingMaster_Data, pdonum);
                        LeaveMutex();
                    }

                    return;
                } else return;//la vitesse de 0 est déjà appliquée.

            }
            EnterMutex();
            PDOEventTimerAlarm(&SpirallingMaster_Data, pdonum);
            LeaveMutex();
        }
        else if(slave_get_profile_with_index(i) == PROF_COUPLROT){
        /**VIA PDO**/
            EnterMutex();
            if(vit != ConsVit_R) ConsVit_R = vit;
            if (ConsVit_R >= 0){
                if (ConsVit_R <= MOTOR_ROT_MAX_VEL) ConsCoupl_R = (INTEGER16)(MOTOR_ROT_COUPL_DEFAULT + (850-MOTOR_ROT_COUPL_DEFAULT)*(double)(ConsVit_R)/MOTOR_ROT_MAX_VEL);
                else ConsCoupl_R = 850;
            } else if (ConsVit_R < 0) {
                if (ConsVit_R >= -MOTOR_ROT_MAX_VEL) ConsCoupl_R = (INTEGER16)(-MOTOR_ROT_COUPL_DEFAULT + (850-MOTOR_ROT_COUPL_DEFAULT)*(double)(ConsVit_R)/MOTOR_ROT_MAX_VEL);
                else ConsCoupl_R = -850;
            }
            PDOEventTimerAlarm(&SpirallingMaster_Data, (UNS32)slave_get_param_in_num("GetPDOT0Num", i));
            LeaveMutex();
        /**VIA SDO**/
        /*
            SDOR sdoTargetCoupl = {0x6071,0,int16};
            INTEGER16 data;
            EnterMutex();
            if (ConsVit_R <= MOTOR_ROT_MAX_VEL) data = (INTEGER16)(MOTOR_ROT_COUPL_DEFAULT + (1000-MOTOR_ROT_COUPL_DEFAULT)*(double)(ConsVit_R/MOTOR_ROT_MAX_VEL));
            else data = 1000;
            LeaveMutex();
            if(!cantools_write_sdo(slave_get_node_with_index(i), sdoTargetCoupl, &data)){
                errgen_set(ERR_SAVE_TORQUE, slave_get_title_with_index(i));
            }
            */
        }
    }
}

void cantools_ApplyVelTrans(INTEGER32 vit)
{
    int i;
    UNS32 pdonum;
    INTEGER32 vtm, vtc;
    INTEGER32 data;
    printf("7\n");
    for (i=0;i<SLAVE_NUMBER; i++){
        if(slave_get_profile_with_index(i) == PROF_VITTRANS || slave_get_profile_with_index(i) == PROF_COUPLTRANS){
            slave_set_param("Vel2send", i, vit);
        }
        if(slave_get_profile_with_index(i) == PROF_VITTRANS){
            EnterMutex();
            vtm = CaptVit_T;
            vtc = ConsVit_T;
            if(vtc!=vit){
                vtc = vit;
                ConsVit_T = vit;
            }
            LeaveMutex();

            pdonum = slave_get_param_in_num("GetPDOT0Num",i);
            //printf("pdonum = %d", slave_get_param_in_num("Vel2sendTransPDONum",i));
            //gerer les problèmes de non-application de l'acceleration au demarrage et a l'arrêt

            if(vtm == 0 || vtc == 0){
                if((vtm == 0) && (vtc != 0)){
                    data = vtc/abs(vtc);

                    EnterMutex();
                    ConsVit_T = data;
                    PDOEventTimerAlarm(&SpirallingMaster_Data, pdonum);
                    ConsVit_T = vtc;
                    LeaveMutex();

                    usleep(1000);
                } else if ((vtc == 0) && (vtm != 0)) {
                    data = vtm/abs(vtm);

                    EnterMutex();
                    ConsVit_T = data;
                    PDOEventTimerAlarm(&SpirallingMaster_Data, pdonum);
                    LeaveMutex();
                    trans_motor_index = i;
                    if(g_thread_try_new(NULL, cantools_waitStopTrans_Thread, &trans_motor_index, NULL) == NULL){
                        //bloquer le moteur direct
                        EnterMutex();
                        ConsVit_T = vtc;
                        PDOEventTimerAlarm(&SpirallingMaster_Data, pdonum);
                        LeaveMutex();
                    }

                    return;
                } else return;//la vitesse de 0 est déjà appliquée.

            }
            EnterMutex();
            PDOEventTimerAlarm(&SpirallingMaster_Data, pdonum);
            LeaveMutex();
        }
        else if(slave_get_profile_with_index(i) == PROF_COUPLTRANS){
            printf("9\n");
            /**VIA PDO**/
            EnterMutex();
            if(ConsVit_T != vit) ConsVit_T = vit;
            ConsCoupl_T = motor_get_couple(vit);
            /*
            if(ConsVit_T >= 0) {
                if (ConsVit_T <= MOTOR_TRANS_MAX_VEL) ConsCoupl_T = (INTEGER16)(MOTOR_TRANS_COUPL_DEFAULT + (850-MOTOR_TRANS_COUPL_DEFAULT)*(double)(ConsVit_T)/MOTOR_TRANS_MAX_VEL);
                else ConsCoupl_T = 850;
            } else if (ConsVit_T < 0) {
                if (ConsVit_T >= -MOTOR_TRANS_MAX_VEL) ConsCoupl_T = (INTEGER16)(-MOTOR_TRANS_COUPL_DEFAULT + (850-MOTOR_TRANS_COUPL_DEFAULT)*(double)(ConsVit_T)/MOTOR_TRANS_MAX_VEL);
                else ConsCoupl_T = -850;
            }*/
            printf("ConsCoupl_T =%d\n", ConsCoupl_T);
            PDOEventTimerAlarm(&SpirallingMaster_Data, (UNS32)slave_get_param_in_num("GetPDOT0Num", i));
            LeaveMutex();

            /**VIA SDO**/
            /*
            SDOR sdoTargetCoupl = {0x6071,0,int16};
            INTEGER16 data;
            EnterMutex();
            if(ConsVit_T != vit) ConsVit_T = vit;
            if (ConsVit_T <= MOTOR_TRANS_MAX_VEL) data = (INTEGER16)(MOTOR_TRANS_COUPL_DEFAULT + (1000-MOTOR_TRANS_COUPL_DEFAULT)*(double)(ConsVit_T/MOTOR_TRANS_MAX_VEL));
            else data = 1000;
            LeaveMutex();
            printf("data = %d", data);
            if(!cantools_write_sdo(slave_get_node_with_index(i), sdoTargetCoupl, &data)){
                errgen_set(ERR_SAVE_TORQUE, slave_get_title_with_index(i));
            }
            */
        }
    }
}


//fin ajout 220116
/**
* INITIALISATION LOOP
**/
gpointer cantools_init_loop(gpointer data) {
    int i,MasterState = 0;
    INTEGER32 j = 0;
    while (run_init == -1) sleep(1);
    //ajout 230116
    serialtools_minimum_init();
    //fin ajout 230116
    master_init();

    // Chargement de l'interface
    g_idle_add(slave_gui_load_state,NULL);
    g_timeout_add(500,keyword_maj,NULL);

    while (run_init == 1) {
        j++;
        MasterState = 0;
        for (i=0; i<SLAVE_NUMBER;i++) {
            if (slave_get_param_in_num("State",i) == STATE_LSS_CONFIG) {
                MasterState = 1;
            }
        }
        if(MasterState) {
            if (getState(&SpirallingMaster_Data) == Operational)
                setState(&SpirallingMaster_Data, Pre_operational);
        } else {
            if (getState(&SpirallingMaster_Data) == Pre_operational)
                setState(&SpirallingMaster_Data, Operational);
        }
        for (i=0; i<SLAVE_NUMBER;i++) {
            // Configuration LSS
            if (slave_get_param_in_num("State",i) == STATE_LSS_CONFIG) {
                printf("\n\nSLAVE STATE : %s \nnode %d index %d \n\n",slave_get_param_in_char("State",i), slave_get_node_with_index(i),i);
                masterSendNMTstateChange (&SpirallingMaster_Data, slave_get_node_with_index(i), NMT_Stop_Node);
                if (!lsstools_loop(slave_get_node_with_index(i),0)) {
                    slave_set_param("StateError",i,ERROR_STATE_LSS);
                    errgen_state = ERR_SLAVE_CONFIG_LSS;
                    errgen_aux = slave_get_title_with_index(i);
                    g_idle_add(errgen_set_safe(NULL),NULL);
                    slave_set_param("Active",i,STATE_DISCONNECTED);
                    printf("State disc\n");
                } else {
                    slave_set_param("State",i,STATE_PREOP);
                }
            }
            // Boot up
            if (MasterState == 0 && slave_get_param_in_num("State",i) == STATE_PREOP) {
                printf("\n\nSLAVE STATE : %s \nnode %d index %d \n\n",slave_get_param_in_char("State",i), slave_get_node_with_index(i),i);
                masterSendNMTstateChange (&SpirallingMaster_Data, slave_get_node_with_index(i), NMT_Reset_Node);
            }
            // Configuration PreOp
            if (slave_get_param_in_num("State",i) == STATE_CONFIG) {
                printf("\n\nSLAVE STATE : %s \nnode %d index %d \n\n",slave_get_param_in_char("State",i), slave_get_node_with_index(i),i);
                if(slave_config(slave_get_node_with_index(i))) {
                    slave_set_param("State",i,STATE_OP);
                } else {
                    slave_set_param("StateError",i,ERROR_STATE_CONFIG);
                    errgen_state = ERR_SLAVE_CONFIG;
                    errgen_aux = slave_get_title_with_index(i);
                    g_idle_add(errgen_set_safe(NULL),NULL);
                    slave_set_param("Active",i,0);
                }
            }
            // Passage en mode operational
            if (slave_get_param_in_num("State",i) == STATE_OP) {
                printf("\n\nSLAVE STATE : %s \nnode %d index %d \n\n",slave_get_param_in_char("State",i), slave_get_node_with_index(i),i);
                masterSendNMTstateChange (&SpirallingMaster_Data, slave_get_node_with_index(i), NMT_Start_Node);
            }
            // Mise des moteurs en mode opérationnel
            if (slave_get_param_in_num("State",i) == STATE_SON) {
                printf("\n\nSLAVE STATE : %s \nnode %d index %d \n\n",slave_get_param_in_char("State",i), slave_get_node_with_index(i),i);
                if(motor_switch_on(slave_get_node_with_index(i))) {
                    slave_set_param("State",i,STATE_READY);
                } else {
                    slave_set_param("StateError",i,ERROR_STATE_SON);
                    errgen_state = ERR_SLAVE_CONFIG_MOTOR_SON;
                    errgen_aux = slave_get_title_with_index(i);
                    g_idle_add(errgen_set_safe(NULL),NULL);
                    slave_set_param("Active",i,0);
                }
            }
           // Détection d'une baisse de voltage
            if (slave_get_param_in_num("Active",i) == 1) {
                if (slave_get_param_in_num("Volt",i) > old_voltage[i])
                    old_voltage[i] = slave_get_param_in_num("Volt",i);
                if (old_voltage[i] > 0 && slave_get_param_in_num("Volt",i) < 0.50*old_voltage[i]) {
                    slave_set_param("Active",i,0);
                    old_voltage[i] = 0;
                    errgen_state = ERR_MOTOR_LOW_VOLTAGE;
                    errgen_aux = slave_get_param_in_char("SlaveTitle",i);
                    g_idle_add(errgen_set_safe(NULL),NULL);
                    slave_set_param("StateError",i,ERROR_STATE_VOLTAGE);
                }
            }
        }
        usleep(1000000);
    }
    return 0;
}
