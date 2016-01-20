
#include "master.h"
#include "slave.h"
#include <stdio.h>
#include <string.h>
#include <stdlib.h>
#include <gtk/gtk.h>
#include "SpirallingControl.h"
#include "keyword.h"
#include "cantools.h"
#include "lsstools.h"
#include "gui.h"
#include "od_callback.h"
#include "errgen.h"
#include "profile.h"
extern int SLAVE_NUMBER;
extern s_BOARD MasterBoard;
char* baud_rate="1M";

/*****************************************************************************/

/**
* Initialisation du maitre
* Appelé dans le programme principal
**/
void master_init() {
    *SpirallingMaster_Data.bDeviceNodeId = 0x01;
    setState(&SpirallingMaster_Data, Initialisation);
}
/**
* START AND STOP SYNC
* 0 : erreur
* 1 : ok
**/
static int master_start_sync(int start) {
    UNS32 dat = 0x00000080;
    if (start)
        dat = 0x40000080;
    if(!cantools_write_local(0x1005,0x00,&dat,sizeof(UNS32))) {
        return 0;
    } else {
        return 1;
    }
}
/**
* Master configuration
**/
static int master_config() {
    UNS32 dat;
    int i = 0;
    /** SYNC PERIOD **/
    if(!master_start_sync(0)) { // Arret de la synchro
        errgen_set(ERR_MASTER_START_SYNC,NULL);
        return 0;
    }
    dat = CYCLE_PERIOD; // Affectation de la periode de synchro
    if(!cantools_write_local(0x1006,0x00,&dat,sizeof(UNS32))) {
        errgen_set(ERR_MASTER_SET_PERIOD,NULL);
        return 0;
    }
    /** HEARTBEAT CONSUMER **/
    for (i=0; i < SLAVE_NUMBER; i++) {
        dat = HB_CONS_BASE + (slave_get_node_with_index(i)-1)*HB_CONS_BASE_OFFSET; // 500 + (n-1)*50
        if(!cantools_write_local(0x1016,slave_get_node_with_index(i)-1,&dat,sizeof(UNS32))) {
            errgen_set(ERR_MASTER_SET_HB_CONS,NULL);
            return 0;
        }
    }
    /** PDOR et PDOT CONFIGURATION **/
    int j;
    UNS8 trans =0xFF;
    for (i=0; i < SLAVE_NUMBER; i++) {
        for (j=0; j<4; j++){
        //reception
            dat = 0x40000180 + 0x100*j  + slave_get_node_with_index(i);
            if (!cantools_write_local(0x1400+(4*i+j),0x01,&dat,sizeof(UNS32))) {
                errgen_set(ERR_MASTER_CONFIG_PDOR,NULL);
                printf("dat %x\n",dat);
                return 0;
            }
            //transmission
            dat = 0x40000200 + 0x100*j  + slave_get_node_with_index(i);
            if (!cantools_write_local(0x1800+(4*i+j),0x01,&dat,sizeof(UNS32))) {
                errgen_set(ERR_MASTER_CONFIG_PDOT,NULL);
                printf("dat %x\n",dat);
                return 0;
            }
            if (!cantools_write_local(0x1800+(4*i+j),0x02,&trans,sizeof(UNS8))) {
                errgen_set(ERR_MASTER_CONFIG_PDOT,NULL);
                printf("trans %x\n",trans);
                return 0;
            }
        }
        int l = slave_get_profile_with_index(i);
        if (l == PROF_VITROT){
            //mapping PDOT maitre
            if (!cantools_PDO_map_local_config( 0x1A00+4*i, 0x201A0020, 0)){
                errgen_set(ERR_MASTER_CONFIG_MAP, NULL);
                return 0;
            }
            //mapping PDOR maitre
            //la vitesse
            if(!cantools_PDO_map_local_config( 0x1601+4*i, 0x201C0020, 0x201D0020, 0)){
                errgen_set(ERR_MASTER_CONFIG_MAP, NULL);
                return 0;
            }
            //params PDOR maitre specifique profil
            UNS16 inhibit = 0;
            UNS16 event = 50000;
            if (!cantools_write_local(0x1800+(4*i),0x03,&inhibit,sizeof(UNS16))) {
                errgen_set(ERR_MASTER_CONFIG_PDOT,NULL);
                printf("inhibit rot %x\n",inhibit);
                return 0;
            }
            if (!cantools_write_local(0x1800+(4*i),0x05,&event,sizeof(UNS16))) {
                errgen_set(ERR_MASTER_CONFIG_PDOT,NULL);
                printf("event rot %x\n",event);
                return 0;
            }
        } else if (l == PROF_VITTRANS){
            //mapping PDOT maitre
             if (!cantools_PDO_map_local_config( 0x1A00+4*i, 0x20190020,0)){
                errgen_set(ERR_MASTER_CONFIG_MAP, NULL);
                return 0;
            }
            //mapping PDOR maitre
            if(!cantools_PDO_map_local_config( 0x1601+4*i, 0x201B0020, 0)){
                errgen_set(ERR_MASTER_CONFIG_MAP, NULL);
                return 0;
            }
            UNS16 inhibit = 0;
            UNS16 event = 50000;
            if (!cantools_write_local(0x1800+(4*i),0x03,&inhibit,sizeof(UNS16))) {
                errgen_set(ERR_MASTER_CONFIG_PDOT,NULL);
                printf("inhibit trans %x\n",inhibit);
                return 0;
            }
            if (!cantools_write_local(0x1800+(4*i),0x05,&event,sizeof(UNS16))) {
                errgen_set(ERR_MASTER_CONFIG_PDOT,NULL);
                printf("event trans %x\n",event);
                return 0;
            }
         } else if (l == PROF_COUPLROT){
         } else if (l == PROF_COUPLTRANS){
         } else if (l == PROF_LIBRE){
         }


    }
    /**mise en 0xFF des PDO inutiles**/
    trans = 0xFF;
    for (i=SLAVE_NUMBER; i<SLAVE_NUMBER_LIMIT; i++){
        for (j=0; j<4; j++){
            if(!cantools_write_local(0x1800+4*i+j, 0x02, &trans, sizeof(UNS8))){
                errgen_set(ERR_MASTER_CONFIG_PDOT,NULL);
                printf("dat %x\n",dat);
                return 0;
            }
        }
    }

    return 1;

    /**PDOT map CONFIGURATION (de base: rien)**/
}


void SpirallingMaster_heartbeatError(CO_Data* d, UNS8 heartbeatID) {
	printf("SpirallingMaster_heartbeatError %d\n", heartbeatID);
}


void SpirallingMaster_initialisation(CO_Data* d) {
	printf("SpirallingMaster_initialisation\n");
    if(!master_config()) {     //configuration locale
        errgen_set(ERR_MASTER_CONFIG,NULL);
    }
}

void SpirallingMaster_preOperational(CO_Data* d) {
	printf("SpirallingMaster_preOperational\n");
    int i;
    if (!master_start_sync(1)) { // Démarrage de la synchro
        errgen_set(ERR_MASTER_START_SYNC,NULL);
    }
    for (i=0; i<SLAVE_NUMBER; i++) {
        printf("%d",i);
        if (slave_get_param_in_num("Active",i));
            slave_set_param("State",i,STATE_LSS_CONFIG);
    }
}

void SpirallingMaster_operational(CO_Data* d) {
	printf("SpirallingMaster_operational\n");
}

void SpirallingMaster_stopped(CO_Data* d) {
	printf("SpirallingMaster_stopped\n");
}

void SpirallingMaster_post_sync(CO_Data* d) {
     //sendPDOevent(d);
}

void SpirallingMaster_post_emcy(CO_Data* d, UNS8 nodeID, UNS16 errCode, UNS8 errReg) {
	printf("Master received EMCY message. Node: %2.2x  ErrorCode: %4.4x  ErrorRegister: %2.2x\n", nodeID, errCode, errReg);
}

void SpirallingMaster_post_SlaveStateChange(CO_Data* d, UNS8 nodeId, e_nodeState newNodeState) {
    switch(newNodeState) {
        case Initialisation:
            printf("Node %u state is now  : Initialisation\n", nodeId);
        break;
        case Pre_operational:
            printf("Node %u state is now  : Preop\n", nodeId);

        break;
        case Disconnected:
            printf("\n\nNode %u state is now  : Disconnected\n\n", nodeId);
        break;
        case Connecting:
            printf("Node %u state is now  : Connecting\n", nodeId);
        break;
        case Preparing:
            printf("Node %u state is now  : Preparing\n", nodeId);
        break;
        case Stopped:
            printf("Node %u state is now  : Stopped\n", nodeId);
        break;
        case Operational:
            printf("Node %u state is now  : Operational\n", nodeId);
            if (slave_get_param_in_num("Active",slave_get_index_with_node(nodeId)))
                slave_set_param("State",slave_get_index_with_node(nodeId),STATE_SON);
        break;
        case Unknown_state:
            printf("Node %u state is now  : Unknown_state\n", nodeId);
        break;
        default:
            printf("Error : node %u unexpected state : %d\n", nodeId, newNodeState);
        break;
    }
}
static unsigned long MasterSyncCount = 0;
void SpirallingMaster_post_TPDO(CO_Data* d) {
    if (MasterSyncCount%10 == 0){
        printf("\nSpirallingMaster_post_TPDO MasterSyncCount = %lu \n", MasterSyncCount);

        printf("MotRotVars :ConsigneVitesse = %d, CaptureVitesse = %d, CapturePosition = %d, Acceleration = %u, Deceleration = %u\n",
            ConsVit_R, CaptVit_R, CaptPos_R, CaptAccel_R, CaptDecel_R);

        printf("MotTransVars :ConsigneVitesse = %d, CaptureVitesse = %d, Acceleration = %u, Deceleration = %u\n",
            ConsVit_T, CaptVit_T, CaptAccel_T, CaptDecel_T);
    }

    MasterSyncCount++;

}

void SpirallingMaster_post_SlaveBootup(CO_Data* d, UNS8 nodeid) {
	printf("SpirallingMaster_post_SlaveBootup %x\n", nodeid);
    if (slave_get_param_in_num("Active",slave_get_index_with_node(nodeid)))
        slave_set_param("State",slave_get_index_with_node(nodeid),STATE_CONFIG);
}
