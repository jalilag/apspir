/*
This file is part of CanFestival, a library implementing CanOpen Stack.

Copyright (C): Edouard TISSERANT and Francis DUPIN

See COPYING file for copyrights details.

This library is free software; you can redistribute it and/or
modify it under the terms of the GNU Lesser General Public
License as published by the Free Software Foundation; either
version 2.1 of the License, or (at your option) any later version.

This library is distributed in the hope that it will be useful,
but WITHOUT ANY WARRANTY; without even the implied warranty of
MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the GNU
Lesser General Public License for more details.

You should have received a copy of the GNU Lesser General Public
License along with this library; if not, write to the Free Software
Foundation, Inc., 59 Temple Place, Suite 330, Boston, MA  02111-1307  USA
*/
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
#include "od_callback.h"

// ajout 220116
#include "profile.h"
//fin ajout 220116

extern s_BOARD MasterBoard;
char* baud_rate="1M";
extern SLAVES_conf slaves[SLAVE_NUMBER_LIMIT];
extern int SLAVE_NUMBER;

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
    //ajout 260116
    od_callback_define();
    //fin ajout 260116
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
    /** PDOR CONFIGURATION **/
    int j;
    UNS8 trans =0xFF;
    for (i=0; i < SLAVE_NUMBER; i++) {
        for (j=0; j<4; j++){
            dat = 0x40000180 + 0x100*j  + slave_get_node_with_index(i);
            if (!cantools_write_local(0x1400+(4*i+j),0x01,&dat,sizeof(UNS32))) {
                errgen_set(ERR_MASTER_CONFIG_PDOR,NULL);
                printf("dat %x\n",dat);
                return 0;
            }
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
        //debut ajout 220116
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

         } else if (l == PROF_COUPLROT || l == PROF_COUPLTRANS){
            //mapping PDOT maitre
            if(l == PROF_COUPLROT){
                if (!cantools_PDO_map_local_config( 0x1A00+4*i, 0x20300010,0)){
                    errgen_set(ERR_MASTER_CONFIG_MAP, NULL);
                    return 0;
                }
            } else if (l == PROF_COUPLTRANS) {
                if (!cantools_PDO_map_local_config( 0x1A00+4*i, 0x20310010,0)){
                    errgen_set(ERR_MASTER_CONFIG_MAP, NULL);
                    return 0;
                }
            }
            //mapping PDOR maitre
            UNS32 velIndex = 0x20260020 + 0x10000*i;
            if(!cantools_PDO_map_local_config( 0x1601+4*i,velIndex,0)){
                errgen_set(ERR_MASTER_CONFIG_MAP, NULL);
                return 0;
            }
         } else if (l == PROF_LIBRE){
         }
         //config PDOT1 pour chaque moteur
         if( l == PROF_VITROT || l == PROF_VITTRANS || l == PROF_COUPLROT || l == PROF_COUPLTRANS){
            UNS16 inhibit = 0;
            UNS16 event = 5000;
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
        }
        //fin ajout 220116
    }
    return 1;
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
        if (slave_get_param_in_num("Active",i))
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
     sendPDOevent(d);
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
//ajout 220116
static unsigned long MasterSyncCount = 0;
extern unsigned long CaptPos_R_Time;
//fin ajout 220116
void SpirallingMaster_post_TPDO(CO_Data* d) {

//ajout 220116
if (MasterSyncCount%10 == 0){
        printf("\nSpirallingMaster_post_TPDO MasterSyncCount = %lu \n", MasterSyncCount);
        if(slave_profile_exist(PROF_VITROT)){
            printf("MotRotVars :ConsigneVitesse = %d, CaptureVitesse = %d, CapturePosition = %d, TimePositionCapture = %lu, Acceleration = %u, Deceleration = %u\n",
                ConsVit_R, CaptVit_R, CaptPos_R, CaptPos_R_Time, CaptAccel_R, CaptDecel_R);
        }
        if(slave_profile_exist(PROF_COUPLROT)){
            printf("MotRotCoupl : Consigne Couple: %d, Capture Couple: %d\n", ConsCoupl_R, CaptCoupl_R);
        }
        if(slave_profile_exist(PROF_VITTRANS)){
            printf("MotTransVars :ConsigneVitesse = %d, CaptureVitesse = %d, Acceleration = %u, Deceleration = %u\n",
                ConsVit_T, CaptVit_T, CaptAccel_T, CaptDecel_T);
        }
        if(slave_profile_exist(PROF_COUPLTRANS)){
            printf("MotTransCoupl : Consigne Couple: %d, Capture Couple: %d, Capture Vitesse: %d\n", ConsCoupl_T, CaptCoupl_T,
                slave_get_param_in_num("Velocity",slave_get_index_with_node(slave_get_node_with_profile(PROF_COUPLTRANS))));
        }
        printf("\n");
    }
    MasterSyncCount++;
//fin ajout 220116
}

void SpirallingMaster_post_SlaveBootup(CO_Data* d, UNS8 nodeid) {
	printf("SpirallingMaster_post_SlaveBootup %x\n", nodeid);
    if (slave_get_param_in_num("Active",slave_get_index_with_node(nodeid)))
        slave_set_param("State",slave_get_index_with_node(nodeid),STATE_CONFIG);
}
