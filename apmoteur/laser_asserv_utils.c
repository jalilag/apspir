#include <stdio.h>
#include <string.h>
#include "slave.h"
#include "profile.h"
#include "errgen.h"
#include "cantools.h"
#include "keyword.h"
#include "motor.h"
#include "SpirallingMaster.h"
#include "laser_asserv.h"
#include "laser_asserv_cst.h"

extern UNS16 errgen_state;
extern char* errgen_aux;
extern int SLAVE_NUMBER;

static int laser_asserv_utils_set_MotRot_Accel(void)
{
    int index_rot, index_trans;
    if(slave_get_indexList_from_Profile(PROF_VITROT, &index_rot)){
        errgen_state = ERR_LASER_ASSERV_GETINDEXROT;
        g_idle_add(errgen_set_safe(NULL), NULL);
        return 1;
    }

    UNS32 accel_T, decel_T;
    UNS32 accel_R, decel_R;
    SDOR Accel = {0x6083, 0x00, 0x07};
    SDOR Decel = {0x6084, 0x00, 0x07};
    slave_get_indexList_from_Profile(PROF_VITTRANS, &index_trans);
    if(index_trans >=0)
    {
        //lecture des accelerations
        if (!cantools_read_sdo(slave_get_node_with_index(index_trans),Accel,&accel_T)) {
            errgen_state = ERR_ROT_GET_ACCEL;
            g_idle_add(errgen_set_safe(NULL), NULL);
            return 1;
        }
        if (!cantools_read_sdo(slave_get_node_with_index(index_trans),Decel,&decel_T)) {
            errgen_state = ERR_ROT_GET_DECEL;
            g_idle_add(errgen_set_safe(NULL), NULL);
            return 1;
        }
    } else {//aucun moteurs vitesse translation
        errgen_state = ERR_LASER_ASSERV_GETINDEXTRANS;
        g_idle_add(errgen_set_safe(NULL), NULL);
        return 1;
    }
    CaptAccel_T = accel_T;
    CaptDecel_T = decel_T;
    ConsAccel_T = accel_T;
    ConsDecel_T = decel_T;
    //calcul des acceleration rotation correspondante

    if(laser_asserv_CalcRotAccel(&ml, &sl, &accel_T, &accel_R)){
        errgen_state = ERR_ROT_CALC_ACCEL;
        g_idle_add(errgen_set_safe(NULL), NULL);
        return 1;
    }
    if(laser_asserv_CalcRotAccel(&ml, &sl, &decel_T, &decel_R)){
        errgen_state = ERR_ROT_CALC_ACCEL;
        g_idle_add(errgen_set_safe(NULL), NULL);
        return 1;
    }


    //ecriture des accelerations
    if (accel_R > 1000){
        if (!cantools_write_sdo(slave_get_node_with_index(index_rot), Accel, &accel_R)){
            errgen_state = ERR_ROT_WRITE_ACCEL;
            g_idle_add(errgen_set_safe(NULL), NULL);
            return 1;
        }
    }
    if (decel_R > 1000){
        if (!cantools_write_sdo(slave_get_node_with_index(index_rot), Decel, &decel_R)){
            errgen_state = ERR_ROT_WRITE_ACCEL;
            g_idle_add(errgen_set_safe(NULL), NULL);
            return 1;
        }
    }
    CaptAccel_R = accel_R;
    CaptDecel_R = decel_R;
    ConsAccel_R = accel_R;
    ConsDecel_R = decel_R;
    return 0;
}


void laser_asserv_utils_StartRot(void){
    struct laser_data d;
    int motrotvel_index;
    int res;
    if (slave_profile_exist(PROF_VITROT)){
        if(!run_asserv){
            //recuperer les parametres du module asserv
            if(laser_asserv_cst_recup()) laser_asserv_cst_set_default();

            /**Start Moteurs Rotation**/
            //demarrage des moteurs rotation
            int i;
            for (i=0;i<SLAVE_NUMBER;i++){
                if (slave_get_profile_with_index(i) == PROF_VITROT || slave_get_profile_with_index(i) == PROF_COUPLROT){
                    if (motor_get_state((UNS16)slave_get_param_in_num("Power",i)) == SON && slave_get_param_in_num("Active",i)){
                        motor_start(slave_get_node_with_index(i),1);
                    }
                }
            }

            /**Get Laser Start Position et check laser**/
            if(!((errgen_laserState & MASTER_NOT_STARTED) && (errgen_laserState & SLAVE_NOT_STARTED))){
                if(laser_asserv_GetStartPosition(&ml, &sl)) {
                    errgen_state = ERR_LASER_ASSERV_GETSTARTPOS;
                    g_idle_add(errgen_set_safe(NULL), NULL);
                    return;
                }
                printf("START DISTANCE = %lu\n", Laser_Asserv_GetStartDistance());

                /**Get_Ref_Position MotRot**/
                if (slave_get_indexList_from_Profile(PROF_VITROT, &motrotvel_index)){
                    printf("slave_get_indexList error in laser_asserv_utils_StartRot\n");
                    return;//internal error
                }
                SDOR RefPos_R = {0x6064, 0x00, int32};
                if (!cantools_read_sdo(slave_get_node_with_index(motrotvel_index),RefPos_R,&MotRot_Ref_Position)) {
                    errgen_state = ERR_SLAVE_CONFIG_ROT_REFPOS;
                    g_idle_add(errgen_set_safe(NULL), NULL);
                    return;
                }
                /**Set MotRot Acceleration**/
                res = laser_asserv_utils_set_MotRot_Accel();
                if(res) return;

                /**LANCE ASSERVISSEMENT**/
                if(laser_asserv_rotation_lance()){
                    errgen_state = ERR_LASER_ASSERV_START;
                    g_idle_add(errgen_set_safe(NULL),NULL);
                }
            } else {
                errgen_state = ERR_LASER_INIT_FATAL;
                g_idle_add(errgen_set_safe(NULL), NULL);
            }
        }
    } else {
        errgen_state = ERR_LASER_ASSERV_GETINDEXROT;//erreur aucun mot rot vitesse défini
        g_idle_add(errgen_set_safe(NULL), NULL);
        return;
    }
}


void laser_asserv_utils_StopRot(void){

    if(run_asserv){
        if(laser_asserv_rotation_stop()){
            errgen_state = ERR_LASER_ASSERV_STOP;
            g_idle_add(errgen_set_safe(NULL), NULL);
        }
    }
    int i;
    for (i=0;i<SLAVE_NUMBER;i++){
        if (slave_get_profile_with_index(i) == PROF_VITROT || slave_get_profile_with_index(i) == PROF_COUPLROT){
            if (motor_get_state((UNS16)slave_get_param_in_num("Power",i)) == OENABLED && slave_get_param_in_num("Active",i)){
                motor_start(slave_get_node_with_index(i),0);
            }
        }
    }

}

//on relance la rotation sans acquisition des constantes de démarrage.
void laser_asserv_utils_resumeRot(void){
    int i;
    if (slave_profile_exist(PROF_VITROT)){
        if(!run_asserv){
            /**Start Moteurs Rotation**/
            for (i=0;i<SLAVE_NUMBER;i++){
                if (slave_get_profile_with_index(i) == PROF_VITROT || slave_get_profile_with_index(i) == PROF_COUPLROT){
                    if (motor_get_state((UNS16)slave_get_param_in_num("Power",i)) == SON && slave_get_param_in_num("Active",i)){
                        motor_start(slave_get_node_with_index(i),1);
                    }
                }
            }
            //correction unique(utile dans le cas ou on est sortie de la tolérance)
            if(laser_asserv_UniqCorrPos(&ml, &sl)){
                errgen_state = ERR_LASER_ASSERV_PRECORR;
                g_idle_add(errgen_set_safe(NULL), NULL);
            }
            /**LANCE ASSERVISSEMENT**/
            if(laser_asserv_rotation_lance()){
                errgen_state = ERR_LASER_ASSERV_START;
                g_idle_add(errgen_set_safe(NULL),NULL);
            }
        }
    }
}


void laser_asserv_utils_StartTrslLent(void){
    int i;
    if(!run_asserv){
        if(laser_asserv_GetStartPosition(&ml, &sl)) {
            errgen_state = ERR_LASER_ASSERV_GETSTARTPOS;
            g_idle_add(errgen_set_safe(NULL), NULL);
            return;
        }
    }
    if(run_trsl){
        if(laser_asserv_translation_stop()){
            errgen_state = ERR_LASER_ASSERV_STOP;
            g_idle_add(errgen_set_safe(NULL), NULL);
            return;
        }
    }
    //fixer sens moteur trsl. Attention dépend de comment sont placer les moteurs.
    for (i=0; i<SLAVE_NUMBER;i++) {
        if ((slave_get_profile_with_index(i) == PROF_VITTRANS  || slave_get_profile_with_index(i) == PROF_COUPLTRANS) && slave_get_param_in_num("Active",i)) {
            if(!motor_forward(slave_get_node_with_index(i),0)) {
                errgen_state = ERR_MOTOR_FORWARD; errgen_aux = slave_get_title_with_index(i);
                g_idle_add(errgen_set_safe(NULL), NULL);
                return;
            }
        }
    }
    //demarrer moteurs trsl
    for (i=0;i<SLAVE_NUMBER;i++){
        if (slave_get_profile_with_index(i) == PROF_VITTRANS || slave_get_profile_with_index(i) == PROF_COUPLTRANS){
            if (motor_get_state((UNS16)slave_get_param_in_num("Power",i)) == SON && slave_get_param_in_num("Active",i)){
                motor_start(slave_get_node_with_index(i),1);
            }
        }
    }
    if(laser_asserv_translation_lance_lent()){
        errgen_state = ERR_LASER_ASSERV_START;
        g_idle_add(errgen_set_safe(NULL), NULL);
    }
}
void laser_asserv_utils_StartTrslRapide(void){
    int i;
    if(!run_asserv){
        if(laser_asserv_GetStartPosition(&ml, &sl)) {
            errgen_state = ERR_LASER_ASSERV_GETSTARTPOS;
            g_idle_add(errgen_set_safe(NULL), NULL);
            return;
        }
    }
    if(run_trsl){
        if(laser_asserv_translation_stop()){
            errgen_state = ERR_LASER_ASSERV_STOP;
            g_idle_add(errgen_set_safe(NULL), NULL);
            return;
        }
    }
    //fixer sens moteur trsl. Attention dépend de comment sont placer les moteurs.
    for (i=0; i<SLAVE_NUMBER;i++) {
        if ((slave_get_profile_with_index(i) == PROF_VITTRANS  || slave_get_profile_with_index(i) == PROF_COUPLTRANS) && slave_get_param_in_num("Active",i)) {
            if(!motor_forward(slave_get_node_with_index(i),0)) {
                errgen_state = ERR_MOTOR_FORWARD; errgen_aux = slave_get_title_with_index(i);
                g_idle_add(errgen_set_safe(NULL), NULL);
                return;
            }
        }
    }
    //demarrer moteurs trsl
    for (i=0;i<SLAVE_NUMBER;i++){
        if (slave_get_profile_with_index(i) == PROF_VITTRANS || slave_get_profile_with_index(i) == PROF_COUPLTRANS){
            if (motor_get_state((UNS16)slave_get_param_in_num("Power",i)) == SON && slave_get_param_in_num("Active",i)){
                motor_start(slave_get_node_with_index(i),1);
            }
        }
    }
    if(laser_asserv_translation_lance_rapide()){
        errgen_state = ERR_LASER_ASSERV_START;
        g_idle_add(errgen_set_safe(NULL), NULL);
    }
}
void laser_asserv_utils_reStartTrslLent(void){
    int i;
    if(run_trsl){
        if(laser_asserv_translation_stop()){
            errgen_state = ERR_LASER_ASSERV_STOP;
            g_idle_add(errgen_set_safe(NULL), NULL);
            return;
        }
    }
    for (i=0;i<SLAVE_NUMBER;i++){
        if (slave_get_profile_with_index(i) == PROF_VITTRANS || slave_get_profile_with_index(i) == PROF_COUPLTRANS){
            if (motor_get_state((UNS16)slave_get_param_in_num("Power",i)) == SON && slave_get_param_in_num("Active",i)){
                motor_start(slave_get_node_with_index(i),1);
            }
        }
    }
    if(laser_asserv_translation_lance_lent()){
        errgen_state = ERR_LASER_ASSERV_START;
        g_idle_add(errgen_set_safe(NULL), NULL);
    }
}
void laser_asserv_utils_reStartTrslRapide(void){
    int i;
    if(run_trsl){
        if(laser_asserv_translation_stop()){
            errgen_state = ERR_LASER_ASSERV_STOP;
            g_idle_add(errgen_set_safe(NULL), NULL);
            return;
        }
    }
    for (i=0;i<SLAVE_NUMBER;i++){
        if (slave_get_profile_with_index(i) == PROF_VITTRANS || slave_get_profile_with_index(i) == PROF_COUPLTRANS){
            if (motor_get_state((UNS16)slave_get_param_in_num("Power",i)) == SON && slave_get_param_in_num("Active",i)){
                motor_start(slave_get_node_with_index(i),1);
            }
        }
    }
    if(laser_asserv_translation_lance_rapide()){
        errgen_state = ERR_LASER_ASSERV_START;
        g_idle_add(errgen_set_safe(NULL), NULL);
    }
}
void laser_asserv_utils_StopTrsl(void){
    int i;
    if(run_trsl){
        if(laser_asserv_translation_stop()){
            errgen_state = ERR_LASER_ASSERV_STOP;
            g_idle_add(errgen_set_safe(NULL), NULL);
        }
    }
    for (i=0;i<SLAVE_NUMBER;i++){
        if (slave_get_profile_with_index(i) == PROF_VITTRANS || slave_get_profile_with_index(i) == PROF_COUPLTRANS){
            if (motor_get_state((UNS16)slave_get_param_in_num("Power",i)) == OENABLED && slave_get_param_in_num("Active",i)){
                motor_start(slave_get_node_with_index(i),0);
            }
        }
    }
}
void laser_asserv_utils_TrslManualInc(void){
    int velinc;

    int mottrans_index;
    if (slave_profile_exist(PROF_VITTRANS)){
        slave_get_indexList_from_Profile(PROF_VITTRANS,&mottrans_index);
        velinc = slave_get_param_in_num("Velinc", mottrans_index);
    } else if (slave_profile_exist(PROF_COUPLTRANS)){
        slave_get_indexList_from_Profile(PROF_COUPLTRANS, &mottrans_index);
        velinc = slave_get_param_in_num("Velinc", mottrans_index);
    } else {
        errgen_state = ERR_LASER_ASSERV_GETINDEXTRANS;
        g_idle_add(errgen_set_safe(NULL),NULL);
        return;
    }

    ConsVit_T += velinc;

    //rotation pas active
    if(!run_asserv) {
        printf("butvelup cantools_ApplyVelTrans\n");
        cantools_ApplyVelTrans(ConsVit_T);
        return;
    }
    //rotation active
    if(run_asserv){
        if(laser_asserv_User_Velocity_Change(&ml, &sl, ConsVit_T)){
            errgen_state = ERR_LASER_ASSERV_SENS;
            g_idle_add(errgen_set_safe(NULL),NULL);
        }
    }
}
void laser_asserv_utils_TrslManualDec(void){
    int velinc;

    int mottrans_index;
    if (slave_profile_exist(PROF_VITTRANS)){
        slave_get_indexList_from_Profile(PROF_VITTRANS,&mottrans_index);
        velinc = slave_get_param_in_num("Velinc", mottrans_index);
    } else if (slave_profile_exist(PROF_COUPLTRANS)){
        slave_get_indexList_from_Profile(PROF_COUPLTRANS, &mottrans_index);
        velinc = slave_get_param_in_num("Velinc", mottrans_index);
    } else {
        errgen_state = ERR_LASER_ASSERV_GETINDEXTRANS;
        g_idle_add(errgen_set_safe(NULL),NULL);
        return;
    }

    ConsVit_T -= velinc;

   //rotation pas active
    if(!run_asserv) {
        cantools_ApplyVelTrans(ConsVit_T);
        return;
    }
    //rotation active
    if(run_asserv){
        if(laser_asserv_User_Velocity_Change(&ml, &sl, ConsVit_T)){
            errgen_state = ERR_LASER_ASSERV_SENS;
            g_idle_add(errgen_set_safe(NULL),NULL);
        }
    }
}
int laser_asserv_utils_RotIsActive(void){
    if (run_asserv) return 1;
    return 0;
}
int laser_asserv_utils_TransIsActive(void){
    if(run_trsl) return 1;
    return 0;
}
void laser_asserv_utils_StartPose(char * Vitesse){
    laser_asserv_utils_StartRot();
    if(strcmp(Vitesse, "rapide") == 0) laser_asserv_utils_StartTrslRapide();
    else if (strcmp(Vitesse, "lent") == 0) laser_asserv_utils_StartTrslLent();
}
void laser_asserv_utils_StopPose(void){
    laser_asserv_utils_StopTrsl();
    laser_asserv_utils_StopRot();
}
void laser_asserv_utils_ContinuePose(char * Vitesse){
    laser_asserv_utils_resumeRot();
    if(strcmp(Vitesse, "rapide") == 0) laser_asserv_utils_reStartTrslRapide();
    else if (strcmp(Vitesse, "lent") == 0) laser_asserv_utils_reStartTrslLent();
}
