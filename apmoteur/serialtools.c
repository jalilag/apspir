#include "serialtools.h"
#include "laser_asserv.h"
#include "errgen.h"
#include "keyword.h"
#include "slave.h"
#include "gui.h"
#include "strtools.h"

int serialtools_in_reinit_laser = 0;
extern int SLAVE_NUMBER;

gboolean serialtools_init_laser(gpointer err_init) {
    unsigned int err_l;
    unsigned int laser_state;
    struct laser_data d;

    if(serialtools_in_reinit_laser) {
        errgen_laserState = 0;
    }

    /**configuring serial port access permissions**/
    if(Laser_serial_config()){
        //errgen_set(ERR_LASER_SERIAL_CONFIG, NULL);
        return FALSE;//erreur
    }

    /**INIT 2 LASER**/
    if(err_l = Laser_Init2Laser(&ml, &sl)){
        if(err_l == ERR_LASER_INIT_FATAL){
            //errgen_set(ERR_LASER_INIT_FATAL, NULL);
            errgen_laserState = ERR_LASER_INIT_FATAL;
            return FALSE;//erreur
        }
        if(err_l == LASER_MASTER_INIT_ERROR){
            //errgen_set(LASER_ERROR(LASER_MASTER_INIT_ERROR), NULL);
            errgen_laserState = MASTER_NOT_STARTED;
            if(Laser_Start1Laser(&sl)){
                //errgen_set(ERR_LASER_INIT_FATAL, NULL);
                errgen_laserState = ERR_LASER_INIT_FATAL;
                return FALSE;//erreur
            } else {
                gui_image_set("imgLaserGStatInfo", "gtk-no", 2);
                gui_image_set("imgLaserDStatInfo", "gtk-yes", 2);
                gui_label_set("labLaser1StatInfo", MASTER_NOT_STARTED_LABEL);
                gui_label_set("labLaser2StatInfo", LASER_STATUS_OK_LABEL);
            }
        } else if (err_l == LASER_MASTER_INIT_ERROR2){
            //errgen_set(LASER_ERROR(LASER_MASTER_INIT_ERROR2), NULL);
            errgen_laserState = MASTER_NOT_STARTED;
            if(Laser_Start1Laser(&sl)){
                errgen_laserState = ERR_LASER_INIT_FATAL;
                //errgen_set(ERR_LASER_INIT_FATAL, NULL);
                return FALSE;//erreur
            } else {
                gui_image_set("imgLaserGStatInfo", "gtk-no", 2);
                gui_image_set("imgLaserDStatInfo", "gtk-yes", 2);
                gui_label_set("labLaser1StatInfo", MASTER_NOT_STARTED_LABEL);
                gui_label_set("labLaser2StatInfo", LASER_STATUS_OK_LABEL);
            }
        } else if (err_l == LASER_SLAVE_INIT_ERROR){
            //errgen_set(LASER_ERROR(LASER_SLAVE_INIT_ERROR), NULL);
            errgen_laserState = SLAVE_NOT_STARTED;
            if(Laser_Start1Laser(&ml)){
                //errgen_set(ERR_LASER_INIT_FATAL, NULL);
                errgen_laserState = ERR_LASER_INIT_FATAL;
                return FALSE;//erreur
            } else {
                gui_image_set("imgLaserGStatInfo", "gtk-yes", 2);
                gui_image_set("imgLaserDStatInfo", "gtk-no", 2);
                gui_label_set("labLaser1StatInfo", LASER_STATUS_OK_LABEL);
                gui_label_set("labLaser2StatInfo", SLAVE_NOT_STARTED_LABEL);
            }
        } else if (err_l == LASER_SLAVE_INIT_ERROR2){
            //errgen_set(LASER_ERROR(LASER_SLAVE_INIT_ERROR2), NULL);
            errgen_laserState = SLAVE_NOT_STARTED;
            if(Laser_Start1Laser(&ml)){
                //errgen_set(ERR_LASER_INIT_FATAL, NULL);
                errgen_laserState = ERR_LASER_INIT_FATAL;
                return FALSE;//erreur
            } else {
                gui_image_set("imgLaserGStatInfo", "gtk-yes", 2);
                gui_image_set("imgLaserDStatInfo", "gtk-no", 2);
                gui_label_set("labLaser1StatInfo", LASER_STATUS_OK_LABEL);
                gui_label_set("labLaser2StatInfo", SLAVE_NOT_STARTED_LABEL);
            }
        }
    } else {
        /**START 2 LASER**/
        if(err_l = Laser_Start2Laser(&ml, &sl)){
            if(err_l == ERR_LASER_INIT_FATAL){
                if(err_l & LASER_GETPOSOFFSET_ERROR){
                    //errgen_set(LASER_ERROR(LASER_GETPOSOFFSET_ERROR), NULL);
                }
                errgen_laserState = ERR_LASER_INIT_FATAL;
                //errgen_set(ERR_LASER_INIT_FATAL, NULL);
                return FALSE;//erreur
            }
            if(err_l == LASER_MASTER_START_ERROR){
                //errgen_set(LASER_ERROR(LASER_MASTER_START_ERROR), NULL);
                errgen_laserState = MASTER_NOT_STARTED;
                gui_image_set("imgLaserGStatInfo", "gtk-no", 2);
                gui_image_set("imgLaserDStatInfo", "gtk-yes", 2);
                gui_label_set("labLaser1StatInfo", MASTER_NOT_STARTED_LABEL);
                gui_label_set("labLaser2StatInfo", LASER_STATUS_OK_LABEL);
            } else if(err_l == LASER_SLAVE_START_ERROR){
                //errgen_set(LASER_ERROR(LASER_SLAVE_START_ERROR), NULL);
                errgen_laserState = SLAVE_NOT_STARTED;
                gui_image_set("imgLaserGStatInfo", "gtk-yes", 2);
                gui_image_set("imgLaserDStatInfo", "gtk-no", 2);
                gui_label_set("labLaser1StatInfo", LASER_STATUS_OK_LABEL);
                gui_label_set("labLaser2StatInfo", SLAVE_NOT_STARTED_LABEL);
            }
        }
        if(!err_l){
            gui_image_set("imgLaserDStatInfo", "gtk-yes", 2);
            gui_image_set("imgLaserGStatInfo", "gtk-yes", 2);
            gui_label_set("labLaser1StatInfo", LASER_STATUS_OK_LABEL);
            gui_label_set("labLaser2StatInfo", LASER_STATUS_OK_LABEL);
            errgen_laserState = 0;
        }
    }
//    if(g_thread_try_new(NULL,serialtools_check_laser_500ms,NULL,NULL) == NULL){
//        //errgen_set(ERR_LASER_STARTCHECKTHREAD, NULL);
//    }
    g_timeout_add(500,serialtools_check_laser_500ms,NULL);


    //printf ("err_l = %x", err_l);
   return FALSE;//ok

}

gboolean serialtools_checkPlotState_laser(gpointer err_check) {
    unsigned int err_l;
    unsigned int laser_state;
    unsigned long mes1, mes2;
    struct laser_data d;
    int i;

    if ((errgen_laserState == ERR_LASER_FATAL) || ((errgen_laserState & MASTER_NOT_STARTED) && (errgen_laserState & SLAVE_NOT_STARTED))){
        gui_image_set("imgLaserDStatInfo", "gtk-no", 2);
        gui_image_set("imgLaserGStatInfo", "gtk-no", 2);
        printf("14\n");
        return;
    }

    if(laser_state = Laser_GetData(&ml, &sl, &d)){
        if((laser_state & ERR_LASER_FATAL) == ERR_LASER_FATAL){
            printf("15\n");
            //errgen_set(LASER_ERROR(ERR_LASER_FATAL), NULL);
            gui_image_set("imgLaserDStatInfo", "gtk-no", 2);
            gui_image_set("imgLaserGStatInfo", "gtk-no", 2);
            errgen_laserState |= ERR_LASER_FATAL;
            //arrêt des moteurs
            for (i=0; i<SLAVE_NUMBER; i++) {
                motor_start(slave_get_node_with_index(i),0);
            }
            return;
            //todo arrêter les moteurs
        }
        if(laser_state & MASTER_FAILED_TIMEOUT){
            printf("16\n");
            gui_label_set("labLaser1StatInfo", MASTER_FAILED_TIMEOUT_LABEL);
            gui_image_set("imgLaserGStatInfo", "gtk-no", 2);
            errgen_laserState |= MASTER_FAILED_TIMEOUT;
            }
        else if(laser_state & SLAVE_FAILED_TIMEOUT){
            printf("17\n");
            gui_label_set("labLaser2StatInfo", MASTER_FAILED_TIMEOUT_LABEL);
            gui_image_set("imgLaserDStatInfo", "gtk-no", 2);
            errgen_laserState |= SLAVE_FAILED_TIMEOUT;
        }
        else if(laser_state & MASTER_FAILED_DATCONSISTENCY){
            printf("18\n");
            gui_label_set("labLaser1StatInfo", MASTER_FAILED_DATCONSISTENCY_LABEL);
            gui_image_set("imgLaserGStatInfo", "gtk-no", 2);
            errgen_laserState |= MASTER_FAILED_DATCONSISTENCY;
        }
        else if(laser_state & SLAVE_FAILED_DATCONSISTENCY){
            printf("19\n");
            gui_label_set("labLaser2StatInfo", SLAVE_FAILED_DATCONSISTENCY_LABEL);
            gui_image_set("imgLaserDStatInfo", "gtk-no", 2);
            errgen_laserState |= SLAVE_FAILED_DATCONSISTENCY;
        }
        if(laser_state & MASTER_NOT_STARTED){
            printf("20\n");
            if(!(errgen_laserState & MASTER_NOT_STARTED)){
                printf("21\n");
                //errgen_set(LASER_ERROR(MASTER_NOT_STARTED), NULL);
                gui_label_set("labLaser1StatInfo", MASTER_NOT_STARTED_LABEL);
                gui_image_set("imgLaserGStatInfo", "gtk-no", 2);
                errgen_laserState |= MASTER_NOT_STARTED;
            }
        }
        else if (laser_state & SLAVE_NOT_STARTED){
            printf("22\n");
            if(!(errgen_laserState & SLAVE_NOT_STARTED)){
                printf("23\n");
                //errgen_set(LASER_ERROR(SLAVE_NOT_STARTED), NULL);
                gui_label_set("labLaser2StatInfo", SLAVE_NOT_STARTED_LABEL);
                gui_image_set("imgLaserDStatInfo", "gtk-no", 2);
                errgen_laserState |= SLAVE_NOT_STARTED;
            }
        }
    } else {
        printf("24\n");
        gui_label_set("labLaser1StatInfo", LASER_STATUS_OK_LABEL);
        gui_label_set("labLaser2StatInfo", LASER_STATUS_OK_LABEL);
        gui_image_set("imgLaserDStatInfo", "gtk-yes", 2);
        gui_image_set("imgLaserGStatInfo", "gtk-yes", 2);
        errgen_laserState = 0;
    }

    //printf("laser_state checkplotstate= %x, laser_errgen_laserState = %x\n", laser_state, errgen_laserState);

//affichage mesures non vérifiées
    Laser_GetUnverifiedData(&sl, &mes1);
    gui_label_set("labMesureLaser2", strtools_gnum2str_all_decimal((UNS32*)&mes1, uint32));

    Laser_GetUnverifiedData(&ml, &mes2);
    gui_label_set("labMesureLaser1", strtools_gnum2str_all_decimal((UNS32*)&mes2, uint32));

    /**Decommenter pour affichage uniquement si la valeur est correcte**/
/*
    if(!((laser_state & SLAVE_FAILED_TIMEOUT) || (laser_state & SLAVE_FAILED_DATCONSISTENCY) || (laser_state & SLAVE_NOT_STARTED))){
        Laser_GetUnverifiedData(&sl, &mes1);
        gui_label_set("labMesureLaser2", strtools_gnum2str_all_decimal((UNS32*)&mes1, 0x50));
    }
    else gui_label_set("labMesureLaser2", "NA");

    if(!((laser_state & MASTER_FAILED_TIMEOUT) || (laser_state & MASTER_FAILED_DATCONSISTENCY) || (laser_state & MASTER_NOT_STARTED))){
        Laser_GetUnverifiedData(&ml, &mes2);
        gui_label_set("labMesureLaser1", strtools_gnum2str_all_decimal((UNS32*)&mes2, 0x50));
    }
    else gui_label_set("labMesureLaser1", "NA");
*/
    //Affichage mesure vérifiée
    gui_label_set("labMesureLaserGlobal", strtools_gnum2str_all_decimal((UNS32*)&(d.mes), uint32));

    return FALSE;

}

static int run_laser_check = 0;
gpointer serialtools_check_laser_500ms(gpointer data) {
    run_laser_check = 1;
    while(run_laser_check) {
        //check laser state

        if (!serialtools_in_reinit_laser) g_idle_add(serialtools_checkPlotState_laser, NULL);
        if((errgen_laserState & ERR_LASER_FATAL) == ERR_LASER_FATAL){
            errgen_set(LASER_ERROR(ERR_LASER_FATAL), NULL);
            run_laser_check = 0;
        }
        //Affichage erreur position cables estimée
        //gui_label_set("labEstimErrPosCabl", "NA");
    }
    return TRUE;
}

int err_exit_laser = 0;
gboolean serialtools_exit_laser(gpointer err_exit) {
    printf("cantools exit laser: errgen_laserState = %x\n", errgen_laserState);
    run_laser_check = 0;
    *(int*)err_exit = 0;

    if(!(errgen_laserState & MASTER_NOT_STARTED)){
        printf("In exit ml\n");
        if (Laser_Exit1Laser( &ml )<0){
            printf("Master laser exit error\n");
            //errgen_set(LASER_ERROR(LASER_MASTER_EXIT_ERROR), NULL);
            if(err_exit != NULL)
                *(int*)err_exit = 1;
        }
    }
    if(!(errgen_laserState & SLAVE_NOT_STARTED)){
        printf("In exit sl\n");
        if (Laser_Exit1Laser( &sl )<0){
            printf("Slave laser exit error\n");
            //errgen_set(LASER_ERROR(LASER_SLAVE_EXIT_ERROR), NULL);
            if(err_exit != NULL)
                *(int*)err_exit = 2;
        }
    }
    if(errgen_laserState == ERR_LASER_INIT_FATAL){
        if (err_exit != NULL)
            *(int*)err_exit = 3;
    }

    gui_label_set("labLaser1StatInfo", MASTER_NOT_STARTED_LABEL);
    gui_label_set("labLaser2StatInfo", SLAVE_NOT_STARTED_LABEL);
    gui_image_set("imgLaserDStatInfo", "gtk-no", 2);
    gui_image_set("imgLaserGStatInfo", "gtk-no", 2);
    errgen_laserState = MASTER_NOT_STARTED | SLAVE_NOT_STARTED;
    return FALSE;
}

int serialtools_reinit_laser(void){
    int res;
    serialtools_in_reinit_laser = 1;
    if(errgen_laserState){
        //serialtools_exit_laser(NULL);
        if(run_laser_check) run_laser_check = 0;
        g_idle_add(serialtools_exit_laser,&err_exit_laser);
        if(err_exit_laser == 1){
            errgen_set(LASER_ERROR(LASER_MASTER_EXIT_ERROR), NULL);
        } else if (err_exit_laser == 2){
            errgen_set(LASER_ERROR(LASER_SLAVE_EXIT_ERROR), NULL);
        } else if (err_exit_laser == 3){
            errgen_set(ERR_LASER_INIT_FATAL, NULL);
        }
        g_idle_add(serialtools_init_laser,NULL);

        if(errgen_laserState == ERR_LASER_INIT_FATAL){
            errgen_set(ERR_LASER_INIT_FATAL, NULL);
            return 1;
        }

        if(Laser_GetPositionOffset(&ml, &sl)<0){
            printf("Laser_GetPosOffset error\n");
            return 1;
        }
    }
    serialtools_in_reinit_laser = 0;
    return 0;
}
