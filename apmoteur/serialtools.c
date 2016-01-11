#include "laser_asserv.h"
#include "errgen.h"
#include "keyword.h"
#include "slave.h"
#include "serialtools.h"

int serialtools_in_reinit_laser = 0;

int serialtools_init_laser(void)
{
    unsigned int err_l;
    unsigned int laser_state;
    struct laser_data d;

    if(serialtools_in_reinit_laser) errgen_laserState = 0;

    /**configuring serial port acces permissions**/
    if(Laser_serial_config()){
        errgen_set(ERR_LASER_SERIAL_CONFIG, NULL);
        return 1;
    }

    /**INIT 2 LASER**/
    if(err_l = Laser_Init2Laser(&ml, &sl)){
        if(err_l == ERR_LASER_INIT_FATAL){
            printf("1\n");
            errgen_set(ERR_LASER_INIT_FATAL, NULL);
            errgen_laserState = ERR_LASER_INIT_FATAL;
            return 1;
        }
        if(err_l == LASER_MASTER_INIT_ERROR){
            printf("2\n");
            errgen_set(LASER_ERROR(LASER_MASTER_INIT_ERROR), NULL);
            errgen_laserState = MASTER_NOT_STARTED;
            if(Laser_Start1Laser(&sl)){
                printf("3\n");
                errgen_set(ERR_LASER_INIT_FATAL, NULL);
                errgen_laserState = ERR_LASER_INIT_FATAL;
                return 1;
            }
            else{
                gui_image_set("imgLaserGStatInfo", "gtk-no", 2);
                gui_image_set("imgLaserDStatInfo", "gtk-yes", 2);
                gui_label_set("labLaser1StatInfo", MASTER_NOT_STARTED_LABEL);
                gui_label_set("labLaser2StatInfo", LASER_STATUS_OK_LABEL);
            }
        }
        else if (err_l == LASER_MASTER_INIT_ERROR2){
            printf("4\n");
            errgen_set(LASER_ERROR(LASER_MASTER_INIT_ERROR2), NULL);
            errgen_laserState = MASTER_NOT_STARTED;
            if(Laser_Start1Laser(&sl)){
                printf("5\n");
                errgen_laserState = ERR_LASER_INIT_FATAL;
                errgen_set(ERR_LASER_INIT_FATAL, NULL);
                return 1;
            }
            else{
                gui_image_set("imgLaserGStatInfo", "gtk-no", 2);
                gui_image_set("imgLaserDStatInfo", "gtk-yes", 2);
                gui_label_set("labLaser1StatInfo", MASTER_NOT_STARTED_LABEL);
                gui_label_set("labLaser2StatInfo", LASER_STATUS_OK_LABEL);
            }
        }
        else if (err_l == LASER_SLAVE_INIT_ERROR){
            printf("6\n");
            errgen_set(LASER_ERROR(LASER_SLAVE_INIT_ERROR), NULL);
            errgen_laserState = SLAVE_NOT_STARTED;
            if(Laser_Start1Laser(&ml)){
                printf("7\n");
                errgen_set(ERR_LASER_INIT_FATAL, NULL);
                errgen_laserState = ERR_LASER_INIT_FATAL;
                return 1;
            }
            else{
                gui_image_set("imgLaserGStatInfo", "gtk-yes", 2);
                gui_image_set("imgLaserDStatInfo", "gtk-no", 2);
                gui_label_set("labLaser1StatInfo", LASER_STATUS_OK_LABEL);
                gui_label_set("labLaser2StatInfo", SLAVE_NOT_STARTED_LABEL);
            }
        }
        else if (err_l == LASER_SLAVE_INIT_ERROR2){
            printf("8\n");
            errgen_set(LASER_ERROR(LASER_SLAVE_INIT_ERROR2), NULL);
            errgen_laserState = SLAVE_NOT_STARTED;
            if(Laser_Start1Laser(&ml)){
                printf("9\n");
                errgen_set(ERR_LASER_INIT_FATAL, NULL);
                errgen_laserState = ERR_LASER_INIT_FATAL;
                return 1;
            }
            else{
                gui_image_set("imgLaserGStatInfo", "gtk-yes", 2);
                gui_image_set("imgLaserDStatInfo", "gtk-no", 2);
                gui_label_set("labLaser1StatInfo", LASER_STATUS_OK_LABEL);
                gui_label_set("labLaser2StatInfo", SLAVE_NOT_STARTED_LABEL);
            }
        }
    }
    else{
        /**START 2 LASER**/
        if(err_l = Laser_Start2Laser(&ml, &sl)){
            if(err_l == ERR_LASER_INIT_FATAL){
                if(err_l & LASER_GETPOSOFFSET_ERROR){
                    errgen_set(LASER_ERROR(LASER_GETPOSOFFSET_ERROR), NULL);
                }
                printf("10\n");
                errgen_laserState = ERR_LASER_INIT_FATAL;
                errgen_set(ERR_LASER_INIT_FATAL, NULL);
                return 1;
            }
            if(err_l == LASER_MASTER_START_ERROR){
                printf("11\n");
                errgen_set(LASER_ERROR(LASER_MASTER_START_ERROR), NULL);
                errgen_laserState = MASTER_NOT_STARTED;
                gui_image_set("imgLaserGStatInfo", "gtk-no", 2);
                gui_image_set("imgLaserDStatInfo", "gtk-yes", 2);
                gui_label_set("labLaser1StatInfo", MASTER_NOT_STARTED_LABEL);
                gui_label_set("labLaser2StatInfo", LASER_STATUS_OK_LABEL);
            }
            else if(err_l == LASER_SLAVE_START_ERROR){
                printf("12\n");
                errgen_set(LASER_ERROR(LASER_SLAVE_START_ERROR), NULL);
                errgen_laserState = SLAVE_NOT_STARTED;
                gui_image_set("imgLaserGStatInfo", "gtk-yes", 2);
                gui_image_set("imgLaserDStatInfo", "gtk-no", 2);
                gui_label_set("labLaser1StatInfo", LASER_STATUS_OK_LABEL);
                gui_label_set("labLaser2StatInfo", SLAVE_NOT_STARTED_LABEL);
            }
        }
        if(!err_l){
            printf("13\n");
            gui_image_set("imgLaserDStatInfo", "gtk-yes", 2);
            gui_image_set("imgLaserGStatInfo", "gtk-yes", 2);
            gui_label_set("labLaser1StatInfo", LASER_STATUS_OK_LABEL);
            gui_label_set("labLaser2StatInfo", LASER_STATUS_OK_LABEL);
            errgen_laserState = 0;
        }
    }
    //printf ("err_l = %x", err_l);

    return 0;

}

void serialtools_checkPlotState_laser(void)
{
    unsigned int err_l;
    unsigned int laser_state;
    unsigned long mes1, mes2;
    struct laser_data d;
    int i;

    if ((errgen_laserState == ERR_LASER_FATAL) || ((errgen_laserState & MASTER_NOT_STARTED) && (errgen_laserState & SLAVE_NOT_STARTED))){
        gui_image_set("imgLaserDStatInfo", "gtk-no", 2);
        gui_image_set("imgLaserGStatInfo", "gtk-no", 2);
        return;
    }


    if(laser_state = Laser_GetData(&ml, &sl, &d)){
        if((laser_state & ERR_LASER_FATAL) == ERR_LASER_FATAL){
            errgen_set(LASER_ERROR(ERR_LASER_FATAL), NULL);
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
            gui_label_set("labLaser1StatInfo", MASTER_FAILED_TIMEOUT_LABEL);
            gui_image_set("imgLaserGStatInfo", "gtk-no", 2);
            errgen_laserState |= MASTER_FAILED_TIMEOUT;
            }
        else if(laser_state & SLAVE_FAILED_TIMEOUT){
            gui_label_set("labLaser2StatInfo", MASTER_FAILED_TIMEOUT_LABEL);
            gui_image_set("imgLaserDStatInfo", "gtk-no", 2);
            errgen_laserState |= SLAVE_FAILED_TIMEOUT;
        }
        else if(laser_state & MASTER_FAILED_DATCONSISTENCY){
            gui_label_set("labLaser1StatInfo", MASTER_FAILED_DATCONSISTENCY_LABEL);
            gui_image_set("imgLaserGStatInfo", "gtk-no", 2);
            errgen_laserState |= MASTER_FAILED_DATCONSISTENCY;
        }
        else if(laser_state & SLAVE_FAILED_DATCONSISTENCY){
            gui_label_set("labLaser2StatInfo", SLAVE_FAILED_DATCONSISTENCY_LABEL);
            gui_image_set("imgLaserDStatInfo", "gtk-no", 2);
            errgen_laserState |= SLAVE_FAILED_DATCONSISTENCY;
        }
        if(laser_state & MASTER_NOT_STARTED){
            if(!(errgen_laserState & MASTER_NOT_STARTED)){
                errgen_set(LASER_ERROR(MASTER_NOT_STARTED), NULL);
                gui_label_set("labLaser1StatInfo", MASTER_NOT_STARTED_LABEL);
                gui_image_set("imgLaserGStatInfo", "gtk-no", 2);
                errgen_laserState |= MASTER_NOT_STARTED;
            }
        }
        else if (laser_state & SLAVE_NOT_STARTED){
            if(!(errgen_laserState & SLAVE_NOT_STARTED)){
                errgen_set(LASER_ERROR(SLAVE_NOT_STARTED), NULL);
                gui_label_set("labLaser2StatInfo", SLAVE_NOT_STARTED_LABEL);
                gui_image_set("imgLaserDStatInfo", "gtk-no", 2);
                errgen_laserState |= SLAVE_NOT_STARTED;
            }
        }
    }
    else{
        gui_label_set("labLaser1StatInfo", LASER_STATUS_OK_LABEL);
        gui_label_set("labLaser2StatInfo", LASER_STATUS_OK_LABEL);
        gui_image_set("imgLaserDStatInfo", "gtk-yes", 2);
        gui_image_set("imgLaserGStatInfo", "gtk-yes", 2);
        errgen_laserState = 0;
    }

    //printf("laser_state checkplotstate= %x, laser_errgen_laserState = %x\n", laser_state, errgen_laserState);

//affichage mesures non vérifiées
    Laser_GetUnverifiedData(&sl, &mes1);
    gui_label_set("labMesureLaser2", strtools_gnum2str((UNS32*)&mes1, 0x50));

    Laser_GetUnverifiedData(&ml, &mes2);
    gui_label_set("labMesureLaser1", strtools_gnum2str((UNS32*)&mes2, 0x50));

    /**Decommenter pour affichage uniquement si la valeur est correcte**/
/*
    if(!((laser_state & SLAVE_FAILED_TIMEOUT) || (laser_state & SLAVE_FAILED_DATCONSISTENCY) || (laser_state & SLAVE_NOT_STARTED))){
        Laser_GetUnverifiedData(&sl, &mes1);
        gui_label_set("labMesureLaser2", strtools_gnum2str((UNS32*)&mes1, 0x50));
    }
    else gui_label_set("labMesureLaser2", "NA");

    if(!((laser_state & MASTER_FAILED_TIMEOUT) || (laser_state & MASTER_FAILED_DATCONSISTENCY) || (laser_state & MASTER_NOT_STARTED))){
        Laser_GetUnverifiedData(&ml, &mes2);
        gui_label_set("labMesureLaser1", strtools_gnum2str((UNS32*)&mes2, 0x50));
    }
    else gui_label_set("labMesureLaser1", "NA");
*/
    //Affichage mesure vérifiée
    gui_label_set("labMesureLaserGlobal", strtools_gnum2str((UNS32*)&(d.mes), 0x50));

}

void serialtools_exit_laser(void)
{
    printf("cantools exit laser: errgen_laserState = %x\n", errgen_laserState);

    if(!(errgen_laserState & MASTER_NOT_STARTED)){
        printf("In exit ml\n");
        if (Laser_Exit1Laser( &ml )<0)
            errgen_set(LASER_ERROR(LASER_MASTER_EXIT_ERROR), NULL);
    }
    if(!(errgen_laserState & SLAVE_NOT_STARTED)){
        printf("In exit sl\n");
        if (Laser_Exit1Laser( &sl )<0)
            errgen_set(LASER_ERROR(LASER_SLAVE_EXIT_ERROR), NULL);
    }
    if(errgen_laserState == ERR_LASER_INIT_FATAL)
        return;

    gui_label_set("labLaser1StatInfo", MASTER_NOT_STARTED_LABEL);
    gui_label_set("labLaser2StatInfo", SLAVE_NOT_STARTED_LABEL);
    gui_image_set("imgLaserDStatInfo", "gtk-no", 2);
    gui_image_set("imgLaserGStatInfo", "gtk-no", 2);
    errgen_laserState = MASTER_NOT_STARTED | SLAVE_NOT_STARTED;
}

int serialtools_reinit_laser(void){
    int res;
    serialtools_in_reinit_laser = 1;
    if(errgen_laserState){
        serialtools_exit_laser();
        if(serialtools_init_laser())
            return 1;

        if(Laser_GetPositionOffset(&ml, &sl)<0)
            return 1;
    }
    serialtools_in_reinit_laser = 0;
    return 0;
}
