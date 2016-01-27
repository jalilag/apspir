#include "laser_asserv.h"
#include "errgen.h"
#include "keyword.h"
#include "slave.h"
#include "serialtools.h"
#include "gui.h"
#include "strtools.h"


extern UNS16 errgen_state;
extern char * errgen_aux;

int serialtools_in_reinit_laser = 0;
void serialtools_minimum_init(void){
	Laser_Init_Minimal(&ml);
	Laser_Init_Minimal(&sl);
	Laser_Init_Minimal(&lsim);
}
void serialtools_plotLaserState(void)
{
    struct laser_data d;
    unsigned long mes1, mes2;
    UNS8 err = errgen_laserState;
    if(((err & ERR_LASER_FATAL) == ERR_LASER_FATAL) || ((err & MASTER_NOT_STARTED) && (err & SLAVE_NOT_STARTED))){
        gui_image_set("imgLaserDStatInfo", "gtk-no", 2);
        gui_image_set("imgLaserGStatInfo", "gtk-no", 2);
        gui_label_set("labLaser1StatInfo", MASTER_NOT_STARTED_LABEL);
        gui_label_set("labLaser2StatInfo", SLAVE_NOT_STARTED_LABEL);
	return;
    }

    if((err & 0x15) == MASTER_FAILED_TIMEOUT) {
	gui_label_set("labLaser1StatInfo", MASTER_FAILED_TIMEOUT_LABEL);
	gui_image_set("imgLaserGStatInfo", "gtk-no", 2);
    } else if ((err & 0x15) == MASTER_FAILED_DATCONSISTENCY) {
      	gui_label_set("labLaser1StatInfo", MASTER_FAILED_DATCONSISTENCY_LABEL);
	gui_image_set("imgLaserGStatInfo", "gtk-no", 2);
    } else if ((err & 0x15) == MASTER_NOT_STARTED) {
        gui_label_set("labLaser1StatInfo", MASTER_NOT_STARTED_LABEL);
        gui_image_set("imgLaserGStatInfo", "gtk-no", 2);
    } else {
        gui_label_set("labLaser1StatInfo", LASER_STATUS_OK_LABEL);
	gui_image_set("imgLaserGStatInfo", "gtk-yes", 2);
    }

    if((err & 0x2A) == SLAVE_FAILED_TIMEOUT) {
	gui_label_set("labLaser2StatInfo", MASTER_FAILED_TIMEOUT_LABEL);
	gui_image_set("imgLaserDStatInfo", "gtk-no", 2);
    } else if ((err & 0x2A) == SLAVE_FAILED_DATCONSISTENCY) {
	gui_label_set("labLaser2StatInfo", SLAVE_FAILED_DATCONSISTENCY_LABEL);
	gui_image_set("imgLaserDStatInfo", "gtk-no", 2);
    } else if ((err & 0x2A) == SLAVE_NOT_STARTED) {
        gui_label_set("labLaser2StatInfo", SLAVE_NOT_STARTED_LABEL);
	gui_image_set("imgLaserDStatInfo", "gtk-no", 2);
    } else {
        gui_label_set("labLaser2StatInfo", LASER_STATUS_OK_LABEL);
	gui_image_set("imgLaserDStatInfo", "gtk-yes", 2);
    }

    //affichage mesures non vérifiées
    Laser_GetUnverifiedData(&sl, &mes1);
    gui_label_set("labMesureLaser2", strtools_gnum2str_all_decimal((UNS32*)&mes1, uint32));

    Laser_GetUnverifiedData(&ml, &mes2);
    gui_label_set("labMesureLaser1", strtools_gnum2str_all_decimal((UNS32*)&mes2, uint32));

    //Affichage mesure retenue
    if(err & 0x15) gui_label_set("labMesureLaserGlobal", strtools_gnum2str_all_decimal((UNS32*)&mes1, uint32));
    else gui_label_set("labMesureLaserGlobal", strtools_gnum2str_all_decimal((UNS32*)&mes2, uint32));

}

void serialtools_checkState_laser(void)
{
    struct laser_data d;
    if (((errgen_laserState & ERR_LASER_FATAL) == ERR_LASER_FATAL) || ((errgen_laserState & MASTER_NOT_STARTED) && (errgen_laserState & SLAVE_NOT_STARTED))) return;
    errgen_laserState = Laser_GetData(&ml, &sl, &d);
}

static int run_laser_check = 0;
gpointer serialtools_check_laser_500ms(gpointer data){
    run_laser_check = 1;
    while(run_laser_check){
        //check laser state

        if (!serialtools_in_reinit_laser) serialtools_checkState_laser();
        if((errgen_laserState & ERR_LASER_FATAL) == ERR_LASER_FATAL){
            errgen_state = LASER_ERROR(ERR_LASER_FATAL);
            g_idle_add(errgen_set_safe(NULL),NULL);
            serialtools_exit_laser();
        }
        usleep(500000);
    }
    g_thread_exit(NULL);
}

unsigned int serialtools_init_laser(void)
{
    unsigned int err_l;
    unsigned int laser_state;
    struct laser_data d;

    if(serialtools_in_reinit_laser) {
        errgen_laserState = 0;
    }

    /**configuring serial port acces permissions**/
    if(Laser_serial_config()){
        errgen_laserState = MASTER_NOT_STARTED | SLAVE_NOT_STARTED;
        return ERR_LASER_SERIAL_CONFIG;
    }

    /**INIT 2 LASER**/
    if(err_l = Laser_Init2Laser(&ml, &sl)){
        if(err_l == ERR_LASER_INIT_FATAL){
            errgen_laserState = ERR_LASER_INIT_FATAL;
            return ERR_LASER_INIT_FATAL;//erreur
        }
        else if((err_l == LASER_MASTER_INIT_ERROR) || err_l == LASER_MASTER_INIT_ERROR2){
            errgen_laserState = MASTER_NOT_STARTED;
            if(Laser_Start1Laser(&sl)){
                errgen_laserState = ERR_LASER_INIT_FATAL;
                return ERR_LASER_INIT_FATAL;//erreur
            }
        }
        else if ((err_l == LASER_SLAVE_INIT_ERROR) || (err_l == LASER_SLAVE_INIT_ERROR2)){
            errgen_laserState = SLAVE_NOT_STARTED;
            if(Laser_Start1Laser(&ml)){
                errgen_laserState = ERR_LASER_INIT_FATAL;
                return ERR_LASER_INIT_FATAL;//erreur
            }
        }
    }
    else{
        /**START 2 LASER**/
        if(err_l = Laser_Start2Laser(&ml, &sl)){
            if(err_l == ERR_LASER_INIT_FATAL){
                if(err_l & LASER_GETPOSOFFSET_ERROR){
                    printf("Laser Start PositionOffset error\n");
                }
                errgen_laserState = ERR_LASER_INIT_FATAL;
                return ERR_LASER_INIT_FATAL;//erreur
            }
            if(err_l == LASER_MASTER_START_ERROR){
                errgen_laserState = MASTER_NOT_STARTED;
            }
            else if(err_l == LASER_SLAVE_START_ERROR){
                errgen_laserState = SLAVE_NOT_STARTED;
            }
        }
        if(!err_l){
            errgen_laserState = 0;
        }
    }
    if(g_thread_try_new(NULL,serialtools_check_laser_500ms,NULL,NULL) == NULL){
        return ERR_LASER_STARTCHECKTHREAD;
    }

    //printf ("err_l = %x", err_l);
   return 0;//ok

}


int err_exit_laser = 0;
void serialtools_exit_laser(void)
{
    printf("cantools exit laser: errgen_laserState = %x\n", errgen_laserState);
    run_laser_check = 0;

    if(!(errgen_laserState & MASTER_NOT_STARTED)){
        printf("In exit ml\n");
        if (Laser_Exit1Laser( &ml )<0){
            printf("Master laser exit error\n");
        }
        errgen_laserState |= MASTER_NOT_STARTED;
    }
    if(!(errgen_laserState & SLAVE_NOT_STARTED)){
        printf("In exit sl\n");
        if (Laser_Exit1Laser( &sl )<0){
            printf("Slave laser exit error\n");
        }
        errgen_laserState |= SLAVE_NOT_STARTED;
    }
}

unsigned int serialtools_reinit_laser(void){
    unsigned int res;
    serialtools_in_reinit_laser = 1;
    if(errgen_laserState){
        serialtools_exit_laser();
        res = serialtools_init_laser();
    }
    serialtools_in_reinit_laser = 0;
    return res;
}
