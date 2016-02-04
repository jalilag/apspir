#include "laser_asserv.h"
#include "errgen.h"
#include "keyword.h"
#include "slave.h"
#include "serialtools.h"
#include "gui.h"
#include "strtools.h"


extern UNS16 errgen_state;
extern char * errgen_aux;
extern UNS8 errgen_laserState;
extern int run_laser;
laser ml, sl, lsim;

int serialtools_in_reinit_laser = 0;
void serialtools_minimum_init(void){
	Laser_Init_Minimal(&ml);
	Laser_Init_Minimal(&sl);
	Laser_Init_Minimal(&lsim);
}
void serialtools_plotLaserState(void) {
    if (run_laser == 0) {
        GtkWidget* but = gui_get_widget("butLaserLoad");
        if (but != NULL) {
            if (gtk_style_context_has_class(gtk_widget_get_style_context(but),"butRed")) {
                gtk_style_context_remove_class(gtk_widget_get_style_context(but),"butRed");
                gtk_style_context_add_class (gtk_widget_get_style_context(but), "butGreen");
            }
            gtk_button_set_image(GTK_BUTTON(but),GTK_WIDGET(gtk_image_new_from_icon_name("gtk-connect",GTK_ICON_SIZE_DND)));
        }
    } else if (run_laser == 3) {
        GtkWidget* but = gui_get_widget("butLaserLoad");
        if (but != NULL) {
            if (gtk_style_context_has_class(gtk_widget_get_style_context(but),"butGreen")) {
                gtk_style_context_remove_class(gtk_widget_get_style_context(but),"butGreen");
                gtk_style_context_add_class (gtk_widget_get_style_context(but), "butRed");
            }
            gtk_button_set_image(GTK_BUTTON(but),GTK_WIDGET(gtk_image_new_from_icon_name("user-offline",GTK_ICON_SIZE_DND)));
        }
    }
    struct laser_data d;
    UNS8 err = errgen_laserState;
    if(((err & ERR_LASER_FATAL) == ERR_LASER_FATAL) || ((err & MASTER_NOT_STARTED) && (err & SLAVE_NOT_STARTED))){
        gui_image_set("imgLaserStateG", "gtk-no", 2);
        gui_image_set("imgLaserStateD", "gtk-no", 2);
        gui_label_set("labLaserStateG", MASTER_NOT_STARTED_LABEL);
        gui_label_set("labLaserStateD", SLAVE_NOT_STARTED_LABEL);
	return;
    }

    if((err & 0x15) == MASTER_FAILED_TIMEOUT) {
        gui_label_set("labLaserStateG", MASTER_FAILED_TIMEOUT_LABEL);
        gui_image_set("imgLaserStateG", "gtk-no", 2);
    } else if ((err & 0x15) == MASTER_FAILED_DATCONSISTENCY) {
      	gui_label_set("labLaserStateG", MASTER_FAILED_DATCONSISTENCY_LABEL);
        gui_image_set("imgLaserStateG", "gtk-no", 2);
    } else if ((err & 0x15) == MASTER_NOT_STARTED) {
        gui_label_set("labLaserStateG", MASTER_NOT_STARTED_LABEL);
        gui_image_set("imgLaserStateG", "gtk-no", 2);
    } else {
        gui_label_set("labLaserStateG", LASER_STATUS_OK_LABEL);
        gui_image_set("imgLaserStateG", "gtk-yes", 2);
    }

    if((err & 0x2A) == SLAVE_FAILED_TIMEOUT) {
        gui_label_set("labLaserStateD", MASTER_FAILED_TIMEOUT_LABEL);
        gui_image_set("imgLaserStateD", "gtk-no", 2);
    } else if ((err & 0x2A) == SLAVE_FAILED_DATCONSISTENCY) {
        gui_label_set("labLaserStateD", SLAVE_FAILED_DATCONSISTENCY_LABEL);
        gui_image_set("imgLaserStateD", "gtk-no", 2);
    } else if ((err & 0x2A) == SLAVE_NOT_STARTED) {
        gui_label_set("labLaserStateD", SLAVE_NOT_STARTED_LABEL);
        gui_image_set("imgLaserStateD", "gtk-no", 2);
    } else {
        gui_label_set("labLaserStateD", LASER_STATUS_OK_LABEL);
        gui_image_set("imgLaserStateD", "gtk-yes", 2);
    }

    //affichage mesures non vérifiées
    double res1 = serialtools_get_laser_data(&sl);
    gui_label_set("labLaserDataD", strtools_gnum2str(&res1, 0x10));

    double res2 = serialtools_get_laser_data(&ml);
    gui_label_set("labLaserDataG", strtools_gnum2str(&res2, 0x10));

    //Affichage mesure retenue
    double res3 = serialtools_get_laser_data_valid();
    gui_label_set("labLaserDataV",strtools_gnum2str(&res3, 0x10));
}
double serialtools_get_laser_data_valid(void) {
    unsigned long mes1, mes2;
    Laser_GetUnverifiedData(&sl, &mes1);
    Laser_GetUnverifiedData(&ml, &mes2);
    if(errgen_laserState & 0x15) return (double)mes1/10000;
    else return (double)mes2/10000;
}

double serialtools_get_laser_data(void* las) {
    unsigned long val = 0;
    double res=(double)val;
    Laser_GetUnverifiedData(las, &val);
    if (val != 0) res = (double)val/10000;
    return res;
}

void serialtools_checkState_laser(void) {
    struct laser_data d;
    if (((errgen_laserState & ERR_LASER_FATAL) == ERR_LASER_FATAL) || ((errgen_laserState & MASTER_NOT_STARTED) && (errgen_laserState & SLAVE_NOT_STARTED))) return;
    errgen_laserState = Laser_GetData(&ml, &sl, &d);
}

void serialtools_check_laser_500ms(void){
    if (!serialtools_in_reinit_laser) serialtools_checkState_laser();
    if((errgen_laserState & ERR_LASER_FATAL) == ERR_LASER_FATAL){
        errgen_state = LASER_ERROR(ERR_LASER_FATAL);
        g_idle_add(errgen_set_safe(NULL),NULL);
        run_laser = 0;
        serialtools_exit_laser();

    }
}
gpointer serialtools_init(gpointer data) {
    run_laser = 1;
    unsigned int Lstate = serialtools_init_laser();
    if(Lstate == ERR_LASER_INIT_FATAL){
        errgen_state = ERR_LASER_INIT_FATAL;
        g_idle_add(errgen_set_safe(NULL),NULL);
        run_laser = 0;
    } else if (Lstate == ERR_LASER_SERIAL_CONFIG){
        errgen_state = ERR_LASER_SERIAL_CONFIG;
        g_idle_add(errgen_set_safe(NULL),NULL);
        run_laser = 0;
    } else if (Lstate == ERR_LASER_STARTCHECKTHREAD){
       errgen_state = ERR_LASER_STARTCHECKTHREAD;
       g_idle_add(errgen_set_safe(NULL),NULL);
        run_laser = 0;
    } else
        run_laser = 2;
    return 0;
}

unsigned int serialtools_init_laser(void) {
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

   return 0;//ok

}


int err_exit_laser = 0;
void serialtools_exit_laser(void)
{
    printf("cantools exit laser: errgen_laserState = %x\n", errgen_laserState);

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
