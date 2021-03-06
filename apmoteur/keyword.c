#include "keyword.h"
#include <gtk/gtk.h>
#include "gtksig.h"
#include "gui.h"
#include <math.h>
#include <stdio.h>
#include "master.h"
#include "strtools.h"
#include "slave.h"
#include "motor.h"
#include "SpirallingControl.h"
#include "profile.h"
#include "serialtools.h"
#include <time.h>

extern int SLAVE_NUMBER;
extern SLAVES_conf slaves[SLAVE_NUMBER_LIMIT];
extern PROF profiles[PROFILE_NUMBER];
extern int run_laser,set_up;
extern INTEGER32 rot_pos; // Position de démarrage moteur
extern double length_start,length_actual_laser; // Longueur de début
extern CONFIG conf1;
extern INTEGER32 rot_pos_err_in_step,rot_pos_err_mean_in_step;
extern double rot_pos_err_in_mm,rot_pos_err_mean_in_mm;

void keyword_init () {

/** TITRE DES WINDOWS **/
    gtk_window_set_title(gui_get_window("mainWindow"), APPLI_TITLE);
    if(gtk_window_set_icon_from_file(gui_get_window("mainWindow"),"images/icon.png",NULL) != 1)
        printf("Erreur set icon\n");
/** HEADER BUTTONS **/
//    gui_button_set("butInit",INITIALIZATION,"gtk-apply");
    gui_button_set("butStop",ARRET,"gtk-close");
    gui_button_set("butParams",CONFIGURE,"gtk-preferences");

/** BOX STATUTS **/
    // Titre des colonnes
    gui_label_set("labState",STATUT);
    gui_label_set("labStateError", STATE_ERROR );
    gui_label_set("labPower", POWER_STATE);
    gui_label_set("labPowerError", POWER_STATE_ERROR);
    gui_label_set("labVolt", VOLTAGE);
    gui_label_set("labTemp", TEMPERATURE);

    // Valeur par défaut
/** BOX TRANSLATION **/
    // Labels boutons
//    gui_label_set("labBoxTranslation",BOX_TRANS_TITLE);
//    gui_label_set("labDirection",DIRECTION);
//    gui_label_set("labVelStop",ACTIVE);
//    gui_label_set("labSetVel",VELOCITY_SET);
//    gui_label_set("labWriteVel",VELOCITY_SEND);
//    gui_label_set("labReadVel",VELOCITY_ACTUAL);
//
//
//    gui_button_set("radForward",FORWARD,NULL);
//    gui_button_set("butVelUp",NULL,"gtk-add");
//    gui_button_set("butVelDown",NULL,"gtk-remove");
//
//    gtk_switch_set_active(gui_get_switch("butVelStart"),FALSE);

/** BOX SET UP **/
    gui_label_set("labBoxSetUp",BOX_SETUP_TITLE);
    gui_label_set("labTrans",TRANSLATION);
    gui_label_set("labRot",ROTATION);
    gui_button_set("butTransUp",NULL,"gtk-go-up");
    gui_button_set("butTransDown",NULL,"gtk-go-down");
    gui_button_set("butTransStop",NULL,"gtk-media-pause");
    gui_button_set("butRotRight",NULL,"edit-redo");
    gui_button_set("butRotLeft",NULL,"edit-undo");

/** BOX STATUT **/
    gui_label_set("stateBarTitle",ETAT);
    gtk_label_set_ellipsize (gui_get_label("stateBarContent"),PANGO_ELLIPSIZE_START);

/** BOX PARAMS **/
    gui_label_set("labParamTitle",PARAMETERS);
    gui_button_set("butParamReturn",RETURN,"gtk-close");
    gui_button_set("butParamSave",SAVE,"gtk-save");
    gui_button_set("butParamMotor",MOTOR_PARAM_TITLE,"gtk-save");
    gui_button_set("butParamProfile",PROFIL_PARAM_TITLE,"gtk-save");
    gui_button_set("butParamHelix",HELIX_PARAM_TITLE,"gtk-save");
    gui_button_set("butParamGeom",GEOM_PARAM_TITLE,"gtk-save");
    gui_button_set("butParamAsserv",ASSERV_PARAM_TITLE,"gtk-save");
    gtk_button_set_image(gui_get_button("butParamMotor"),GTK_WIDGET(gtk_image_new_from_file("images/moteur.png")));
    gtk_button_set_image(gui_get_button("butParamProfile"),GTK_WIDGET(gtk_image_new_from_file("images/profil.png")));
    gtk_button_set_image(gui_get_button("butParamHelix"),GTK_WIDGET(gtk_image_new_from_file("images/spiral.png")));
    gtk_button_set_image(gui_get_button("butParamGeom"),GTK_WIDGET(gtk_image_new_from_file("images/geom.png")));
    gtk_button_set_image(gui_get_button("butParamAsserv"),GTK_WIDGET(gtk_image_new_from_file("images/geom.png")));

/** BOX LASER **/
    gui_image_set("imgLaserStateG", "gtk-no", 2);
    gui_image_set("imgLaserStateD", "gtk-no", 2);
    gui_label_set("labLaserStateG", DEFAULT);
    gui_label_set("labLaserStateD", DEFAULT);
    gui_label_set("labLaserDataG", DEFAULT);
    gui_label_set("labLaserDataD", DEFAULT);
    gui_label_set("labLaserDataV", DEFAULT);
    gui_label_set("labLaserTitle", LASER);
    gui_label_set("labLaserTitleG", LASERG);
    gui_label_set("labLaserTitleD", LASERD);
    gui_label_set("labLaserState",STATE_LASER);
    gui_label_set("labLaserData", DISTANCE);
    gui_button_set("butLaserLoad","","gtk-save");
    gtk_button_set_image(gui_get_button("butLaserLoad"),GTK_WIDGET(gtk_image_new_from_icon_name("user-offline",GTK_ICON_SIZE_DND)));

/** BOX SET **/
    gui_label_set("labTitleSetUp",SET_UP_TITLE);
    gui_button_set("butStartSet",START,"media-playback-start");
    gui_button_set("butContinueSet",CONTINUE,"view-refresh");
    gui_button_set("butStopSet",STOP,"process-stop");


}
//double length_old = 0;

extern int motor_running;
int iii = 0;
gboolean keyword_maj(gpointer data) {
    //
    if (set_up) {
        GtkWidget* lev = gui_local_get_widget(gui_get_widget("gridVisu"),"levelBar");
        gtk_level_bar_set_value(GTK_LEVEL_BAR(lev),(conf1.pipeLength+conf1.length2pipe - length_actual_laser)*100/conf1.pipeLength);
    }
    // Laser
    serialtools_plotLaserState();
    gui_local_label_set("labErrInstMm",strtools_gnum2str(&rot_pos_err_in_mm,0x10),"gridVisu");
    gui_local_label_set("labErrMeanMm",strtools_gnum2str(&rot_pos_err_mean_in_mm,0x10),"gridVisu");
    gui_local_label_set("labErrInstStep",strtools_gnum2str(&rot_pos_err_in_step,0x04),"gridVisu");
    gui_local_label_set("labErrMeanStep",strtools_gnum2str(&rot_pos_err_mean_in_step,0x04),"gridVisu");
    int i = 0,j=0,k;
    char* key;
    char* item2show[7] = {"State","StateError","Power","PowerError","Temp","Volt",NULL};
    // Spinner
    int l = 0;
    for (i=0; i<SLAVE_NUMBER; i++) {
        if (slave_get_param_in_num("State",i) != STATE_DISCONNECTED && slave_get_param_in_num("State",i) != STATE_READY)
            l++;
        if (slave_get_param_in_num("Active",i)) {
            if (motor_get_target((UNS16)slave_get_param_in_num("Power",i)) == 0)
                j++;
        }
    }

    if (l>0 || j>0 || (run_laser>0 && run_laser<3)) {
        gtk_spinner_start(GTK_SPINNER(gui_get_object("chargement")));
        gui_widget2show("chargement",NULL);
    } else {
        gtk_spinner_stop(GTK_SPINNER(gui_get_object("chargement")));
        gui_widget2hide("chargement",NULL);
        if (motor_running) {
            int state=0;
            if (motor_get_state((UNS16)slave_get_param_in_num("Power",slave_get_index_with_profile_id("RotVit"))) == OENABLED) {
                Exit(0);
                slave_set_param("State",slave_get_index_with_profile_id("RotVit"),STATE_PREOP);
            } else Exit(0);
            motor_running = 0;
        }
    }
    slave_gen_plot();

//    if (motor_get_slippage((UNS16)slave_get_param_in_num("Power",slave_get_index_with_profile_id("TransVit"))))
//        gui_push_state("Slippage");

    for (i=0; i<SLAVE_NUMBER; i++) {
        j = i+1; k=0;
        key = strtools_gnum2str(&j,0x02);
        if (slave_get_param_in_num("Active",i) == 1 ) {
            GtkWidget* labtitle = gui_local_get_widget(gui_get_widget("gridState"),strtools_concat("labM",key,"SlaveTitle",NULL));
            gui_local_label_set(strtools_concat("labM",key,"SlaveTitle",NULL),strtools_concat(slave_get_param_in_char("SlaveTitle",i),"\n",slave_get_param_in_char("SlaveProfile",i),NULL),"gridState");
            if (gtk_style_context_has_class(gtk_widget_get_style_context(labtitle),"green"))
                gtk_style_context_remove_class(gtk_widget_get_style_context(labtitle),"green");
            if (gtk_style_context_has_class(gtk_widget_get_style_context(labtitle),"purple"))
                gtk_style_context_remove_class(gtk_widget_get_style_context(labtitle),"purple");
            if (gtk_style_context_has_class(gtk_widget_get_style_context(labtitle),"brown"))
                gtk_style_context_remove_class(gtk_widget_get_style_context(labtitle),"brown");
            if (slave_get_param_in_num("SlaveProfile",i) == 0)
                gtk_style_context_add_class (gtk_widget_get_style_context(labtitle), "green");
            else if (slave_get_param_in_num("SlaveProfile",i) == 1)
                gtk_style_context_add_class (gtk_widget_get_style_context(labtitle), "greenLight");
            else if (slave_get_param_in_num("SlaveProfile",i) == 2)
                gtk_style_context_add_class (gtk_widget_get_style_context(labtitle), "purple");
            else if (slave_get_param_in_num("SlaveProfile",i) == 3)
                gtk_style_context_add_class (gtk_widget_get_style_context(labtitle), "purpleLight");
            else if (slave_get_param_in_num("SlaveProfile",i) == 4)
                gtk_style_context_add_class (gtk_widget_get_style_context(labtitle), "brown");

            while (item2show[k] != NULL) {
                gui_local_label_set(strtools_concat("labM",key,item2show[k],NULL),slave_get_param_in_char(item2show[k],i),"mainWindow");
                k++;
            }
            gui_local_image_set(strtools_concat("imgM",key,"StateImg",NULL),slave_get_param_in_char("StateImg",i),2,"mainWindow");
            if (profile_get_id_with_index(slave_get_param_in_num("SlaveProfile",i)) == "TransVit") {
                gui_label_set(strtools_concat("labM",key,"Vel",NULL),slave_get_param_in_char("Velocity",i));
                gui_label_set(strtools_concat("labM",key,"Vel2send",NULL),slave_get_param_in_char("Vel2send",i));
            }
        } else {
            char* item2show[5] = {"Power","PowerError","Temp","Volt",NULL};
            while (item2show[k] != NULL) {
                gui_local_label_set(strtools_concat("labM",key,item2show[k],NULL),DEFAULT,"mainWindow");
                k++;
            }
            gui_local_label_set(strtools_concat("labM",key,"State",NULL),slave_get_param_in_char("State",i),"mainWindow");
            gui_local_image_set(strtools_concat("imgM",key,"StateImg",NULL),slave_get_param_in_char("StateImg",i),2,"mainWindow");
        }
        // Ecriture des vitesses
        FILE* f = fopen(strtools_concat("dat/",slave_get_param_in_char("SlaveTitle",i),"_",profile_get_id_with_index(slave_get_param_in_num("SlaveProfile",i)),".dat",NULL),"a");
        char temps[20];
        time_t now = time(NULL);
        strftime(temps, 20, "%H:%M:%S", localtime(&now));
        if (f != NULL) {
            if (profile_get_id_with_index(slave_get_param_in_num("SlaveProfile",i)) == "TransVit")
                fputs(strtools_concat(temps," ",slave_get_param_in_char("Velocity",i)," ",slave_get_param_in_char("Vel2send",i),"\n",NULL),f);
            else
                fputs(strtools_concat(temps," ",slave_get_param_in_char("Velocity",i),"\n",NULL),f);
            fclose(f);
        }
    }
    if(slave_get_node_with_profile(0) != 0x00) {
        gui_label_set("labTransVel2send",slave_get_param_in_char("Vel2send",slave_get_index_with_node(slave_get_node_with_profile(0))));
        gui_label_set("labTransVel",slave_get_param_in_char("Velocity",slave_get_index_with_node(slave_get_node_with_profile(0))));
    }

    gtk_widget_queue_draw(gui_get_widget("mainWindow"));
    return TRUE;
}

gboolean keyword_active_maj(gpointer data) {
    UNS8* res = data;
    int i = slave_get_index_with_node(*res);
    int var = slave_get_param_in_num("Active",i);
    int j = i+1;
    if(var == 0) {
        GtkWidget* but = gui_local_get_widget(gui_get_widget("gridState"),strtools_concat("butActive",strtools_gnum2str(&j,0x02),NULL));
        if (but != NULL) {
            gtk_style_context_remove_class (gtk_widget_get_style_context(but), "butRed");
            gtk_button_set_image(GTK_BUTTON(but),GTK_WIDGET(gtk_image_new_from_icon_name("gtk-connect",GTK_ICON_SIZE_DND)));
            gtk_style_context_add_class (gtk_widget_get_style_context(but), "butGreen");
            gtk_widget_show(but);
        }
        motor_switch_off(slave_get_node_with_index(i));
        slave_set_param("State",i,STATE_DISCONNECTED);
        masterSendNMTstateChange(&SpirallingMaster_Data,slave_get_node_with_index(i),NMT_Stop_Node);
    } else {
        GtkWidget* but = gui_local_get_widget(gui_get_widget("gridState"),strtools_concat("butActive",strtools_gnum2str(&j,0x02),NULL));
        gtk_button_set_image(GTK_BUTTON(but),GTK_WIDGET(gtk_image_new_from_icon_name("user-offline",GTK_ICON_SIZE_DND)));
        gtk_style_context_remove_class (gtk_widget_get_style_context(but), "butGreen");
        gtk_style_context_add_class (gtk_widget_get_style_context(but), "butRed");
        gtk_widget_show(but);
        if (slave_get_param_in_num("StateError",i) == ERROR_STATE_LSS)
            slave_set_param("State",i,STATE_LSS_CONFIG);
        else if (slave_get_param_in_num("StateError",i) == ERROR_STATE_VOLTAGE)
            slave_set_param("State",i,STATE_LSS_CONFIG);
        else
            slave_set_param("State",i,STATE_PREOP);
        slave_set_param("StateError",i,ERROR_STATE_NULL);
    }
    return FALSE;
}

