#include "keyword.h"
#include <gtk/gtk.h>
#include "gtksig.h"
#include "gui.h"
#include "master.h"
#include "strtools.h"
#include "slave.h"
#include "motor.h"
#include "SpirallingControl.h"
#include "profile.h"
#include <time.h>

extern int SLAVE_NUMBER;
extern SLAVES_conf slaves[SLAVE_NUMBER_LIMIT];
extern PROF profiles[PROFILE_NUMBER];

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
    gui_label_set("labBoxTranslation",BOX_TRANS_TITLE);
    gui_label_set("labDirection",DIRECTION);
    gui_label_set("labVelStop",ACTIVE);
    gui_label_set("labSetVel",VELOCITY_SET);
    gui_label_set("labWriteVel",VELOCITY_SEND);
    gui_label_set("labReadVel",VELOCITY_ACTUAL);


    gui_button_set("radForward",FORWARD,NULL);
    gui_button_set("butVelUp",NULL,"gtk-add");
    gui_button_set("butVelDown",NULL,"gtk-remove");

    gtk_switch_set_active(gui_get_switch("butVelStart"),FALSE);

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
    gtk_button_set_image(gui_get_button("butParamMotor"),GTK_WIDGET(gtk_image_new_from_file("images/moteur.png")));
    gtk_button_set_image(gui_get_button("butParamProfile"),GTK_WIDGET(gtk_image_new_from_file("images/profil.png")));
    gtk_button_set_image(gui_get_button("butParamHelix"),GTK_WIDGET(gtk_image_new_from_file("images/spiral.png")));
    gtk_button_set_image(gui_get_button("butParamGeom"),GTK_WIDGET(gtk_image_new_from_file("images/geom.png")));

/** BOX LASER **/
    gui_image_set("imgLaserStateG", "gtk-no", 2);
    gui_image_set("imgLaserStateD", "gtk-no", 2);
    gui_label_set("labLaserStateG", DEFAULT);
    gui_label_set("labLaserStateD", DEFAULT);
    gui_label_set("labLaserDataG", DEFAULT);
    gui_label_set("labLaserDataD", DEFAULT);
    gui_label_set("labLaserDataV", DEFAULT);
    gui_label_set("labLaserTitleG", LASERG);
    gui_label_set("labLaserTitleD", LASERD);
    gui_label_set("labLaserState",STATE_LASER);
    gui_label_set("labLaserData", DISTANCE);

}
extern int motor_running;
gboolean keyword_maj(gpointer data) {
    int i = 0,j=0,k;
    char* key;
    char* item2show[7] = {"State","StateError","Power","PowerError","Temp","Volt",NULL};

    int l = 0;
    for (i=0; i<SLAVE_NUMBER; i++) {
        if (slave_get_param_in_num("State",i) != STATE_DISCONNECTED && slave_get_param_in_num("State",i) != STATE_READY)
            l++;
        if (slave_get_param_in_num("Active",i)) {
            if (motor_get_target((UNS16)slave_get_param_in_num("Power",i)) == 0)
                j++;
        }
    }
    if (l>0) {
        gtk_spinner_start(GTK_SPINNER(gui_get_object("chargement")));
        gui_widget2show("chargement",NULL);
    } else {
        if (j>0) {
            gui_widget2show("chargement",NULL);
            gtk_spinner_start(GTK_SPINNER(gui_get_object("chargement")));
        } else {
            gtk_spinner_stop(GTK_SPINNER(gui_get_object("chargement")));
            gui_widget2hide("chargement",NULL);
            if (motor_running) {
                Exit(0);
                motor_running = 0;
            }
        }
    }
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
        printf("Vitesse %s %s\n",slave_get_param_in_char("SlaveTitle",i),slave_get_param_in_char("Velocity",i));
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
    // Vérification du switch translation
//    int switch_but = gui_switch_is_active("butVelStart");
//    printf("%d\n",switch_but);
//    if (switch_but == 1) {
//        for (i=0; i<SLAVE_NUMBER; i++) {
//            if (slave_get_param_in_num("SlaveProfile",i) == 0) {
//                if (slave_get_param_in_num("Active",i) == 1) {
//                    if (motor_get_state((UNS16)slave_get_param_in_num("Power",i)) == SON) {
//                        motor_start(slave_get_node_with_index(i),1);
//                    }
//                }
//            }
//        }
//    } else if (switch_but == 0) {
//        for (i=0; i<SLAVE_NUMBER; i++) {
//            if (slave_get_param_in_num("SlaveProfile",i) == 0) {
//                if (slave_get_param_in_num("Active",i) == 1) {
//                    if (motor_get_state((UNS16)slave_get_param_in_num("Power",i)) == OENABLED) {
//                        motor_start(slave_get_node_with_index(i),0);
//                    }
//                }
//            }
//        }
//    }
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
    printf("%d %d %d\n",*res,i,var);
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

