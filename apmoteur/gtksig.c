#include <stdio.h>
#include <string.h>
#include <unistd.h>
#include <stdlib.h>
#include <signal.h>
#include <gtk/gtk.h>
#include "canfestival.h"

#include "gtksig.h"
#include "keyword.h"
#include "master.h"
#include "gui.h"
#include "cantools.h"
#include "SpirallingControl.h"
#include "strtools.h"
#include "motor.h"
#include "errgen.h"
#include "slave.h"
#include "profile.h"
//ajout 220116
#include "laser_asserv_cst.h"
#include "laser_asserv_utils.h"
#include "serialtools.h"
extern UNS16 errgen_state;
extern char* errgen_aux;
//fin ajout 220116
GtkBuilder *builder;

extern int run_init, SLAVE_NUMBER, current_menu;
extern PROF profiles[PROFILE_NUMBER];
extern INTEGER32 velocity_inc[SLAVE_NUMBER_LIMIT];
extern PARAM pardata[PARAM_NUMBER];
extern GMutex lock_gui_box;

void gtksig_init () {
    // SIGNALS MAIN
    g_signal_connect (gtk_builder_get_object (builder, "butStop"), "clicked", G_CALLBACK (on_butStop_clicked),NULL);
    g_signal_connect (gtk_builder_get_object (builder, "mainWindow"), "delete-event", G_CALLBACK (on_butQuit_clicked),NULL);
    g_signal_connect (gtk_builder_get_object (builder, "butParams"), "clicked", G_CALLBACK (on_butParams_clicked),NULL);
    g_signal_connect (gtk_builder_get_object (builder, "radForward"), "toggled", G_CALLBACK (on_radForward_toggled),NULL);
    g_signal_connect (gtk_builder_get_object (builder, "radBackward"), "toggled", G_CALLBACK (on_radBackward_toggled),NULL);
    g_signal_connect (gtk_builder_get_object (builder, "butVelUp"), "clicked", G_CALLBACK (on_butVelUp_clicked),NULL);
    g_signal_connect (gtk_builder_get_object (builder, "butVelDown"), "clicked", G_CALLBACK (on_butVelDown_clicked),NULL);

    g_signal_connect (gtk_builder_get_object (builder, "butVelStart"), "notify", G_CALLBACK (on_butVelStart_active_notify),NULL);
    // SIGNALS DIAL
    g_signal_connect (gtk_builder_get_object (builder, "butInitDialClose"), "clicked", G_CALLBACK (on_butInitDialClose_clicked),NULL);
    g_signal_connect (gtk_builder_get_object (builder, "windowDialInit"), "response", G_CALLBACK (on_butInitDialClose_clicked),NULL);
    g_signal_connect (gtk_builder_get_object (builder, "butPopupClose"), "clicked", G_CALLBACK (on_butPopupClose_clicked),NULL);

    //SIGNALS PARAMS
    g_signal_connect (gtk_builder_get_object (builder, "butParamReturn"), "clicked", G_CALLBACK (on_butParamReturn_clicked),NULL);
    g_signal_connect (gtk_builder_get_object (builder, "butParamSave"), "clicked", G_CALLBACK (on_butParamSave_clicked),NULL);
    g_signal_connect (gtk_builder_get_object (builder, "butParamMotor"), "clicked", G_CALLBACK (on_butParamMotor_clicked),NULL);
    g_signal_connect (gtk_builder_get_object (builder, "butParamProfile"), "clicked", G_CALLBACK (on_butParamProfile_clicked),NULL);
    g_signal_connect (gtk_builder_get_object (builder, "butParamHelix"), "clicked", G_CALLBACK (on_butParamHelix_clicked),NULL);

    // Mise en place
    g_signal_connect (gtk_builder_get_object (builder, "butTransUp"), "clicked", G_CALLBACK (on_butTransUp_clicked),NULL);
    g_signal_connect (gtk_builder_get_object (builder, "butTransDown"), "clicked", G_CALLBACK (on_butTransDown_clicked),NULL);
    g_signal_connect (gtk_builder_get_object (builder, "butTransStop"), "clicked", G_CALLBACK (on_butTransStop_clicked),NULL);
    g_signal_connect (gtk_builder_get_object (builder, "butRotRight"), "clicked", G_CALLBACK (on_butRotRight_clicked),NULL);
    g_signal_connect (gtk_builder_get_object (builder, "butRotLeft"), "clicked", G_CALLBACK (on_butRotLeft_clicked),NULL);
//    g_signal_connect (gtk_builder_get_object (builder, "butRotInit"), "clicked", G_CALLBACK (on_butRotInit_clicked),NULL);

//ajout 220116
//SIGNAL LASER
    g_signal_connect (gtk_builder_get_object (builder, "but_Start_Laser"), "clicked", G_CALLBACK (on_but_Start_Laser_clicked), NULL);
    g_signal_connect (gtk_builder_get_object (builder, "butReinitLaser"), "clicked", G_CALLBACK (on_butReinitLaser_clicked), NULL);

    // SIGNALS ROTATION
    //gestion pose automatique avec trsl par moteurs
    g_signal_connect (gtk_builder_get_object (builder, "but_StartPose"), "clicked", G_CALLBACK (on_but_StartPose_clicked), NULL);
    g_signal_connect (gtk_builder_get_object (builder, "but_ContinuePose"), "clicked", G_CALLBACK (on_but_ContinuePose_clicked), NULL);
    g_signal_connect (gtk_builder_get_object (builder, "but_StopPose"), "clicked", G_CALLBACK (on_but_StopPose_clicked), NULL);
    g_signal_connect (gtk_builder_get_object (builder, "but_StartPoseTreuil"), "clicked", G_CALLBACK (on_but_StartPoseTreuil_clicked), NULL);
    g_signal_connect (gtk_builder_get_object (builder, "but_ContinuerPoseTreuil"), "clicked", G_CALLBACK (on_but_ContinuePoseTreuil_clicked), NULL);
    g_signal_connect (gtk_builder_get_object (builder, "but_StopPoseTreuil"), "clicked", G_CALLBACK (on_but_StopPoseTreuil_clicked), NULL);
    //gestion pose automatique avec trsl par treuil

    //gestion parametre pose
    g_signal_connect (gtk_builder_get_object (builder, "but_paramPose"), "clicked", G_CALLBACK (on_but_paramPose_clicked), NULL);
    g_signal_connect (gtk_builder_get_object (builder, "but_saveParamAsserv"), "clicked", G_CALLBACK (on_but_saveParamAsserv_clicked), NULL);
    g_signal_connect (gtk_builder_get_object (builder, "but_returnParamAsserv"), "clicked", G_CALLBACK (on_but_returnParamAsserv_clicked), NULL);
    g_signal_connect (gtk_builder_get_object (builder, "but_paramDefault"), "clicked", G_CALLBACK (on_but_paramDefault_clicked), NULL);


//fin ajout 220116
}

// Bouton arret
void on_butStop_clicked (GtkWidget* pEntry) {
    Exit(0);
}

// Fermeture de la boite de dialogue
void on_butInitDialClose_clicked (GtkWidget* pEntry) {
    gtk_widget_hide(GTK_WIDGET(gtk_builder_get_object (builder, "windowDialInit")));
    g_mutex_unlock(&lock_gui_box);
}
void on_butPopupClose_clicked (GtkWidget* pEntry) {
    gtk_widget_hide(GTK_WIDGET(gtk_builder_get_object (builder, "windowPopup")));
}
// Fermeture de la window
void on_butQuit_clicked (GtkWidget* pEntry) {
    Exit(2);
}
void on_butParams_clicked(GtkWidget* pEntry) {
    if(gui_spinner_is_active("chargement")) return;
    g_mutex_lock(&lock_gui_box);
    Exit(0);
    slave_gui_param_gen(current_menu);
    gui_widget2show("windowParams",NULL);
}

void on_butVelStart_active_notify(GtkWidget* pEntry) {
    int i, j = gui_switch_is_active("butVelStart");
    if(gui_spinner_is_active("chargement")) return;
    cantools_ApplyVelTrans(0);
    for (i=0; i<SLAVE_NUMBER; i++) {
        if ((slave_get_profile_with_index(i) == PROF_COUPLTRANS || slave_get_profile_with_index(i) == PROF_VITTRANS) && slave_get_param_in_num("Active",i)) {
            slave_set_param("Vel2send",i,0);
            if (j == 1) {
                if (motor_get_state((UNS16)slave_get_param_in_num("Power",i)) == SON)
                    motor_start(slave_get_node_with_index(i),1);
            } else if (j == 0) {
                if (motor_get_state((UNS16)slave_get_param_in_num("Power",i)) == OENABLED) {
                    motor_start(slave_get_node_with_index(i),0);
                }
            }
        }
    }
    if(j){
        if (gui_but_is_checked("radForward")) on_radForward_toggled(NULL);
        else on_radBackward_toggled(NULL);
    }
}

void on_radBackward_toggled(GtkWidget* pEntry) {
    if(gui_spinner_is_active("chargement")) return;
    if(gui_but_is_checked("radBackward")) {
        int i;
        //ajout 220116
        cantools_ApplyVelTrans(0);
        //ajout 220116
        for (i=0; i<SLAVE_NUMBER;i++) {
            if ((slave_get_profile_with_index(i) == PROF_VITTRANS || slave_get_profile_with_index(i) == PROF_COUPLTRANS) && slave_get_param_in_num("Active",i)) {
                if(!motor_forward(slave_get_node_with_index(i),1)) {
                    errgen_state = ERR_MOTOR_BACKWARD; errgen_aux = slave_get_title_with_index(i);
                    g_idle_add(errgen_set_safe(NULL), NULL);
                    return;
                }
            }
        }
    }
}

void on_radForward_toggled(GtkWidget* pEntry) {
    if(gui_spinner_is_active("chargement")) return;
    if(gui_but_is_checked("radForward")) {
        int i;
        //ajout 220116
        cantools_ApplyVelTrans(0);
        //fin ajout 220116
        for (i=0; i<SLAVE_NUMBER;i++) {
            if ((slave_get_profile_with_index(i) == PROF_VITTRANS  || slave_get_profile_with_index(i) == PROF_COUPLTRANS) && slave_get_param_in_num("Active",i)) {
                if(!motor_forward(slave_get_node_with_index(i),0)) {
                    errgen_state = ERR_MOTOR_FORWARD; errgen_aux = slave_get_title_with_index(i);
                    g_idle_add(errgen_set_safe(NULL), NULL);
                    return;
                }
            }
        }
    }
}

void on_butVelUp_clicked(GtkWidget* pEntry) {
   if(gui_spinner_is_active("chargement")) return;
/*
    int i,j;
    for (i=0;i<SLAVE_NUMBER;i++) {
        if (slave_get_param_in_num("SlaveProfile",i) == 0 && slave_get_param_in_num("Active",i)) {
            slave_set_param("Vel2send",i,slave_get_param_in_num("Vel2send",i) + slave_get_param_in_num("Velinc",i));
        }
    }
*/
    //ajout 220116

    laser_asserv_utils_TrslManualInc();

    //fin ajout 220116
}
void on_butVelDown_clicked(GtkWidget* pEntry) {
     if(gui_spinner_is_active("chargement")) return;
/*
    int i,j;
    for (i=0;i<SLAVE_NUMBER;i++) {
        if (slave_get_param_in_num("SlaveProfile",i) == 0 && slave_get_param_in_num("Active",i)) {
            if (slave_get_param_in_num("Vel2send",i) - slave_get_param_in_num("Velinc",i) > 0)
                slave_set_param("Vel2send",i,slave_get_param_in_num("Vel2send",i) - slave_get_param_in_num("Velinc",i));
            else
                slave_set_param("Vel2send",i,0);
        }
    }
*/
    //ajout 220116

    laser_asserv_utils_TrslManualDec();

    //fin ajout 220116
}

/** BOITE DIAL PARAMS **/
void on_butParamReturn_clicked(GtkWidget* pEntry) {
    if (current_menu == 0) gtk_widget_destroy(gui_local_get_widget(gui_get_widget("boxParam"),"gridMotor"));
    else if (current_menu == 1) gtk_widget_destroy(gui_local_get_widget(gui_get_widget("boxParam"),"gridProfile"));
    gui_widget2hide("windowParams",NULL);
    g_mutex_unlock(&lock_gui_box);
}
void on_butParamSave_clicked(GtkWidget* pEntry) {
    slave_save_param (current_menu);
}
/**
* Bouton ajout champs paramètre moteur
**/
void on_butAddSlave_clicked (GtkWidget* pEntry) {
    GtkWidget* grid = gui_local_get_widget(gui_get_widget("boxParam"),"gridMotor");
    int i = 2,j,k,l;
    char* labtxt[7] = {"SlaveTitle","Vendor","Product","Revision","Serial","SlaveProfile",NULL};
    while(gtk_grid_get_child_at(GTK_GRID(grid),0,i) != NULL) i++;
    j = i-2;
    gtk_combo_box_text_append(GTK_COMBO_BOX_TEXT(gui_local_get_widget(grid,"listDel")),strtools_gnum2str(&j,0x02),strtools_concat(NODE," : ",strtools_gnum2str(&j,0x02),NULL));

    for (k=0;k<5;k++) {
        GtkWidget* lab = gui_create_widget("ent",strtools_concat("labParamM",strtools_gnum2str(&j,0x02),
            labtxt[k],NULL),NULL,NULL);
        gtk_grid_attach(GTK_GRID(grid),lab,k,i,1,1);
        if (k==0) gtk_widget_set_margin_left (lab,10);
    }
    GtkWidget* comb = gui_create_widget("combo",strtools_concat("labParamM",strtools_gnum2str(&j,0x02),"SlaveProfile",NULL),NULL,NULL);
    for (l=0; l<PROFILE_NUMBER;l++) {
        gtk_combo_box_text_append(GTK_COMBO_BOX_TEXT(comb),strtools_gnum2str(&l,0x02),profiles[l].title);
    }
    gtk_grid_attach(GTK_GRID(grid),comb,5,i,1,1);
    gtk_widget_show_all(gui_get_widget("boxParam"));
}
/**
* Bouton suppression champs paramètre moteur
**/
void on_butDelSlave_clicked(GtkWidget* pEntry) {
    GtkWidget* grid = gui_local_get_widget(gui_get_widget("boxParam"),"gridMotor");
    GtkWidget* listDel = gui_local_get_widget(grid,"listDel");
    int i = gtk_combo_box_get_active(GTK_COMBO_BOX(listDel)),j,k;
    if (i != -1) {
        const gchar* key = gtk_combo_box_get_active_id(GTK_COMBO_BOX(listDel));
        k = gui_str2num(key);
        gtk_combo_box_text_remove (GTK_COMBO_BOX_TEXT(listDel), i);
        for (j=0; j<6;j++) {
            gtk_widget_destroy (gtk_grid_get_child_at(GTK_GRID(grid),j,k+2));
        }
        gtk_widget_show_all(gui_get_widget("boxParam"));
    }
}
/**
* Bouton Header paramètre moteur
**/
void on_butParamMotor_clicked(GtkWidget* pEntry) {
    current_menu = 0;
    slave_gui_param_gen(current_menu);
}
/**
* Bouton Header paramètre profil
**/
void on_butParamProfile_clicked(GtkWidget* pEntry) {
    current_menu = 1;
    slave_gui_param_gen(current_menu);
}
/**
* Bouton Header paramètre Helix
**/
void on_butParamHelix_clicked(GtkWidget* pEntry) {
    current_menu = 3;
    slave_gui_param_gen(current_menu);
}
/**
* Code exécuté lors du choix d'un élément de la liste profil
**/
void on_listProfile_changed(GtkWidget* pEntry) {
    int ind = gtk_combo_box_get_active(GTK_COMBO_BOX(gui_local_get_widget(gui_get_widget("boxParam"),"listProfile")));
    if (ind != -1) {
        int i = 3,j,k=0,l;
        GtkWidget* grid = gui_local_get_widget(gui_get_widget("boxParam"),"gridProfile");
        while(gtk_grid_get_child_at(GTK_GRID(grid),0,i) != NULL) {
            gtk_widget_destroy (gtk_grid_get_child_at(GTK_GRID(grid),0,i));
            gtk_widget_destroy (gtk_grid_get_child_at(GTK_GRID(grid),1,i));
            i++;
        }
        if (gtk_grid_get_child_at(GTK_GRID(grid),2,1) != NULL) {
            gtk_widget_destroy (gtk_grid_get_child_at(GTK_GRID(grid),2,1));
            gtk_widget_destroy (gtk_grid_get_child_at(GTK_GRID(grid),3,1));
            gtk_widget_destroy (gtk_grid_get_child_at(GTK_GRID(grid),2,2));
            gtk_widget_destroy (gtk_grid_get_child_at(GTK_GRID(grid),3,2));
        }
        FILE* motor_fn = fopen(profile_get_filename_with_index(ind),"r");
        if (motor_fn != NULL) {
            GtkWidget* butAdd = gui_create_widget("but","butAddField",ADD,"stdButton","butBlue",NULL);
            g_signal_connect (G_OBJECT(butAdd), "clicked", G_CALLBACK (on_butAddField_clicked),NULL);
            gtk_button_set_image(GTK_BUTTON(butAdd),GTK_WIDGET(gtk_image_new_from_icon_name("gtk-add",GTK_ICON_SIZE_BUTTON)));
            GtkWidget* combAdd = gui_create_widget("combo","listAddField",NULL,NULL);
            GtkWidget* butDel = gui_create_widget("but","butDelField",REMOVE,"stdButton","butBlue",NULL);
            g_signal_connect (G_OBJECT(butDel), "clicked", G_CALLBACK (on_butDelField_clicked),NULL);
            gtk_button_set_image(GTK_BUTTON(butDel),GTK_WIDGET(gtk_image_new_from_icon_name("list-remove",GTK_ICON_SIZE_BUTTON)));
            GtkWidget* combDel = gui_create_widget("combo","listDelField",NULL,NULL);
            gtk_grid_attach(GTK_GRID(grid),combAdd,2,1,1,1);
            gtk_grid_attach(GTK_GRID(grid),butAdd,3,1,1,1);
            gtk_grid_attach(GTK_GRID(grid),combDel,2,2,1,1);
            gtk_grid_attach(GTK_GRID(grid),butDel,3,2,1,1);

            for (l=0; l<PARAM_NUMBER; l++) {
                gtk_combo_box_text_append(GTK_COMBO_BOX_TEXT(combAdd),pardata[l].gui_code,pardata[l].title);
            }
            char title[20];
            UNS32 data;
            char chaine[1024] = "";
            while(fgets(chaine,1024,motor_fn) != NULL) {
                k++;
                GtkWidget* ent;
                if (sscanf(chaine,"%19s ; %d",title,&data) == 2) {
                    int ii = motor_get_param_index(title);
                    if (strcmp(title,"Pdor1") == 0 || strcmp(title,"Pdor2") == 0 || strcmp(title,"Pdor3") == 0  ||
                        strcmp(title,"Pdor4") == 0  || strcmp(title,"Pdot3") == 0  || strcmp(title,"Pdot4") == 0 ||
                        strcmp(title,"TorqueHmtActive") == 0 || strcmp(title,"TorqueHmtControl") == 0
                        ) {
                        if (sscanf(chaine,"%19s ; %x",title,&data) == 2)
                            ent = gui_create_widget("ent",strtools_concat("entParamP",strtools_gnum2str(&ii,0x02),"_",strtools_gnum2str(&k,0x02),NULL),strtools_gnum2str(&data,motor_get_param_type(title)),NULL);
                    } else
                        ent = gui_create_widget("ent",strtools_concat("entParamP",strtools_gnum2str(&ii,0x02),"_",strtools_gnum2str(&k,0x02),NULL),strtools_gnum2str(&data,0x04),NULL);
                    GtkWidget* lab = gui_create_widget("lab",strtools_concat("labParamP",strtools_gnum2str(&ii,0x02),"_",strtools_gnum2str(&k,0x02),NULL),strtools_concat(strtools_gnum2str(&k,0x02)," : ",motor_get_param_title(title),NULL),"cell2","bold",NULL);
                    gtk_grid_attach(GTK_GRID(grid),lab,0,2+k,1,1);
                    gtk_grid_attach(GTK_GRID(grid),ent,1,2+k,1,1);
                    gtk_widget_set_halign(lab,GTK_ALIGN_START);
                    gtk_widget_set_halign(ent,GTK_ALIGN_START);
                    gtk_combo_box_text_append(GTK_COMBO_BOX_TEXT(combDel),strtools_gnum2str(&k,0x02),strtools_concat(FIELD," : ",strtools_gnum2str(&k,0x02),NULL));
                }
            }
            gtk_widget_show_all(GTK_WIDGET(grid));
        }
    }
}
/**
* Bouton ajout d'un paramètre profil
**/
void on_butAddField_clicked (GtkWidget* pEntry) {
    GtkWidget* grid = gui_local_get_widget(gui_get_widget("boxParam"),"gridProfile");
    GtkWidget* comb = gui_local_get_widget(gui_get_widget("boxParam"),"listAddField");
    int ind = gtk_combo_box_get_active(GTK_COMBO_BOX(comb));
    if (ind != -1) {
        int i = 3,j;
        while(gtk_grid_get_child_at(GTK_GRID(grid),0,i) != NULL) i++;
        j = i-2;
        GtkWidget* lab = gui_create_widget("lab",strtools_concat("labParamP",strtools_gnum2str(&ind,0x02),"_",strtools_gnum2str(&j,0x02),NULL),strtools_concat(strtools_gnum2str(&j,0x02)," : ",pardata[ind].title,NULL),"cell2","bold",NULL);
        GtkWidget* ent = gui_create_widget("ent",strtools_concat("entParamP",strtools_gnum2str(&ind,0x02),"_",strtools_gnum2str(&j,0x02),NULL),NULL,NULL);
        gtk_widget_set_halign(lab,GTK_ALIGN_START);
        gtk_widget_set_halign(ent,GTK_ALIGN_START);
        gtk_combo_box_text_append(GTK_COMBO_BOX_TEXT(gui_local_get_widget(grid,"listDelField")),strtools_gnum2str(&j,0x02),strtools_concat(FIELD," : ",strtools_gnum2str(&j,0x02),NULL));
        gtk_grid_attach(GTK_GRID(grid),lab,0,i,1,1);
        gtk_grid_attach(GTK_GRID(grid),ent,1,i,1,1);
        gtk_widget_show_all(grid);
    }
}
/**
* Bouton supprimant un parametre profil
**/
void on_butDelField_clicked (GtkWidget* pEntry) {
    GtkWidget* grid = gui_local_get_widget(gui_get_widget("boxParam"),"gridProfile");
    GtkWidget* comb = gui_local_get_widget(gui_get_widget("boxParam"),"listDelField");
    int ind = gtk_combo_box_get_active(GTK_COMBO_BOX(comb));
    if (ind != -1){
        const gchar* key = gtk_combo_box_get_active_id(GTK_COMBO_BOX(comb));
        int k = gui_str2num(key);
        gtk_widget_destroy (gtk_grid_get_child_at(GTK_GRID(grid),0,k+2));
        gtk_widget_destroy (gtk_grid_get_child_at(GTK_GRID(grid),1,k+2));
        gtk_combo_box_text_remove (GTK_COMBO_BOX_TEXT(comb), ind);
    }
}
/**
* Bouton activant ou désactivant un noeud
**/
void on_butActive_clicked (GtkWidget* pEntry,void* data) {
    UNS8* dat = data;
    int i = slave_get_index_with_node(*dat);
    if (slave_get_param_in_num("Active",i) == 0)
        slave_set_param("Active",i,1);
    else
        slave_set_param("Active",i,0);
}
/**
* Bouton mode libre
**/
void on_butFree_clicked(GtkWidget* pEntry,void* data) {
    if(gui_spinner_is_active("chargement")) return;
    UNS8* dat = data;
    int i = slave_get_index_with_node(*dat);
    printf("dat %d et i %d\n",*dat,i);
    int pold = slave_get_param_in_num("SlaveProfile",i);
    if (slave_get_param_in_num("State",i) == STATE_READY) {
        if (pold != profile_get_index_with_id("Libre") && motor_get_state((UNS16)slave_get_param_in_num("Power",i)) == SON) {
            slave_set_param("SlaveProfile",i,profile_get_index_with_id("Libre"));
            if (!slave_config_with_file(*dat))
                errgen_set(ERR_SLAVE_CONFIG,NULL);
            motor_start(*dat,1);
	    if (gui_but_is_checked("radForward"))
                motor_forward(*dat,0);
            else
                motor_forward(*dat,1);
            gtk_button_set_image(GTK_BUTTON(pEntry),GTK_WIDGET(gtk_image_new_from_file("images/unlock.png")));
        } else if (pold == profile_get_index_with_id("Libre")) {
            motor_start(*dat,0);
            FILE* fslave = fopen(FILE_SLAVE_CONFIG,"r");
            if (fslave != NULL) {
                char chaine[1024] = "";
                char title[20];
                UNS32 vendor,product,revision,serial;
                int profile,ii=0,res = 0;
                while(fgets(chaine,1024,fslave) != NULL) {
                    if (sscanf(chaine,"--%19s ; %x ; %x ; %x ; %x ; %d",title,&vendor,&product,&revision,&serial,&profile) == 6) {
                        if (ii == i) {
                            slave_set_param("SlaveProfile",i,profile);
                            if (!slave_config_with_file(*dat))
                                errgen_set(ERR_SLAVE_CONFIG,NULL);
                            res = 1;
                            gtk_button_set_image(GTK_BUTTON(pEntry),GTK_WIDGET(gtk_image_new_from_file("images/lock.png")));
                       }
                        ii++;
                    }
                }
                fclose(fslave);
                if(!res) {
                    gui_info_box(NODE_PROFILE,PROFILE_NOT_FOUND,NULL);
                }
            } else errgen_set(ERR_FILE_OPEN,FILE_SLAVE_CONFIG);
        }
    }
}


void on_butAddStep_clicked(GtkWidget* pEntry) {
    GtkWidget* grid = gui_local_get_widget(gui_get_widget("boxParam"),"gridHelix");
    GtkWidget* comb = gui_local_get_widget(gui_get_widget("boxParam"),"listStep");
    int i = 4,j;
    while(gtk_grid_get_child_at(GTK_GRID(grid),1,i) != NULL) {
        i++;
    }
    if (i == 4) {
        // Lab title
        GtkWidget* lab4 = gui_create_widget("lab","labStepTitle",HELIX_STEP_TITLE,"fontBig","bold","cell2",NULL);
        gtk_grid_attach(GTK_GRID(grid),lab4,1,3,1,1);
        gtk_widget_set_halign(lab4,GTK_ALIGN_START);
        GtkWidget* lab5 = gui_create_widget("lab","labLengthTitle",HELIX_LENGTH,"fontBig","bold","cell2",NULL);
        gtk_grid_attach(GTK_GRID(grid),lab5,2,3,1,1);
        gtk_widget_set_halign(lab5,GTK_ALIGN_START);
    }
    j = i-3;
    GtkWidget* lab = gui_create_widget("lab",strtools_concat("labHelix_",strtools_gnum2str(&j,0x02),NULL),strtools_gnum2str(&j,0x02),"fontBig","bold","cell2",NULL);
    GtkWidget* ent1 = gui_create_widget("ent",strtools_concat("entHelix_",strtools_gnum2str(&j,0x02),NULL),NULL,NULL);
    GtkWidget* ent2 = gui_create_widget("ent",strtools_concat("entLength_",strtools_gnum2str(&j,0x02),NULL),NULL,NULL);
    g_signal_connect (G_OBJECT(ent2), "changed", G_CALLBACK (on_lengthDef_changed),NULL);
    gtk_grid_attach(GTK_GRID(grid),lab,0,i,1,1);
    gtk_grid_attach(GTK_GRID(grid),ent1,1,i,1,1);
    gtk_grid_attach(GTK_GRID(grid),ent2,2,i,1,1);
    gtk_combo_box_text_append(GTK_COMBO_BOX_TEXT(comb),strtools_gnum2str(&j,0x02),strtools_concat(HELIX_STEP," : ",strtools_gnum2str(&j,0x02),NULL));
    gtk_widget_show_all(grid);
}

void on_butDelStep_clicked(GtkWidget* pEntry) {
    GtkWidget* grid = gui_local_get_widget(gui_get_widget("boxParam"),"gridHelix");
    GtkWidget* comb = gui_local_get_widget(gui_get_widget("boxParam"),"listStep");
    int ind = gtk_combo_box_get_active(GTK_COMBO_BOX(comb));
    if (ind != -1) {
        const gchar* key = gtk_combo_box_get_active_id(GTK_COMBO_BOX(comb));
        int k = gui_str2num(key);
        gtk_widget_destroy (gtk_grid_get_child_at(GTK_GRID(grid),0,k+3));
        gtk_widget_destroy (gtk_grid_get_child_at(GTK_GRID(grid),1,k+3));
        gtk_widget_destroy (gtk_grid_get_child_at(GTK_GRID(grid),2,k+3));
        gtk_combo_box_text_remove (GTK_COMBO_BOX_TEXT(comb), ind);
    }
}

void on_lengthDef_changed (GtkWidget* pEntry) {
    GtkWidget* grid = gui_local_get_widget(gui_get_widget("boxParam"),"gridHelix");
    GtkWidget* lab = gui_local_get_widget(gui_get_widget("boxParam"),"labLengthDefined");
    GtkWidget* comb = gui_local_get_widget(gui_get_widget("boxParam"),"listStep");
    int pipeLength = gui_str2num(gtk_entry_get_text(GTK_ENTRY(gtk_grid_get_child_at(GTK_GRID(grid),1,2))));

    int i=4,ii=0,N = gtk_tree_model_iter_n_children(gtk_combo_box_get_model(GTK_COMBO_BOX(comb)),NULL);
    int dat=0;
    while(ii < N) {
        if (gtk_grid_get_child_at(GTK_GRID(grid),0,i) != NULL) {
            ii++;
            dat+= gui_str2num(gtk_entry_get_text(GTK_ENTRY(gtk_grid_get_child_at(GTK_GRID(grid),2,i))));
        }
        i++;
    }
    gtk_label_set_text(GTK_LABEL(lab),strtools_concat(LENGTH_DEFINED," : ",strtools_gnum2str(&dat,0x02),NULL));
    if (pipeLength < dat) {
        if (gtk_style_context_has_class(gtk_widget_get_style_context(lab),"greenColor"))
            gtk_style_context_remove_class(gtk_widget_get_style_context(lab),"greenColor");
        gtk_style_context_add_class (gtk_widget_get_style_context(lab), "redColor");
    } else {
        if (gtk_style_context_has_class(gtk_widget_get_style_context(lab),"redColor"))
            gtk_style_context_remove_class(gtk_widget_get_style_context(lab),"redColor");
        gtk_style_context_add_class (gtk_widget_get_style_context(lab), "greenColor");
    }
}

void on_butTransUp_clicked (GtkWidget* pEntry) {
    if(gui_spinner_is_active("chargement")) return;
    int i;
    Exit(0);
    for (i=0; i<SLAVE_NUMBER;i++) {
        if (slave_get_param_in_num("Active",i)) {
            if (profile_get_id_with_index(slave_get_param_in_num("SlaveProfile",i)) == "TransCouple") {
                motor_set_param(slave_get_node_with_index(i),"Couple",800);
                motor_forward(slave_get_node_with_index(i),0);
                motor_start(slave_get_node_with_index(i),1);
            }
        }
    }
    for (i=0; i<SLAVE_NUMBER;i++) {
        if (slave_get_param_in_num("Active",i)) {
            if (profile_get_id_with_index(slave_get_param_in_num("SlaveProfile",i)) == "TransVit") {
                motor_forward(slave_get_node_with_index(i),0);
                motor_start(slave_get_node_with_index(i),1);
                motor_set_param(slave_get_node_with_index(i),"Vitesse",280000);
            }
        }
    }
}
void on_butTransDown_clicked (GtkWidget* pEntry) {
    if(gui_spinner_is_active("chargement")) return;
    int i;
    Exit(0);
    for (i=0; i<SLAVE_NUMBER;i++) {
        if (slave_get_param_in_num("Active",i)) {
            if (profile_get_id_with_index(slave_get_param_in_num("SlaveProfile",i)) == "TransCouple") {
                motor_set_param(slave_get_node_with_index(i),"Couple",800);
                motor_forward(slave_get_node_with_index(i),1);
                motor_start(slave_get_node_with_index(i),1);
            }
        }
    }
    for (i=0; i<SLAVE_NUMBER;i++) {
        if (slave_get_param_in_num("Active",i)) {
            if (profile_get_id_with_index(slave_get_param_in_num("SlaveProfile",i)) == "TransVit") {
                motor_forward(slave_get_node_with_index(i),1);
                motor_start(slave_get_node_with_index(i),1);
                motor_set_param(slave_get_node_with_index(i),"Vitesse",280000);
            }
        }
    }
}

void on_butTransStop_clicked(GtkWidget* pEntry) {
    int i;
    for (i=0; i<SLAVE_NUMBER;i++) {
        if (slave_get_param_in_num("Active",i)) {
            if (profile_get_id_with_index(slave_get_param_in_num("SlaveProfile",i)) == "TransVit" ||
            profile_get_id_with_index(slave_get_param_in_num("SlaveProfile",i)) == "TransCouple")
                motor_start(slave_get_node_with_index(i),0);
        }
    }
}

int motor_running = 0;
void on_butRotRight_clicked (GtkWidget* pEntry) {
    if(gui_spinner_is_active("chargement")) return;
    Exit(0);
    int i;
    for (i=0; i<SLAVE_NUMBER;i++) {
        if (slave_get_param_in_num("Active",i)) {
            printf("i : %d node %x Profile %s\n",i,slave_get_node_with_index(i),profile_get_id_with_index(slave_get_param_in_num("SlaveProfile",i)));
            if (profile_get_id_with_index(slave_get_param_in_num("SlaveProfile",i)) == "RotVit") {
                if (!motor_forward(slave_get_node_with_index(i),0)) return;
                if (!motor_start(slave_get_node_with_index(i),1)) return;
                if (!motor_set_param(slave_get_node_with_index(i),"Profile",1)) return;
                if (!motor_set_param(slave_get_node_with_index(i),"Position",2850000)) return;
		if (!motor_set_param(slave_get_node_with_index(i),"VelocityMax",285000)) return;
            }
            if (profile_get_id_with_index(slave_get_param_in_num("SlaveProfile",i)) == "RotCouple") {
                if (!motor_set_param(slave_get_node_with_index(i),"Profile",4)) return;
                if (!motor_set_param(slave_get_node_with_index(i),"Couple",500)) return;
                if (!motor_forward(slave_get_node_with_index(i),0)) return;
                if (!motor_start(slave_get_node_with_index(i),1)) return;
            }
        }
    }
    for (i=0; i<SLAVE_NUMBER;i++) {
        if (slave_get_param_in_num("Active",i)) {
            if (profile_get_id_with_index(slave_get_param_in_num("SlaveProfile",i)) == "RotVit") {
                if (!motor_set_param(slave_get_node_with_index(i),"ControlWord",127)) return;
                sleep(1);
                if (!motor_set_param(slave_get_node_with_index(i),"ControlWord",111)) return;
            }
        }
    }
    motor_running = 1;
}

void on_butRotLeft_clicked (GtkWidget* pEntry) {
    if(gui_spinner_is_active("chargement")) return;
    Exit(0);
    int i;
    for (i=0; i<SLAVE_NUMBER;i++) {
        if (slave_get_param_in_num("Active",i)) {
            printf("i : %d node %x Profile %s\n",i,slave_get_node_with_index(i),profile_get_id_with_index(slave_get_param_in_num("SlaveProfile",i)));
            if (profile_get_id_with_index(slave_get_param_in_num("SlaveProfile",i)) == "RotVit") {
                if (!motor_forward(slave_get_node_with_index(i),1)) return;
                if (!motor_start(slave_get_node_with_index(i),1)) return;
                if (!motor_set_param(slave_get_node_with_index(i),"Profile",1)) return;
                if (!motor_set_param(slave_get_node_with_index(i),"Position",2850000)) return;
		 if (!motor_set_param(slave_get_node_with_index(i),"VelocityMax",285000)) return;
            }
            if (profile_get_id_with_index(slave_get_param_in_num("SlaveProfile",i)) == "RotCouple") {
                if (!motor_set_param(slave_get_node_with_index(i),"Profile",4)) return;
                if (!motor_set_param(slave_get_node_with_index(i),"Couple",500)) return;
                if (!motor_forward(slave_get_node_with_index(i),1)) return;
                if (!motor_start(slave_get_node_with_index(i),1)) return;
            }
        }
    }
    for (i=0; i<SLAVE_NUMBER;i++) {
        if (slave_get_param_in_num("Active",i)) {
            if (profile_get_id_with_index(slave_get_param_in_num("SlaveProfile",i)) == "RotVit") {
                if (!motor_set_param(slave_get_node_with_index(i),"ControlWord",127)) return;
                sleep(1);
                if (!motor_set_param(slave_get_node_with_index(i),"ControlWord",111)) return;
            }
        }
    }
    motor_running = 1;
}
//ajout 220116


void on_but_Start_Laser_clicked(GtkWidget* pEntry){
//Init Laser
  unsigned int res;
    res = serialtools_init_laser();
    if(res == ERR_LASER_INIT_FATAL){
        errgen_state = ERR_LASER_INIT_FATAL;
        g_idle_add(errgen_set_safe(NULL),NULL);
    } else if (res == ERR_LASER_SERIAL_CONFIG){
        errgen_state = ERR_LASER_SERIAL_CONFIG;
        g_idle_add(errgen_set_safe(NULL),NULL);
    } else if (res == ERR_LASER_STARTCHECKTHREAD){
       errgen_state = ERR_LASER_STARTCHECKTHREAD;
       g_idle_add(errgen_set_safe(NULL),NULL);
    }
}

void on_butReinitLaser_clicked(GtkWidget* pEntry)
{
    unsigned int res;
    if(errgen_laserState){
        res = serialtools_reinit_laser();
        if(res == ERR_LASER_INIT_FATAL){
            errgen_state = ERR_LASER_INIT_FATAL;
            g_idle_add(errgen_set_safe(NULL),NULL);
        } else if (res == ERR_LASER_SERIAL_CONFIG){
            errgen_state = ERR_LASER_SERIAL_CONFIG;
        g_idle_add(errgen_set_safe(NULL),NULL);
        } else if (res == ERR_LASER_STARTCHECKTHREAD){
           errgen_state = ERR_LASER_STARTCHECKTHREAD;
           g_idle_add(errgen_set_safe(NULL),NULL);
        }
    }
}

//fin ajout 220116
//ajout 270116
void on_but_StartPose_clicked(GtkWidget* pEntry){
    if(gui_spinner_is_active("chargement")) return;
    if(gui_but_is_checked("radiobutton_poseLent")){
        laser_asserv_utils_StartPose("lent");
    } else if (gui_but_is_checked("radiobutton_poseRapide")){
        laser_asserv_utils_StartPose("rapide");
    }
}
void on_but_ContinuePose_clicked(GtkWidget* pEntry){
    if(gui_spinner_is_active("chargement")) return;
    if(gui_but_is_checked("radiobutton_poseLent")){
        laser_asserv_utils_ContinuePose("lent");
    } else if (gui_but_is_checked("radiobutton_poseRapide")){
        laser_asserv_utils_ContinuePose("rapide");
    }
}
void on_but_StopPose_clicked(GtkWidget* pEntry){
    if(gui_spinner_is_active("chargement")) return;
    laser_asserv_utils_StopPose();
}
void on_but_StartPoseTreuil_clicked(GtkWidget* pEntry){
    laser_asserv_utils_StartRot();
}
void on_but_ContinuePoseTreuil_clicked(GtkWidget* pEntry){
    laser_asserv_utils_resumeRot();
}
void on_but_StopPoseTreuil_clicked(GtkWidget* pEntry){
    laser_asserv_utils_StopRot();
}

void on_but_paramPose_clicked(GtkWidget* pEntry){
    if(gui_spinner_is_active("chargement")) return;
    gui_widget2show("win_asserv_param",NULL);

    if(laser_asserv_cst_recup()) laser_asserv_cst_set_default();

    laser_asserv_cst_Get();

}
void on_but_saveParamAsserv_clicked(GtkWidget* pEntry){
    int data2;
    laser_asserv_cst_Set();

    if (!laser_asserv_cst_verify()) {
        errgen_state = ERR_LASER_ASSERV_PARAM;
        g_idle_add(errgen_set_safe(NULL), NULL);
        laser_asserv_cst_set_default();
    } else {
        if(laser_asserv_cst_save()) {
            errgen_state = ERR_LASER_ASSERV_PARAM_SAVE;
            g_idle_add(errgen_set_safe(NULL), NULL);
        }
    }

    gui_widget2hide("win_asserv_param",NULL);
}
void on_but_returnParamAsserv_clicked(GtkWidget* pEntry){
    gui_widget2hide("win_asserv_param",NULL);
}
void on_but_paramDefault_clicked(GtkWidget* pEntry){

    laser_asserv_cst_set_default();
    laser_asserv_cst_Get();
}
