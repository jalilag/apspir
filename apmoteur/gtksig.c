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

    //SIGNALS PARAMS
    g_signal_connect (gtk_builder_get_object (builder, "butParamReturn"), "clicked", G_CALLBACK (on_butParamReturn_clicked),NULL);
    g_signal_connect (gtk_builder_get_object (builder, "butParamSave"), "clicked", G_CALLBACK (on_butParamSave_clicked),NULL);
    g_signal_connect (gtk_builder_get_object (builder, "butParamMotor"), "clicked", G_CALLBACK (on_butParamMotor_clicked),NULL);
    g_signal_connect (gtk_builder_get_object (builder, "butParamProfile"), "clicked", G_CALLBACK (on_butParamProfile_clicked),NULL);
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

// Fermeture de la window
void on_butQuit_clicked (GtkWidget* pEntry) {
    Exit(2);
}
void on_butParams_clicked(GtkWidget* pEntry) {
    g_mutex_lock(&lock_gui_box);
    Exit(0);
    slave_gui_param_gen(current_menu);
    gui_widget2show("windowParams",NULL);
}

void on_butVelStart_active_notify(GtkWidget* pEntry) {
    int i, j = gui_switch_is_active("butVelStart");
//    SDOR test = {0x60FF,0x00,0x04};
//    INTEGER32 datTest = 0;
//    cantools_write_sdo(0x02,test,&datTest);

    for (i=0; i<SLAVE_NUMBER; i++) {
        if (slave_get_param_in_num("SlaveProfile",i) == 0 && slave_get_param_in_num("Active",i)) {
            slave_set_param("Vel2send",i,0);
            if (j == 1) {
                if (motor_get_state((UNS16)slave_get_param_in_num("Power",i)) == SON)
                    motor_start(slave_get_node_with_index(i),1);
            } else if (j==0) {
                if (motor_get_state((UNS16)slave_get_param_in_num("Power",i)) == OENABLED) {
                    motor_start(slave_get_node_with_index(i),0);
                }
            }
        }
    }
}

void on_radBackward_toggled(GtkWidget* pEntry) {
    if(gui_but_is_checked("radBackward")) {
        int i;
        for (i=0; i<SLAVE_NUMBER;i++) {
            if (slave_get_param_in_num("SlaveProfile",i) == 0 && slave_get_param_in_num("Active",i)) {
                if(!motor_forward(slave_get_node_with_index(i),0))
                    errgen_set(ERR_MOTOR_BACKWARD,slave_get_title_with_index(i));
                slave_set_param("Vel2send",i,0);
            }
        }
    }
}

void on_radForward_toggled(GtkWidget* pEntry) {
    if(gui_but_is_checked("radForward")) {
        int i;
        for (i=0; i<SLAVE_NUMBER;i++) {
            if (slave_get_param_in_num("SlaveProfile",i) == 0 && slave_get_param_in_num("Active",i)) {
                if(!motor_forward(slave_get_node_with_index(i),1))
                    errgen_set(ERR_MOTOR_FORWARD,slave_get_title_with_index(i));
            slave_set_param("Vel2send",i,0);
            }
        }
    }
}

void on_butVelUp_clicked(GtkWidget* pEntry) {
    int i,j;
    for (i=0;i<SLAVE_NUMBER;i++) {
        if (slave_get_param_in_num("SlaveProfile",i) == 0 && slave_get_param_in_num("Active",i)) {
            slave_set_param("Vel2send",i,slave_get_param_in_num("Vel2send",i) + slave_get_param_in_num("Velinc",i));
        }
    }
}
void on_butVelDown_clicked(GtkWidget* pEntry) {
    int i,j;
    for (i=0;i<SLAVE_NUMBER;i++) {
        if (slave_get_param_in_num("SlaveProfile",i) == 0 && slave_get_param_in_num("Active",i)) {
            if (slave_get_param_in_num("Vel2send",i) - slave_get_param_in_num("Velinc",i) > 0)
                slave_set_param("Vel2send",i,slave_get_param_in_num("Vel2send",i) - slave_get_param_in_num("Velinc",i));
            else
                slave_set_param("Vel2send",i,0);
        }
    }
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
    UNS8* dat = data;
    int i = slave_get_index_with_node(*dat);
    int pold = slave_get_param_in_num("SlaveProfile",i);
    if (slave_get_param_in_num("State",i) == STATE_READY) {
        if (pold != 2 && motor_get_state((UNS16)slave_get_param_in_num("Power",i)) == SON) {
            slave_set_param("SlaveProfile",i,2);
            if (!slave_config_with_file(*dat))
                errgen_set(ERR_SLAVE_CONFIG,NULL);
            motor_start(*dat,1);
            gtk_button_set_image(GTK_BUTTON(pEntry),GTK_WIDGET(gtk_image_new_from_file("images/unlock.png")));
        } else if (pold == 2) {
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
