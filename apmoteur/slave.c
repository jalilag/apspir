#include "slave.h"
#include "SpirallingControl.h"
#include "cantools.h"
#include "keyword.h"
#include "strtools.h"
#include <stdlib.h>
#include "errgen.h"
#include "gui.h"
#include "motor.h"
#include <gtk/gtk.h>
#include <gtk/gtkx.h>
#include <glib.h>
#include "master.h"
#include "gtksig.h"
#include "profile.h"
#include <time.h>
#include "asserv.h"

extern pthread_mutex_t lock_slave;
extern GMutex lock_gui, lock_err;
extern int SLAVE_NUMBER, run_init;

extern INTEGER32 velocity_inc[SLAVE_NUMBER_LIMIT];

extern SLAVES_conf slaves[SLAVE_NUMBER_LIMIT];
extern PROF profiles[PROFILE_NUMBER];
extern PARAM pardata[PARAM_NUMBER];
extern PARVAR vardata[VAR_NUMBER];
extern int set_up;
extern struct timespec tstart, tend;
CONFIG conf1;

static int step[2][20] = {{0}};
static int support[20]={0};

extern double time_start,time_actual_sync,time_actual_laser,min_length,tcalc;
extern int trans_type,rot_direction,max_error;
extern INTEGER32 trans_vel,vitesse_max;
extern UNS32 tsync;
/**
* Configuration des esclaves
* 0: Echec; 1 Reussite
**/
int slave_config(UNS8 nodeID) {
    /* CONFIG COM */
    if (!slave_config_com(nodeID)) return 0;
    /* CONFIG FIXE */
    if (!slave_config_with_file(nodeID)) return 0;
    /* SPECIFIC CONFIG */
    if (!slave_config_specific(nodeID)) return 0;
    return 1;
}

/**
* Configuration des paramètres de com
* 0: Echec; 1 Reussite
**/
static int slave_config_com(UNS8 nodeID) {
    /** ACTIVATION HEARTBEAT PRODUCER **/
    UNS16 heartbeat_prod = HB_PROD;
    SDOR hbprod = {0x1017,0x00,0x06};
    if(!cantools_write_sdo(nodeID,hbprod,&heartbeat_prod)) {
        errgen_set(ERR_SLAVE_CONFIG_HB,slave_get_title_with_node(nodeID));
        return 0;
    }
    /** CONFIGURATION PDOT **/
    // StatusWord, Error Code
    if(!cantools_PDO_map_config(nodeID,0x1A00,0x60410010,0x603F0010,0)) {
        errgen_set(ERR_SLAVE_CONFIG_PDOT,slave_get_title_with_node(nodeID));
        return 0;
    }
    // Voltage, Temp
    if(!cantools_PDO_map_config(nodeID,0x1A01,0x20150110,0x20180108,0)) {
        errgen_set(ERR_SLAVE_CONFIG_PDOT,slave_get_title_with_node(nodeID));
        return 0;
    }
    /** ACTIVATION DES PDOs **/
    if (!cantools_PDO_trans(nodeID,0x1800,0xFF,0x000A,0x0000)) {
        errgen_set(ERR_SLAVE_CONFIG_ACTIVE_PDO,slave_get_title_with_node(nodeID));
        return 0;
    }
    if (!cantools_PDO_trans(nodeID,0x1801,0xFF,0x1388,0x0000)) {
        errgen_set(ERR_SLAVE_CONFIG_ACTIVE_PDO,slave_get_title_with_node(nodeID));
        return 0;
    }
    if (!cantools_PDO_trans(nodeID,0x1802,0xFF,0x0000,0x0000)) {
        errgen_set(ERR_SLAVE_CONFIG_ACTIVE_PDO,slave_get_title_with_node(nodeID));
        return 0;
    }
    if (!cantools_PDO_trans(nodeID,0x1803,0xFF,0x0000,0x0000)) {
        errgen_set(ERR_SLAVE_CONFIG_ACTIVE_PDO,slave_get_title_with_node(nodeID));
        return 0;
    }
    return 1;
}

static int slave_config_specific(UNS8 nodeID) {
    if (slave_get_profile_with_node(nodeID) == profile_get_index_with_id("RotVit")) {
        if(!cantools_PDO_map_config(nodeID,0x1A02,0x606C0020,0x60640020,0)) { // vel,pos
            errgen_set(ERR_SLAVE_CONFIG_PDOT,slave_get_title_with_node(nodeID));
            return 0;
        }
        if (!cantools_PDO_trans(nodeID,0x1802,0x01,0x0000,0x0000)) {
            errgen_set(ERR_SLAVE_CONFIG_ACTIVE_PDO,slave_get_title_with_node(nodeID));
            return 0;
        }
        if(!cantools_PDO_map_config(nodeID,0x1600,0x60FF0020,0x60830020,0)) { // Vitesse, acc
            errgen_set(ERR_SLAVE_CONFIG_PDOT,slave_get_title_with_node(nodeID));
            return 0;
        }
        if(!cantools_PDO_map_config(nodeID,0x1601,0x60840020,0)) { //dcc
            errgen_set(ERR_SLAVE_CONFIG_PDOR,slave_get_title_with_node(nodeID));
            return 0;
        }
        if (!cantools_PDO_trans(nodeID,0x1400,0xFF,0x0000,0x0000)) {
            errgen_set(ERR_SLAVE_CONFIG_ACTIVE_PDO,slave_get_title_with_node(nodeID));
            return 0;
        }
        if (!cantools_PDO_trans(nodeID,0x1401,0xFF,0x0000,0x0000)) {
            errgen_set(ERR_SLAVE_CONFIG_ACTIVE_PDO,slave_get_title_with_node(nodeID));
            return 0;
        }
    }
    if (slave_get_profile_with_node(nodeID) == profile_get_index_with_id("RotCouple")) {
        if(!cantools_PDO_map_config(nodeID,0x1A02,0x606C0020,0)) { // vel
            errgen_set(ERR_SLAVE_CONFIG_PDOT,slave_get_title_with_node(nodeID));
            return 0;
        }
        if (!cantools_PDO_trans(nodeID,0x1802,0xFF,0x0000,0x0000)) {
            errgen_set(ERR_SLAVE_CONFIG_ACTIVE_PDO,slave_get_title_with_node(nodeID));
            return 0;
        }
        if(!cantools_PDO_map_config(nodeID,0x1600,0x60710010,0)) { // Couple
            errgen_set(ERR_SLAVE_CONFIG_PDOT,slave_get_title_with_node(nodeID));
            return 0;
        }
        if (!cantools_PDO_trans(nodeID,0x1400,0xFF,0x0000,0x0000)) {
            errgen_set(ERR_SLAVE_CONFIG_ACTIVE_PDO,slave_get_title_with_node(nodeID));
            return 0;
        }
    }
    return 1;
}
/**
* Configuration des esclaves à partir du fichier profile correspondant
* 0: Echec; 1 Reussite
**/
int slave_config_with_file(UNS8 nodeID) {
    FILE* motor_fn = fopen(profile_get_filename_with_index(slave_get_profile_with_node(nodeID)),"r");
    if (motor_fn != NULL) {
        UNS32 datu32;
        INTEGER32 dati32;
        char title[20];
        int k;
        UNS32 pdot3[8]={0},pdot4[8]={0},pdor1[8]={0},pdor2[8]={0},pdor3[8]={0},pdor4[8]={0};
        int p1=0,p2=0,p3=0,p4=0,p5=0,p6=0;
        char chaine[1024] = "";
        while(fgets(chaine,1024,motor_fn) != NULL) {
            if (sscanf(chaine,"%19s ; ",title) == 1) {
                if (strcmp(title,"Pdor1") == 0) {
                    if (sscanf(chaine,"%19s ; %x",title,&datu32) == 2) {pdor1[p1]=(UNS32)datu32; p1++;}
                } else if (strcmp(title,"Pdor2") == 0) {
                    if (sscanf(chaine,"%19s ; %x",title,&datu32) == 2) {pdor2[p2]=(UNS32)datu32; p2++;}
                } else if (strcmp(title,"Pdor3") == 0) {
                    if (sscanf(chaine,"%19s ; %x",title,&datu32) == 2) {pdor3[p3]=(UNS32)datu32; p3++;}
                } else if (strcmp(title,"Pdor4") == 0) {
                    if (sscanf(chaine,"%19s ; %x",title,&datu32) == 2) {pdor4[p4]=(UNS32)datu32; p4++;}
                } else if (strcmp(title,"Pdot3") == 0) {
                    if (sscanf(chaine,"%19s ; %x",title,&datu32) == 2) {pdot3[p5]=(UNS32)datu32; p5++;}
                } else if (strcmp(title,"Pdot4") == 0) {
                    if (sscanf(chaine,"%19s ; %x",title,&datu32) == 2) {pdot4[p6]=(UNS32)datu32; p6++;}
                } else if (strcmp(title,"Velinc") == 0) {
                    if (sscanf(chaine,"%19s ; %d",title,&dati32) == 2) slave_set_param("Velinc",slave_get_index_with_node(nodeID),dati32);
                } else if (strcmp(title,"TorqueHmtActive") == 0 || strcmp(title,"TorqueHmtControl") == 0) {
                    if (sscanf(chaine,"%19s ; %x",title,&datu32) == 2) {
                        if(!motor_set_param(nodeID,title,(INTEGER32)datu32)) {
                            printf("title %19s data %x\n",title,datu32); exit(1);
                            return 0;
                        }
                    }
                } else {
                    if (sscanf(chaine,"%19s ; %d",title,&dati32) == 2) {
                        if(!motor_set_param(nodeID,title,(INTEGER32)dati32)) {
                            printf("title %s data %d\n",title,dati32); exit(1);
                            return 0;
                        }
                    }
                }
            }
        }
        void* pdor[4]= {&pdor1,&pdor2,&pdor3,&pdor4};
        int i;
        for (i=0; i<4; i++) {
            UNS32 *datar = *(pdor+i);
            if (datar[0] != 0) {
                if(!cantools_PDO_map_config(nodeID,0x1600+i,datar[0],datar[1],datar[2],datar[3],datar[4],datar[5],datar[6],datar[7],0)) {
                    errgen_set(ERR_SLAVE_CONFIG_PDOR,slave_get_title_with_node(nodeID));
                    return 0;
                }
                if (!cantools_PDO_trans(nodeID,0x1400+i,0xFF,0x0000,0x0000)) {
                    errgen_set(ERR_SLAVE_CONFIG_ACTIVE_PDO,slave_get_title_with_node(nodeID));
                    return 0;
                }
            }
        }
        void* pdot[2]= {&pdot3,&pdot4};
        for (i=0; i<2; i++) {
            UNS32 *datat = *(pdot+i);
            if (datat[0] != 0) {
                if(!cantools_PDO_map_config(nodeID,0x1A00+2+i,datat[0],datat[1],datat[2],datat[3],datat[4],datat[5],datat[6],datat[7],0)) {
                    errgen_set(ERR_SLAVE_CONFIG_PDOR,slave_get_title_with_node(nodeID));
                    return 0;
                }
                if (!cantools_PDO_trans(nodeID,0x1800+2+i,0xFF,0x0000,0x0000)) {
                    errgen_set(ERR_SLAVE_CONFIG_ACTIVE_PDO,slave_get_title_with_node(nodeID));
                    return 0;
                }
            }
        }
        fclose(motor_fn);
    } else {
        errgen_set(ERR_FILE_OPEN,profile_get_filename_with_index(slave_get_profile_with_node(nodeID)));
        return 0;
    }
    return 1;
}
/**
* Génération du grid state
**/
gboolean slave_gui_load_state(gpointer data) {
    int i,j,k;
    char* key;
    GtkGrid* grid = gui_get_grid("gridState");
    GtkWidget* lab = gui_create_widget("lab","labStateTitle",MOTORS,"h1","black",NULL);
    gtk_widget_set_hexpand(lab,TRUE);
    gtk_grid_attach (grid, lab, 0, 0, SLAVE_NUMBER+1, 1);
    j=0;
    char* dat2plot[9] = {"SlaveTitle","StateImg","State","StateError","Power","PowerError","Volt","Temp",NULL};
    for (i=0;i<SLAVE_NUMBER;i++) {
        j++; k=0;
        while (dat2plot[k] != NULL) {
            if (dat2plot[k] == "SlaveTitle") key = strtools_concat(slave_get_param_in_char("SlaveTitle",i),"\n",slave_get_param_in_char("SlaveProfile",i),NULL);
            else key = DEFAULT;
            GtkWidget* lab = gui_create_widget("lab",strtools_concat("labM",strtools_gnum2str(&j,0x02),dat2plot[k],NULL),key,"cell",NULL);
            if (dat2plot[k] == "StateImg") {
                lab = gui_create_widget("img",strtools_concat("imgM",strtools_gnum2str(&j,0x02),dat2plot[k],NULL),slave_get_param_in_char("StateImg",i),"cell",NULL);
            }
            gtk_grid_attach (grid, lab, i+1, k+1, 1, 1);
            if (i == SLAVE_NUMBER-1) gtk_widget_set_margin_right (lab,3);
            if (dat2plot[k] == "State") {
                gtk_style_context_add_class (gtk_widget_get_style_context(lab), "cellTop");
                gtk_style_context_add_class (gtk_widget_get_style_context(lab), "bold");
            }
            if (dat2plot[k] == "SlaveTitle") {
                gtk_style_context_add_class (gtk_widget_get_style_context(lab), "bold");
                gtk_label_set_justify(GTK_LABEL(lab), GTK_JUSTIFY_CENTER);
                if (slave_get_param_in_num("SlaveProfile",i) == 0)
                    gtk_style_context_add_class (gtk_widget_get_style_context(lab), "green");
                else if (slave_get_param_in_num("SlaveProfile",i) == 1)
                    gtk_style_context_add_class (gtk_widget_get_style_context(lab), "purple");
            }
            k++;
        }
        GtkGrid* grid2 = GTK_GRID(gtk_grid_new());
        gtk_grid_attach (grid, GTK_WIDGET(grid2), i+1, k+1, 1, 1);
        gtk_widget_set_halign(GTK_WIDGET(grid2),GTK_ALIGN_FILL);
        gtk_widget_set_hexpand(GTK_WIDGET(grid2),TRUE);
        GtkWidget* but = gui_create_widget("but",strtools_concat("butActive",strtools_gnum2str(&j,0x02),NULL),NULL,"stdButton2","butRed",NULL);
        gtk_button_set_image(GTK_BUTTON(but),GTK_WIDGET(gtk_image_new_from_icon_name("user-offline",GTK_ICON_SIZE_DND)));
        GtkWidget* but2 = gui_create_widget("but",strtools_concat("butHelp",strtools_gnum2str(&j,0x02),NULL),NULL,"stdButton2","butBlack",NULL);
        gtk_button_set_image(GTK_BUTTON(but2),GTK_WIDGET(gtk_image_new_from_file("images/lock.png")));
        gtk_widget_set_halign(but,GTK_ALIGN_CENTER);
        gtk_widget_set_halign(but2,GTK_ALIGN_CENTER);
        gtk_widget_set_hexpand(but,TRUE);
        gtk_widget_set_hexpand(but2,TRUE);
        gtk_style_context_add_class (gtk_widget_get_style_context(GTK_WIDGET(grid2)), "cell");
        gtk_grid_attach (grid2, but, 0, 0, 1, 1);
        gtk_grid_attach (grid2, but2, 1, 0, 1, 1);
        g_signal_connect (G_OBJECT(but), "clicked", G_CALLBACK (on_butActive_clicked),&slaves[i].node);
        g_signal_connect (G_OBJECT(but2), "clicked", G_CALLBACK (on_butFree_clicked),&slaves[i].node);
    }
    gtk_widget_show_all (gui_get_widget("mainWindow"));
    return FALSE;
}
/** Generation de la page param **/
int slave_gui_param_gen(int ind) {
    if (gui_local_get_widget(gui_get_widget("boxParam"),"gridMotor") != NULL)
        gtk_widget_destroy(gui_local_get_widget(gui_get_widget("boxParam"),"gridMotor"));
    if (gui_local_get_widget(gui_get_widget("boxParam"),"gridProfile") != NULL)
        gtk_widget_destroy(gui_local_get_widget(gui_get_widget("boxParam"),"gridProfile"));
    if (gui_local_get_widget(gui_get_widget("boxParam"),"gridHelix") != NULL)
        gtk_widget_destroy(gui_local_get_widget(gui_get_widget("boxParam"),"gridHelix"));
    if (gui_local_get_widget(gui_get_widget("boxParam"),"gridGeom") != NULL)
        gtk_widget_destroy(gui_local_get_widget(gui_get_widget("boxParam"),"gridGeom"));
    if (gui_local_get_widget(gui_get_widget("boxParam"),"gridAsserv") != NULL)
        gtk_widget_destroy(gui_local_get_widget(gui_get_widget("boxParam"),"gridAsserv"));
    if (ind == 0) {
        // grid
        GtkGrid* grid = gui_local_grid_set("gridMotor",MOTOR_PARAM_TITLE,6,"black");
        gtk_box_pack_start (gui_get_box("boxParam"),GTK_WIDGET(grid),TRUE,TRUE,0);
        gtk_box_reorder_child(gui_get_box("boxParam"),GTK_WIDGET(grid),2);
        // but add
        GtkWidget* lab = gui_create_widget("but","butAddSlave",ADD,"stdButton","butBlue",NULL);
        gtk_button_set_image(GTK_BUTTON(lab),GTK_WIDGET(gtk_image_new_from_icon_name("gtk-add",GTK_ICON_SIZE_BUTTON)));
        gtk_widget_set_margin_right (lab,10);
        gtk_grid_attach(grid,lab,5,1,1,1);
        g_signal_connect (G_OBJECT(lab), "clicked", G_CALLBACK (on_butAddSlave_clicked),NULL);
        // but del
        GtkWidget* lab2 = gui_create_widget("but","butDelSlave",REMOVE,"stdButton","butBlue",NULL);
        gtk_button_set_image(GTK_BUTTON(lab2),GTK_WIDGET(gtk_image_new_from_icon_name("list-remove",GTK_ICON_SIZE_BUTTON)));
        gtk_widget_set_margin_left (lab2,10);
        gtk_grid_attach(grid,lab2,0,1,1,1);
        g_signal_connect (G_OBJECT(lab2), "clicked", G_CALLBACK (on_butDelSlave_clicked),NULL);
        //list to del
        GtkWidget* combdel = gui_create_widget("combo","listDel",NULL,NULL);
        gtk_grid_attach(grid,combdel,1,1,1,1);
        char* labtxt[7] = {"SlaveTitle","Vendor","Product","Revision","Serial","SlaveProfile",NULL};
        int i;
        for (i=0;i<6;i++) {
            GtkWidget* lab = gui_create_widget("lab",strtools_concat("labParamM",labtxt[i],NULL),
                slave_get_param_title(labtxt[i]),"bold",NULL);
            gtk_grid_attach(grid,lab,i,2,1,1);
            gtk_widget_set_margin_left (lab,13);
            if (i==0) gtk_widget_set_margin_left (lab,18);
            gtk_widget_set_halign(lab,GTK_ALIGN_START);

        }
        int j,k,l;
        for (i=0;i<SLAVE_NUMBER;i++) {
            j=i+1;
            gtk_combo_box_text_append(GTK_COMBO_BOX_TEXT(combdel),strtools_gnum2str(&j,0x02),strtools_concat(NODE," : ",strtools_gnum2str(&j,0x02),NULL));
            for (k=0;k<5;k++) {
                GtkWidget* lab = gui_create_widget("ent",strtools_concat("labParamM",strtools_gnum2str(&j,0x02),
                    labtxt[k],NULL),slave_get_param_in_char(labtxt[k],i),NULL);
                gtk_grid_attach(grid,lab,k,i+3,1,1);
                if (k==0) gtk_widget_set_margin_left (lab,10);
            }
            GtkWidget* comb = gui_create_widget("combo",strtools_concat("labParamM",strtools_gnum2str(&j,0x02),"SlaveProfile",NULL),NULL,NULL);
            for (l=0; l<PROFILE_NUMBER;l++) {
                gtk_combo_box_text_append(GTK_COMBO_BOX_TEXT(comb),strtools_gnum2str(&l,0x02),profiles[l].title);
            }
            gtk_grid_attach(grid,comb,5,i+3,1,1);
            gtk_widget_set_margin_right (comb,10);
            gtk_combo_box_set_active (GTK_COMBO_BOX(comb), slave_get_profile_with_index(i));
        }
    } else if (ind == 1) {
        // Grid
        GtkGrid* grid = gui_local_grid_set("gridProfile",PROFIL_PARAM_TITLE,4,"black");
        gtk_box_pack_start (gui_get_box("boxParam"),GTK_WIDGET(grid),TRUE,TRUE,0);
        gtk_box_reorder_child(gui_get_box("boxParam"),GTK_WIDGET(grid),2);
        GtkWidget* comb = gui_create_widget("combo","listProfile",NULL,NULL);
        gtk_grid_attach(grid,comb,0,1,1,1);
        int i;
        for (i=0; i<PROFILE_NUMBER; i++) {
            gtk_combo_box_text_append(GTK_COMBO_BOX_TEXT(comb),strtools_gnum2str(&i,0x02),profiles[i].title);
        }
        g_signal_connect (G_OBJECT(comb), "changed", G_CALLBACK (on_listProfile_changed),NULL);
    } else if (ind == 2) {
        // grid
        GtkGrid* grid = gui_local_grid_set("gridGeom",GEOM_PARAM_TITLE,5,"black");
        gtk_box_pack_start (gui_get_box("boxParam"),GTK_WIDGET(grid),TRUE,TRUE,0);
        gtk_box_reorder_child(gui_get_box("boxParam"),GTK_WIDGET(grid),2);
        // Labs
        GtkWidget* lab = gui_create_widget("lab","labLength2Pipe",strtools_concat(LENGTH_2_PIPE," : ",NULL),"fontBig","bold","cell2",NULL);
        gtk_grid_attach(grid,lab,0,1,1,1);
        GtkWidget* ent = gui_create_widget("ent","entLength2Pipe",NULL,NULL);
        gtk_grid_attach(grid,ent,1,1,1,1);
        gtk_widget_set_halign(lab,GTK_ALIGN_START);
        gtk_widget_set_halign(ent,GTK_ALIGN_START);
        // but add
        GtkWidget* lab2 = gui_create_widget("but","butAddSupport",ADD,"stdButton","butBlue",NULL);
        gtk_button_set_image(GTK_BUTTON(lab2),GTK_WIDGET(gtk_image_new_from_icon_name("gtk-add",GTK_ICON_SIZE_BUTTON)));
        gtk_grid_attach(grid,lab2,3,1,1,1);
        gtk_widget_set_margin_right (lab2,10);
        gtk_widget_set_halign(lab2,GTK_ALIGN_FILL);
        g_signal_connect (G_OBJECT(lab2), "clicked", G_CALLBACK (on_butAddSupport_clicked),NULL);
        // but del
        GtkWidget* lab3 = gui_create_widget("but","butDelSupport",REMOVE,"stdButton","butBlue",NULL);
        gtk_button_set_image(GTK_BUTTON(lab3),GTK_WIDGET(gtk_image_new_from_icon_name("list-remove",GTK_ICON_SIZE_BUTTON)));
        gtk_widget_set_margin_left (lab3,10);
        gtk_grid_attach(grid,lab3,4,2,1,1);
        gtk_widget_set_margin_right (lab3,10);
        gtk_widget_set_halign(lab3,GTK_ALIGN_FILL);
        g_signal_connect (G_OBJECT(lab3), "clicked", G_CALLBACK (on_butDelSupport_clicked),NULL);
        GtkWidget* comb = gui_create_widget("combo","listSupport",NULL,NULL);
        gtk_grid_attach(grid,comb,3,2,1,1);
        // Chargement des données
        FILE* geom_fn = fopen(FILE_GEOM_CONFIG,"r");
        if (geom_fn != NULL) {
            char title[20];
            int datreflect,dat1;
            int i=4,j;
            char chaine[1024] = "";
            while(fgets(chaine,1024,geom_fn) != NULL) {
                printf("%s\n",chaine);
                if (sscanf(chaine,"%19s %d",title,&datreflect) == 2 && strcmp(title,"Length2Pipe") == 0) {
                    gtk_entry_set_text(GTK_ENTRY(gui_local_get_widget(gui_get_widget("boxParam"),"entLength2Pipe")), strtools_gnum2str(&datreflect,0x04));
                }
                if (sscanf(chaine,"--%d",&dat1) == 1) {
                    if (i == 4) {
                        // Lab title
                        GtkWidget* lab4 = gui_create_widget("lab","labSupportTitle",SUPPORT_TITLE,"fontBig","bold","cell2",NULL);
                        gtk_grid_attach(GTK_GRID(grid),lab4,0,3,1,1);
                        gtk_widget_set_halign(lab4,GTK_ALIGN_START);
                        GtkWidget* lab5 = gui_create_widget("lab","labLengthTitle",SUPPORT_LENGTH,"fontBig","bold","cell2",NULL);
                        gtk_grid_attach(GTK_GRID(grid),lab5,1,3,1,1);
                        gtk_widget_set_halign(lab5,GTK_ALIGN_START);
                    }
                    j = i-3;
                    GtkWidget* lab = gui_create_widget("lab",strtools_concat("labSupport_",strtools_gnum2str(&j,0x04),NULL),strtools_gnum2str(&j,0x04),"fontBig","bold","cell2",NULL);
                    GtkWidget* ent1 = gui_create_widget("ent",strtools_concat("entSupport_",strtools_gnum2str(&j,0x04),NULL),strtools_gnum2str(&dat1,0x04),NULL);
                    gtk_widget_set_halign(lab,GTK_ALIGN_START);
                    gtk_widget_set_halign(ent1,GTK_ALIGN_START);

                    gtk_grid_attach(GTK_GRID(grid),lab,0,i,1,1);
                    gtk_grid_attach(GTK_GRID(grid),ent1,1,i,1,1);

                    gtk_combo_box_text_append(GTK_COMBO_BOX_TEXT(comb),strtools_gnum2str(&j,0x04),strtools_concat(SUPPORT_TITLE," : ",strtools_gnum2str(&j,0x02),NULL));
                    i++;
                }

            }
            fclose(geom_fn);
        }

    } else if (ind == 3) {
        // grid
        GtkGrid* grid = gui_local_grid_set("gridHelix",HELIX_PARAM_TITLE,10,"black");
        gtk_box_pack_start (gui_get_box("boxParam"),GTK_WIDGET(grid),TRUE,TRUE,0);
        gtk_box_reorder_child(gui_get_box("boxParam"),GTK_WIDGET(grid),2);
        // Labs
        GtkWidget* lab = gui_create_widget("lab","labTimeSet",strtools_concat(TIME_2_SET," : ",NULL),"fontBig","bold","cell2",NULL);
        gtk_grid_attach(grid,lab,0,1,1,1);
        GtkWidget* ent = gui_create_widget("ent","entTimeSet",NULL,NULL);
        gtk_grid_attach(grid,ent,1,1,1,1);
        gtk_widget_set_halign(lab,GTK_ALIGN_START);
        gtk_widget_set_halign(ent,GTK_ALIGN_START);
        GtkWidget* lab7 = gui_create_widget("lab","labPipeLength",strtools_concat(PIPE_LENGTH," : ",NULL),"fontBig","bold","cell2",NULL);
        gtk_grid_attach(grid,lab7,0,2,1,1);
        GtkWidget* ent7 = gui_create_widget("ent","entPipeLength",NULL,NULL);
        g_signal_connect (G_OBJECT(ent7), "changed", G_CALLBACK (on_lengthDef_changed),NULL);
        gtk_grid_attach(grid,ent7,1,2,1,1);
        gtk_widget_set_halign(ent7,GTK_ALIGN_START);
        gtk_widget_set_halign(lab7,GTK_ALIGN_START);
        // but add
        GtkWidget* lab2 = gui_create_widget("but","butAddStep",ADD,"stdButton","butBlue",NULL);
        gtk_button_set_image(GTK_BUTTON(lab2),GTK_WIDGET(gtk_image_new_from_icon_name("gtk-add",GTK_ICON_SIZE_BUTTON)));
        gtk_grid_attach(grid,lab2,8,1,1,1);
        gtk_widget_set_margin_right (lab2,10);
        gtk_widget_set_halign(lab2,GTK_ALIGN_FILL);
        g_signal_connect (G_OBJECT(lab2), "clicked", G_CALLBACK (on_butAddStep_clicked),NULL);
        // but del
        GtkWidget* lab3 = gui_create_widget("but","butDelStep",REMOVE,"stdButton","butBlue",NULL);
        gtk_button_set_image(GTK_BUTTON(lab3),GTK_WIDGET(gtk_image_new_from_icon_name("list-remove",GTK_ICON_SIZE_BUTTON)));
        gtk_widget_set_margin_left (lab3,10);
        gtk_grid_attach(grid,lab3,9,2,1,1);
        gtk_widget_set_margin_right (lab3,10);
        gtk_widget_set_halign(lab3,GTK_ALIGN_FILL);
        g_signal_connect (G_OBJECT(lab3), "clicked", G_CALLBACK (on_butDelStep_clicked),NULL);
        GtkWidget* comb = gui_create_widget("combo","listStep",NULL,NULL);
        gtk_grid_attach(grid,comb,8,2,1,1);
        // Chargement des données
        FILE* helix_fn = fopen(FILE_HELIX_CONFIG,"r");
        int dat3 = 0;
        if (helix_fn != NULL) {
            char title[20];
            int dattime, dat1,dat2,datpipe;
            int i=4,j;
            char chaine[1024] = "";
            while(fgets(chaine,1024,helix_fn) != NULL) {
                if (sscanf(chaine,"%19s %d",title,&dattime) == 2 && strcmp(title,"Time") == 0) {
                    gtk_entry_set_text(GTK_ENTRY(gui_local_get_widget(gui_get_widget("boxParam"),"entTimeSet")), strtools_gnum2str(&dattime,0x04));
                }
                if (sscanf(chaine,"%19s %d",title,&datpipe) == 2 && strcmp(title,"Pipe") == 0) {
                    gtk_entry_set_text(GTK_ENTRY(ent7), strtools_gnum2str(&datpipe,0x04));
                }

                if (sscanf(chaine,"%d %d",&dat1,&dat2) == 2) {
                    dat3+=dat2;
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
                    GtkWidget* lab = gui_create_widget("lab",strtools_concat("labHelix_",strtools_gnum2str(&j,0x04),NULL),strtools_gnum2str(&j,0x04),"fontBig","bold","cell2",NULL);
                    GtkWidget* ent1 = gui_create_widget("ent",strtools_concat("entHelix_",strtools_gnum2str(&j,0x04),NULL),strtools_gnum2str(&dat1,0x04),NULL);
                    GtkWidget* ent2 = gui_create_widget("ent",strtools_concat("entLength_",strtools_gnum2str(&j,0x04),NULL),strtools_gnum2str(&dat2,0x04),NULL);
                    g_signal_connect (G_OBJECT(ent2), "changed", G_CALLBACK (on_lengthDef_changed),NULL);

                    gtk_grid_attach(GTK_GRID(grid),lab,0,i,1,1);
                    gtk_grid_attach(GTK_GRID(grid),ent1,1,i,1,1);
                    gtk_grid_attach(GTK_GRID(grid),ent2,2,i,1,1);

                    gtk_combo_box_text_append(GTK_COMBO_BOX_TEXT(comb),strtools_gnum2str(&j,0x04),strtools_concat(HELIX_STEP," : ",strtools_gnum2str(&j,0x02),NULL));
                    i++;
                }
            }
            fclose(helix_fn);
        }
        GtkWidget* lab6 = gui_create_widget("lab","labLengthDefined",strtools_concat(LENGTH_DEFINED," : ",strtools_gnum2str(&dat3,0x02),NULL),"fontBBig","bold","cell2","greenColor",NULL);
        gtk_grid_attach(grid,lab6,2,2,2,1);
        gtk_widget_set_halign(lab6,GTK_ALIGN_START);
    } else if(ind == 4) {
        // grid
        GtkGrid* grid = gui_local_grid_set("gridAsserv",ASSERV_PARAM_TITLE,12,"black");
        gtk_box_pack_start (gui_get_box("boxParam"),GTK_WIDGET(grid),TRUE,TRUE,0);
        gtk_box_reorder_child(gui_get_box("boxParam"),GTK_WIDGET(grid),2);
        GtkWidget* lab1 = gui_create_widget("lab","labTransType",TRANSLATION_TYPE,"cell2","bold",NULL);
        gtk_widget_set_halign(lab1,GTK_ALIGN_START);
        GtkWidget* rad1 = gui_create_widget("radio","radCouple",TRANSLATION_TYPE_TORQUE,NULL);
        g_signal_connect (rad1, "toggled", G_CALLBACK (on_radCouple_toggled),NULL);
        gtk_widget_set_halign(rad1,GTK_ALIGN_START);
        GtkWidget* rad2 = gui_create_widget("radio","radVit",TRANSLATION_TYPE_SPEED,NULL);
        gtk_widget_set_halign(rad2,GTK_ALIGN_START);
        g_signal_connect (rad2, "toggled", G_CALLBACK (on_radVit_toggled),NULL);
        GtkWidget* rad3 = gui_create_widget("radio","radTreuil",TRANSLATION_TYPE_TREUIL,NULL);
        gtk_widget_set_halign(rad3,GTK_ALIGN_START);
        g_signal_connect (rad3, "toggled", G_CALLBACK (on_radTreuil_toggled),NULL);
        gtk_radio_button_join_group(GTK_RADIO_BUTTON(rad2),GTK_RADIO_BUTTON(rad1));
        gtk_radio_button_join_group(GTK_RADIO_BUTTON(rad3),GTK_RADIO_BUTTON(rad1));
        GtkWidget* lab2 = gui_create_widget("lab","labVitRotMax",ROT_VIT_MAX,"cell2","bold",NULL);
        gtk_widget_set_halign(lab2,GTK_ALIGN_START);
        GtkWidget* ent2 = gui_create_widget("ent","entVitRotMax",NULL,NULL);
        gtk_widget_set_halign(ent2,GTK_ALIGN_START);
        GtkWidget* lab3 = gui_create_widget("lab","labPeriodSync",SYNC_PERIOD,"cell2","bold",NULL);
        gtk_widget_set_halign(lab3,GTK_ALIGN_START);
        GtkWidget* ent3 = gui_create_widget("ent","entPeriodSync",NULL,NULL);
        gtk_widget_set_halign(ent3,GTK_ALIGN_START);
        GtkWidget* lab4 = gui_create_widget("lab","labPeriodCor",CORRECTION_PERIOD,"cell2","bold",NULL);
        gtk_widget_set_halign(lab4,GTK_ALIGN_START);
        GtkWidget* ent4 = gui_create_widget("ent","entPeriodCor",NULL,NULL);
        gtk_widget_set_halign(ent4,GTK_ALIGN_START);
        GtkWidget* lab6 = gui_create_widget("lab","labErrorMax",MAX_ERROR,"cell2","bold",NULL);
        gtk_widget_set_halign(lab6,GTK_ALIGN_START);
        GtkWidget* ent6 = gui_create_widget("ent","entErrorMax",NULL,NULL);
        gtk_widget_set_halign(ent6,GTK_ALIGN_START);
        GtkWidget* lab7 = gui_create_widget("lab","labRotDirection",ROT_DIRECTION,"cell2","bold",NULL);
        gtk_widget_set_halign(lab7,GTK_ALIGN_START);
        GtkWidget* rad4 = gui_create_widget("radio","radDroit",HELIX_RIGHT,NULL);
        gtk_widget_set_halign(rad1,GTK_ALIGN_START);
        GtkWidget* rad5 = gui_create_widget("radio","radGauche",HELIX_LEFT,NULL);
        gtk_widget_set_halign(rad2,GTK_ALIGN_START);
        gtk_radio_button_join_group(GTK_RADIO_BUTTON(rad5),GTK_RADIO_BUTTON(rad4));
        gtk_grid_attach(grid,lab1,0,1,1,1);
        gtk_grid_attach(grid,rad1,1,1,1,1);
        gtk_grid_attach(grid,rad2,2,1,1,1);
        gtk_grid_attach(grid,rad3,3,1,1,1);
        gtk_grid_attach(grid,lab7,0,3,1,1);
        gtk_grid_attach(grid,rad4,1,3,1,1);
        gtk_grid_attach(grid,rad5,2,3,1,1);
        gtk_grid_attach(grid,lab2,0,4,1,1);
        gtk_grid_attach(grid,ent2,1,4,1,1);
        gtk_grid_attach(grid,lab3,0,5,1,1);
        gtk_grid_attach(grid,ent3,1,5,1,1);
        gtk_grid_attach(grid,lab4,0,6,1,1);
        gtk_grid_attach(grid,ent4,1,6,1,1);
        gtk_grid_attach(grid,lab6,0,7,1,1);
        gtk_grid_attach(grid,ent6,1,7,1,1);
        FILE* asserv_fn = fopen(FILE_ASSERV_CONFIG,"r");
        if (asserv_fn != NULL) {
            char title[20];
            double datTransType,datRotDir,datTimeSync,datTimeCor,datMaxError,datVmax,datV;
            char chaine[1024] = "";
            while(fgets(chaine,1024,asserv_fn) != NULL) {
                if (sscanf(chaine,"%19s %lf",title,&datTransType) == 2 && strcmp(title,"TransType") == 0) {
                    if ((int)datTransType == 1) gtk_toggle_button_set_active(GTK_TOGGLE_BUTTON(rad1),TRUE);
                    if ((int)datTransType == 2) gtk_toggle_button_set_active(GTK_TOGGLE_BUTTON(rad2),TRUE);
                    if ((int)datTransType == 3) gtk_toggle_button_set_active(GTK_TOGGLE_BUTTON(rad3),TRUE);
                }
                if (sscanf(chaine,"%19s %lf",title,&datRotDir) == 2 && strcmp(title,"RotDirection") == 0) {
                    if ((int)datRotDir == 0) gtk_toggle_button_set_active(GTK_TOGGLE_BUTTON(rad4),TRUE);
                    if ((int)datRotDir == 1) gtk_toggle_button_set_active(GTK_TOGGLE_BUTTON(rad5),TRUE);
                }
                if (sscanf(chaine,"%19s %lf",title,&datV) == 2 && strcmp(title,"VitTrans") == 0) {
                    GtkWidget* ent5 = gui_local_get_widget(gui_get_widget("boxParam"),"entVitTrans");
                    if (ent5 != NULL)
                        gtk_entry_set_text(GTK_ENTRY(ent5), strtools_gnum2str(&datV,0x10));
                }
                if (sscanf(chaine,"%19s %lf",title,&datVmax) == 2 && strcmp(title,"VitRotMax") == 0) {
                    INTEGER32 dat = (INTEGER32)datVmax;
                    gtk_entry_set_text(GTK_ENTRY(ent2), strtools_gnum2str(&dat,0x04));
                }
                if (sscanf(chaine,"%19s %lf",title,&datTimeSync) == 2 && strcmp(title,"PeriodSync") == 0) {
                    INTEGER32 dat = (INTEGER32)datTimeSync;
                    gtk_entry_set_text(GTK_ENTRY(ent3), strtools_gnum2str(&dat,0x04));
                }
                if (sscanf(chaine,"%19s %lf",title,&datTimeCor) == 2 && strcmp(title,"PeriodCor") == 0) {
                    INTEGER32 dat = (INTEGER32)datTimeCor;
                    gtk_entry_set_text(GTK_ENTRY(ent4), strtools_gnum2str(&dat,0x04));
                }
                if (sscanf(chaine,"%19s %lf",title,&datMaxError) == 2 && strcmp(title,"ErrorMax") == 0) {
                    INTEGER32 dat = (INTEGER32)datMaxError;
                    gtk_entry_set_text(GTK_ENTRY(ent6), strtools_gnum2str(&dat,0x04));
                }
            }
            fclose(asserv_fn);
        }
    }
    gtk_widget_show_all(gui_get_widget("boxParam"));
}
/**
* Definition des esclaves
**/
int slave_read_definition() {
    FILE* fslave = fopen(FILE_SLAVE_CONFIG,"r");
    if (fslave != NULL) {
        char chaine[1024] = "";
        char title1[20],title2[20];
        UNS32 vendor,product,revision,serial;
        int profile,i1=0, i2=0;
        while(fgets(chaine,1024,fslave) != NULL) {
            if (sscanf(chaine,"--%19s ; %x ; %x ; %x ; %x ; %d",title1,&vendor,&product,&revision,&serial,&profile) == 6) {
                slaves[i1].node = (UNS8)i1 + 0x02;
                slaves[i1].title = g_strdup(title1);
                slaves[i1].vendor = vendor;
                slaves[i1].product = product;
                slaves[i1].revision = revision;
                slaves[i1].serial = serial;
                slaves[i1].active = 1;
                slaves[i1].profile = profile;
                slaves[i1].state = 0;
                slaves[i1].state_error = 0;
                i1++;
            }
//            if (sscanf(chaine,"##%19s",title2) == 1){
//                slave_profile[i2].title = g_strdup(title2);
//                i2++;
//            }
        }

        SLAVE_NUMBER = i1;
        fclose(fslave);
        if (i1 == 0) return 0;
    } else {
        errgen_set(ERR_FILE_OPEN,FILE_SLAVE_CONFIG);
        return 0;
    }
    return 1;
}
int slave_save_param (int index) {
    if (index == 0) {
        GtkWidget* grid = gui_local_get_widget(gui_get_widget("boxParam"),"gridMotor");
        GtkWidget* comb = gui_local_get_widget(grid,"listDel");

        int i = 3,j,k,l;
        char* labtxt[7] = {"SlaveTitle","Vendor","Product","Revision","Serial","SlaveProfile",NULL};
        char* str2build="";
        int ii=0,N = gtk_tree_model_iter_n_children(gtk_combo_box_get_model(GTK_COMBO_BOX(comb)),NULL);
        while(ii < N) {
            if (gtk_grid_get_child_at(GTK_GRID(grid),0,i) != NULL) {
                ii++;
                str2build = strtools_concat(str2build,"--",NULL);
                j=i-2;
                for (k=0;k<5;k++) {
                    const char* key = gtk_entry_get_text(GTK_ENTRY(gui_local_get_widget(grid,strtools_concat("labParamM",strtools_gnum2str(&j,0x02),labtxt[k],NULL))));
                    if (key == NULL) {
                        free(str2build);
                        return 0;
                    }
                    str2build = strtools_concat(str2build,(char*)key, " ; ",NULL);
                }
                l = gtk_combo_box_get_active(GTK_COMBO_BOX(gui_local_get_widget(grid,strtools_concat("labParamM",strtools_gnum2str(&j,0x02),"SlaveProfile",NULL))));
                if (l == -1) {
                    free(str2build);
                    return 0;
                }
                str2build = strtools_concat(str2build,strtools_gnum2str(&l,0x02), "\n",NULL);
                i++;
            }
        }
        if (!strtools_build_file(FILE_SLAVE_CONFIG,str2build)) {
            if (str2build != "") free(str2build);
            return 0;
        }
        if (str2build != "") free(str2build);
        gui_widget2hide("windowParams",NULL);
        if (run_init == -1) {
            if (!slave_read_definition()) {
                run_init = 0;
                errgen_set(ERR_FILE_PROFILE,profile_get_filename_with_index(i));
            }
            run_init = 1;
        } else {
            gui_info_box(RESET_APPLICATION_TITLE,RESET_APPLICATION_CONTENT,NULL);
            Exit(1);
            execlp("./SpirallingControl","SpirallingControl",NULL);
        }
        return 1;
    } else if (index == 1) {
        GtkWidget* grid = gui_local_get_widget(gui_get_widget("boxParam"),"gridProfile");
        GtkWidget* comb = gui_local_get_widget(gui_get_widget("boxParam"),"listDelField");
        char* str2build="";
        int i=3,data,j;
        int ii=0,N = gtk_tree_model_iter_n_children(gtk_combo_box_get_model(GTK_COMBO_BOX(comb)),NULL);
        while(ii < N) {
            if (gtk_grid_get_child_at(GTK_GRID(grid),0,i) != NULL) {
                ii++;
                j=i-2;
                const char* key = gtk_widget_get_name(gtk_grid_get_child_at(GTK_GRID(grid),0,i));
                const char* key2 = gtk_entry_get_text(GTK_ENTRY(gtk_grid_get_child_at(GTK_GRID(grid),1,i)));
                if (key == NULL ) {
                    if (str2build != "") free(str2build);
                    return 0;
                }
                if ( gtk_entry_get_text_length (GTK_ENTRY(gtk_grid_get_child_at(GTK_GRID(grid),1,i))) == 0)
                    return 0;
                if (sscanf(key,"labParamP%d_",&data) == 1) {
                    str2build = strtools_concat(str2build,pardata[data].gui_code, " ; ",key2,"\n",NULL);
                }
            }
            i++;
        }
        int ind = gtk_combo_box_get_active(GTK_COMBO_BOX(gui_local_get_widget(gui_get_widget("boxParam"),"listProfile")));
        if(!strtools_build_file(profile_get_filename_with_index(ind),str2build)) {
            if (str2build != "") free(str2build);
            return 0;
        }
        if (str2build != "") free(str2build);
        gui_widget2hide("windowParams",NULL);
        gui_info_box(SAVE,SAVE_SUCCESS,NULL);
        for (i=0; i<SLAVE_NUMBER;i++) {
            if (slave_get_param_in_num("SlaveProfile",i) == ind && slave_get_param_in_num("Active",i) == 1)
                slave_set_param("State",i,STATE_PREOP);
        }
        return 1;
    } else if (index == 2) {
        GtkWidget* grid = gui_local_get_widget(gui_get_widget("boxParam"),"gridGeom");
        GtkWidget* comb = gui_local_get_widget(gui_get_widget("boxParam"),"listSupport");
        int error = 0;
        char* str2build ="";
        if (gtk_entry_get_text_length (GTK_ENTRY(gtk_grid_get_child_at(GTK_GRID(grid),1,1))) == 0 ) {
            gui_info_popup(LENGTH_2_PIPE_ERROR,NULL);
            return 0;
        }
        const char* l2pipetxt = gtk_entry_get_text(GTK_ENTRY(gtk_grid_get_child_at(GTK_GRID(grid),1,1)));
        str2build = strtools_concat(str2build,"Length2Pipe ",l2pipetxt,"\n",NULL);

        int ii=0,i = 4, N = gtk_tree_model_iter_n_children(gtk_combo_box_get_model(GTK_COMBO_BOX(comb)),NULL);
        int dat1 = 0;
        if (N == 0) {
            gui_info_popup(SUPPORT_LENGTH_ERROR,NULL);
            return 0;
        }
        while(ii < N) {
            if (gtk_grid_get_child_at(GTK_GRID(grid),0,i) != NULL) {
                ii++;
                if (gtk_entry_get_text_length (GTK_ENTRY(gtk_grid_get_child_at(GTK_GRID(grid),1,i))) == 0 ) {
                    gui_info_popup(SUPPORT_LENGTH_ERROR,NULL);
                    return 0;
                }
                const char* ent1 = gtk_entry_get_text(GTK_ENTRY(gtk_grid_get_child_at(GTK_GRID(grid),1,i)));
                str2build = strtools_concat(str2build,"--",(char*)ent1,"\n",NULL);
            }
            i++;
        }
        if(!strtools_build_file(FILE_GEOM_CONFIG,str2build)) {
            if (str2build != "") free(str2build);
            return 0;
        }
        if (str2build != "") free(str2build);
        gui_widget2hide("windowParams",NULL);
        gui_info_box(SAVE,SAVE_SUCCESS,NULL);
        g_idle_add(slave_gui_load_visu,NULL);
        return 1;
    } else if (index == 3) {
        GtkWidget* grid = gui_local_get_widget(gui_get_widget("boxParam"),"gridHelix");
        GtkWidget* comb = gui_local_get_widget(gui_get_widget("boxParam"),"listStep");
        int error = 0;
        char *errortxt="";
        char* str2build ="";

        if (gtk_entry_get_text_length (GTK_ENTRY(gtk_grid_get_child_at(GTK_GRID(grid),1,1))) == 0 ) {
            gui_info_popup(TIME_ERROR,NULL);
            return 0;
        }
        const char* timetxt = gtk_entry_get_text(GTK_ENTRY(gtk_grid_get_child_at(GTK_GRID(grid),1,1)));
        int time = gui_str2num(timetxt);
        if (time > TIME_SET_LIMIT)
            str2build = strtools_concat(str2build,"Time ",timetxt,"\n",NULL);
        else {
            int t = TIME_SET_LIMIT;
            gui_info_popup(strtools_concat(TIME_MIN_ERROR, " ", strtools_gnum2str(&t,0x04)," s",NULL),NULL);
            return 0;
        }
        if (gtk_entry_get_text_length (GTK_ENTRY(gtk_grid_get_child_at(GTK_GRID(grid),1,2))) == 0 ) {
            gui_info_popup(PIPE_LENGTH_ERROR,NULL);
            return 0;
        }
        const char* pipeLengthtxt = gtk_entry_get_text(GTK_ENTRY(gtk_grid_get_child_at(GTK_GRID(grid),1,2)));
        int pipeLength = gui_str2num(pipeLengthtxt);
        str2build = strtools_concat(str2build,"Pipe ",pipeLengthtxt,"\n",NULL);
        int ii=0,i = 4, N = gtk_tree_model_iter_n_children(gtk_combo_box_get_model(GTK_COMBO_BOX(comb)),NULL);
        int dat1 = 0;
        while(ii < N) {
            if (gtk_grid_get_child_at(GTK_GRID(grid),0,i) != NULL) {
                ii++;
                if (gtk_entry_get_text_length (GTK_ENTRY(gtk_grid_get_child_at(GTK_GRID(grid),1,i))) == 0 ) {
                    gui_info_popup(STEP_ERROR,NULL);
                    return 0;
                }
                if (gtk_entry_get_text_length (GTK_ENTRY(gtk_grid_get_child_at(GTK_GRID(grid),2,i))) == 0 ) {
                    gui_info_popup(LENGTH_ERROR,NULL);
                    return 0;
                }
                const char* ent1 = gtk_entry_get_text(GTK_ENTRY(gtk_grid_get_child_at(GTK_GRID(grid),1,i)));
                const char* ent2 = gtk_entry_get_text(GTK_ENTRY(gtk_grid_get_child_at(GTK_GRID(grid),2,i)));
                if (gui_str2num(ent1) < STEP_LIMIT && gui_str2num(ent1) != 0 ) {
                    gui_info_popup(STEP_ERROR,NULL);
                    return 0;
                }
                dat1 += gui_str2num(gtk_entry_get_text(GTK_ENTRY(gtk_grid_get_child_at(GTK_GRID(grid),2,i))));
                str2build = strtools_concat(str2build,(char*)ent1," ",ent2,"\n",NULL);
            }
            i++;
        }
        if (pipeLength < dat1) {
            gui_info_popup(OVER_LENGTH_ERROR,NULL);
            return 0;
        }

        if(!strtools_build_file(FILE_HELIX_CONFIG,str2build)) {
            if (str2build != "") free(str2build);
            return 0;
        }
        if (str2build != "") free(str2build);
        gui_widget2hide("windowParams",NULL);
        gui_info_box(SAVE,SAVE_SUCCESS,NULL);
        g_idle_add(slave_gui_load_visu,NULL);
        return 1;
    } else if(index == 4) {
        GtkWidget* grid = gui_local_get_widget(gui_get_widget("boxParam"),"gridAsserv");
        int N = CONST_NUMBER_LIMIT+1,i,ii;
        i = 1;
        char* str2build = "";
        while(i < N) {
            if (gtk_grid_get_child_at(GTK_GRID(grid),0,i) != NULL) {
                const char* name = gtk_widget_get_name(gtk_grid_get_child_at(GTK_GRID(grid),1,i));
                if (asserv_get_const_index_with_id((char*)name) != -1) {
                    if (gtk_entry_get_text_length (GTK_ENTRY(gtk_grid_get_child_at(GTK_GRID(grid),1,i))) == 0 ) {
                        gui_info_popup(EMPTY_FIELD_ERROR,NULL);
                        return 0;
                    }
                    double val = gui_str2double(gtk_entry_get_text(GTK_ENTRY(gtk_grid_get_child_at(GTK_GRID(grid),1,i))));
                    if(!asserv_check_const(asserv_get_const_name_with_id((char*)name),val)) return 0;
                    str2build = strtools_concat(str2build,asserv_get_const_name_with_id((char*)name)," ",strtools_gnum2str(&val,0x10),"\n",NULL);
                }
                if (strcmp(gtk_widget_get_name(gtk_grid_get_child_at(GTK_GRID(grid),0,i)),strtools_concat("lab","TransType",NULL)) == 0) {
                    if(gtk_toggle_button_get_active(GTK_TOGGLE_BUTTON(gtk_grid_get_child_at(GTK_GRID(grid),1,i))))
                        str2build = strtools_concat(str2build,"TransType 1\n",NULL);
                    if(gtk_toggle_button_get_active(GTK_TOGGLE_BUTTON(gtk_grid_get_child_at(GTK_GRID(grid),2,i))))
                        str2build = strtools_concat(str2build,"TransType 2\n",NULL);
                    if(gtk_toggle_button_get_active(GTK_TOGGLE_BUTTON(gtk_grid_get_child_at(GTK_GRID(grid),3,i))))
                        str2build = strtools_concat(str2build,"TransType 3\n",NULL);
                }
                if (strcmp(gtk_widget_get_name(gtk_grid_get_child_at(GTK_GRID(grid),0,i)),strtools_concat("lab","RotDirection",NULL)) == 0) {
                    if(gtk_toggle_button_get_active(GTK_TOGGLE_BUTTON(gtk_grid_get_child_at(GTK_GRID(grid),1,i))))
                        str2build = strtools_concat(str2build,"RotDirection 0\n",NULL);
                    if(gtk_toggle_button_get_active(GTK_TOGGLE_BUTTON(gtk_grid_get_child_at(GTK_GRID(grid),2,i))))
                        str2build = strtools_concat(str2build,"RotDirection 1\n",NULL);
                }
            }
            i++;
        }
        if(!strtools_build_file(FILE_ASSERV_CONFIG,str2build)) {
            if (str2build != "") free(str2build);
            return 0;
        }
        if (str2build != "") free(str2build);
        if(!slave_load_const()) {printf("Error\n");return 0;}
        gui_widget2hide("windowParams",NULL);
        gui_info_box(SAVE,SAVE_SUCCESS,NULL);
    }
}
/**
* Check des fichiers profils
**/
int slave_check_profile_file(int profId) {
    int i,j=0,res;
    FILE *f = fopen(profile_get_filename_with_index(profId),"r");
    printf("File %s\n",profile_get_filename_with_index(profId));
    if (f != NULL) {
        char title[20]="";
        int data;
        char chaine[1024]="";
        while(fgets(chaine,1024,f) != NULL) {
            if (sscanf(chaine,"%19s ; %d",title,&data) == 2) j++;
                res = 0;
                for (i=0; i<PARAM_NUMBER; i++) {
                    if (strcmp(pardata[i].gui_code,title) == 0 ) res = 1;
                }
                if (res == 0) {
                    j = 0;
                    errgen_set(ERR_FILE_PROFILE_INVALID_PARAM,profile_get_filename_with_index(profId));
                }
        }
        fclose(f);
        if (j > 0) return 1;
        else {
            strtools_build_file(profile_get_filename_with_index(profId)," ");
            errgen_set(ERR_FILE_EMPTY,profile_get_filename_with_index(profId));
        }
    } else {
        errgen_set(ERR_FILE_OPEN,profile_get_filename_with_index(profId));
        if (!strtools_build_file(profile_get_filename_with_index(profId)," ")) {
            errgen_set(ERR_FILE_GEN,profile_get_filename_with_index(profId));
            return 0;
        }
    }
    return 1;
}

/**
* Retourne node en fonction de i
**/
UNS8 slave_get_node_with_index(int i) {
    if (i<SLAVE_NUMBER) {
        pthread_mutex_lock (&lock_slave);
        UNS8 dat = slaves[i].node;
        pthread_mutex_unlock (&lock_slave);
        return dat;
    } else {
        printf("1Index trop grand : %d \n",i); exit(EXIT_FAILURE);
    }
}
/**
* Retourne node en fonction du profil
* Attention plusieurs noeuds peuvent avoir le meme profil
**/
UNS8 slave_get_node_with_profile(int profInd) {
    int i;
    UNS8 res = 0x00;
    pthread_mutex_lock (&lock_slave);
    for (i=0; i<SLAVE_NUMBER; i++) {
        if (slaves[i].profile == profInd)
            res=slaves[i].node;
    }
    pthread_mutex_unlock (&lock_slave);
    return res;
}
UNS8 slave_get_node_with_profile_id(char* pid) {
    int i, profInd = profile_get_index_with_id(pid);
    UNS8 res = 0x00;
    pthread_mutex_lock (&lock_slave);
    for (i=0; i<SLAVE_NUMBER; i++) {
        if (slaves[i].profile == profInd)
            res=slaves[i].node;
    }
    pthread_mutex_unlock (&lock_slave);
    return res;
}

/**
* Retourne state en fonction index
**/
static int slave_get_state_with_index(int i) {
    if (i<SLAVE_NUMBER) {
        pthread_mutex_lock (&lock_slave);
        int dat = slaves[i].state;
        pthread_mutex_unlock (&lock_slave);
        return dat;
    } else {
        printf("Index trop grand : %d \n",i); exit(EXIT_FAILURE);
    }
}

/**
* Affectation du state en fonction index
**/
static void slave_set_state_with_index(int i, int dat) {
    if (i<SLAVE_NUMBER) {
        pthread_mutex_lock (&lock_slave);
        slaves[i].state = dat;
        pthread_mutex_unlock (&lock_slave);
    } else {
        printf("Index trop grand : %d \n",i); exit(EXIT_FAILURE);
    }
}
/**
* Retourne stateerror en fonction index
**/
static int slave_get_state_error_with_index(int i) {
    if (i<SLAVE_NUMBER) {
        pthread_mutex_lock (&lock_slave);
        int dat = slaves[i].state_error;
        pthread_mutex_unlock (&lock_slave);
        return dat;
    } else {
        printf("Index trop grand : %d",i);
        exit(EXIT_FAILURE);
    }
}
/**
* Affectation du state_error en fonction de l'index
**/
static void slave_set_state_error_with_index (int i, int dat) {
    if (i<SLAVE_NUMBER) {
        pthread_mutex_lock (&lock_slave);
        slaves[i].state_error = dat;
        pthread_mutex_unlock (&lock_slave);
    } else {
        printf("Index trop grand : %d",i);
        exit(EXIT_FAILURE);
    }
}


static void slave_set_active_with_index(int i, int var) {
    if (i<SLAVE_NUMBER) {
        pthread_mutex_lock (&lock_slave);
        slaves[i].active = var;
        pthread_mutex_unlock (&lock_slave);
    } else {
        printf("Index trop grand : %d \n",i); exit(EXIT_FAILURE);
    }
}
static int slave_get_active_with_index(int i) {
    if (i<SLAVE_NUMBER) {
        pthread_mutex_lock (&lock_slave);
        int var = slaves[i].active;
        pthread_mutex_unlock (&lock_slave);
        return var;
    } else {
        printf("Index trop grand : %d \n",i); exit(EXIT_FAILURE);
    }
}

/**
* Retourne index en fonction du nodeID
**/
int slave_get_index_with_node(UNS8 nodeID) {
    int i,res = -1;
    pthread_mutex_lock (&lock_slave);
    for (i=0; i<SLAVE_NUMBER; i++) {
        if (slaves[i].node == nodeID) res=i;
    }
    pthread_mutex_unlock (&lock_slave);
    if (res != -1)
        return res;
    else {
        printf("nodeID non trouvé %#.2x",nodeID);
        exit(EXIT_FAILURE);
    }
}
int slave_get_index_with_profile_id(char* pid) {
    int i,res = -1;
    int profind = profile_get_index_with_id(pid);
    pthread_mutex_lock (&lock_slave);
    for (i=0; i<SLAVE_NUMBER; i++) {
        if (slaves[i].profile == profind) res=i;
    }
    pthread_mutex_unlock (&lock_slave);
    if (res != -1)
        return res;
    else {
        printf("Profile non trouvé %s",pid);
        exit(EXIT_FAILURE);
    }
}/**
* retourne profile en fonction de l'index
**/
int slave_get_profile_with_index(int i) {
    if (i<SLAVE_NUMBER) {
        int dat;
        pthread_mutex_lock (&lock_slave);
        dat = slaves[i].profile;
        pthread_mutex_unlock (&lock_slave);
        return dat;
    } else {
        printf("Index trop grand : %d",i);
       // exit(EXIT_FAILURE);
    }
}
char* slave_get_profile_id_with_index(int i) {
    return profile_get_id_with_index(slave_get_profile_with_index(i));
}
void slave_set_profile_with_index(int i, int dat) {
    if (i<SLAVE_NUMBER) {
        pthread_mutex_lock (&lock_slave);
        slaves[i].profile = dat;
        pthread_mutex_unlock (&lock_slave);
    } else {
        printf("Index trop grand : %d",i);
        exit(EXIT_FAILURE);
    }
}
static UNS32 slave_get_vendor_with_index(int i) {
    if (i<SLAVE_NUMBER) {
        UNS32 dat;
        pthread_mutex_lock (&lock_slave);
        dat = slaves[i].vendor;
        pthread_mutex_unlock (&lock_slave);
        return dat;
    } else {
        printf("Index trop grand : %d",i);
        exit(EXIT_FAILURE);
    }
}
static UNS32 slave_get_product_with_index(int i) {
    if (i<SLAVE_NUMBER) {
        UNS32 dat;
        pthread_mutex_lock (&lock_slave);
        dat = slaves[i].product;
        pthread_mutex_unlock (&lock_slave);
        return dat;
    } else {
        printf("Index trop grand : %d",i);
        exit(EXIT_FAILURE);
    }
}
static UNS32 slave_get_revision_with_index(int i) {
    if (i<SLAVE_NUMBER) {
        UNS32 dat;
        pthread_mutex_lock (&lock_slave);
        dat = slaves[i].revision;
        pthread_mutex_unlock (&lock_slave);
        return dat;
    } else {
        printf("Index trop grand : %d",i);
        exit(EXIT_FAILURE);
    }
}
static UNS32 slave_get_serial_with_index(int i) {
    if (i<SLAVE_NUMBER) {
        UNS32 dat;
        pthread_mutex_lock (&lock_slave);
        dat = slaves[i].serial;
        pthread_mutex_unlock (&lock_slave);
        return dat;
    } else {
        printf("Index trop grand : %d",i);
        exit(EXIT_FAILURE);
    }
}/**
* retourne profile en fonction du noeud
**/
int slave_get_profile_with_node(UNS8 node) {
    int i = slave_get_index_with_node(node);
    int dat;
    pthread_mutex_lock (&lock_slave);
    dat = slaves[i].profile;
    pthread_mutex_unlock (&lock_slave);
    return dat;
}

/**
* Retourne le titre en fonction de i
**/
char* slave_get_title_with_index(int i) {
    char* dat;
    pthread_mutex_lock (&lock_slave);
    dat = slaves[i].title;
    pthread_mutex_unlock (&lock_slave);
    return dat;
}
/**
* Retourne le titre en fonction de node
**/
char* slave_get_title_with_node(UNS8 node) {
    int i = slave_get_index_with_node(node);
    char* dat;
    pthread_mutex_lock (&lock_slave);
    dat = slaves[i].title;
    pthread_mutex_unlock (&lock_slave);
    return dat;
}
/**
* Retourne la correspondance en texte de l'état
**/
static char* slave_get_state_title(int state) {
    if(state == STATE_LSS_CONFIG) return STATE_LSS_CONFIG_TXT;
    else if(state == STATE_PREOP) return STATE_PREOP_TXT;
    else if(state == STATE_CONFIG) return STATE_CONFIG_TXT;
    else if(state == STATE_OP) return STATE_OP_TXT;
    else if(state == STATE_SON) return STATE_SON_TXT;
    else if(state == STATE_READY) return STATE_READY_TXT;
    else if(state == STATE_DISCONNECTED) return STATE_DISCONNECTED_TXT;
    else return DEFAULT;
}
/**
* Retourne le nom de l'icone correspondant à l'état
**/
char* slave_get_state_img (int state) {
    if(state == STATE_LSS_CONFIG) return "gtk-properties";
    else if(state == STATE_PREOP) return "media-seek-forward";
    else if(state == STATE_CONFIG) return "gtk-execute";
    else if(state == STATE_OP) return "go-next";
    else if(state == STATE_SON) return "gtk-connect";
    else if(state == STATE_READY) return "gtk-yes";
    else if(state == STATE_DISCONNECTED) return "user-offline";
    else return "gtk-no";
}
/**
* Retourne la correspondance en texte de l'état erreur
**/
static char* slave_get_state_error_title(int state) {
    if(state == ERROR_STATE_NULL) return ERROR_STATE_NULL_TXT;
    else if(state == ERROR_STATE_LSS) return ERROR_STATE_LSS_TXT;
    else if(state == ERROR_STATE_CONFIG) return ERROR_STATE_CONFIG_TXT;
    else if(state == ERROR_STATE_OP) return ERROR_STATE_OP_TXT;
    else if(state == ERROR_STATE_SON) return ERROR_STATE_SON_TXT;
    else if(state == ERROR_STATE_VOLTAGE) return ERROR_STATE_VOLTAGE_TXT;
    else return DEFAULT;
}
char* slave_get_profile_name(int index) {
    return profile_get_title_with_index(slave_get_profile_with_index(index));
}
/**
* Retourne en INTEGER32 le paramètre
**/
INTEGER32 slave_get_param_in_num(char* parid, int index) {
    int i;
    for (i=0;i<VAR_NUMBER;i++) {
        if (parid == vardata[i].id) {
            if (parid == "Volt") {
                UNS16 *data = *(vardata[i].tab+index);
                INTEGER32 data2= (INTEGER32)*data;
                data2 = data2/10;
                return data2;
            } else if (parid == "Velinc") {
                INTEGER32* data = (INTEGER32*)vardata[i].tab;
                return (INTEGER32)data[index];
            } else if (parid == "State") {
                return (INTEGER32)slave_get_state_with_index(index);
            } else if (parid == "StateError") {
                return (INTEGER32)slave_get_state_with_index(index);
            } else if (parid == "SlaveProfile") {
                return (INTEGER32)slave_get_profile_with_index(index);
            } else if (parid == "Vendor") {
                return (INTEGER32)slave_get_vendor_with_index(index);
            } else if (parid == "Product") {
                return (INTEGER32)slave_get_product_with_index(index);
            } else if (parid == "Revision") {
                return (INTEGER32)slave_get_revision_with_index(index);
            } else if (parid == "Serial") {
                return (INTEGER32)slave_get_serial_with_index(index);
            } else if (parid == "Active") {
                return (INTEGER32)slave_get_active_with_index(index);
            } else {
                if (vardata[i].type == 0x02) {
                    INTEGER8 *data = *(vardata[i].tab+index);
                    return (INTEGER32)*data;
                } else if (vardata[i].type == 0x03) {
                    INTEGER16 *data = *(vardata[i].tab+index);
                    return (INTEGER32)*data;
                } else if (vardata[i].type == 0x04) {
                    INTEGER32 *data = *(vardata[i].tab+index);
                    return (INTEGER32)*data;
                } else if (vardata[i].type == 0x05) {
                    UNS8 *data = *(vardata[i].tab+index);
                    return (INTEGER32)*data;
                } else if (vardata[i].type == 0x06) {
                    UNS16 *data = *(vardata[i].tab+index);
                    return (INTEGER32)*data;
                } else if (vardata[i].type == 0x07) {
                    UNS32 *data = *(vardata[i].tab+index);
                    return (INTEGER32)*data;
                }
            }
        }
    }
}
/**
* Affecte un paramètre
**/
void slave_set_param(char* parid, int index,INTEGER32 dat) {

    int i, res=0;
    for (i=0;i<VAR_NUMBER;i++) {
        if (parid == vardata[i].id) {
            res = 1;
            if (parid == "Velinc") {
                INTEGER32* data = (INTEGER32*)vardata[i].tab;
                data[index]=(INTEGER32)dat;
            } else if (parid == "State") {
                slave_set_state_with_index(index,dat);
            } else if (parid == "StateError") {
                slave_set_state_error_with_index(index,dat);
            } else if (parid == "Active") {
                slave_set_active_with_index(index,dat);
                g_idle_add(keyword_active_maj,&slaves[index].node);
            } else if (parid == "SlaveProfile") {
                slave_set_profile_with_index(index,dat);
            } else {
                if (vardata[i].type == 0x02) {
                    INTEGER8 *data = *(vardata[i].tab+index);
                    *data = (INTEGER8)dat;
                } else if (vardata[i].type == 0x03) {
                    INTEGER16 *data = *(vardata[i].tab+index);
                    *data = (INTEGER16)dat;
                } else if (vardata[i].type == 0x04) {
                    printf("1index %d dat %d %d\n",index,dat,slave_get_param_in_num("Vel2send",slave_get_index_with_profile_id("RotVit")));
                    INTEGER32 *data = *(vardata[i].tab+index);
                    *data = (INTEGER32)dat;
                    printf("2index %d dat %d %d\n",index,dat,slave_get_param_in_num("Vel2send",slave_get_index_with_profile_id("RotVit")));
                } else if (vardata[i].type == 0x05) {
                    UNS8 *data = *(vardata[i].tab+index);
                    *data = (UNS8)dat;
                } else if (vardata[i].type == 0x06) {
                    UNS16 *data = *(vardata[i].tab+index);
                    *data = (UNS16)dat;
                } else if (vardata[i].type == 0x07) {
                    UNS32 *data = *(vardata[i].tab+index);
                    *data = (UNS32)dat;
                }
            }
        }
    }
    if (res == 0) {
        printf("PARID %s non trouvé \n",parid);
        exit(1);
    }
}
/**
* Retourne en char* le paramètre
**/
char* slave_get_param_in_char(char* parid, int index) {
    int i;
    int res = 0;
    for (i=0;i<VAR_NUMBER;i++) {
        if (parid == vardata[i].id) {
            res = 1;
            if (parid == "Volt") {
                UNS16 *data = *(vardata[i].tab+index);
                INTEGER32 data2= (INTEGER32)*data;
                data2 = data2/10;
                return strtools_gnum2str(&data2,0x04);
            } else if (parid == "SlaveTitle") {
                return slave_get_title_with_index(index);
            } else if (parid == "StateImg") {
                return slave_get_state_img(slave_get_param_in_num("State",index));
            } else if (parid == "SlaveProfile") {
                return profiles[slave_get_profile_with_index(index)].title;
            } else if (parid == "Vendor") {
                UNS32 dat = slave_get_vendor_with_index(index);
                return strtools_gnum2str(&dat,0x07);
            } else if (parid == "Product") {
                UNS32 dat = slave_get_product_with_index(index);
                return strtools_gnum2str(&dat,0x07);
            } else if (parid == "Revision") {
                UNS32 dat = slave_get_revision_with_index(index);
                return strtools_gnum2str(&dat,0x07);
            } else if (parid == "Serial") {
                UNS32 dat = slave_get_serial_with_index(index);
                return strtools_gnum2str(&dat,0x07);
            } else if (parid == "State") {
                return slave_get_state_title(slave_get_state_with_index(index));
            } else if (parid == "StateError") {
                return slave_get_state_error_title(slave_get_state_error_with_index(index));
            } else if (parid == "Power") {
                UNS16*data = *(vardata[i].tab+index);
                return motor_get_state_title(motor_get_state(*data));
            } else if (parid == "PowerError") {
                UNS16*data = *(vardata[i].tab+index);
                return motor_get_error_title(*data);
            } else
            return strtools_gnum2str(*(vardata[i].tab+index),vardata[i].type);
        }
    }
    if (res == 0) {
        printf("Par ID non trouvé %s",parid);
        exit(1);
    }

}

char* slave_get_param_title (char* parid) {
    int i;
    for (i=0;i<VAR_NUMBER;i++) {
        if (parid == vardata[i].id)
            return vardata[i].title;
    }
    printf("parid : %s non trouvé",parid);
    exit(1);
}

gboolean slave_gui_load_visu(gpointer data) {
    int i,j,valid=0,i1=0,i2=0;
    for (i=0;i<20;i++) {
        for (j=0;j<2;j++){
            step[j][i] = 0;
        }
        support[i] = 0;
    }
    conf1.step_size = 0;
    conf1.support_size = 0;
    FILE* helix_fn = fopen(FILE_HELIX_CONFIG,"r");
    if (helix_fn != NULL) {
        char title[20];
        int dattime, dat1,dat2,datpipe;
        int i=4,j;
        char chaine[1024] = "";
        while(fgets(chaine,1024,helix_fn) != NULL) {
            if (sscanf(chaine,"%19s %d",title,&dattime) == 2 && strcmp(title,"Time") == 0) {
                conf1.time = dattime;
                valid++;
            }
            if (sscanf(chaine,"%19s %d",title,&datpipe) == 2 && strcmp(title,"Pipe") == 0) {
                conf1.pipeLength = datpipe;
                valid++;
            }
            if (sscanf(chaine,"%d %d",&dat1,&dat2) == 2) {
                step[0][i1] = dat1;
                step[1][i1] = dat2;
                i1++;
                conf1.step_size = i1;
            }
        }
        fclose(helix_fn);
    }
    FILE* geom_fn = fopen(FILE_GEOM_CONFIG,"r");
    if (geom_fn != NULL) {
        char title[20];
        int datreflect,dat3;
        int i=4,j;
        char chaine[1024] = "";
        while(fgets(chaine,1024,geom_fn) != NULL) {
            if (sscanf(chaine,"%19s %d",title,&datreflect) == 2 && strcmp(title,"Length2Pipe") == 0) {
                conf1.length2pipe = datreflect;
                valid++;
            }
            if (sscanf(chaine,"--%d",&dat3) == 1) {
                support[i2] = dat3;
                i2++;
                conf1.support_size = i2;
            }
        }
        fclose(geom_fn);
    }
    // Grid
    GtkWidget* gridPar = gui_get_widget("gridVisu");
    if (gui_local_get_widget(gridPar,"gridHelix") != NULL)
        gtk_widget_destroy(gui_local_get_widget(gridPar,"gridHelix"));
    if (gui_local_get_widget(gridPar,"gridVel") != NULL)
        gtk_widget_destroy(gui_local_get_widget(gridPar,"gridVel"));
    if (gui_local_get_widget(gridPar,"gridErr") != NULL)
        gtk_widget_destroy(gui_local_get_widget(gridPar,"gridErr"));
    if (gui_local_get_widget(gridPar,"gridErrlab") != NULL)
        gtk_widget_destroy(gui_local_get_widget(gridPar,"gridErrlab"));
    GtkGrid* grid = gui_local_grid_set("gridHelix",NULL,conf1.pipeLength,"");
    GtkGrid* gridVel = gui_local_grid_set("gridVel",NULL,1,"");
    GtkGrid* gridErr = gui_local_grid_set("gridErr",NULL,1,"");
    GtkGrid* gridErrlab = gui_local_grid_set("gridErrlab",NULL,1,"");
    gtk_grid_attach(GTK_GRID(gridPar),GTK_WIDGET(grid),0,0,1,2);
    gtk_grid_attach(GTK_GRID(gridPar),GTK_WIDGET(gridVel),1,0,1,1);
    gtk_grid_attach(GTK_GRID(gridPar),GTK_WIDGET(gridErr),1,1,1,2);
    gtk_grid_attach(GTK_GRID(gridPar),GTK_WIDGET(gridErrlab),0,2,1,1);
    GtkWidget* labErr = gui_create_widget("lab",NULL,NULL,"cell",NULL);
    GtkWidget* labErr1 = gui_create_widget("lab","labErrInst",ERROR_INST,"cell","bold",NULL);
    GtkWidget* labErr2 = gui_create_widget("lab","labErrMean",ERROR_MEAN,"cell","bold","botBottom",NULL);
    GtkWidget* labErr3 = gui_create_widget("lab","labUnitMm",UNIT_MM,"cell","bold",NULL);
    GtkWidget* labErr4 = gui_create_widget("lab","labUnitStep",UNIT_STEP,"cell","bold","botRight",NULL);
    GtkWidget* labErr5 = gui_create_widget("lab","labErrInstMm",DEFAULT,"cell",NULL);
    GtkWidget* labErr6 = gui_create_widget("lab","labErrInstStep",DEFAULT,"cell","botRight",NULL);
    GtkWidget* labErr7 = gui_create_widget("lab","labErrMeanMm",DEFAULT,"cell","botBottom",NULL);
    GtkWidget* labErr8 = gui_create_widget("lab","labErrMeanStep",DEFAULT,"cell","botRight","botBottom",NULL);

    gtk_grid_attach(gridErrlab,labErr,0,0,1,1);
    gtk_grid_attach(gridErrlab,labErr1,0,1,1,1);
    gtk_grid_attach(gridErrlab,labErr2,0,2,1,1);
    gtk_grid_attach(gridErrlab,labErr3,1,0,1,1);
    gtk_grid_attach(gridErrlab,labErr4,2,0,1,1);
    gtk_grid_attach(gridErrlab,labErr5,1,1,1,1);
    gtk_grid_attach(gridErrlab,labErr6,2,1,1,1);
    gtk_grid_attach(gridErrlab,labErr7,1,2,1,1);
    gtk_grid_attach(gridErrlab,labErr8,2,2,1,1);
    gtk_widget_set_halign(labErr1,GTK_ALIGN_START);
    gtk_widget_set_halign(labErr2,GTK_ALIGN_START);
    if (valid == 3 && i1>0 && i2>0) {
        GtkWidget* lev = gtk_level_bar_new();
        gtk_level_bar_set_mode(GTK_LEVEL_BAR(lev),GTK_LEVEL_BAR_MODE_CONTINUOUS);
        gtk_level_bar_set_min_value(GTK_LEVEL_BAR(lev),0);
        gtk_level_bar_set_max_value(GTK_LEVEL_BAR(lev),100);
        gtk_level_bar_set_value(GTK_LEVEL_BAR(lev),0);
        gtk_grid_attach(grid,lev,0,4,conf1.pipeLength,1);
        int j,ii=0;
        for (i=0;i<conf1.pipeLength;i++) {
            j = i+1;
                GtkWidget* lab = gui_create_widget("lab",strtools_concat("lab_",strtools_gnum2str(&j,0x04),NULL),strtools_gnum2str(&j,0x04),"cell3","bold","black",NULL);
                gtk_grid_attach(grid,lab,i,1,1,1);
        }
        for (i=0; i<conf1.step_size; i++) {
            j = i+1;
            if (j%2 == 0) {
                GtkWidget* lab = gui_create_widget("lab",strtools_concat("labStep_",strtools_gnum2str(&j,0x04),NULL),strtools_gnum2str(&step[0][i],0x04),"bold","purple",NULL);
                gtk_grid_attach(grid,lab,ii,2,step[1][i],1);
            } else {
                GtkWidget* lab = gui_create_widget("lab",strtools_concat("labStep_",strtools_gnum2str(&j,0x04),NULL),strtools_gnum2str(&step[0][i],0x04),"bold","purpleLight",NULL);
                gtk_grid_attach(grid,lab,ii,2,step[1][i],1);
            }
            ii += step[1][i];
        }
        int l;
        for (i=0; i<conf1.support_size; i++) {
            l = conf1.length2pipe+conf1.pipeLength-support[i];

            if (l >= 0 && l <= conf1.pipeLength) {
                j = i+1;
                GtkWidget* lab = gui_create_widget("lab",strtools_concat("labSupport_",strtools_gnum2str(&j,0x04),NULL),"X","bold","blue",NULL);
                if (l == 0) gtk_grid_attach(grid,lab,l,3,1,1);
                else if (l == conf1.pipeLength) gtk_grid_attach(grid,lab,l,3,1,1);
                else gtk_grid_attach(grid,lab,l,3,1,1);
            }
        }
        gtk_grid_set_column_homogeneous(grid,TRUE);
        GtkWidget* labplot = gui_create_widget("img","imgPlot",NULL,NULL);
        gtk_grid_attach(GTK_GRID(grid),labplot,0,0,conf1.pipeLength,1);
        GtkWidget* labplotvel = gui_create_widget("img","imgPlotVel",NULL,NULL);
        gtk_grid_attach(GTK_GRID(gridVel),labplotvel,0,0,1,1);
        GtkWidget* labploterr = gui_create_widget("img","imgPlotErr",NULL,NULL);
        gtk_grid_attach(GTK_GRID(gridErr),labploterr,0,0,1,1);
        slave_gen_plot();
    } else {
        GtkWidget* labplot = gui_create_widget("lab","labConfigError",CONFIG_NOT_SET,"bold","cell2","red",NULL);
        gtk_grid_attach(GTK_GRID(grid),labplot,0,0,conf1.pipeLength,1);
    }
    gtk_widget_show_all(gui_get_widget("mainWindow"));
    return FALSE;
}


int slave_gen_plot() {
    GtkWidget* gridPar = gui_get_widget("gridVisu");
    GtkWidget* grid = gui_local_get_widget(gridPar,"gridHelix");
    gtk_widget_set_halign(grid,GTK_ALIGN_FILL);
    gtk_widget_set_hexpand(grid,FALSE);
    GtkWidget* gridVel = gui_local_get_widget(gridPar,"gridVel");
    gtk_widget_set_halign(gridVel,GTK_ALIGN_FILL);
    gtk_widget_set_hexpand(gridVel,TRUE);
    INTEGER32 wgrid = gtk_widget_get_allocated_width(grid);
    INTEGER32 wgridVel = gtk_widget_get_allocated_width(gridVel);
    char* str2build = "";
    str2build = strtools_concat(str2build, "\nset style line 1 lc rgb '#5e9c36' pt 6 ps 1 lt 1 lw 2",NULL);
    str2build = strtools_concat(str2build, "\nset style line 2 lc rgb '#8b1a0e' pt 1 ps 1 lt 1 lw 2",NULL);
    str2build = strtools_concat(str2build, "\nset style line 3 lc rgb '#8B0E6B' pt 1 ps 1 lt 1 lw 2",NULL);
    str2build = strtools_concat(str2build, "\nset style line 4 lc rgb '#619FD2' pt 1 ps 1 lt 1 lw 2",NULL);
    str2build = strtools_concat(str2build, "\nset style line 5 lc rgb '#DFB53E' pt 1 ps 1 lt 1 lw 2",NULL);
    str2build = strtools_concat(str2build, "\nset style line 11 lc rgb '#808080' lt 1",NULL);
    str2build = strtools_concat(str2build, "\nset style line 12 lc rgb '#808080' lt 0 lw 1",NULL);
    str2build = strtools_concat(str2build, "\nset border 3 back ls 11",NULL);
    str2build = strtools_concat(str2build, "\nset rmargin 0",NULL);
    char* str2build2 = str2build;
    char* str2build3 = str2build;
    str2build = strtools_concat(str2build, "\nset key outside left horizontal top textcolor rgb \"white\"",NULL);
    str2build = strtools_concat("\nset output 'temp.png'",str2build,NULL);
    str2build = strtools_concat("set terminal pngcairo size ",strtools_gnum2str(&wgrid,0x04),",150 enhanced font 'Verdana,8' background rgb 'black'",str2build,NULL);
    str2build = strtools_concat(str2build, "\nset xrange [0:30]",NULL);
    str2build = strtools_concat(str2build, "\nset yrange [-1:1]",NULL);
    str2build = strtools_concat(str2build, "\nset sample 10000",NULL);
    str2build = strtools_concat(str2build, "\nset lmargin 0",NULL);

    str2build2 = strtools_concat("\nset output 'temp_vel.png'",str2build2,NULL);
    str2build2 = strtools_concat("set terminal pngcairo size ",strtools_gnum2str(&wgridVel,0x04),",150 enhanced font 'Verdana,8' background rgb 'black'",str2build2,NULL);
    str2build2 = strtools_concat(str2build2, "\nset lmargin 4",NULL);
    str2build2 = strtools_concat(str2build2, "\nset yrange [0:6]",NULL);
    str2build2 = strtools_concat(str2build2, "\nset key inside right horizontal top textcolor rgb \"white\"",NULL);

    str2build3 = strtools_concat("\nset output 'temp_err.png'",str2build3,NULL);
    str2build3 = strtools_concat("set terminal pngcairo size ",strtools_gnum2str(&wgridVel,0x04),",150 enhanced font 'Verdana,8' background rgb 'black'",str2build3,NULL);
    str2build3 = strtools_concat(str2build3, "\nset lmargin 7",NULL);
    str2build3 = strtools_concat(str2build3, "\nset yrange [-25000:150000]",NULL);
    str2build3 = strtools_concat(str2build3, "\nset grid",NULL);
    str2build3 = strtools_concat(str2build3, "\nset key inside right horizontal top textcolor rgb \"white\"",NULL);
    int l,y1=0,y2=0;
    for (l=0; l<conf1.step_size;l++) {
        if (l == 0) {
            y1=0; y2=step[1][l];
        }else {
            y1 = y2;
            y2=y2+step[1][l];
        }
        str2build = strtools_concat(str2build, "\ng",strtools_gnum2str(&l,0x02),"(x)=((x>=",strtools_gnum2str(&y1,0x04)," && x<=",strtools_gnum2str(&y2,0x04),") ? 1 : 1/0)",NULL);
    }
    y1 = 0;
    int coord[20] = {0};
    char* k;
    for (l=0; l<conf1.step_size;l++) {
        if (l == 0) {
            str2build = strtools_concat(str2build,"\nplot ",NULL);
            k = "0";
            if (step[0][l] == 0) {
                str2build = strtools_concat(str2build, "sin(0)*g",strtools_gnum2str(&l,0x02),"(x) with lines linestyle 1",NULL);
            } else {
                str2build = strtools_concat(str2build, "sin(2*pi/",strtools_gnum2str(&step[0][l],0x02),
                "*x",")*g",strtools_gnum2str(&l,0x02),"(x) with lines linestyle 1",NULL);
            }
        } else {
            if (step[0][l] == 0) {
                str2build = strtools_concat(str2build, "sin(2*pi/",strtools_gnum2str(&step[0][l-1],0x02),
                "*",strtools_gnum2str(&y1,0x02),"+",k,")*g",strtools_gnum2str(&l,0x02),"(x) with lines linestyle 1",NULL);
            } else {
                if (step[0][l-1] == 0) {
                    if (l==1)
                        k = strtools_concat(k,"-2*pi/",strtools_gnum2str(&step[0][l],0x02),
                        "*",strtools_gnum2str(&y1,0x02),NULL);
                    else
                        k = strtools_concat(k,"+2*pi/",strtools_gnum2str(&step[0][l-2],0x02),
                        "*",strtools_gnum2str(&coord[l-2],0x02),"-2*pi/",strtools_gnum2str(&step[0][l],0x02),
                        "*",strtools_gnum2str(&y1,0x02),NULL);
                } else {
                    k = strtools_concat(k,"+2*pi/",strtools_gnum2str(&step[0][l-1],0x02),
                    "*",strtools_gnum2str(&y1,0x02),"-2*pi/",strtools_gnum2str(&step[0][l],0x02),
                    "*",strtools_gnum2str(&y1,0x02),NULL);
                }
                str2build = strtools_concat(str2build, "sin(2*pi/",strtools_gnum2str(&step[0][l],0x02),
                "*x+",k,")*g",strtools_gnum2str(&l,0x02),"(x) with lines linestyle 1",NULL);
            }
        }
        str2build = strtools_concat(str2build, " title \"Courbe théorique\"",NULL);

        y1 += step[1][l];
        coord[l] = y1;
        if (l != conf1.step_size-1) str2build = strtools_concat(str2build,",",NULL);
    }
    if (set_up) {
        str2build = strtools_concat(str2build, ", \"",FILE_HELIX_RECORDED,"\" using 1:2 title \"Courbe réelle\" with lines linestyle 2",NULL);
        double temps = time_actual_sync- time_start + 15;
        if (temps < 30) {
            str2build2 = strtools_concat(str2build2, "\nset xrange [0:30]",NULL);
            str2build3 = strtools_concat(str2build3, "\nset xrange [0:30]",NULL);
        } else {
            INTEGER32 deb = (int)temps - (int)30;
            INTEGER32 fin = (int)(temps);
            str2build2 = strtools_concat(str2build2, "\nset xrange [",strtools_gnum2str(&deb,0x04),":",strtools_gnum2str(&fin,0x04),"]",NULL);
            str2build3 = strtools_concat(str2build3, "\nset xrange [",strtools_gnum2str(&deb,0x04),":",strtools_gnum2str(&fin,0x04),"]",NULL);
        }
        str2build2 = strtools_concat(str2build2,"\nplot \"",FILE_VELOCITY_LASER,"\" using 1:2 title \"Vit. Laser Mean\" with lines linestyle 2",NULL);
        str2build2 = strtools_concat(str2build2,", \"",FILE_VELOCITY_LASER,"\" using 1:3 title \"Vit. Laser Inst.\" with lines linestyle 3",NULL);
        str2build2 = strtools_concat(str2build2,", \"",FILE_VELOCITY_SYNC,"\" using 1:2 title \"Vit. Translation\" with lines linestyle 4",NULL);
        str2build2 = strtools_concat(str2build2,", \"",FILE_VELOCITY_SYNC,"\" using 1:3 title \"Vit. Rotation\" with lines linestyle 5",NULL);
        str2build3 = strtools_concat(str2build3,"\nplot \"",FILE_ERROR,"\" using 1:2 title \"Erreur\" with lines linestyle 2",NULL);
        str2build3 = strtools_concat(str2build3,",\"",FILE_ERROR,"\" ","using 1:3 title \"Erreur moyenne\" with lines linestyle 1",NULL);
        if(!strtools_build_file("script_vel.gnu",str2build2)) {
            if (str2build2 != "") free(str2build2);
            return 0;
        }
        pid_t pidVel = fork();
        if (pidVel < 0) {
            printf("A fork error has occurred.\n");
            exit(-1);
        } else {
            if (pidVel == 0) {
                execlp("gnuplot","gnuplot", "script_vel.gnu",NULL);
            } else {
                wait(0);
            }
        }
        GtkWidget* lab2 = gui_local_get_widget (gridPar,"imgPlotVel");
        // Verifier si une image existe
        gtk_image_clear(GTK_IMAGE(lab2));
        gtk_image_set_from_file(GTK_IMAGE(lab2),"temp_vel.png");
        if(!strtools_build_file("script_err.gnu",str2build3)) {
            if (str2build3 != "") free(str2build3);
            return 0;
        }
        pid_t pidErr = fork();
        if (pidErr < 0) {
            printf("A fork error has occurred.\n");
            exit(-1);
        } else {
            if (pidErr == 0) {
                execlp("gnuplot","gnuplot", "script_err.gnu",NULL);
            } else {
                wait(0);
            }
        }
        GtkWidget* lab3 = gui_local_get_widget (gridPar,"imgPlotErr");
        // Verifier si une image existe
        gtk_image_clear(GTK_IMAGE(lab3));
        gtk_image_set_from_file(GTK_IMAGE(lab3),"temp_err.png");

    }
    if(!strtools_build_file("script.gnu",str2build)) {
        if (str2build != "") free(str2build);
        return 0;
    }
    pid_t pid = fork();
    if (pid < 0) {
        printf("A fork error has occurred.\n");
        exit(-1);
    } else {
        if (pid == 0) {
            execlp("gnuplot","gnuplot", "script.gnu",NULL);
        } else {
            wait(0);
        }
    }
    GtkWidget* lab = gui_local_get_widget (gridPar,"imgPlot");
    gtk_image_clear(GTK_IMAGE(lab));
    gtk_image_set_from_file(GTK_IMAGE(lab),"temp.png");
    return 1;
}

int slave_get_step_with_length(double lrec) {
    int i,lm=0,lM=0;
    if (lrec <= min_length) return step[0][0];
    for (i=0;i<conf1.step_size;i++) {
        lM += step[1][i];
        if (lrec>(double)lm && lrec <= (double)lM)
            return step[0][i];
        lm=lM;
    }
    return 10000000;
}

INTEGER32 slave_get_position_with_length (double l) {
    int i, l_cumul=0;
    INTEGER32 pos=0;
    if (l <= 0.01) l=0;
    for (i=0;i<conf1.step_size;i++) {
        if (l_cumul + step[1][i] < l) {
            l_cumul += step[1][i];
            pos += 51200*500*step[1][i]/step[0][i];
        } else {
            pos += 51200*500*(l-l_cumul)/step[0][i];
        }
    }
    return pos;
}

int slave_load_const() {
    FILE* asserv_fn = fopen(FILE_ASSERV_CONFIG,"r");
    if (asserv_fn == NULL) return 0;
    char title[20];
    double datTransType,datRotDir,datTimeSync,datTimeCor,datMaxError,datVmax,datV;
    char chaine[1024] = "";
    int i = 0;
    while(fgets(chaine,1024,asserv_fn) != NULL) {
        if (sscanf(chaine,"%19s %lf",title,&datTransType) == 2 && strcmp(title,"TransType") == 0) {
            trans_type=(int)datTransType;
            i++;
        }
        if (sscanf(chaine,"%19s %lf",title,&datRotDir) == 2 && strcmp(title,"RotDirection") == 0) {
            rot_direction = (int)datRotDir;
            i++;
        }
        if (sscanf(chaine,"%19s %lf",title,&datV) == 2 && strcmp(title,"VitTrans") == 0) {
            trans_vel = (INTEGER32)datV/60*STEP_PER_REV*TRANS_REDUCTION/WHEEL_PERIMETER;
            i++;
        }
        if (sscanf(chaine,"%19s %lf",title,&datVmax) == 2 && strcmp(title,"VitRotMax") == 0) {
            vitesse_max = (INTEGER32)datV;
            i++;
        }
        if (sscanf(chaine,"%19s %lf",title,&datTimeSync) == 2 && strcmp(title,"PeriodSync") == 0) {
            tsync = (UNS32)datTimeSync*1000;
            i++;
        }
        if (sscanf(chaine,"%19s %lf",title,&datTimeCor) == 2 && strcmp(title,"PeriodCor") == 0) {
            tcalc = datTimeCor/1000;
            i++;
        }
        if (sscanf(chaine,"%19s %lf",title,&datMaxError) == 2 && strcmp(title,"ErrorMax") == 0) {
            max_error = (int)datMaxError;
            i++;
        }
    }
    fclose(asserv_fn);
    if(i>=6) return 1;
    else return 0;
}

int slave_check_asserv_config() {
    if(!slave_load_const()) {
        char* str2build = "TransType 1 \nRotDirection 0 \nVitRotMax 300000 \nPeriodSync 100 \nPeriodCor 500 \nErrorMax 2 \n";
        if(!strtools_build_file(FILE_ASSERV_CONFIG,str2build)) return 0;
    }
    return 1;
}
