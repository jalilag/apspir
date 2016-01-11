#include "slave.h"
#include "SpirallingControl.h"
#include "cantools.h"
#include "keyword.h"
#include "strtools.h"
#include <stdlib.h>
#include "errgen.h"
#include "gui.h"
#include "motor.h"
#include <glib.h>
#include "master.h"
#include "gtksig.h"
extern pthread_mutex_t lock_slave;
extern GMutex lock_gui, lock_err;
<<<<<<< HEAD
extern int PROFILE_NUMBER, run_init;

extern INTEGER32 velocity_inc[SLAVE_NUMBER_LIMIT];

extern PROF slave_profile[PROFILE_NUMBER_LIMIT];
extern PARAM pardata[PARAM_NUMBER];
extern PARVAR vardata[VAR_NUMBER];

int slave_get_LSS_data(CO_Data * d){
    int i, count=0;
    //masterSendNMTstateChange(d, 0x00, NMT_Stop_Node);
    //masterRequestNodeState(d, 0x00);//broadcast
    //sleep(1);
    //detecter le nombre d'esclaves:
    masterRequestNodeState(d, 0x00);
    masterSendNMTstateChange(d, 0x00, NMT_Reset_Node);
    sleep(1);
    for (i=0; i<NMT_MAX_NODE_ID; i++){
        printf("%d\n",d->NMTable[i]);
        if(d->NMTable[i] != Unknown_state){
            count++;
        }
    }
    SLAVE_NUMBER = count-1;
    printf("SLAVE_NUMBER = %d", SLAVE_NUMBER);
    if(SLAVE_NUMBER<=0){
        //a ajouter: errgen_set()
        return 1;
    }

    count=2;
    for (i=1; i<NMT_MAX_NODE_ID; i++){
        if(d->NMTable[i] != Unknown_state){//le noeud est présent sur le réseau
            //obtenir les données de configuration
            UNS32 Vendor_id;
            UNS32 Product_code;
            UNS32 Revision_number;
            UNS32 Serial_number;

            SDOR sdo_info = {0x1018,0x01,uint32};
            if(!cantools_read_sdo(i,sdo_info,&Vendor_id)){
                errgen_set(ERR_LOAD_ID_DATA, NULL);
                return 1;
            }
            sdo_info = (SDOR){0x1018,0x02,uint32};
            if(!cantools_read_sdo(i,sdo_info,&Product_code)){
                errgen_set(ERR_LOAD_ID_DATA, NULL);
                return 1;
            }
            sdo_info = (SDOR){0x1018,0x03,uint32};
            if(!cantools_read_sdo(i,sdo_info,&Revision_number)){
                errgen_set(ERR_LOAD_ID_DATA, NULL);
                return 1;
            }
            sdo_info = (SDOR){0x1018,0x04,uint32};
            if(!cantools_read_sdo(i,sdo_info,&Serial_number)){
                errgen_set(ERR_LOAD_ID_DATA, NULL);
                return 1;
            }
            pthread_mutex_lock(&lock_slave);
            slaves[count-2] = (SLAVES_conf){ 3, "Translation",count, Vendor_id, Product_code, Revision_number, Serial_number,STATE_LSS_CONFIG,0,0x0000};
            pthread_mutex_unlock(&lock_slave);
            count++;
        }
    }
    masterSendNMTstateChange(d, 0x00, NMT_Stop_Node);
    setState(d, Initialisation);

    return 0;

}

=======
extern int SLAVE_NUMBER, run_init;

extern INTEGER32 velocity_inc[SLAVE_NUMBER_LIMIT];

extern SLAVES_conf slaves[SLAVE_NUMBER_LIMIT];
extern PROF profiles[PROFILE_NUMBER];
extern PARAM pardata[PARAM_NUMBER];
extern PARVAR vardata[VAR_NUMBER];

>>>>>>> c5e0f29c6bce206973f4fc7f2336d0ec6b7ba5e7
/**
* Configuration des esclaves
* 0: Echec; 1 Reussite
**/
int slave_config(UNS8 nodeID) {
    /* CONFIG COM */
    if (!slave_config_com(nodeID)) return 0;
    /* CONFIG FIXE */
    if (!slave_config_with_file(nodeID)) return 0;
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
/**
* Configuration des esclaves à partir du fichier profile correspondant
* 0: Echec; 1 Reussite
**/
int slave_config_with_file(UNS8 nodeID) {
    FILE* motor_fn = fopen(slave_get_profile_filename(slave_get_profile_with_node(nodeID)),"r");
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
            UNS32 *data = *(pdor+i);
            if (data[0] != 0) {
                if(!cantools_PDO_map_config(nodeID,0x1600+i,data[0],data[1],data[2],data[3],data[4],data[5],data[6],data[7],0)) {
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
            INTEGER32 *data = *(pdot+i);
            if (data[0] != 0) {
                if(!cantools_PDO_map_config(nodeID,0x1A00+2+i,data[0],data[1],data[2],data[3],data[4],data[5],data[6],data[7],0)) {
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
        errgen_set(ERR_FILE_OPEN,slave_get_profile_filename(slave_get_profile_with_node(nodeID)));
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
    GtkWidget* lab = gui_create_widget("lab","labStateTitle",STATUT_TITLE,"h1","black",NULL);
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
}
/** Generation de la page param **/
int slave_gui_param_gen(int ind) {
    if (gui_local_get_widget(gui_get_widget("boxParam"),"gridMotor") != NULL)
        gtk_widget_destroy(gui_local_get_widget(gui_get_widget("boxParam"),"gridMotor"));
    if (gui_local_get_widget(gui_get_widget("boxParam"),"gridProfile") != NULL)
        gtk_widget_destroy(gui_local_get_widget(gui_get_widget("boxParam"),"gridProfile"));
    if (ind == 0) {
        // grid
        GtkGrid* grid = gui_local_grid_set("gridMotor",MOTOR_PARAM_TITLE,6,"black");
        gtk_box_pack_start (gui_get_box("boxParam"),GTK_WIDGET(grid),TRUE,TRUE,0);
        gtk_box_reorder_child(gui_get_box("boxParam"),GTK_WIDGET(grid),2);
        // but add
        GtkWidget* lab = gui_create_widget("but","butAddSlave",ADD);
        gtk_button_set_image(GTK_BUTTON(lab),GTK_WIDGET(gtk_image_new_from_icon_name("gtk-add",GTK_ICON_SIZE_BUTTON)));
        gtk_grid_attach(grid,lab,5,1,1,1);
        g_signal_connect (G_OBJECT(lab), "clicked", G_CALLBACK (on_butAddSlave_clicked),NULL);
        // but del
        GtkWidget* lab2 = gui_create_widget("but","butDelSlave",REMOVE);
        gtk_button_set_image(GTK_BUTTON(lab2),GTK_WIDGET(gtk_image_new_from_icon_name("list-remove",GTK_ICON_SIZE_BUTTON)));
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
        }
        int j,k,l;
        for (i=0;i<SLAVE_NUMBER;i++) {
            j=i+1;
            gtk_combo_box_text_append(GTK_COMBO_BOX_TEXT(combdel),strtools_gnum2str(&j,0x02),strtools_concat(NODE," : ",strtools_gnum2str(&j,0x02),NULL));
            for (k=0;k<5;k++) {
                GtkWidget* lab = gui_create_widget("ent",strtools_concat("labParamM",strtools_gnum2str(&j,0x02),
                    labtxt[k],NULL),slave_get_param_in_char(labtxt[k],i),NULL);
                gtk_grid_attach(grid,lab,k,i+3,1,1);
            }
            GtkWidget* comb = gui_create_widget("combo",strtools_concat("labParamM",strtools_gnum2str(&j,0x02),"SlaveProfile",NULL),NULL,NULL);
            for (l=0; l<PROFILE_NUMBER;l++) {
                gtk_combo_box_text_append(GTK_COMBO_BOX_TEXT(comb),strtools_gnum2str(&l,0x02),profiles[l].title);
            }
            gtk_grid_attach(grid,comb,5,i+3,1,1);
            gtk_combo_box_set_active (GTK_COMBO_BOX(comb), slave_get_profile_with_index(i));
        }
    } else if (ind == 1){
        GtkGrid* grid = gui_local_grid_set("gridProfile",MOTOR_PARAM_TITLE,4,"black");
        gtk_box_pack_start (gui_get_box("boxParam"),GTK_WIDGET(grid),TRUE,TRUE,0);
        gtk_box_reorder_child(gui_get_box("boxParam"),GTK_WIDGET(grid),2);
        GtkWidget* comb = gui_create_widget("combo","listProfile",NULL,NULL);
        gtk_grid_attach(grid,comb,0,1,1,1);
        int i;
        for (i=0; i<PROFILE_NUMBER; i++) {
            gtk_combo_box_text_append(GTK_COMBO_BOX_TEXT(comb),strtools_gnum2str(&i,0x02),profiles[i].title);
        }
        g_signal_connect (G_OBJECT(comb), "changed", G_CALLBACK (on_listProfile_changed),NULL);
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
        int i = 3,j,k,l;
        char* labtxt[7] = {"SlaveTitle","Vendor","Product","Revision","Serial","SlaveProfile",NULL};
        char* str2build="";
        while(gtk_grid_get_child_at(GTK_GRID(grid),0,i) != NULL) {
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
//        if (PROFILE_NUMBER > 0)
//            for (i=0; i<PROFILE_NUMBER; i++)
//                str2build = strtools_concat(str2build,"##",slave_profile[i].title, "\n",NULL);
//        else
//            str2build = strtools_concat(str2build,"##Translation\n##Rotation\n##Libre",NULL);
        if (!strtools_build_file(FILE_SLAVE_CONFIG,str2build)) {
            if (str2build != "") free(str2build);
            return 0;
        }
        if (str2build != "") free(str2build);
        gui_widget2hide("windowParams",NULL);
        if (run_init == -1) {
            if (!slave_read_definition()) {
                run_init = 0;
                errgen_set(ERR_FILE_PROFILE,slave_get_profile_filename(i));
            }
            run_init = 1;
        } else {
            gui_info_box(RESET_APPLICATION_TITLE,RESET_APPLICATION_CONTENT,NULL);
            Exit(1);
            execlp("./SpirallingControl","SpirallingControl",NULL);
        }
    } else if (index == 1) {
        GtkWidget* grid = gui_local_get_widget(gui_get_widget("boxParam"),"gridProfile");
        char* str2build="";
        int i=3,data,j;
        while(gtk_grid_get_child_at(GTK_GRID(grid),0,i) != NULL) {
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
            i++;
        }
        int ind = gtk_combo_box_get_active(GTK_COMBO_BOX(gui_local_get_widget(gui_get_widget("boxParam"),"listProfile")));
        if(!strtools_build_file(slave_get_profile_filename(ind),str2build)) {
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
    }
}
/**
* Check des fichiers profils
**/
int slave_check_profile_file(int profId) {
    int i,j=0,res;
    FILE *f = fopen(slave_get_profile_filename(profId),"r");
    if (f != NULL) {
        char title[20];
        int data;
        char chaine[1024]="";
        while(fgets(chaine,1024,f) != NULL) {
            if (sscanf(chaine,"%19s ; %d",title,&data) == 2) j++;
                res = 0;
                for (i=0; i<PARAM_NUMBER; i++)
                    if (strcmp(pardata[i].gui_code,title) == 0 ) res = 1;
                if (res == 0) {
                    j = 0;
                    errgen_set(ERR_FILE_PROFILE_INVALID_PARAM,slave_get_profile_filename(profId));
                }
        }
        fclose(f);
        if (j > 0) return 1;
        else {
            strtools_build_file(slave_get_profile_filename(profId)," ");
            errgen_set(ERR_FILE_EMPTY,slave_get_profile_filename(profId));
        }
    } else {
        errgen_set(ERR_FILE_OPEN,slave_get_profile_filename(profId));
        if (!strtools_build_file(slave_get_profile_filename(profId)," ")) {
            errgen_set(ERR_FILE_GEN,slave_get_profile_filename(profId));
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
        printf("Index trop grand : %d \n",i); exit(EXIT_FAILURE);
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
/**
* Retourne state en fonction index
**/
int slave_get_state_with_index(int i) {
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
/**
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
        exit(EXIT_FAILURE);
    }
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
/**
* Renvoi le nom du fichier correspondant au profil
**/
char* slave_get_profile_filename(int index) {
    return g_strconcat("profile_",g_utf8_strdown(profiles[index].id,-1),"_config.txt",NULL);
}
char* slave_get_profile_name(int index) {
    return profiles[index].title;
}
/**
* Retourne en INTEGER32 le paramètre
**/
INTEGER32 slave_get_param_in_num(char* parid, int index) {
    int i;
    for (i=0;i<VAR_NUMBER;i++) {
        if (parid == vardata[i].id) {
            if (!strcmp(parid, "Volt")) {
                UNS16 *data = *(vardata[i].tab+index);
                INTEGER32 data2= (INTEGER32)*data;
                data2 = data2/10;
                return data2;
            } else if (!strcmp(parid, "Velinc")) {
                INTEGER32* data = (INTEGER32*)vardata[i].tab;
                return (INTEGER32)data[index];
            } else if (!strcmp(parid, "Vel2sendPDONum")){
                return index;
            } else if (!strcmp(parid, "State")) {
                return (INTEGER32)slave_get_state_with_index(index);
            } else if (!strcmp(parid,"StateError")) {
                return (INTEGER32)slave_get_state_with_index(index);
            } else if (!strcmp(parid, "SlaveProfile")) {
                return (INTEGER32)slave_get_profile_with_index(index);
            } else if (!strcmp(parid, "Vendor")) {
                return (INTEGER32)slave_get_vendor_with_index(index);
            } else if (!strcmp(parid, "Product")) {
                return (INTEGER32)slave_get_product_with_index(index);
            } else if (!strcmp(parid, "Revision")) {
                return (INTEGER32)slave_get_revision_with_index(index);
            } else if (!strcmp(parid, "Serial")) {
                return (INTEGER32)slave_get_serial_with_index(index);
            } else if (!strcmp(parid, "Active")) {
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
                    INTEGER32 *data = *(vardata[i].tab+index);
                    *data = (INTEGER32)dat;
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

/**retourne la liste des index de profile Profile correspondent**/
//retourne 0 si pas d'erreur
//retourne 1 si le nombre d'index vérifiant le truc est > à sizeof(indexlist) ou si aucun index à été trouvé
int slave_get_indexList_from_ProfileName(char* ProfileName, int* indexList){
    int i;
    for (i=0; i<sizeof(indexList)/sizeof(int); i++){
        indexList[i] = -1;
    }
    int maxcount = sizeof(indexList)/sizeof(int);
    int count=0;
    for (i=0;i<SLAVE_NUMBER;i++){
        if(!strcmp(slave_get_profile_name(i),Profile)){
            if(count < maxcount) {
                indexList[count] = i;
                count++;
            }
            else return 1;//indexList too small to contain all data
        }
    }
    if (count == 0) return 1;// aucun moteur défini
    return 0;
}
