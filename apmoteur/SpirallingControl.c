#include <stdio.h>
#include <string.h>
#include <unistd.h>
#include <stdlib.h>
#include <signal.h>
#include <gtk/gtk.h>
#include <glib.h>
#include "canfestival.h"
#include "keyword.h"

#include "master.h"
#include "slave.h"
#include "SpirallingControl.h"
#include "cantools.h"
#include "gtksig.h"
#include "gui.h"
#include "lsstools.h"
#include "od_callback.h"
#include "strtools.h"
#include "errgen.h"
#include "motor.h"

#include "laser_asserv.h"
#include "serialtools.h"

//lasers
laser ml, sl, lsim;
//definition de l'hélice
struct Helix_User_Data HelixUserData;

// Plateforme MASTER
s_BOARD MasterBoard = {"0","500k"};

// Définition des esclaves

volatile SLAVES_conf slaves[SLAVE_NUMBER_LIMIT];

volatile PROF profiles[PROFILE_NUMBER] = {
    {0,"TransVit","Translation (vitesse)"},
    {1,"TransCouple","Translation (couple)"},
    {2,"RotVit","Rotation (vitesse)"},
    {3,"RotCouple","Rotation (couple)"},
    {4,"Libre","Libre"}
};
int SLAVE_NUMBER;
pthread_mutex_t lock_slave = PTHREAD_MUTEX_INITIALIZER; // Mutex de slaves
GMutex lock_gui_box;
// Les paramètres
INTEGER32 old_voltage [SLAVE_NUMBER_LIMIT]={0};
// Récupération des variables numériques à traiter
void* power[SLAVE_NUMBER_LIMIT]= {&StatusWord_1,&StatusWord_2,&StatusWord_3,&StatusWord_4};
void* power_error[SLAVE_NUMBER_LIMIT]= {&ErrorCode_1,&ErrorCode_2,&ErrorCode_3,&ErrorCode_4};
void* temperature[SLAVE_NUMBER_LIMIT]= {&InternalTemp_1,&InternalTemp_2,&InternalTemp_3,&InternalTemp_4};
void* voltage[SLAVE_NUMBER_LIMIT] = {&Voltage_1,&Voltage_2,&Voltage_3,&Voltage_4};
void* velocity[SLAVE_NUMBER_LIMIT] = {&Velocity_1,&Velocity_2,&Velocity_3,&Velocity_4};
void* vel2send[SLAVE_NUMBER_LIMIT] = {&Vel2Send_1,&Vel2Send_2,&Vel2Send_3,&Vel2Send_4};
void* position[SLAVE_NUMBER_LIMIT] = {&Position_1, &Position_2, &Position_3, &Position_4};
void* accel[SLAVE_NUMBER_LIMIT] = {&Acceleration_1, &Acceleration_2, &Acceleration_3, &Acceleration_4};
void* decel[SLAVE_NUMBER_LIMIT] = {&Deceleration_1, &Deceleration_2, &Deceleration_3, &Deceleration_4};
void* accel2send[SLAVE_NUMBER_LIMIT] = {&Accel2send_1, &Accel2send_2, &Accel2send_3, &Accel2send_4};
void* decel2send[SLAVE_NUMBER_LIMIT] = {&Decel2send_1, &Decel2send_2, &Decel2send_3, &Decel2send_4};
INTEGER32 velocity_inc[SLAVE_NUMBER_LIMIT]={0};

// Structure d'acces aux variables defini dans le maitre
volatile PARVAR vardata[VAR_NUMBER] = {
    {"Active",0x00,NULL,ACTIVE},
    {"SlaveTitle",0x00,NULL,NODE},
    {"StateImg",0x00,NULL,NULL},
    {"Vendor",0x07,NULL,VENDOR},
    {"Product",0x07,NULL,PRODUCT},
    {"Revision",0x07,NULL,REVISION},
    {"Serial",0x07,NULL,SERIAL},
    {"SlaveProfile",0x02,NULL,NODE_PROFILE},
    {"State",0x02,NULL,STATUT_TITLE},
    {"StateError",0x02,NULL,STATE_ERROR},
    {"Power",0x06,power,POWER_STATE},
    {"PowerError",0x06,power_error,POWER_STATE_ERROR},
    {"Temp",0x02,temperature,TEMPERATURE},
    {"Volt",0x04,voltage,VOLTAGE},
    {"Velinc",0x04,(void*)velocity_inc,DEFAULT},
    {"Velocity",0x04,velocity,DEFAULT},
    {"Vel2send",0x04,vel2send,DEFAULT},
    {"Vel2sendPDONum",uint8,NULL,DEFAULT},
    {"Position", int32, position, DEFAULT},
    {"Acceleration", uint32, accel, DEFAULT},
    {"Deceleration", uint32, decel, DEFAULT},
    {"Accel2send", uint32, accel2send, DEFAULT},
    {"Decel2send", uint32, decel2send, DEFAULT}
};
// Boucle d'initialisation
int run_init = 1;
int run_gui_loop = 0;
// Liste des paramètres CANOPEN modifiables
volatile PARAM pardata[PARAM_NUMBER] = {
    {PDOR1,"Pdor1",0x1600,0x00,0x07,0,0x70000000,ERR_SET_PDO,0,0},
    {PDOR2,"Pdor2",0x1601,0x00,0x07,0,0x70000000,ERR_SET_PDO,0,0},
    {PDOR3,"Pdor3",0x1602,0x00,0x07,0,0x70000000,ERR_SET_PDO,0,0},
    {PDOR4,"Pdor4",0x1603,0x00,0x07,0,0x70000000,ERR_SET_PDO,0,0},
    {PDOT3,"Pdot3",0x1A02,0x00,0x07,0,0x70000000,ERR_SET_PDO,0,0},
    {PDOT4,"Pdot4",0x1A03,0x00,0x07,0,0x70000000,ERR_SET_PDO,0,0},
    {CURRENT,"Current",0x2204,0x00,0x05,0,100,ERR_SET_CURRENT,ERR_READ_CURRENT,ERR_SAVE_CURRENT},
    {PROFILE_MODE,"Profile",0x6060,0x00,0x02,0,10,ERR_SET_PROFILE,ERR_READ_PROFILE,ERR_SAVE_PROFILE},
    {ACCELERATION,"Acc",0x6083,0x00,0x07,0,10000000,ERR_SET_ACCELERATION,ERR_READ_ACCELERATION,ERR_SAVE_ACCELERATION},
    {DECELERATION,"Dcc",0x6084,0x00,0x07,0,10000000,ERR_SET_DECELERATION,ERR_READ_DECELERATION,ERR_SAVE_DECELERATION},
    {VELOCITY_INC,"Velinc",0x0000,0x00,0x04,0,500000,ERR_SET_VELOCITY_INC,0x0000,0x0000},
    {DECELERATION_QS,"Dccqs",0x6085,0x00,0x07,0,1000000,ERR_SET_DECELERATION_QS,ERR_READ_DECELERATION_QS,ERR_SAVE_DECELERATION_QS},
    {TORQUE,"Torque",0x6071,0x00,0x03,0,1000,ERR_SET_TORQUE,ERR_READ_TORQUE,ERR_SAVE_TORQUE},
    {TORQUE_RAMP,"TorqueSlope",0x6087,0x00,0x07,0,10000,ERR_SET_TORQUE_SLOPE,ERR_READ_TORQUE_SLOPE,ERR_SAVE_TORQUE_SLOPE},
    {TORQUE_VELOCITY,"TorqueVel",0x2704,0x00,0x05,0,255,ERR_SET_TORQUE_VELOCITY,ERR_READ_TORQUE_VELOCITY,ERR_SAVE_TORQUE_VELOCITY},
    {TORQUE_VELOCITY_MAKEUP,"TorqueVelMake",0x2703,0x00,0x07,0,10000000,ERR_SET_TORQUE_VELOCITY_MAKEUP,ERR_READ_TORQUE_VELOCITY_MAKEUP,ERR_SAVE_TORQUE_VELOCITY_MAKEUP},
    {TORQUE_HMT_ACTIVATE,"TorqueHmtActive",0x2701,0x00,0x05,0,0x80,ERR_SET_HMT_ACTIVATE,ERR_READ_HMT_ACTIVATE,ERR_SAVE_HMT_ACTIVATE},
    {TORQUE_HMT_CONTROL,"TorqueHmtControl",0x2702,0x00,0x05,0,0xFF,ERR_SET_HMT_CONTROL,ERR_READ_HMT_ACTIVATE,ERR_SAVE_HMT_ACTIVATE}
};
// Menu courant
int current_menu = 0;

// A définir mettre a jour
void catch_signal(int sig) {
  signal(SIGTERM, catch_signal);
  signal(SIGINT, catch_signal);
  printf("Got Signal %d\n",sig);
}

/**
* Fermeture de l'application
* 0 : Arret
* 1 : Mise hors tension des moteurs
* 2 : Arret de l'appli
**/
void Exit(int type) {
    int i = 0;
    gtk_switch_set_active(gui_get_switch("butVelStart"),0);
    for (i=0; i<SLAVE_NUMBER; i++) {
        if (slave_get_param_in_num("Active",i)) {
            motor_start(slave_get_node_with_index(i),0);
            if (type > 0)
                motor_switch_off(slave_get_node_with_index(i));
        }
    }
    if (type > 1) {
		unsigned int err_l;
        run_init = 0;
		//exit asserv
		if(run_asserv){
		    printf("EXIT ASSERV\n");
		    if(laser_asserv_stop()){
		        errgen_set(ERR_LASER_ASSERV_STOP, NULL);
		    }
		}

		//fermeture laser
		serialtools_exit_laser();

		//fermeture moteurs
		for (i=0; i<SLAVE_NUMBER; i++) {
		    motor_start(slave_get_node_with_index(i),0);
		    motor_switch_off(slave_get_node_with_index(i));
		}
        masterSendNMTstateChange (&SpirallingMaster_Data, 0, NMT_Stop_Node);
        if (getState(&SpirallingMaster_Data) != Unknown_state &&
        getState(&SpirallingMaster_Data) != Stopped)
            setState(&SpirallingMaster_Data, Stopped);
        if (run_gui_loop) {
            gtk_main_quit();
        }
		canClose(&SpirallingMaster_Data);
	    TimerCleanup();
        exit(EXIT_FAILURE);
    }
}

int main(int argc,char **argv) {
// Initialisation de l'interface graphique
    gui_init();
    gchar* thgui = "gui";
    GError * guierr;
    GThread * guithread = g_thread_try_new (thgui, gui_main,NULL, &guierr);
    if (guithread == NULL)
        errgen_set(ERR_GUI_LOOP_RUN,NULL);
    gtk_level_bar_set_value(GTK_LEVEL_BAR(gui_get_object("gui_level_bar")),0.25);

    // Configuration du socket
    pid_t pid = fork();

    if (pid < 0) {
        printf("A fork error has occurred.\n");
        exit(-1);
    } else {
        if (pid == 0) {
            execlp("./test.sh","test.sh",NULL);
            errgen_set(ERR_DRIVER_UP,NULL); // Driver error
        } else {
            wait(0);
        }
    }
    gtk_level_bar_set_value(GTK_LEVEL_BAR(gui_get_object("gui_level_bar")),0.50);

	//DEFINITION DE LA CONSIGNE HELICE UTILISATEUR

   	//a faire : interface
  unsigned long d[1];
  long int p[1];
  d[0] = 30000;
  p[0] = 4000;

  HelixUserData.dmax = d[0];
  HelixUserData.NbrOfEntries = 1;
  HelixUserData.d = d;
  HelixUserData.p = p;

// Handler pour arret manuel
	signal(SIGTERM, catch_signal);
	signal(SIGINT, catch_signal);

// Chemin vers la librairie CANFESTIVAL
    char* LibraryPath="../drivers/can_socket/libcanfestival_can_socket.so";

// Chargement de la libraire
	if (LoadCanDriver(LibraryPath) == NULL)
        errgen_set(ERR_DRIVER_LOAD,NULL);
    gtk_level_bar_set_value(GTK_LEVEL_BAR(gui_get_object("gui_level_bar")),0.70);

// Ouverture du port CAN
    if(!canOpen(&MasterBoard,&SpirallingMaster_Data))
        errgen_set(ERR_DRIVER_OPEN,NULL);
    gtk_level_bar_set_value(GTK_LEVEL_BAR(gui_get_object("gui_level_bar")),0.80);


// Definition des esclaves
    if (!slave_read_definition()) {
        run_init = -1;
        errgen_set(ERR_FILE_SLAVE_DEF,FILE_SLAVE_CONFIG);
    }
// Control des fichiers de configuration
    int i,res;
    for (i=0; i < PROFILE_NUMBER; i++) {
        if (!slave_check_profile_file(i))
            errgen_set(ERR_FILE_PROFILE,slave_get_profile_filename(i));

    }
    gtk_level_bar_set_value(GTK_LEVEL_BAR(gui_get_object("gui_level_bar")),0.90);

	TimerInit();

    // Callback du maitre
    SpirallingMaster_Data.heartbeatError = SpirallingMaster_heartbeatError;
    SpirallingMaster_Data.initialisation = SpirallingMaster_initialisation;
    SpirallingMaster_Data.preOperational = SpirallingMaster_preOperational;
    SpirallingMaster_Data.operational = SpirallingMaster_operational;
    SpirallingMaster_Data.stopped = SpirallingMaster_stopped;
    SpirallingMaster_Data.post_sync = SpirallingMaster_post_sync;
    SpirallingMaster_Data.post_TPDO = SpirallingMaster_post_TPDO;
    SpirallingMaster_Data.post_emcy = SpirallingMaster_post_emcy;
    SpirallingMaster_Data.post_SlaveBootup = SpirallingMaster_post_SlaveBootup;
    SpirallingMaster_Data.post_SlaveStateChange = SpirallingMaster_post_SlaveStateChange;


    // Initialisation de la boucle d'initialisation
    gchar* thinit = "init_loop";
    GError * initerr;
    GThread * initthread = g_thread_try_new (thinit, cantools_init_loop,NULL, &initerr);
    if (initthread == NULL)
        errgen_set(ERR_INIT_LOOP_RUN,NULL);
    g_thread_join (initthread);
    g_thread_join (guithread);

    canClose(&SpirallingMaster_Data);
    TimerCleanup();
	return 0;
}
