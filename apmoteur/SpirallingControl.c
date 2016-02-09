#include "SpirallingControl.h"
#include <stdio.h>
#include <string.h>
#include <unistd.h>
#include <stdlib.h>
#include <signal.h>
#include <gtk/gtk.h>
#include <glib.h>
#include "canfestival.h"
#include "keyword.h"
#include "serialtools.h"
#include "master.h"
#include "slave.h"
#include "cantools.h"
#include "gtksig.h"
#include "gui.h"
#include "lsstools.h"
#include "od_callback.h"
#include "strtools.h"
#include "errgen.h"
#include "motor.h"
#include "profile.h"
#include <time.h>

s_BOARD MasterBoard = {"0","1M"};

/** RECUPERATION DES VARIABLES GLOBALES **/
    // Récupération des variables numériques à traiter
void* power[SLAVE_NUMBER_LIMIT]= {&StatusWord_1,&StatusWord_2,&StatusWord_3,&StatusWord_4,&StatusWord_5};
void* power_error[SLAVE_NUMBER_LIMIT]= {&ErrorCode_1,&ErrorCode_2,&ErrorCode_3,&ErrorCode_4,&ErrorCode_5};
void* temperature[SLAVE_NUMBER_LIMIT]= {&Temperature_1,&Temperature_2,&Temperature_3,&Temperature_4,&Temperature_5};
void* voltage[SLAVE_NUMBER_LIMIT] = {&Voltage_1,&Voltage_2,&Voltage_3,&Voltage_4,&Voltage_5};
void* velocity[SLAVE_NUMBER_LIMIT] = {&VelocityR_1,&VelocityR_2,&VelocityR_3,&VelocityR_4,&VelocityR_5};
void* vel2send[SLAVE_NUMBER_LIMIT] = {&VelocityS_1,&VelocityS_2,&VelocityS_3,&VelocityS_4,&VelocityS_5};
void* position[SLAVE_NUMBER_LIMIT] = {&PositionR_1,&PositionR_2,&PositionR_3,&PositionR_4,&PositionR_5};
INTEGER32 velocity_inc[SLAVE_NUMBER_LIMIT]={0};
INTEGER32 old_voltage [SLAVE_NUMBER_LIMIT]={0};
/** STRUCTURE **/
    // Moteurs
volatile SLAVES_conf slaves[SLAVE_NUMBER_LIMIT];
int SLAVE_NUMBER;
pthread_mutex_t lock_slave = PTHREAD_MUTEX_INITIALIZER; // Mutex de slaves
    // Profiles
volatile PROF profiles[PROFILE_NUMBER] = {
    {0,"TransVit","Translation (vitesse)"},
    {1,"TransCouple","Translation (couple)"},
    {2,"RotVit","Rotation (vitesse)"},
    {3,"RotCouple","Rotation (couple)"},
    {4,"Libre","Libre"}
};
    // Acces rapide aux variables locales
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
    {"Position",0x04,position,DEFAULT},
    {"PowerError",0x06,power_error,POWER_STATE_ERROR},
    {"Temp",0x02,temperature,TEMPERATURE},
    {"Volt",0x04,voltage,VOLTAGE},
    {"Velinc",0x04,(void*)velocity_inc,DEFAULT},
    {"Velocity",0x04,velocity,DEFAULT},
    {"Vel2send",0x04,vel2send,DEFAULT}
};
    // Acces variables distantes
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
    {TORQUE_HMT_CONTROL,"TorqueHmtControl",0x2702,0x00,0x05,0,0xFF,ERR_SET_HMT_CONTROL,ERR_READ_HMT_ACTIVATE,ERR_SAVE_HMT_ACTIVATE},
    {VELOCITY,"Velocity",0x60FF,0x00,0x04,0,1000000,ERR_SET_VELOCITY,ERR_READ_VELOCITY,ERR_SAVE_VELOCITY},
    {TARGET_POSITION,"TargetPosition",0x607A,0x00,0x04,0,26000000,ERR_SET_POSITION,ERR_READ_POSITION,ERR_SAVE_POSITION},
    {CONTROL_WORD,"ControlWord",0x6040,0x00,0x06,0,0XFFFF,ERR_SET_CONTROL,ERR_READ_CONTROL,ERR_SAVE_CONTROL},
    {MAX_VELOCITY,"VelocityMax",0x6081,0x00,0x07,0,512000,ERR_SET_VELOCITY_MAX,ERR_READ_VELOCITY_MAX,ERR_SAVE_VELOCITY_MAX},
    {POSITION,"Position",0x6064,0x00,0x04,0,999999999,0,ERR_READ_POSITION,0}
};

    // Liste correspondance variables locales et distantes
volatile LOCVAR local[LOCVAR_NUMBER] = {
    {0x60410010,0x20000010}, //StatusWord
    {0x603F0010,0x20050010}, //Error
    {0x20150110,0x200A0010}, //Voltage
    {0x20180108,0x200F0008}, //Temp
    {0x606C0020,0x20140020}, //Vel R
    {0x60FF0020,0x20190020}, //Vel S
    {0x60640020,0x201E0020} //Profile
};

/** PARAMETRES GENERAUX **/
int run_init = 1;
int run_gui_loop = 0;
int run_laser = 0;
GThread * lthread;
GMutex lock_gui_box;
UNS16 errgen_state = 0x0000;
char* errgen_aux = NULL;
    // Menu courant
int current_menu = 0;
    // Pose is running
int set_up = 0;
    // Variable de temps
volatile struct timespec tstart={0,0}, tend={0,0};
    // Incrémentation du moteur rotvit
INTEGER32 rot_pos;
    // Mesure laser
double length_start,actual_length;


/** FONCTIONS **/
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
        run_init = 0;
        run_laser = 0;
		serialtools_exit_laser();
        masterSendNMTstateChange (&SpirallingMaster_Data, 0, NMT_Stop_Node);
        if (getState(&SpirallingMaster_Data) != Unknown_state &&
        getState(&SpirallingMaster_Data) != Stopped)
            setState(&SpirallingMaster_Data, Stopped);
        if (run_gui_loop) {
            gtk_main_quit();
        }
        exit(EXIT_FAILURE);
    }
}

int main(int argc,char **argv) {
// Initialisation de l'interface graphique
    gui_init();
    gchar* thgui = "gui";
    GError * guierr;
    GThread * guithread = g_thread_try_new (thgui, gui_main,NULL, &guierr);
    if (guithread == NULL) {
        errgen_state = ERR_GUI_LOOP_RUN;
        g_idle_add(errgen_set_safe(NULL),NULL);
    }

    // Configuration du socket
    pid_t pid = fork();

    if (pid < 0) {
        printf("A fork error has occurred.\n");
        exit(-1);
    } else {
        if (pid == 0) {
            execlp("./load_can.sh","load_can.sh",NULL);
            errgen_state = ERR_DRIVER_UP;
            g_idle_add(errgen_set_safe(NULL),NULL);
        } else {
            wait(0);
        }
    }

// Handler pour arret manuel
	signal(SIGTERM, catch_signal);
	signal(SIGINT, catch_signal);

// Initialisation des lasers
    gchar* thlaser = "laser_init";
    GError * lasererr;
    lthread = g_thread_try_new (thlaser, serialtools_init,NULL, &lasererr);
    if (lthread == NULL) {
        errgen_state = ERR_LASER_LOOP_RUN;
        g_idle_add(errgen_set_safe(NULL),NULL);
    }

// Chemin vers la librairie CANFESTIVAL
    char* LibraryPath="../drivers/can_socket/libcanfestival_can_socket.so";
// Chargement de la libraire
	if (LoadCanDriver(LibraryPath) == NULL) {
        errgen_state = ERR_DRIVER_LOAD;
        g_idle_add(errgen_set_safe(NULL),NULL);
    }

// Ouverture du port CAN
    if(!canOpen(&MasterBoard,&SpirallingMaster_Data)) {
        errgen_state = ERR_DRIVER_OPEN;
        g_idle_add(errgen_set_safe(NULL),NULL);
    }


// Definition des esclaves
    if (!slave_read_definition()) {
        run_init = -1;
        errgen_state = ERR_FILE_SLAVE_DEF;
        errgen_aux = FILE_SLAVE_CONFIG;
        g_idle_add(errgen_set_safe(NULL),NULL);
    }
// Control des fichiers de configuration
    int i,res;
    for (i=0; i < PROFILE_NUMBER; i++) {
        if (!slave_check_profile_file(i)) {
            errgen_state = ERR_FILE_PROFILE;
            errgen_aux = profile_get_filename_with_index(i);
            g_idle_add(errgen_set_safe(NULL),NULL);
        }
    }

// Initialisation des fichiers vitesse
    for (i=0; i<SLAVE_NUMBER;i++) {
        FILE* f = fopen(strtools_concat("dat/",slave_get_param_in_char("SlaveTitle",i),"_",profile_get_id_with_index(slave_get_param_in_num("SlaveProfile",i)),".dat",NULL),"w");
        char temps[20];
        time_t now = time(NULL);
        strftime(temps, 20, "%d-%m-%Y %H:%M:%S", localtime(&now));
        if (f != NULL) {
            fputs(strtools_concat(profile_get_id_with_index(slave_get_param_in_num("SlaveProfile",i))," -> ",slave_get_param_in_char("SlaveTitle",i),"\nDate : ",temps,"\n","Temps   ","Vitesse\n",NULL),f);
            fclose(f);
        }
    }

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
    if (initthread == NULL) {
        errgen_state = ERR_INIT_LOOP_RUN;
        g_idle_add(errgen_set_safe(NULL),NULL);
    }
    g_thread_join (initthread);
    g_thread_join (guithread);

    canClose(&SpirallingMaster_Data);
    TimerCleanup();
	return 0;
}
