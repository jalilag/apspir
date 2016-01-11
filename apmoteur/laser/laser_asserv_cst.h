//params
#define LASER_ASSERV_SYNC_TIME 150000
#define LASER_ASSERV_DELTAVITESSEMAX_CORRECTION 50000 //steps/s
#define LASER_ASSERV_MOTROT_POS_INCERTITUDE 51200//19200  1mm erreur laser pour p=4m/tour equivaut à 2.5x10^-4 tour machine soit 1/8 tour moteur(r=500) soit 6400 step
#define LASER_ASSERV_ERR_VITESSE 30
#define LASER_ASSERV_MAX_MOTROT_VELOCITY 1000000
#define LASER_ASSERV_MIN_VEL_APPLY_TIME 500000 //en us

//status mvt_check
#define MOVEMENT_OK 0x0100
#define TYPE_OF_MVT_FAILURE 0x0200
#define FIND_CONSIGNE_FAILURE 0x0400
#define ASSERV_FAILURE 0x0800
#define MAX_ROT_VEL_REACHED 0x1000
#define USER_POS_TOL_ERROR 0x2000
#define DPARC_CALC_ERROR 0x4000
#define LASER_ASSERV_READ_DAT_ERROR 0x8000

//type of mvt
#define VITESSE_CONSTANTE 1
#define PHASE_ACCEL 2
#define PHASE_DECEL 3
#define QUICKSTOP 4
#define STOPPED 5

//bit of status word motrot
#define TARGET_REACHED 0x0400
//controlword
#define HALT_BIT_CTRL_WD 0x0100

//facteurs
#define STEPS_PER_REV 51200
#define REDUCTION_ROTATION 500
#define REDUCTION_TRANS 75
#define REL_TRSL_ROT 1000  //roue de diametre tel que pi*D=1000mm (1000mm/min ## (1000=REL_TRSL_ROT)mm/tour *1 tour/min)
#define PIPE_DIAM 3260 //en dmm

