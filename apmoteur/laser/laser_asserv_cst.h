
extern unsigned int User_Tolerance;// = 100 par défault//en dixieme de mm.


extern INTEGER32 Vitesse_de_croisiere;
extern INTEGER32 Vitesse_lente;

//params asserv rotation
//intern
#define LASER_MES_ERROR 60 //en dmm
//Extern
#define LASER_ASSERV_SYNC_TIME 300000
#define LASER_ASSERV_DELTAVITESSEMAX_CORRECTION 250000 //steps/s
#define LASER_ASSERV_MOTROT_POS_INCERTITUDE 19200//19200  1mm erreur laser pour p=4m/tour equivaut à 2.5x10^-4 tour machine soit 1/8 tour moteur(r=500) soit 6400 step
#define LASER_ASSERV_ERR_VITESSE_ROT 30 //ref FindTypeOfMvt: il s'agit de l'erreur en step/s rotation.
#define LASER_ASSERV_MAX_MOTROT_VELOCITY 1000000
#define LASER_ASSERV_MIN_VEL_APPLY_TIME 500000 //en us
#define LASER_ASSERV_MAX_CORR_TIME 2000000

extern int laser_asserv_sync_time;
extern int laser_asserv_deltavitessemax_correction;
extern int laser_asserv_motrot_pos_incertitude;
extern int laser_asserv_err_vitesse_rot;
extern int laser_asserv_max_motrot_velocity;
extern int laser_asserv_min_vel_apply_time;
extern int laser_asserv_max_corr_time;

//params asserv translation
//intern
#define LASER_ASSERV_ERR_VITESSE_TRANS 10 //ref laser_asserv_check_trans_vel: il s'agit de l'erreur veltrans min en dmm/s detectable laser.
#define LASER_ASSERV_ERR_VITESSE_TRANS_INSTANTANNEE 60
//extern
#define LASER_ASSERV_TRSL_SYNC_TIME 500000
#define LASER_ASSERV_TRSL_CORRECTION_DV 100000 //steps/s
#define LASER_ASSERV_TRSL_COUPL_INC 100//pour mil
#define LASER_ASSERV_TRSL_MINDETECT_DV 20000//steps/s
#define LASER_ASSERV_TOL_DIFF_MOTTRANS_INDIC 10000 //stp/s -> en vitesse/vitesse mettre à faible valeur(100) en couple couple ou vitesse/couple-> mettre valeur de l'ordre de laser_to 10000 incertitude positionnemnt en mode coupl;
#define LASER_ACCEL_ERROR 10

extern int laser_asserv_trsl_sync_time;
extern int laser_asserv_trsl_correction_dv;
extern int laser_asserv_trsl_coupl_inc;
extern int laser_asserv_trsl_mindetect_dv;
extern int laser_asserv_tol_diff_mottrans_indic;
extern int laser_accel_error;

//status mvt_check
#define MOVEMENT_OK 0x0100
#define TYPE_OF_MVT_FAILURE 0x0200
#define FIND_CONSIGNE_FAILURE 0x0400
#define ASSERV_FAILURE 0x0800
#define MAX_ROT_VEL_REACHED 0x1000
#define USER_POS_TOL_ERROR 0x2000
#define DPARC_CALC_ERROR 0x4000
#define ASSERV_ERR_PATINAGE 0x8000
#define ERR_DECAL_TEMPS 0x10000

//status Trsl check
#define ERR_VTRANS_TO_VMOT 0x0200
#define WARNING_ACCELERATION_TRSL_NON_RESPECT 0x20000
#define WARNING_TRSL_CORRECTION_ENCOURS 0x40000
#define WARNING_VEL_LASER_MOT_INCONSISTENT 0x80000
#define VELOCITY_SIGN_FAILURE 0x100000

#define LASER_ASSERV_INTERNAL_ERR 0x115E00
//type of mvt
#define VITESSE_CONSTANTE 1
#define PHASE_ACCEL 2
#define PHASE_DECEL 3

//facteurs
#define STEPS_PER_REV 51200
#define REDUCTION_ROTATION 500
#define REDUCTION_TRANS 75
#define REL_TRSL_ROT 817  //roue de diametre tel que pi*D=1000mm (1000mm/min ## (1000=REL_TRSL_ROT)mm/tour *1 tour/min)
#define PIPE_DIAM 3260 //en dmm
extern int pipe_diam;


int laser_asserv_cst_verify(void);
void laser_asserv_cst_set_default(void);
int laser_asserv_cst_save(void);
int laser_asserv_cst_recup(void);
void laser_asserv_cst_Get(void);
void laser_asserv_cst_Set(void);


