#include <stdio.h>
#include <stdlib.h>
#include "canfestival.h"
#include "laser_asserv_cst.h"
#include "gui.h"
#include "strtools.h"

unsigned int User_Tolerance = 100;//en dixieme de mm.
INTEGER32 Vitesse_de_croisiere = 300000;
INTEGER32 Vitesse_lente = 100000;

//constante utilisateur rotation
int laser_asserv_sync_time = LASER_ASSERV_SYNC_TIME;
int laser_asserv_deltavitessemax_correction = LASER_ASSERV_DELTAVITESSEMAX_CORRECTION;
int laser_asserv_motrot_pos_incertitude = LASER_ASSERV_MOTROT_POS_INCERTITUDE;
int laser_asserv_err_vitesse_rot = LASER_ASSERV_ERR_VITESSE_ROT;
int laser_asserv_max_motrot_velocity = LASER_ASSERV_MAX_MOTROT_VELOCITY;
int laser_asserv_min_vel_apply_time = LASER_ASSERV_MIN_VEL_APPLY_TIME;
int laser_asserv_max_corr_time = LASER_ASSERV_MAX_CORR_TIME;
//constante utilisateur translation
int laser_asserv_trsl_mindetect_dv = LASER_ASSERV_TRSL_MINDETECT_DV;
int laser_asserv_trsl_correction_dv = LASER_ASSERV_TRSL_CORRECTION_DV;
int laser_asserv_trsl_sync_time = LASER_ASSERV_TRSL_SYNC_TIME;
int laser_asserv_tol_diff_mottrans_indic = LASER_ASSERV_TOL_DIFF_MOTTRANS_INDIC;
int laser_asserv_trsl_coupl_inc = LASER_ASSERV_TRSL_COUPL_INC;
int laser_accel_error = LASER_ACCEL_ERROR;
//autre constantes
int pipe_diam = PIPE_DIAM;

//fonctions Verify
int User_Tolerance_Verify(unsigned int ut){
    if ( 1 <= ut && ut <=100 ) return 1;
    else return 0;
}

int laser_asserv_sync_time_Verify(int st){
    if ( 150000 <= st ) return 1;
    else return 0;
}

int laser_asserv_motrot_pos_incertitude_Verify(int mpi){
    if ( 10000 <= mpi ) return 1;
    else return 0;
}

 int laser_asserv_err_vitesse_rot_Verify(int evr){
    if( 0 <= evr ) return 1;
    else return 0;
 }

 int laser_asserv_max_motrot_velocity_Verify(int mrv){
    if( 100000 <= mrv ) return 1;
    else return 0;
 }

 int laser_asserv_min_vel_apply_time_Verify(int mvap){
    if( 1 <= mvap ) return 1;
    else return 0;
 }

int laser_asserv_max_corr_time_Verify(int t){
    if( laser_asserv_min_vel_apply_time < t) return 1;
    else return 0;
}

 int laser_asserv_trsl_mindetect_dv_Verify(int tmdv){
    if( 1 <= tmdv ) return 1;
    else return 0;
 }

 int laser_asserv_trsl_correction_dv_Verify(int tcdv){
    if( 1 <= tcdv ) return 1;
    else return 0;
 }

int laser_asserv_trsl_sync_time_Verify(int tst){
    if( 150000 <= tst ) return 1;
    else return 0;
}

int laser_asserv_tol_diff_mottrans_indic_Verify(int tdi){
    if( 1 <= tdi ) return 1;
    else return 0;
}

int laser_asserv_trsl_coupl_inc_Verify(int ci){
    if( 1 <= ci ) return 1;
    else return 0;
}

int laser_accel_error_Verify(int lae){
    if( 1 <= lae ) return 1;
    else return 0;
}

int pipeDiam_Verify(int pd){
    if( 1 <= pd ) return 1;
    else return 0;
}

int Vitesse_de_croisiere_Verify(int v){
    if( 0 <= v ) return 1;
    else return 0;
}

int Vitesse_lente_Verify(int v){
    if( 0 <= v ) return 1;//valide
    else return 0;//pas valide
}

//fonctions Get
unsigned int User_Tolerance_Get(void){
    return (User_Tolerance / 10);
}
INTEGER32 Vitesse_de_croisiere_Get(void){
    return (INTEGER32)((double)Vitesse_de_croisiere / STEPS_PER_REV * 60 * REL_TRSL_ROT / REDUCTION_TRANS);
}
INTEGER32 Vitesse_lente_Get(void){
    return (INTEGER32)((double)Vitesse_lente * 60 * REL_TRSL_ROT / STEPS_PER_REV / REDUCTION_TRANS);
}

int pipe_diam_Get(void){
    return pipe_diam / 10;
}

int laser_asserv_sync_time_Get(void){
    return (laser_asserv_sync_time / 1000);
}
int laser_asserv_deltavitessemax_correction_Get(void){//rotation
    return (laser_asserv_deltavitessemax_correction / 1000);
}
int laser_asserv_motrot_pos_incertitude_Get(void){
    return (laser_asserv_motrot_pos_incertitude * 360 / STEPS_PER_REV);
}
int laser_asserv_max_motrot_velocity_Get(void){
    return (int)((double)laser_asserv_max_motrot_velocity * 360 * 60 / STEPS_PER_REV / REDUCTION_ROTATION);
}
int laser_asserv_min_vel_apply_time_Get(void){
    return (laser_asserv_min_vel_apply_time / 1000);
}
int laser_asserv_max_corr_time_Get(void){
    return (laser_asserv_max_corr_time / 1000);
}

int laser_asserv_trsl_mindetect_dv_Get(void){
    return (laser_asserv_trsl_mindetect_dv / 1000);
}
int laser_asserv_trsl_correction_dv_Get(void){
    return (laser_asserv_trsl_correction_dv / 1000);
}
int laser_asserv_trsl_sync_time_Get(void){
    return (laser_asserv_trsl_sync_time / 1000);
}
int laser_asserv_trsl_coupl_inc_Get(void){
    return laser_asserv_trsl_coupl_inc / 10;
}
int laser_accel_error_Get(void){
    return laser_accel_error;
}

//Fonctions Set
void User_Tolerance_Set(unsigned int ut){
    User_Tolerance = 10*ut;
}
void Vitesse_de_croisiere_Set(INTEGER32 v){
    Vitesse_de_croisiere = (INTEGER32)((double)v * STEPS_PER_REV / 60 / REL_TRSL_ROT * REDUCTION_TRANS + 1);
}
void Vitesse_lente_Set(INTEGER32 v){
    Vitesse_lente = (INTEGER32)((double)v * STEPS_PER_REV / 60 / REL_TRSL_ROT * REDUCTION_TRANS + 1);
}

void laser_asserv_sync_time_Set(int st){
    laser_asserv_sync_time = 1000 * st;
}
void laser_asserv_deltavitessemax_correction_Set(int dvmr){
    laser_asserv_deltavitessemax_correction = dvmr * 1000;
}
void  laser_asserv_motrot_pos_incertitude_Set(int mrpi){
    laser_asserv_motrot_pos_incertitude = mrpi * STEPS_PER_REV / 360;
}
void laser_asserv_max_motrot_velocity_Set(int mrv){
    laser_asserv_max_motrot_velocity = (int)((double)mrv * STEPS_PER_REV / 360 * REDUCTION_ROTATION / 60 + 1);
}
void laser_asserv_min_vel_apply_time_Set(int t){
    laser_asserv_min_vel_apply_time = 1000 * t;
}
void laser_asserv_max_corr_time_Set(int t){
    laser_asserv_max_corr_time = 1000 * t;
}

void laser_asserv_trsl_mindetect_dv_Set(int mddv){
    laser_asserv_trsl_mindetect_dv = mddv * 1000;
}
void laser_asserv_trsl_correction_dv_Set(int c){
    laser_asserv_trsl_correction_dv = c * 1000;
}
void laser_asserv_trsl_sync_time_Set(int st){
    laser_asserv_trsl_sync_time =  st * 1000;
}
void laser_asserv_trsl_coupl_inc_Set(int coupl){
    laser_asserv_trsl_coupl_inc = coupl * 10;
}
void laser_accel_error_Set(int a){
    laser_accel_error = a;
}
void pipe_diam_Set(int d){
    pipe_diam = d * 10;
}

//fonctions utilisateur
int laser_asserv_cst_verify(void){
    if ( User_Tolerance_Verify(User_Tolerance)
        && Vitesse_de_croisiere_Verify(Vitesse_de_croisiere)
        && Vitesse_lente_Verify(Vitesse_lente)
        && laser_asserv_sync_time_Verify(laser_asserv_sync_time)
        && laser_asserv_motrot_pos_incertitude_Verify(laser_asserv_motrot_pos_incertitude)
        && laser_asserv_err_vitesse_rot_Verify(laser_asserv_err_vitesse_rot)
        && laser_asserv_max_motrot_velocity_Verify(laser_asserv_max_motrot_velocity)
        && laser_asserv_min_vel_apply_time_Verify(laser_asserv_min_vel_apply_time)
        && laser_asserv_max_corr_time_Verify(laser_asserv_max_corr_time)
        && laser_asserv_trsl_mindetect_dv_Verify(laser_asserv_trsl_mindetect_dv)
        && laser_asserv_trsl_correction_dv_Verify(laser_asserv_trsl_correction_dv)
        && laser_asserv_trsl_sync_time_Verify(laser_asserv_sync_time)
        && laser_asserv_tol_diff_mottrans_indic_Verify(laser_asserv_tol_diff_mottrans_indic)
        && laser_asserv_trsl_coupl_inc_Verify(laser_asserv_trsl_coupl_inc)
        && laser_accel_error_Verify(laser_accel_error)
        && pipeDiam_Verify(pipe_diam)) return 1;//entrees pas valide
    else return 0;//entree valide

}

void laser_asserv_cst_set_default(void){
    User_Tolerance = 100;//en dixieme de mm.
    Vitesse_de_croisiere = 300000;
    Vitesse_lente = 100000;
    //constante utilisateur rotation
    laser_asserv_sync_time = LASER_ASSERV_SYNC_TIME;
    laser_asserv_deltavitessemax_correction = LASER_ASSERV_DELTAVITESSEMAX_CORRECTION;
    laser_asserv_motrot_pos_incertitude = LASER_ASSERV_MOTROT_POS_INCERTITUDE;
    laser_asserv_err_vitesse_rot = LASER_ASSERV_ERR_VITESSE_ROT;
    laser_asserv_max_motrot_velocity = LASER_ASSERV_MAX_MOTROT_VELOCITY;
    laser_asserv_min_vel_apply_time = LASER_ASSERV_MIN_VEL_APPLY_TIME;
    laser_asserv_max_corr_time = LASER_ASSERV_MAX_CORR_TIME;
    //constante utilisateur translation
    laser_asserv_trsl_mindetect_dv = LASER_ASSERV_TRSL_MINDETECT_DV;
    laser_asserv_trsl_correction_dv = LASER_ASSERV_TRSL_CORRECTION_DV;
    laser_asserv_trsl_sync_time = LASER_ASSERV_TRSL_SYNC_TIME;
    laser_asserv_tol_diff_mottrans_indic = LASER_ASSERV_TOL_DIFF_MOTTRANS_INDIC;
    laser_asserv_trsl_coupl_inc = LASER_ASSERV_TRSL_COUPL_INC;
    laser_accel_error = LASER_ACCEL_ERROR;
    //autre constantes
    pipe_diam = PIPE_DIAM;
}



FILE * laser_asserv_cst_f;
const char * laser_asserv_cst_fn = "./laser/laser_asserv_cst.txt";



int laser_asserv_cst_save(void){

    laser_asserv_cst_f = fopen(laser_asserv_cst_fn, "w");
    if (laser_asserv_cst_f == NULL) return 1;

    int res = fprintf(laser_asserv_cst_f, "%u %d %d %d %d %d %d %d %d %d %d %d %d %d %d\n",
        User_Tolerance, Vitesse_de_croisiere, Vitesse_lente, laser_asserv_sync_time, laser_asserv_deltavitessemax_correction,
        laser_asserv_motrot_pos_incertitude, laser_asserv_max_motrot_velocity, laser_asserv_min_vel_apply_time, laser_asserv_max_corr_time, laser_asserv_trsl_mindetect_dv,
        laser_asserv_trsl_correction_dv, laser_asserv_trsl_sync_time, laser_asserv_trsl_coupl_inc, laser_accel_error, pipe_diam);
    if (res < 0) return 1;

    fclose(laser_asserv_cst_f);
    return 0;

}

int laser_asserv_cst_recup(void){
    laser_asserv_cst_f = fopen(laser_asserv_cst_fn, "r");
    if (laser_asserv_cst_f == NULL) return 1;
    int res = fscanf(laser_asserv_cst_f, "%u %d %d %d %d %d %d %d %d %d %d %d %d %d %d",
        &User_Tolerance, &Vitesse_de_croisiere, &Vitesse_lente, &laser_asserv_sync_time, &laser_asserv_deltavitessemax_correction,
        &laser_asserv_motrot_pos_incertitude, &laser_asserv_max_motrot_velocity, &laser_asserv_min_vel_apply_time, &laser_asserv_max_corr_time, &laser_asserv_trsl_mindetect_dv,
        &laser_asserv_trsl_correction_dv, &laser_asserv_trsl_sync_time, &laser_asserv_trsl_coupl_inc, &laser_accel_error, &pipe_diam);
    if(res < 0) return 1;

    fclose(laser_asserv_cst_f);
    return 0;
}

void laser_asserv_cst_Get(void){
    unsigned int data;
    int data2;
    data = User_Tolerance_Get(); gui_entry_set("ent_errPos", strtools_gnum2str_all_decimal(&data, uint32));
    data2 = Vitesse_lente_Get(); gui_entry_set("ent_vitesseEscargot", strtools_gnum2str(&data2, int32));
    data2 = Vitesse_de_croisiere_Get(); gui_entry_set("ent_vitesseCroisiere", strtools_gnum2str(&data2, int32));
    data2 = pipe_diam_Get(); gui_entry_set("ent_pipeDiam", strtools_gnum2str(&data2, int32));

    data2 = laser_asserv_sync_time_Get(); gui_entry_set("ent_syncTimeRot", strtools_gnum2str(&data2, int32));
    data2 = laser_asserv_motrot_pos_incertitude_Get(); gui_entry_set("ent_errMinReact", strtools_gnum2str(&data2, int32));
    data2 = laser_asserv_min_vel_apply_time_Get(); gui_entry_set("ent_minTCorr", strtools_gnum2str(&data2, int32));
    data2 = laser_asserv_max_corr_time_Get(); gui_entry_set("ent_maxTCorr", strtools_gnum2str(&data2, int32));
    data2 = laser_asserv_deltavitessemax_correction_Get(); gui_entry_set("ent_dvMaxRot", strtools_gnum2str(&data2, int32));
    data2 = laser_asserv_max_motrot_velocity_Get(); gui_entry_set("ent_VMaxRot", strtools_gnum2str(&data2, int32));

    data2 = laser_asserv_trsl_correction_dv_Get(); gui_entry_set("ent_dvMaxTrsl", strtools_gnum2str(&data2, int32));
    data2 = laser_asserv_trsl_mindetect_dv_Get(); gui_entry_set("ent_factSensiVit", strtools_gnum2str(&data2, int32));
    data2 = laser_asserv_trsl_sync_time_Get(); gui_entry_set("ent_syncTimeTrsl", strtools_gnum2str(&data2, int32));
    data2 = laser_asserv_trsl_coupl_inc_Get(); gui_entry_set("ent_incCoupl", strtools_gnum2str(&data2, int32));
    data2 = laser_accel_error_Get(); gui_entry_set("ent_factSensiVarVit", strtools_gnum2str(&data2, int32));
}

void laser_asserv_cst_Set(void){

    User_Tolerance_Set((unsigned int)gui_str2num(gui_entry_get("ent_errPos")));
    Vitesse_lente_Set((INTEGER32)gui_str2num(gui_entry_get("ent_vitesseEscargot")));
    Vitesse_de_croisiere_Set((INTEGER32)gui_str2num(gui_entry_get("ent_vitesseCroisiere")));
    pipe_diam_Set((int)gui_str2num(gui_entry_get("ent_pipeDiam")));

    laser_asserv_sync_time_Set((int)gui_str2num(gui_entry_get("ent_syncTimeRot")));
    laser_asserv_motrot_pos_incertitude_Set((int)gui_str2num(gui_entry_get("ent_errMinReact")));
    laser_asserv_min_vel_apply_time_Set((int)gui_str2num(gui_entry_get("ent_minTCorr")));
    laser_asserv_max_corr_time_Set((int)gui_str2num(gui_entry_get("ent_maxTCorr")));
    laser_asserv_deltavitessemax_correction_Set((int)gui_str2num(gui_entry_get("ent_dvMaxRot")));
    laser_asserv_max_motrot_velocity_Set((int)gui_str2num(gui_entry_get("ent_VMaxRot")));

    laser_asserv_trsl_correction_dv_Set((int)gui_str2num(gui_entry_get("ent_dvMaxTrsl")));
    laser_asserv_trsl_mindetect_dv_Set((int)gui_str2num(gui_entry_get("ent_factSensiVit")));
    laser_asserv_trsl_sync_time_Set((int)gui_str2num(gui_entry_get("ent_syncTimeTrsl")));
    laser_asserv_trsl_coupl_inc_Set((int)gui_str2num(gui_entry_get("ent_incCoupl")));
    laser_accel_error_Set((int)gui_str2num(gui_entry_get("ent_factSensiVarVit")));
}
