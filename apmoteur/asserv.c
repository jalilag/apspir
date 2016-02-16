#include "asserv.h"
#include <stdlib.h>
#include <stdio.h>
#include <string.h>
#include <unistd.h>
#include "SpirallingControl.h"
#include "slave.h"
#include "profile.h"
#include "strtools.h"
#include <math.h>
#include "cantools.h"
#include "motor.h"
#define NUMBER_VELOCITY 20

CONS cst[CONST_NUMBER_LIMIT] = {
    {"User_Tolerance",0,1,100},
    {"Vitesse_De_Croisiere",0,0,0},
    {"Vitesse_Lente",0,0,0},
    {"Sync_Time_Rot",0,150000,10000000},
    {"dVmax_Correction",0,0,0},
    {"Incertitude_Rot",0,10000,0},
    {"Max_Vel_Rot",0,0,0},
    {"Min_Time_Apply_Vel",0,0,0},
    {"Max_Time_Correction",0,0,0},
    {"Min_dV_Translation",0,0,0},
    {"dV_Correction_Trans",0,0,0},
    {"dV_Correction_Trans",0,0,0},
    {"Sync_Time_Trans",0,0,0},
    {"Couple_Inc_Trans",0,0,0},
    {"Acc_Error",0,0,0},
    {"Pipe_Diameter",0,0,0}
};

// Variable externe
extern double length_start,length_actual_laser,length_actual_sync;
extern double time_start,time_actual_laser,time_actual_sync;
extern double length_covered_laser,length_covered_sync,mean_velocity;
extern INTEGER32 rot_pos_start,rot_pos_actual;
extern int current_step, SLAVE_NUMBER,set_up,num_of_cycle_nom,num_of_cycle_cor;

static INTEGER32 vel_nom_old=0,vel_cor_old=0;

int asserv_init() {
    double l,t;

    if(!serialtools_get_laser_data_valid(&l,&t)) {
        return 0;
    }

    // Initialisation des variables globales
    length_start = l;
    length_actual_laser = length_start;
    length_actual_sync = length_start;
    length_covered_laser = 0;
    length_covered_sync = 0;
    time_start = cantools_get_time();
    time_actual_laser = time_start;
    time_actual_sync = time_start;
    current_step = slave_get_step_with_length(0);
    rot_pos_start = slave_get_param_in_num("Position",slave_get_index_with_profile_id("RotVit"));
    rot_pos_actual = rot_pos_start;
    // Initialisation des fichiers
    FILE* helix_dat = fopen(FILE_HELIX_RECORDED,"w"); // Position
    if (helix_dat == NULL) return 0;
    fputs("#    Distance    Position\n",helix_dat);
    fputs("0 0\n",helix_dat);
    fclose(helix_dat);
    FILE* vel_laser_dat = fopen(FILE_VELOCITY_LASER,"w"); // Vitesse
    if (vel_laser_dat == NULL) return 0;
    fputs("# time VlaserMean VlaserInst\n",vel_laser_dat);
    fputs("0 0 0\n",vel_laser_dat);
    fclose(vel_laser_dat);
    FILE* vel_sync_dat = fopen(FILE_VELOCITY_SYNC,"w"); // Vitesse
    if (vel_sync_dat == NULL) return 0;
    fputs("# time Vtrans Vrot\n",vel_sync_dat);
    fputs("0 0 0\n",vel_sync_dat);
    fclose(vel_sync_dat);
    FILE* err_dat = fopen(FILE_ERROR,"w"); // Vitesse
    if (err_dat == NULL) return 0;
    fputs("# time err(step)\n",err_dat);
    fputs("0 0\n",err_dat);
    fclose(err_dat);
    return 1;
}

int asserv_check() {
    double t,l;
    INTEGER32 rot_pos_theo;
    // Mesure laser
    if (!serialtools_get_laser_data_valid(&l,&t)) {
        set_up = 0;
        return 0;
    }
    rot_pos_actual = slave_get_param_in_num("Position",slave_get_index_with_profile_id("RotVit"));
    // longueur, temps et vitesse actuelle
    double vel_actual = fabs((l - length_actual_laser)/(t-time_actual_laser));
    length_actual_laser = l;
    length_covered_laser = fabs(length_actual_laser - length_start);
    time_actual_laser = t;
    // Calcul de la longueur laser ajustée au temps synchro
    double dt = time_actual_sync - time_actual_laser;
    length_actual_sync = mean_velocity*dt+length_actual_laser;
    length_covered_sync = fabs(length_actual_sync - length_start);
    current_step = slave_get_step_with_length(length_covered_sync);
    // Calcul de la position theorique à la longueur au temps sync
    rot_pos_theo = slave_get_position_with_length(length_covered_sync);
    // Calcul de la position effective
    INTEGER32 rot_pos = abs(rot_pos_actual - rot_pos_start);
    // Calcul de l'erreur
    INTEGER32 rot_pos_err_in_step = rot_pos_theo-rot_pos;
    double rot_pos_err_in_length = M_PI*PIPE_DIAMETER/(STEP_PER_REV*ROT_REDUCTION)*rot_pos_err_in_step;
    // Maj des fichiers
    double vtrans = (double)slave_get_param_in_num("Velocity",slave_get_index_with_profile_id("TransCouple"))*WHEEL_PERIMETER/(STEP_PER_REV*TRANS_REDUCTION)*60; // Vitesse trans
    double vrot = (double)slave_get_param_in_num("Velocity",slave_get_index_with_profile_id("RotVit"))*current_step/(STEP_PER_REV*ROT_REDUCTION)*60; // Vitesse rot
    FILE* helix_dat = fopen(FILE_HELIX_RECORDED,"a");
    if (helix_dat != NULL) {
            double rrpos = sin((double)rot_pos*2*M_PI/(STEP_PER_REV*ROT_REDUCTION));
            fputs(strtools_replace_char(strtools_concat(strtools_gnum2str(&length_actual_sync,0x10)," ",strtools_gnum2str(&rrpos,0x10),"\n",NULL),',','.'),helix_dat);
            fclose(helix_dat);
    }
    FILE* vel_laser_dat = fopen(FILE_VELOCITY_LASER,"a");
    if (vel_laser_dat != NULL) {
        t = time_actual_laser - time_start;
        double meanv = 60*mean_velocity;
        double vinst = 60*vel_actual;
        fputs(strtools_replace_char(strtools_concat(strtools_gnum2str(&t,0x10)," ",strtools_gnum2str(&meanv,0x10)," ",strtools_gnum2str(&vinst,0x10),"\n",NULL),',','.'),vel_laser_dat);
        fclose(vel_laser_dat);
    }
    FILE* vel_sync_dat = fopen(FILE_VELOCITY_SYNC,"a");
    if (vel_sync_dat != NULL) {
        t = time_actual_sync - time_start;
        fputs(strtools_replace_char(strtools_concat(strtools_gnum2str(&t,0x10)," ",strtools_gnum2str(&vtrans,0x10)," ",strtools_gnum2str(&vrot,0x10),"\n",NULL),',','.'),vel_sync_dat);
        fclose(vel_sync_dat);
    }
    FILE* err_dat = fopen(FILE_ERROR,"a");
    if (err_dat != NULL) {
        t = time_actual_sync - time_start;
        fputs(strtools_replace_char(strtools_concat(strtools_gnum2str(&t,0x10)," ",strtools_gnum2str(&rot_pos_err_in_step,0x04),"\n",NULL),',','.'),err_dat);
        fclose(err_dat);
    }
    // Correction de la vitesse
    num_of_cycle_cor = 1;
    int i = slave_get_index_with_profile_id("RotVit");
    double tsync = (double)CYCLE_PERIOD/1000000;
    INTEGER32 vel_rot_nom = STEP_PER_REV*ROT_REDUCTION*mean_velocity/current_step;
    INTEGER32 vel_rot_cor = 2*rot_pos_err_in_step/(num_of_cycle_cor*tsync);
    while(vel_rot_cor > 100000) {
        num_of_cycle_cor++;
        vel_rot_cor = 2*rot_pos_err_in_step/(num_of_cycle_cor*tsync);
    }

    INTEGER32 acc_rot_nom = (vel_rot_nom-vel_nom_old)/tsync;
    INTEGER32 acc_rot_cor = (vel_rot_cor-vel_cor_old)/tsync;
    INTEGER32 vel_rot = vel_rot_cor + vel_rot_nom;
    INTEGER32 acc_rot = acc_rot_nom + acc_rot_cor;
    vel_nom_old = vel_rot_nom;
    vel_cor_old = vel_rot_cor;
    printf("current step %d\n",current_step);
    printf("tsync %f\n",tsync);
    printf("mean vel %f\n",mean_velocity);
    printf("rot_pos_actual %d\n",rot_pos_actual);
    printf("rot_pos_start %d\n",rot_pos_start);
    printf("rot_pos %d\n",rot_pos);
    printf("rot_pos_theo %d\n",rot_pos_theo);
    printf("length_covered %f\n",length_covered_laser);
    printf("length_covered on sync %f\n",length_covered_sync);
    printf("Vel rot nom %d\n",vel_rot_nom);
    printf("Vel rot cor %d\n",vel_rot_cor);
    printf("Vel rot %d\n",vel_rot);
    printf("acc rot nom %d\n",acc_rot_nom);
    printf("acc rot cor %d\n",acc_rot_cor);
    printf("acc rot %d\n",acc_rot);
    if (vel_rot<0) vel_rot = 0;
    if (acc_rot>0) slave_set_param("Acc2send",i,acc_rot);
    else slave_set_param("Dcc2send",i,abs(acc_rot));
    slave_set_param("Vel2send",i,vel_rot);
    return 1;
}


static int asserv_rot_acc(void) {
//    INTEGER32 accel_T, decel_T,accel_R, decel_R;
//    if(!motor_get_param(slave_get_node_with_profile_id("RotVit"),"Acc",&accel_R)) return 0;
//    if(!motor_get_param(slave_get_node_with_profile_id("RotVit"),&decel_R)) return 0;
//    if(!motor_get_param(slave_get_node_with_profile_id("TransVit"),&accel_T)) return 0;
//    if(!motor_get_param(slave_get_node_with_profile_id("TransVit"),&decel_T)) return 0;
//    //calcul des acceleration rotation correspondante
//    int K;
//    if (current_step != 0 ) {
//        K = WHEEL_PERIMETER/current_step * ROT_REDUCTION/TRANS_REDUCTION;
//        accel_R = accel_T*K;
//        decel_R = decel_T*K;
//        //ecriture des accelerations
//        if(!motor_set_param(slave_get_node_with_profile_id("RotVit"),"Acc",accel_R)) return 0;
//        if(!motor_set_param(slave_get_node_with_profile_id("RotVit"),"Dcc",decel_R)) return 0;
//    }
//    return 1;
}

gpointer asserv_calc_mean_velocity(gpointer data) {
    double l = length_actual_laser ,t = time_actual_laser;
    double ll, tt,vel=0,vel_cumul=0;
    double velvec [NUMBER_VELOCITY] = {0};
    int i;
    while(set_up) {
        usleep(100000);
        ll = l; tt = t;
        if (serialtools_get_laser_data_valid(&l,&t)) {
            if (t > tt) {
                vel = fabs((l-ll)/(t-tt));
                vel_cumul = 0;
                for (i=1; i<NUMBER_VELOCITY;i++) {
                    velvec[i-1] = velvec[i];
                    vel_cumul += velvec[i];
                }
                velvec[NUMBER_VELOCITY-1] = vel;
                mean_velocity = vel_cumul/NUMBER_VELOCITY;
            }
        }
    }
    return 0;
}


