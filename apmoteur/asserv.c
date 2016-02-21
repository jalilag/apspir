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
#include "gui.h"
#include "keyword.h"
#include <gtk/gtk.h>
#include "master.h"
#define NUMBER_VELOCITY 20

// Variable externe
extern double length_start,length_actual_laser,length_actual_sync,min_length;
extern double time_start,time_actual_laser,time_actual_sync,tcalc;
extern double length_covered_laser,length_covered_sync,mean_velocity;
extern INTEGER32 rot_pos_start,rot_pos_actual,trans_vel,rot_vel;
extern int current_step, SLAVE_NUMBER,set_up,trans_type,trans_direction,rot_direction;
extern INTEGER32 vitesse_max;
extern INTEGER32 rot_pos_err_in_step,rot_pos_err_mean_in_step;
extern double rot_pos_err_in_mm,rot_pos_err_mean_in_mm;

static INTEGER32 vel_cor_old=0;
static double vel_rot_cor_old = 0,rot_pos_err_cumul=0;
static INTEGER32 rot_pos_err_old = 0;
static int ind=0;

CONS cst[CONST_NUMBER_LIMIT] = {
    {"TransType",1,1,3},
    {"VitTrans",3,1,3},
    {"RotDirection",0,0,1},
    {"VitRotMax",300000,1,300000},
    {"PeriodSync",100,50,1000},
    {"PeriodCor",500,200,1000},
    {"ErrorMax",10,1,100}
};

int asserv_check_const(char* name,double val) {
    int i = asserv_get_const_index_with_name(name);
    if (val >= cst[i].valmin && val <= cst[i].valmax) return 1;
    else {
        gui_info_popup(strtools_concat(CONST_CHECK_ERROR_1,name,CONST_CHECK_ERROR_2,strtools_gnum2str(&cst[i].valmin,0x10),CONST_CHECK_ERROR_3,strtools_gnum2str(&cst[i].valmax,0x10),NULL),NULL);
        return 0;
    }
}

int asserv_get_const_index_with_name(char* name) {
    int i;
    for (i=0;i<CONST_NUMBER_LIMIT;i++) {
        if (strcmp(cst[i].name,name)==0) return i;
    }
    return -1;
}
int asserv_get_const_index_with_id(char* id) {
    int i;
    for (i=0;i<CONST_NUMBER_LIMIT;i++) {
        if (strcmp(strtools_concat("ent",cst[i].name,NULL),id)==0) return i;
    }
    return -1;
}
char* asserv_get_const_name_with_id(char* id) {
    int i;
    for (i=0;i<CONST_NUMBER_LIMIT;i++) {
        if (strcmp(strtools_concat("ent",cst[i].name,NULL),id)==0) return cst[i].name;
    }
    return NULL;
}


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
    printf("rot_pos 1 %d %d %d\n",rot_pos_start,rot_pos_actual,PositionR_4);
    rot_pos_err_cumul = 0;
    rot_pos_err_in_step = 0;
    ind = 0;
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
    if (t-time_actual_laser >= tcalc) {
        rot_pos_actual = slave_get_param_in_num("Position",slave_get_index_with_profile_id("RotVit"));
        printf("rot_pos 2 %d %d %d\n",rot_pos_start,rot_pos_actual,PositionR_4);
        INTEGER32 vel_rot_actual = slave_get_param_in_num("Velocity",slave_get_index_with_profile_id("RotVit"));
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
        if (rot_pos < 100) rot_pos =0;
        // Calcul de l'erreur
        rot_pos_err_in_step = rot_pos_theo-rot_pos;
        ind++;
        rot_pos_err_cumul += abs(rot_pos_err_in_step);
        rot_pos_err_mean_in_step = rot_pos_err_cumul/ind;
        rot_pos_err_in_mm = M_PI*PIPE_DIAMETER/(STEP_PER_REV*ROT_REDUCTION)*rot_pos_err_in_step;
        rot_pos_err_mean_in_mm = M_PI*PIPE_DIAMETER/(STEP_PER_REV*ROT_REDUCTION)*rot_pos_err_mean_in_step;
        // Maj des fichiers
        double vtrans = (double)slave_get_param_in_num("Velocity",slave_get_index_with_profile_id("TransCouple"))*WHEEL_PERIMETER/(STEP_PER_REV*TRANS_REDUCTION)*60; // Vitesse trans
        double vrot = (double)slave_get_param_in_num("Velocity",slave_get_index_with_profile_id("RotVit"))*current_step/(STEP_PER_REV*ROT_REDUCTION)*60; // Vitesse rot
        FILE* helix_dat = fopen(FILE_HELIX_RECORDED,"a");
        if (helix_dat != NULL) {
                double rrpos = sin((double)rot_pos*2*M_PI/(STEP_PER_REV*ROT_REDUCTION));
                fputs(strtools_replace_char(strtools_concat(strtools_gnum2str(&length_covered_sync,0x10)," ",strtools_gnum2str(&rrpos,0x10),"\n",NULL),',','.'),helix_dat);
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
            fputs(strtools_replace_char(strtools_concat(strtools_gnum2str(&t,0x10)," ",strtools_gnum2str(&rot_pos_err_in_step,0x04)," ",strtools_gnum2str(&rot_pos_err_mean_in_step,0x04),"\n",NULL),',','.'),err_dat);
            fclose(err_dat);
        }
        // Correction de la vitesse
        int num_of_cycle_cor = 1;
        int i = slave_get_index_with_profile_id("RotVit");
        INTEGER32 rot_pos_err_new = rot_pos_err_in_step - rot_pos_err_old/2;
        INTEGER32 rot_pos_err_res = rot_pos_err_old/2;
        rot_pos_err_old = rot_pos_err_new;
        INTEGER32 vel_rot_nom = STEP_PER_REV*ROT_REDUCTION*mean_velocity/current_step;
        INTEGER32 vel_rot_cor_actual = vel_rot_actual-vel_rot_nom;
        INTEGER32 vel_rot_cor = (rot_pos_err_new/2+rot_pos_err_res)/tcalc-vel_rot_cor_actual;
        INTEGER32 vel_rot = vel_rot_cor + vel_rot_nom;
        while(vel_rot > vitesse_max) {
            num_of_cycle_cor++;
            vel_rot_cor = vel_rot_cor/num_of_cycle_cor;
            vel_rot = vel_rot_cor + vel_rot_nom;
        }
        INTEGER32 acc_rot = (vel_rot-slave_get_param_in_num("Velocity",slave_get_index_with_profile_id("RotVit")))/tcalc;
        printf("---------\nSYNCRO\n---------\n");
        printf("current step           : %d\n",current_step);
        printf("time                  : %f %f\n",time_actual_laser, time_actual_sync);
        printf("mean vel               : %f\n",mean_velocity);
        printf("rot_pos_actual         : %d\n",rot_pos_actual);
        printf("rot_pos_start          : %d\n",rot_pos_start);
        printf("rot_pos                : %d\n",rot_pos);
        printf("rot_pos_theo           : %d\n",rot_pos_theo);
        printf("length_start           : %f\n",length_start);
        printf("length                 : %f\n",length_actual_laser);
        printf("length_covered         : %f\n",length_covered_laser);
        printf("length_covered on sync : %f\n",length_covered_sync);
        printf("Vel rot nom            : %d\n",vel_rot_nom);
        printf("Vel rot cor            : %d\n",vel_rot_cor);
        printf("Vel rot                : %d\n",vel_rot);
        printf("acc rot                : %d\n",acc_rot);
        if (vel_rot<=0) vel_rot = 0;
        if (acc_rot>0) slave_set_param("Acc2send",i,acc_rot);
        else slave_set_param("Dcc2send",i,abs(acc_rot));
        slave_set_param("Couple2send",i,motor_get_couple_for_rot(vel_rot));
        printf("Couple                 : %d",slave_get_param_in_num("Couple2send",slave_get_index_with_profile_id("RotVit")));
        slave_set_param("Vel2send",i,vel_rot);
        printf("Vel %d Vel2send %d Acc2send %d\n",slave_get_param_in_num("Velocity",slave_get_index_with_profile_id("RotVit")),slave_get_param_in_num("Vel2send",slave_get_index_with_profile_id("RotVit")),slave_get_param_in_num("Acc2send",slave_get_index_with_profile_id("RotVit")));
    }
    return 1;
}

gpointer asserv_calc_mean_velocity(gpointer data) {
    double l = length_actual_laser ,t = time_actual_laser;
    double ll, tt,vel=0,vel_cumul=0;
    double velvec [NUMBER_VELOCITY] = {trans_vel/STEP_PER_REV/TRANS_REDUCTION*WHEEL_PERIMETER};
    mean_velocity= trans_vel/STEP_PER_REV/TRANS_REDUCTION*WHEEL_PERIMETER;
    int i;
    while(set_up) {
        usleep(100000);
        ll = l; tt = t;
        if (serialtools_get_laser_data_valid(&l,&t)) {
            if (t > tt && ll-l > min_length)  {
                vel = fabs((l-ll)/(t-tt));
                vel_cumul = 0;
                for (i=1; i<NUMBER_VELOCITY;i++) {
                    velvec[i-1] = velvec[i];
                    vel_cumul += velvec[i];
                }
                velvec[NUMBER_VELOCITY-1] = vel;

                if (vel_cumul/NUMBER_VELOCITY < 0.005) mean_velocity = 0;
                else mean_velocity = vel_cumul/NUMBER_VELOCITY;
            }
        }
    }
    return 0;
}

int asserv_motor_config() {
    if (trans_type == 1) trans_vel = 250000;
    if (trans_type == 3) trans_vel = 0;
    int i;
    for (i=0;i<SLAVE_NUMBER;i++) {
        if (slave_get_param_in_num("Active",i)) {
            if(slave_get_profile_id_with_index(i) == "TransCouple") {
                if (trans_type < 3) {
                    if (!motor_forward(slave_get_node_with_index(i),trans_direction)) return 0;
                    INTEGER32 acc = 1000/tcalc;
                    if(!motor_set_param(slave_get_node_with_index(i),"TorqueSlope",acc)) return 0;
                    if(!motor_set_param(slave_get_node_with_index(i),"Torque",1000)) return 0;
                }
            }
            if(slave_get_profile_id_with_index(i) == "TransVit") {
                if (trans_type == 2) {
                    if (!motor_forward(slave_get_node_with_index(i),trans_direction)) return 0;
                    INTEGER32 acc = trans_vel/tcalc;
                    if(!motor_set_param(slave_get_node_with_index(i),"Acc",acc)) return 0;
                    slave_set_param("Vel2send",i,trans_vel);
                }
            }
            if (slave_get_profile_id_with_index(i) == "RotVit") {
                if (!motor_forward(slave_get_node_with_index(i),trans_direction)) return 0;
                INTEGER32 rot_vel;
                if (trans_vel == 0) rot_vel = 0;
                else rot_vel = STEP_PER_REV*rot_direction/(current_step/(trans_vel/STEP_PER_REV/TRANS_REDUCTION*WHEEL_PERIMETER));
                INTEGER32 acc = rot_vel/tcalc;
                slave_set_param("Acc2send",i,acc);
                slave_set_param("Vel2send",i,rot_vel);
            }
            if(slave_get_profile_id_with_index(i) == "RotCouple") {
                if (!motor_forward(slave_get_node_with_index(i),rot_direction)) return 0;
                INTEGER32 acc = 1000/tcalc;
                if(!motor_set_param(slave_get_node_with_index(i),"TorqueSlope",acc)) return 0;
                slave_set_param("Couple2send",i,1000);
            }
        }
    }
    return 1;
}

int asserv_motor_start() {
    int i;
    if (trans_type == 1) {
        for (i=0;i<SLAVE_NUMBER;i++) {
            if (slave_get_param_in_num("Active",i)) {
                if (slave_get_profile_id_with_index(i) == "TransCouple" ||
                    slave_get_profile_id_with_index(i) == "RotVit" ||
                    slave_get_profile_id_with_index(i) == "RotCouple")
                    if(!motor_start(slave_get_node_with_index(i),1)) return 0;
            }
        }
    } else if (trans_direction == 2) {
        for (i=0;i<SLAVE_NUMBER;i++) {
            if (slave_get_param_in_num("Active",i)) {
                if (slave_get_profile_id_with_index(i) == "TransCouple" ||
                    slave_get_profile_id_with_index(i) == "TransVit" ||
                    slave_get_profile_id_with_index(i) == "RotVit" ||
                    slave_get_profile_id_with_index(i) == "RotCouple")
                    if(!motor_start(slave_get_node_with_index(i),1)) return 0;
            }
        }
    } else if (trans_direction == 3) {
        for (i=0;i<SLAVE_NUMBER;i++) {
            if (slave_get_param_in_num("Active",i)) {
                if (slave_get_profile_id_with_index(i) == "RotVit" ||
                    slave_get_profile_id_with_index(i) == "RotCouple")
                    if(!motor_start(slave_get_node_with_index(i),1)) return 0;
            }
        }
    }
    return 1;
}
