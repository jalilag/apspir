#include "errgen.h"
#include <stdlib.h>
#include "gui.h"
#include "keyword.h"
#include "strtools.h"
#include "gtksig.h"

extern UNS16 errgen_state;
extern int current_menu, run_init;
extern GMutex lock_gui_box;

void errgen_set(UNS16 dat, char* op) {
    if (dat == 0) dat = errgen_state;
    char* title = strtools_concat(ERROR, " : ", strtools_gnum2str(&dat,0x06)," ",errgen_get_title(dat), NULL);
    char* content = strtools_concat(errgen_get_content(dat),"\n",ERROR, " : ", strtools_gnum2str(&dat,0x06),NULL);
    if (op != NULL) content = strtools_concat(content,"\n",ELEM_KEY,op,NULL);
    if (dat == ERR_GUI_LOOP_RUN) {
        printf("%s",title);
        printf("\n");
        printf("%s",content);
        free(content); free(title);
        exit(EXIT_FAILURE);
    }

    if (
        dat == ERR_DRIVER_UP ||
        dat == ERR_DRIVER_LOAD ||
        dat == ERR_DRIVER_OPEN ||
        dat == ERR_INIT_LOOP_RUN ||
        dat == ERR_MASTER_CONFIG ||
        dat == ERR_FILE_PROFILE
    ) {
        content = strtools_concat(content,"\n\n",APPLICATION_SHUTDOWN,NULL);
        g_mutex_lock(&lock_gui_box);
        gui_info_box(title,content,NULL);
        free(content); free(title);
        g_mutex_lock(&lock_gui_box);
        Exit(2);
        g_mutex_unlock(&lock_gui_box);
    } else if (
        dat == ERR_SLAVE_CONFIG ||
        dat == ERR_SLAVE_CONFIG_LSS ||
        dat == ERR_SLAVE_CONFIG_MOTOR_SON
    ) {
        content = strtools_concat(content,"\n\n",SLAVE_INACTIVE,NULL);
        g_mutex_lock(&lock_gui_box);
        gui_info_box(title,content,NULL);
    } else if (dat == ERR_FILE_SLAVE_DEF) {
        content = strtools_concat(content,"\n\n",MOTOR_ID,NULL);
        g_mutex_lock(&lock_gui_box);
        gui_info_box(title,content,NULL);
        free(content); free(title);
        if (run_init != 0)
            on_butParams_clicked(NULL);
        else
            Exit(2);
    } else{
        g_mutex_lock(&lock_gui_box);
        gui_info_box(title,content,NULL);
        free(content); free(title);
    }
}


static char* errgen_get_title(UNS16 dat) {
    if (dat == ERR_DRIVER_UP) return ERR_DRIVER_UP_TITLE;
    else if (dat == ERR_DRIVER_LOAD) return ERR_DRIVER_LOAD_TITLE;
    else if (dat == ERR_DRIVER_OPEN) return ERR_DRIVER_OPEN_TITLE;
    else if (dat == ERR_INIT_LOOP_RUN) return ERR_INIT_LOOP_RUN_TITLE;
    else if (dat == ERR_GUI_LOOP_RUN) return ERR_GUI_LOOP_RUN_TITLE;

    else if (dat == ERR_FILE_OPEN) return ERR_FILE_OPEN_TITLE;
    else if (dat == ERR_FILE_EMPTY) return ERR_FILE_EMPTY_TITLE;
    else if (dat == ERR_FILE_GEN) return ERR_FILE_GEN_TITLE;
    else if (dat == ERR_FILE_SLAVE_DEF) return ERR_FILE_SLAVE_DEF_TITLE;
    else if (dat == ERR_FILE_PROFILE) return ERR_FILE_PROFILE_TITLE;
    else if (dat == ERR_FILE_PROFILE_INVALID_PARAM) return ERR_FILE_PROFILE_INVALID_PARAM_TITLE;
    else if (dat == ERR_MASTER_STOP_SYNC) return ERR_MASTER_STOP_SYNC_TITLE;
    else if (dat == ERR_MASTER_START_SYNC) return ERR_MASTER_START_SYNC_TITLE;
    else if (dat == ERR_MASTER_SET_PERIOD) return ERR_MASTER_SET_PERIOD_TITLE;
    else if (dat == ERR_MASTER_SET_HB_CONS) return ERR_MASTER_SET_HB_CONS_TITLE;
    else if (dat == ERR_MASTER_CONFIG_PDOR) return ERR_MASTER_CONFIG_PDOR_TITLE;
    else if (dat == ERR_MASTER_CONFIG_PDOT) return ERR_MASTER_CONFIG_PDOT_TITLE;
    else if (dat == ERR_MASTER_CONFIG) return ERR_MASTER_CONFIG_TITLE;

    else if (dat == ERR_SLAVE_CONFIG) return ERR_SLAVE_CONFIG_TITLE;
    else if (dat == ERR_SLAVE_CONFIG_LSS) return ERR_SLAVE_CONFIG_LSS_TITLE;
    else if (dat == ERR_SLAVE_CONFIG_HB) return ERR_SLAVE_CONFIG_HB_TITLE;
    else if (dat == ERR_SLAVE_CONFIG_PDOT) return ERR_SLAVE_CONFIG_PDOT_TITLE;
    else if (dat == ERR_SLAVE_CONFIG_PDOR) return ERR_SLAVE_CONFIG_PDOR_TITLE;
    else if (dat == ERR_SLAVE_CONFIG_ACTIVE_PDO) return ERR_SLAVE_CONFIG_ACTIVE_PDO_TITLE;
    else if (dat == ERR_SLAVE_CONFIG_CURRENT) return ERR_SLAVE_CONFIG_CURRENT_TITLE;
    else if (dat == ERR_SLAVE_CONFIG_PROFILE) return ERR_SLAVE_CONFIG_PROFILE_TITLE;
    else if (dat == ERR_SLAVE_CONFIG_MAX_VELOCITY) return ERR_SLAVE_CONFIG_MAX_VELOCITY_TITLE;
    else if (dat == ERR_SLAVE_CONFIG_ACCELERATION) return ERR_SLAVE_CONFIG_ACCELERATION_TITLE;
    else if (dat == ERR_SLAVE_CONFIG_DECELERATION) return ERR_SLAVE_CONFIG_DECELERATION_TITLE;
    else if (dat == ERR_SLAVE_CONFIG_DECELERATION_QS) return ERR_SLAVE_CONFIG_DECELERATION_QS_TITLE;
    else if (dat == ERR_SLAVE_CONFIG_TORQUE) return ERR_SLAVE_CONFIG_TORQUE_TITLE;
    else if (dat == ERR_SLAVE_CONFIG_TORQUE_SLOPE) return ERR_SLAVE_CONFIG_TORQUE_SLOPE_TITLE;
    else if (dat == ERR_SLAVE_CONFIG_TORQUE_MAX_VELOCITY) return ERR_SLAVE_CONFIG_TORQUE_MAX_VELOCITY_TITLE;
    else if (dat == ERR_SLAVE_CONFIG_MOTOR_SON) return ERR_SLAVE_CONFIG_MOTOR_SON_TITLE;
    else if (dat == ERR_SLAVE_SAVE_CONFIG) return ERR_SLAVE_SAVE_CONFIG_TITLE;
    else if (dat == ERR_SLAVE_LOAD_CONFIG) return ERR_SLAVE_LOAD_CONFIG_TITLE;
    else if (dat == ERR_SLAVE_LOAD_INTERFACE) return ERR_SLAVE_LOAD_INTERFACE_TITLE;

    else if (dat == ERR_READ_ACCELERATION) return ERR_READ_ACCELERATION_TITLE;
    else if (dat == ERR_READ_DECELERATION) return ERR_READ_DECELERATION_TITLE;
    else if (dat == ERR_READ_DECELERATION_QS) return ERR_READ_DECELERATION_QS_TITLE;
    else if (dat == ERR_READ_TORQUE) return ERR_READ_TORQUE_TITLE;
    else if (dat == ERR_READ_TORQUE_SLOPE) return ERR_READ_TORQUE_SLOPE_TITLE;
    else if (dat == ERR_READ_TORQUE_VELOCITY) return ERR_READ_TORQUE_VELOCITY_TITLE;
    else if (dat == ERR_READ_CURRENT) return ERR_READ_CURRENT_TITLE;
    else if (dat == ERR_READ_PROFILE) return ERR_READ_PROFILE_TITLE;
    else if (dat == ERR_READ_TORQUE_VELOCITY_MAKEUP) return ERR_READ_TORQUE_VELOCITY_MAKEUP_TITLE;
    else if (dat == ERR_READ_HMT_ACTIVATE) return ERR_READ_HMT_ACTIVATE_TITLE;
    else if (dat == ERR_READ_HMT_CONTROL) return ERR_READ_HMT_CONTROL_TITLE;

    else if (dat == ERR_SET_ACCELERATION) return ERR_SET_ACCELERATION_TITLE;
    else if (dat == ERR_SET_DECELERATION) return ERR_SET_DECELERATION_TITLE;
    else if (dat == ERR_SET_DECELERATION_QS) return ERR_SET_DECELERATION_QS_TITLE;
    else if (dat == ERR_SET_VELOCITY_INC) return ERR_SET_VELOCITY_INC_TITLE;
    else if (dat == ERR_SET_TORQUE) return ERR_SET_TORQUE_TITLE;
    else if (dat == ERR_SET_TORQUE_SLOPE) return ERR_SET_TORQUE_SLOPE_TITLE;
    else if (dat == ERR_SET_TORQUE_VELOCITY) return ERR_SET_TORQUE_VELOCITY_TITLE;
    else if (dat == ERR_SET_CURRENT) return ERR_SET_CURRENT_TITLE;
    else if (dat == ERR_SET_PROFILE) return ERR_SET_PROFILE_TITLE;
    else if (dat == ERR_SET_TORQUE_VELOCITY) return ERR_SET_TORQUE_VELOCITY_TITLE;
    else if (dat == ERR_SET_PDO) return ERR_SET_PDO_TITLE;
    else if (dat == ERR_SET_TORQUE_VELOCITY_MAKEUP) return ERR_SET_TORQUE_VELOCITY_MAKEUP_TITLE;
    else if (dat == ERR_SET_HMT_ACTIVATE) return ERR_SET_HMT_ACTIVATE_TITLE;
    else if (dat == ERR_READ_HMT_CONTROL) return ERR_SET_HMT_CONTROL_TITLE;


    else if (dat == ERR_SAVE_ACCELERATION) return ERR_SAVE_ACCELERATION_TITLE;
    else if (dat == ERR_SAVE_DECELERATION) return ERR_SAVE_DECELERATION_TITLE;
    else if (dat == ERR_SAVE_DECELERATION_QS) return ERR_SAVE_DECELERATION_QS_TITLE;
    else if (dat == ERR_SAVE_TORQUE) return ERR_SAVE_TORQUE_TITLE;
    else if (dat == ERR_SAVE_TORQUE_SLOPE) return ERR_SAVE_TORQUE_SLOPE_TITLE;
    else if (dat == ERR_SAVE_TORQUE_VELOCITY) return ERR_SAVE_TORQUE_VELOCITY_TITLE;
    else if (dat == ERR_SAVE_CURRENT) return ERR_SAVE_CURRENT_TITLE;
    else if (dat == ERR_SAVE_TORQUE_VELOCITY_MAKEUP) return ERR_SAVE_TORQUE_VELOCITY_MAKEUP_TITLE;
    else if (dat == ERR_SAVE_HMT_ACTIVATE) return ERR_SAVE_HMT_ACTIVATE_TITLE;
    else if (dat == ERR_SAVE_HMT_CONTROL) return ERR_SAVE_HMT_CONTROL_TITLE;

    else if (dat == ERR_MOTOR_PAUSE) return ERR_MOTOR_PAUSE_TITLE;
    else if (dat == ERR_MOTOR_RUN) return ERR_MOTOR_RUN_TITLE;
    else if (dat == ERR_MOTOR_OFF) return ERR_MOTOR_OFF_TITLE;
    else if (dat == ERR_MOTOR_FORWARD) return ERR_MOTOR_FORWARD_TITLE;
    else if (dat == ERR_MOTOR_BACKWARD) return ERR_MOTOR_BACKWARD_TITLE;
    else if (dat == ERR_MOTOR_LOW_VOLTAGE) return ERR_MOTOR_LOW_VOLTAGE_TITLE;

    else return DEFAULT;
}

static char* errgen_get_content(UNS16 dat) {
    if (dat == ERR_DRIVER_UP) return ERR_DRIVER_UP_CONTENT;
    else if (dat == ERR_DRIVER_LOAD) return ERR_DRIVER_LOAD_CONTENT;
    else if (dat == ERR_DRIVER_OPEN) return ERR_DRIVER_OPEN_CONTENT;
    else if (dat == ERR_INIT_LOOP_RUN) return ERR_INIT_LOOP_RUN_CONTENT;
    else if (dat == ERR_GUI_LOOP_RUN) return ERR_GUI_LOOP_RUN_CONTENT;

    else if (dat == ERR_FILE_OPEN) return ERR_FILE_OPEN_CONTENT;
    else if (dat == ERR_FILE_EMPTY) return ERR_FILE_EMPTY_CONTENT;
    else if (dat == ERR_FILE_GEN) return ERR_FILE_GEN_CONTENT;
    else if (dat == ERR_FILE_SLAVE_DEF) return ERR_FILE_SLAVE_DEF_CONTENT;
    else if (dat == ERR_FILE_PROFILE) return ERR_FILE_PROFILE_CONTENT;
    else if (dat == ERR_FILE_PROFILE_INVALID_PARAM) return ERR_FILE_PROFILE_INVALID_PARAM_CONTENT;


    else if (dat == ERR_MASTER_STOP_SYNC) return ERR_MASTER_STOP_SYNC_CONTENT;
    else if (dat == ERR_MASTER_START_SYNC) return ERR_MASTER_START_SYNC_CONTENT;
    else if (dat == ERR_MASTER_SET_PERIOD) return ERR_MASTER_SET_PERIOD_CONTENT;
    else if (dat == ERR_MASTER_SET_HB_CONS) return ERR_MASTER_SET_HB_CONS_CONTENT;
    else if (dat == ERR_MASTER_CONFIG_PDOR) return ERR_MASTER_CONFIG_PDOR_CONTENT;
    else if (dat == ERR_MASTER_CONFIG_PDOT) return ERR_MASTER_CONFIG_PDOT_CONTENT;
    else if (dat == ERR_MASTER_CONFIG) return ERR_MASTER_CONFIG_CONTENT;

    else if (dat == ERR_SLAVE_CONFIG) return ERR_SLAVE_CONFIG_CONTENT;
    else if (dat == ERR_SLAVE_CONFIG_LSS) return ERR_SLAVE_CONFIG_LSS_CONTENT;
    else if (dat == ERR_SLAVE_CONFIG_HB) return ERR_SLAVE_CONFIG_HB_CONTENT;
    else if (dat == ERR_SLAVE_CONFIG_PDOT) return ERR_SLAVE_CONFIG_PDOT_CONTENT;
    else if (dat == ERR_SLAVE_CONFIG_PDOR) return ERR_SLAVE_CONFIG_PDOR_CONTENT;
    else if (dat == ERR_SLAVE_CONFIG_ACTIVE_PDO) return ERR_SLAVE_CONFIG_ACTIVE_PDO_CONTENT;
    else if (dat == ERR_SLAVE_CONFIG_CURRENT) return ERR_SLAVE_CONFIG_CURRENT_CONTENT;
    else if (dat == ERR_SLAVE_CONFIG_PROFILE) return ERR_SLAVE_CONFIG_PROFILE_CONTENT;
    else if (dat == ERR_SLAVE_CONFIG_MAX_VELOCITY) return ERR_SLAVE_CONFIG_MAX_VELOCITY_CONTENT;
    else if (dat == ERR_SLAVE_CONFIG_ACCELERATION) return ERR_SLAVE_CONFIG_ACCELERATION_CONTENT;
    else if (dat == ERR_SLAVE_CONFIG_DECELERATION) return ERR_SLAVE_CONFIG_DECELERATION_CONTENT;
    else if (dat == ERR_SLAVE_CONFIG_DECELERATION_QS) return ERR_SLAVE_CONFIG_DECELERATION_QS_CONTENT;
    else if (dat == ERR_SLAVE_CONFIG_TORQUE) return ERR_SLAVE_CONFIG_TORQUE_CONTENT;
    else if (dat == ERR_SLAVE_CONFIG_TORQUE_SLOPE) return ERR_SLAVE_CONFIG_TORQUE_SLOPE_CONTENT;
    else if (dat == ERR_SLAVE_CONFIG_TORQUE_MAX_VELOCITY) return ERR_SLAVE_CONFIG_TORQUE_MAX_VELOCITY_CONTENT;
    else if (dat == ERR_SLAVE_CONFIG_MOTOR_SON) return ERR_SLAVE_CONFIG_MOTOR_SON_CONTENT;
    else if (dat == ERR_SLAVE_SAVE_CONFIG) return ERR_SLAVE_SAVE_CONFIG_CONTENT;
    else if (dat == ERR_SLAVE_LOAD_CONFIG) return ERR_SLAVE_LOAD_CONFIG_CONTENT;
    else if (dat == ERR_SLAVE_LOAD_INTERFACE) return ERR_SLAVE_LOAD_INTERFACE_CONTENT;

    else if (dat == ERR_READ_ACCELERATION) return ERR_READ_ACCELERATION_CONTENT;
    else if (dat == ERR_READ_DECELERATION) return ERR_READ_DECELERATION_CONTENT;
    else if (dat == ERR_READ_DECELERATION_QS) return ERR_READ_DECELERATION_QS_CONTENT;
    else if (dat == ERR_READ_TORQUE) return ERR_READ_TORQUE_CONTENT;
    else if (dat == ERR_READ_TORQUE_SLOPE) return ERR_READ_TORQUE_SLOPE_CONTENT;
    else if (dat == ERR_READ_TORQUE_VELOCITY) return ERR_READ_TORQUE_VELOCITY_CONTENT;
    else if (dat == ERR_READ_CURRENT) return ERR_READ_CURRENT_CONTENT;
    else if (dat == ERR_READ_PROFILE) return ERR_READ_PROFILE_CONTENT;
    else if (dat == ERR_READ_TORQUE_VELOCITY_MAKEUP) return ERR_READ_TORQUE_VELOCITY_MAKEUP_CONTENT;
    else if (dat == ERR_READ_HMT_CONTROL) return ERR_READ_HMT_CONTROL_CONTENT;

    else if (dat == ERR_SET_ACCELERATION) return ERR_SET_ACCELERATION_CONTENT;
    else if (dat == ERR_SET_DECELERATION) return ERR_SET_DECELERATION_CONTENT;
    else if (dat == ERR_SET_DECELERATION_QS) return ERR_SET_DECELERATION_QS_CONTENT;
    else if (dat == ERR_SET_VELOCITY_INC) return ERR_SET_VELOCITY_INC_CONTENT;
    else if (dat == ERR_SET_TORQUE) return ERR_SET_TORQUE_CONTENT;
    else if (dat == ERR_SET_TORQUE_SLOPE) return ERR_SET_TORQUE_SLOPE_CONTENT;
    else if (dat == ERR_SET_TORQUE_VELOCITY) return ERR_SET_TORQUE_VELOCITY_CONTENT;
    else if (dat == ERR_SET_CURRENT) return ERR_SET_CURRENT_CONTENT;
    else if (dat == ERR_SET_PROFILE) return ERR_SET_PROFILE_CONTENT;
    else if (dat == ERR_SET_TORQUE_VELOCITY) return ERR_SET_TORQUE_VELOCITY_CONTENT;
    else if (dat == ERR_SET_PDO) return ERR_SET_PDO_CONTENT;
    else if (dat == ERR_SET_TORQUE_VELOCITY_MAKEUP) return ERR_SET_TORQUE_VELOCITY_MAKEUP_CONTENT;
    else if (dat == ERR_SET_HMT_CONTROL) return ERR_SET_HMT_CONTROL_CONTENT;

    else if (dat == ERR_SAVE_ACCELERATION) return ERR_SAVE_ACCELERATION_CONTENT;
    else if (dat == ERR_SAVE_DECELERATION) return ERR_SAVE_DECELERATION_CONTENT;
    else if (dat == ERR_SAVE_DECELERATION_QS) return ERR_SAVE_DECELERATION_QS_CONTENT;
    else if (dat == ERR_SAVE_TORQUE) return ERR_SAVE_TORQUE_CONTENT;
    else if (dat == ERR_SAVE_TORQUE_SLOPE) return ERR_SAVE_TORQUE_SLOPE_CONTENT;
    else if (dat == ERR_SAVE_TORQUE_VELOCITY) return ERR_SAVE_TORQUE_VELOCITY_CONTENT;
    else if (dat == ERR_SAVE_CURRENT) return ERR_SAVE_CURRENT_CONTENT;
    else if (dat == ERR_SAVE_TORQUE_VELOCITY_MAKEUP) return ERR_SAVE_TORQUE_VELOCITY_MAKEUP_CONTENT;
    else if (dat == ERR_SAVE_HMT_CONTROL) return ERR_SAVE_HMT_CONTROL_CONTENT;

    else if (dat == ERR_MOTOR_PAUSE) return ERR_MOTOR_PAUSE_CONTENT;
    else if (dat == ERR_MOTOR_RUN) return ERR_MOTOR_RUN_CONTENT;
    else if (dat == ERR_MOTOR_OFF) return ERR_MOTOR_OFF_CONTENT;
    else if (dat == ERR_MOTOR_FORWARD) return ERR_MOTOR_FORWARD_CONTENT;
    else if (dat == ERR_MOTOR_BACKWARD) return ERR_MOTOR_BACKWARD_CONTENT;
    else if (dat == ERR_MOTOR_LOW_VOLTAGE) return ERR_MOTOR_LOW_VOLTAGE_CONTENT;

    else return DEFAULT;
}
