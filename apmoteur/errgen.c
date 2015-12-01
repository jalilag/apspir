#include "errgen.h"
#include <stdlib.h>
#include "gui.h"
#include "keyword.h"
#include "strtools.h"

UNS16 errgen_state = 0x0000;

void errgen_set(UNS8 dat) {
    errgen_state = dat;
    errgen_catch_err(dat);
}

static void errgen_catch_err(UNS8 dat) {
    char* title = strtools_concat(ERROR, " : ", strtools_gnum2str(&dat,0x05)," ",errgen_get_title(dat), NULL);
    char* content = strtools_concat(errgen_get_content(dat),"\n",ERROR, " : ", strtools_gnum2str(&dat,0x05),NULL);
    if (dat == ERR_GUI_LOOP_RUN) {
        printf("%s",title);
        printf("\n");
        printf("%s",content);
        exit(EXIT_FAILURE);
    }

    if (
        dat == ERR_DRIVER_UP ||
        dat == ERR_DRIVER_LOAD ||
        dat == ERR_DRIVER_OPEN ||
        dat == ERR_FILE_CONFIG_GEN ||
        dat == ERR_INIT_LOOP_RUN ||
        dat == ERR_MASTER_CONFIG ||
        dat == ERR_SLAVE_CONFIG ||
        dat == ERR_LSS_CONFIG ||
        dat == ERR_SLAVE_CONFIG_MOTOR_SON
    ) {
        content = strtools_concat(content,"\n\n",APPLICATION_SHUTDOWN,NULL);
        gui_info_box(title,content,NULL);
        Exit();
    } else
    gui_info_box(title,content,NULL);

}



static char* errgen_get_title(UNS8 dat) {
    if (dat == ERR_DRIVER_UP) return ERR_DRIVER_UP_TITLE;
    if (dat == ERR_DRIVER_LOAD) return ERR_DRIVER_LOAD_TITLE;
    if (dat == ERR_DRIVER_OPEN) return ERR_DRIVER_OPEN_TITLE;
    if (dat == ERR_FILE_CONFIG_GEN) return ERR_FILE_CONFIG_GEN_TITLE;
    if (dat == ERR_INIT_LOOP_RUN) return ERR_INIT_LOOP_RUN_TITLE;
    if (dat == ERR_GUI_LOOP_RUN) return ERR_GUI_LOOP_RUN_TITLE;
    if (dat == ERR_MASTER_STOP_SYNC) return ERR_MASTER_STOP_SYNC_TITLE;
    if (dat == ERR_MASTER_START_SYNC) return ERR_MASTER_START_SYNC_TITLE;
    if (dat == ERR_MASTER_SET_PERIOD) return ERR_MASTER_SET_PERIOD_TITLE;
    if (dat == ERR_MASTER_SET_HB_CONS) return ERR_MASTER_SET_HB_CONS_TITLE;
    if (dat == ERR_MASTER_CONFIG_PDOR1) return ERR_MASTER_CONFIG_PDOR1_TITLE;
    if (dat == ERR_MASTER_CONFIG_PDOR2) return ERR_MASTER_CONFIG_PDOR2_TITLE;
    if (dat == ERR_MASTER_CONFIG_PDOR3) return ERR_MASTER_CONFIG_PDOR3_TITLE;
    if (dat == ERR_MASTER_CONFIG_PDOT1) return ERR_MASTER_CONFIG_PDOT1_TITLE;
    if (dat == ERR_MASTER_CONFIG) return ERR_MASTER_CONFIG_TITLE;
    if (dat == ERR_LSS_CONFIG) return ERR_LSS_CONFIG_TITLE;
    if (dat == ERR_SLAVE_CONFIG) return ERR_SLAVE_CONFIG_TITLE;
    if (dat == ERR_SLAVE_CONFIG_HB) return ERR_SLAVE_CONFIG_HB_TITLE;
    if (dat == ERR_SLAVE_CONFIG_PDOT1) return ERR_SLAVE_CONFIG_PDOT1_TITLE;
    if (dat == ERR_SLAVE_CONFIG_PDOT2) return ERR_SLAVE_CONFIG_PDOT2_TITLE;
    if (dat == ERR_SLAVE_CONFIG_PDOT3) return ERR_SLAVE_CONFIG_PDOT3_TITLE;
    if (dat == ERR_SLAVE_CONFIG_PDOR1) return ERR_SLAVE_CONFIG_PDOR1_TITLE;
    if (dat == ERR_SLAVE_CONFIG_ACTIVE_PDO) return ERR_SLAVE_CONFIG_ACTIVE_PDO_TITLE;
    if (dat == ERR_SLAVE_CONFIG_CURRENT) return ERR_SLAVE_CONFIG_CURRENT_TITLE;
    if (dat == ERR_SLAVE_CONFIG_PROFILE) return ERR_SLAVE_CONFIG_PROFILE_TITLE;
    if (dat == ERR_SLAVE_CONFIG_MAX_VELOCITY) return ERR_SLAVE_CONFIG_MAX_VELOCITY_TITLE;
    if (dat == ERR_SLAVE_CONFIG_ACCELERATION) return ERR_SLAVE_CONFIG_ACCELERATION_TITLE;
    if (dat == ERR_SLAVE_CONFIG_DECELERATION) return ERR_SLAVE_CONFIG_DECELERATION_TITLE;
    if (dat == ERR_SLAVE_CONFIG_DECELERATION_QS) return ERR_SLAVE_CONFIG_DECELERATION_QS_TITLE;
    if (dat == ERR_SLAVE_CONFIG_TORQUE) return ERR_SLAVE_CONFIG_TORQUE_TITLE;
    if (dat == ERR_SLAVE_CONFIG_TORQUE_SLOPE) return ERR_SLAVE_CONFIG_TORQUE_SLOPE_TITLE;
    if (dat == ERR_SLAVE_CONFIG_TORQUE_MAX_VELOCITY) return ERR_SLAVE_CONFIG_TORQUE_MAX_VELOCITY_TITLE;
    if (dat == ERR_SLAVE_CONFIG_MOTOR_SON) return ERR_SLAVE_CONFIG_MOTOR_SON_TITLE;
    if (dat == ERR_SLAVE_SAVE_CONFIG) return ERR_SLAVE_SAVE_CONFIG_TITLE;
    if (dat == ERR_MOTOR_PAUSE) return ERR_MOTOR_PAUSE_TITLE;
    if (dat == ERR_MOTOR_RUN) return ERR_MOTOR_RUN_TITLE;
    if (dat == ERR_MOTOR_OFF) return ERR_MOTOR_OFF_TITLE;
    if (dat == ERR_LOAD_ACCELERATION) return ERR_LOAD_ACCELERATION_TITLE;
    if (dat == ERR_LOAD_DECELERATION) return ERR_LOAD_DECELERATION_TITLE;
    if (dat == ERR_LOAD_DECELERATION_QS) return ERR_LOAD_DECELERATION_QS_TITLE;
    if (dat == ERR_LOAD_SLAVE_CONFIG) return ERR_LOAD_SLAVE_CONFIG_TITLE;
    if (dat == ERR_MOTOR_FORWARD) return ERR_MOTOR_FORWARD_TITLE;
    if (dat == ERR_MOTOR_BACKWARD) return ERR_MOTOR_BACKWARD_TITLE;
    if (dat == ERR_ACCELERATION_SET) return ERR_ACCELERATION_SET_TITLE;
    if (dat == ERR_VELOCITY_SET) return ERR_VELOCITY_SET_TITLE;
    if (dat == ERR_ACCELERATION_SAVE) return ERR_ACCELERATION_SAVE_TITLE;
    if (dat == ERR_DECELERATION_SAVE) return ERR_DECELERATION_SAVE_TITLE;
    if (dat == ERR_DECELERATION_QS_SAVE) return ERR_DECELERATION_QS_SAVE_TITLE;
    if (dat == ERR_SAVE_FILE_GEN) return ERR_SAVE_FILE_GEN_TITLE;
}

static char* errgen_get_content(UNS8 dat) {
    if (dat == ERR_DRIVER_UP) return ERR_DRIVER_UP_CONTENT;
    if (dat == ERR_DRIVER_LOAD) return ERR_DRIVER_LOAD_CONTENT;
    if (dat == ERR_DRIVER_OPEN) return ERR_DRIVER_OPEN_CONTENT;
    if (dat == ERR_FILE_CONFIG_GEN) return ERR_FILE_CONFIG_GEN_CONTENT;
    if (dat == ERR_INIT_LOOP_RUN) return ERR_INIT_LOOP_RUN_CONTENT;
    if (dat == ERR_GUI_LOOP_RUN) return ERR_GUI_LOOP_RUN_CONTENT;
    if (dat == ERR_MASTER_STOP_SYNC) return ERR_MASTER_STOP_SYNC_CONTENT;
    if (dat == ERR_MASTER_START_SYNC) return ERR_MASTER_START_SYNC_CONTENT;
    if (dat == ERR_MASTER_SET_PERIOD) return ERR_MASTER_SET_PERIOD_CONTENT;
    if (dat == ERR_MASTER_SET_HB_CONS) return ERR_MASTER_SET_HB_CONS_CONTENT;
    if (dat == ERR_MASTER_CONFIG_PDOR1) return ERR_MASTER_CONFIG_PDOR1_CONTENT;
    if (dat == ERR_MASTER_CONFIG_PDOR2) return ERR_MASTER_CONFIG_PDOR2_CONTENT;
    if (dat == ERR_MASTER_CONFIG_PDOR3) return ERR_MASTER_CONFIG_PDOR3_CONTENT;
    if (dat == ERR_MASTER_CONFIG_PDOT1) return ERR_MASTER_CONFIG_PDOT1_CONTENT;
    if (dat == ERR_MASTER_CONFIG) return ERR_MASTER_CONFIG_CONTENT;
    if (dat == ERR_LSS_CONFIG) return ERR_LSS_CONFIG_CONTENT;
    if (dat == ERR_SLAVE_CONFIG) return ERR_SLAVE_CONFIG_CONTENT;
    if (dat == ERR_SLAVE_CONFIG_HB) return ERR_SLAVE_CONFIG_HB_CONTENT;
    if (dat == ERR_SLAVE_CONFIG_PDOT1) return ERR_SLAVE_CONFIG_PDOT1_CONTENT;
    if (dat == ERR_SLAVE_CONFIG_PDOT2) return ERR_SLAVE_CONFIG_PDOT2_CONTENT;
    if (dat == ERR_SLAVE_CONFIG_PDOT3) return ERR_SLAVE_CONFIG_PDOT3_CONTENT;
    if (dat == ERR_SLAVE_CONFIG_PDOR1) return ERR_SLAVE_CONFIG_PDOR1_CONTENT;
    if (dat == ERR_SLAVE_CONFIG_ACTIVE_PDO) return ERR_SLAVE_CONFIG_ACTIVE_PDO_CONTENT;
    if (dat == ERR_SLAVE_CONFIG_CURRENT) return ERR_SLAVE_CONFIG_CURRENT_CONTENT;
    if (dat == ERR_SLAVE_CONFIG_PROFILE) return ERR_SLAVE_CONFIG_PROFILE_CONTENT;
    if (dat == ERR_SLAVE_CONFIG_MAX_VELOCITY) return ERR_SLAVE_CONFIG_MAX_VELOCITY_CONTENT;
    if (dat == ERR_SLAVE_CONFIG_ACCELERATION) return ERR_SLAVE_CONFIG_ACCELERATION_CONTENT;
    if (dat == ERR_SLAVE_CONFIG_DECELERATION) return ERR_SLAVE_CONFIG_DECELERATION_CONTENT;
    if (dat == ERR_SLAVE_CONFIG_DECELERATION_QS) return ERR_SLAVE_CONFIG_DECELERATION_QS_CONTENT;
    if (dat == ERR_SLAVE_CONFIG_TORQUE) return ERR_SLAVE_CONFIG_TORQUE_CONTENT;
    if (dat == ERR_SLAVE_CONFIG_TORQUE_SLOPE) return ERR_SLAVE_CONFIG_TORQUE_SLOPE_CONTENT;
    if (dat == ERR_SLAVE_CONFIG_TORQUE_MAX_VELOCITY) return ERR_SLAVE_CONFIG_TORQUE_MAX_VELOCITY_CONTENT;
    if (dat == ERR_SLAVE_CONFIG_MOTOR_SON) return ERR_SLAVE_CONFIG_MOTOR_SON_CONTENT;
    if (dat == ERR_SLAVE_SAVE_CONFIG) return ERR_SLAVE_SAVE_CONFIG_CONTENT;
    if (dat == ERR_MOTOR_PAUSE) return ERR_MOTOR_PAUSE_CONTENT;
    if (dat == ERR_MOTOR_RUN) return ERR_MOTOR_RUN_CONTENT;
    if (dat == ERR_MOTOR_OFF) return ERR_MOTOR_OFF_CONTENT;
    if (dat == ERR_LOAD_ACCELERATION) return ERR_LOAD_ACCELERATION_CONTENT;
    if (dat == ERR_LOAD_DECELERATION) return ERR_LOAD_DECELERATION_CONTENT;
    if (dat == ERR_LOAD_DECELERATION_QS) return ERR_LOAD_DECELERATION_QS_CONTENT;
    if (dat == ERR_LOAD_SLAVE_CONFIG) return ERR_LOAD_SLAVE_CONFIG_CONTENT;
    if (dat == ERR_MOTOR_FORWARD) return ERR_MOTOR_FORWARD_CONTENT;
    if (dat == ERR_MOTOR_BACKWARD) return ERR_MOTOR_BACKWARD_CONTENT;
    if (dat == ERR_ACCELERATION_SET) return ERR_ACCELERATION_SET_CONTENT;
    if (dat == ERR_VELOCITY_SET) return ERR_VELOCITY_SET_CONTENT;
    if (dat == ERR_ACCELERATION_SAVE) return ERR_ACCELERATION_SAVE_CONTENT;
    if (dat == ERR_DECELERATION_SAVE) return ERR_DECELERATION_SAVE_CONTENT;
    if (dat == ERR_DECELERATION_QS_SAVE) return ERR_DECELERATION_QS_SAVE_CONTENT;
    if (dat == ERR_SAVE_FILE_GEN) return ERR_SAVE_FILE_GEN_CONTENT;
}

