#ifndef _GTKSIG_H
#define _GTKSIG_H
#include <gtk/gtk.h>

void gtksig_init(); // Charge les données

void on_butStop_clicked (GtkWidget* pEntry); // Arret
void on_butInitDialClose_clicked (GtkWidget* pEntry); // Boite de dial

void on_butQuit_clicked (GtkWidget* pEntry); // Quitte appli
void on_butParams_clicked(GtkWidget* pEntry);
void on_butVelStart_active_notify(GtkWidget* pEntry);

void on_but_StartRot_notify(GtkWidget* pEntry);
void on_but_Start_Laser_clicked(GtkWidget* pEntry);
void on_but_Module_Laser_clicked(GtkWidget* pEntry);
void on_butReinitLaser_clicked(GtkWidget* pEntry);

void on_LaserSimuOpt_clicked(GtkWidget* pEntry);
void on_LaserSimuClose_clicked(GtkWidget* pEntry);


void on_LASERSTARTSIMU_clicked(GtkWidget* pEntry);
void on_LASERSTOPSIMU_clicked(GtkWidget* pEntry);

void on_setVelincSimuTrans_clicked(GtkWidget* pEntry);
void on_sim_inc_pos_clicked(GtkWidget* pEntry);
void on_sim_dec_pos_clicked(GtkWidget* pEntry);
void on_sim_inc_vel_clicked(GtkWidget* pEntry);
void on_sim_dec_vel_clicked(GtkWidget* pEntry);

void on_radBackward_toggled(GtkWidget* pEntry);
void on_radForward_toggled(GtkWidget* pEntry);
void on_butVelUp_clicked(GtkWidget* pEntry);
void on_butVelDown_clicked(GtkWidget* pEntry);

void on_butParamReturn_clicked(GtkWidget* pEntry);
void on_butParamSave_clicked(GtkWidget* pEntry);

void on_butDialYes_clicked(GtkWidget* pEntry);
void on_butDialNo_clicked(GtkWidget* pEntry);

void on_butAddSlave_clicked (GtkWidget* pEntry);
void on_butDelSlave_clicked(GtkWidget* pEntry);

void on_butParamMotor_clicked(GtkWidget* pEntry);
void on_butParamProfile_clicked(GtkWidget* pEntry);
void on_listProfile_changed(GtkWidget* pEntry);
void on_butAddField_clicked (GtkWidget* pEntry);
void on_butDelField_clicked (GtkWidget* pEntry);

void on_butActive_clicked (GtkWidget* pEntry,void* data);
void on_butFree_clicked(GtkWidget* pEntry,void* data) ;

#endif
