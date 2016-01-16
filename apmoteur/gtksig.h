#ifndef _GTKSIG_H
#define _GTKSIG_H
#include <gtk/gtk.h>

void gtksig_init(); // Charge les données

void on_butStop_clicked (GtkWidget* pEntry); // Arret
void on_butInitDialClose_clicked (GtkWidget* pEntry); // Boite de dial
void on_butPopupClose_clicked (GtkWidget* pEntry);

void on_butQuit_clicked (GtkWidget* pEntry); // Quitte appli
void on_butParams_clicked(GtkWidget* pEntry);
void on_butVelStart_active_notify(GtkWidget* pEntry);
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
void on_butParamHelix_clicked(GtkWidget* pEntry);
void on_listProfile_changed(GtkWidget* pEntry);
void on_butAddField_clicked (GtkWidget* pEntry);
void on_butDelField_clicked (GtkWidget* pEntry);

void on_butActive_clicked (GtkWidget* pEntry,void* data);
void on_butFree_clicked(GtkWidget* pEntry,void* data) ;

void on_butAddStep_clicked(GtkWidget* pEntry);
void on_butDelStep_clicked(GtkWidget* pEntry);
void on_lengthDef_changed (GtkWidget* pEntry);

void on_butTransUp_clicked (GtkWidget* pEntry);
void on_butTransDown_clicked (GtkWidget* pEntry);
void on_butTransStop_clicked(GtkWidget* pEntry);
void on_butRotRight_clicked (GtkWidget* pEntry);
void on_butRotLeft_clicked (GtkWidget* pEntry);
#endif
