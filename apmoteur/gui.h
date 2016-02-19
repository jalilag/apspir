#ifndef _GUI_H
#define _GUI_H
#include "canfestival.h"
#include <gtk/gtk.h>

//typedef struct GUIELEM GUIELEM;
//struct GUIELEM {
//    char* id;
//    char* title;
//};

void gui_label_set(gchar* lab_id, gchar* lab_title);
void gui_label_set_objs(char* lab_title, char* lab_id, ... );
void gui_image_set(char* img_id, char* icon_name, int iconsize);
void gui_button_set(char* but_txt, char* labtxt, char* icon_name);
void gui_entry_set(gchar* ent_id, char* labtxt);

void gui_local_label_set(gchar* lab_id, gchar* lab_title, gchar* par_id);
void gui_local_image_set(gchar* img_id, char* icon_name, int iconsize,char* par_id);

int gui_switch_is_active(char* labid);
const gchar* gui_entry_get(gchar* ent_id);
void gui_push_state(char * txtState);
int but_is_checked(char* labid); // Savoir si un bouton est coché
gint64 gui_str2num(const gchar *txtdata); // Convertit str en double
double gui_str2double(const gchar *txtdata); // Convertit str en double
void gui_widget2show(char* s1,...); // Affiche les widget en arg
void gui_widget2hide(char* s1,...);
void gui_info_box(char* boxTitle, char* boxContent, char* icon);
void gui_info_popup(char* boxContent, char* icon);
void gui_init();
gpointer gui_main(gpointer data);
void gui_gen_param (gchar* labID, gchar* entID, gchar* labtxt,GtkGrid* grid, int gridLine);
GtkWidget* gui_local_get_widget(GtkWidget* parent, const gchar* name);
void gui_local_entry_set(gchar * parentID, gchar * entID, gchar* labtxt);
GtkGrid* gui_local_grid_set(gchar* gridID,char* gridTitle,gint numcol,char* color);
GtkBox* gui_get_box(gchar* labid);
GtkWidget* gui_create_widget(gchar* type,gchar* labid, gchar* labtxt, ...);
int gui_spinner_is_active(char* labid);
//void motor_running();

GObject* gui_get_object(gchar* labid); // Renvoie l'objet gtk
GtkWindow* gui_get_window(gchar* labid);
GtkWidget* gui_get_widget(gchar* labid);
GtkLabel* gui_get_label(gchar* labid);
GtkButton* gui_get_button(gchar* labid);
GtkImage* gui_get_image(gchar* labid);
GtkEntry* gui_get_entry(gchar* labid);
GtkMenu* gui_get_menu(gchar* labid);
GtkDialog* gui_get_dialog(gchar* labid);
GtkSwitch* gui_get_switch(gchar* labid);
GtkGrid* gui_get_grid(gchar* labid);
GtkSpinner* gui_get_spinner(gchar* labid);
GtkAdjustment* gui_get_adjust(gchar* labid);
#endif
