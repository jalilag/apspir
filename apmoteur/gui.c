#include "gui.h"
#include <stdio.h>
#include <string.h>
#include <stdlib.h>
#include <glib.h>
#include <gtk/gtk.h>
#include "strtools.h"
#include "keyword.h"
#include "SpirallingControl.h"

GtkBuilder *builder;
extern GMutex lock_gui;
extern int run_gui_loop;

GObject* gui_get_object(gchar* labid) {
    return G_OBJECT(gtk_builder_get_object (builder, labid));
}
GtkWindow* gui_get_window(gchar* labid) {
    return GTK_WINDOW(gui_get_object(labid));
}
GtkWidget* gui_get_widget(gchar* labid) {
    return GTK_WIDGET(gui_get_object(labid));
}
GtkLabel* gui_get_label(gchar* labid) {
    return GTK_LABEL(gui_get_object(labid));
}
GtkButton* gui_get_button(gchar* labid) {
    return GTK_BUTTON(gui_get_object(labid));
}
GtkImage* gui_get_image(gchar* labid) {
    return GTK_IMAGE(gui_get_object(labid));
}
GtkEntry* gui_get_entry(gchar* labid) {
    return GTK_ENTRY(gui_get_object(labid));
}
GtkMenu* gui_get_menu(gchar* labid) {
    return GTK_MENU(gui_get_object(labid));
}
GtkDialog* gui_get_dialog(gchar* labid) {
    return GTK_DIALOG(gui_get_object(labid));
}
GtkToggleButton* gui_get_toggle(gchar* labid) {
    return GTK_TOGGLE_BUTTON(gui_get_object(labid));
}
GtkSwitch* gui_get_switch(gchar* labid) {
    return GTK_SWITCH(gui_get_object(labid));
}
GtkGrid* gui_get_grid(gchar* labid) {
    return GTK_GRID(gui_get_object(labid));
}
GtkSpinner* gui_get_spinner(gchar* labid) {
    return GTK_SPINNER(gui_get_object(labid));
}
GtkBox* gui_get_box(gchar* labid) {
    return GTK_BOX(gui_get_object(labid));
}
GtkAdjustment* gui_get_adjust(gchar* labid) {
    return GTK_ADJUSTMENT(gui_get_object(labid));
}
GtkWidget* gui_local_get_widget(GtkWidget* parent, const gchar* name) {
    if (g_ascii_strncasecmp(gtk_widget_get_name(parent),name,-1) == 0) {
        return parent;
    }
    if (GTK_IS_BIN(parent)) {
        GtkWidget *child = gtk_bin_get_child(GTK_BIN(parent));
        gui_local_get_widget(child,name);
    }
    if (GTK_IS_CONTAINER(parent)) {
        GList *children = gtk_container_get_children(GTK_CONTAINER(parent));
        int i;
        for (i=0; i < g_list_length(children); i++) {
            GtkWidget* widget = gui_local_get_widget(g_list_nth_data(children,i),name);
            if (widget != NULL) {
                return widget;
            }
        }
    }
    return NULL;
}
void gui_label_set(gchar* lab_id, gchar* lab_title) {
    GtkLabel* lab = gui_get_label(lab_id);
    if (lab != NULL && lab_title != NULL)
        if (strlen(lab_title) > 0)
            if (gtk_label_get_text(lab) != lab_title) {
                gtk_label_set_text(lab, lab_title);
            }
}

void gui_local_label_set(gchar* lab_id, gchar* lab_title, gchar* par_id) {
    GtkWidget* lab = gui_local_get_widget(gui_get_widget(par_id),lab_id);
    if (lab != NULL && lab_title != NULL)
        if (strlen(lab_title) > 0)
            if (gtk_label_get_label(GTK_LABEL(lab)) != lab_title) {
                gtk_label_set_label(GTK_LABEL(lab), lab_title);
            }
}

void gui_label_set_objs(char* lab_title, gchar* lab_id, ... ) {
    gtk_label_set_text(gui_get_label(lab_id), lab_title);
    va_list ap;
    va_start(ap,lab_id);
    gchar *arg = va_arg(ap,gchar*);
    while ( arg != NULL) {
        if (gui_get_object(arg) != NULL) {
            gtk_label_set_text(gui_get_label(arg), lab_title);
        }
        arg = va_arg(ap,gchar*);
    }
    va_end(ap);
}

void gui_image_set(gchar* img_id, char* icon_name, int iconsize) {
    GtkImage* img = gui_get_image(img_id);
    if (img != NULL) {
        const gchar** icon_act;
        GtkIconSize* size_act;
        gtk_image_get_icon_name(img,icon_act,size_act);
        if (icon_act == NULL || (icon_act != NULL && (char*)*icon_act != icon_name)) {
            GtkIconSize i;
            switch(iconsize) {
                case 1:
                    i = GTK_ICON_SIZE_LARGE_TOOLBAR;
                break;
                case 2:
                    i = GTK_ICON_SIZE_DND;
                break;
                case 3:
                    i = GTK_ICON_SIZE_DIALOG;
                break;
                default:
                    i = GTK_ICON_SIZE_BUTTON;
                break;
            }
            gtk_image_set_from_icon_name (img,icon_name,i);
        }
    }
}
void gui_local_image_set(gchar* img_id, char* icon_name, int iconsize,char* par_id) {
    GtkWidget* img = gui_local_get_widget(gui_get_widget(par_id),img_id);
    if ( img != NULL && icon_name != NULL){
        const gchar** icon_act = NULL;
        GtkIconSize* size_act;
        gtk_image_get_icon_name(GTK_IMAGE(img),icon_act,size_act);
        if (icon_act == NULL || (icon_act != NULL && (char*)*icon_act != icon_name)) {
            GtkIconSize i;
            switch(iconsize) {
                case 1:
                    i = GTK_ICON_SIZE_LARGE_TOOLBAR;
                break;
                case 2:
                    i = GTK_ICON_SIZE_DND;
                break;
                case 3:
                    i = GTK_ICON_SIZE_DIALOG;
                break;
                default:
                    i = GTK_ICON_SIZE_BUTTON;
                break;
            }
            gtk_image_set_from_icon_name (GTK_IMAGE(img),icon_name,i);
        }
    }
}



void gui_button_set(gchar* but_id, char* labtxt, char* icon_name) {
    if (gui_get_object(but_id) != NULL) {
        gtk_button_set_label (gui_get_button(but_id), labtxt);
        if (icon_name!= NULL)
            gtk_button_set_image (gui_get_button(but_id),gtk_image_new_from_icon_name (icon_name,GTK_ICON_SIZE_BUTTON));
    }
}

void gui_entry_set(gchar* ent_id, char* labtxt) {
    if (gui_get_object(ent_id) != NULL) {
        gtk_entry_set_text(gui_get_entry(ent_id),labtxt);
    }
}

const gchar* gui_entry_get(gchar* ent_id) {
    return gtk_entry_get_text(gui_get_entry(ent_id));
}

void gui_push_state(gchar* txtState) {
    gchar* prev_txt = (gchar*)gtk_label_get_text (gui_get_label ("stateBarContent"));
    int N1 = strtools_num_char_in_str(prev_txt, "\n");
    int N2 = strtools_num_char_in_str(prev_txt, "\0");
    if (N1 == 0 && N2 == 0) {
        gui_label_set("stateBarContent",txtState);
    } else {
        if (N1 == 15) {
            char** tab = strtools_split(prev_txt,"\n",0);
            int i;
            prev_txt = tab[1];
            for (i=2; i<15;i++) {
                prev_txt = strtools_concat(prev_txt,"\n",tab[i],NULL);
            }
        }
        gui_label_set("stateBarContent",strtools_concat(txtState,"\n",prev_txt,NULL));
    }

}

int gui_but_is_checked(char* labid) {
    return gtk_toggle_button_get_active (GTK_TOGGLE_BUTTON(gtk_builder_get_object (builder, labid)));
}
int gui_switch_is_active(char* labid) {
    GtkSwitch* s=gui_get_switch(labid);
    if (s != NULL)
        return gtk_switch_get_active(s);
    else
        return -1;
}

gint64 gui_str2num(const gchar *txtdata) {
    return g_ascii_strtoll(txtdata,NULL,10);
}
double gui_str2double(const gchar *txtdata) {
    return g_strtod(txtdata,NULL);
}

void gui_widget2show(gchar* s1,...) {
    gtk_widget_show(gui_get_widget(s1));
    va_list ap;
    va_start(ap,s1);
    gchar *arg = va_arg(ap,gchar*);
    while ( arg != NULL) {
        gtk_widget_show(gui_get_widget(arg));
        arg = va_arg(ap,gchar*);
    }
    va_end(ap);
}

void gui_widget2hide(gchar* s1,...) {
    gtk_widget_hide(gui_get_widget(s1));
    va_list ap;
    va_start(ap,s1);
    gchar *arg = va_arg(ap,gchar*);
    while ( arg != NULL) {
        gtk_widget_hide(gui_get_widget(arg));
        arg = va_arg(ap,gchar*);
    }
    va_end(ap);
}

void gui_info_box(char* boxTitle, char* boxContent, char* icon) {
    gtk_window_set_title(gui_get_window("windowDialInit"), boxTitle);
    gui_label_set("labDialContent",boxContent);
    if (icon != NULL) {
        gui_image_set("labDialImg",icon,3);
    } else {
        gui_image_set("labDialImg","gtk-info",3);
    }
    gtk_dialog_run(gui_get_dialog("windowDialInit"));
}
void gui_info_popup(char* boxContent, char* icon) {
    gui_label_set("labPopupContent",boxContent);
    if (icon != NULL) {
        gui_image_set("labPopupImg",icon,3);
    } else {
        gui_image_set("labPopupImg","gtk-info",3);
    }
    gtk_dialog_run(gui_get_dialog("windowPopup"));
}
gpointer gui_main(gpointer data) {
    run_gui_loop =1;
    gtk_main();
    return 0;
}

void gui_init() {
    // Initialisation variable GTK
    GError *csserror = NULL;
    XInitThreads(); // A mettre avant gtk_init
    gtk_init (NULL, NULL);
    builder = gtk_builder_new (); // Variable globale

    // Import GLADE
    gtk_builder_add_from_file (builder, "XMLappli/Gappli_v2.glade", NULL);

    // CSS
    GFile * file = g_file_new_for_path ("./stylesheet.css");
    GtkCssProvider *cssprovider = gtk_css_provider_new ();
    if (!gtk_css_provider_load_from_file (cssprovider, file, &csserror)) {
        g_printerr ("%s\n", csserror->message);
        exit (csserror->code);
    }
    gtk_style_context_add_provider_for_screen (gdk_display_get_default_screen (gtk_widget_get_display (gui_get_widget("mainWindow"))), GTK_STYLE_PROVIDER (cssprovider), GTK_STYLE_PROVIDER_PRIORITY_USER);

    // Init
    gtksig_init();
    keyword_init();
    gtk_widget_show_all (gui_get_widget("mainWindow"));
}


GtkGrid* gui_local_grid_set(gchar* gridID,char* gridTitle,gint numcol,char* color) {
    GtkGrid* grid = GTK_GRID(gtk_grid_new());
    gtk_widget_set_name(GTK_WIDGET(grid),gridID);
    if (gridTitle != NULL) {
        GtkWidget* lab = gtk_label_new(gridTitle);
        gtk_grid_attach (grid, lab, 0, 0, numcol, 1);
        gtk_widget_set_hexpand(lab,TRUE);
        gtk_widget_set_halign(lab,GTK_ALIGN_FILL);
        if (color != NULL) {
            gtk_style_context_add_class (gtk_widget_get_style_context(lab), "h1");
            gtk_style_context_add_class (gtk_widget_get_style_context(lab), color);
        }
    }
    gtk_widget_set_halign(GTK_WIDGET(grid),GTK_ALIGN_FILL);
    gtk_style_context_add_class (gtk_widget_get_style_context(GTK_WIDGET(grid)), "box");
    gtk_style_context_add_class (gtk_widget_get_style_context(GTK_WIDGET(grid)), strtools_concat(color,"Box",NULL));
    gtk_widget_set_margin_top (GTK_WIDGET(grid),5);
    gtk_widget_set_margin_bottom (GTK_WIDGET(grid),0);
    gtk_widget_set_margin_right (GTK_WIDGET(grid),10);
    gtk_widget_set_margin_left (GTK_WIDGET(grid),10);
    return grid;
}



GtkWidget* gui_create_widget(gchar* type,gchar* labid, gchar* labtxt, ...) {
    GtkWidget* lab;
    if (type == "lab") lab = gtk_label_new(labtxt);
    else if (type == "img") lab = gtk_image_new_from_icon_name(labtxt,GTK_ICON_SIZE_DND);
    else if (type == "ent") {
        lab = gtk_entry_new();
        if (labtxt != NULL) gtk_entry_set_text(GTK_ENTRY(lab),labtxt);
    }
    else if (type == "combo") lab = gtk_combo_box_text_new();
    else if (type == "radio") lab = gtk_radio_button_new_with_label(NULL,labtxt);
    else if (type == "but") {
        lab = gtk_button_new_with_label(labtxt);
        gtk_button_set_always_show_image(GTK_BUTTON(lab),TRUE);
        gtk_style_context_add_class ( gtk_widget_get_style_context(lab), "stdButton");
        gtk_button_set_image_position (GTK_BUTTON(lab),GTK_POS_LEFT);
    }
    gtk_widget_set_name(lab, labid);
    va_list ap;
    va_start(ap,labtxt);
    gchar *arg = va_arg(ap,gchar*);
    while ( arg != NULL) {
        gtk_style_context_add_class ( gtk_widget_get_style_context(lab), arg);
        arg = va_arg(ap,gchar*);
    }
    va_end(ap);
    if (type == "ent" || type == "combo" || type == "but" ) {
        gtk_widget_set_margin_top (lab,5);
        gtk_widget_set_margin_bottom (lab,5);
        gtk_widget_set_margin_right (lab,5);
        gtk_widget_set_margin_left (lab,5);
    }

    return lab;
}

int gui_spinner_is_active(char* labid) {
    int res = -1;
    g_object_get(gui_get_object("chargement"),"active",&res,NULL);
    return res;
}
