#ifndef _SLAVE_H
#define _SLAVE_H
#include "SpirallingControl.h"
#include "data.h"
#include <gtk/gtk.h>

typedef struct SLAVES_conf SLAVES_conf;
struct SLAVES_conf {
    int profile;
    char* title;
	UNS8 node;      // NodeId
	UNS32 vendor;   // Vendor ID
	UNS32 product;  // Product Code
	UNS32 revision; // Rev number
    UNS32 serial;   // Serial number
    int state;      // Etat global
    int state_error;// Erreur lors des passages d'état
    int active;
};

typedef struct PARVAR PARVAR;
struct PARVAR {
    char* id;
    UNS8 type;
    void** tab;
    char* title;
};


UNS8 slave_get_node_with_index(int i);
UNS8 slave_get_node_with_profile(int profInd);
int slave_get_index_with_node(UNS8 nodeID);

static int slave_get_state_with_index(int i);
static void slave_set_state_with_index(int i, int dat);
static int slave_get_state_error_with_index(int i);
static void slave_set_state_error_with_index (int i, int dat);
static char* slave_get_state_title(int state);
static char* slave_get_state_error_title(int state);
static UNS32 slave_get_vendor_with_index(int i);
static UNS32 slave_get_product_with_index(int i);
static UNS32 slave_get_revision_with_index(int i);
static UNS32 slave_get_serial_with_index(int i);
static int slave_get_active_with_index(int i);
static void slave_set_active_with_index(int i, int var);

int slave_get_profile_with_index(int i);
int slave_get_profile_with_node(UNS8 node);


char* slave_get_title_with_index(int i);
char* slave_get_title_with_node(UNS8 node);
char* slave_get_state_img (int state);
char* slave_get_profile_filename(int index);
char* slave_get_profile_name(int index);

int slave_config(UNS8 nodeID);
static int slave_config_com(UNS8 nodeID);
int slave_config_with_file(UNS8 node);
int slave_gui_param_gen(int ind);
int slave_save_param (int index);
gboolean slave_gui_load_state(gpointer data);
int slave_check_profile_file();
int slave_read_definition();

char* slave_get_param_in_char(char* parid, int index);
INTEGER32 slave_get_param_in_num(char* parid, int index);
void slave_set_param(char* parid, int index,INTEGER32);
char* slave_get_param_title (char* parid);

int slave_get_indexList_from_Profile(int Profile, int * indexList);
int slave_profile_exist(int profile);
#endif // _SLAVE_H
