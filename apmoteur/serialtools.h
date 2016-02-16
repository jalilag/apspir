#include <gtk/gtk.h>
#include <glib.h>
#include <time.h>
void serialtools_minimum_init(void);
void serialtools_plotLaserState(void);
unsigned int serialtools_init_laser(void);
void serialtools_exit_laser(void);
unsigned int serialtools_reinit_laser(void);
gpointer serialtools_init(gpointer data);
extern int serialtools_in_reinit_laser;
double serialtools_get_laser_data(void* las);
int serialtools_get_laser_data_valid(double* length, double* actTime);
