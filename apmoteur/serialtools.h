#include <gtk/gtk.h>
#include <glib.h>
void serialtools_minimum_init(void);
void serialtools_plotLaserState(void);
unsigned int serialtools_init_laser(void);
void serialtools_exit_laser(void);
unsigned int serialtools_reinit_laser(void);
extern int serialtools_in_reinit_laser;
