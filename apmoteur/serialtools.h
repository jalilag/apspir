
#include <gtk/gtk.h>
#include <glib.h>
extern int serialtools_in_reinit_laser;
extern int err_exit_laser;
gboolean serialtools_init_laser(gpointer err_init);
gboolean serialtools_checkPlotState_laser(gpointer err_check);
gboolean serialtools_check_laser_500ms(gpointer data);
gboolean serialtools_exit_laser(gpointer err_exit);
int serialtools_reinit_laser(void);
