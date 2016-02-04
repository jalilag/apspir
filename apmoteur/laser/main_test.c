#include "Laser_Close_Cfile_gen.h"
#include "stdio.h"
int main(void)
{
  FILE * f;
  FILE * g;
  char * fname = "toto.c";
  char * fname2 = "toto2.c";
  f = fopen(fname, "w");
  g = fopen(fname2, "w");

  fprintf(f, "%s%d", LASER_CLOSE_CFILE, 0);
  fprintf(f, "%s", LASER_CLOSE_CFILE_P2);
  fprintf(g, "%s%d", LASER_CLOSE_CFILE, 1);
  fprintf(g, "%s", LASER_CLOSE_CFILE_P2);
  fclose(f);
  fclose(g);
}
