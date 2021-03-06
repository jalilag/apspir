#include "SpirallingMaster.h"
//#include "laser.h"
#include "laser_asserv.h"
#include "slave.h"
#include "errgen.h"

#include "laser_simulation.h"

#define DEFAULT_START_DISTANCE 30000

int laser_simu = 0;
static INTEGER32 VelincSimuTrans = 0;
static INTEGER32 vel_err = 0, const_err = 0;

void laser_simulation_set_VelincSimuTrans(INTEGER32 val){
    VelincSimuTrans = val;
}
void laser_simulation_inc_const_err(INTEGER32 val){
    const_err+=val;
}
void laser_simulation_dec_const_err(INTEGER32 val){
    const_err-=val;
}
void laser_simulation_inc_vel_err(INTEGER32 val){
    vel_err+=val;
}
void laser_simulation_dec_vel_err(INTEGER32 val){
    vel_err-=val;
}
INTEGER32 laser_simulation_get_VelincSimuTrans(void){
    return VelincSimuTrans;
}
INTEGER32 laser_simulation_get_const_err(void){
    return const_err;
}
INTEGER32 laser_simulation_get_vel_err(void){
    return vel_err;
}
void * laser_simulation_SimuThread_func(void * arg)
{
  unsigned long t0, t1;
  INTEGER32 capt_v, cons_v;
  double dat;
  double Cst = (double)10000/STEPS_PER_REV/REDUCTION_TRANS;

  const char * FileName = "./laser/data_simu/GuiOutputData.dat";
  FILE * fd;

  Laser_Init_Simu(&lsim);

  dat = (double)DEFAULT_START_DISTANCE;

  if((fd = fopen(FileName, "w"))==NULL) {
    printf("ERROR opening Sim File\n");
    pthread_exit(NULL);
  }

  t0 = GetDate_us();
  //cas pas de glissement
  while(laser_simu)
    {
      capt_v = ConsVit_T;
      cons_v = CaptVit_T;

      t1 = GetDate_us();

      if(const_err){
       dat -= const_err;
       const_err = 0;
      }

      dat -= ((double)(capt_v+vel_err)*(t1-t0)/1000000)*Cst;

      //printf("\nLASER_SIMU_THREAD_GET: dat = %d, time = %lu, t1-t0 = %lu, capt_v = %d, cons_v = %d\n", (INTEGER32)dat, t1, t1-t0, capt_v, cons_v);
      fprintf(fd, "%lf %lu %lu %d %d\n", dat, t1, t1-t0, capt_v, cons_v);

      t0 = t1;
      Save_Data_Laser((laser*)&lsim, (INTEGER32)dat, t1);
      usleep(150000);
    }

    pthread_exit(NULL);

}

//uniquement pour la simulation moteur
void * on_ACCELERER_clicked_thread(void * ag)
{
  unsigned long t0, t1;

  t0 = GetDate_us();
  while(CaptVit_T<ConsVit_T)
    {
      t1 = GetDate_us();
      if((CaptVit_T+(INTEGER32)((double)CaptAccel_T*(t1-t0)/1000000)) > ConsVit_T)
	{
	  CaptVit_T = ConsVit_T;
	  break;
	}
      else
	CaptVit_T += (INTEGER32)((double)CaptAccel_T*(t1-t0)/1000000);

      //LeaveMutex();
      t0 = t1;
      usleep(50000);
      //EnterMutex();
    }

  //LeaveMutex();
  pthread_exit(NULL);
}

//uniquement pour la simu moteur
void laser_simulation_on_ACCELERER_clicked(void)
{
  pthread_t r_t;

  if(pthread_create(&r_t, NULL, on_ACCELERER_clicked_thread, NULL))
    {
      printf("ON ACCELERER CLICKED ERROR\n");
    }

}

//uniquement pour la simu moteur
void * on_RALENTIR_clicked_thread(void * arg)
{
  unsigned long t0, t1;

  t0 = GetDate_us();
  while(CaptVit_T>ConsVit_T)
    {
      t1 = GetDate_us();
      if((CaptVit_T-(INTEGER32)((double)CaptDecel_T*(t1-t0)/1000000)) < ConsVit_T)
	{
	  CaptVit_T = ConsVit_T;
	  break;
	}
      else
	CaptVit_T -= (INTEGER32)((double)CaptDecel_T*(t1-t0)/1000000);

      //printf("CAPTURE VITESSE T = %d\n", CaptVit_T);

      t0 = t1;
      usleep(50000);
    }

  //LeaveMutex();

  pthread_exit(NULL);
}

//uniquement pour la simu moteur
void laser_simulation_on_RALENTIR_clicked(void)
{

  pthread_t r_t;

  if(pthread_create(&r_t, NULL, on_RALENTIR_clicked_thread, NULL))
    {
      printf("ON RALENTIR CLICKED ERROR\n");
    }
}

