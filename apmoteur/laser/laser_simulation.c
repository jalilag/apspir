#include "SpirallingMaster.h"
//#include "laser.h"
#include "laser_asserv.h"
#include "slave.h"
#include "errgen.h"

#include "laser_simulation.h"

int laser_simu = 0;

static INTEGER32 vel_err = 0, const_err = 0;


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

  //Fixation artificielle Laser Start Position
  laser_asserv_GetStartPosition(&lsim, NULL);
  dat = (double)Start_Distance;

  if((fd = fopen(FileName, "w"))==NULL) {
    printf("ERROR opening Sim File\n");
    pthread_exit(NULL);
  }

  int index_trans;
    if(slave_get_indexList_from_Title("Translation", &index_trans)){
        if(index_trans<0){
            errgen_set(ERR_LASER_ASSERV_NOTRANSMOTDEFINED, NULL);
            laser_simu = 0;
            pthread_exit(NULL);
        }
    }

  t0 = GetDate_us();
  //cas pas de glissement
  while(laser_simu)
    {
      capt_v = slave_get_param_in_num("Velocity", index_trans);
      cons_v = slave_get_param_in_num("Vel2send", index_trans);

      t1 = GetDate_us();

      if(const_err){
       dat -= const_err;
       const_err = 0;
      }

      dat -= ((double)(capt_v+vel_err)*(t1-t0)/1000000)*Cst;

      printf("\nLASER_SIMU_THREAD_GET: dat = %d, time = %lu, t1-t0 = %lu, capt_v = %d, cons_v = %d\n", (INTEGER32)dat, t1, t1-t0, capt_v, cons_v);
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

  INTEGER32 CaptureVitesse_MotTrans, ConsigneVitesse_MotTrans, CaptureAccel_MotTrans;
  int index_trans;
    if(slave_get_indexList_from_Title("Translation", &index_trans)){
        if(index_trans<0){
            errgen_set(ERR_LASER_ASSERV_NOTRANSMOTDEFINED, NULL);
            laser_simu = 0;
            pthread_exit(NULL);
        }
    }
  ConsigneVitesse_MotTrans = slave_get_param_in_num("Vel2send", index_trans);
  CaptureVitesse_MotTrans = slave_get_param_in_num("Velocity", index_trans);

  t0 = GetDate_us();
  while(CaptureVitesse_MotTrans<ConsigneVitesse_MotTrans)
    {
      ConsigneVitesse_MotTrans = slave_get_param_in_num("Vel2send", index_trans);
      CaptureVitesse_MotTrans = slave_get_param_in_num("Velocity", index_trans);
      CaptureAccel_MotTrans = slave_get_param_in_num("Acceleration", index_trans);

      t1 = GetDate_us();
      if((CaptureVitesse_MotTrans+(INTEGER32)((double)CaptureAccel_MotTrans*(t1-t0)/1000000)) > ConsigneVitesse_MotTrans)
	{
	  CaptureVitesse_MotTrans = ConsigneVitesse_MotTrans;
	  break;
	}
      else
	CaptureVitesse_MotTrans += (INTEGER32)((double)CaptureAccel_MotTrans*(t1-t0)/1000000);

      LeaveMutex();
      t0 = t1;
      usleep(50000);
      EnterMutex();
    }

  LeaveMutex();
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
  INTEGER32 CaptureDecel_MotTrans,ConsigneVitesse_MotTrans, CaptureVitesse_MotTrans;
  unsigned long t0, t1;
  int index_trans;

    if(slave_get_indexList_from_Title("Translation", &index_trans)){
        if(index_trans<0){
            errgen_set(ERR_LASER_ASSERV_NOTRANSMOTDEFINED, NULL);
            laser_simu = 0;
            pthread_exit(NULL);
        }
    }
  ConsigneVitesse_MotTrans = slave_get_param_in_num("Vel2send", index_trans);
  CaptureVitesse_MotTrans = slave_get_param_in_num("Velocity", index_trans);

  t0 = GetDate_us();
  while(CaptureVitesse_MotTrans>ConsigneVitesse_MotTrans)
    {
      ConsigneVitesse_MotTrans = slave_get_param_in_num("Vel2send", index_trans);
      CaptureVitesse_MotTrans = slave_get_param_in_num("Velocity", index_trans);
      CaptureDecel_MotTrans = slave_get_param_in_num("Deceleration", index_trans);

      t1 = GetDate_us();
      if((CaptureVitesse_MotTrans-(INTEGER32)((double)CaptureDecel_MotTrans*(t1-t0)/1000000)) < ConsigneVitesse_MotTrans)
	{
	  CaptureVitesse_MotTrans = ConsigneVitesse_MotTrans;
	  break;
	}
      else
	CaptureVitesse_MotTrans -= (INTEGER32)((double)CaptureDecel_MotTrans*(t1-t0)/1000000);

      //printf("CAPTURE VITESSE T = %d\n", CaptureVitesse_MotTrans);

      t0 = t1;
      usleep(50000);
    }

  LeaveMutex();

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

