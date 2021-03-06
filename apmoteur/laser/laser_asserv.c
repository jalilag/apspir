#include "stdlib.h"
#include "stdio.h"
#include "math.h"//racine
#include "unistd.h"
#include "time.h"
#include "pthread.h"
#include "laser_asserv.h"

#include "SpirallingMaster.h"
#include "cantools.h"

#define STAND_ALONE_TEST 0
#if STAND_ALONE_TEST
struct Helix_User_Data HelixUserData;//simule la variable globale utilisateur
INTEGER32 MotRot_Ref_Position = 0;
#endif

int run_asserv = 0;

static struct laser_asserv_thread lats;

INTEGER32 MotRot_Ref_Position = 0;
unsigned int User_Tolerance = 100;//en dixieme de mm.
static unsigned long Start_Distance = 300000;
unsigned long Pipe_Length = 30000;

static FILE * laser_asserv_report_file;
static const char* laser_asserv_report_filename = "./laser/work_report/report_laserAsserv.dat";

unsigned long Laser_Asserv_GetStartDistance(void){
    return Start_Distance;
}

int d_to_dparcouru(struct laser_data * d)
{
    long int l = Start_Distance-d->mes;
    if(l <= -LASER_MES_ERROR) return 1;
    else if ((l<=0) && l >= -LASER_MES_ERROR) d->mes = 0;
    else d->mes = Start_Distance - (d->mes);

    d->vitesse = -d->vitesse;
    d->vitesse_1s = -d->vitesse_1s;

    return 0;
}

int laser_asserv_decalage_temp(unsigned long p_capt_time, struct laser_data * d){
    long int l = (d->vitesse_1s*( (long int)p_capt_time - (long int)d->t ))/1000000 + d->mes;
    if(l <= -LASER_MES_ERROR) return 1;
    else if ((l <= 0) && l >= -LASER_MES_ERROR) d->mes = 0;
    else d->mes = (unsigned long)l;//+= (d->vitesse_1s*( p_capt_time - d->t ))/1000000;
    d->t = p_capt_time;
    return 0;
}


unsigned int laser_asserv_GetPipeLength(laser * ml, laser * sl)
{
  unsigned int status_laser = 0;
  struct laser_data d;
  if(((status_laser = Laser_GetData(ml, sl, &d)) & ERR_LASER_FATAL)==ERR_LASER_FATAL)
    return ERR_LASER_FATAL;

  Pipe_Length = d.mes;
  return 0;
}

unsigned int laser_asserv_GetStartPosition(laser * ml, laser * sl)
{
  struct laser_data d;

  if(((Laser_GetData(ml, sl, &d)) & ERR_LASER_FATAL)==ERR_LASER_FATAL)
    return ERR_LASER_FATAL;

  Start_Distance = d.mes;
  return 0;
}

int Recup_NumInterval(unsigned long * d, long int * res)//d en ddixieme de mm
{
  long int i;
  long int NumInterval = 0;
  long int l = 0;
 //verifier que la distance n'est pas supérieures au max possible
  if(HelixUserData.dmax < (double)(*d)/10)
    {
      printf("Distance measured is higher than max user distance\n");
      return -1;
    }

  //trouver l'intervalle de distance dans lequel on se trouve
  for (i=0;i<HelixUserData.NbrOfEntries; i++)
    {
      l += HelixUserData.d[i];
      //printf("i = %ld\n", i);
      //printf("l = %lu\n", l);
      //printf("d/10 = %lf\n", (double)(*d)/10);
      if( (double)(*d)/10 <= l )
	{
	  NumInterval = i;
	  break;
	}

    }

  *res = NumInterval;
  //printf("NumInterval = %ld, l = %lu\n", NumInterval, l);
  return 0;
}

int FindConsigne_PosRot(unsigned long * d/*distance mesurée dmm*/, long int * Consigne)//res en steps
{
  long int i;
  long int NumInterval = 0;
  long int l = 0;
  long int PosRot = MotRot_Ref_Position;

  if(Recup_NumInterval(d, &NumInterval))
    return -1;

  l = 0;
  //s'il n'y a qu'un interval
  if(NumInterval == 0 && HelixUserData.p[0]>0)
     {
       PosRot += (long int)((double)((*d))*REDUCTION_ROTATION/((double)HelixUserData.p[NumInterval]*10/STEPS_PER_REV));
       *Consigne = PosRot;
       return 0;
     }
  else if (NumInterval == 0 && HelixUserData.p[0] == 0)
    {
      *Consigne = PosRot;
      return 0;
    }
  //si plus d'un interval trouver la pos départ consigne de notre intervalle
  for (i=0; i<NumInterval; i++)
    {
      if(HelixUserData.p[i] != 0)
	{
	  l += HelixUserData.d[i];
	  PosRot += (long int)((double)(HelixUserData.d[i])*REDUCTION_ROTATION/((double)HelixUserData.p[i]/STEPS_PER_REV));
	  //printf("PosRot = %lu\n", PosRot);
	}
      else if (HelixUserData.p[i] == 0)
	l += HelixUserData.d[i];
      else
	{
	  printf("Incorrect User Entries Helix data\n");
	  return -1;
	}
    }

  if(HelixUserData.p[NumInterval]!=0)
    PosRot += (long int)((double)(((double)(*d)/10-(double)l)*REDUCTION_ROTATION)/((double)HelixUserData.p[NumInterval]/STEPS_PER_REV));


  //printf("(*d)/10-l =  %lu, l = %lu", ((*d)/10-l), l);
  *Consigne = PosRot;

  if(labs(PosRot)>0x7FFFFFFF) {
    printf("FindConsigne PosRot ERROR: Calculated Position is too large for an int32\n");
    return 1;
  }

  return 0;
}

int FindConsigne_VitRot(unsigned long * d/*distance mesuree dmm*/, INTEGER32 * v_mtrans, INTEGER32 * res)
{

  long int NumInterval;
  long int p;

  if(Recup_NumInterval(d, &NumInterval))
    return -1;
  //printf("d = %lu, Numinterval = %ld\n", *d, NumInterval);
  p = HelixUserData.p[NumInterval]; //en mm/tour
  //printf("p = %ld\n", p);
  if(p == 0)
    *res = 0;
  else
    {
      *res = (INTEGER32)((*v_mtrans) * (double)REL_TRSL_ROT/p * (double)REDUCTION_ROTATION/REDUCTION_TRANS);
    }

  //printf("res = %d, p = %ld, v_mtrans = %f\n", *res, p, (float)REL_TRSL_ROT/p * (float)REDUCTION_ROTATION/REDUCTION_TRANS);
  return 0;
}
//
//pas encore testée
//v_ltrans en dmm/s
int FindConsigne_VitRot_with_laser(unsigned long * d/*distance mesuree dmm*/, long int * v_ltrans, INTEGER32 * res)
{

  long int NumInterval;
  long int p;

  if(Recup_NumInterval(d, &NumInterval))
    return -1;
  //printf("d = %lu, Numinterval = %ld\n", *d, NumInterval);
  p = HelixUserData.p[NumInterval]; //en mm/tour
  //printf("p = %ld\n", p);
  if(p == 0)
    *res = 0;
  else
    {
      *res = (INTEGER32)((*v_ltrans) * STEPS_PER_REV * REDUCTION_ROTATION / p / 10);
    }

  //printf("res = %d, p = %ld, v_mtrans = %f\n", *res, p, (float)REL_TRSL_ROT/p * (float)REDUCTION_ROTATION/REDUCTION_TRANS);
  return 0;
}

//v_cons_mrot en dixieme de mm/s
unsigned int FindTypeOfMvt(INTEGER32 v_cons_m, INTEGER32 v_capt_m)
{

      //printf("FINDTYPEOFMVT v_capt = %d, v_cons_m = %d, labs(v1-v2) = %d", v_capt_m, v_cons_m, labs(v_capt_m-v_cons_m));
      if((labs(v_capt_m)>labs(v_cons_m)+LASER_ASSERV_ERR_VITESSE_ROT) & ((long long int)v_capt_m*v_cons_m >= 0))
	return PHASE_DECEL;
      else if((labs(v_capt_m)<labs(v_cons_m)-LASER_ASSERV_ERR_VITESSE_ROT) & ((long long int)v_capt_m*v_cons_m >= 0))
	return PHASE_ACCEL;
      else if (labs(v_capt_m - v_cons_m) <= LASER_ASSERV_ERR_VITESSE_ROT)
	return VITESSE_CONSTANTE;
      else
	{
	  printf("ERROR FindTypeOfMvt\n");
	  return TYPE_OF_MVT_FAILURE;
	}

}

static struct Laser_x_thread vc;

void * Thread_Appli_Vitesse(void * StructConsigneVitesse)
{
  struct Laser_x_thread v = *(struct Laser_x_thread*)StructConsigneVitesse;

  printf("VITESSE APPLIQUEE -> %d\n",v.cv );
  cantools_ApplyVelRot(v.cv);

  usleep(v.time);

  printf("VITESSE APPLIQUEE -> %d\n",v.dv );
  cantools_ApplyVelRot(v.dv);

  pthread_exit(NULL);
}
//cp: consigne position en step
//p_capt_mrot: position motrot capturee en step
//v_old_mrot: vitesse motrot en step/s
//vc: sortie: struct d'asservissement (voir .h)
int Find_Asserv_Params( long int * cp, INTEGER32 * p_capt_mrot, double * dvit_corr, INTEGER32 * v_old_mrot, struct Laser_x_thread * vc)
{
  double a  = (double)ConsAccel_R;
  double d = (double)ConsDecel_R;
  double DeltaS = (double)labs(*cp - *p_capt_mrot);
  double dv = *dvit_corr;
  double t1;
  double res = 0;

  vc->dv = *v_old_mrot;

  if(*cp - *p_capt_mrot >= LASER_ASSERV_MOTROT_POS_INCERTITUDE){
    if( (*v_old_mrot)*(*v_old_mrot+dv) < 0 ){
        vc->cv = 0;
        dv = -(*v_old_mrot);
    } else {
        vc->cv = *v_old_mrot+dv;
    }
  }
  else if (*cp - *p_capt_mrot <= -LASER_ASSERV_MOTROT_POS_INCERTITUDE){
    if( (*v_old_mrot)*(*v_old_mrot-dv) < 0 ){
        vc->cv = 0;
        dv = *v_old_mrot;
    } else {
        vc->cv = *v_old_mrot-dv;
    }
  }
  else
    {
      printf("Captured velocity is equal to find Asserv_Param output velocity\n");
      return -1;
    }

  t1 = dv/a;
  if(*cp - *p_capt_mrot >= LASER_ASSERV_MOTROT_POS_INCERTITUDE){
      if((res = (DeltaS-a*t1*t1))<=0)
        vc->time = (unsigned long)(sqrt(DeltaS/a)*1000000);
      else
        vc->time = (unsigned long)((DeltaS-a*t1*t1)/dv*1000000+t1);//en micro s
  } else if (*cp - *p_capt_mrot <= -LASER_ASSERV_MOTROT_POS_INCERTITUDE) {
      if((res = (DeltaS-d*t1*t1))<=0)
        vc->time = (unsigned long)(sqrt(DeltaS/d)*1000000);
      else
        vc->time = (unsigned long)((DeltaS-d*t1*t1)/dv*1000000+t1);//en micro s

  }


  printf("DeltaS = %lf, a = %lf, t1 = %lf, res = %lf, consigne temps = %lu, consigne vitesse %d\n", DeltaS, a, t1, res, vc->time, vc->cv);

  return 0;
}

int Find_Asserv_Params2( long int * cp, INTEGER32 * p_capt_mrot, INTEGER32 * v_old_mrot, struct Laser_x_thread * vc)
{
  double d_vit_corr = LASER_ASSERV_DELTAVITESSEMAX_CORRECTION;

  if(Find_Asserv_Params(cp, p_capt_mrot, &d_vit_corr, v_old_mrot, vc) < 0){
    return -1;
  }
  while (vc->time <= LASER_ASSERV_MIN_VEL_APPLY_TIME){
    d_vit_corr = d_vit_corr/2;
    if(Find_Asserv_Params(cp, p_capt_mrot, &d_vit_corr, v_old_mrot, vc) < 0){
        return -1;
    }
  }

  return 0;

}

unsigned int CheckAndCorrect_VelocityRelation(unsigned long * d, INTEGER32 * v_cons_mt, INTEGER32 * v_cons_mrot)
{
  INTEGER32 v_cons_calc_mrot;
  unsigned int res = 0;

  if(FindConsigne_VitRot(d, v_cons_mt, &v_cons_calc_mrot))
    return FIND_CONSIGNE_FAILURE;

  if(labs(v_cons_calc_mrot) > LASER_ASSERV_MAX_MOTROT_VELOCITY)
    {
      printf("ERROR: Maximum velocity rotation reached\n");
      v_cons_calc_mrot = LASER_ASSERV_MAX_MOTROT_VELOCITY;
      res = MAX_ROT_VEL_REACHED;
    }

  if((*v_cons_mrot) != v_cons_calc_mrot)
    {
      *v_cons_mrot = v_cons_calc_mrot;
      cantools_ApplyVelRot(v_cons_calc_mrot);
    }


  return res;
}

unsigned int laser_asserv_CalcRotAccel(laser * ml, laser * sl, UNS32 * a_cons_mt, UNS32 * a_cons_mrot)
{
  long int p = 0;
  long int NumInterval = 0;
  double K;
  UNS32 a_cons_calc_mrot;
  struct laser_data d;
    printf("0\n");

  if(Laser_GetData(ml, sl, &d) == ERR_LASER_FATAL){
    return FIND_CONSIGNE_FAILURE;
  }
    printf("1\n");

  if(d_to_dparcouru(&d)) return FIND_CONSIGNE_FAILURE;
    printf("2\n");

  if(Recup_NumInterval(&(d.mes), &NumInterval))
    return FIND_CONSIGNE_FAILURE;
    printf("3\n");

  p = labs(HelixUserData.p[NumInterval]);
    printf("4\n");

  if(p == 0)
    K = 0.0;
  else
    {
      K = (double)REL_TRSL_ROT/p * (double)REDUCTION_ROTATION/REDUCTION_TRANS;
    }
    printf("5\n");

  a_cons_calc_mrot = (UNS32)((*a_cons_mt)*K);
  if(*a_cons_mrot != a_cons_calc_mrot){
    *a_cons_mrot = a_cons_calc_mrot;
  }
  return 0;
}
//d: distance laser mesuree
//d: consigne de pos en step calcule
//pcaptrot: position moteur rot en en step
//user_tol: tolerance d'écart en dixieme de mm.
int laser_asserv_estim_error_dmm(INTEGER32 posErr, unsigned int * res)
{
  long double c=0;

  c = M_PI*PIPE_DIAM*abs(posErr)/STEPS_PER_REV/REDUCTION_ROTATION;//evlt multiplier par cos(alpha)
  *res = (unsigned int)c;

  return 0;
}


//applique la consigne en même temps. Change la vitesse du(des) moteur rot en fonction de la consigne vitesse translation.
unsigned int laser_asserv_User_Velocity_Change(laser * ml, laser * sl, INTEGER32 v_cons_mt)
{
  struct laser_data d;
  INTEGER32 v_cons_mrot;

  if((Laser_GetData(ml, sl, &d) & ERR_LASER_FATAL)==ERR_LASER_FATAL)
    return 1;

  if(d_to_dparcouru(&d)) return 1;

  if(FindConsigne_VitRot(&(d.mes), &v_cons_mt, &v_cons_mrot) & FIND_CONSIGNE_FAILURE)
    return 1;

  printf("USER VELOCITY CHANGE vconsmt = %d, vconsmrot =%d\n", v_cons_mt, v_cons_mrot);
  cantools_ApplyVelRot(v_cons_mrot);
  cantools_ApplyVelTrans(v_cons_mt);
  //printf("\nUSER VELOCITY CHANGED TO %d\n\n", v_cons_mt);
  return 0;
}


//calcule et donne bêtement les consigne utilisateur au moteur
unsigned int laser_asserv_Follow_Consigne(laser * ml, laser * sl)
{
  struct laser_data d;
  INTEGER32 v_cons_mt, v_cons_mrot;

  v_cons_mt = ConsVit_T;
  v_cons_mrot = ConsVit_R;

  if((Laser_GetData(ml, sl, &d) & ERR_LASER_FATAL)==ERR_LASER_FATAL)
    return 1;

  if(d_to_dparcouru(&d)) return 1;
  //printf("RESULTAT MESURE LASER : d = %lu, t = %lu, v = %ld\n", d.mes, d.t, d.vitesse);

  if(CheckAndCorrect_VelocityRelation(&(d.mes), &v_cons_mt, &v_cons_mrot) & FIND_CONSIGNE_FAILURE)
    {
      printf("Find Consigne Failure\n");
      return 1;
    }

  return 0;
}

extern unsigned long CaptPos_R_Time;
unsigned int laser_asserv_Verify_Movement(laser * ml, laser * sl, unsigned long * next_Sync_call_Nr, unsigned long * SyncTimeInterval/*us*/)
//variables globale: data moteur rotation
//"CaptureVitesse_MotRot","CapturePosition_MotRot", "ConsigneVitesse_MotRot", "Status_word_MotRot",  "Control_word_MotRot
//"CaptureVitesse_T, CapturePosition_T, ConsigneVitesse_MotTrans, ConsignePosition_T
//data moteur translation: "Status_word_MotTrans"
{
  struct laser_data d;
  INTEGER32 v_capt_mrot, p_capt_mrot, v_cons_mrot;
  unsigned long p_capt_time;
  INTEGER32 v_cons_mt;
  INTEGER32 temp;
  UNS16 stat_motrot, ctrl_motrot;
  unsigned int TypeOfMvt = 0;
  long int cp = 0;//consigne position motrot en step
  unsigned int res = 0;
  unsigned int err = 0;

  fprintf(laser_asserv_report_file,"Laser_Verify_Movement=>");

  //si aucun evenement ou une erreur on rappelle la fonction à la prochaine mesure laser (env 150ms).
  *next_Sync_call_Nr += 1;

  EnterMutex();
  v_capt_mrot = CaptVit_R;
  p_capt_mrot = CaptPos_R;
  p_capt_time = CaptPos_R_Time;
  v_cons_mrot = ConsVit_R;
  v_cons_mt = ConsVit_T;
  LeaveMutex();

  if((Laser_GetData(ml, sl, &d) & ERR_LASER_FATAL)==ERR_LASER_FATAL)
    return ERR_LASER_FATAL;

  //v_cons_mt = (INTEGER32)d.vitesse_1s;

  fprintf(laser_asserv_report_file,"RESULTAT MESURE LASER : d = %lu, t = %lu, v = %ld\n", d.mes, d.t, d.vitesse);

  if(d_to_dparcouru((struct laser_data *)&d))
    return DPARC_CALC_ERROR;

  fprintf(laser_asserv_report_file,"RESULTAT CALC D PARCOURU : d = %lu, t = %lu, v = %ld\n", d.mes, d.t, d.vitesse);
  //TODO: Faire les vérif : moins de la pos initiale ou plus que pos finale->déclencher arrêt (Utilise d, Start_Distance, Pipe_Length).

  temp = v_cons_mrot;
  if((res = CheckAndCorrect_VelocityRelation(&(d.mes), &v_cons_mt, &v_cons_mrot)) & FIND_CONSIGNE_FAILURE)
    return res;
  //s'il y a eu correction sortir et attendre le prochain sync
  if(temp!=v_cons_mrot)
    {
      fprintf(laser_asserv_report_file,"CheckAndCorrect_VelRel v_cons_mrot = %d\n", v_cons_mrot);
      return res;
    }

  //ajouter correction de décalage temporel.
  if(laser_asserv_decalage_temp(p_capt_time, &d)){
    return ERR_DECAL_TEMPS;
  }

  fprintf(laser_asserv_report_file,"RESULTAT CALC D PARCOURU + TIME CORRECTION : d = %lu, t moteur = %lu, v_1s = %ld\n", d.mes, d.t, d.vitesse_1s);


  if(FindConsigne_PosRot(&(d.mes), &cp))
    return FIND_CONSIGNE_FAILURE;

  fprintf(laser_asserv_report_file,"CONSIGNE POSROT = %ld, POSITION MESUREE = %d\n", cp, p_capt_mrot);
  fprintf(laser_asserv_report_file,"cp - p_capt_mrot = %ld\n", cp - p_capt_mrot);

  if((TypeOfMvt = FindTypeOfMvt(v_cons_mrot, v_capt_mrot)) & TYPE_OF_MVT_FAILURE)
    return TYPE_OF_MVT_FAILURE;

  fprintf(laser_asserv_report_file,"TYPE OF MOVEMENT = %u\n", TypeOfMvt);

  if(labs(cp-p_capt_mrot)<LASER_ASSERV_MOTROT_POS_INCERTITUDE)
    {
      fprintf(laser_asserv_report_file,"POSITION OK\n");
      return MOVEMENT_OK;
    }
  else
    {
      switch(TypeOfMvt)
	{
	case VITESSE_CONSTANTE:
	  {
	    pthread_t thread_v_const;
	    fprintf(laser_asserv_report_file,"VITESSE CONSTANTE + erreur pos\n");

	    laser_asserv_estim_error_dmm((INTEGER32)(cp-p_capt_mrot), &err);

	    if(err < User_Tolerance)
	      {
		if(Find_Asserv_Params2(&cp, &p_capt_mrot, &v_cons_mrot/*entrees*/, &vc/*sortie*/))
		  {
		    fprintf(laser_asserv_report_file,"ERROR in Find Asserv_Params\n");
		    return ASSERV_FAILURE;
		  }
		else
		  {
		    fprintf(laser_asserv_report_file,"ASSERV PARAMS time = %lu, cv = %d, dv = %d\n", vc.time, vc.cv, vc.dv);
		    *next_Sync_call_Nr += vc.time/(*SyncTimeInterval);
		    if(pthread_create(&thread_v_const, NULL, Thread_Appli_Vitesse, &vc))
		      {
			fprintf(laser_asserv_report_file,"ERROR in correction thread create. Laser_verify_mvt\n");
			return ASSERV_FAILURE;
		      }
		  }
	      }
	    else
	      {
		return USER_POS_TOL_ERROR;
	      }

	    break;
	  }
	case PHASE_ACCEL:
	case PHASE_DECEL:
	  fprintf(laser_asserv_report_file,"PHASE ACCEL/DECEl\n");
	default:
	  //rien
	  break;
	}
    }

  return res;
}

void * laser_asserv_Boucle_Asservissement(void * arg)
{
 unsigned int res;
 unsigned long next_sync = 0;
 unsigned long sync_count = 0;
 unsigned long SyncTime = LASER_ASSERV_SYNC_TIME;

 run_asserv = 1;
 laser_asserv_report_file = fopen(laser_asserv_report_filename, "w");

 while(run_asserv)
    {
        if(sync_count == next_sync){
            res = laser_asserv_Verify_Movement(&ml, &sl, &next_sync, &SyncTime);
            fprintf(laser_asserv_report_file,"RESULT of Laser_Verify_Mvt: %x\n", res);
            switch (res){
                case MOVEMENT_OK: break;
                case MAX_ROT_VEL_REACHED:
                    fprintf(laser_asserv_report_file,"LASER_ASSERV WARNING: Maximum rotation velocity reached\n"); break;
                case ASSERV_ERR_PATINAGE:
                    fprintf(laser_asserv_report_file,"LASER_ASSERV WARNING: Patinage detecté\n"); break;
                case 0:
                    fprintf(laser_asserv_report_file, "LASER_ASSERV. Une correction est en cours\n"); break;
                case ASSERV_FAILURE:
                case TYPE_OF_MVT_FAILURE:
                case FIND_CONSIGNE_FAILURE:
                case FIND_CONSIGNE_FAILURE | MAX_ROT_VEL_REACHED:
                case DPARC_CALC_ERROR:
                case ERR_DECAL_TEMPS:
                    fprintf(laser_asserv_report_file,"LASER_ASSERV INTERNAL ERROR. ErrorCode = %x\n", res); break;
                case ERR_LASER_FATAL:
                    fprintf(laser_asserv_report_file,"LASER_ASSERV LASER ERROR.\n"); break;
                case USER_POS_TOL_ERROR:
                    fprintf(laser_asserv_report_file,"LASER_ASSERV USER POSITION TOLERANCE ERROR\n");break;
                default:
                    fprintf(laser_asserv_report_file,"LASER_ASSERV UNKNOWN ERROR, res = %x\n", res);
            }
            if((res & USER_POS_TOL_ERROR) || (res & ERR_LASER_FATAL) || (res & LASER_ASSERV_INTERNAL_ERR)){

                while(){
                    cantools_ApplyVelRot(0);
                    cantools_ApplyVelTrans(0);
                }
                run_asserv = 0;
                break;
            }
        }
        sync_count++;
        usleep(SyncTime);
    }
    fclose(laser_asserv_report_file);
    pthread_exit(NULL);

}

void * laser_asserv_Boucle_Asservissement_Simu(void * arg)
{
 unsigned int res;
 unsigned long next_sync = 0;
 unsigned long sync_count = 0;
 unsigned long SyncTime = LASER_ASSERV_SYNC_TIME;//150000;

 run_asserv = 1;
 laser_asserv_report_file = fopen(laser_asserv_report_filename, "w");

 while(run_asserv)
    {

        if(sync_count == next_sync){
            res = laser_asserv_Verify_Movement(&lsim, NULL, &next_sync, &SyncTime);
            fprintf(laser_asserv_report_file,"RESULT of Laser_Verify_Mvt: %x\n", res);
            switch (res){
                case MOVEMENT_OK: break;
                case MAX_ROT_VEL_REACHED:
                    fprintf(laser_asserv_report_file,"LASER_ASSERV WARNING: Maximum rotation velocity reached\n"); break;
                case ASSERV_ERR_PATINAGE:
                    fprintf(laser_asserv_report_file,"LASER_ASSERV WARNING: Patinage detecté\n"); break;
                case 0:
                    fprintf(laser_asserv_report_file, "LASER_ASSERV. Une correction est en cours\n"); break;
                case ASSERV_FAILURE:
                case TYPE_OF_MVT_FAILURE:
                case FIND_CONSIGNE_FAILURE:
                case FIND_CONSIGNE_FAILURE | MAX_ROT_VEL_REACHED:
                case DPARC_CALC_ERROR:
                case ERR_DECAL_TEMPS:
                    fprintf(laser_asserv_report_file,"LASER_ASSERV INTERNAL ERROR. ErrorCode = %x\n", res); break;
                case ERR_LASER_FATAL:
                    fprintf(laser_asserv_report_file,"LASER_ASSERV LASER ERROR.\n"); break;
                case USER_POS_TOL_ERROR:
                    fprintf(laser_asserv_report_file,"LASER_ASSERV USER POSITION TOLERANCE ERROR\n");break;
                default:
                    fprintf(laser_asserv_report_file,"LASER_ASSERV UNKNOWN ERROR\n");
            }
            if(res & USER_POS_TOL_ERROR){
                cantools_ApplyVelRot(0);
                cantools_ApplyVelTrans(0);
                run_asserv = 0;
                fprintf(laser_asserv_report_file,"USER POSITION TOLERANCE ERROR\n");
                break;
            }
        }
        sync_count++;
        usleep(SyncTime);
    }
    fclose(laser_asserv_report_file);
    pthread_exit(NULL);

}


int laser_asserv_lance(void)
{
    if(pthread_create(&(lats.thread_Nr), NULL, laser_asserv_Boucle_Asservissement, NULL)){
        return 1;
    }
    return 0;
}

int laser_asserv_lance_simu(void)
{
    if(pthread_create(&(lats.thread_Nr), NULL, laser_asserv_Boucle_Asservissement_Simu, NULL)){
        return 1;
    }

    return 0;

}

int laser_asserv_stop(void)
{
    run_asserv = 0;
    if(pthread_join(lats.thread_Nr, NULL))
    {
        return 1;
    }
    return 0;
}
#if STAND_ALONE_TEST
int main(void)
{
  struct laser_data ld;
  unsigned long d[3];
  long int p[3];
  long int res = 0;
  d[0] = 1000;
  d[1] = 30000;
  d[2] = 1000;
  p[0] = 0;
  p[1] = 4000;
  p[2] = 0;//mm

  HelixUserData.dmax = d[0]+d[1]+d[2];
  HelixUserData.NbrOfEntries = 3;
  HelixUserData.d = d;
  HelixUserData.p = p;

  printf("h.d = %lu %lu %lu, h.p = %ld %ld %ld\n", HelixUserData.d[0], HelixUserData.d[1], HelixUserData.d[2],
	 HelixUserData.p[0], HelixUserData.p[1], HelixUserData.p[2]);

  ld.mes = 310540;//mm/10!
  ld.vitesse = 0;
  ld.t = 120000;

  if(FindConsigne_PosRot(&(ld.mes), &res))
    printf("ERROR in FindConsigne PosRot\n");
  else
    printf("Consigne Pos Rot = %ld\n", res);

  //User_Velocity_Change
  laser l;
  INTEGER32 res2;
  INTEGER32 v_trans = 67200;
  int count=0;

  if(FindConsigne_VitRot(&ld.mes, &v_trans, &res2)<0)
    printf("ERROR in FindConsigne_VitRot\n");
  else
    printf("Vit trans %d -> Vit Rot = %d\n", v_trans, res2);

  res2 = 0;
  CheckAndCorrect_VelocityRelation(&ld ,&v_trans, &res2);
  printf("v_trans = %d, vrotcons calc %d\n", v_trans, res2);
  /*
    Laser_Init_Simu(&l);
    Laser_Start_Simu(&l);
    sleep(5);
  while (count<200)
    {
      User_Velocity_Change(&l, NULL, 67200);
      pthread_mutex_lock(&(l.mutex));
      printf("\nRESULTAT MESURE LASER : d = %lu, t = %lu, v = %ld\n", l.laser_dat[0].mes, l.laser_dat[0].t, l.laser_dat[0].vitesse);
      pthread_mutex_unlock(&(l.mutex));
      printf("Consigne trsl = %d=>", ConsigneVitesse_MotTrans);
      printf("Consigne rot = %d\n", ConsigneVitesse_MotRot);
      count++;
      usleep(150000);
    }
  */

  return 0;
}
#endif
