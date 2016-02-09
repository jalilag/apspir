#include "stdlib.h"
#include "stdio.h"
#include "math.h"//racine
#include "unistd.h"
#include "time.h"
#include "laser_asserv.h"

#include "SpirallingControl.h"
#include "SpirallingMaster.h"
#include "cantools.h"
#include "slave.h"
#include "profile.h"

int run_asserv = 0;
int run_trsl = 0;

struct laser_asserv_thread lats;
struct laser_asserv_thread asserv_translation;
struct laser_asserv_thread RotPosCorr;

INTEGER32 MotRot_Ref_Position = 0;
static unsigned long Start_Distance = 300000;
unsigned long Pipe_Length = 30000;

FILE * laser_asserv_report_file;
FILE * laser_asserv_trsl_report_file;
const char* laser_asserv_report_filename = "./laser/work_report/report_laserAsserv.dat";
const char* laser_asserv_trsl_report_filename = "./laser/work_report/report_trslAsserv.dat";

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
    d->a_1s = - d->a_1s;

    return 0;
}

int vtrans_to_vmot( long int * vtrans, INTEGER32 *vmot){
    long int l = *vtrans;
    if(l < -LASER_ASSERV_ERR_VITESSE_TRANS) *vmot = 0;//return 1;
    else if ((l <= 0) && (l>=-LASER_ASSERV_ERR_VITESSE_TRANS)){
        *vmot = 0;
    } else {
        *vmot = (INTEGER32)((*vtrans) * STEPS_PER_REV * REDUCTION_TRANS / REL_TRSL_ROT / 10);
    }
    return 0;
}

int laser_asserv_decalage_temp(unsigned long p_capt_time, struct laser_data * d){
    //partie position:
    long int l = (d->vitesse_1s*( (long int)p_capt_time - (long int)d->t ))/1000000 + d->mes;
    if(l <= -LASER_MES_ERROR) return 1;
    else if ((l <= 0) && l >= -LASER_MES_ERROR) d->mes = 0;
    else d->mes = (unsigned long)l;//+= (d->vitesse_1s*( p_capt_time - d->t ))/1000000;
    //partie vitesse

    d->t = p_capt_time;
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
//v_mstrans en step/s
//res en step/s
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
//d mesure laser en dmm
//v_capt_m en step/s
unsigned int FindTypeOfMvt_Laser(unsigned long * d, int a)
{
      //if(laser_asserv_report_file != NULL) fprintf(laser_asserv_report_file, "FINDTYPEOFMVT a = %d\n", a);
      if( abs(a) <= laser_accel_error) return VITESSE_CONSTANTE;
      else if (a > laser_accel_error) return PHASE_ACCEL;
      else if (a < -laser_accel_error) return PHASE_DECEL;
      else
	{
	  printf("ERROR FindTypeOfMvt\n");
	  return TYPE_OF_MVT_FAILURE;
	}
}

static struct Laser_x_thread vc;
static int interaction_check = 0;//poureviter que les changements utilisateur ne corrompent les corrections.

void * Thread_Appli_Vitesse(void * StructConsigneVitesse)
{
  struct Laser_x_thread v = *(struct Laser_x_thread*)StructConsigneVitesse;
  interaction_check = 1;
  if(interaction_check){
      cantools_ApplyVelRot(v.cv);
      //if(laser_asserv_report_file != NULL) fprintf(laser_asserv_report_file, "VITESSE APPLIQUEE -> %d\n",v.cv );
    }

  usleep(v.time);

  if(interaction_check) {
    cantools_ApplyVelRot(v.dv);
    //if(laser_asserv_report_file != NULL) fprintf(laser_asserv_report_file, "VITESSE APPLIQUEE -> %d\n",v.dv );
  }
  interaction_check = 0;
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

  if(*cp - *p_capt_mrot >= laser_asserv_motrot_pos_incertitude){
//    if( (*v_old_mrot)*(*v_old_mrot+dv) < 0 ){
//        vc->cv = 0;
//        dv = -(*v_old_mrot);
//    } else {
//        vc->cv = *v_old_mrot+dv;
//    }
    vc->cv = *v_old_mrot+dv;
  }
  else if (*cp - *p_capt_mrot <= -laser_asserv_motrot_pos_incertitude){
//    if( (*v_old_mrot)*(*v_old_mrot-dv) < 0 ){
//        vc->cv = 0;
//        dv = *v_old_mrot;
//    } else {
//        vc->cv = *v_old_mrot-dv;
//    }
    vc->cv = *v_old_mrot-dv;
  }
  else
    {
      printf("Captured velocity is equal to find Asserv_Param output velocity\n");
      return -1;
    }

  t1 = dv/a;
  if(*cp - *p_capt_mrot >= laser_asserv_motrot_pos_incertitude){
      if((res = (DeltaS-a*t1*t1))<=0)
        vc->time = (unsigned long)(sqrt(DeltaS/a)*1000000);
      else
        vc->time = (unsigned long)((DeltaS-a*t1*t1)/dv*1000000+t1);//en micro s
  } else if (*cp - *p_capt_mrot <= -laser_asserv_motrot_pos_incertitude) {
      if((res = (DeltaS-d*t1*t1))<=0)
        vc->time = (unsigned long)(sqrt(DeltaS/d)*1000000);
      else
        vc->time = (unsigned long)((DeltaS-d*t1*t1)/dv*1000000+t1);//en micro s

  }
  //if(laser_asserv_report_file != NULL) fprintf(laser_asserv_report_file, "DeltaS = %lf, a = %lf, t1 = %lf, res = %lf, consigne temps = %lu, consigne vitesse %d\n", DeltaS, a, t1, res, vc->time, vc->cv);

  return 0;
}

int Find_Asserv_Params2( long int * cp, INTEGER32 * p_capt_mrot, INTEGER32 * v_old_mrot, struct Laser_x_thread * vc)
{
  double d_vit_corr = laser_asserv_deltavitessemax_correction;

  if(Find_Asserv_Params(cp, p_capt_mrot, &d_vit_corr, v_old_mrot, vc) < 0){
    return -1;
  }
  while (vc->time <= laser_asserv_min_vel_apply_time){
    d_vit_corr = d_vit_corr/2;
    if(Find_Asserv_Params(cp, p_capt_mrot, &d_vit_corr, v_old_mrot, vc) < 0){
        return -1;
    }
  }

  if(vc->time > (unsigned long)laser_asserv_max_corr_time) vc->time = (unsigned long)laser_asserv_max_corr_time;
  return 0;

}
//v_cons_mt en step moteurs/s
unsigned int CheckAndCorrect_VelocityRelation(unsigned long * d, INTEGER32 * v_cons_mt, INTEGER32 * v_cons_mrot)
{
  INTEGER32 v_cons_calc_mrot;
  unsigned int res = 0;

  if(FindConsigne_VitRot(d, v_cons_mt, &v_cons_calc_mrot))
    return FIND_CONSIGNE_FAILURE;

  if(labs(v_cons_calc_mrot) > laser_asserv_max_motrot_velocity)
    {
      printf("ERROR: Maximum velocity rotation reached\n");
      v_cons_calc_mrot = laser_asserv_max_motrot_velocity;
      res = MAX_ROT_VEL_REACHED;
    }

  if((*v_cons_mrot) != v_cons_calc_mrot)
    {
      *v_cons_mrot = v_cons_calc_mrot;
      cantools_ApplyVelRot(v_cons_calc_mrot);
    }


  return res;
}

//v_cons_mt en dmm/s
unsigned int CheckAndCorrect_VelocityRelation_Laser(unsigned long * d, long int * v_cons_mt, INTEGER32 * v_cons_mrot)
{
  INTEGER32 v_cons_calc_mrot;
  INTEGER32 v_minDetectTrans = laser_asserv_trsl_mindetect_dv, v_minDetectRot;
  unsigned int res = 0;

  if(FindConsigne_VitRot_with_laser(d, v_cons_mt, &v_cons_calc_mrot))//avec vitesse laser
    return FIND_CONSIGNE_FAILURE;

  if(FindConsigne_VitRot(d, &v_minDetectTrans, &v_minDetectRot))
    return FIND_CONSIGNE_FAILURE;

  if(labs(v_cons_calc_mrot) > laser_asserv_max_motrot_velocity)
    {
      printf("ERROR: Maximum velocity rotation reached\n");
      v_cons_calc_mrot = laser_asserv_max_motrot_velocity;
      res = MAX_ROT_VEL_REACHED;
    }

  if(abs((*v_cons_mrot) - v_cons_calc_mrot) > abs(v_minDetectRot))
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

  if(Laser_GetData(ml, sl, &d) == ERR_LASER_FATAL){
    return FIND_CONSIGNE_FAILURE;
  }

  if(d_to_dparcouru(&d)) return FIND_CONSIGNE_FAILURE;

  if(Recup_NumInterval(&(d.mes), &NumInterval))
    return FIND_CONSIGNE_FAILURE;

  p = labs(HelixUserData.p[NumInterval]);

  if(p == 0)
    K = 0.0;
  else
    {
      K = (double)REL_TRSL_ROT/p * (double)REDUCTION_ROTATION/REDUCTION_TRANS;
    }

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

  c = M_PI*pipe_diam*abs(posErr)/STEPS_PER_REV/REDUCTION_ROTATION;//evlt multiplier par cos(alpha)
  *res = (unsigned int)c;

  return 0;
}


//applique la consigne en même temps. Change la vitesse du(des) moteur rot en fonction de la consigne vitesse translation.
unsigned int laser_asserv_User_Velocity_Change(laser * ml, laser * sl, INTEGER32 v_cons_mt)
{
  struct laser_data d;
  INTEGER32 v_cons_mrot;

  interaction_check = 0;

  if(v_cons_mt == 0) {
    cantools_ApplyVelRot(0);
    cantools_ApplyVelTrans(0);
    return 0;
  }

  if((Laser_GetData(ml, sl, &d) & ERR_LASER_FATAL)==ERR_LASER_FATAL)
    return 1;

  if(d_to_dparcouru(&d)) return 1;

  if(FindConsigne_VitRot(&(d.mes), &v_cons_mt, &v_cons_mrot) & FIND_CONSIGNE_FAILURE)
    return 1;

  cantools_ApplyVelTrans(v_cons_mt);
  cantools_ApplyVelRot(v_cons_mrot);

  return 0;
}

int laser_asserv_User_Velocity_Change_sans_laser(struct laser_data * d, INTEGER32 v_cons_mt){
    INTEGER32 v_cons_mrot;

    interaction_check = 0;
    if(v_cons_mt == 0) {
        cantools_ApplyVelRot(0);
        cantools_ApplyVelTrans(0);
        return 0;
    }

    if(FindConsigne_VitRot(&(d->mes), &v_cons_mt, &v_cons_mrot) & FIND_CONSIGNE_FAILURE)
        return 1;

    cantools_ApplyVelTrans(v_cons_mt);
    cantools_ApplyVelRot(v_cons_mrot);
    return 0;
}

int laser_asserv_User_Velocity_Change_sans_laser_sans_couple(struct laser_data * d, INTEGER32 v_cons_mt){
    INTEGER32 v_cons_mrot;
    interaction_check = 0;

    if(v_cons_mt == 0) {
        cantools_ApplyVelRot(0);
        cantools_ApplyVelTrans(0);
        return 0;
    }
    if(FindConsigne_VitRot(&(d->mes), &v_cons_mt, &v_cons_mrot) & FIND_CONSIGNE_FAILURE)
        return 1;


    cantools_ApplyUniqVelTrans(v_cons_mt);
    cantools_ApplyVelRot(v_cons_mrot);
    return 0;
}

extern unsigned long CaptPos_R_Time;
//une correction en rotation unique. Pour redémarrage en cas de sortie des tolérances utilisateur.
int laser_asserv_UniqCorrPos(laser * ml, laser * sl)
{
    struct laser_data d;
    INTEGER32 v_cons_mrot = 0, p_capt_mrot;
    long int cp;
    EnterMutex();
    p_capt_mrot = CaptPos_R;
    LeaveMutex();
    if ((Laser_GetData(ml, sl, &d) & ERR_LASER_FATAL) == ERR_LASER_FATAL) return 1;
    if(d_to_dparcouru((struct laser_data *)&d)) return 1;
    if(FindConsigne_PosRot(&(d.mes), &cp)) return 1;
    if(labs(cp-p_capt_mrot)>laser_asserv_motrot_pos_incertitude){
        //correction
        if(Find_Asserv_Params2(&cp, &p_capt_mrot, &v_cons_mrot, &vc)) return 1;
        if(pthread_create(&(RotPosCorr.thread_Nr), NULL, Thread_Appli_Vitesse, &vc)) return 1;
        if(pthread_join(RotPosCorr.thread_Nr, NULL)) return 1;
    }
    return 0;
}

/**VERSION SEMI_LASER. ASSERVISSEMENT POSITION + ASSERVISSEMENT VITESSE TOUT LASER**/
unsigned int laser_asserv_Verify_Movement(laser * ml, laser * sl, unsigned long * next_Sync_call_Nr, unsigned long * SyncTimeInterval)
//variables globale: data moteur rotation
//"CaptureVitesse_MotRot","CapturePosition_MotRot", "ConsigneVitesse_MotRot", "Status_word_MotRot",  "Control_word_MotRot
//"CaptureVitesse_T, CapturePosition_T, ConsigneVitesse_MotTrans, ConsignePosition_T
//data moteur translation: "Status_word_MotTrans"
{
  struct laser_data d;
  INTEGER32 v_capt_mrot, p_capt_mrot, v_cons_mrot;
  unsigned long p_capt_time;
  long int v_cons_mt;
  unsigned int TypeOfMvt = 0;
  long int cp = 0;//consigne position motrot en step
  unsigned int res = 0;
  unsigned int err = 0;
  unsigned long t = GetDate_us();

  if(laser_asserv_report_file != NULL) fprintf(laser_asserv_report_file,"\n\nLaser_Verify_Movement at t = %lu=>", t);

  //si aucun evenement ou une erreur on rappelle la fonction à la prochaine mesure laser (env 150ms).
  *next_Sync_call_Nr += 1;

  EnterMutex();
  v_capt_mrot = CaptVit_R;
  p_capt_mrot = CaptPos_R;
  p_capt_time = CaptPos_R_Time;
  v_cons_mrot = ConsVit_R;
  LeaveMutex();

  if((Laser_GetData(ml, sl, &d) & ERR_LASER_FATAL)==ERR_LASER_FATAL){
    return ERR_LASER_FATAL;
  }

/**PARTIE ASSERV VITESSE**/

  if(d_to_dparcouru((struct laser_data *)&d)){
    return DPARC_CALC_ERROR;
  }

  //ajouter correction de décalage temporel.
  if(laser_asserv_decalage_temp(p_capt_time, &d)){
    return ERR_DECAL_TEMPS;
  }

  if (laser_asserv_report_file != NULL) fprintf(laser_asserv_report_file,"GOT DATA : Laser d = %lu, t = %lu, v = %ld, a = %ld\n Moteur Capt V = %d, Cons V = %d\n",
    d.mes, d.t, d.vitesse_1s, d.a_1s, v_capt_mrot, v_cons_mrot);


  if(FindConsigne_PosRot(&(d.mes), &cp)){
    return FIND_CONSIGNE_FAILURE;
  }

  laser_asserv_estim_error_dmm((INTEGER32)(cp-p_capt_mrot), &err);
  if(laser_asserv_report_file != NULL) fprintf(laser_asserv_report_file,"CABLE POSITION ERROR RECORDED = %u\n", err);

  if(err > User_Tolerance) return USER_POS_TOL_ERROR;
  //TODO: Faire les vérif : moins de la pos initiale ou plus que pos finale->déclencher arrêt (Utilise d, Start_Distance, Pipe_Length).


/**PARTIE ASSERV POSITION(si vitesse ok)**/

  if((TypeOfMvt = FindTypeOfMvt_Laser(&(d.mes), d.a_1s)) & TYPE_OF_MVT_FAILURE){
    return TYPE_OF_MVT_FAILURE;
  }
  //TypeOfMvt = VITESSE_CONSTANTE;

  if(laser_asserv_report_file != NULL) fprintf(laser_asserv_report_file,"TYPE OF MOVEMENT = %u\n", TypeOfMvt);

  if(labs(cp-p_capt_mrot)<laser_asserv_motrot_pos_incertitude)
    {
      if(laser_asserv_report_file != NULL) fprintf(laser_asserv_report_file,"POSITION OK\n");
      return MOVEMENT_OK;
    }
  else
    {
      switch(TypeOfMvt)
	{
	case VITESSE_CONSTANTE:
	  {
	    if(laser_asserv_report_file != NULL) fprintf(laser_asserv_report_file,"VITESSE CONSTANTE + erreur pos\n");
	    //correction vitesse
	    v_cons_mt = d.vitesse_1s;
	    if((res = CheckAndCorrect_VelocityRelation_Laser(&(d.mes), &v_cons_mt, &v_cons_mrot)) & FIND_CONSIGNE_FAILURE){
            return res;
        }
        if(laser_asserv_report_file != NULL) fprintf(laser_asserv_report_file, "v_cons_mrot = %d\n", v_cons_mrot);
        //correction position
	      //v_cons_mrot en entree, vc sortie
		if(Find_Asserv_Params2(&cp, &p_capt_mrot, &v_cons_mrot, &vc))
		  {
		    if(laser_asserv_report_file != NULL) fprintf(laser_asserv_report_file,"ERROR in Find Asserv_Params\n");
		    return ASSERV_FAILURE;
		  }
		else
		  {
		    if(laser_asserv_report_file != NULL) fprintf(laser_asserv_report_file,"ASSERV PARAMS time = %lu, cv = %d, dv = %d\n", vc.time, vc.cv, vc.dv);
		    *next_Sync_call_Nr += vc.time/(*SyncTimeInterval);
		    if(pthread_attr_setdetachstate(&(RotPosCorr.attribute), PTHREAD_CREATE_DETACHED))
		      {
                if(laser_asserv_report_file != NULL) fprintf(laser_asserv_report_file, "ERROR in correction attr set. Laser_verify_mvt\n");
                return ASSERV_FAILURE;
		      }
		    if(pthread_create(&(RotPosCorr.thread_Nr), &(RotPosCorr.attribute), Thread_Appli_Vitesse, &vc))
		      {
                if(laser_asserv_report_file != NULL) fprintf(laser_asserv_report_file,"ERROR in correction thread create. Laser_verify_mvt\n");
                return ASSERV_FAILURE;
		      }
		  }
	    break;
	  }
	case PHASE_ACCEL:
	case PHASE_DECEL:
	  if(laser_asserv_report_file != NULL) fprintf(laser_asserv_report_file,"PHASE ACCEL/DECEl\n");
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
 unsigned long SyncTime = laser_asserv_sync_time;

 run_asserv = 1;

 while(run_asserv)
    {
        usleep(SyncTime);
        if(sync_count == next_sync){
            res = laser_asserv_Verify_Movement(&ml, &sl, &next_sync, &SyncTime);
            if(laser_asserv_report_file != NULL) fprintf(laser_asserv_report_file,"RESULT of Laser_Verify_Mvt: %x\n", res);
            switch (res){
                case MOVEMENT_OK: break;
                case MAX_ROT_VEL_REACHED:
                    if(laser_asserv_report_file != NULL) fprintf(laser_asserv_report_file,"LASER_ASSERV WARNING: Maximum rotation velocity reached\n");
                    break;
                case ASSERV_ERR_PATINAGE:
                    if(laser_asserv_report_file != NULL) fprintf(laser_asserv_report_file,"LASER_ASSERV WARNING: Patinage detecté\n");
                    break;
                case 0:
                    if(laser_asserv_report_file != NULL) fprintf(laser_asserv_report_file, "LASER_ASSERV. Une correction est en cours\n");
                    break;
                case ASSERV_FAILURE:
                case TYPE_OF_MVT_FAILURE:
                case FIND_CONSIGNE_FAILURE:
                case FIND_CONSIGNE_FAILURE | MAX_ROT_VEL_REACHED:
                case DPARC_CALC_ERROR:
                case ERR_DECAL_TEMPS:
                    if(laser_asserv_report_file != NULL) fprintf(laser_asserv_report_file,"LASER_ASSERV INTERNAL ERROR. ErrorCode = %x\n", res);
                    break;
                case ERR_LASER_FATAL:
                    if(laser_asserv_report_file != NULL) fprintf(laser_asserv_report_file,"LASER_ASSERV LASER ERROR.\n");
                    break;
                case USER_POS_TOL_ERROR:
                    if(laser_asserv_report_file != NULL) fprintf(laser_asserv_report_file,"LASER_ASSERV USER POSITION TOLERANCE ERROR\n");
                    break;
                default:
                    if(laser_asserv_report_file != NULL) fprintf(laser_asserv_report_file,"LASER_ASSERV UNKNOWN ERROR, res = %x\n", res);
                    break;
            }
            if((res & USER_POS_TOL_ERROR) || (res & ERR_LASER_FATAL) || (res & LASER_ASSERV_INTERNAL_ERR)){
                if(laser_asserv_report_file != NULL) fprintf(laser_asserv_report_file, "ARRET DE L'ASSERVISSEMENT\n");
                run_asserv = 0;
                if(run_trsl) laser_asserv_translation_stop();
                break;
            }
        }
        sync_count++;
    }
    interaction_check = 0;
    cantools_ApplyVelRot(0);
    pthread_exit(NULL);

}

static unsigned int count_synctrsl = 0;
static INTEGER32 Derniere_Consigne = 0;
unsigned int laser_asserv_CheckAndCorrect_Trsl(laser * ml, laser * sl, INTEGER32 Vitesse_Consigne){

    INTEGER32 v_cons_mvt, v_capt_mvt;
    struct laser_data d;
    INTEGER16 coupl;
    /**recupération des données**/
    EnterMutex();
    v_cons_mvt = ConsVit_T;
    v_capt_mvt = CaptVit_T;
    coupl = ConsCoupl_T;
    LeaveMutex();
    if((Laser_GetData(ml, sl, &d) & ERR_LASER_FATAL) == ERR_LASER_FATAL) return ERR_LASER_FATAL;
    if(laser_asserv_report_file != NULL) fprintf(laser_asserv_trsl_report_file, "laser: dist = %lu, time = %lu, vel = %ld, acc = %ld\n", d.mes, d.t, d.vitesse_1s, d.a_1s);
    if(d_to_dparcouru(&d)) return DPARC_CALC_ERROR;
    //que des vitesses positives
    if(v_cons_mvt < 0 || v_capt_mvt < 0) return VELOCITY_SIGN_FAILURE;
    /**Traitement**/
    INTEGER32 vmot_t_laser;
    if(vtrans_to_vmot(&(d.vitesse_1s), &vmot_t_laser)) return ERR_VTRANS_TO_VMOT;

    count_synctrsl++;

    // tout en step moteurs
    //vtrans laser : vmot_t_laser
    //vconsigne moteur : v_cons_mvt
    //vcapture moteur : v_capt_mvt
    //vderniere consigne moteur : Derniere_Consigne
    if(laser_asserv_report_file != NULL) fprintf(laser_asserv_trsl_report_file, "Moteurs trsl: Consigne VT = %d, Capture VT = %d, Vitesse L lent= %d,  Accel L = %ld, Couple = %d\n",
        v_cons_mvt, v_capt_mvt, vmot_t_laser, d.a_1s, coupl);

    if(abs(d.a_1s) <= laser_accel_error){
    //vitesse stabilisée
        //verification que la vitesse mesurée est bonne
        if(abs(v_capt_mvt - vmot_t_laser) <= laser_asserv_trsl_mindetect_dv){
          //vitesse mesurée est bonne
            if(abs(v_capt_mvt - v_cons_mvt) <= laser_asserv_tol_diff_mottrans_indic){
                //capture correspond à consigne
                if(laser_asserv_report_file != NULL) fprintf(laser_asserv_trsl_report_file, "asserv trsl cas 1\n");
                if(((v_cons_mvt+laser_asserv_trsl_correction_dv) > Vitesse_Consigne) && (v_cons_mvt != Vitesse_Consigne)){
                    Derniere_Consigne = v_cons_mvt;
                    v_cons_mvt = Vitesse_Consigne;
                    if(run_asserv){//si rotation active
                        if(laser_asserv_User_Velocity_Change_sans_laser(&d,v_cons_mvt)) return FIND_CONSIGNE_FAILURE;
                    } else {
                        cantools_ApplyVelTrans(v_cons_mvt);
                    }
                } else if((v_cons_mvt+laser_asserv_trsl_correction_dv) <= Vitesse_Consigne) {
                    Derniere_Consigne = v_cons_mvt;
                    v_cons_mvt += laser_asserv_trsl_correction_dv;
                    if (run_asserv){//si rotation active
                        if(laser_asserv_User_Velocity_Change_sans_laser(&d,v_cons_mvt)) return FIND_CONSIGNE_FAILURE;
                    } else {
                        cantools_ApplyVelTrans(v_cons_mvt);
                    }
                }
                count_synctrsl = 0;
                return MOVEMENT_OK;
            } else {
                //if(laser_asserv_report_file != NULL) fprintf(laser_asserv_trsl_report_file, "asserv trsl cas 2: capture correspond pas à consigne\n");
                //capture correspond pas à consigne: glissement mais vitesse stabilisée ou trop de couple et vitesse trop rapide
                if(abs(vmot_t_laser - v_cons_mvt) <= laser_asserv_trsl_mindetect_dv){
                    //difficulté du moteur vitesse mais avancé correcte grace au moteur couple-> on touche à rien
                } else if (vmot_t_laser > v_cons_mvt + laser_asserv_trsl_mindetect_dv){
                    //si vitesse trop rapide: point dur débloqué -> on réapplique couple par default.
                    cantools_ApplyVelTrans(v_cons_mvt);
                } else {
                    //trop lent le mot vitesse en difficultés. On l'aide avec du couple. Si impossible on ralenti.
                    if (coupl <= 1000-laser_asserv_trsl_coupl_inc) {
                        coupl+=laser_asserv_trsl_coupl_inc;
                        cantools_ApplyCouplTrans(coupl);
                    }
                    else if (coupl > 1000 - laser_asserv_trsl_coupl_inc && coupl < 1000) {
                        coupl = 1000;
                        cantools_ApplyCouplTrans(coupl);
                    } else {
                        //couple deja au max on ralenti la vitesse
                        Derniere_Consigne = v_cons_mvt; count_synctrsl = 0;
                        if(v_cons_mvt - laser_asserv_trsl_correction_dv <= 0) v_cons_mvt = v_cons_mvt / 2;
                        else v_cons_mvt -= laser_asserv_trsl_correction_dv;
                        if (run_asserv){
                            if(laser_asserv_User_Velocity_Change_sans_laser_sans_couple(&d,v_cons_mvt)) return FIND_CONSIGNE_FAILURE;
                        } else {
                            cantools_ApplyUniqVelTrans(v_cons_mvt);
                        }
                    }

                }
                return WARNING_TRSL_CORRECTION_ENCOURS;
            }
        } else {
            //patinage du moteur vitesse
            if(laser_asserv_report_file != NULL) fprintf(laser_asserv_trsl_report_file, "asserv trsl cas 3: patinage du moteur vitesse\n");
            if( v_cons_mvt < vmot_t_laser - laser_asserv_trsl_mindetect_dv){
                //on va trop vite selon le laser-> du a un reglage couple trop important qui fait patiner la vitesse. repasse au reglage par default:
                cantools_ApplyVelTrans(v_cons_mvt);
                return WARNING_VEL_LASER_MOT_INCONSISTENT;
            }
            if (coupl <= 1000-laser_asserv_trsl_coupl_inc) {
                coupl+=laser_asserv_trsl_coupl_inc;
                cantools_ApplyCouplTrans(coupl);
            }
            else if (coupl > 1000 - laser_asserv_trsl_coupl_inc && coupl < 1000) {
                coupl = 1000;
                cantools_ApplyCouplTrans(coupl);
            } else {
                //couple deja au max on ralenti la vitesse
                Derniere_Consigne = v_cons_mvt; count_synctrsl = 0;
                if(v_cons_mvt - laser_asserv_trsl_correction_dv <= 0) v_cons_mvt = v_cons_mvt / 2;
                else v_cons_mvt -= laser_asserv_trsl_correction_dv;
                cantools_ApplyUniqVelTrans(v_cons_mvt);//if(laser_asserv_User_Velocity_Change_sans_laser(&d,v_cons_mvt)) return FIND_CONSIGNE_FAILURE
                count_synctrsl = 0;
            }
            //vitesse de capture moteur correspond pas à vitesse de mesure laser: on patine
            //TODO: mettre des cantools_ApplyCouplTrans
            return WARNING_VEL_LASER_MOT_INCONSISTENT;
        }
    } else if (abs(d.a_1s) > laser_accel_error){
        if(laser_asserv_report_file != NULL) fprintf(laser_asserv_trsl_report_file, "asserv trsl cas 4\n");
        //pas stabilise:
        if(vmot_t_laser > (v_cons_mvt + laser_asserv_trsl_mindetect_dv)){
            //cas acceleration forte suite à débloquage-> on repasse au réglage par default.
            cantools_ApplyVelTrans(v_cons_mvt);
        }
        if(v_capt_mvt < v_cons_mvt){
            //phase accel
            if((abs(Derniere_Consigne + ConsAccel_T * laser_asserv_trsl_sync_time * count_synctrsl - v_capt_mvt) < laser_asserv_tol_diff_mottrans_indic)
                || ((abs(v_cons_mvt - v_capt_mvt) < laser_asserv_tol_diff_mottrans_indic) && (abs(v_cons_mvt - vmot_t_laser) < laser_asserv_trsl_mindetect_dv))){
                return MOVEMENT_OK;
            } else {
                return WARNING_ACCELERATION_TRSL_NON_RESPECT;
            }
        } else if (v_capt_mvt >= v_cons_mvt){
            //phase decel
            if((abs(Derniere_Consigne - ConsDecel_T * laser_asserv_trsl_sync_time * count_synctrsl - v_capt_mvt) < laser_asserv_tol_diff_mottrans_indic)
                 || ((abs(v_cons_mvt - v_capt_mvt) < laser_asserv_tol_diff_mottrans_indic) && (abs(v_cons_mvt - vmot_t_laser) < laser_asserv_trsl_mindetect_dv))){
                return MOVEMENT_OK;
            } else {
                return WARNING_ACCELERATION_TRSL_NON_RESPECT;
            }
        }

    }


    return 0;
}

void* laser_asserv_Boucle_Translation(void* vel){
    unsigned int res;
    INTEGER32 Vitesse = *(INTEGER32*)vel;
    run_trsl = 1;
    if(laser_asserv_report_file != NULL) fprintf(laser_asserv_trsl_report_file,"LASER ASSERVISSEMENT TRANSLATION STARTED WITH VEL %d\n", Vitesse);
    while(run_trsl){
        usleep(laser_asserv_trsl_sync_time);
        res = laser_asserv_CheckAndCorrect_Trsl(&ml, &sl,Vitesse);
        switch(res){
            case MOVEMENT_OK:
            case 0:
                if(laser_asserv_report_file != NULL) fprintf(laser_asserv_trsl_report_file,"LASER_ASSERV Boucle translation movement ok\n");
                break;
            case WARNING_ACCELERATION_TRSL_NON_RESPECT:
                if(laser_asserv_report_file != NULL) fprintf(laser_asserv_trsl_report_file,"LASER_ASSERV WARNING Boucle translation accel non respect\n");
                break;
            case WARNING_TRSL_CORRECTION_ENCOURS:
                if(laser_asserv_report_file != NULL) fprintf(laser_asserv_trsl_report_file, "LASER_ASSERV TRSL WARNING changement de vitesse\n");
                break;
            case WARNING_VEL_LASER_MOT_INCONSISTENT:
                if(laser_asserv_report_file != NULL) fprintf(laser_asserv_trsl_report_file, "LASER ASSERV TRSL WARNING patinage du moteur vitesse\n");
                break;
            case ERR_VTRANS_TO_VMOT:
            case DPARC_CALC_ERROR:
            case VELOCITY_SIGN_FAILURE:
                if(laser_asserv_report_file != NULL) fprintf(laser_asserv_trsl_report_file,"LASER_ASSERV Boucle translation Internal error: %x\n", res);
                break;
            case ERR_LASER_FATAL:
                if(laser_asserv_report_file != NULL) fprintf(laser_asserv_trsl_report_file,"LASER_ASSERV Boucle translation Erreur Laser\n");
                break;
            default:
                if(laser_asserv_report_file != NULL) fprintf(laser_asserv_trsl_report_file,"LASER_ASSERV Boucle translation Unknown error: %x\n", res);
                break;
        }
        if(res == ERR_LASER_FATAL || res & LASER_ASSERV_INTERNAL_ERR){
            if(laser_asserv_report_file != NULL) fprintf(laser_asserv_trsl_report_file, "LASER_ASSERV TRSL STOP FAILURE\n");
            run_trsl = 0;
            if (run_asserv) run_asserv = 0;
            break;
        }
    }
    cantools_ApplyVelTrans(0);
    pthread_exit(NULL);
}

int laser_asserv_translation_lance_lent(void){
    laser_asserv_trsl_report_file = fopen(laser_asserv_trsl_report_filename, "w");
    if(pthread_create(&(asserv_translation.thread_Nr), NULL, laser_asserv_Boucle_Translation, &Vitesse_lente)){
        return 1;
    }
    return 0;
}

int laser_asserv_translation_lance_rapide(void){
    laser_asserv_trsl_report_file = fopen(laser_asserv_trsl_report_filename, "w");
    if(pthread_create(&(asserv_translation.thread_Nr), NULL, laser_asserv_Boucle_Translation, &Vitesse_de_croisiere)){
        return 1;
    }
    return 0;
}

int laser_asserv_translation_stop(void){
    run_trsl = 0;
    if(pthread_join(asserv_translation.thread_Nr, NULL)) return 1;
    fclose(laser_asserv_trsl_report_file);
    return 0;
}

int laser_asserv_rotation_lance(void)
{
    laser_asserv_report_file = fopen(laser_asserv_report_filename, "w");
    if(pthread_create(&(lats.thread_Nr), NULL, laser_asserv_Boucle_Asservissement, NULL)){
        return 1;
    }
    return 0;
}

int laser_asserv_rotation_stop(void)
{
    run_asserv = 0;
    if(pthread_join(lats.thread_Nr, NULL))
    {
        return 1;
    }
    fclose(laser_asserv_report_file);
    return 0;
}
