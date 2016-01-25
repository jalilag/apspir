#include "struct_laser.h"

#include "errgen_laser.h"

//REGLAGES
//constantes
#define NUM_READPORT_ERROR_TOL 40
#define TIMECONSISTENCY_VAL 1000000 // in micro s
#define POSITIONCONSISTENCY_VAL 60 // en 10eme de mm
#define MESCONSISTENCY_VAL(vitesse_machine) (/*POSITIONCONSISTENCY_VAL*/60+labs(vitesse_machine)/2)//vitesse_machine en 10eme de mm/s
#define CONVERT_MUS_S(t) ((double)(t)/1000000)
#define MAX_ACCEL 167 //10⁴/60 idem 1m/min/sec

#define TIME_VITESSE_CALC 1000000 //en micro sec. attente entre deux mesures de position
#define TIME_ERROR_VAL_VITESSE 50//micro sec: temps de récup des données
/*****************configuration des droits d'accès aux ports*****************************************/

int Laser_serial_config(void);

/*****************cas ou utilisateur a besoin de lancer un laser séparément**************************/

//FONCTION D'INIT LASER
//func return -1 failure, 0 success

int Laser_Init(laser * l);
int Laser_Recv_Start(laser * l);
int Laser_wait_until_ready(laser * l);

unsigned int Laser_Start1Laser(laser * l);
int Laser_Exit1Laser(laser * l);


//FONCTION EXIT LASER
//func return -1 failure, 0 success
int Laser_Recv_Stop(laser * l);
int Laser_Close(laser * l);

//posOffset
int Laser_GetPositionOffset(laser * master, laser * slave);//appellée par Laser_Start2Laser(prend la diff master.mes-slave.mes)

//mesures
//retourne 0 si ok 1 si l pas près
unsigned int Laser_GetUnverifiedData(laser * l, unsigned long * mesl);

/*****************cas ou utilisateur souhaite definir direct 2 laser********************************/

//FONCTION D'INIT LASER
//resultats retournes (voir define ci-dessus)
unsigned int Laser_Init2Laser(laser * ml, laser * sl);//init
unsigned int Laser_Start2Laser(laser * ml, laser * sl);//recv start , wait until ready, GetPosOffset
//FONCTION EXIT LASER
//utiliser Laser_Exit1Laser 2 fois.

/****************FONCTION RECUP DONNEES***************************************************/

//return laser_status
//temps d'exec <50us, sort la derniere mesure et la valide
//retourne état laser (voir define plus haut)
unsigned int Laser_GetData(laser * master, laser * slave, struct laser_data * d);


//temps d'exec >1s precision abs 30dmm/s à vitesse constante
unsigned int Laser_GetVitesse(laser * master, laser * slave, struct laser_data * d);


/**Laser Simulation**/
void Laser_Init_Simu(laser * l);
unsigned long GetDate_us(void);
