#include "canfestival.h"
#include "laser.h"
//#include <pthread.h>
#include "laser_asserv_cst.h"

//struct asservissement
struct Laser_x_thread
{
  unsigned long time;
  INTEGER32 cv;//consigne vitesse steps/s
  INTEGER32 dv; //dernière vitesse steps/s
};
//Struct thread asserv
struct laser_asserv_thread
{
    pthread_t thread_Nr;
    pthread_mutex_t mutex;
    pthread_attr_t attribute;
};

//indicateurs de fonctionnement du module
extern int run_asserv;//1=running, 0=not running
extern int run_trsl;

/**a fournir par l'utilisateur**/

extern laser ml, sl, lsim;

//donnée hélice utilisateur
extern struct Helix_User_Data HelixUserData;

//données ext à fournir. Sinon utilisation des valeurs par défault.
unsigned int laser_asserv_GetStartPosition(laser * ml, laser * sl);//position machine
extern INTEGER32 MotRot_Ref_Position;// default value = 0stp. pos au démarrage du moteur
unsigned long Laser_Asserv_GetStartDistance(void);//default value = 300000;//dmm
extern unsigned long Pipe_Length;//default value = 30000;

/**a definir avant lancement: acceleration de consigne rotation**/
unsigned int laser_asserv_CalcRotAccel(laser * ml, laser * sl, UNS32 * a_cons_mt, UNS32 * a_cons_mrot);

/**Correction en position à l'arrêt (utile lorsqu'on est sortie des tolérances pour continuer la pose)**/
int laser_asserv_UniqCorrPos(laser * ml, laser * sl);

/**lancement et arrêt du module**/
int laser_asserv_rotation_lance(void);
int laser_asserv_rotation_stop(void);

//changement consigne de vitesse de translation par l'utilisateur
unsigned int laser_asserv_User_Velocity_Change(laser * ml, laser * sl, INTEGER32 v_cons_mt);



/**TRANSLATION: a lancer après rotation**/

int laser_asserv_translation_lance_lent(void);
int laser_asserv_translation_lance_rapide(void);
int laser_asserv_translation_stop(void);
