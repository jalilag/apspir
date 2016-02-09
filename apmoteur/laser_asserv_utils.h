void laser_asserv_utils_TrslManualInc(void);
void laser_asserv_utils_TrslManualDec(void);
void laser_asserv_utils_StartRot(void);//->bouton demarrage de la rotation uniquement: cas treuil
void laser_asserv_utils_StopRot(void);//->bouton setop rotation uniquement: cas treuil
void laser_asserv_utils_resumeRot(void);//->bouton continuer rotation: cas treuil
int laser_asserv_utils_RotIsActive(void);
int laser_asserv_utils_TransIsActive(void);
void laser_asserv_utils_StartPose(char * Vitesse);//-> bonton démarrage avec obtentions des constantes de démarrage
void laser_asserv_utils_StopPose(void);//-> bouton pause ou arrêt de la pose
void laser_asserv_utils_ContinuePose(char * Vitesse);//-> bouton redémarrage après pause (lancement sans recupération des constantes de démarrage)
