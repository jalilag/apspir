#ifndef _KEYWORD_H
#define _KEYWORD_H
#include "data.h"
#include <gtk/gtk.h>
void keyword_init ();
gboolean keyword_maj(gpointer data);
gboolean keyword_active_maj(gpointer data);

// STYLE
#define ADD "Ajouter"
#define APPLICATION_SHUTDOWN "L'application va maintenant se terminer."
#define ACTIVE "Actif"
#define ABSOLUTE "Absolue"
#define ACCELERATION "Accélération"
#define ACTUAL_STATE "Etat actuel"
#define APPLI_TITLE "Spiralling Machine Appli"
#define ARRET "Arret"
#define BACKWARD "Arrière"
#define BOX_TRANS_TITLE "Pilotage de la translation"
#define COBID "COB ID"
#define CONFIGURE "Configuration"
#define COUPLE "Couple (%)"
#define COUPLE_SLOPE "Pente de couple (pour mille/s)"
#define CURRENT "Courant électrique"
#define DECELERATION "Décélération"
#define DECELERATION_QS "QS Décélération"
#define DEFAULT "NULL"
#define DENOMINATION "Dénomination"
#define DIAL_BOX_INIT_TITLE "Erreur d'initialisation"
#define DIRECTION "Direction"
#define DELAYED "Différé"
#define ETAT "Etat"
#define ELEM_KEY "Elément concerné : "
#define FIELD "Champs"
#define FREE "Libre"
#define FORWARD "Avant"
#define ERROR_NUMBER "Nombre d'erreur"
#define ERROR "Erreur"
#define INCREMENTATION "Position"
#define INDEX "Index"
#define INITIALIZATION "Initialization"
#define INIT_ERROR "Erreur d'initialisation"
#define INIT_RUNNING "Initialisation en cours ..."
#define INIT_SUCCESS "Initialisation réussi"
#define GO "Go"
#define HALT "Actif"
#define HELIX_STEP "Pas d'hélice"
#define HELIX_PARAM_TITLE "Définition des pas d'hélice"
#define IMMEDIATE "Immédiat"
#define LAB_INIT_ERROR "Le passage en mode OPERATIONEL a échoué pour l'élémént"
#define LASER "Laser"
#define LASERG "Laser Gauche"
#define LASERD "Laser Droit"
#define LASER_DIST "Distance en m"
#define LASER_TITLE "Lecture Laser"
#define LENGHT_DEFINED "Distance défini"
#define LOCAL_WRITING_SUCCESS "Configuration PDOr du maitre avec succès"
#define LOCAL_WRITING_ERROR "Configuration PDOr du maitre échouée"
#define LSS_ERROR "Erreur de configuration LSS"
#define LSS_SUCCESS "Configuration LSS réussi"
#define LSS_CONF_NODE_ID_SUCCESS "Configuration du NODE ID réussi"
#define LSS_CONF_NODE_ID_ERROR "Erreur de configuration du NODE ID"
#define LSS_VENDOR_ID_SUCCESS "LSS Vendor ID Success"
#define MAX_VELOCITY "Vitesse Maximale"
#define MOTOR_ID "Veuillez entrer les identifiants moteurs."
#define MOTOR_PARAM_TITLE "Définition des moteurs"
#define MOTOR_TRANS_1 "Moteur translation 1"
#define MOTOR_TRANS_2 "Moteur translation 2"
#define MOTOR_ROT "Moteur rotation"
#define MOTOR_VEL "Moteur vitesse"
#define NO "Non"
#define NODE "Noeud"
#define NODE_PROFILE "Profil"
#define NODE_INDEX "Index"
#define NOW "En cours"
#define ON "Start"
#define OFF "OFF"
#define PIPE_LENGTH "Longueur du pipe"
#define PDOR1 "PDOR 1"
#define PDOR2 "PDOR 2"
#define PDOR3 "PDOR 3"
#define PDOR4 "PDOR 4"
#define PDOT3 "PDOT 3"
#define PDOT4 "PDOT 4"
#define PARAMETERS "Paramètres"
#define PLAY_SLOW "Lent"
#define PLAY_QUICK "Rapide"
#define PLAY_HALT "Pause"
#define POWER_STATE "Etat de marche"
#define POWER_STATE_ERROR "Erreur de mise en marche"
#define PRODUCT "Product code"
#define PROFIL_PARAM_TITLE "Définition des Profils"
#define PROFILE_MODE "Mode"
#define PROFILE_NOT_FOUND "Profil moteur introuvable dans le fichier de définition des esclaves"
#define READ_ABORT_ERROR "Lecture impossible"
#define READ_RUNNING "Lecture en cours ..."
#define READ_SIZE_ERROR "La taille annoncée n'est pas suffisante"
#define READ_SUCCESS "Lecture avec succés"
#define READ_TYPE_ERROR "Type non défini"
#define RELATIVE "Relatif"
#define REMOVE "Supprimer"
#define RETURN "Retour"
#define REVISION "Rev number"
#define SAVE "Sauvegarde"
#define SAVE_ABORT "La sauvegarde a échoué"
#define SAVE_SUCCESS "Les paramètres ont bien été sauvegardé"
#define SAVE_CONFIG "Souhaitez-vous mettre à jour les fichiers de configuration avec les nouveaux paramètres ?"
#define SDO_ERROR "Traitement de la SDO échoué"
#define SDO_SUCCESS "SDO traitée avec succès"
#define SDO_READING_ERROR "Erreur de lecture de SDO"
#define SDO_READING_RUNNING "Lecture de SDO en cours"
#define SDO_READING_TIMEOUT "Lecture SDO trop lente"
#define SDO_READING_SUCCESS "Lecture de SDO avec succès"
#define SDO_WRITING_RUNNING "Ecriture de SDO en cours"
#define SDO_WRITING_SUCCESS "Ecriture de SDO avec succès"
#define SDO_WRITING_ERROR "Erreur d'écriture SDO"
#define SDO_WRITING_TIMEOUT "Ecriture SDO trop lente"
#define SDO_WRITING_RELOAD "Tenter de renouveler l'écriture de la SDO"
#define SERIAL "Numéro de série"
#define SLAVE_INACTIVE "Le noeud a été désactivé"
#define START_NODE "Démarrage du noeud"
#define START_RUNNING "Demarrage des périphériques en cours ..."
#define START_SUCCESS "Demarrage des périphériques réussi"
#define START_ERROR_CONTENT "Les noeuds n'ont pas pu être démarrer normalement. Veuillez éteindre et rallumer les périphériques et relancer l'application."
#define START_ERROR_TITLE "Erreur de démarrage des noeuds"
#define STATE_CON "En connexion"
#define STATE_DISC "Déconnecté"
#define STATE_INIT "Initialisation"
#define STATE_PREP "En Préparation"
#define STATE_PREOPE "Pré-opérationel"
#define STATE_OPE "Opérationel"
#define STATE_STOP "Arrété"
#define STATE_UNK "Etat Inconnu"
#define STATE_ERROR "Erreur d'initialisation"
#define STATUT_TITLE "Status"
#define STATUT_BAR "STATUS -> "
#define STATUT "Statuts des périphériques"
#define START "Go"
#define STOP "Stop"
#define STOP_SUCCESS "Arret des périphériques réussi"
#define STOP_RUNNING "Arret des périphériques en cours ..."
#define STOP_ERROR "Erreur d'arret : "
#define SLAVE_CONFIGURATION "Configuration de l'esclave"
#define SUBINDEX "Sub-Index"
#define SWITCH_OFF_SUCCESS "Arret du périphérique réussi"
#define SWITCH_OFF_ERROR "Erreur d'arret du périphérique"
#define SWITCH_ON_ERROR "Erreur de mise en marche"
#define SWITCH_ON_SUCCESS "Démarrage du périphérique avec succès"
#define TORQUE_HMT "Control HMT"
#define TRANSMISSION "Type de transmission"
#define TRANSLATION_TITLE "Avancée de la machine"
#define TRANSLATION_VIT_TITLE "Vitesse en m"
#define TRANSLATION_READ_INFO "Données moteurs translations"
#define RESET_APPLICATION_TITLE "Redémarrage"
#define RESET_APPLICATION_CONTENT "L'application va maintenant redémarrer."
#define ROTATION_CONTROL_TITLE "Controle de la rotation"
#define ROTATION_READ_INFO "Données moteur rotation"
#define TARGET_PROFILE "Incrementation"
#define TARGET_TORQUE "Couple"
#define TARGET_VELOCITY "Vitesse"
#define TEMPERATURE "Température"
#define TIME_2_FINISH "Temps de pose"
#define TIME "Durée"
#define TORQUE_RAMP "Torque ramp"
#define TORQUE_VELOCITY "Ratio Vitesse (couple)"
#define TORQUE_VELOCITY_MAKEUP "Vitesse max (couple)"
#define TORQUE_HMT_ACTIVATE "Activation HMT"
#define TORQUE_HMT_CONTROL "Control HMT"
#define TORQUE "Couple"
#define VALUE "Valeur"
#define VENDOR "Vendeur ID"
#define VOLTAGE "Tension"
#define VELOCITY "Vitesse"
#define VELOCITY_ACTUAL "Vitesse actuelle"
#define VELOCITY_INC "Inc. Vitesse"
#define VELOCITY_SET "Controle Vitesse"
#define VELOCITY_SEND "Vitesse souhaitée"
#define VELOCITY_END "Vitesse d'arret"
#define VITESSE_UNIT "m/s"
#define YES "Oui"
#define WARNING "Attention"

// Motor powor state
#define NR2SON 0x01
#define SOND 0x02
#define R2SON 0x03
#define SON 0x04
#define OENABLED 0x05
#define QUICK_STOP_A 0x06
#define FAULT_REACTION_A 0x07
#define FAULT 0x08
#define NOT_DEFINED 0x00

#define NR2SON_txt "Not ready to switch ON"
#define SOND_txt "Switch ON Disabled"
#define R2SON_txt "Ready to switch ON"
#define SON_txt "Switched ON"
#define OENABLED_txt "Operation Enabled"
#define QUICK_STOP_A_txt "Quick stop active"
#define FAULT_REACTION_A_txt "Fault reaction active"
#define FAULT_txt "Fault"
#define NOT_DEFINED_txt "Non défini"

// INIT STATE
#define STATE_DISCONNECTED 0
#define STATE_LSS_CONFIG 1
#define STATE_PREOP 2
#define STATE_CONFIG 3
#define STATE_OP 4
#define STATE_SON 5
#define STATE_READY 6

#define STATE_DISCONNECTED_TXT "Déconnecté"
#define STATE_LSS_CONFIG_TXT "Configuration LSS"
#define STATE_PREOP_TXT "Pre-opérationnel"
#define STATE_CONFIG_TXT "Configuration"
#define STATE_OP_TXT "Opérationnel"
#define STATE_SON_TXT "Mise en marche"
#define STATE_READY_TXT "Pret"


// INIT STATE ERROR
#define ERROR_STATE_NULL 0
#define ERROR_STATE_LSS 1
#define ERROR_STATE_CONFIG 2
#define ERROR_STATE_OP  3
#define ERROR_STATE_SON 4
#define ERROR_STATE_VOLTAGE 5


#define ERROR_STATE_NULL_TXT "Pas d'erreur"
#define ERROR_STATE_LSS_TXT "Erreur LSS"
#define ERROR_STATE_CONFIG_TXT "Erreur Configuration"
#define ERROR_STATE_OP_TXT  "Erreur OP"
#define ERROR_STATE_SON_TXT "Erreur SON"
#define ERROR_STATE_VOLTAGE_TXT "Erreur tension"

// MOTOR ERROR CODE
#define MOTOR_ERROR_CAN_OVERRUN 0x8110
#define MOTOR_ERROR_CAN_PASSIVE 0x8120
#define MOTOR_ERROR_HEARTBEAT 0x8130
#define MOTOR_ERROR_BUSOFF_RECOVER 0x8140
#define MOTOR_ERROR_BUSOFF_PENDING 0x8141
#define MOTOR_ERROR_PDO 0x8210
#define MOTOR_ERROR_TEMP 0x4210
#define MOTOR_ERROR_IDLE 0xFF01

#define MOTOR_ERROR_CAN_OVERRUN_TXT "CAN network overrun"
#define MOTOR_ERROR_CAN_PASSIVE_TXT "CAN in error passive mode"
#define MOTOR_ERROR_HEARTBEAT_TXT "Heartbeat error"
#define MOTOR_ERROR_BUSOFF_RECOVER_TXT "Reset depuis l'état BUSOFF"
#define MOTOR_ERROR_BUSOFF_PENDING_TXT "Etat BUSOFF"
#define MOTOR_ERROR_PDO_TXT "Erreur de PDO"
#define MOTOR_ERROR_TEMP_TXT "Erreur de température"
#define MOTOR_ERROR_IDLE_TXT "Erreur Idle"


// ERROR TITLE
#define ERR_DRIVER_UP_TITLE "Chargement du driver"
#define ERR_DRIVER_LOAD_TITLE "Chargement de la librairie"
#define ERR_DRIVER_OPEN_TITLE "Ouverture du bus"
#define ERR_INIT_LOOP_RUN_TITLE "Boucle d'initialisation"
#define ERR_GUI_LOOP_RUN_TITLE "Interface graphique"

#define ERR_FILE_OPEN_TITLE "Ouverture de fichier"
#define ERR_FILE_EMPTY_TITLE "Fichier vide"
#define ERR_FILE_GEN_TITLE "Création de fichier"
#define ERR_FILE_SLAVE_DEF_TITLE "Fichier de définition des esclaves"
#define ERR_FILE_PROFILE_TITLE "Fichier de définition des profils"
#define ERR_FILE_PROFILE_INVALID_PARAM_TITLE "Paramètre invalide"

#define ERR_MASTER_STOP_SYNC_TITLE "Synchronisation"
#define ERR_MASTER_START_SYNC_TITLE "Synchronisation"
#define ERR_MASTER_SET_PERIOD_TITLE "Période de synchronisation"
#define ERR_MASTER_SET_HB_CONS_TITLE "Heartbeat MASTER"
#define ERR_MASTER_CONFIG_PDOR_TITLE "PDOR MASTER"
#define ERR_MASTER_CONFIG_PDOT_TITLE "PDOT MASTER"
#define ERR_MASTER_CONFIG_TITLE  "Configuration MASTER"

#define ERR_SLAVE_CONFIG_TITLE  "Configuration SLAVE"
#define ERR_SLAVE_CONFIG_LSS_TITLE  "Configuration LSS"
#define ERR_SLAVE_CONFIG_HB_TITLE  "Heartbeat SLAVE"
#define ERR_SLAVE_CONFIG_PDOT_TITLE "PDOT SLAVE"
#define ERR_SLAVE_CONFIG_PDOR_TITLE "PDOR SLAVE"
#define ERR_SLAVE_CONFIG_ACTIVE_PDO_TITLE "PDO SLAVE"
#define ERR_SLAVE_CONFIG_CURRENT_TITLE "Courant électrique"
#define ERR_SLAVE_CONFIG_PROFILE_TITLE "Mode d'utilisation"
#define ERR_SLAVE_CONFIG_MAX_VELOCITY_TITLE "Vitesse max"
#define ERR_SLAVE_CONFIG_ACCELERATION_TITLE "Accélération"
#define ERR_SLAVE_CONFIG_DECELERATION_TITLE "Décélération"
#define ERR_SLAVE_CONFIG_DECELERATION_QS_TITLE "Décélation rapide"
#define ERR_SLAVE_CONFIG_TORQUE_TITLE "Couple"
#define ERR_SLAVE_CONFIG_TORQUE_SLOPE_TITLE "Pente du couple"
#define ERR_SLAVE_CONFIG_TORQUE_MAX_VELOCITY_TITLE "Vitesse max"
#define ERR_SLAVE_CONFIG_MOTOR_SON_TITLE "Démarrage moteur"
#define ERR_SLAVE_SAVE_CONFIG_TITLE "Sauvegarde de la configuration"
#define ERR_SLAVE_LOAD_CONFIG_TITLE "Chargement des paramètres"
#define ERR_SLAVE_LOAD_INTERFACE_TITLE "GUI Paramètres"

#define ERR_READ_ACCELERATION_TITLE "Accélération"
#define ERR_READ_DECELERATION_TITLE "Décélération"
#define ERR_READ_DECELERATION_QS_TITLE "Décélération rapide"
#define ERR_READ_TORQUE_TITLE "Couple"
#define ERR_READ_TORQUE_SLOPE_TITLE "Pente du couple"
#define ERR_READ_TORQUE_VELOCITY_TITLE "Vitesse max en mode couple"
#define ERR_READ_CURRENT_TITLE "Courant électrique"
#define ERR_READ_PROFILE_TITLE "Mode d'utilisation"
#define ERR_READ_TORQUE_VELOCITY_MAKEUP_TITLE "Lecture vitesse make-up (couple)"
#define ERR_READ_HMT_ACTIVATE_TITLE "Lecture activation HMT"
#define ERR_READ_HMT_CONTROL_TITLE "Lecture controle HMT a échoué"

#define ERR_SET_ACCELERATION_TITLE "Accélération"
#define ERR_SET_DECELERATION_TITLE "Décélération"
#define ERR_SET_DECELERATION_QS_TITLE "Décélération rapide"
#define ERR_SET_VELOCITY_INC_TITLE "Vitesse"
#define ERR_SET_TORQUE_TITLE "Couple"
#define ERR_SET_TORQUE_SLOPE_TITLE "Pente de couple"
#define ERR_SET_TORQUE_VELOCITY_TITLE "Vitesse max en mode couple"
#define ERR_SET_CURRENT_TITLE "Courant électrique"
#define ERR_SET_PROFILE_TITLE "Mode d'utilisation"
#define ERR_SET_TORQUE_VELOCITY_MAKEUP_TITLE "Saisie vitesse make-up (couple)"
#define ERR_SET_HMT_ACTIVATE_TITLE "Saisie activation HMT"
#define ERR_SET_HMT_CONTROL_TITLE "Saisie controle HMT a échoué"
#define ERR_SET_PDO_TITLE "Saisie mapping PDO"

#define ERR_SAVE_ACCELERATION_TITLE "Accélération"
#define ERR_SAVE_DECELERATION_TITLE "Décélération"
#define ERR_SAVE_DECELERATION_QS_TITLE "Décélération rapide"
#define ERR_SAVE_TORQUE_TITLE "Couple"
#define ERR_SAVE_TORQUE_SLOPE_TITLE "Pente de couple"
#define ERR_SAVE_TORQUE_VELOCITY_TITLE "Vitesse max en mode couple"
#define ERR_SAVE_CURRENT_TITLE "Courant électrique"
#define ERR_SAVE_PROFILE_TITLE "Mode d'utilisation"
#define ERR_SAVE_TORQUE_VELOCITY_MAKEUP_TITLE "Sauvegarde vitesse make-up (couple)"
#define ERR_SAVE_HMT_ACTIVATE_TITLE "Sauvegarde activation HMT"
#define ERR_SAVE_HMT_CONTROL_TITLE "Sauvegarde controle HMT a échoué"

#define ERR_MOTOR_PAUSE_TITLE "Mise en pause du moteur"
#define ERR_MOTOR_RUN_TITLE "Mise en mouvement du moteur"
#define ERR_MOTOR_OFF_TITLE "Extinction du moteur"
#define ERR_MOTOR_FORWARD_TITLE "Changement de sens"
#define ERR_MOTOR_BACKWARD_TITLE "Changement de sens"
#define ERR_MOTOR_LOW_VOLTAGE_TITLE "Baisse de tension"

// ERROR CONTENT
#define ERR_DRIVER_UP_CONTENT "Le chargement du driver SOCKET CAN a échoué"
#define ERR_DRIVER_LOAD_CONTENT "Le chargement de la librairie CANFESTIVAL a échoué"
#define ERR_DRIVER_OPEN_CONTENT "L'ouverture du bus can pour l'envoie d'information a échoué"
#define ERR_INIT_LOOP_RUN_CONTENT "Le lancement de la boucle d'initialisation a échoué"
#define ERR_GUI_LOOP_RUN_CONTENT "Le lancement de l'interface graphique a échoué"

#define ERR_FILE_OPEN_CONTENT "L'Ouverture du fichier a échoué, un nouveau fichier va donc être généré."
#define ERR_FILE_EMPTY_CONTENT "Le fichier listé ci-dessous est vide"
#define ERR_FILE_GEN_CONTENT "La génération du fichier a échoué."
#define ERR_FILE_PROFILE_INVALID_PARAM_CONTENT "Un paramètre présent de le fichier n'a pas été reconnu. Le profil a été réinitialisé."

#define ERR_FILE_SLAVE_DEF_CONTENT "La lecture du fichier definissant les esclaves a échoué ou celui-ci ne contient aucune donnée."
#define ERR_FILE_PROFILE_CONTENT "Fichier de définition des profils"

#define ERR_MASTER_STOP_SYNC_CONTENT "L'arret de la synchronisation a échoué"
#define ERR_MASTER_START_SYNC_CONTENT "Le démarrage de la synchronisation a échoué"
#define ERR_MASTER_SET_PERIOD_CONTENT "L'affectation de la période de synchronisation a échoué"
#define ERR_MASTER_SET_HB_CONS_CONTENT "L'affectation du heartbeat dans le dictionnaire du maitre a échoué"
#define ERR_MASTER_CONFIG_PDOR_CONTENT "La configuration de la PDO récepteur du maitre a échoué"
#define ERR_MASTER_CONFIG_PDOT_CONTENT "La configuration de la PDO transmetteur du maitre a échoué"
#define ERR_MASTER_CONFIG_CONTENT "La configuration du maitre a échoué"

#define ERR_SLAVE_CONFIG_CONTENT "La configuration d'un esclave a échoué"
#define ERR_SLAVE_CONFIG_LSS_CONTENT "La configuration LSS a échoué"
#define ERR_SLAVE_CONFIG_HB_CONTENT "L'affectation du heartbeat dans le dictionnaire esclave a échoué"
#define ERR_SLAVE_CONFIG_PDOT_CONTENT "La configuration de la PDO transmetteur de l'esclave a échoué"
#define ERR_SLAVE_CONFIG_PDOR_CONTENT  "La configuration de la PDO recepteur de l'esclave a échoué"
#define ERR_SLAVE_CONFIG_ACTIVE_PDO_CONTENT "L'activation d'une PDO a échoué"
#define ERR_SLAVE_CONFIG_CURRENT_CONTENT "L'affectation du courant électrique de fonctionnement pour un esclave a échoué"
#define ERR_SLAVE_CONFIG_PROFILE_CONTENT "L'affectation du mode d'utilisation pour un esclave a échoué"
#define ERR_SLAVE_CONFIG_MAX_VELOCITY_CONTENT "L'affectation de la vitesse max pour un esclave a échoué"
#define ERR_SLAVE_CONFIG_ACCELERATION_CONTENT "L'affectation de l'accélération pour un esclave a échoué"
#define ERR_SLAVE_CONFIG_DECELERATION_CONTENT "L'affectation de la décélération pour un esclave a échoué"
#define ERR_SLAVE_CONFIG_DECELERATION_QS_CONTENT "L'affectation de la décélération rapide pour un esclave a échoué"
#define ERR_SLAVE_CONFIG_TORQUE_CONTENT "L'affectation du couple pour un esclave a échoué"
#define ERR_SLAVE_CONFIG_TORQUE_SLOPE_CONTENT "L'affectation de la pente de montée en couple pour un esclave a échoué"
#define ERR_SLAVE_CONFIG_TORQUE_MAX_VELOCITY_CONTENT "L'affectation de vitesse max en mode couple pour un esclave a échoué"
#define ERR_SLAVE_CONFIG_MOTOR_SON_CONTENT "La mise en marche d'un esclave a échoué"
#define ERR_SLAVE_SAVE_CONFIG_CONTENT "La sauvegarde des paramètres de configuration a échoué pour un des esclaves"
#define ERR_SLAVE_LOAD_CONFIG_CONTENT "La lecture des paramètres de configuration a échoué"
#define ERR_SLAVE_LOAD_INTERFACE_CONTENT "Le chargement de l'interface paramètre a échoué'"

#define ERR_READ_ACCELERATION_CONTENT "La lecture de l'accélération d'un des moteurs a échoué"
#define ERR_READ_DECELERATION_CONTENT "La lecture de la décélération d'un des moteurs a échoué"
#define ERR_READ_DECELERATION_QS_CONTENT  "La lecture de la décélération rapide d'un des moteurs a échoué"
#define ERR_READ_TORQUE_CONTENT "La lecture du couple d'un des moteurs a échoué"
#define ERR_READ_TORQUE_SLOPE_CONTENT "La lecture de la pente de couple d'un des moteurs a échoué"
#define ERR_READ_TORQUE_VELOCITY_CONTENT "La lecture de la vitesse en mode couple couple d'un des moteurs a échoué"
#define ERR_READ_CURRENT_CONTENT "La lecture du courant électrique d'un des moteurs a échoué"
#define ERR_READ_PROFILE_CONTENT "La lecture du mode d'utilisation d'un des moteurs a échoué"
#define ERR_READ_TORQUE_VELOCITY_MAKEUP_CONTENT "La lecture de la vitesse make-up (couple) a échoué"
#define ERR_READ_HMT_ACTIVATE_CONTENT "La lecture de l'activation HMT a échoué"
#define ERR_READ_HMT_CONTROL_CONTENT "La lecture du controle HMT a échoué"

#define ERR_SET_ACCELERATION_CONTENT "L'accélération saisie est invalide"
#define ERR_SET_DECELERATION_CONTENT "La deceleration saisie est invalide"
#define ERR_SET_DECELERATION_QS_CONTENT "La deceleration rapide saisie est invalide"
#define ERR_SET_VELOCITY_INC_CONTENT "La vitesse saisie est invalide"
#define ERR_SET_TORQUE_CONTENT "Le couple saisie est invalide"
#define ERR_SET_TORQUE_SLOPE_CONTENT "La pente de couple saisie est invalide"
#define ERR_SET_TORQUE_VELOCITY_CONTENT "La vitesse en mode couple saisie est invalide"
#define ERR_SET_CURRENT_CONTENT "Le courant électrique saisie est invalide"
#define ERR_SET_PROFILE_CONTENT "Le mode d'utilisation saisie est invalide"
#define ERR_SET_PDO_CONTENT "Le PDO mapping saisie est invalide."
#define ERR_SET_TORQUE_VELOCITY_MAKEUP_CONTENT "La saisie de la vitesse make-up (couple) est invalide"
#define ERR_SET_HMT_ACTIVATE_CONTENT "La saisie de l'activation HMT est invalide"
#define ERR_SET_HMT_CONTROL_CONTENT "La saisie du controle HMT a échoué"

#define ERR_SAVE_ACCELERATION_CONTENT "La sauvegarde de l'accélération a échoué"
#define ERR_SAVE_DECELERATION_CONTENT "La sauvegarde de la décélération a échoué"
#define ERR_SAVE_DECELERATION_QS_CONTENT "La sauvegarde de la décélation rapide a échoué"
#define ERR_SAVE_TORQUE_CONTENT "La sauvegarde du couple a échoué"
#define ERR_SAVE_TORQUE_SLOPE_CONTENT "La sauvegarde de la pente de couple a échoué"
#define ERR_SAVE_TORQUE_VELOCITY_CONTENT "La sauvegarde de la vitesse en mode couple a échoué"
#define ERR_SAVE_CURRENT_CONTENT "La sauvegarde du courant électrique a échoué"
#define ERR_SAVE_PROFILE_CONTENT "La sauvegarde du profil moteur a échoué"
#define ERR_SAVE_TORQUE_VELOCITY_MAKEUP_CONTENT "La sauvegarde de la vitesse make-up (couple) a échoué"
#define ERR_SAVE_HMT_ACTIVATE_CONTENT "La sauvegarde de l'activation HMT  a échoué"
#define ERR_SAVE_HMT_CONTROL_CONTENT "La sauvegarde du controle HMT a échoué"

#define ERR_MOTOR_PAUSE_CONTENT "La mise en pause d'un 'des moteurs a échoué"
#define ERR_MOTOR_RUN_CONTENT "La mise en mouvement d'un des moteurs a échoué"
#define ERR_MOTOR_OFF_CONTENT "L'extinction d'un des moteurs a échoué"
#define ERR_MOTOR_FORWARD_CONTENT "Le changement de sens du moteur vers l'avant a échoué"
#define ERR_MOTOR_BACKWARD_CONTENT "Le changement de sens du moteur vers l'arrière a échoué"
#define ERR_MOTOR_LOW_VOLTAGE_CONTENT "Un moteur a subi une baisse de tension ou a été débranché. Le moteur a été désactivé."

#endif // _SIGNAUX_H
