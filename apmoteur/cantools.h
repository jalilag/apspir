#ifndef _CANTOOLS_H
#define _CANTOOLS_H
#include "canfestival.h"
#include <gtk/gtk.h>


typedef struct SDOR SDOR;
struct SDOR {
	UNS16 index;
	UNS8 subindex;
	UNS8 type; // as defined in objdictdef.h
};

int cantools_read_sdo(UNS8 nodeid,SDOR sdo, void* data);
int cantools_write_sdo(UNS8 nodeid,SDOR sdo, void* data);
int cantools_write_local(UNS16 Mindex, UNS8 Msubindex, void* data, UNS32 datsize);
int cantools_PDO_trans(UNS8 nodeID, UNS16 index, UNS8 trans,UNS16 inhibit,UNS16 event);
int cantools_PDO_map_config(UNS8 nodeID, UNS16 PDOMapIndex,...);
int cantools_sync(int start);
//ajout 220116

int cantools_PDO_map_local_config(UNS16 PDOMapIndex, ...);
int cantools_desactive_PDO_reseau(UNS8 nodeID, UNS16 index);
void cantools_ApplyVelRot(INTEGER32 vit);
void cantools_ApplyVelTrans(INTEGER32 vit);

//fin ajout 220116

gpointer cantools_init_loop(gpointer data);

#endif
