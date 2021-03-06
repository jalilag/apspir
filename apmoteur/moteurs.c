/*
This file is part of CanFestival, a library implementing CanOpen Stack.

Copyright (C): Edouard TISSERANT and Francis DUPIN

See COPYING file for copyrights details.

This library is free software; you can redistribute it and/or
modify it under the terms of the GNU Lesser General Public
License as published by the Free Software Foundation; either
version 2.1 of the License, or (at your option) any later version.

This library is distributed in the hope that it will be useful,
but WITHOUT ANY WARRANTY; without even the implied warranty of
MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the GNU
Lesser General Public License for more details.

You should have received a copy of the GNU Lesser General Public
License along with this library; if not, write to the Free Software
Foundation, Inc., 59 Temple Place, Suite 330, Boston, MA  02111-1307  USA
*/

#include "moteurs.h"
#include "Master.h"
#include "Apmoteur.h"
#include "motor.h"
#include "canfestival.h"
#include "gui.h"
#include "strtools.h"
#include "keyword.h"




extern s_BOARD SlaveBoard;
/*****************************************************************************/
void moteur_heartbeatError(CO_Data* d, UNS8 heartbeatID) {
	eprintf("Slave_heartbeatError %d (NODE : %d)\n",heartbeatID,getNodeId(d));
}

void moteur_initialisation(CO_Data* d) {
	eprintf("Slave_initialisation (NODE : %d) \n",getNodeId(d));
}

void moteur_preOperational(CO_Data* d) {
	eprintf("Slave_preOperational (NODE : %d)\n",getNodeId(d));

	if(!motor_switch_off(getNodeId(d),StatusWord_V)) {
        gui_info_box(START_ERROR_TITLE,START_ERROR_CONTENT, NULL);
	} else {
       // motorConfig();
	}


}

void moteur_operational(CO_Data* d) {
	eprintf("Slave_Operational (NODE : %d)\n",getNodeId(d));
	// CONFIGURATION

    if (motor_switch_on(getNodeId(d))) {
        gui_push_state(SWITCH_ON_SUCCESS);
    }
}

void moteur_stopped(CO_Data* d) {
	eprintf("Slave_stopped (NODE : %d)\n",getNodeId(d));

}
void moteur_post_sync(CO_Data* d) {
      eprintf("Slave_post_sync (NODE : %d)\n",getNodeId(d));
}

void moteur_post_TPDO(CO_Data* d) {
	eprintf("Slave_post_TPDO (NODE : %d)\n",getNodeId(d));

	/* send an error and recover inmediately every 12 cycles */
/*	if(SlaveMap13 % 12 == 0)
	{
		EMCY_setError(d, 0x4200, 0x08, 0x0000);
		EMCY_errorRecovered(d, 0x4200);
	}
*/
}

void moteur_storeODSubIndex(CO_Data* d, UNS16 wIndex, UNS8 bSubindex) {
	/*TODO :
	 * - call getODEntry for index and subindex,
	 * - save content to file, database, flash, nvram, ...
	 *
	 * To ease flash organisation, index of variable to store
	 * can be established by scanning d->objdict[d->ObjdictSize]
	 * for variables to store.
	 *
	 * */
	eprintf("Slave_storeODSubIndex : %4.4x %2.2x\n", wIndex,  bSubindex);
}

void moteur_post_emcy(CO_Data* d, UNS8 nodeID, UNS16 errCode, UNS8 errReg) {
	eprintf("Slave received EMCY message. Node: %2.2x  ErrorCode: %4.4x  ErrorRegister: %2.2x\n", nodeID, errCode, errReg);
}
