#! gmake

#
# Copyright (C) 2006 Laurent Bessard
# 
# This file is part of canfestival, a library implementing the canopen
# stack
# 
# This library is free software; you can redistribute it and/or
# modify it under the terms of the GNU Lesser General Public
# License as published by the Free Software Foundation; either
# version 2.1 of the License, or (at your option) any later version.
# 
# This library is distributed in the hope that it will be useful,
# but WITHOUT ANY WARRANTY; without even the implied warranty of
# MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the GNU
# Lesser General Public License for more details.
# 
# You should have received a copy of the GNU Lesser General Public
# License along with this library; if not, write to the Free Software
# Foundation, Inc., 59 Temple Place, Suite 330, Boston, MA  02111-1307  USA
# 

TARGET = unix
CAN_DRIVER = can_socket
TIMERS_DRIVER = timers_unix

all: driver

driver:

ifneq ($(TIMERS_DRIVER),timers_)
	$(MAKE) -C $(TIMERS_DRIVER) $@
endif

ifneq ($(CAN_DRIVER),can_)
	$(MAKE) -C $(CAN_DRIVER) $@
endif
ifneq ($(TARGET),none)
	$(MAKE) -C $(TARGET) $@
endif

install:

ifneq ($(TIMERS_DRIVER),timers_)
	$(MAKE) -C $(TIMERS_DRIVER) $@
endif

ifneq ($(CAN_DRIVER),can_)
	$(MAKE) -C $(CAN_DRIVER) $@
endif
ifneq ($(TARGET),none)
	$(MAKE) -C $(TARGET) $@
endif

uninstall:

ifneq ($(TIMERS_DRIVER),timers_)
	$(MAKE) -C $(TIMERS_DRIVER) $@
endif

ifneq ($(CAN_DRIVER),can_)
	$(MAKE) -C $(CAN_DRIVER) $@
endif
ifneq ($(TARGET),none)
	$(MAKE) -C $(TARGET) $@
endif

clean:

ifneq ($(TIMERS_DRIVER),timers_)
	$(MAKE) -C $(TIMERS_DRIVER) $@
endif

ifneq ($(CAN_DRIVER),can_)
	$(MAKE) -C $(CAN_DRIVER) $@
endif
ifneq ($(TARGET),none)
	$(MAKE) -C $(TARGET) $@
endif

mrproper: clean

ifneq ($(TIMERS_DRIVER),timers_)
	$(MAKE) -C $(TIMERS_DRIVER) $@
endif

ifneq ($(CAN_DRIVER),can_)
	$(MAKE) -C $(CAN_DRIVER) $@
endif
ifneq ($(TARGET),none)
	$(MAKE) -C $(TARGET) $@
endif


