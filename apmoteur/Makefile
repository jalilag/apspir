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

CC = gcc
CXX = g++
LD = g++
GTK_CFLAGS = $$(pkg-config --cflags gtk+-3.0)
GTK_LDFLAGS = $$(pkg-config --libs gtk+-3.0)
OPT_CFLAGS = -g -O2
CFLAGS = $(OPT_CFLAGS) $(GTK_CFLAGS)
LDFLAGS = $(OPT_CFLAGS)
PROG_CFLAGS =  -fPIC -DCO_ENABLE_LSS
EXE_CFLAGS =  -lpthread -lrt -ldl $(GTK_LDFLAGS) -lX11
OS_NAME = Linux
ARCH_NAME = x86_64
PREFIX = /usr/local
TARGET = unix
CAN_DRIVER = can_virtual
TIMERS_DRIVER = timers_unix
APMOTEUR = "SpirallingControl"
FUNCVAR = slave.o master.o gtksig.o gui.o strtools.o cantools.o lsstools.o od_callback.o keyword.o motor.o errgen.o profile.o serialtools.o laser/laser.o asserv.o

INCLUDES = -I./laser -I../include -I../include/$(TARGET) -I../include/$(CAN_DRIVER) -I../include/$(TIMERS_DRIVER)

MASTER_OBJS = SpirallingMaster.o SpirallingControl.o $(FUNCVAR)

OBJS = $(MASTER_OBJS) ../src/libcanfestival.a ../drivers/$(TARGET)/libcanfestival_$(TARGET).a


Debug: $(APMOTEUR)

../drivers/$(TARGET)/libcanfestival_$(TARGET).a:
	$(MAKE) -C  ../drivers/$(TARGET) libcanfestival_$(TARGET).a


$(APMOTEUR): $(OBJS)
	$(LD) $(LDFLAGS) $(PROG_CFLAGS) ${PROGDEFINES} $(INCLUDES) -o $@ $(OBJS) $(EXE_CFLAGS)


SpirallingMaster.c: SpirallingMaster.od
	$(MAKE) -C /home/jalil/Documents/programmation/python/objedit/ gnosis
	python /home/jalil/Documents/programmation/python/objedit/objdictgen.py SpirallingMaster.od SpirallingMaster.c

%.o: %.c
	$(CC) $(CFLAGS) $(PROG_CFLAGS) ${PROGDEFINES} $(INCLUDES)  -o $@ -c $<

clean:
	rm -f $(MASTER_OBJS)
	rm -f $(APMOTEUR)
	rm -f SpirallingMaster.c

mrproper: clean
	rm -f SpirallingMaster.c

install: Apmoteur
	mkdir -p $(DESTDIR)$(PREFIX)/bin/
	cp $< $(DESTDIR)$(PREFIX)/bin/

uninstall:
	rm -f $(DESTDIR)$(PREFIX)/bin/apmoteur
