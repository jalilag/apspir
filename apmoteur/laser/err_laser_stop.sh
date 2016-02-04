#!/bin/bash
## a mettre dans /bin/bash comme socket_conf
##remplacer evlt les chemins absolue
echo "cript started"
gcc main_test.c -o run
./run
echo "toto generated"
gcc toto.c -o laser_stop
gcc toto2.c -o laser_stop1
echo "compile done"
sudo ./laser_stop
sudo ./laser_stop
sudo ./laser_stop1
sudo ./laser_stop1
echo "exec compiled code done"
rm laser_stop laser_stop1 run
rm toto.c toto2.c
echo "files deleted"
