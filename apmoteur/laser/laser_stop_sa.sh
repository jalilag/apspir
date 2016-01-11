#!/bin/bash
## donner les droits de root aux scripts avec visudo p.ex.
echo "cript started"
gcc toto.c -o laser_stop_sa
echo "compile done"
sudo ./laser_stop_sa
sudo ./laser_stop_sa
echo "exec compiled code done"
sudo rm laser_stop_sa
sudo rm toto.c
echo "files deleted"
