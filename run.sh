#!/bin/bash
cd /home/pi/eyeris
flock -xn run.lck -c "/home/pi/eyeris/build/eyeris"
