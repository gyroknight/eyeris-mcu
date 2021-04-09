#!/bin/bash
cd /home/pi/eyeris
flock -xn run.lck -c "sudo /home/pi/eyeris/build/eyeris"
