#!/bin/bash

sudo rm -rf /opt/ros/indigo/include/ompl
sudo ln -s /usr/local/include/ompl/ /opt/ros/indigo/include/ompl
sudo rm /opt/ros/indigo/lib/x86_64-linux-gnu/libompl.so*
sudo ln -s /usr/local/lib/x86_64-linux-gnu/libompl.so /opt/ros/indigo/lib/x86_64-linux-gnu/libompl.so
sudo ln -s /usr/local/lib/x86_64-linux-gnu/libompl.so.12 /opt/ros/indigo/lib/x86_64-linux-gnu/libompl.so.12
sudo ln -s /usr/local/lib/x86_64-linux-gnu/libompl.so.1.2.1 /opt/ros/indigo/lib/x86_64-linux-gnu/libompl.so.1.2.1
