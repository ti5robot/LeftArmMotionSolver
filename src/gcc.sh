#!/bin/bash
export CPLUS_INCLUDE_PATH=/home/ti5robot/mechanical_arm_7_axis/include:$CPLUS_INCLUDE_PATH
export LD_LIBRARY_PATH=$LD_LIBRARY_PATH:/home/ti5robot/mechanical_arm_7_axis/include/can
g++ main.cpp communication.cpp mathfunc.cpp Ti5BASIC.cpp  Ti5MOVE.cpp Ti5LOGIC.cpp tool.cpp     electrical_machinery.cpp  -L../include/can  -I/usr/include/python3.10 -lpython3.10 -lmylibscan -lcontrolcan -lspdlog -lfmt -ludev  -o move_sov
