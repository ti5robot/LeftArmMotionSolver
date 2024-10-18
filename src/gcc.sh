#!/bin/bash
export CPLUS_INCLUDE_PATH=~/LeftArmMotionSolver/include:$CPLUS_INCLUDE_PATH
export LD_LIBRARY_PATH=$LD_LIBRARY_PATH:~/LeftArmMotionSolver/include/can
g++ main.cpp  -L./include -lmylibti5 -L./include/can -I/usr/include/python3.10 -lpython3.10 -lmylibscan -lcontrolcan -lspdlog -lfmt -ludev -o move_sov
