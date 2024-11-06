#!/bin/bash
g++ main.cpp  -L./include -lmylibti5 -L./include/can  -lmylibscan -lcontrolcan  -lfmt -ludev -o move_sov
