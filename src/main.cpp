#include <stdio.h>
#include <cstring>
#include <stdlib.h>
#include <sstream>
#include <fstream>
#include <cmath>
#include <iostream>
#include <unistd.h>
#include <cstdlib>

#include "Ti5MOVE.h"
#include "Ti5BASIC.h"
#include "Ti5LOGIC.h"
#include "mathfunc.h"
#include "tool.h"
#include <csignal>

#define RESET "\033[0m"
#define RED "\033[31m"     /* Red */
#define GREEN "\033[32m"   /* Green */
#define YELLOW "\033[33m"  /* Yellow */
#define BLUE "\033[34m"    /* Blue */
#define MAGENTA "\033[35m" /* Magenta */
#define CYAN "\033[36m"    /* Cyan */

using namespace std;

void signalHandler(int signum)
{
    char aaa;
    cout <<RED<< "Interrupt signal (" << signum << ") received.\n"<<RESET ;
    brake();
    cout <<RED<< "stop!!" << RESET << endl;
    inspect_brake();

    Exit();
    exit(signum);
}


int main()
{
    NMAX = 3000;

    vector<string> productSerialNumbers = query_can();
    if (productSerialNumbers.empty())
    {
        cout << RED << "未找到任何 USB 设备，请插入设备后重试！" << RESET << endl;
        exit(0);
    }
    else
    {
        cout << CYAN << "找到的 CAN 设备序列号：" << RESET;
        for (const string &serialNumber : productSerialNumbers)
        {
            cout << CYAN << serialNumber << RESET << endl;
        }
    }

    string ip = ip_address();
    cout << MAGENTA << "ip=" << ip << RESET << endl;

    cout << "qweqwdsaradasd" << endl;
    signal(SIGINT, signalHandler);

    Start();
    cout << "login success" << endl;


    float pos[6] = {150, 266.047, -400, -1.89695, 2.8, 0.826565};
    float value = 237.875;
    int dim = 1;
    bool absolute = true;
    float saveJoint[7];
    for (int i = 0; i < 3; i++)
    {

        pos_to_move(pos, value, dim, absolute);
        TH.show_crtj();
        TH.showpointsinfo();
        TH.show_crtpos();
        write_value(1,"1.txt",saveJoint);
        cout << "step1-------------------" << endl;

        value = 60;
        dim = 1;
        absolute = false;
        pos_to_move(pos, value, dim, absolute);
        TH.show_crtj();
        TH.showpointsinfo();
        TH.show_crtpos();
        cout << "step2-------------------" << endl;

        pos[0] += 80, pos[1] += 10, pos[2] += 80;
        value = 237.875;
        dim = 1;
        absolute = true;
        pos_to_move(pos, value, dim, absolute);
        TH.show_crtj();
        TH.showpointsinfo();
        TH.show_crtpos();
        cout << "step3-------------------" << endl;
    }

    float goal_j2[7]={3.14099,1.17288,-0.608523,0.799968,-2.9569,0.478535,-0.682677};
    joint_to_move(goal_j2);
    sleep(1);
    mechanical_arm_origin();
    Exit();
    return 0;
}
