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
#include "communication.h"
#include "mathfunc.h"
#include "tool.h"

#include "electrical_machinery.h"

#include <csignal>

#define RESET "\033[0m"
#define RED "\033[31m"     /* Red */
#define GREEN "\033[32m"   /* Green */
#define YELLOW "\033[33m"  /* Yellow */
#define BLUE "\033[34m"    /* Blue */
#define MAGENTA "\033[35m" /* Magenta */
#define CYAN "\033[36m"    /* Cyan */

using namespace std;

string filename;
// string device_485_name;
char device[] = "/dev/ttyUSB0";

pthread_mutex_t mutex = PTHREAD_MUTEX_INITIALIZER;
pthread_cond_t cond = PTHREAD_COND_INITIALIZER;
int thread_flag = 0;
std::string ip_test = "192.168.130.11"; // socket通信测试参数

void signalHandler(int signum)
{
    char aaa;
    cout <<RED<< "Interrupt signal (" << signum << ") received.\n"<<RESET ;
    brake();
    cout <<RED<< "stop!!" << RESET << endl;
    inspect_brake();

    logout();
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

    login();
    cout << "login success" << endl;
    // float goal_j[7]={0,-1.3,0.2,-0.6,-0.1,-0.2,0.3};TH.forward_move(goal_j);TH.show_crtpos();
    // plan_move();sleep(2);
    // sleep(4);
    /*
    //float goal_j[7]={3.00159,1.3,-0.999998,0.799998,-2.44159,0.4,-0.500003};TH.forward_move(goal_j);TH.show_crtpos();
    float goal_j[7]={0};
    for(int i=0;i<7;i++){
        goal_j[i]=0.5;
        TH.forward_move(goal_j);//TH.show_crtpos();
        plan_move();sleep(2);
        goal_j[i]=0;
        TH.forward_move(goal_j);//TH.show_crtpos();
        plan_move();sleep(2);
    }

    */

    float pos[6] = {150, 266.047, -400, -1.89695, 2.8, 0.826565};
    float value = 237.875;
    int dim = 1;
    bool absolute = true;
    float saveJoint[7];
    for (int i = 0; i < 3; i++)
    {

        move_to_pos(pos, value, dim, absolute);
        TH.show_crtj();
        TH.showpointsinfo();
        TH.show_crtpos();
        write_value(1,"1.txt",saveJoint);
        cout << "step1-------------------" << endl;

        value = 60;
        dim = 1;
        absolute = false;
        move_to_pos(pos, value, dim, absolute);
        TH.show_crtj();
        TH.showpointsinfo();
        TH.show_crtpos();
        cout << "step2-------------------" << endl;

        pos[0] += 80, pos[1] += 10, pos[2] += 80;
        value = 237.875;
        dim = 1;
        absolute = true;
        move_to_pos(pos, value, dim, absolute);
        TH.show_crtj();
        TH.showpointsinfo();
        TH.show_crtpos();
        cout << "step3-------------------" << endl;
    }

    float goal_j2[7]={3.14099,1.17288,-0.608523,0.799968,-2.9569,0.478535,-0.682677};
    joint_to_move(goal_j2);
    sleep(1);
    mechanical_arm_origin();
    logout();
    return 0;
}
