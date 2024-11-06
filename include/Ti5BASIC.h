#ifndef Ti5BASIC_H
#define Ti5BASIC_H

#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include <sys/types.h>
#include <sys/socket.h>
#include <ifaddrs.h>
#include <netinet/in.h>
#include <arpa/inet.h>
#include <iostream>
#include <fstream>
#include <string>
#include <unordered_map>
#include <sstream>
#include <map>
#include <unistd.h>
#include <fcntl.h>
#include <termios.h>
#include <sys/stat.h>
#include <sys/mman.h>
#include <vector>
#include <regex>

#include "Ti5LOGIC.h"
#include "Ti5MOVE.h"

#include "can/tcontrolcanfactor.h"

#define MAX_IP_ADDR_LEN 256 // 存储IP地址的最大长度

// extern float arr_s[6];

extern bool flag;

extern string log_path; // log文件

extern char LogInfo[100]; // 存储写入log文件的信息

bool get_Parameter(uint32_t *parameterList, int parameterType, int nodeCount);
bool get_Parameter(uint8_t *nodeList, uint32_t *parameterList, int parameterType, int nodeCount);
bool set_Parameter(uint32_t *parameterList, int parameterType, int nodeCount);
bool set_Parameter(uint8_t *nodeList, uint32_t *parameterList, int parameterType, int nodeCount);

extern "C"
{ // 添加extern "C"

  // 写入调试信息到文件
  void writeDebugInfoToFile(const char *func_name, const char *info);

  // 输出数组的调试信息
  void printArrayDebugInfo(float arr[], int size, const char *arr_name);

  // 查询机械臂是否停止运动
  bool inspect_brake();

  void exit_progrem(); // 退出程序

  std::string ip_address();

  bool Start(); // 登录can设备

  bool Exit(); // 登出can设备

  /*std::string query_can();
  查询can设备号
  */
  std::vector<std::string> query_can();

    /*获取电机错误状态
    返回值：为电机错误
      0：无错误
      1：软件错误
      2：过压
      4：欠压
      16：启动错误
  */
  int get_elektrische_Maschinen_status();

  /*清除电机错误*/
  void clear_elc_error();

} // 添加extern "C"
#endif
