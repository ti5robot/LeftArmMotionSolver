#Ti5robot

欢迎您使用Ti5robot机械臂，并感谢您的购买。

本文档记载了有关机械臂的安装、调试、以及如何基于API进行二次开发的相关信息。

机械臂使用人员应充分了解风险，使用前必须认真阅读本手册，严格遵守手册中的规范和要求。

## 简介

机械臂具有开放性的程序接口和拓展接口，机械臂末端可快速换装不同执行器；能够用于电商物流、新消费、日常生活等多种场景。

## 注意事项

1.请务必按照本手册中的要求安装机械臂、连接线缆

2.确保机械臂的活动范围内不会碰撞到人或其他物品，以免发生意外

3.在使用前，需要专业的人员进行调试

4.在使用SDK时，必须确保输入的参数和操作流程是正确的

5.请注意机械臂运行速度，过快时务必小心

6.机械臂使用结束后，请务必断电

7.机械臂断电后，请务必将控制程序关闭

8.避免在潮湿或粉尘的环境下使用机械臂

9.请务必将机械臂存放、安装到儿童碰不到的地方，以免发生危险


# SDK介绍

机械臂控制的代码code中，分别是`include`，`src`，`log`以及`usrlib`。

+ [include] 存储着机械臂所需的头文件。
+ [src] 一般控制机械臂的文件放在此处，其中`main.cpp`是一个示例程序。
+ [log]sdk中存放log的文件夹。
+ [usrlib]包含SDK所需的so文件

## 1. include

除以下提到的文件外，用户无需查看该文件夹下的其他文件。

### 1.1 mathfunc.h
机械臂的数学模型函数


### 1.2 Ti5BASIC.h

机械臂控制基础库，包含了基本控制以及信息，用户在使用时需要根据自身使用方式自行选择调用。

+ bool inspect_brake();
  ```
  函数功能：查询机械臂是否停止运动
  返回值：true停止，false未停止
  参数：无
  示例：bool result=inspect_brake()
  ```
+ void exit_progrem();//函数功能：退出程序
  ```
  函数功能：查询机械臂是否停止运动
  返回值：true停止，false未停止
  参数：无
  示例：
      int main()
      {
          Start();
          cout<<"login success"<<endl;
          mechanical_arm_origin();
          sleep(3);
          show_value("pos:",TH.pos);
          cout<<endl;
          show_value("TH.j= ",TH.j);
          exit_progrem();
          Exit();
          return 0;
      }
  ```
+ void writeDebugInfoToFile(const char *func_name, const char *info);
  ```
  函数功能：将信息写入log中
  返回值：无
  参数：
      *func_name：函数名字
      *info：要写入log的信息内容
  示例：
      getCurrentposition->getParameter(canidList, reg_min_app_position, MotorTypeHelper::REG_MIN_APP_POSITION, IDNUM);
      for (int i = 0; i < 6; i++)
      {
            cout << "电机" << i << "最大负向位置: " << static_cast<int32_t>(reg_min_app_position[i]) << endl;
            sprintf(LogInfo, "电机%d最大正向位置: %d", i, static_cast<int32_t>(reg_min_app_position[i]));
            writeDebugInfoToFile(__func__, LogInfo);
      }
      cout << endl;
  ```

+ void Start();
  ```
  函数功能：登录can设备
  返回值：无
  参数：无
  示例：
      Start();
      getCurrentposition->getParameter(canidList, reg_min_app_position, MotorTypeHelper::REG_MIN_APP_POSITION, IDNUM);
      for (int i = 0; i < 6; i++)
      {
            cout << "电机" << i << "最大负向位置: " << static_cast<int32_t>(reg_min_app_position[i]) << endl;
            sprintf(LogInfo, "电机%d最大正向位置: %d", i, static_cast<int32_t>(reg_min_app_position[i]));
            writeDebugInfoToFile(__func__, LogInfo);
      }
      cout << endl;
  ```

+ void Exit();//函数功能：登出can设备
    ```
  函数功能：登录can设备
  返回值：无
  参数：无
  示例：
      Start();
      getCurrentposition->getParameter(canidList, reg_min_app_position, MotorTypeHelper::REG_MIN_APP_POSITION, IDNUM);
      for (int i = 0; i < 6; i++)
      {
            cout << "电机" << i << "最大负向位置: " << static_cast<int32_t>(reg_min_app_position[i]) << endl;
            sprintf(LogInfo, "电机%d最大正向位置: %d", i, static_cast<int32_t>(reg_min_app_position[i]));
            writeDebugInfoToFile(__func__, LogInfo);
      }
      cout << endl;
      Exit();
  ```

+ std::vector<std::string> query_can();
  ```
  函数功能：查询can设备号
  返回值：无
  参数：无
  示例：
      vector<string> productSerialNumbers = query_can();
      if (productSerialNumbers.empty()) {
        cout << RED<<"未找到任何 USB 设备，请插入设备后重试！" <<RESET << endl;
        exit(0);
      } else {
        cout <<CYAN<< "找到的 CAN 设备序列号：" <<RESET;
        for (const string& serialNumber : productSerialNumbers) {
            cout << CYAN <<serialNumber <<RESET << endl;
        }
      }
  ```

### 1.3 Ti5LOGIC.h

该文件是机械臂的算法库，包括正逆解，碰撞检测，规划路径等。
使用方法：根据需求在规划机械臂运动的时候可以调用该库的函数。

### 1.4 Ti5MOVE.h

机械臂运动控制库，包含的功能有直线运动，圆弧运动，机械臂初始化，机械臂手动模式控制，机械臂关节运动，机械臂坐标运动，获取机械臂当前角度值，获取机械臂当前位姿，机械臂刹车，查看机械臂是否停止运动，机械臂手动规划路径点并记录到文件中，机械臂加载路径文件。

+ void mechanical_arm_origin();
  ```
  函数功能：机械臂初始化位置
  返回值：无
  参数：无
  示例：
      int main()
      {
          Start();
          mechanical_arm_origin();
          Exit();
          return 0;
       }
  ```


+ bool brake();
   ```
  函数功能：机械臂刹车
  返回值：成功返回true
  参数：无
  示例：
     void signalHandler(int signum)
     {
        char aaa;
        cout << "Interrupt signal (" << signum << ") received.\n";
        brake();  
        cout << "stop!!" << endl;
        inspect_brake(); 
        Exit();
        exit(signum);
     }
  ```

+ bool IsBrake();//机械臂是否停止运动
  ```
  函数功能：机械臂刹车
  返回值：成功返回true
  参数：无
  示例：
     void signalHandler(int signum)
     {
        char aaa;
        cout << "Interrupt signal (" << signum << ") received.\n";
        brake();  
        cout << "stop!!" << endl;
        inspect_brake(); 
        Exit();
        exit(signum);
     }
  ```
     
+ void get_current_angle(float goal_j[7]);//获取当前角度
  ```
  函数功能：获取当前角度
  返回值：无
  参数：goal_j：存储角度的数组
  示例：
     int main()
     {
        float saveJoint[7]={0};
        get_current_angle(saveJoint);
        for(int i=0,i<7;i++)
        {
          cout<<"joint:"<<saveJoint[i];
        }
        return 0;
     }
  ```

+ void get_current_pose(float posz[6]);//获取当前位姿
  ```
    函数功能：获取当前位姿
    返回值：无
    参数：
        posz：存储位姿的数组
    */
  示例：
     int main()
     {
        Start();
        float savepos[6]={0};
        get_current_pose(savepos);
        for(int i=0,i<6;i++)
        {
          cout<<"joint:"<<savepos[i];
        }
        Exit();
        return 0;
     }
  ```

+ void joint_to_move(float *goal_j);//机械臂关节运动
  ```
    函数功能：机械臂关节运动
    返回值：无
    参数：
        goal_j：目标关节角
    */
  示例：
     int main()
     {
        Start();
        float num[7]={3.14099,1.17288,-0.608523,0.799968,-2.9569,0.478535,-0.682677};
        joint_to_move(num);
        Exit();
        return 0;
     }
  ```

+ void move_to_pos(float *pos,float value,int dim,bool absolute);//机械臂位姿运动
  ```
    函数功能：机械臂位姿运动
    返回值：无
    参数：
        pos：目标位置
        value：dim的值
        dim：0~2 代表x,y,z
        absolute：true的时候是绝对位置（以胸部原点位置），false的时候是相对位置（以当前点胳膊轴位置）
    */
  示例：
     int main()
     {
        Start();
        float num[6]={150, 266.047, -400, -1.89695, 2.8, 0.826565};
        move_to_pos(pos, value, dim, absolute);
        Exit();
        return 0;
     }
  ```




### 1.5 tool.h

该文件是一些`tool`，具体函数使用及参数请查看该文件。

### 1.6. can
该文件夹包含`can`通讯的头文件，机械臂是通过can通讯与控制机联通的，具体函数功能及参数请查看里面所包含的文件中注释了解函数作用。

## 3.src
### 3.1 main.cpp

该文件是一个简单的示例程序，调用了`Ti5BASIC.h`中的`query_can()`函数，首先查找是否连接了can设备，然后调用`mechanical_arm_origin()`让机械臂回到初始位置,然后有一个小动作。

### 3.2 gcc.sh

该文件中的内容是编译命令，编译的时候可以使用该命令直接编译，也可以使用g++命令+对应参数直接编译

### 3.2 编译

最后执行`gcc.sh`文件进行编译或通过以下命令进行编译生成可执行文件`move_sov`。(注意：以下路径是默认路径，如果修改了路径要替换成自己的)
```
g++ main.cpp  -L./include -lmylibti5 -L./include/can -lmylibscan -lcontrolcan -lspdlog -lfmt -ludev -o move_sov
```
**运行**:
```
sudo ./move_sov
```
注意机械臂处在一个安全的环境中

## 开发须知
由于电机内部的算法，指令发送的值与实际执行的值有微小误差。

