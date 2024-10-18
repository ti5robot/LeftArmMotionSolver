#ifndef TEST_DRAG_H
#define TEST_DRAG_H

#include "tool.h"
#include "Ti5LOGIC.h"
#include "Ti5BASIC.h"
#include "can/SingleCaninterface.h"
#include "can/motortypehelper.h"
#include <math.h>
#include <stdio.h>


#include <stdlib.h>
#include <unistd.h>
#include <pthread.h>
#include <string.h>

#include <fstream>
#include <chrono>
#include <thread>


#include <iostream>
#include <vector>
#include <sstream>
#include <iomanip>


#include <string>
#include <regex>





#include <stdexcept>
#include <cmath>





/*
#define MAX_TRAJECTORY_SIZE 10000


typedef struct {
    double jointAngles[6];
} JointState;

typedef struct {
    JointState trajectory[MAX_TRAJECTORY_SIZE];
    size_t size;
    int isRecording;
    pthread_t recordingThread;
} TrajectoryRecorder;

extern uint32_t pa[IDNUM];
extern float ang[IDNUM];
extern int status_flag;
// extern uint8_t qqq[6];

extern "C"
{ // 添加：extern C
   bool get_joint_angle();

   //1kg拖动示教
   float *GravityCompensation(float frr[6]);

   void* recordTrajectory(void *arg);
   void startRecording(TrajectoryRecorder *recorder);
   void stopRecording(TrajectoryRecorder *recorder, const char* filename);
   void* exit_drag_teaching(void *arg);

   int drag_teaching();

   void draggg_teaching(const char* filename); 

} // 添加：extern C
*/



// 用于存储点的结构
struct PPoint {
    float angles[6];
};

#define MAX_TRAJECTORY_SIZE 10000

// typedef struct {
//     double jointAngles[6];
// } JointState;

// typedef struct {
//     JointState trajectory[MAX_TRAJECTORY_SIZE];
//     size_t size;
//     int isRecording;
//     pthread_t recordingThread;
// } TrajectoryRecorder;
extern uint32_t pa[IDNUM];
extern float ang[6];
extern int status_flag;

bool get_joint_angle();
// float* GravityCompensation(const float* angles);
float *GravityCompensation(float frr[6]);
// int drag_teaching();
void drag_teaching(std::ofstream& outfile);
void save_trajectory();
void read_trajectory();
// 保存新数据到文件
void save_points_to_file(const std::string& filename, const std::vector<PPoint>& points);
std::vector<PPoint> insert_intermediate_points(const std::vector<PPoint>& points) ;
std::vector<PPoint> read_points_from_file(const std::string& filename);
// void* exit_drag_teach(void *arg);
// void* record_trajectory(void *arg);
// void write_trajectory_to_file(const char *filename, const TrajectoryRecorder *recorder);



std::vector<std::vector<float>> drag_interpolate_test(const std::vector<float> &start, const std::vector<float> &end, float step);
int drag_read_data_test(const std::string &filename);
#endif