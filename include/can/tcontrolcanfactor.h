#ifndef TCONTROLCANFACTOR_H
#define TCONTROLCANFACTOR_H


#include <memory>
#include "tcontrolcan.h"

enum class CanType{Can=0, CanOpen=1,CanOpen4=2};

class TControlCanFactor{

public:
    explicit TControlCanFactor(CanType type,int canID=1);

public:
    bool open();
    bool close();

    void setCanID(int canID){ tControlCAN->setCanID(canID);}

    int readRobotParameter(uint8_t requestType, float &recieveData);
    int readParameter(uint8_t requestType, int32_t &recieveData);
    int setParameter(uint8_t requestType, int32_t &recieveData);
    int setRobotParameter(uint8_t robotType, float &recieveData);
    int setGetCSP(int value, int16_t &current, int16_t &speed, int32_t &position);

    int getStatus();
    int getErrorStatus();
    std::vector<int>  getErrorInfos();
    int saveParameter();

    int clearFault();
    int stop();



private:
    std::shared_ptr<TControlCANBase>  tControlCAN;

};

#endif // TCONTROLCANFACTOR_H
