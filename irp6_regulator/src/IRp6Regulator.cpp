#include <rtt/TaskContext.hpp>
#include <rtt/Port.hpp>
#include <std_msgs/Float64.h>
#include <std_msgs/Int64.h>
#include <rtt/Component.hpp>
#include <std_srvs/Empty.h>
#include <ros/ros.h>

#include "Regulator.h"

//SARKOFAG
/*
const double A[] = {0.0};
const double BB0[] = {1.05};
const double BB1[] = {1};

const int ENC_RES = {4000};
*/
//IRP6

const double A[6] = {0.412429378531, 0.655629139073, 0.315789473684, 0.548946716233, 0.391982182628, 0.3};
const double BB0[6] = {2.594932 * 0.6, 1.030178 * 0.6, 1.997464 * 0.6, 1.576266 * 0.4, 1.114648 * 0.4, 1.364 * 0.4};
const double BB1[6] = {2.504769 * 0.6, 0.986142 * 0.6, 1.904138 * 0.6, 1.468599 * 0.4, 1.021348 * 0.4, 1.264 * 0.4};

const int ENC_RES[] = {4000, 4000, 4000, 4000, 4000, 2000};



using namespace RTT;

class IRp6Regulator : public RTT::TaskContext{

private: 

InputPort<std::vector<double> >posInc_in;
InputPort<std::vector<int> >deltaInc_in;

OutputPort<std::vector<double> >computedPwm_out;

std::vector<double> posIncData;
std::vector<int> deltaIncData;

int number_of_drives;

Regulator regulator[6];


public:

IRp6Regulator(const std::string& name):
	TaskContext(name),
    posInc_in("posInc_in"),
    deltaInc_in("deltaInc_in"),
    computedPwm_out("computedPwm_out")
{

    this->addEventPort(posInc_in).doc("Receiving a value of position step");
    this->addPort(deltaInc_in).doc("Receiving a value of measured increment.");
    this->addPort(computedPwm_out).doc("Sending value of calculated pwm.");

    this->addProperty("number_of_drives", number_of_drives).doc("Number of drives in robot");

}

~IRp6Regulator(){}

private:

bool configureHook()
{

    for(int i=0; i<number_of_drives; i++)
    {
        regulator[i].reset();
        regulator[i].setParam(A[i], BB0[i], BB1[i]);
    }
    std::cout << "REG CONF" << std::endl;
}

void updateHook()
{
    if(NewData==posInc_in.read(posIncData) && NewData==deltaInc_in.read(deltaIncData))
    {
        /*for(int i=0; i<number_of_drives; i++)
        {
            std::cout << "POS INC" << posIncData[i] << std::endl;
        }*/
        computedPwm_out.write(computePwmValue(posIncData,deltaIncData));
    }
    /*else
        std::cout << "NO DATA" << std::endl;
    */


}

std::vector<double> computePwmValue(const std::vector<double>& posInc, const std::vector<int>& deltaInc)
{
    std::vector<double> ret(number_of_drives);
    for(int i=0; i<number_of_drives; i++)
    {
        ret[i] = regulator[i].doServo(posInc[i], deltaInc[i]);
        //std::cout << "REG COMPUTED" << ret[i] << std::endl;
    }

    return ret;
}


};
ORO_CREATE_COMPONENT(IRp6Regulator)
