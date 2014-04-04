#include <rtt/TaskContext.hpp>
#include <rtt/Port.hpp>
#include <std_msgs/Float64.h>
#include <std_msgs/Int64.h>
#include <std_msgs/Bool.h>
#include <rtt/Component.hpp>
#include <std_srvs/Empty.h>
#include <ros/ros.h>

#include "hi_moxa.h"

//SARKOFAG
/*
const int NUMBER_OF_DRIVES = 1;
const int16_t MAX_CURRENT[] = {25000};
const double MAX_INCREMENT[] = { 400 };
const unsigned int CARD_ADDRESSES[] = { 0 };
const int TX_PREFIX_LEN = 5;

const double SYNCHRO_JOINT_POSITION[] = { -168 };
const int ENC_RES[] = {4000};
const double SYNCHRO_STEP_COARSE[] = {0.05};
const double SYNCHRO_STEP_FINE[] = {-0.02};

const double GEAR = 150;
const int ENC_RES[] = {4000};
const double SYNCHRO_JOINT_POSITION[] = { -168 };
*/

//IRP6

const int NUMBER_OF_DRIVES = 6;
const int16_t MAX_CURRENT[] = {25000, 18000, 15000, 17000, 10000, 2000};
const double MAX_INCREMENT[] = { 1000, 1000, 1000, 1000, 1000, 1000 };
const unsigned int CARD_ADDRESSES[] = { 0, 1, 2, 3, 4, 5 };
const int TX_PREFIX_LEN = 0;

const double GEAR[6] = {-158.0, 2*M_PI/5.0, 2*M_PI/5.0, -128.0, -128.0*0.6, 288.8845};
const double THETA[6] = {0.0, 2.203374e+02, 1.838348e+02, 1.570796e+00, 0.0, 0.0};

const double SYNCHRO_MOTOR_POSITION[6] = {-15.9, -5.0, -8.527, 151.31, 432.25, 791.0};
const double 	SYNCHRO_JOINT_POSITION[6] = { SYNCHRO_MOTOR_POSITION[0] - GEAR[0] * THETA[0],
                                            SYNCHRO_MOTOR_POSITION[1] - GEAR[1] * THETA[1],
                                              SYNCHRO_MOTOR_POSITION[2] - GEAR[2] * THETA[2],
                                              SYNCHRO_MOTOR_POSITION[3] - GEAR[3] * THETA[3],
                                              SYNCHRO_MOTOR_POSITION[4] - GEAR[4] * THETA[4] - SYNCHRO_MOTOR_POSITION[3],
                                              SYNCHRO_MOTOR_POSITION[5] - GEAR[5] * THETA[5] };

const int ENC_RES[6] = {4000, 4000, 4000, 4000, 4000, 2000};
const double SYNCHRO_STEP_COARSE[6] = {-0.03, -0.03, -0.03, -0.03, -0.03, -0.05};
const double SYNCHRO_STEP_FINE[6] = {0.007, 0.007, 0.007, 0.007, 0.007, 0.05};


using namespace RTT;

typedef enum { NOT_SYNCHRONIZED, SERVOING, SYNCHRONIZING } State;
typedef enum { MOVE_TO_SYNCHRO_AREA, STOP, MOVE_FROM_SYNCHRO_AREA, WAIT_FOR_IMPULSE, SYNCHRO_END } SynchroState;


class HardwareInterface : public RTT::TaskContext{

private: 
 
InputPort<std::vector<double> >computedPwm_in;

OutputPort<std::vector<double> >posInc_out;
OutputPort<std::vector<long int> >deltaInc_out;

InputPort<std::vector<double> > dsrJntPos_port_;
OutputPort<std::vector<double> > cmdJntPos_port_;

std::vector<double> dsrJntPos_;
std::vector<double> cmdJntPos_;

//OutputPort<std_msgs::Bool> is_synchronized;

int number_of_drives;
bool auto_synchronize;

double counter;

State state;
SynchroState synchro_state;
int synchro_drive;

std::vector<double> pos_inc;

std::vector<long int> increment;
std::vector<int> inc_pos;
std::vector<double> joint_pos;
std::vector<double> motor_pos;
std::vector<double> motor_pos_old;
std::vector<double> pwmData;

std::vector<double> pwm;

hi_moxa::HI_moxa hi_;

bool debug;

//OperationCaller<bool(std_srvs::Empty::Request&, std_srvs::Empty::Response&)> updated;


public:

HardwareInterface(const std::string& name):
	TaskContext(name),
    computedPwm_in("computedPwm_in"),
    posInc_out("posInc_out"),
    deltaInc_out("deltaInc_out"),
    hi_(NUMBER_OF_DRIVES-1, CARD_ADDRESSES, MAX_INCREMENT, TX_PREFIX_LEN)
{
    this->addEventPort(computedPwm_in).doc("Receiving a value of computed PWM.");
    this->addPort(posInc_out).doc("Sends out a value of expected position increment.");
    this->addPort(deltaInc_out).doc("Sends out a value increment increase in cycle.");

    this->ports()->addPort("desJntPos", dsrJntPos_port_);
    this->ports()->addPort("cmdJntPos", cmdJntPos_port_);

    this->addProperty("number_of_drives", number_of_drives).doc("Number of drives in robot");

    this->provides()->addOperation("showPosition",&HardwareInterface::showPosition,this,RTT::OwnThread);



}

~HardwareInterface(){}

private:


bool configureHook()
{

    counter = 0.0;
    debug = true;
    auto_synchronize = true;

    inc_pos.resize(number_of_drives);
    increment.resize(number_of_drives);
    pos_inc.resize(number_of_drives);
    motor_pos_old.resize(number_of_drives);
    joint_pos.resize(number_of_drives);
    pwmData.resize(number_of_drives);
    pwm.resize(number_of_drives);

    for(int i=0; i<number_of_drives; i++)
    {
        inc_pos[i] = 0;
        increment[i] = 0;
        pos_inc[i] = 0;
        motor_pos_old[i] = 0;
        joint_pos[i] = 0;
        pwm[0] = 0;
    }

    std::vector<std::string> ports;

    //sarkofag
      //ports.push_back("/dev/ttyMI7");
    //irp6

      ports.push_back("/dev/ttyMI8");
      ports.push_back("/dev/ttyMI9");
      ports.push_back("/dev/ttyMI10");
      ports.push_back("/dev/ttyMI11");
      ports.push_back("/dev/ttyMI12");
      ports.push_back("/dev/ttyMI13");


	try
	{
		hi_.init(ports);
        for(int i=0; i<number_of_drives; i++)
        {
            hi_.set_parameter_now(i, NF_COMMAND_SetDrivesMaxCurrent, MAX_CURRENT[i]);
            hi_.set_pwm_mode(i);
        }
        /*NF_STRUCT_Regulator tmpReg =
                    { hi_.convert_to_115(0.0600), hi_.convert_to_115(0.0500), hi_.convert_to_115(0.0), 0 };
        hi_.set_parameter_now(0, NF_COMMAND_SetCurrentRegulator, tmpReg);
        */

	}
	catch (std::exception& e)
	{
		log(Info) << e.what() << endlog();	
	}

    dsrJntPos_.resize(number_of_drives);
    cmdJntPos_.resize(number_of_drives);

    std::cout << "HI CONF" << std::endl;

	return true;

}

bool startHook()
{
    try
    {
        //std::cout << "HI START 1" << std::endl;

        hi_.HI_read_write_hardware();

        //std::cout << "HI START 2" << std::endl;

        if(!hi_.robot_synchronized())
        {
            std::cout << "Robot not synchronized" << std::endl;
            if(auto_synchronize)
            {
                std::cout << "Auto synchronize" << std::endl;
                state = SYNCHRONIZING;
                synchro_state = MOVE_TO_SYNCHRO_AREA;
                synchro_drive = 0;
            }
            else
                state = NOT_SYNCHRONIZED;

        }
        else
        {
            std::cout << "Robot synchronized" << std::endl;

            for(int i=0; i<number_of_drives; i++)
            {
                inc_pos[i] = hi_.get_position(i);
                //showPosition();

                motor_pos_old[i] = i2jp(inc_pos[i]);
                joint_pos[i] = mp2i(motor_pos_old[i]);
                dsrJntPos_[i] = motor_pos_old[i];
            }

            cmdJntPos_ = dsrJntPos_;
            state = SERVOING;
        }
    }
    catch (const std::exception& e)
    {
      std::cout << e.what() << std::endl;
    }

    //regulator.reset();
    for(int i=0; i<number_of_drives; i++)
    {
        pos_inc[i] = 0.0;
    }

    std::cout << "HI START" << std::endl;
}


void updateHook()
{


    if(NewData==computedPwm_in.read(pwmData))
    {
        for(int i=0; i<number_of_drives; i++)
        {
            pwm[i] = pwmData[i];
        }
        //std::cout << "PWM RECIVED: " << pwm[0] << std::endl;
    } else {
        RTT::log(RTT::Error) << "NO PWM DATA" << RTT::endlog();
    }

    for(int i=0; i<number_of_drives; i++)
        {
            hi_.set_pwm(i, pwm[i]);
        }

    hi_.HI_read_write_hardware();

    switch(state)
    {
        case NOT_SYNCHRONIZED :

            for(int i=0; i<number_of_drives; i++)
            {
                pos_inc [i]= 0.0;
            }


        break;

        case SERVOING :
            if (dsrJntPos_port_.read(dsrJntPos_) == RTT::NewData)
            {
                for(int i=0; i<number_of_drives; i++)
                {
                    if(debug)
                    {
                        std::cout << "dsrJntPos_ recived: " << dsrJntPos_[i] << std::endl;
                    }
                    joint_pos[i] = dsrJntPos_[i];
                    pos_inc[i] = jp2i(joint_pos[i]-motor_pos_old[i]);
                    motor_pos_old[i] = joint_pos[i];
                }

                /*motor_pos = i2mp(joint_pos);
                pos_inc = (motor_pos - motor_pos_old) * ((double)const_sarkofag::ENC_RES[0]/(2.0*M_PI));

                motor_pos_old = motor_pos;*/

                /*if(debug)
                {
                    std::cout << "pos_inc: " << pos_inc << std::endl;
                }*/
            }

            else
            {
                for(int i=0; i<number_of_drives; i++)
                {
                    pos_inc[i] = 0.0;
                }
            }
            cmdJntPos_ = dsrJntPos_;


        break;

        case SYNCHRONIZING :
            switch(synchro_state)
            {
                case MOVE_TO_SYNCHRO_AREA :
                    if(hi_.in_synchro_area(synchro_drive))
                    {
                        std::cout << "[servo " << synchro_drive << " ] MOVE_TO_SYNCHRO_AREA ended" << std::endl;
                        pos_inc[synchro_drive] = 0.0;
                        synchro_state = STOP;
                    }
                    else
                    {
                        //ruszam powoli w stronę synchro area
                        if(debug) std::cout << "[servo " << synchro_drive << " ] MOVE_TO_SYNCHRO_AREA" << std::endl;
                        pos_inc[synchro_drive] = SYNCHRO_STEP_COARSE[synchro_drive] * (ENC_RES[synchro_drive]/(2.0*M_PI));
                    }
                break;

                case STOP :
                    //tutaj jakis timeout
                    hi_.start_synchro(synchro_drive);
                    synchro_state = MOVE_FROM_SYNCHRO_AREA;

                break;

                case MOVE_FROM_SYNCHRO_AREA :
                    if(!hi_.in_synchro_area(synchro_drive))
                    {
                        if(debug) std::cout << "[servo " << synchro_drive << " ] MOVE_FROM_SYNCHRO_AREA ended" << std::endl;

                        synchro_state = WAIT_FOR_IMPULSE;
                    }
                    else
                    {
                        if(debug) std::cout << "[servo " << synchro_drive << " ] MOVE_FROM_SYNCHRO_AREA" << std::endl;
                        pos_inc[synchro_drive] = SYNCHRO_STEP_FINE[synchro_drive] * (ENC_RES[synchro_drive]/(2.0*M_PI));
                    }
                break;

                case WAIT_FOR_IMPULSE:
                    if(hi_.drive_synchronized(synchro_drive))
                    {
                        if(debug) std::cout << "[servo " << synchro_drive << " ] WAIT_FOR_IMPULSE ended" << std::endl;

                        for(int i=0; i<number_of_drives; i++)
                        {
                            pos_inc [i]= 0.0;
                        }

                        hi_.finish_synchro(synchro_drive);
                        hi_.reset_position(synchro_drive);
                        //regulator.reset();

                        if(++synchro_drive < number_of_drives)
                        {
                          synchro_state = MOVE_TO_SYNCHRO_AREA;
                        }
                        else
                        {
                          synchro_state = SYNCHRO_END;
                        }

                    }
                    else
                    {
                        if(debug) std::cout << "[servo " << synchro_drive << " ] WAIT_FOR_IMPULSE" << std::endl;
                        pos_inc[synchro_drive] = SYNCHRO_STEP_FINE[synchro_drive] * (ENC_RES[synchro_drive]/(2.0*M_PI));
                    }
                break;

                case SYNCHRO_END :

                    if(debug) std::cout << "[servo " << synchro_drive << " ] SYNCHRONIZING ended" << std::endl;
                    state = SERVOING;
                break;
            }
        break;

    }


    for(int i=0; i<number_of_drives; i++)
    {
        increment[i]= hi_.get_increment(i);

        if(abs(increment[i]) > 400)
        {
            increment[i] = 0;
        }

        if(fabs(pos_inc[i]) > 400)
        {
            pos_inc[i] = 0;
        }
    }

    //std::cout << "INCREMENT: " << increment[0] << std::endl;

    //sendPosInc(pos_inc);
    //sendDeltaInc(increment);

    deltaInc_out.write(increment);
    posInc_out.write(pos_inc);

    if(state==SERVOING)
    {
        cmdJntPos_port_.write(cmdJntPos_);
    }
}


void showPosition()
{
    for(int i=0; i<number_of_drives; i++)
    {
        std::cout <<  "POSITION [" << i << "]:" << std::endl;
        std::cout <<  "increments : " << inc_pos[i] << std::endl;
        std::cout << "joint_position: " << i2jp(inc_pos[i]) << std::endl;
        std::cout << "motor_position: " << i2mp(inc_pos[i]) << std::endl;
    }
}


double i2jp (double iPosition)
{
    return iPosition  * ((2.0 * M_PI) / ENC_RES[0]);
}

double jp2i (double mPosition)
{
    return mPosition  * (ENC_RES[0] / (2.0 * M_PI));
}

double i2mp (double iPosition)
{
    double joints = i2jp(iPosition);
    return (joints - SYNCHRO_JOINT_POSITION[0]) / GEAR[0];
}

double mp2i (double mPosition)
{
    mPosition = (mPosition * GEAR[0]) + SYNCHRO_JOINT_POSITION[0];
    return jp2i(mPosition);
}

/*void sendPosInc(double data)
{
    std_msgs::Float64 currentData;
    currentData.data = data;

    posInc_out.write(currentData);
}

void sendDeltaInc(double data)
{
    std_msgs::Float64 incrementData;
    incrementData.data = data;

    deltaInc_out.write(incrementData);
}*/

};
ORO_CREATE_COMPONENT(HardwareInterface)
