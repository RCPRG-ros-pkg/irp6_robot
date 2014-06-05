#include <rtt/TaskContext.hpp>
#include <rtt/Port.hpp>
#include <rtt/Component.hpp>

#include <Eigen/Dense>

#include "HardwareInterface.h"

HardwareInterface::HardwareInterface(const std::string& name) :
		TaskContext(name, PreOperational), hi_(NUMBER_OF_DRIVES - 1,CARD_ADDRESSES, MAX_INCREMENT, TX_PREFIX_LEN),
 		servo_stop_iter_(0){
	this->addPort("computedPwm_in", computedPwm_in).doc(
			"Receiving a value of computed PWM.");
	this->addPort("posInc_out", posInc_out).doc(
			"Sends out a value of expected position increment.");
	this->addPort("deltaInc_out", deltaInc_out).doc(
			"Sends out a value increment increase in cycle.");

    this->ports()->addPort("MotorPosition", port_motor_position_);
    this->ports()->addPort("MotorPositionCommand", port_motor_position_command_);

    this->addProperty("number_of_drives", number_of_drives).doc("Number of drives in robot");
}

HardwareInterface::~HardwareInterface() {

private:

bool HardwareInterface::configureHook() {

  counter = 0.0;
  debug = true;
  auto_synchronize = true;

  increment.resize(number_of_drives);
  pos_inc.resize(number_of_drives);
  pwm.resize(number_of_drives);

  for(int i=0; i<number_of_drives; i++)
  {
    increment[i] = 0;
    pos_inc[i] = 0;
    pwm[0] = 0;
  }

  std::vector<std::string> ports;

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
	}
	catch (std::exception& e)
	{
		log(Info) << e.what() << endlog();
		return false;
	}

  motor_position_.resize(number_of_drives);
  motor_position_command_.resize(number_of_drives);
  motor_position_command_old_.resize(number_of_drives);
  
	return true;

}

bool HardwareInterface::startHook()
{
    try
    {
        hi_.HI_read_write_hardware();

        if(!hi_.robot_synchronized())
        {
            RTT::log(RTT::Info) << "Robot not synchronized" << RTT::endlog();
            if(auto_synchronize)
            {
                RTT::log(RTT::Info) << "Auto synchronize" << RTT::endlog();
                state = SYNCHRONIZING;
                synchro_state = MOVE_TO_SYNCHRO_AREA;
                synchro_drive = 0;
            }
            else
                state = NOT_SYNCHRONIZED;

        }
        else
        {
            RTT::log(RTT::Info) << "Robot synchronized" << RTT::endlog();

            for(int i=0; i<number_of_drives; i++)
            {
                motor_position_command_(i) = (double)hi_.get_position(i) * ((2.0 * M_PI) / ENC_RES[i]);
                motor_position_command_old_(i) = motor_position_command_(i);
            }

            state = SERVOING;
        }
    }
    catch (const std::exception& e)
    {
      RTT::log(RTT::Error) << e.what() << RTT::endlog();
      return false;
    }

    for(int i=0; i<number_of_drives; i++)
    {
        pos_inc[i] = 0.0;
    }
    
    return true;
}


void HardwareInterface::updateHook()
{


    if(NewData!=computedPwm_in.read(pwm)) {
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
            if (port_motor_position_command_.read(motor_position_command_) == RTT::NewData)
            {
                for(int i=0; i<number_of_drives; i++)
                {
                    pos_inc[i] =  (motor_position_command_(i) - motor_position_command_old_(i)) * (ENC_RES[i] / (2.0 * M_PI));
                    motor_position_command_old_(i) = motor_position_command_(i);
                }
            }
            else
            {
                for(int i=0; i<number_of_drives; i++)
                {
                    pos_inc[i] = 0.0;
                }
            }
            
            for(int i=0; i<number_of_drives; i++)
            {
              motor_position_(i) = (double)hi_.get_position(i) * ((2.0 * M_PI) / ENC_RES[i]);
            }
            port_motor_position_.write(motor_position_);
        break;

        case SYNCHRONIZING :
            switch(synchro_state)
            {
                case MOVE_TO_SYNCHRO_AREA :
                    if(hi_.in_synchro_area(synchro_drive))
                    {
                        RTT::log(RTT::Debug) << "[servo " << synchro_drive << " ] MOVE_TO_SYNCHRO_AREA ended" << RTT::endlog();
                        pos_inc[synchro_drive] = 0.0;
                        synchro_state = STOP;
                    }
                    else
                    {
                        //ruszam powoli w stronę synchro area
                        RTT::log(RTT::Debug) << "[servo " << synchro_drive << " ] MOVE_TO_SYNCHRO_AREA" << RTT::endlog();
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
                        RTT::log(RTT::Debug) << "[servo " << synchro_drive << " ] MOVE_FROM_SYNCHRO_AREA ended" << RTT::endlog();

                        synchro_state = WAIT_FOR_IMPULSE;
                    }
                    else
                    {
                        RTT::log(RTT::Debug) << "[servo " << synchro_drive << " ] MOVE_FROM_SYNCHRO_AREA" << RTT::endlog();
                        pos_inc[synchro_drive] = SYNCHRO_STEP_FINE[synchro_drive] * (ENC_RES[synchro_drive]/(2.0*M_PI));
                    }
                break;

                case WAIT_FOR_IMPULSE:
                    if(hi_.drive_synchronized(synchro_drive))
                    {
                        RTT::log(RTT::Debug)  << "[servo " << synchro_drive << " ] WAIT_FOR_IMPULSE ended" << RTT::endlog();

                        for(int i=0; i<number_of_drives; i++)
                        {
                            pos_inc [i]= 0.0;
                        }

                        hi_.finish_synchro(synchro_drive);
                        hi_.reset_position(synchro_drive);

                        motor_position_command_(synchro_drive) = (double)hi_.get_position(synchro_drive) * ((2.0 * M_PI) / ENC_RES[synchro_drive]);
                        motor_position_command_old_(synchro_drive) = motor_position_command_(synchro_drive);
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
                        RTT::log(RTT::Debug) << "[servo " << synchro_drive << " ] WAIT_FOR_IMPULSE" << RTT::endlog();
                        pos_inc[synchro_drive] = SYNCHRO_STEP_FINE[synchro_drive] * (ENC_RES[synchro_drive]/(2.0*M_PI));
                    }
                break;

                case SYNCHRO_END :

            for (int i = 0; i < number_of_drives; i++) {
              motor_position_command_(i) = motor_position_command_old_(i) = hi_
                  .get_position(i) * (2.0 * M_PI) / ENC_RES[i];
            }

            if ((servo_stop_iter_--) <= 0) {
              state = SERVOING;
              RTT::log(RTT::Debug) << "[servo " << synchro_drive
                                   << " ] SYNCHRONIZING ended" << RTT::endlog();
              std::cout << "synchro finished" << std::endl;
            }
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

    deltaInc_out.write(increment);
    posInc_out.write(pos_inc);
}

};
ORO_CREATE_COMPONENT(HardwareInterface)
