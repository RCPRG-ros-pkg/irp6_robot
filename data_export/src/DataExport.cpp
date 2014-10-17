#include <rtt/TaskContext.hpp>
#include <rtt/Port.hpp>
#include <std_msgs/Float64.h>
#include <std_msgs/Int64.h>
#include <rtt/Component.hpp>
#include <std_srvs/Empty.h>
#include <ros/ros.h>

#include "DataExport.h"

const int MAX_PWM = 190;

DataExport::DataExport(const std::string& name)
: TaskContext(name),
  positionData(0.0),
  incrementData(0.0),
  //voltageData(0.0),
  currentData(0.0)
{
   this->addProperty("number_of_drives", number_of_drives_).doc("");
   this->addProperty("filename", filename_).doc("");
   this->addProperty("debug", debug_).doc("");
   this->addProperty("matfile", matfile_).doc("");
   this->addProperty("csvfile", csvfile_).doc("");

   this->addProperty("hi_port_param_0", hi_port_param_[0]).doc("");
   this->addProperty("hi_port_param_1", hi_port_param_[1]).doc("");
   this->addProperty("hi_port_param_2", hi_port_param_[2]).doc("");
   this->addProperty("hi_port_param_3", hi_port_param_[3]).doc("");
   this->addProperty("hi_port_param_4", hi_port_param_[4]).doc("");
   this->addProperty("hi_port_param_5", hi_port_param_[5]).doc("");
   this->addProperty("hi_port_param_6", hi_port_param_[6]).doc("");
   this->addProperty("hi_port_param_7", hi_port_param_[7]).doc("");
   this->addProperty("hi_port_param_8", hi_port_param_[8]).doc("");
   this->addProperty("hi_port_param_9", hi_port_param_[9]).doc("");
   this->addProperty("hi_port_param_10", hi_port_param_[10]).doc("");
   this->addProperty("hi_port_param_11", hi_port_param_[11]).doc("");
   this->addProperty("hi_port_param_12", hi_port_param_[12]).doc("");
   this->addProperty("hi_port_param_13", hi_port_param_[13]).doc("");
   this->addProperty("hi_port_param_14", hi_port_param_[14]).doc("");
   this->addProperty("hi_port_param_15", hi_port_param_[15]).doc("");

   step=0;
}

DataExport::~DataExport() {

   if (csvfile_)
   {
      csv.close();
   }
}

bool DataExport::configureHook() {

   label_.resize(number_of_drives_);

   port_motor_position_list_.resize(number_of_drives_);
   port_motor_increment_list_.resize(number_of_drives_);
   port_motor_current_list_.resize(number_of_drives_);

   for (size_t i = 0; i < number_of_drives_; i++)
   {
      //std::cout << "i: "  << i << " label: " << hi_port_param_[i].label << std::endl;
      label_[i] = hi_port_param_[i].label;

      char MotorPosition_port_name[32];
      snprintf(MotorPosition_port_name, sizeof(MotorPosition_port_name),
               "MotorPosition_%s", hi_port_param_[i].label.c_str());
      port_motor_position_list_[i] = new typeof(*port_motor_position_list_[i]);
      this->ports()->addPort(MotorPosition_port_name,
                             *port_motor_position_list_[i]);

      char MotorIncrement_port_name[32];
      snprintf(MotorIncrement_port_name, sizeof(MotorIncrement_port_name),
               "MotorIncrement_%s", hi_port_param_[i].label.c_str());
      port_motor_increment_list_[i] = new typeof(*port_motor_increment_list_[i]);
      this->ports()->addPort(MotorIncrement_port_name,
                             *port_motor_increment_list_[i]);

      char MotorCurrent_port_name[32];
      snprintf(MotorCurrent_port_name, sizeof(MotorCurrent_port_name),
               "MotorCurrent_%s", hi_port_param_[i].label.c_str());
      port_motor_current_list_[i] = new typeof(*port_motor_current_list_[i]);
      this->ports()->addPort(MotorCurrent_port_name,
                             *port_motor_current_list_[i]);
   }

   reset();

   char cCurrentPath[FILENAME_MAX];

   if (!getcwd(cCurrentPath, sizeof(cCurrentPath)))
   {
      return errno;
   }

   cCurrentPath[sizeof(cCurrentPath) - 1] = '\0';

   time_t     now = time(0);
   struct tm  tstruct;
   char       buf[80];
   tstruct = *localtime(&now);
   strftime(buf, sizeof(buf), "_%Y-%m-%d_%X", &tstruct);

   string filename = string(cCurrentPath);
   int ros = filename.find(".ros");
   filename = filename.erase(ros) + filename_ + string(buf);

   if (csvfile_)
   {
      csv.open((filename+".csv").c_str());
      //csv << "step;";

      for (size_t i = 0; i < number_of_drives_; i++)
      {
         csv << label_[i]<< "_position;";
         csv << label_[i]<< "_increment;";
         csv << label_[i]<< "_current;";
         csv << ";";
      }

      csv << endl;
   }

   return true;
}

void DataExport::updateHook()
{
   step++;

   //if (csvfile_)
   //{
   //   csv << step;
   //}

   bool rec = false;

   if (NewData == port_motor_position_list_[0]->read(positionData)
         && NewData == port_motor_increment_list_[0]->read(incrementData)
         && NewData == port_motor_current_list_[0]->read(currentData))
   {
      rec = true;

      if (csvfile_)
      {
         csv << positionData << ";";
         csv << incrementData << ";";
         csv << currentData << ";";
         csv << ";";
      }

      if (debug_)
      {
         std::cout << step;
         std::cout << "  position: " << positionData;
         std::cout << "  increment: " << incrementData;
         std::cout << "  current: " << currentData;
         std::cout << std::endl;
      }
   }

   for (size_t i = 1; i < number_of_drives_; i++)
   {
      if (NewData == port_motor_position_list_[i]->read(positionData)
            && NewData == port_motor_increment_list_[i]->read(incrementData)
            && NewData == port_motor_current_list_[i]->read(currentData))
      {
         if (csvfile_)
         {
            csv << positionData << ";";
            csv << incrementData << ";";
            csv << currentData << ";";
            csv << ";";
         }
      }
   }

   if (csvfile_ && rec)
   {
      csv << endl;
   }

}


void DataExport::reset()
{}

ORO_CREATE_COMPONENT(DataExport)
