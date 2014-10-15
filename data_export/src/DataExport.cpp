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
  position_in("position_in"),
  increment_in("increment_in"),
  //voltage_in("voltage_in"),
  current_in("current_in"),
  positionData(0.0),
  incrementData(0.0),
  //voltageData(0.0),
  currentData(0.0)
{

   this->addEventPort(position_in).doc("Receiving a value of motor position");
   this->addPort(increment_in).doc("Receiving a value of motor increment");
   //this->addPort(voltage_in).doc("Receiving a value of motor voltage");
   this->addPort(current_in).doc("Receiving a value of motor current");


   this->addProperty("drive_number", drive_number_).doc("");
   this->addProperty("debug", debug_).doc("");
   this->addProperty("position", position_).doc("");
   this->addProperty("increment", increment_).doc("");
   this->addProperty("voltage", voltage_).doc("");
   this->addProperty("current", current_).doc("");
   this->addProperty("matfile", matfile_).doc("");
   this->addProperty("csvfile", csvfile_).doc("");

   step_reg=0;
}

DataExport::~DataExport() {

   if (csvfile_)
   {
      csv.close();
   }
}

bool DataExport::configureHook() {
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
   strftime(buf, sizeof(buf), "%Y-%m-%d_%X", &tstruct);

   string filename = string(cCurrentPath);
   int ros = filename.find(".ros");
   ostringstream ss;
   ss << drive_number_;
   filename = filename.erase(ros) + string(buf) + "_drive:" + ss.str();;

   if (csvfile_)
   {
      csv.open((filename+".csv").c_str());

      if (position_)
      {
         csv << "position ";
      }

      if (increment_)
      {
         csv << "increment ";
      }

      /*if (voltage_)
      {
         csv << "voltage ";
      }*/

      if (current_)
      {
         csv << "current ";
      }

      csv << endl;
   }

   return true;
}

void DataExport::updateHook()
{
   step_reg++;

   if (NewData == position_in.read(positionData)
         && NewData == increment_in.read(incrementData)
         //&& NewData == voltage_in.read(voltageData)
         && NewData == current_in.read(currentData))
   {

      if (csvfile_)
      {
         if (position_)
         {
            csv << positionData << " ";
         }

         if (increment_)
         {
            csv << incrementData << " ";
         }

         /*if (voltage_)
         {
            csv << voltageData << " ";
         }*/

         if (current_)
         {
            csv << currentData << " ";
         }

         csv << endl;
      }

      if (debug_)
      {
         std::cout << step_reg;
         std::cout << "  position: " << positionData;
         std::cout << "  increment: " << incrementData;
         //std::cout << "  voltage: " << voltageData;
         std::cout << "  current: " << currentData;
         std::cout << std::endl;
      }
   }
}


void DataExport::reset()
{}

ORO_CREATE_COMPONENT(DataExport)
