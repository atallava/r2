//===============================================================================
/*! 
  \file husky_openloop_control_node.cpp

  \author Venkat Rajagopalan
  \date   02/18/2014
  
  \attention
  Copyright (C) 2014 \n
  National Robotics Engineering Center (NREC) \n
  Robotics Institute
  Carnegie Mellon University
  All Rights Reserved \n
*/
//===============================================================================
#include <vector>

#include <ros/ros.h>
#include <ros/console.h>
#include <sensor_msgs/Joy.h>
#include <geometry_msgs/Twist.h>

#include <boost/shared_ptr.hpp>

#include <Utils/FileUtil.h>

#include <VehicleState/VehicleState.h>
#include <Simple3DSim/Simple3DSim.h>

#include <PathTracker/ForwardSimulator.h>


// this is a major hack - we should set this up from the PS3 #defines
static const int32_t launchButtonIdx(15);

static bool launchButtonPressed(false);

//-------------------------------------------------------------------------------
void updateJoystickState(const sensor_msgs::Joy::ConstPtr& joystickState)
{
  if(1 == joystickState->buttons[launchButtonIdx])
    {
      launchButtonPressed = true;
    }
}

//-------------------------------------------------------------------------------
int main(int argc, char* argv[])
{
  // initialize
  launchButtonPressed = false;

  // the desired cycle rate of the algorithm
  const double cycleRate(20.0);


  // load the parameters from file
  const std::string fileName("VehicleSimParams.xml");
  const std::string dirName("PathTracker");
  std::string paramsFileName;

  if(!ddt::FileUtil::buildParamsFileName(fileName,
                                         dirName,
                                         paramsFileName))
    {
      ROS_FATAL("Could not find the parameter file: %s in directory %s",
                fileName.c_str(), 
                dirName.c_str());
      return -1;
    }

  // the parameters for the vehicle simulator
  ddt::VehicleSimParams vehicleParams;

  // load the parameters
  try
    {
      vehicleParams.loadParams(paramsFileName);
    }
  catch(const std::runtime_error& e)
    {
      ROS_FATAL("Exception caught: %s", e.what());
      return -1;
    }

  // the vehicle simulator
  boost::shared_ptr<ddt::Simple3DSim> sim;
  sim.reset(new ddt::Simple3DSim(vehicleParams));

  // the forward simulator object
  vmi::ForwardSimulator fwdSim(sim, cycleRate);

  // build the desired path
  std::vector<vmi::LocVel_T> desiredPath;
  double desiredVel = 1.0;
  vmi::LocVel_T locVel;
  locVel.speed = desiredVel;

  // construct a straight line path
  double yy = 0.0;
  double xx = 0.0;
  for(xx = 0.0; xx <= 5.0; xx+= 1.0)
    {
      locVel.loc.x() = xx;
      locVel.loc.y() = yy;
      
      // add to the container
      desiredPath.push_back(locVel);
    }

  // // right turn
  // for(yy = 0.0; yy <= 5.0; yy+= 1.0)
  //   {
  //     locVel.loc.x() = xx;
  //     locVel.loc.y() = yy;
      
  //     // add to the container
  //     desiredPath.push_back(locVel);
  //   }

  // right turn
  for(yy = 0.0; yy >= -5.0; yy-= 0.5)
    {
      locVel.loc.x() = xx;
      locVel.loc.y() = yy;
      
      // add to the container
      desiredPath.push_back(locVel);
    }

#if 0
  for(xx = 0.0; xx <= 5.0; xx+= 0.5)
    {
      locVel.loc.x() = xx;
      locVel.loc.y() = yy;
      
      // add to the container
      desiredPath.push_back(locVel);
    }

  // to the right
  for(yy = 0.0; yy <= 2.5; yy+= 0.5)
    {
      locVel.loc.x() = xx;
      locVel.loc.y() = yy;
      
      // add to the container
      desiredPath.push_back(locVel);
    }

  // straight
  for(; xx <= 7.5; xx+= 0.5)
    {
      locVel.loc.x() = xx;
      locVel.loc.y() = yy;
      
      // add to the container
      desiredPath.push_back(locVel);
    }

  // to the left
  for(; yy >= -2.5; yy-= 0.5)
    {
      locVel.loc.x() = xx;
      locVel.loc.y() = yy;
      
      // add to the container
      desiredPath.push_back(locVel);
    }

  // straight
  for(; xx <= 10.0; xx+= 0.5)
    {
      locVel.loc.x() = xx;
      locVel.loc.y() = yy;
      
      // add to the container
      desiredPath.push_back(locVel);
    }

  // to the right
  for(yy = 0.0; yy <= 2.5; yy+= 0.5)
    {
      locVel.loc.x() = xx;
      locVel.loc.y() = yy;
      
      // add to the container
      desiredPath.push_back(locVel);
    }

  // straight
  for(; xx <= 12.5; xx+= 0.5)
    {
      locVel.loc.x() = xx;
      locVel.loc.y() = yy;
      
      // add to the container
      desiredPath.push_back(locVel);
    }

  // to the left
  for(; yy >= 0.0; yy-= 0.5)
    {
      locVel.loc.x() = xx;
      locVel.loc.y() = yy;
      
      // add to the container
      desiredPath.push_back(locVel);
    }

  // straight
  for(; xx <= 15.0; xx+= 0.5)
    {
      locVel.loc.x() = xx;
      locVel.loc.y() = yy;
      
      // add to the container
      desiredPath.push_back(locVel);
    }

#endif

  // construct the vehicle state object (we start at 0, 0, 0)
  ddt::VehicleState vs;

  std::vector<vmi::SimResults_T> resultsNoSlip;
  std::vector<vmi::SimResults_T> resultsWithSlip;

  // some large simulation time
  const double simTime(100.0);
  
  // forward simulate to get the controls
  if(!fwdSim.simulate(vs, 
                      desiredPath, 
                      resultsNoSlip, 
                      resultsWithSlip,
                      simTime))
    {
      ROS_FATAL("could not forward simulate the path!!");
      return -1;
    }

  // initialize 
  ros::init(argc, argv, "husky_openloop_control_node");

  // the node handle to communicate with the ros subsystem
  ros::NodeHandle nodeHandle;

  // the topic name to publish on 
  const std::string topicName("plan_cmd_vel");

  // the queue size (not sure how big this should be)
  const uint32_t queueSize(1);

  // send the last command to subscribers that are newly connecting
  const bool latch(false);
  
  // instantiate the publisher object
  ros::Publisher publisher = 
    nodeHandle.advertise<geometry_msgs::Twist>(topicName, queueSize, latch);

  // subscriber
  ros::Subscriber joySubscriber =
    nodeHandle.subscribe<sensor_msgs::Joy>("joy", 10, updateJoystickState);

  // the cycle rate
  ros::Rate rateController(cycleRate);

  // the desired command
  geometry_msgs::Twist cmd;

  std::size_t cmdIdx(0);
  // loop almost forever
  while(ros::ok())
    {
      // this will spin as fast as possible
      if(!launchButtonPressed)
        {
          // send a default message just before quitting
          cmd = geometry_msgs::Twist();

          publisher.publish(cmd);
          ros::spinOnce();
        }
      else
        {
          // compose the command
          cmd.linear.x = resultsNoSlip[cmdIdx].desiredSpeed;
          cmd.angular.z = -(cmd.linear.x * resultsNoSlip[cmdIdx].desiredCurvature);

          ROS_WARN("Step %lu, Vx: %.3f, Vw: %.3f",cmdIdx, cmd.linear.x, cmd.angular.z);
          
          publisher.publish(cmd);
          ros::spinOnce();
          
          // to the next index
          if(++cmdIdx >= resultsNoSlip.size())
            {
              break;
            }
        }

      // rate limit
      rateController.sleep();
    }

  // send a default message just before quitting
  cmd = geometry_msgs::Twist();
  publisher.publish(cmd);
  ros::spinOnce();
  
  return 0;
}

