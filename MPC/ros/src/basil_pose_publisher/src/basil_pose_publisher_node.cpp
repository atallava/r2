//===============================================================================
/*! 
  \file basi_pose_publisher_node.cpp

  \author Venkat Rajagopalan
  \date   29/10/2015
  
  \attention
  Copyright (C) 2015 \n
  National Robotics Engineering Center (NREC) \n
  Robotics Institute
  Carnegie Mellon University
  All Rights Reserved \n
*/
//===============================================================================

#include <cstdio>
#include <fstream>

#include <boost/shared_ptr.hpp>

#include <ros/ros.h>
#include <ros/console.h>
#include <std_msgs/String.h>

#include <Utils/FileUtil.h>
#include <Utils/UDPConnection.h>

#include "PoseDataGatherer.h"
#include "sinsmsg.h"

//-------------------------------------------------------------------------------
// global members
ros::Publisher localPosePublisher, globalPosePublisher;


//-------------------------------------------------------------------------------
// the slot mechanism
void newInsMsgSlot(BASIL::INS::InsMsg& msg)
{

  // the pose message that will be sent out
  std_msgs::String poseMsg;
  
  // here, serialize the data 
  if(!BASIL::INS::toString(msg, poseMsg.data))
    {
      ROS_FATAL("Error serializing pose data!!");
      throw std::runtime_error("Error serializing pose data!!");
    }

  // publish the data on the appropriate channel
  if(0 == msg.header.deviceId)
    {
      localPosePublisher.publish(poseMsg);
    }
  else if(2 == msg.header.deviceId)
    {
      globalPosePublisher.publish(poseMsg);
    } 
}

//-------------------------------------------------------------------------------
int main(int argc, char* argv[])
{
  // the desired cycle rate of the algorithm
  const double cycleRate(200.0);

  // // the io service object
  // boost::shared_ptr<boost::asio::io_service> ios;
  // ios.reset(new boost::asio::io_service());

  // // the connection to the pose system (this needs to be setup at some time)
  // ddt::UDPConnection resetCon(ios);

  // // TODO - instantiate the UDP connection 
  // const std::string hostName("192.168.101.255");
  // const uint16_t resetPortNumber(1533);

  // // connect to the pose solution stream
  // if(!resetCon.connect(hostName, resetPortNumber, false))
  //   {
  //     ROS_FATAL("Error connecting to pose server on host: %s and port %d", 
  //               hostName.c_str(),
  //               resetPortNumber);

  //     return -1;
  //   }

  // initialize 
  ros::init(argc, argv, "basil_pose_publisher_node");

  // the node handle to communicate with the ros subsystem
  ros::NodeHandle nodeHandle;

  // the queue size (not sure how big this should be)
  const uint32_t queueSize(1);

  // send the last pose to subscribers that are newly connecting
  const bool latch(true);
  
  // instantiate the publisher object
  localPosePublisher = 
    nodeHandle.advertise<std_msgs::String>(BASIL::INS::LOCAL_POSE_TOPIC, queueSize, latch);

  // instantiate the publisher object
  globalPosePublisher = 
    nodeHandle.advertise<std_msgs::String>(BASIL::INS::GLOBAL_POSE_TOPIC, queueSize, latch);
  
  // the cycle rate
  ros::Rate rateController(cycleRate);

  // the serialized pose data message that will be published
  std_msgs::String poseMsg;

  // break out of the loop?
  bool quit(false);

  // instance of the pose data gatherer
  vmi::PoseDataGatherer poseDataGatherer;

  //
  vmi::PoseDataGatherer::InsMsgSlot_T slot(boost::bind(&newInsMsgSlot, _1)); 
  poseDataGatherer.connectInsMsgSignal(slot);

  // loop almost forever
  while(ros::ok())
    {
      // callbacks and 
      ros::spinOnce();

      // rate limit the loop
      rateController.sleep();
    }

  return 0;
}


