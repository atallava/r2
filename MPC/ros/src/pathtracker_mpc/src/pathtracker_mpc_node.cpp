//===============================================================================
/*! 
  \file pathtracker_mpc_node.cpp

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

#include <cstdio>
#include <fstream>

#include <boost/shared_ptr.hpp>

#include <ros/ros.h>
#include <ros/console.h>
#include <geometry_msgs/Twist.h>
#include <std_msgs/String.h>

#include <Utils/FileUtil.h>
#include <Utils/UDPConnection.h>

#include <VehicleState/VehicleState.h>
#include <Simple3DSim/Simple3DSim.h>

#include <PathTracker/ControlGenerator.h>
#include <PathTracker/PathLocalizer.h>

#include "sinsmsg.h"

//-------------------------------------------------------------------------------
// global, static instance of the pose message
//-------------------------------------------------------------------------------
static BASIL::INS::InsMsg gsPoseMsg;
static bool gsValidPoseMsg(false);

//-------------------------------------------------------------------------------
bool poseMsgCallback(const std_msgs::String::ConstPtr& msg);

bool resetPose(ddt::UDPConnection& udpConnection);

bool loadPath(const std::string fileName, std::vector<vmi::LocVel_T>& desiredPath);

bool parseLocalPoseFile(const char* localPoseFile);

//-------------------------------------------------------------------------------
int main(int argc, char* argv[])
{
  // if we have arguments, we are just parsing the local pose file
  if(2 == argc)
    {
      parseLocalPoseFile(argv[1]);
      return 0;
    }

  // the desired cycle rate of the algorithm
  const double cycleRate(10.0);

  // the io service object
  boost::shared_ptr<boost::asio::io_service> ios;
  ios.reset(new boost::asio::io_service());

  // the connection to the pose system (this needs to be setup at some time)
  ddt::UDPConnection resetCon(ios);

  // TODO - instantiate the UDP connection 
  const std::string hostName("127.0.0.1");
  const uint16_t resetPortNumber(1533);

  // connect to the pose solution stream
  if(!resetCon.connect(hostName, resetPortNumber, false))
    {
      ROS_FATAL("Error connecting to pose server on host: %s and port %d", 
                hostName.c_str(),
                resetPortNumber);

      return -1;
    }

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

  // instantiate the control generator
  vmi::ControlGenerator controlGenerator(sim, cycleRate);

  // TODO - load the desired path  and pass that along 
  std::vector<vmi::LocVel_T> desiredPath;

  const std::string pathFileName("DesiredPath.txt");
  std::string fullPathFileName;
  if(!ddt::FileUtil::buildParamsFileName(pathFileName,
                                         dirName,
                                         fullPathFileName))
    {
      ROS_FATAL("cant find the path file %s in dir %s",
                pathFileName.c_str(),
                dirName.c_str());
      return -1;
    }

  // load the desired path
  if(!loadPath(fullPathFileName, desiredPath))
    {
      ROS_FATAL("couldnt load the desired path file %s",
                fullPathFileName.c_str());
      return -1;
    }

  // pass the desired path along
  if(!controlGenerator.setDesiredPath(desiredPath))
    {
      std::cerr << "Error passing along the desired path to the "
		<< " control generator object!"
		<< std::endl;
      return -1;
    }

  // initialize 
  ros::init(argc, argv, "pathtracker_mpc_node");

  // the node handle to communicate with the ros subsystem
  ros::NodeHandle nodeHandle;

  // the topic name to publish on 
  const std::string topicName("plan_cmd_vel");

  // the queue size (not sure how big this should be)
  const uint32_t queueSize(10);

  // send the last command to subscribers that are newly connecting
  const bool latch(false);
  
  // instantiate the publisher object
  ros::Publisher publisher = 
    nodeHandle.advertise<geometry_msgs::Twist>(topicName, queueSize, latch);

  // subscriber for the pose message
  ros::Subscriber subscriber = 
    nodeHandle.subscribe<std_msgs::String>(BASIL::INS::GLOBAL_POSE_TOPIC, 1, poseMsgCallback);
 
  // the cycle rate
  ros::Rate rateController(cycleRate);

  // the desired command
  geometry_msgs::Twist cmd;

  // // send a "reset" command to the the SAGE board
  // resetPose(resetCon);

  // internal representation of the current pose solution
  ddt::VehicleState vs;

  // a reference to the navState part of the vehicle state
  ddt::NavState navState;

  // the desired steering curvature
  double desiredCurvature(0.0);

  // the deisred speed
  double desiredSpeed(0.0);

  // break out of the loop?
  bool quit(false);

  // the global velocity
  Eigen::Vector3f g_dot;

  // the body velocity
  Eigen::Vector3f xi;

  // the SO(2) rotation matrix
  Eigen::Matrix3f rot(Eigen::Matrix3f::Identity());
  double sa, ca;

  // loop almost forever
  while(ros::ok())
    {
      // TODO - sanity on the solution status field
      if(gsValidPoseMsg && BASIL::INS::NAVIGATING == gsPoseMsg.payload.status)
        {
          // data conversion from InsMsg to VehicleState
          navState.m_tranRelX = navState.m_tranAbsX = gsPoseMsg.payload.position[0];
          navState.m_tranRelY = navState.m_tranAbsY = gsPoseMsg.payload.position[1];
          navState.m_tranRelZ = navState.m_tranAbsZ = gsPoseMsg.payload.position[2];

          // update the yaw, pitch and roll
          navState.m_tranAbsYaw.setRadians(gsPoseMsg.payload.orientation[2]);
	  
          navState.m_roll.setDegrees(gsPoseMsg.payload.orientation[0]);
          navState.m_pitch.setDegrees(gsPoseMsg.payload.orientation[1]);
          navState.m_yaw.setDegrees(gsPoseMsg.payload.orientation[2]);

          // SE(2) world velocity
          g_dot(0) = gsPoseMsg.payload.velocity[0];
          g_dot(1) = gsPoseMsg.payload.velocity[1];
          g_dot(2) = 0.0;

          // compute sine and cosine 
          sincos(gsPoseMsg.payload.orientation[2], &sa, &ca);

          // SO(2) rotation matrix
          rot(0,0) = ca;
          rot(0,1) = -sa;
          rot(1,0) = sa;
          rot(1,1) = ca;

          // the body velocity
          xi = rot.transpose() * g_dot;
          
          // update navState with body velocities
          navState.m_velX = xi(0);
          navState.m_velY = xi(1);
          navState.m_velZ = gsPoseMsg.payload.velocity[2];

          // update the current speed
          navState.m_speed = sqrt(navState.m_velX*navState.m_velX +
                		  navState.m_velY*navState.m_velY);
          
          // update the vehicle state object
          vs.setNavState(navState);

#if 0         
          // here, compute the control and convert it to a Twist message
          if(!controlGenerator.computeControls(vs,
                                               desiredCurvature,
                                               desiredSpeed))
            {
              std::cerr << "Error computing optimal controls!!" << std::endl;
              break;
            }
#else

          // here, compute the control and convert it to a Twist message
          if(!controlGenerator.computeControls(vs,
                                               desiredCurvature,
                                               desiredSpeed,
                                               true))
            {
              std::cerr << "Error computing optimal controls!!" << std::endl;
              break;
            }
#endif
          
          // convert the desiredCurvature and desiredSpeed to a twist message
          cmd.linear.x = desiredSpeed;
          cmd.angular.z = -desiredSpeed * desiredCurvature;
          
          // publish
          publisher.publish(cmd);
        }
      else // the solution status is something other than NAVIGATING
        {
	  //ROS_ERROR("Solution status is something OTHER than navigating!!!");
          cmd = geometry_msgs::Twist();
          publisher.publish(cmd);
        }

      // callbacks and 
      ros::spinOnce();

      // rate limit the loop
      rateController.sleep();
    }

  // send a default message just before quitting
  cmd = geometry_msgs::Twist();
  publisher.publish(cmd);
  ros::spinOnce();
  
  return 0;
}


// //-------------------------------------------------------------------------------
// bool resetPose(ddt::UDPConnection& resetCon)
// {

//   //
//   std::cout << "resetting the local pose solution!!!" << std::endl;

//   // the pose reset packet
//   vmi::PoseResetPacket_T packet;

//   // send the pose reset packet
//   return resetCon.write(&packet, sizeof(vmi::PoseResetPacket_T));
// }

//-----------------------------------------------------------------------------
bool loadPath(const std::string fileName, 
              std::vector<vmi::LocVel_T>& desiredPath)
{
  //
  desiredPath.clear();

  std::ifstream ifp(fileName.c_str());
    
  // a line of data
  std::string line;

  // lines containing comment character (anywhere) are ignored
  const char comment('#');

  // the delimiter for the characters
  const char delimiter(',');

  // the location of the delimiter in the line
  std::string::size_type loc(0);
  std::string::size_type prevLoc(0);
  double value(0.0);
  bool parsedAllFields(false);

  // the location, velocity structure
  vmi::LocVel_T locVel;

  // iterate till the end of the stream
  while(!ifp.eof())
    {
      // grab a "line" from the file
      std::getline(ifp, line);

      // if the line is not empty
      if(!line.empty())
        {
          // if there are no comments in the line
          if(std::string::npos == line.find(comment))
            {
              // iterate through the number of fields (this is fixed)
              loc = 0;
              prevLoc = loc;
              parsedAllFields = false;
              for(std::size_t idx = 0; idx < 4; idx++)
                {
                  if(idx != 3)
                    {
                      // if the line contains no comment characters
                      loc = line.find_first_of(delimiter, prevLoc);

                      if(std::string::npos == loc)
                        {
                          break;
                        }
                    }
                  else
                    {
                      loc = line.length();
                    }
                  
                  // parse out the value
                  value = 
                    std::atof(line.substr(prevLoc, loc - prevLoc).c_str());

                  // update prevLoc
                  prevLoc = loc+1;
                    
                  // update the field based on the value of idx
                  switch(idx)
                    {
                    case 0: 
                      {
                        locVel.loc.x() = value;
                        break;
                      }
                    case 1:
                      {
                        locVel.loc.y() = value;
                        break;
                      }
                    case 2:
                      {
                        locVel.speed = value;
                        break;
                      }
                    case 3:
                      {
                        locVel.yawRate = value;
                        parsedAllFields = true;
                        break;
                      }
                    default:
                      {
                        break;
                      }
                    }
                }

              // if all the fields were parsed, add locVel to m_desiredPath
              if(parsedAllFields)
                {
                  desiredPath.push_back(locVel);
                }
            }
        }
    }
                        
  // close the handle
  ifp.close();

  return !desiredPath.empty();
}

//-------------------------------------------------------------------------------
bool parseLocalPoseFile(const char* localPoseFile)
{
  std::cerr << "Out for repairs!!" << std::endl;
  return false;

  // if(NULL == localPoseFile)
  //   {
  //     return false;
  //   }

  // FILE* fp = fopen(localPoseFile, "rb");
  // if(NULL == fp)
  //   {
  //     std::cerr << "cannot open the file: " << localPoseFile << std::endl;
  //     return false;
  //   }

  // // TODO - load the desired path  and pass that along 
  // std::vector<vmi::LocVel_T> desiredPath;

  // const std::string dirName("PathTracker");
  // const std::string pathFileName("DesiredPath.txt");
  // std::string fullPathFileName;
  // if(!ddt::FileUtil::buildParamsFileName(pathFileName,
  //                                        dirName,
  //                                        fullPathFileName))
  //   {
  //     ROS_FATAL("cant find the path file %s in dir %s",
  //               pathFileName.c_str(),
  //               dirName.c_str());
  //     return -1;
  //   }

  // // load the desired path
  // if(!loadPath(fullPathFileName, desiredPath))
  //   {
  //     ROS_FATAL("couldnt load the desired path file %s",
  //               fullPathFileName.c_str());
  //     return -1;
  //   }

  // vmi::PathLocalizer localizer;
  // for(std::size_t idx = 0; idx < desiredPath.size(); idx++)
  //   {
  //     localizer.addPoint(desiredPath[idx].loc, desiredPath[idx].speed);
  //   }

  // // std::cout << "Total path Length: " << localizer.getTotalPathLength()
  // //           << " m" << std::endl;

  // std::cout.setf(std::ios_base::fixed);
  // std::cout.precision(4);

  // double cy(0.0), sy(0.0);
  // double v_fwd(0.0), v_side(0.0);

  // nrec::geometry::Point2D_d vehicleLoc, closestPt, lookAheadPt;
  // double distToClosestPt(0.0);
  // const double laDist(1.0);
  // std::size_t closestSegmentIdx(0);
  
  // // loop and read
  // while(!feof(fp))
  //   {
  //     BASIL::INS::InsMsg msg;
  //     if(1 != fread(&msg, sizeof(BASIL::INS::InsMsg), 1, fp))
  //       {
  //         // std::cerr << "Error reading elements from file: " << localPoseFile
  //         //           << std::endl;
  //         break;
  //       }
      
  //     // update vehicleLoc
  //     vehicleLoc.x() = msg.payload.position[0];
  //     vehicleLoc.y() = msg.payload.position[1];
      
  //     // localize the vehicle to the path
  //     if(!localizer.localize(vehicleLoc,
  //                                  closestPt,
  //                                  distToClosestPt,
  //                                  closestSegmentIdx,
  //                                  laDist,
  //                                  lookAheadPt,
  //                                  true))
  //       {
  //         std::cerr << "Error localizing the vehicle location to the path!!"
  //                   << std::endl;
  //         return false;
  //       }
  
  //     std::cout << distToClosestPt << ","
  //               << sqrt(msg.payload.velocity[0]*msg.payload.velocity[0]+
  //                       msg.payload.velocity[1]*msg.payload.velocity[1])
  //               << std::endl;
      
  //     // // compute the cosine and sine of the yaw
  //     // sincos(msg.payload.orientation[2], &sy, &cy);

  //     // // rotate the velocity vector
  //     // v_fwd = cy * msg.payload.velocity[0] + sy * msg.payload.velocity[1];
  //     // v_side = -sy * msg.payload.velocity[0] + cy * msg.payload.velocity[1]; 

  //     // std::cout << msg.payload.position[0] << "," 
  //     //           << msg.payload.position[1] << ","
  //     //           << sqrt(v_fwd * v_fwd + v_side * v_side)
  //     //           << std::endl;
  //   }

  // fclose(fp);

  // return true;
  
}

//-------------------------------------------------------------------------------
bool poseMsgCallback(const std_msgs::String::ConstPtr& msg)
{
  // deserialize the message
  if(!BASIL::INS::fromString(msg->data, gsPoseMsg))
    {
      ROS_FATAL("Error de-serializing the data from topic: %s",
                BASIL::INS::GLOBAL_POSE_TOPIC.c_str());
      return false;
    }
  
  // we have a valid pose message
  gsValidPoseMsg = true;

  return true;  
}

