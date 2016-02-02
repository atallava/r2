//===============================================================================
/*! 
  \file testApp.PathTracker.cpp

  \author Venkat Rajagopalan
  \date   02/18/2014
  
  \attention
  Copyright (C) 2011 \n
  National Robotics Engineering Center (NREC) \n
  Robotics Institute
  Carnegie Mellon University
  All Rights Reserved \n
*/
//===============================================================================
#include <boost/shared_ptr.hpp>
#include <boost/make_shared.hpp>
#include <boost/thread.hpp>

#include <Utils/FileUtil.h>
#include <Utils/IPCBase.h>
#include <Utils/GlobalTime.h>

#include <PathTracker/ControlGenerator.h>
#include <Simple3DSim/VehicleSimParams.h>
#include <VehicleState/VehicleState.h>

#include <Plugins/CameraSim/CameraSimParams.h>

#include <sys/signal.h>

//-------------------------------------------------------------------------------
static bool gs_quit(false);

//-------------------------------------------------------------------------------
void signalHandler(int signal)
{
  if (SIGINT == signal)
    {
      gs_quit = true;
    }
}


//-------------------------------------------------------------------------------
int main(int argc, char* argv[])
{
  
  // not requested to quit
  gs_quit = false;

  // setup the signal handler
  signal(SIGINT, signalHandler);

  // the parameters file for the vehicle simulator
  ddt::VehicleSimParams vehicleParams;

  // the path comes in from "other models in the camera simulation section"
  ddt::CameraSimParams camParams;

  // load the parameters from file
  const std::string fileName("VehicleSimParams.xml");
  const std::string dirName("PathTracker");
  std::string paramsFileName;


  if(!ddt::FileUtil::buildParamsFileName(fileName,
                                         dirName,
                                         paramsFileName))
    {
      std::cerr << "Could not find the parameter file:  "
                << fileName
                << " in the directory: " 
                << dirName << std::endl;
      return -1;
    }
  
  // load the parameters
  try
    {
      vehicleParams.loadParams(paramsFileName);
      camParams.loadParams(paramsFileName);
    }
  catch(const std::runtime_error& e)
    {
      std::cerr << "Exception caught: " << e.what() << std::endl;
      return -1;
    }

  // the instance of the controlIPC object
  boost::shared_ptr<ddt::IPCBase> controlIPC = 
    boost::make_shared<ddt::IPCBase>(vehicleParams.m_controlIPCBufferName);

  // the instance of the statusIPC object
  boost::shared_ptr<ddt::IPCBase> statusIPC = 
    boost::make_shared<ddt::IPCBase>(vehicleParams.m_statusIPCBufferName);


  // the instance of the controlIPC object
  boost::shared_ptr<ddt::IPCBase> shadowControlIPC = 
    boost::make_shared<ddt::IPCBase>(vehicleParams.m_shadowControlIPCBufferName);

  // the instance of the statusIPC object
  boost::shared_ptr<ddt::IPCBase> shadowStatusIPC = 
    boost::make_shared<ddt::IPCBase>(vehicleParams.m_shadowStatusIPCBufferName);


  // populate the desired path
  std::vector<vmi::LocVel_T> desiredPath;
  vmi::LocVel_T locVel;
  double desiredVel = 5.0;
  locVel.speed = desiredVel;

  std::map<std::string, ddt::CameraSimParams::ModelPoses_T>::const_iterator iter;
  for(iter = camParams.m_otherModels.begin();
      iter != camParams.m_otherModels.end();
      ++iter)
    {
      for(std::size_t idx = 0; idx < iter->second.size(); idx++)
        {
          locVel.loc.x() = iter->second.at(idx).y() + camParams.m_originUTMEasting;
          locVel.loc.y() = iter->second.at(idx).x() + camParams.m_originUTMNorthing;

          desiredPath.push_back(locVel);
        }
    }

  // the desired cycle rate
  const double cycleRate(20.0);
  const double cycleTime(1.0/cycleRate);

  // construct the simulator
  boost::shared_ptr<ddt::Simple3DSim> sim = 
    boost::make_shared<ddt::Simple3DSim>(vehicleParams);

  // instantiate the control generator
  vmi::ControlGenerator cg(sim, cycleRate), cgOpt(sim,cycleRate);

  // pass the path along to the path tracker
  if(!cg.setDesiredPath(desiredPath))
    {
      std::cout << "could not set the desired path!!" << std::endl;
      return -1;
    }

  // pass the path along to the path tracker
  if(!cgOpt.setDesiredPath(desiredPath))
    {
      std::cout << "could not set the desired path!!" << std::endl;
      return -1;
    }


  // the default control command
  ddt::VehicleCtrlCmd cmd, cmdOpt;
  cmd.setControlSteering(true);
  cmd.setControlSpeed(true);

  cmdOpt.setControlSteering(true);
  cmdOpt.setControlSpeed(true);


  // the current time
  double cycleStartTime(0.0), deltaTime(0.0);

  // the current vehicle state
  ddt::VehicleState vs, vsOpt;


  // the desired curvature and desired speed
  double desiredCurvature(0.0); double desiredSpeed(0.0);

  // the optimal desired curvature and speed
  double optimalDesiredCurvature(0.0); double optimalDesiredSpeed(0.0);

  // absolute value clip on the curvature
  const double curvatureClip(5.0);

  // iterate almost for ever
  while(!gs_quit)
    {
      //
      cycleStartTime = ddt::GlobalTime::getCurrentGlobalTime();

      // current vehicle state for the regular controller
      if(!shadowStatusIPC->retrieve(vs))
        {
          std::cerr << "Error retrieving current vehicle state from the shadow vehicle controller!!"
                    << std::endl;
          break;
        }

      // current vehicle state for the optimal controller
      if(!statusIPC->retrieve(vsOpt))
        {
          std::cerr << "Error retrieving current vehicle state from the real vehicle controller!!"
                    << std::endl;
          break;
        }
      
      // compute the standard control
      if(!cg.computeControls(vs, desiredCurvature, desiredSpeed))
        {
          std::cerr << "Error computing controls!!" << std::endl;
          break;
        }

      // compute the optimale control
      if(!cgOpt.computeControls(vsOpt, 
                                optimalDesiredCurvature,
                                optimalDesiredSpeed,
                                true))
        {
          std::cerr << "Error computing optimal controls!!" << std::endl;
          break;
        }


      // update the desired speed
      cmd.setSpeed(optimalDesiredSpeed);
      cmdOpt.setSpeed(optimalDesiredSpeed);

      // clip the curvature
      if(desiredCurvature > curvatureClip)
        {
          desiredCurvature = curvatureClip;
        }
      else if(desiredCurvature < -curvatureClip)
        {
          desiredCurvature = -curvatureClip;
        }

      // clip the speed
      if(std::fabs(desiredCurvature) >= curvatureClip)
        {
          cmd.setSpeed(1.0);
        }

      // clip the curvature
      if(optimalDesiredCurvature > curvatureClip)
        {
          optimalDesiredCurvature = curvatureClip;
        }
      else if(optimalDesiredCurvature < -curvatureClip)
        {
          optimalDesiredCurvature = -curvatureClip;
        }

      // clip the speed
      if(std::fabs(optimalDesiredCurvature) >= curvatureClip)
        {
          cmdOpt.setSpeed(1.0);
        }
      
      // update the steering curvature for the default controller
      cmd.setSteeringCurvature(desiredCurvature);

      // update the steering curvature for the optimal controller
      cmdOpt.setSteeringCurvature(optimalDesiredCurvature);

      // publish the command on IPC
      if(!shadowControlIPC->publish(cmd))
        {
          std::cerr << "Error publishing control command over shadow IPC!!"
                    << std::endl;
          // break;
        }

      // publish the command on IPC
      if(!controlIPC->publish(cmdOpt))
        {
          std::cerr << "Error publishing control command over IPC!!"
                    << std::endl;
          break;
        }
      
      // current time
      deltaTime = cycleTime - 
        (ddt::GlobalTime::getCurrentGlobalTime() - cycleStartTime);

      // sleep
      if(deltaTime > 0.0)
        {
          boost::this_thread::sleep(boost::posix_time::milliseconds(deltaTime * 1000));
        }
    }

  return 0;
}
