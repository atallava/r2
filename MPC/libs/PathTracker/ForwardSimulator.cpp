//===============================================================================
/*! 
  \file ForwardSimulator.cpp

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

#include "ForwardSimulator.h"

//-------------------------------------------------------------------------------
using namespace boost::log::trivial;
//-------------------------------------------------------------------------------

//-------------------------------------------------------------------------------
namespace vmi
{

  //-----------------------------------------------------------------------------
  boost::log::sources::severity_logger<> ForwardSimulator::m_logger;

  //-----------------------------------------------------------------------------
  ForwardSimulator::ForwardSimulator(boost::shared_ptr<ddt::Simple3DSim> sim,
                                     double pathTrackerCycleRate):
    m_sim(sim),
    m_simCycleRate(100.0),
    m_PTCycleRate(pathTrackerCycleRate)
  {
    m_sim->setTimeStep(1.0E-02);
  }

  //-----------------------------------------------------------------------------
  ForwardSimulator::~ForwardSimulator()
  {
  }

  //-----------------------------------------------------------------------------
  bool ForwardSimulator::simulate(const ddt::VehicleState& initVS,
                                  const std::vector<LocVel_T>& desiredPath,
                                  std::vector<SimResults_T>& resultsNoSlip,
                                  std::vector<SimResults_T>& resultsWithSlip,
                                  const double simulationTime)
  {

    //
    resultsNoSlip.clear();
    resultsWithSlip.clear();

    if(!m_pathTracker.setDesiredPath(desiredPath))
      {
        BOOST_LOG_SEV(m_logger, error) << "Error setting the desired path!";
        return false;
      }


    nrec::geometry::ISOPose3D_d initPose;

#if 1
    initPose.y() = initVS.getNavState().m_tranAbsX;
    initPose.x() = initVS.getNavState().m_tranAbsY;
#else // snap the vehicle 

    // need to swap the easting and northing in initVS
    ddt::NavState navStateMod = initVS.getNavState();
    navStateMod.m_tranAbsX = initVS.getNavState().m_tranAbsY;
    navStateMod.m_tranAbsY = initVS.getNavState().m_tranAbsX;
    
    ddt::VehicleState initVSMod;
    initVSMod.setNavState(navStateMod);

    // the closest point on the path
    nrec::geometry::Point2D_d closestPt;
    if(!m_pathTracker.getClosestPoint(initVSMod, closestPt))
      {
        BOOST_LOG_SEV(m_logger, error) << "Error computing the closest point!!";
        return false;
      }

    initPose.y() = closestPt.x();
    initPose.x() = closestPt.y();

#endif

    // first stage - disable the slipAndTransients
    m_sim->disableSlipAndTransients();
    
    // the current yaw
    const double currentYawRad = nrec::geometry::Angle_d::degToRad * 
      initVS.getNavState().m_tranAbsYaw.getDegrees();

    // for the rotation, the only field that matters is the yaw
    initPose.yaw().setRadians(currentYawRad);

    // setup the simulator with the initial conditions
    if(!m_sim->reset(initPose, 
                     initVS.getNavState().m_velX, 
                     initVS.getNavState().m_velY, 
                     initVS.getNavState().m_yawRate))
      {
        BOOST_LOG_SEV(m_logger, info) << "could not setup the simulator!!!";
        return false;
      }

    // compute the number of simulation timesteps
    const std::size_t maxSteps = simulationTime/m_sim->getTimeStep();

    // the desired speed 
    double desiredSpeed(0.0);

    // the desired steering radius
    double desiredRadius(1.0E06);

    // the desired curvature
    double desiredCurvature(0.0);

    // the current number of steps
    std::size_t numSteps(0);

    ddt::VehicleState vs(initVS);

    ddt::NavState navState;

    // compute the number of simulation cycles to skip before calling the path tracker
    const std::size_t controlSkipSteps = m_simCycleRate/m_PTCycleRate;

    // save the simulation data?
    bool saveSimData(false);

    // the container of simulation results
    resultsNoSlip.reserve(1 + maxSteps/controlSkipSteps);
    resultsWithSlip.reserve(1 + maxSteps/controlSkipSteps);

    // one instance of the simulation result
    SimResults_T result;

    // the linear velocity
    nrec::geometry::Point3D_d linVel;

    // iterate till we hit the maximum number of steps 
    // or till the end of the path
    while(numSteps < maxSteps)
      {
        // if it is time to compute a new control command, do so now
        if(0 == numSteps%controlSkipSteps)
          {
            // set saveSimData to true
            saveSimData = true;
            
            // compute the desired controls
            if(!m_pathTracker.computeControls(vs, desiredRadius, desiredSpeed))
              {
                BOOST_LOG_SEV(m_logger, error) << "Error computing the desired "
                                               << " steering radius and speed!!";
                return false;
              }

            // update desiredCurvature - need to invert to get this to work 
            // in the simulator
            desiredCurvature = 1.0/desiredRadius;


            // pass along the controls to the simulator
            m_sim->setCommandCurvature(desiredSpeed, desiredCurvature);

          }
        else // still following the old control
          {
            saveSimData = false;
          }
        
        // forward simulate
        if(!m_sim->step())
          {
            BOOST_LOG_SEV(m_logger, error) << "Could not call the step method!!";
            return false;
          }

        // need to update vehicle state here
        result.pose = m_sim->getPose();

        // partial update of the navState
        navState.m_tranAbsX = result.pose.x();
        navState.m_tranAbsY = result.pose.y();
        navState.m_tranAbsZ = result.pose.z();
        navState.m_tranAbsYaw.setDegrees(result.pose.yaw().getDegrees());

        // update vs
        vs.setNavState(navState);

        // save the results if necessary
        if(saveSimData)
          {
            // update the actual pose
            result.pose = m_sim->getPose();

            // update the actual pose
            result.relPose = m_sim->getRelPose();

            // update the desired speed and curvature
            result.desiredSpeed = desiredSpeed;
            result.desiredCurvature = desiredCurvature;

            // update actual speed (over ground)
            linVel = m_sim->getVelocity();
            result.actualSpeed = hypot(linVel.x(), linVel.y());

            // update the actual curvature - hack hack hack
            if(0.0 == result.actualSpeed)
              {
                result.actualCurvature = 0.0;
              }
            else
              {
                result.actualCurvature = 
                  m_sim->getAngularVelocity().z()/result.actualSpeed;
              }
            
            // save result
            resultsNoSlip.push_back(result);
          }

        // if the desiredSpeed is 0.0, we are at the end of the line
        if(0.0 == desiredSpeed)
          {
            break;
          }

        // increment the number of steps
        ++numSteps;

      }

    // reset numSteps
    numSteps = 0;
    std::size_t idx = 0;

    // update init pose
    initPose.y() = initVS.getNavState().m_tranAbsX;
    initPose.x() = initVS.getNavState().m_tranAbsY;

    // setup the simulator with the initial conditions
    if(!m_sim->reset(initPose, 
                     initVS.getNavState().m_velX, 
                     initVS.getNavState().m_velY, 
                     initVS.getNavState().m_yawRate))
      {
        BOOST_LOG_SEV(m_logger, info) << "could not setup the simulator!!!";
        return false;
      }

    // std::cout << "enabling transients and running the forward simulation!!"
    //           << std::endl;

    // enable the slip and transients
    m_sim->enableSlipAndTransients();

    // iterate till we hit the maximum number of steps 
    // or till the end of the path
    while(numSteps < maxSteps)
      {
        // if it is time to compute a new control command, do so now
        if(0 == numSteps%controlSkipSteps)
          {
            // set saveSimData to true
            saveSimData = true;
            
            // here, get the commanded curvature  and speed from the resultsNoSlip
            desiredSpeed = resultsNoSlip[idx].desiredSpeed;

            // NO need to invert the desired curvature again
            desiredCurvature = resultsNoSlip[idx].desiredCurvature;

            // pass along the controls to the simulator
            m_sim->setCommandCurvature(desiredSpeed, desiredCurvature);

            // to the next index
            idx++;
            
          }
        else // still following the old control
          {
            saveSimData = false;
          }
        
        // forward simulate
        if(!m_sim->step())
          {
            BOOST_LOG_SEV(m_logger, error) << "Could not call the step method!!";
            return false;
          }

        // need to update vehicle state here
        result.pose = m_sim->getPose();

        // partial update of the navState
        navState.m_tranAbsX = result.pose.x();
        navState.m_tranAbsY = result.pose.y();
        navState.m_tranAbsZ = result.pose.z();
        navState.m_tranAbsYaw.setDegrees(result.pose.yaw().getDegrees());

        // update vs
        vs.setNavState(navState);

        // save the results if necessary
        if(saveSimData)
          {
            // update the actual pose
            result.pose = m_sim->getPose();

            // update the actual pose
            result.relPose = m_sim->getRelPose();

            // update the desired speed and curvature
            result.desiredSpeed = desiredSpeed;
            result.desiredCurvature = desiredCurvature;

            // update actual speed (over ground)
            linVel = m_sim->getVelocity();
            result.actualSpeed = hypot(linVel.x(), linVel.y());

            // update the actual curvature - hack hack hack
            if(0.0 == result.actualSpeed)
              {
                result.actualCurvature = 0.0;
              }
            else
              {
                result.actualCurvature = 
                  m_sim->getAngularVelocity().z()/result.actualSpeed;
              }
            
            // save result
            resultsWithSlip.push_back(result);
          }

        // increment the number of steps
        ++numSteps;

      }


    return true;
  }

}
