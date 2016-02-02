//! Mode:c++

//===============================================================================
/*! 
  \file ForwardSimulator.h

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
#ifndef Forward_Simulator_h
#define Forward_Simulator_h

//-------------------------------------------------------------------------------
// INCLUDES
//-------------------------------------------------------------------------------
#include <boost/noncopyable.hpp>
#include <boost/shared_ptr.hpp>
#include <boost/log/sources/severity_logger.hpp>

#include <geometry/primitives/Point2D.h>

#include <Simple3DSim/Simple3DSim.h>
#include <VehicleState/VehicleState.h>

#include "PathTracker.h"


//-------------------------------------------------------------------------------
namespace vmi
{

  //-----------------------------------------------------------------------------
  // partial results of the simulation
  struct SimResults_T
  {
    nrec::geometry::ISOPose3D_d pose;
    nrec::geometry::ISOPose3D_d relPose;
    double desiredSpeed;
    double desiredCurvature;

    double actualSpeed;
    double actualCurvature;
  };

  //-----------------------------------------------------------------------------
  class ForwardSimulator : public boost::noncopyable 
  {
  public:

    //---------------------------------------------------------------------------
    // the explicit constructor that takes in a shared pointer instance to 
    // the vehicle kinematic simulator
     ForwardSimulator(boost::shared_ptr<ddt::Simple3DSim> sim,
                      double pathTrackerCycleRate);


    //---------------------------------------------------------------------------
    // destructor
    ~ForwardSimulator();

    //---------------------------------------------------------------------------
    // simulate
    bool simulate(const ddt::VehicleState& vs, 
                  const std::vector<LocVel_T>& desiredPath,
                  std::vector<SimResults_T>& resultsNoSlip,
                  std::vector<SimResults_T>& resultsWithSlip,
                  const double simulationTime = 4.0);
                  
  private:

    // default constructor is private here
    ForwardSimulator();

    // instance of the path tracker
    PathTracker m_pathTracker;

    // the vehicle kinematics simulator
    boost::shared_ptr<ddt::Simple3DSim> m_sim;
    
    // the path 
    std::vector<LocVel_T> m_desiredPath;

    // the computed desired steering radius
    double m_desiredRadius;

    // the computed desired longitudinal velocity
    double m_desiredSpeed;

    // the cycle rate of the simulator
    const double m_simCycleRate;

    // the cycle rate of the path tracker
    double m_PTCycleRate;

    // the logging object for this class
    static boost::log::sources::severity_logger<> m_logger;

  };


} // end namespace scope

#endif // PathTracker_Sim_h
