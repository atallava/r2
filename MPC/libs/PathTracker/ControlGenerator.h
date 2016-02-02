//! Mode:c++

//===============================================================================
/*! 
  \file ControlGenerator.h

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
#ifndef Control_Generator_h
#define Control_Generator_h

//-------------------------------------------------------------------------------
// INCLUDES
//-------------------------------------------------------------------------------
#include <boost/noncopyable.hpp>
#include <boost/shared_ptr.hpp>
#include <boost/log/sources/severity_logger.hpp>

#include <eigen3/Eigen/Core>
#include <eigen3/Eigen/StdVector>

#include "PathTracker.h"
#include "ForwardSimulator.h"

//-------------------------------------------------------------------------------
namespace vmi
{
  //-----------------------------------------------------------------------------
  class ControlGenerator : public boost::noncopyable 
  {
  public:
    
    //---------------------------------------------------------------------------
    // the explicit constructor that takes in a shared pointer instance to 
    // the vehicle kinematic simulator
    explicit ControlGenerator(boost::shared_ptr<ddt::Simple3DSim> sim,
                              double pathTrackerCycleRate);
    

    //---------------------------------------------------------------------------
    // destructor
    ~ControlGenerator();

    //---------------------------------------------------------------------------
    // set the desired path
    bool setDesiredPath(const std::vector<LocVel_T>& desiredPath);

    //---------------------------------------------------------------------------
    bool computeControls(const ddt::VehicleState& vs,
                         double& desiredCurvature,
                         double& desiredSpeed,
                         const bool computeOptimalControl = false);

  private:

    // useful typedefs
    typedef Eigen::Matrix<double,8,1> State_T;
    typedef Eigen::Matrix<double,8,8> StateTransition_T;
    typedef Eigen::Matrix<double,8,8> StatePenalty_T;
    typedef Eigen::Matrix<double,8,2> InputTransition_T;
    typedef Eigen::Matrix<double,2,8> Gain_T;
    typedef Eigen::Matrix<double,2,2> InputPenalty_T; 
    typedef Eigen::Matrix<double,2,1> Control_T;
    

    // default constructor is private here
    ControlGenerator();
    
    // shared access to the simulator
    boost::shared_ptr<ddt::Simple3DSim> m_sim;
    
    // the good-old path tracker
    PathTracker m_pt;

    // the desired path
    std::vector<LocVel_T> m_desiredPath;

    // forward simulator
    ForwardSimulator m_fwdSimulator;

    // the cycle rate of the path tracker
    double m_DT;

    // the path localizer to prune the desired path
    PathLocalizer m_pathLocalizer;

    //
    static boost::log::sources::severity_logger<> m_logger;

  };

}

#endif // Optimal_Controller_h
