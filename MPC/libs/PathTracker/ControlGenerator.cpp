//===============================================================================
/*! 
  \file ControlGenerator.cpp

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

//-------------------------------------------------------------------------------
// INCLUDES
//-------------------------------------------------------------------------------
#include "ControlGenerator.h"


//-------------------------------------------------------------------------------
using namespace boost::log::trivial;
//-------------------------------------------------------------------------------

//-------------------------------------------------------------------------------
namespace vmi
{
  //-----------------------------------------------------------------------------
  boost::log::sources::severity_logger<> ControlGenerator::m_logger;

  //-----------------------------------------------------------------------------
  ControlGenerator::ControlGenerator(boost::shared_ptr<ddt::Simple3DSim> sim,
                                     double pathTrackerCycleRate):
    m_sim(sim),
    m_fwdSimulator(sim, pathTrackerCycleRate),
    m_DT(1.0/pathTrackerCycleRate)
  {
  }

  //-----------------------------------------------------------------------------
  ControlGenerator::~ControlGenerator()
  {
    // nothing to tear down
  }

  //---------------------------------------------------------------------------
  bool ControlGenerator::setDesiredPath(const std::vector<LocVel_T>& desiredPath)
  {
    m_desiredPath = desiredPath;

    // set the desired path in the path tracker
    if(!m_pt.setDesiredPath(m_desiredPath))
      {
        BOOST_LOG_SEV(m_logger, error) << "Error setting the desired path!";
        return false;
      }

    for(std::vector<LocVel_T>::const_iterator iter = desiredPath.begin();
        iter != desiredPath.end();
        ++iter)
      {
        m_pathLocalizer.addPoint(iter->loc, iter->speed);
      }
    
    return true;
  }

  //---------------------------------------------------------------------------
  bool ControlGenerator::computeControls(const ddt::VehicleState& vs,
                                         double& desiredCurvature,
                                         double& desiredSpeed,
                                         const bool computeOptimalControl)
  {
    // stop the vehicle
    desiredCurvature = 0.0;
    desiredSpeed = 0.0;

    // if requested to not compute optimal control, just return the control
    // from the path tracker
    if(!computeOptimalControl)
      {
        double desiredRadius(1.0E06);
        if(!m_pt.computeControls(vs, desiredRadius, desiredSpeed))
          {
            return false;
          }

        // update desired curvature (negate the sign of the desiredRadius)
        desiredCurvature = 1.0/desiredRadius;

      }
    else // requested to run optimal control
      {
        // compute the simulation results without slip
        const double simTime(2.0);

        // 
        std::vector<SimResults_T> resultsNoSlip;
        std::vector<SimResults_T> resultsWithSlip;
        if(!m_fwdSimulator.simulate(vs, 
                                    m_desiredPath,
                                    resultsNoSlip,
                                    resultsWithSlip,
                                    simTime))
          {
            BOOST_LOG_SEV(m_logger, error) << "Error forward simulating "
                                           << "with NO slip!!";
            return false;
          }
        
        // the state vector
        std::vector<State_T,Eigen::aligned_allocator<State_T> > x_hat;
        x_hat.resize(resultsNoSlip.size());

        // the angle, the cosine and sine for the angle
        double t1(0.0), c1(0.0), s1(0.0);

        // the angle the cosine and sine for the angle
        double t2(0.0), c2(0.0), s2(0.0);

        double v1(0.0), v2(0.0);
        double k1(0.0), k2(0.0);

        //
        std::vector<double> c1_(x_hat.size());
        std::vector<double> s1_(x_hat.size());
        std::vector<double> c2_(x_hat.size());
        std::vector<double> s2_(x_hat.size());

        std::vector<double> k1_(x_hat.size());
        std::vector<double> v1_(x_hat.size());
        std::vector<double> k2_(x_hat.size());
        std::vector<double> v2_(x_hat.size());

        //
        std::cout.setf(std::ios_base::fixed);
        std::cout.precision(3);


        // populate the state vector
        for(std::size_t idx = 0; idx < resultsNoSlip.size(); idx++)
          {
            // 
            t1 = resultsWithSlip[idx].pose.yaw().getRadians();
            v1 = resultsWithSlip[idx].desiredSpeed;
            k1 = resultsWithSlip[idx].desiredCurvature;
            sincos(t1, &s1, &c1);

            //
            s1_[idx] = s1;
            c1_[idx] = c1;
            k1_[idx] = k1;
            v1_[idx] = v1;
            

            t2 = resultsNoSlip[idx].pose.yaw().getRadians();
            v2 = resultsNoSlip[idx].desiredSpeed;
            k2 = resultsNoSlip[idx].desiredCurvature;
            sincos(t2, &s2, &c2);

            //
            s2_[idx] = s2;
            c2_[idx] = c2;
            k2_[idx] = k2;
            v2_[idx] = v2;

            // deltas
            x_hat[idx](0,0) = 
              resultsWithSlip[idx].pose.x() - resultsNoSlip[idx].pose.x();
            x_hat[idx](1,0) = 
              resultsWithSlip[idx].pose.y() - resultsNoSlip[idx].pose.y();
            x_hat[idx](2,0) = 
              (resultsWithSlip[idx].pose.yaw() - resultsNoSlip[idx].pose.yaw()).getRadians();

            // //
            // std::cout << idx << ","
            //           << resultsNoSlip[idx].pose.x() << "," 
            //           << resultsNoSlip[idx].pose.y() << ","
            //           << resultsNoSlip[idx].pose.yaw().getDegrees() << ","
            //           << resultsNoSlip[idx].desiredSpeed << ","
            //           << resultsNoSlip[idx].desiredCurvature << ","
            //           << resultsWithSlip[idx].pose.x() << "," 
            //           << resultsWithSlip[idx].pose.y() << ","
            //           << resultsWithSlip[idx].pose.yaw().getDegrees() << ","
            //           << resultsWithSlip[idx].desiredSpeed << ","
            //           << resultsWithSlip[idx].desiredCurvature << ","
            //           << x_hat[idx](0,0) << ","
            //           << x_hat[idx](1,0) << ","
            //           << x_hat[idx](2,0)  << std::endl;

            // delta dots
            x_hat[idx](3,0) = v1*c1 - v2*c2;
            x_hat[idx](4,0) = v1*s1 - v2*s2;
            x_hat[idx](5,0) = v1*k1 - v2*k2;
            
            // controls
            x_hat[idx](6,0) = 
              resultsWithSlip[idx].desiredSpeed;
            x_hat[idx](7,0) = 
              resultsWithSlip[idx].desiredCurvature;
          }

        // A Matrix
        StateTransition_T A; 

        // B
        InputTransition_T B; 

        // the Q Matrix - weights for the state 
        StatePenalty_T Q; Q.setIdentity(); 
        Q(2,2) *= nrec::geometry::Angle_d::radToDeg; 
        Q(5,5) *= nrec::geometry::Angle_d::radToDeg * m_DT;

        // the R Matrix
        InputPenalty_T R; R.setIdentity(); 
        R(0,0) *= 1.0E05;
        // R(1,1) *= 1.0E05;

        // the control 
        std::vector<Control_T,Eigen::aligned_allocator<Control_T> > U;
        
        //  the running value function
        StatePenalty_T V; V.setZero();

        // the kalman gain matrix over the horizon
        std::vector<Gain_T, Eigen::aligned_allocator<Gain_T> > gains;
        gains.resize(resultsNoSlip.size());
        U.resize(resultsNoSlip.size());
        
        // iterate through and compute the kalman gain and the value function
        for(int32_t idx = x_hat.size() -1; idx >= 0; idx--)
          {
            // set the diagonals
            A.setIdentity();

            //
            A(0,3) = m_DT;
            A(0,6) = m_DT * c1_[idx];

            //
            A(1,4) = m_DT;
            A(1,6) = m_DT * s1_[idx];

            A(2,5) = m_DT;
            A(2,6) = m_DT * k1_[idx];
            A(2,7) = m_DT * v1_[idx];

            A(3,6) = c1_[idx];

            A(4,6) = s1_[idx];

            A(5,6) = k1_[idx];
            A(5,7) = v1_[idx];

            // compose the B Matrix
            B.setZero();
            
            //
            B(0,0) = m_DT * c1_[idx];

            //
            B(1,0) = m_DT * s1_[idx];

            //
            B(2,0) = m_DT * k1_[idx];
            B(2,1) = m_DT * v1_[idx];

            //
            B(3,0) = c1_[idx];
            
            //
            B(4,0) = s1_[idx];

            //
            B(5,0) = k1_[idx];
            B(5,1) = v1_[idx];

            //
            B(6,0) = 1.0;
            
            //
            B(7,1) = 1.0;

            // compute the gain
            gains[idx] = 
              (R.transpose() + B.transpose() * V * B).inverse() * B.transpose() * V * A;
            
            // compute the adjusted control
            U[idx]  = -gains[idx] * x_hat[idx];

            // compute the adjusted V
            V = (Q + (gains[idx].transpose() * R * gains[idx]) + V);
          }

        // compute the adjusted curvature
        desiredCurvature = resultsNoSlip[0].desiredCurvature - U[0].y();

        // std::cout << "XHat: \n" << x_hat[0]
        //           << "\nGains: \n" << gains[0]
        //           << "\nU: \n" << U[0] 
        //           << "\nDC: " << desiredCurvature
        //           << std::endl;

        // return false;

        // update the optimal desired speed
        if(std::fabs(resultsWithSlip[0].desiredSpeed) < 1.0E-03)
          {
            desiredSpeed = 0.0;
          }
        else // update
          {
            desiredSpeed = resultsNoSlip[0].desiredSpeed - U[0].x();
          }
      }

    return true;
  }

 
 
}

