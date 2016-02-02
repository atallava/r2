//===============================================================================
/*! 
  \file PathTracker.cpp

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
#include <fstream>
#include <boost/lexical_cast.hpp>
#include <boost/log/trivial.hpp>


#include "PathTracker.h"

//-------------------------------------------------------------------------------
using namespace boost::log::trivial;
//-------------------------------------------------------------------------------

//-------------------------------------------------------------------------------
namespace vmi
{

  //-----------------------------------------------------------------------------
  const double PathTracker::ms_maxDecel(-2.0);
  const double PathTracker::ms_maxAccel(1.0);
  const double PathTracker::ms_fixedLADist(1.0);
  const double PathTracker::ms_maxDistanceThreshold(15.0);
  const double PathTracker::ms_progressForDone(0.8);
  boost::log::sources::severity_logger<> PathTracker::m_logger;

  //-----------------------------------------------------------------------------
  PathTracker::PathTracker(): 
    m_desiredRadius(1.0E06),
    m_desiredSpeed(0.0)
  {
  }

  //-----------------------------------------------------------------------------
  PathTracker::~PathTracker()
  {
    // nothing to tear down
  }

  //-----------------------------------------------------------------------------
  bool PathTracker::loadPath(const std::string fileName)
  {
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
    LocVel_T locVel;

    // the desired path
    std::vector<LocVel_T> desiredPath;

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
                    // if the line contains no comment characters
                    loc = line.find_first_of(delimiter, loc);
                    if(std::string::npos == loc)
                      {
                        break;
                      }
                    
                    // parse out the value
                    value = 
                      boost::lexical_cast<double>(line.substr(prevLoc,
                                                              loc - prevLoc));

                    // update prevLoc
                    prevLoc = loc;
                    
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

    // update the desired path
    return setDesiredPath(desiredPath);
  }

  //-----------------------------------------------------------------------------
  bool PathTracker::setDesiredPath(const std::vector<LocVel_T>& desiredPath)
  {
    // now, update the path localizer
    m_localizer.reset();
    for(std::vector<LocVel_T>::const_iterator iter = desiredPath.begin();
        iter != desiredPath.end();
        ++iter)
      {
        // add the current point to the path localizer object
        if(!m_localizer.addPoint(iter->loc, iter->speed))
          {
            BOOST_LOG_SEV(m_logger, error) << "Could not add the point"
                                           << iter->loc
                                           << " to the localizer!!";
            return false;
          }
      }

    const std::vector<WPSegment>& path = m_localizer.getPath();
    m_desiredPath.resize(path.size());

    // std::cout << "Number of segments in the path: " << path.size() << std::endl;
    // std::cout.setf(std::ios_base::fixed);
    // std::cout.precision(3);
    for(std::size_t idx = 0; idx < path.size(); idx++)
      {
        m_desiredPath[idx].loc = path[idx].m_start;
        m_desiredPath[idx].speed = path[idx].m_speed;
        // std::cout << idx 
        //           << " [" << path[idx].m_start.x()
        //           << ", " << path[idx].m_start.y() << "] ->" 
        //           << " [" << path[idx].m_end.x()
        //           << ", " << path[idx].m_end.y() << "]" 
        //           << std::endl;
      }

    return !m_desiredPath.empty();
  }

  //-----------------------------------------------------------------------------
  bool PathTracker::getClosestPoint(const ddt::VehicleState& vs,
                                    nrec::geometry::Point2D_d& closestPt)
  {
    double distToClosestPt(0.0);
    std::size_t closestSegmentIdx(0);
    nrec::geometry::Point2D_d lookAheadPoint;

    nrec::geometry::Point2D_d vehicleLoc;
    const ddt::NavState& navState = vs.getNavState();

    vehicleLoc.x() = navState.m_tranAbsY; // easting
    vehicleLoc.y() = navState.m_tranAbsX; // northing

    
    if(!m_localizer.localize(vehicleLoc,
                             closestPt, 
                             distToClosestPt,
                             closestSegmentIdx,
                             ms_fixedLADist,
                             lookAheadPoint))
      {
        BOOST_LOG_SEV(m_logger, error) << "Could not localize the vehicle "
                                       << "to the path!!";
        return false;
      }
    
    return true;
  }
  
  //-----------------------------------------------------------------------------
  bool PathTracker::computeControls(const ddt::VehicleState& vs,
                                    double& desiredRadius,
                                    double& desiredSpeed)
  {
    
    // start off with a clean slate
    desiredRadius = 1.0E06;
    desiredSpeed = 0.0;

    // if the desired path is empty, there is a problem
    if(m_desiredPath.empty())
      {
        BOOST_LOG_SEV(m_logger, error) << "m_desiredPath is empty!!";
        return false;
      }

    // localize to the desired path
    nrec::geometry::Point2D_d closestPt;
    double distToClosestPt(0.0);
    std::size_t closestSegmentIdx(0);
    nrec::geometry::Point2D_d lookAheadPoint;

    nrec::geometry::Point2D_d vehicleLoc;
    const ddt::NavState& navState = vs.getNavState();

    vehicleLoc.x() = navState.m_tranAbsX; // northing
    vehicleLoc.y() = navState.m_tranAbsY; // easting

    const double currentYawRad = nrec::geometry::Angle_d::degToRad * 
      navState.m_tranAbsYaw.getDegrees();
    
    if(!m_localizer.localize(vehicleLoc,
                             closestPt, 
                             distToClosestPt,
                             closestSegmentIdx,
                             ms_fixedLADist,
                             lookAheadPoint))
      {
        BOOST_LOG_SEV(m_logger, error) << "Could not localize the vehicle "
                                       << "to the path!!";
        return false;
      }

    // if the path is too far off, bail
    if(distToClosestPt >= ms_maxDistanceThreshold)
      {
        std::cout.setf(std::ios_base::fixed);
        std::cout << "Vehicle loc: " << vehicleLoc
                  << " closestPt: " << closestPt
                  << "distance to closest point:  "
                  << distToClosestPt
                  << " is greater than the maximum "
                  << "distance threshold of: " 
                  << ms_maxDistanceThreshold
                  << std::endl;
        return false;
      }

    // get a reference to the desired path
    const std::vector<WPSegment>& path = m_localizer.getPath();

    // sanity on the proximity of the lookAheadPoint to the current location
    // if the vehicle is at the last segment
    if((closestSegmentIdx == path.size()-1) && 
       (path.back().progress(vehicleLoc) >= ms_progressForDone))
      {
        desiredRadius = 1.0E06;
        desiredSpeed = 0.0;
      }
    else // compute the the steering radius
      {
        // transform the look-ahead point to be body-relative
        // assuming  x -forward, y to the left and z up 
        // and the currentYawRad is using the same convention
        const double cos_yaw = cos(currentYawRad);
        const double sin_yaw = sin(currentYawRad);
        
        // swap easting and northing 
        const double x_d = lookAheadPoint.x() - vehicleLoc.x();
        const double y_d = lookAheadPoint.y() - vehicleLoc.y();
        
        const double x_la = cos_yaw * x_d + sin_yaw * y_d;
        const double y_la = -sin_yaw * x_d + cos_yaw * y_d;

        // std::cout.setf(std::ios_base::fixed);
        // std::cout.precision(5);
        // std::cout << "current yaw: " 
	// 	  << nrec::geometry::Angle_d::radToDeg * currentYawRad 
        //           << " P: (" << vehicleLoc.x()
        //           << ", " << vehicleLoc.y() << ") " 
        //           << " LA: (" << lookAheadPoint.x()
        //           <<  ", " << lookAheadPoint.y() << ")"
        //           << " x_d: " << x_d << ", y_d: "  << y_d 
        //           << " distance: " << sqrt(x_d*x_d + y_d*y_d)
        //           << " TLA: (" << x_la << ", " << y_la
        //           << ")" << std::endl;

        // sanity on the x_la
        if(0.0 == x_la)
          {
            desiredRadius = 1.0E06;
          }
        else // compute the steering radius
          {
            desiredRadius = ((x_la*x_la) + (y_la*y_la))/(2.0 * y_la);
          }

        // the desired speed -just the raw desired speed - no interpolation
        desiredSpeed = m_desiredPath[closestSegmentIdx].speed;
      }

    return true;
  }

  //-----------------------------------------------------------------------------
  void PathTracker::purgeOldSegments()
  {
    m_localizer.purgeOldSegments();
  }
  
}
