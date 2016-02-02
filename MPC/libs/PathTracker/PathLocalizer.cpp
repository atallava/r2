//! Mode:c++

//===============================================================================
/*! 
  \file PathLocalizer.cpp

  \author Venkat Rajagopalan
  \date   03/08/2012
  
  \attention
  Copyright (C) 2011 \n
  National Robotics Engineering Center (NREC) \n
  Robotics Institute
  Carnegie Mellon University
  All Rights Reserved \n
*/
//===============================================================================
#include <cmath>
#include <sstream>
#include <boost/foreach.hpp>
#include <boost/math/special_functions/fpclassify.hpp>

#include "PathLocalizer.h"

//-------------------------------------------------------------------------------
using namespace boost::log::trivial;
//-------------------------------------------------------------------------------

namespace vmi
{
  //-------------------------------------------------------------------------------
  // WPSegment - default constructor
  //-------------------------------------------------------------------------------
  WPSegment::WPSegment():
    m_cumDistFromPathStart(0.0),
    m_length(0.0)
  {
  }
  
  //-------------------------------------------------------------------------------
  // WPSegment - destructor
  //-------------------------------------------------------------------------------
  WPSegment::~WPSegment()
  {
  }
  
  //-------------------------------------------------------------------------------
  // WPSegment - constructor with args
  //-------------------------------------------------------------------------------
  WPSegment::WPSegment(const nrec::geometry::Point2D_d& start,
                       const nrec::geometry::Point2D_d& end,
                       const double& cumDistFromPathStart,
                       const double speed):
    m_start(start),
    m_end(end),
    m_cumDistFromPathStart(cumDistFromPathStart),
    m_length(hypot(m_end.x() - m_start.x(), m_end.y() - m_start.y())),
    m_speed(speed)
  {
  }

  //-------------------------------------------------------------------------------
  double WPSegment::progress(const nrec::geometry::Point2D_d& pt) const
  {
    double x_d = 0.0, y_d = 0.0, x_s_p = 0.0, y_s_p = 0.0, alpha = 0.0;
    
    // the x component of the vector m_path[idx]
    x_d = m_end.x() - m_start.x();
    
    // the y component of the vector m_path[idx]
    y_d = m_end.y() - m_start.y();
    
    // the x component of the vector between pt and start
    x_s_p = pt.x() - m_start.x();
    
    // the y component of the vector between pt and start
    y_s_p = pt.y() - m_start.y();
    
    // the "normal" projection
    alpha = ((x_d * x_s_p) + (y_d * y_s_p))/((x_d*x_d) + (y_d*y_d));
    
    return alpha;
  }


  //-------------------------------------------------------------------------------
  // static member initialization for PathLocalizer class
  //-------------------------------------------------------------------------------
  const double PathLocalizer::ms_smallestSegmentLength(1.0E-02);
  const double PathLocalizer::ms_largestSegmentLength(200.0);
  const double PathLocalizer::ms_pathResolution(1.0);
  const std::size_t PathLocalizer::ms_maxPathElements(5000);

  //-------------------------------------------------------------------------------
  // PathLocalizer - default constructor
  //-------------------------------------------------------------------------------
  PathLocalizer::PathLocalizer():
    m_firstPointAdded(false),
    m_searchEntirePath(true),
    m_closestSegmentIdx(0),
    m_pathCumulativeDistance(0.0),
    m_resampleSegments(true)
  {
  }

  //-------------------------------------------------------------------------------
  // PathLocalizer - destructor
  //-------------------------------------------------------------------------------
  PathLocalizer::~PathLocalizer()
  {
  }

  //-------------------------------------------------------------------------------
  // PathLocalizer - reset
  //-------------------------------------------------------------------------------
  void PathLocalizer::reset()
  {
    m_firstPointAdded  = false;
    m_searchEntirePath = true;
    m_closestSegmentIdx = 0;
    m_pathCumulativeDistance = 0.0;
    m_path.clear();
  }

  //-------------------------------------------------------------------------------
  // PathLocalizer - addPoint method
  //-------------------------------------------------------------------------------
  bool PathLocalizer::addPoint(const nrec::geometry::Point2D_d& pt,
                               const double speed)
  {
    // sanity check on pt
    if((0 == boost::math::isfinite(pt.x()) || (0 == boost::math::isfinite(pt.y()))))
      {
        BOOST_LOG_SEV(m_logger, warning)
          << "One or both values of pt is not finite!!!";
        return false;
      }

    // if the current path is empty....
    if(m_path.empty())
      {
        // if m_firstPointAdded is false, update m_firstPoint and 
        // set m_firstPointAdded to true
        if(!m_firstPointAdded)
          {
            m_firstPoint = pt;
            m_firstPointAdded = true;
          }
        else // a waypoint segment can be constructed with the new point
          {
            WPSegment segment(m_firstPoint, pt, m_pathCumulativeDistance, speed);

            // the segment must be at least ms_smallestSegmentLength long
            // and atmost ms_largestSegmentLength long
            if((segment.m_length < ms_smallestSegmentLength) ||
               (segment.m_length > ms_largestSegmentLength))
              {
                // BOOST_LOG_SEV(m_logger, warning)
                //   << "Segment length: " << segment.m_length
                //   << " is not in the range ["
                //   << ms_smallestSegmentLength
                //   << ", "
                //   << ms_largestSegmentLength
                //   << "]";
                return false;
              }
            else // segment length is in a reasonable range
              {
                addSegmentToPath(segment);
              }
          }
      }
    else // m_path has atleast one entry
      {
        // erase elements from the front if the size exceeds the allowed size
        if(m_path.size() > ms_maxPathElements)
          {
            m_path.erase(m_path.begin(), 
                         m_path.begin() + m_path.size() - ms_maxPathElements + 1);

            // set m_searchEntirePath to true
            m_searchEntirePath = true;
          }

        WPSegment segment(m_path.back().m_end, pt, m_pathCumulativeDistance, speed);

        // the segment must be at least ms_smallestSegmentLength long
        // and atmost ms_largestSegmentLength long
        if((segment.m_length < ms_smallestSegmentLength) ||
           (segment.m_length > ms_largestSegmentLength))
          {
            // BOOST_LOG_SEV(m_logger, warning)
            //   << "Segment length: " << segment.m_length
            //   << " is not in the range ["
            //   << ms_smallestSegmentLength
            //   << ", "
            //   << ms_largestSegmentLength
            //   << "]";
            return false;
          }
        else // segment length is in a reasonable range
          {
            addSegmentToPath(segment);
          }
      }

    return true;
  }

  //-----------------------------------------------------------------------------
  // PathLocalizer - localize method
  //-----------------------------------------------------------------------------
  bool PathLocalizer::localize(const nrec::geometry::Point2D_d& pt,
                               nrec::geometry::Point2D_d& closestPt,
                               double& closestPtDist,
                               std::size_t& closestSegmentIdx,
                               const double& lookAheadDistance,
                               nrec::geometry::Point2D_d& lookAheadPoint,
                               const bool searchEntirePath,
                               const double searchWindowSize)
  {

    // sanity check on pt
    if((0 == boost::math::isfinite(pt.x()) || (0 == boost::math::isfinite(pt.y()))))
      {
        BOOST_LOG_SEV(m_logger, warning)
          << "One or both values of pt is not finite!!!";
        return false;
      }
    
    // if m_path is empty,
    if(m_path.empty())
      {
        // the closest point is the same as the point coming in
        closestPt = pt;
        
        // the distance to the closest point is 0.0
        closestPtDist = 0.0;

        // default closestSegmentIdx
        closestSegmentIdx = 0;

        BOOST_LOG_SEV(m_logger, error) << "m_path is empty!";

        // this is a failure case
        return false;
      }
    
    // if requested to search the entire path, set the range appropriately
    std::size_t startIdx = 0;
    std::size_t endIdx = m_path.size() - 1;
    if(m_searchEntirePath || searchEntirePath)
      {
        // if necessary, set m_searchPath to false
        if(m_searchEntirePath)
          {
            m_searchEntirePath = false;
          }
      }
    else // search within a searchWindowSize starting from m_closestSegmentIdx
      {
        // compute the index range (from m_closestSegmentIdx) 
        startIdx = m_closestSegmentIdx;
        endIdx = startIdx;
        double cumDist = m_path[startIdx].m_length;
        for(std::size_t idx = startIdx + 1; idx < m_path.size(); idx++)
          {
            cumDist += m_path[idx].m_length;
            if(cumDist <= searchWindowSize)
              {
                endIdx = idx;
              }
            else
              {
                break;
              }
          }
      }
        
    // find the closest segment
    findClosestSegment(pt, startIdx, endIdx, closestPt, closestPtDist);

    // update closestSegmentIdx
    closestSegmentIdx = m_closestSegmentIdx;

    // find the look-ahead point in the path
    findLookAheadPoint(lookAheadDistance, pt, closestPt, lookAheadPoint);

    return true;
  }

  //-----------------------------------------------------------------------------
  // PathLocalizer - purgeOldSegments
  //-----------------------------------------------------------------------------
  void PathLocalizer::purgeOldSegments(const double pathLengthLimit)
  {
    // if the current cumulative path length is larger than the limit 
    // and if the first entry is not the closest segment
    if((m_pathCumulativeDistance > pathLengthLimit) && (0 < m_closestSegmentIdx))
      {
        // the iterator that points to the last segment that will be purged
        std::vector<WPSegment>::iterator iter = 
          m_path.begin() + m_closestSegmentIdx - 1;

        // the offset for the m_cumDistFromPathStart for the entries that 
        // will not be purged
        const double offset = 
          iter->m_cumDistFromPathStart + iter->m_length;
        
        // update m_pathCumulativeDistance
        m_pathCumulativeDistance -= offset;

        // erase the elements in the range m_path.begin() and iter
        iter = m_path.erase(m_path.begin(), iter);

        // adjust the m_cumDistFromPathStart
        for(; iter != m_path.end(); ++iter)
          {
            iter->m_cumDistFromPathStart -= offset;
          }

        // set m_closestSegmentIdx to 0
        m_closestSegmentIdx = 0;
      }
  }

  //-----------------------------------------------------------------------------
  //-----------------------------------------------------------------------------
  void PathLocalizer::disableSegmentResampling()
  {
    m_resampleSegments = false;
  }

  //-----------------------------------------------------------------------------
  // PathLocalizer - addSegmentToPath method
  //-----------------------------------------------------------------------------
  void PathLocalizer::addSegmentToPath(const WPSegment& segment)
  {
    // if requested to no resample, simply add 
    if(!m_resampleSegments)
      {
        // add segment to the back of m_path
        m_path.push_back(segment);

        // update the cumulative distance 
        m_pathCumulativeDistance += segment.m_length;
        
        return;
      }

    // if segment.m_length <= ms_pathResolution, just add the segment
    if(segment.m_length <= ms_pathResolution)
      {
        // add segment to the back of m_path
        m_path.push_back(segment);

        // update the cumulative distance 
        m_pathCumulativeDistance += segment.m_length;
      }
    else // segment.m_length is > ms_pathResolution
      {
        // split segment into segments of length ms_pathResolution
        // and add them to m_path
        const std::size_t numSegments = 
          static_cast<std::size_t>(segment.m_length/ms_pathResolution);
        const double x_d = 
          (segment.m_end.x() - segment.m_start.x())/(segment.m_length);
        const double y_d = 
          (segment.m_end.y() - segment.m_start.y())/(segment.m_length);

        const double speed = segment.m_speed;

        std::vector<WPSegment> subSegments;
        WPSegment currSegment;
        for(std::size_t segIdx = 1; segIdx <= numSegments; segIdx++)
          {
            // set the appropriate start point for the currSegment
            if(subSegments.empty())
              {
                currSegment.m_start = segment.m_start;
              }
            else
              {
                currSegment.m_start = subSegments.back().m_end;
              }

            // compute the x coordinate of the end point
            currSegment.m_end.x() = 
              segment.m_start.x() + (segIdx * ms_pathResolution * x_d);

            // compute the y coordinate of the end point
            currSegment.m_end.y() = 
              segment.m_start.y() + (segIdx * ms_pathResolution * y_d);
            
            // compute the segment length
            currSegment.m_length = 
              hypot(currSegment.m_end.x() - currSegment.m_start.x(),
                    currSegment.m_end.y() - currSegment.m_start.y());

            // update the speed
            currSegment.m_speed = speed;

            // save currSegment in subSegments
            subSegments.push_back(currSegment);
            
          }

        // special case for the end point
        const double lastSegmentDist = 
          hypot(segment.m_end.x() - subSegments.back().m_end.x(),
                segment.m_end.y() - subSegments.back().m_end.y());

        // update the end point of subSegments.back() or add a separate segment
        if(lastSegmentDist <= 0.5 * ms_pathResolution)
          {
            subSegments[subSegments.size()-1].m_end.x() = segment.m_end.x();
            subSegments[subSegments.size()-1].m_end.y() = segment.m_end.y();
            
            // update the length
            subSegments[subSegments.size()-1].m_length += lastSegmentDist;
          }
        else
          {
            currSegment.m_start = subSegments.back().m_end;
            currSegment.m_end = segment.m_end;
            currSegment.m_length = lastSegmentDist;

            // update the speed
            currSegment.m_speed = speed;

            // save currSegment in subSegments
            subSegments.push_back(currSegment);
          }

        // // log some info
        // std::stringstream ss;
        // ss.setf(std::ios_base::fixed);
        // ss.precision(4);
        // ss << std::endl;
        // ss << "Original Segment: [" << segment.m_start << "-->" 
        //    << segment.m_end << "], L: "  << segment.m_length 
        //    << " CD: " << segment.m_cumDistFromPathStart
        //    << std::endl;

        // ss << "Original Segment split up into: " << subSegments.size()
        //    << " segments" << std::endl;

        // std::size_t count = 1;

        // finally, add all the entries from subSegments to m_path
        BOOST_FOREACH(WPSegment& aSegment, subSegments)
          {
            // update the distance from path start for the current segment
            aSegment.m_cumDistFromPathStart = m_pathCumulativeDistance;

            // add aSegment in m_path
            m_path.push_back(aSegment);

            // update m_pathCumulativeDistance by aSegment.m_length
            m_pathCumulativeDistance += aSegment.m_length;

            // // log 
            // ss << count++ << ": [" << aSegment.m_start << "-->"
            //    << aSegment.m_end << "] L: " << aSegment.m_length
            //    << " CD: " << aSegment.m_cumDistFromPathStart
            //    << std::endl;
          }
        
        // BOOST_LOG_SEV(m_logger, info) << ss.str();
      }

    // // log all the segments in m_path so far
    // std::stringstream ss;
    // ss.setf(std::ios_base::fixed);
    // ss.precision(4);
    // ss << std::endl;
    // ss << "m_path contains: " << m_path.size() << " segments" << std::endl;
    // std::size_t count = 1;
    // BOOST_FOREACH(const WPSegment& seg, m_path)
    //   {
    //     ss << count++ << ": [" << seg.m_start << "-->" << seg.m_end << "] "
    //        << "L " << seg.m_length << " CD: " << seg.m_cumDistFromPathStart
    //        << std::endl;
    //   }

    // BOOST_LOG_SEV(m_logger, info) << ss.str();
  }

  //-----------------------------------------------------------------------------
  // PathLocalizer - findClosestSegment
  //-----------------------------------------------------------------------------
  void PathLocalizer::findClosestSegment(const nrec::geometry::Point2D_d& pt,
                                         const std::size_t& startIdx,
                                         const std::size_t& endIdx,
                                         nrec::geometry::Point2D_d& closestPt,
                                         double& closestPtDist)
  {

    // the container that will hold the results of the localization
    std::vector<LocalizationResults> results;
    results.reserve(endIdx - startIdx + 1);

    // the current localization result
    LocalizationResults lr;

    // some temporary storage 
    double x_d = 0.0, y_d = 0.0, x_s_p = 0.0, y_s_p = 0.0, alpha = 0.0;

    // for elements in m_path with indices in range [startIdx, endIdx]
    for(std::size_t idx = startIdx; idx <= endIdx; idx++)
      {
        // the x component of the vector m_path[idx]
        x_d = m_path[idx].m_end.x() - m_path[idx].m_start.x();

        // the y component of the vector m_path[idx]
        y_d = m_path[idx].m_end.y() - m_path[idx].m_start.y();
        
        // the x component of the vector between pt and m_path[idx].start
        x_s_p = pt.x() - m_path[idx].m_start.x();

        // the y component of the vector between pt and m_path[idx].start
        y_s_p = pt.y() - m_path[idx].m_start.y();

        // the "normal" projection
        alpha = ((x_d * x_s_p) + (y_d * y_s_p))/((x_d*x_d) + (y_d*y_d));
        
        // update lr.m_segmentIdx with the current value of idx
        lr.m_segmentIdx = idx;

        // if alpha is negative, the closest point is m_path[idx].m_start
        if(0.0 > alpha)
          {
            // update lr.m_closestPt
            lr.m_closestPt = m_path[idx].m_start;
          }
        else if(1.0 < alpha) // the closest point is m_path[idx].m_end
          {
            // update lr.m_closestPt
            lr.m_closestPt = m_path[idx].m_end;
          }
        else // the closest point is along the line segment m_path[idx]
          {
            lr.m_closestPt.x() = m_path[idx].m_start.x() + (alpha * x_d);
            lr.m_closestPt.y() = m_path[idx].m_start.y() + (alpha * y_d);
          }
        
        // compute the distance
        lr.m_distToSegment = hypot(lr.m_closestPt.x() - pt.x(),
                                   lr.m_closestPt.y() - pt.y());
        
        // save lr
        results.push_back(lr);
      }

    // sort results
    std::stable_sort(results.begin(), results.end(), LocalizationResults_gt());

    // there is NO check done to see if the path crosses over.
    closestPt = results.front().m_closestPt;
    closestPtDist = results.front().m_distToSegment;

    // update m_closestSegmentIdx
    m_closestSegmentIdx = results.front().m_segmentIdx;

    // std::stringstream ss;
    // ss.setf(std::ios_base::fixed);
    // ss.precision(4);
    // ss << "Path has " << m_path.size() << " points.  Closest Segment Idx: "
    //    << m_closestSegmentIdx
    //    << " dist: " << closestPtDist;

    // BOOST_LOG_SEV(m_logger, info) << ss.str();
      
  }

  //-----------------------------------------------------------------------------
  void PathLocalizer::findLookAheadPoint(const double& lookAheadDistance,
					 const nrec::geometry::Point2D_d& pt,
                                         const nrec::geometry::Point2D_d& closestPt,
                                         nrec::geometry::Point2D_d& lookAheadPoint)
  {
    double cumulativeDist(0.0);
    double dist(0.0);
    bool foundLAPoint(false);

    std::cout.setf(std::ios_base::fixed);
    std::cout.precision(3);
                
    // accumulate the distance to the end from the current segment
    for(std::size_t idx = m_closestSegmentIdx; idx < m_path.size(); idx++)
      {
        if(m_closestSegmentIdx == idx)
          {
            dist = hypot(m_path[idx].m_end.x() - closestPt.x(),
                         m_path[idx].m_end.y() - closestPt.y());

            // exact case
            if(dist == lookAheadDistance)
              {
                lookAheadPoint = m_path[idx].m_end;
                foundLAPoint = true;
                break;
              }
            else if(dist > lookAheadDistance)
              {
                // build the vector
                const double x_d = m_path[idx].m_end.x() - closestPt.x();
                const double y_d = m_path[idx].m_end.y() - closestPt.y();

                // the total path length
                const double pathLength = hypot(x_d, y_d);

                // the scale
                const double scale = lookAheadDistance/pathLength;
                
                lookAheadPoint.x() = closestPt.x() + scale * x_d;
                lookAheadPoint.y() = closestPt.y() + scale * y_d;

                // double x_diff = lookAheadPoint.x() - closestPt.x();
                // double y_diff = lookAheadPoint.y() - closestPt.y();

                // std::cout  << "ID: " << idx
                //            << " CD: " << cumulativeDist
                //            << " LAD: " << lookAheadDistance
                //            // << " L: " << m_path[idx].m_length
                //            // << " S: " << scale
		// 	   << " P: [" << pt.x() << ", " << pt.y() 
                //            << "] CP: [" << closestPt.x() << ", " 
                //            << closestPt.y() << "], LA: ["
                //            << lookAheadPoint.x() << ", " 
                //            << lookAheadPoint.y() << "]"
                //            << " DCP: " << hypot(x_diff, y_diff)
                //            << std::endl;
                
                foundLAPoint = true;
                break;
              }

          }
        else // not the first segment, 
          {
            dist = m_path[idx].m_length;
          }
        
        // if the current segment's length plus the cumulative distance
        // accrued so-far is smaller than the lookAheadDistance, 
        // update the cumulativeDistance and continue along
        if(cumulativeDist + dist < lookAheadDistance)
          {
            cumulativeDist += dist;
          }
        else if(cumulativeDist + dist == lookAheadDistance) // exact match
          {
            lookAheadPoint.x() = m_path[idx].m_end.x();
            lookAheadPoint.y() = m_path[idx].m_end.y();

            // we found the look ahead point
            foundLAPoint = true;
            break;
          }
        else // found the segment within which the lookahead point lies
          {
            // the distance remaining is the difference between
            // the lookAheadDistance and the cumulativeDist clllected so far
            const double distToGo = lookAheadDistance - cumulativeDist;
            
            // the scale is then just the ratio of distToGo to the segment length
            const double scale = distToGo/m_path[idx].m_length;

            // build the vector
            const double x_d = m_path[idx].m_end.x() - m_path[idx].m_start.x();
            const double y_d = m_path[idx].m_end.y() - m_path[idx].m_start.y();

            // update the look-ahead point
            lookAheadPoint.x() = m_path[idx].m_start.x() + scale * x_d;
            lookAheadPoint.y() = m_path[idx].m_start.y() + scale * y_d;

            // double x_diff = lookAheadPoint.x() - closestPt.x();
            // double y_diff = lookAheadPoint.y() - closestPt.y();

            // std::cout  << "ID: " << idx
            //            << " CD: " << cumulativeDist
            //            << " LAD: " << lookAheadDistance
            //            // << " L: " << m_path[idx].m_length
            //            // << " S: " << scale
	    // 	       << " P: [" << pt.x() << ", " << pt.y() 
	    // 	       << "] CP: [" << closestPt.x() << ", " 
            //            << closestPt.y() << "], LA: ["
            //            << lookAheadPoint.x() << ", " 
            //            << lookAheadPoint.y() << "]"
            //           << " DCP: " << hypot(x_diff, y_diff)
            //           << std::endl;
              
            // the look-ahead point has been found
            foundLAPoint = true;

            // we are done
            break;

          }
      }

    // return the last point as the look-ahead point
    if(!foundLAPoint)
      {
        lookAheadPoint = m_path.back().m_end;
      }

    return;
  }

}
