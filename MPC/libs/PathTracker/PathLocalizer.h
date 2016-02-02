//! Mode:c++

//===============================================================================
/*! 
  \file PathLocalizer.h

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

#ifndef Path_Localizer_h
#define Path_Localizer_h

//-------------------------------------------------------------------------------
// INCLUDES
//-------------------------------------------------------------------------------
#include <vector>
#include <algorithm>

#include <boost/noncopyable.hpp>
#include <boost/log/trivial.hpp>
#include <boost/log/sources/severity_logger.hpp>

#include <geometry/primitives/Point2D.h>

//-------------------------------------------------------------------------------
namespace vmi
{

  //-------------------------------------------------------------------------------
  class WPSegment
  {
  public:
    
    //-----------------------------------------------------------------------------
    //! \name Constructors and destructors
    //-----------------------------------------------------------------------------
    //@{
    //! 
    // default constructor
    WPSegment();
    
    // destructor
    ~WPSegment();
    
    // constructor with start, end and the cumulative distance of this segment's
    // start location from the beginning of the path
    WPSegment(const nrec::geometry::Point2D_d& start,
              const nrec::geometry::Point2D_d& end,
              const double& cumDistFromPathStart,
              const double speed);

    // "progress" along the path segment
    double progress(const nrec::geometry::Point2D_d& pt) const;

    
    //@}
    
    // the start location of the segment
    nrec::geometry::Point2D_d m_start;
    
    // the end location of the segment
    nrec::geometry::Point2D_d m_end;
    
    // the cumulative distance from the start of the path
    double m_cumDistFromPathStart;
    
    // length of the segment
    double m_length;

    // the speed along the segment
    double m_speed;
  };

  //-------------------------------------------------------------------------------
  // a results container
  struct LocalizationResults
  {
    // the segment index
    std::size_t m_segmentIdx;
    
    // the distance to the segment
    double m_distToSegment;
    
    // the closest point
    nrec::geometry::Point2D_d m_closestPt;
  };

  //-------------------------------------------------------------------------------
  // a functor for sorting LocalizationResults
  struct LocalizationResults_gt : 
    public std::binary_function<LocalizationResults, LocalizationResults, bool>
  {
  public:
    inline bool operator()(const LocalizationResults& lra, 
                           const LocalizationResults& lrb) const
    {
      return (lra.m_distToSegment <= lrb.m_distToSegment);
    }
  };

  //-------------------------------------------------------------------------------
  class PathLocalizer 
  {
  public:
    //-----------------------------------------------------------------------------
    //! \name Constructors and destructors
    //-----------------------------------------------------------------------------
    //@{
    //! 
    // default constructor
    PathLocalizer();

    // destructor
    ~PathLocalizer();
    //@}

    //-----------------------------------------------------------------------------
    //! reset - put the object back to a sane state
    void reset();

    //-----------------------------------------------------------------------------
    //! get a const reference to the member m_path
    inline const std::vector<WPSegment>& getPath() const;

    //-----------------------------------------------------------------------------
    //! get a const reference to the cumulative distance of the path
    inline const double& getTotalPathLength() const;

    //-----------------------------------------------------------------------------
    //! get the minimum distance betwen (re-sampled) points
    inline static const double& getSamplingResolution();

    //-----------------------------------------------------------------------------
    //! get the minimum distance betwen points
    inline static const double& getMinSegmentLength();

    //-----------------------------------------------------------------------------
    //! get the maximum distance betwen points
    inline static const double& getMaxSegmentLength();

    //-----------------------------------------------------------------------------
    //! add a point to the "end" of the path
    bool addPoint(const nrec::geometry::Point2D_d& pt,
                  const double speed);

    //-----------------------------------------------------------------------------
    //! given a point, find the closest point on the path and the distance to the
    //! closest point on the path
    bool localize(const nrec::geometry::Point2D_d& pt,
                  nrec::geometry::Point2D_d& closestPt,
                  double& closestPtDist,
                  std::size_t& closestSegmentIdx,
                  const double& lookAheadDistance,
                  nrec::geometry::Point2D_d& lookAheadPoint,
                  const bool searchEntirePath = false,
                  const double searchWindowSize = 10.0 // meters
                  );

    //-----------------------------------------------------------------------------
    //! purge all the segments in the m_path container whose index value is 
    //! less than m_closestSegmentIdx
    void purgeOldSegments(const double pathLengthLimit = 50.0);

    //-----------------------------------------------------------------------------
    //! donot modify the segments
    void disableSegmentResampling();
                           

  private:

    // add the segment to the path
    void addSegmentToPath(const WPSegment& segment);

    // find the closest segment
    void findClosestSegment(const nrec::geometry::Point2D_d& pt,
                            const std::size_t& startIdx,
                            const std::size_t& endIdx,
                            nrec::geometry::Point2D_d& closestPt,
                            double& closestPtDist);

    // find the look-ahead point
    void findLookAheadPoint(const double& lookAheadDistance,
			    const nrec::geometry::Point2D_d& loc,
                            const nrec::geometry::Point2D_d& closestPt,
                            nrec::geometry::Point2D_d& lookAheadPoint);

    // the collection of segments that makeup the desired path
    std::vector<WPSegment> m_path;

    // the first point
    nrec::geometry::Point2D_d m_firstPoint;

    // has the first point been added
    bool m_firstPointAdded;

    // should a "global" localization operation be performed?
    bool m_searchEntirePath;

    // the index of the closest segment
    std::size_t m_closestSegmentIdx;

    // the cumulative distance of the path collected so far
    double m_pathCumulativeDistance;

    // should the incoming segments be resampled
    bool m_resampleSegments;

    // the logger object for this class
    boost::log::sources::severity_logger<> m_logger;

    // smallest segment length that is acceptable
    static const double ms_smallestSegmentLength;

    // largest segment length that is acceptable
    static const double ms_largestSegmentLength;

    // the distance in meters between successive path points (internally)
    static const double ms_pathResolution;

    // the maximum number of elements in m_path
    static const std::size_t ms_maxPathElements;
    
  };

  //-----------------------------------------------------------------------------
  //! INLINE methods
  //-----------------------------------------------------------------------------

  //-----------------------------------------------------------------------------
  const std::vector<WPSegment>& PathLocalizer::getPath() const
  {
    return m_path;
  }

  //-----------------------------------------------------------------------------
  const double& PathLocalizer::getTotalPathLength() const
  {
    return m_pathCumulativeDistance;
  }

  //-----------------------------------------------------------------------------
  const double& PathLocalizer::getSamplingResolution()
  {
    return ms_pathResolution;
  }

  //-----------------------------------------------------------------------------
  const double& PathLocalizer::getMinSegmentLength()
  {
    return ms_smallestSegmentLength;
  }

  //-----------------------------------------------------------------------------
  const double& PathLocalizer::getMaxSegmentLength()
  {
    return ms_largestSegmentLength;
  }

}

#endif //< Include guard
