//===============================================================================
/*! 
  \file PoseDataGatherer.h

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
#ifndef POSE_DATA_GATHERER_H
#define POSE_DATA_GATHERER_H

#include <Utils/UDPConnection.h>

#include <boost/signals2.hpp>
#include <boost/thread.hpp>
#include <boost/thread/shared_mutex.hpp>

// include the header from the BASIL development API 
#include "sinsmsg.h"

namespace vmi
{

  //-----------------------------------------------------------------------------
  class PoseDataGatherer
  {
  public:
    
    // default constructor
    PoseDataGatherer();

    // the destructor
    ~PoseDataGatherer();

    // by default, returns the global pose data
    inline BASIL::INS::InsMsg getPoseData();

    // signal-slot mechanism for pushing the data out
    typedef boost::signals2::signal<void (BASIL::INS::InsMsg)> InsMsgSignal_T;
    typedef InsMsgSignal_T::slot_type InsMsgSlot_T;

    // connect to the InsMsgSlot
    inline boost::signals2::connection connectInsMsgSignal(InsMsgSlot_T& slot)
    {
      return m_insMsgSignal.connect(slot);
    }

  private:

    // the data acquisition thread
    void daqThread() throw (std::runtime_error);

    // the IO service object
    boost::shared_ptr<boost::asio::io_service> m_ios;

    // the connection to the pose datagrams
    ddt::UDPConnection m_poseConnection;

    // the thread 
    boost::shared_ptr<boost::thread> m_acqThread;

    // the InsMsg Signal
    InsMsgSignal_T m_insMsgSignal;

    // should the acquistion thread run
    bool m_shouldAcqThreadRun;

    // the mutual exclusion
    boost::mutex m_mutex;

    // the latest pose message (no distinction between the different flavors) 
    BASIL::INS::InsMsg m_poseMsg;
  };

  //-----------------------------------------------------------------------------
  BASIL::INS::InsMsg PoseDataGatherer::getPoseData()
  {
    boost::mutex::scoped_lock lock(m_mutex);
    return m_poseMsg;
  }

  
}

#endif // POSE_DATA_GATHERER_H
