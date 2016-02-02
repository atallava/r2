//===============================================================================
/*! 
  \file PoseDataGatherer.cpp

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

#include <sstream>
#include <cstdio>

#include <boost/bind.hpp>

#include "PoseDataGatherer.h"

namespace vmi
{

  //-----------------------------------------------------------------------------
  PoseDataGatherer::PoseDataGatherer():
    m_ios(new boost::asio::io_service()),
    m_poseConnection(m_ios),
    m_shouldAcqThreadRun(true)
  {
    const std::string hostName("192.168.101.255");
    const uint16_t portNumber(1533);
    if(!m_poseConnection.connect(hostName, portNumber, true))
      {
        std::stringstream ss;
        ss << "Error connecting to host: "
           << hostName
           << " on port: " << portNumber << std::endl;

        throw std::runtime_error(ss.str().c_str());
      }

    // the acquisition thread
    m_acqThread.reset(new boost::thread(boost::bind(&PoseDataGatherer::daqThread,
                                                    this)));
  }

  //-----------------------------------------------------------------------------
  PoseDataGatherer::~PoseDataGatherer()
  {
    // todo - shutdown the acquisition thread
    std::cout << "requesting to shutdown the daq thread...." << std::endl;
    
    // begin critical section
    {
      boost::mutex::scoped_lock lock(m_mutex);

      while(!m_ios->stopped())
	{
	  m_ios->stop();
	  boost::this_thread::sleep(boost::posix_time::seconds(1));
	}

      m_shouldAcqThreadRun = false;
    }
    // end critical section

    // wait for the thread to finish
    m_acqThread->join();

  }

  //-----------------------------------------------------------------------------
  void PoseDataGatherer::daqThread() throw (std::runtime_error)
  {
    // the instance of the pose message that is received
    const bool considerSizeMismatchAsError(false);
    std::size_t bytesRead(0);
    bool rv(false);

    // loop almost for ever
    while(true)
      {
	// this blocking read should keep this thread from hogging CPU
	BASIL::INS::InsMsg poseMsg;
        rv = m_poseConnection.read(&poseMsg, 
				   sizeof(BASIL::INS::InsMsg),
				   considerSizeMismatchAsError);

	// enter the critical section
	{
	  boost::mutex::scoped_lock lock(m_mutex);
          
	  // if requested to shutdown, do so now
	  if(!m_shouldAcqThreadRun)
	    {
	      std::cout << "Requested to shut down...." << std::endl;
	      break;
	    }
	  
	  // if this was actually a BASIL::INS::InsMsg
	  if(rv)
	    {
	      // update the pose message
	      m_poseMsg = poseMsg;

	      // signal
	      m_insMsgSignal(m_poseMsg);

	    }
	}
	// leave the critical section
	
	//boost::this_thread::sleep(boost::posix_time::milliseconds(1));
            
      }
  }

}
