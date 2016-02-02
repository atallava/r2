//===============================================================================
/*! 
  \file SAGEPacketHeader.h

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
#ifndef SAGE_PACKET_HEADER_H
#define SAGE_PACKET_HEADER_H

#include <boost/date_time/posix_time/posix_time.hpp>


namespace vmi
{
  //-----------------------------------------------------------------------------
  // the total number of seconds in the GPS week
  //-----------------------------------------------------------------------------
  static const uint32_t SecondsInGPSWeek(60*60*24*7);

  //-----------------------------------------------------------------------------
  // zero time duration
  //-----------------------------------------------------------------------------
  static const boost::posix_time::time_duration ZeroTd;

  //-----------------------------------------------------------------------------
  // the start of the Unix Epoch  [1/1/1970 00:00:00]
  //-----------------------------------------------------------------------------
  static const boost::gregorian::date UnixEpoch(1970,1,1);

  //-----------------------------------------------------------------------------
  // the UTC time for Unix Epoch
  //-----------------------------------------------------------------------------
  static const boost::posix_time::ptime UnixUTCTime(UnixEpoch, ZeroTd);

  //-----------------------------------------------------------------------------
  // the start of the GPS Epoch  [1/6/1980 00:00:00]
  //-----------------------------------------------------------------------------
  static const boost::gregorian::date GPSEpoch(1980,1,6);

  //-----------------------------------------------------------------------------
  // the UTC time for GPS Epoch
  //-----------------------------------------------------------------------------
  static const boost::posix_time::ptime GPSUTCTime(GPSEpoch, ZeroTd);


  //-----------------------------------------------------------------------------
  // the offset in seconds between GPS and Unix Epochs in UTC Time
  //-----------------------------------------------------------------------------
  static const double GPSToUnixSeconds((GPSUTCTime-UnixUTCTime).total_seconds());

  //-----------------------------------------------------------------------------
  // the SAGE Packet header 
  //-----------------------------------------------------------------------------
  struct SAGEPacketHeader_T
  {
    // sync bytes 
    uint8_t sync[3];

    // header size
    uint8_t headerSize;

    // hardware ID
    uint8_t hardwareId;

    // Message type
    uint8_t messageType;

    // sensor model - also reused for message version
    uint8_t sensorModel;

    // device id
    uint8_t deviceID;

    // the size of the payload
    uint16_t payloadSize;

    // reserved
    uint8_t reserved;

    // time status
    uint8_t timeStatus;

    // gps week
    uint16_t gpsWeek;

    // seconds
    uint32_t seconds;

    // nano seconds
    uint32_t nanoSeconds;

    // extract the timestamp in seconds
    inline double getTimeStampSeconds() const
    {
      double timeStamp = 
        (gpsWeek * SecondsInGPSWeek) + seconds + (nanoSeconds / 1.0E09);
      return timeStamp;
    }

    // extract the timestamp in seconds in UTC
    inline double getTimeStampSecondsUTC() const
    {
      return GPSToUnixSeconds + getTimeStampSeconds();
    }

  } __attribute((packed));

  // the reset packet
  struct PoseResetPacket_T 
  {
    // the default constructor
    PoseResetPacket_T()
    {
      // update the sync bytes
      hdr.sync[0] = 'C';
      hdr.sync[1] = 'M';
      hdr.sync[2] = 'U';

      // update the size
      hdr.headerSize = sizeof(SAGEPacketHeader_T);

      // the message type 
      hdr.messageType = 254;

      // the message version (2 for reset)
      hdr.sensorModel = 2;

      // no payload
      hdr.payloadSize = 0;

      // no crc
      crc[0] = crc[1] = crc[2] = crc[3] = 0x00;
    }

    // the header
    SAGEPacketHeader_T hdr;

    // the CRC 
    uint8_t crc[4];

  }__attribute__((packed));


  // Current solution status, most subscribers will only want to use
  // data from while in the NAVIGATING mode.
  enum SolutionStatus
    {
      CALIBRATING,
      ALIGNING,
      NAVIGATING,
      INVALID,
      STOPPED,
      PAUSED
    };

  // 8 Bit field indicating which sensors were used in the solution.
  enum SolutionType
    {
      NONE                 = 0x00,
      INERTIAL             = 0x01,
      WHEEL_ENCODERS       = 0x02,
      VISUAL_ODOMETRY      = 0x04,
      GPS                  = 0x08,
      RANGE_ODOMETRY       = 0x10,
      VEHICLE_KINEMATICS   = 0x20,
      OTHER                = 0x80
    };
  
  struct Payload
  {
    // INS Mode
    uint8_t status;

    // INS status
    uint8_t solutionType;

    // Imu Model
    uint8_t imuMdl;

    // Position (NED)
    double position[3];

    // Velocity (NED)
    double velocity[3];

    // Euler angles (zyx sequence from Nav. to IMU frame) or
    // Rotation vector to avoid singularities
    double orientation[3];

  } __attribute__((packed));


  struct InsMsg
  {
    // Common header
    SAGEPacketHeader_T header;
  
    // Payload
    Payload payload;
    
    // Basil CRC
    uint8_t crc[4];
    
  } __attribute__((packed));
  
  

}
#endif
