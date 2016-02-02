/**
 * @file   sinsmsg.h
 * @brief  Structure definitions for BASIL supported INSs.
 * @author M. George
 * @date   09/2012
 */
/**************************************************************************
 * Copyright (c) 2012, Carnegie Mellon University                         *
 *                                                                        *
 * DO NOT DISTRIBUTE WITHOUT PRIOR WRITTEN PERMISSION                     *
 *                                                                        *
 * THE MATERIAL EMBODIED IN THIS SOFTWARE IS PROVIDED TO YOU "AS-IS"      *
 * AND WITHOUT WARRANTY OF ANY KIND, EXPRESS, IMPLIED OR OTHERWISE,       *
 * INCLUDING WITHOUT LIMITATION, ANY WARRANTY OF MERCHANTABILITY OR       *
 * FITNESS FOR A PARTICULAR PURPOSE.  IN NO EVENT SHALL CARNEGIE MELLON   *
 * UNIVERSITY BE LIABLE TO YOU OR ANYONE ELSE FOR ANY DIRECT,             *
 * SPECIAL, INCIDENTAL, INDIRECT OR CONSEQUENTIAL DAMAGES OF ANY          *
 * KIND, OR ANY DAMAGES WHATSOEVER, INCLUDING WITHOUT LIMITATION,         *
 * LOSS OF PROFIT, LOSS OF USE, SAVINGS OR REVENUE, OR THE CLAIMS OF      *
 * THIRD PARTIES, WHETHER OR NOT CARNEGIE MELLON UNIVERSITY HAS BEEN      *
 * ADVISED OF THE POSSIBILITY OF SUCH LOSS, HOWEVER CAUSED AND ON         *
 * ANY THEORY OF LIABILITY, ARISING OUT OF OR IN CONNECTION WITH THE      *
 * POSSESSION, USE OR PERFORMANCE OF THIS SOFTWARE.                       *
 **************************************************************************/
#ifndef SINSMSG_H
#define SINSMSG_H

#include <cstring>

#include "basil/basil.h"

// serialization 
#include <boost/serialization/nvp.hpp>
#include <boost/serialization/serialization.hpp>

namespace BASIL
{
  namespace INS
  {
    // Current solution status, most subscribers will only want to use
    // data from while in the NAVIGATING mode.
    enum SolutionStatus
      {
        CALIBRATING,
        ALIGNING,
        NAVIGATING,
        INVALID,
        STOPPED,
        PAUSED,
        ALIGNMENT_FAILED
      };

    // 8 Bit field indicating which sensors were used in the solution.
    enum SolutionType
      {
        NONE                 = 0x00,
        INERTIAL             = 0x01,
        WHEEL_ENCODERS       = 0x02,
        VISUAL_ODOMETRY      = 0x04,
        GNSS                 = 0x08,
        LIDAR_ODOMETRY       = 0x10,
        VEHICLE_KINEMATICS   = 0x20,
        ZUPT                 = 0x40,
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
      BasilEndian<double> position[3];
      // Velocity (NED)
      BasilEndian<double> velocity[3];
      // Euler angles (zyx sequence from Nav. to IMU frame) or
      // Rotation vector to avoid singularities
      BasilEndian<double> orientation[3];

      // Serialization of this struct
      template<class Archive>
      void save(Archive& ar, uint32_t version) const
      {
        const double pos[3] = {position[0], position[1], position[2]};
        const double vel[3] = {velocity[0], velocity[1], velocity[2]};
        const double orient[3] = {orientation[0], orientation[1], orientation[2]};

        ar
          & BOOST_SERIALIZATION_NVP(status)
          & BOOST_SERIALIZATION_NVP(solutionType)
          & BOOST_SERIALIZATION_NVP(imuMdl)
          & BOOST_SERIALIZATION_NVP(pos)
          & BOOST_SERIALIZATION_NVP(vel)
          & BOOST_SERIALIZATION_NVP(orient);
      }

      template<class Archive>
      void load(Archive& ar, uint32_t version)
      {
        double pos[3], vel[3], orient[3];

        ar
          & BOOST_SERIALIZATION_NVP(status)
          & BOOST_SERIALIZATION_NVP(solutionType)
          & BOOST_SERIALIZATION_NVP(imuMdl)
          & BOOST_SERIALIZATION_NVP(pos)
          & BOOST_SERIALIZATION_NVP(vel)
          & BOOST_SERIALIZATION_NVP(orient);

        memcpy(&position, static_cast<const void*>(&pos), 3*sizeof(double)); 
        memcpy(&velocity, static_cast<const void*>(&vel), 3*sizeof(double));
        memcpy(&orientation, static_cast<const void*>(&orient), 3*sizeof(double));
      }

      BOOST_SERIALIZATION_SPLIT_MEMBER();

    } __attribute__((packed));

    struct InsMsg
    {
      // Common header
      BasilHeader header;
      // Payload
      Payload payload;
      // Basil CRC
      BasilCrc crc;

      // Serialization of this struct
      template<class Archive>
      void save(Archive& ar, uint32_t version) const
      {
        const uint16_t tcrc(crc);
        ar
          & BOOST_SERIALIZATION_NVP(header)
          & BOOST_SERIALIZATION_NVP(payload)
          & BOOST_SERIALIZATION_NVP(tcrc);
      }

      template<class Archive>
      void load(Archive& ar, uint32_t version)
      {
        uint16_t tcrc;
        ar
          & BOOST_SERIALIZATION_NVP(header)
          & BOOST_SERIALIZATION_NVP(payload)
          & BOOST_SERIALIZATION_NVP(tcrc);

          crc = tcrc;
      }

      BOOST_SERIALIZATION_SPLIT_MEMBER();

    } __attribute__((packed));

    // stand alone methods to serialize to string and de-serialize from string
    bool toString(const InsMsg& msg, std::string& data);
    bool fromString(const std::string& data, InsMsg& msg);

    // the ROS topic names that will be used in publishing this message
    static const std::string LOCAL_POSE_TOPIC("basil_local_pose");
    static const std::string GLOBAL_POSE_TOPIC("basil_global_pose");

  }
}

BOOST_CLASS_IMPLEMENTATION(BASIL::INS::Payload, boost::serialization::object_serializable);
BOOST_CLASS_TRACKING(BASIL::INS::Payload, boost::serialization::track_never);

BOOST_CLASS_IMPLEMENTATION(BASIL::INS::InsMsg, boost::serialization::object_serializable);
BOOST_CLASS_TRACKING(BASIL::INS::InsMsg, boost::serialization::track_never);

#endif //SINSMSG_H
