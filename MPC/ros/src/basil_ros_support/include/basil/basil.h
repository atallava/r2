/**
 * @file   basil.h
 * @brief  Common definitions and enumerations within BASIL namespace
 * @author M. George, M. Laverne, D. Bennington
 * @date   01/2013
 */
/**************************************************************************
 * Copyright (c) 2011-2013, Carnegie Mellon University                    *
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
#ifndef BASIL_H
#define BASIL_H

// Could switch this to cstdint with C++11
#include <stdint.h>
#include <cmath>
#include <boost/detail/endian.hpp>

// serialization 
#include <boost/serialization/nvp.hpp>
#include <boost/serialization/serialization.hpp>
#include <boost/serialization/split_member.hpp>

// BASIL namespace to be replicated across software projects and devices
// to ensure interoperability.  All enums are limited to 0-255 range to
// ensure they can be serialized to uint8_t values unless otherwise noted.
namespace BASIL
{
  // Endian helper class for simple data types.  On little endian systems
  // setting/getting the member data will result in its return with no
  // operations.  On big endian systems, a byte-swap will be performed on
  // either set or get operations.  This is useful for big endian systems
  // accessing little endian data buffers or big endian endian systems
  // writing little endian data buffers.
  // Warning: Setting and getting will result in no change to the data so
  // in most cases you only want to do one.  value can be set by memcpy
  // without invoking the constructor or a memory buffer can be accessed
  // via a BasilEndian pointer type for example.
  template <typename T>
  class BasilEndian
  {
  public:
    // Default constructor does nothing
    BasilEndian(){}
    // Conversion constructor swaps endian-ness on input
    BasilEndian(const T& t): value(swap_endian(t)){}
    // Conversion operator swaps endian-ness on output
    operator T() const {return swap_endian(value);}
  private:
    // The actual data being stored here
    T value;
    // Helper function to swap the bytes in value
    static T swap_endian(const T& in)
    {
#ifdef BOOST_BIG_ENDIAN
      T out;

      char* dst = reinterpret_cast<char*>(&out);
      const char* src = reinterpret_cast<const char*>(&in + 1);

      for (unsigned int i = 0; i < sizeof(T); i++)
        *dst++ = *--src;

      return out;
#endif
      return in;
    }

    // enable access to class members
    friend class boost::serialization::access;

    // serialization of this struct 
    template<class Archive>
    void serialize(Archive& ar, const uint32_t version)
    {
      ar &
        BOOST_SERIALIZATION_NVP(value);
    }

  }__attribute__((packed));

  // Basil message categories are the basic identifier for subscribing or
  // publishing data.  Most represent one class of sensor but some
  // are more or less generic as needed.
  namespace MESSAGE
  {
    enum MessageType
      {   
        IMU,                	 // 0   - Inertial measurement unit
        GNSS,                	 // 1   - Global navigation satellite sensors
        INS,                	 // 2   - Inertial navigation system
        ENCODER,            	 // 3   - Rotational encoder
        VISUAL_ODOMETRY,       	 // 4   - Visual odometry
        MAGNETOMETER,       	 // 5   - Magnetometer
        BAROMETER,          	 // 6   - Barometer
        PLACE_RECOGNITION,  	 // 7   - Place recognition
        IMU_CALIBRATION,    	 // 8   - IMU calibration
        FILTER_STATISTICS,       // 9   - Sensor fusion filter statistics
        VEHICLE_STATUS,     	 // 10  - Ground vehicle parameters, e.g. throttle, steer angle
        LIDAR,                   // 11  - LIDAR
        CAMERA,                  // 12  - Camera images
        KINEMATICS,			 	 // 13  - Vehicle kinematic data
        PPS,                     // 14  - PPS Signal
        LASER_RANGEFINDER,       // 15  - Laser Rangefinders
        THERMOMETER,			 // 16  - Thermometer
        HYGROMETER,				 // 17  - Humidity Sensor
        TIME,                    // 18  - Time messages
        LIDAR_ODOMETRY,          // 19  - Lidar odometry
        FILTER_STATUS,           // 20  - Sensor fusion filter status
        RADAR,                   // 21  - Radar rangefinder
        BASIL_STATUS	  = 253, // 253 - Status messages from BASIL components
        BASIL_COMMAND	  = 254, // 254 - BASIL Command
        UNDEFINED_MESSAGE = 255
      };
  }
    
  namespace IMU
  {
    // BASIL supported IMUs in MANUFACTURER_MODEL format.  BASIL_IMU is
    // an internal BASIL format that may be used for example when combining
    // multiple single axis sensors into a complete IMU.
    enum ImuMdl
      {
        HONEYWELL_HG1930,
        HONEYWELL_HG1940,
        HONEYWELL_HG9900,
        XSENS_MTI,
        INVENSENSE_MPU9150,
        KVH_CG5100,
        NOVATEL_SPAN_IMU,
        SENSONOR_STIM300,
        KVH_1775,
        NORTHROP_GRUMMAN_UIMU,
        BASIL_IMU 	  = 254,
        UNDEFINED_IMU = 255
      };

    // Some IMUs report rates (angular rate and acceleration, e.g. XSENS_MTI),
    // others report deltas (change in orientation and velocity, e.g. KVH_CG5100)
    // and some report both (e.g. HONEYWELL_HG1930)
    enum DataType
      {
        RATE,
        DELTA,
        RATE_AND_DELTA
      };
  }
    
  namespace LIDAR
  {
    enum LidarMdl
      {
        SICK_LMS511PRO,
        GDRS_RCTA,
        UNDEFINED_LIDAR = 255
      };
  }

  namespace LASER_RANGEFINDER
  {
    enum LaserRangefinderMdl
      {
        BAUMER_OADM_13S7480,
        UNDEFINDED_LASER_RANGEFINDER = 255
      };

  }
	
  namespace THERMOMETER
  {
    enum ThermometerMdl
      {
        TI_TMP431,
        SAGE_THERMOMETER,
        UNDEFINED_THERMOMETER = 255
      };
  };

  namespace HYGROMETER
  {
    enum HygrometerMdl
      {
        HONEYWELL_HIH6131,
        UNDEFINED_HUMIDITY = 255
      };
  };

  // BASIL supported GNSS's in MANUFACTURER_MODEL format.
  namespace GNSS
  {
    enum GnssMdl
      {
        NOVATEL_OEMV,
        NOVATEL_OEM6,
        BASIL_GNSS     = 254,
        UNDEFINED_GNSS = 255
      };
  }

  namespace INS
  {
    // BASIL supported INSs. Most often BASIL itself will supply the
    // INS based on IMU data and use the BASIL_INS identifier.  GENERIC_INS
    // is intended for use when combining components of multiple systems,
    // e.g. orientation from an XSENS AHRS and translation from BASIL_INS.
    enum InsMdl
      {
        BASIL_INS = 254,
        UNDEFINED_INS = 255
      };

    // Classical aided inertial navigation uses an indirect Kalman Filter
    // to estimate inertial errors.  A NINE_STATE filter model has position,
    // velocity and orientation components and assumes noise enters randomly
    // through the IMU measurements.  A FIFTEEN_STATE filter additionally
    // estimate gyro and accelerometer bias drifts.
    enum IndirectFilterModel
      {
        NINE_STATE = 9,
        FIFTEEN_STATE = 15
      };
  }

  // BASIL supported encoders.
  namespace ENCODER
  {
    enum EncoderMdl
      {
        QUADRATURE,
        BASIL_ENCODER = 254,
        UNDEFINED_ENCODER = 255
      };
  }

  // BASIL supported visual (camera) odometry systems
  namespace VISUAL_ODOMETRY
  {
    enum VisualOdometryType
      {
        NREC,
        UNDEFINED_VO = 255
      };
  }

  // BASIL supported lidar odometry systems
  namespace LIDAR_ODOMETRY
  {
    enum LidarOdometryType
      {
        DRC,
        UNDEFINED_LO = 255
      };
  }

  // BASIL supported magnetometers in MANUFACTURER_MODEL format.  These can be
  // compasses, reporting heading or 3-axis sensors reporting direct magnetic field
  // readings.
  namespace MAGNETOMETER
  {
    enum MagnetometerMdl
      {
        HONEYWELL_HMC5883L,
        AKM_AK8975,
        UNDEFINED_MAG = 255
      };
  }

  // BASIL supported barometers in MANUFACTURER_MODEL format.
  namespace BAROMETER
  {
    enum BarometerMdl
      {
        BOSCH_BMP085,
        UNDEFINED_BAROMETER = 255
      };
  }

  // BASIL supported place recognition sensors.  These will not typically be
  // a physical sensor but more likely a software component, e.g. the publicly
  // available Oxford University FABMAP binaries.
  namespace PLACE_RECOGNITION
  {
    enum PlaceRecognitionType
      {
        OXFORD_FABMAP,
        VISUAL_BAG_OF_WORDS_MATCH
      };
  }

  // BASIL implements the following filtering algorithms.  Each filter requires
  // a state transition model and a number of measurement models.  BASIL includes
  // 9 and 15 state INS error models and a few common measurement models.
  namespace FILTER
  {
    enum FilterType
      {
        KALMAN,
        EXTENDED_KALMAN,
        UNSCENTED,
        PARTICLE_SIR,
        UNDEFINED_FILTER = 255
      };
  }

  // BASIL supported vehicle status messages.  These are useful for obtaining the
  // state of a given vehicle, e.g. throttle, steer angle, engine RPM etc.
  namespace VEHICLE_STATUS
  {
    enum VehicleStatusType
      {
        RECBOT_STATUS,
        RECBOT_COMMAND,
        GENERIC_OBD2
      };
  }

  namespace CAMERA
  {
    enum CameraMdl
      {
        NEXUS4_REAR,
        NEXUS4_FRONT
      };

    enum ImageType
      {
        JPG,
        PNG,
        RAW
      };
  }

  // BASIL support kinematics measurements.  Relative pose, absolute pose and
  // velocity type messages are supported.
  namespace KINEMATICS
  {
    enum KinematicsType
      {
        POSE,
        VELOCITY
      };
  }
	
  namespace PPS
  {
    enum PpsSource
      {
        SAGE_PPS,
        UNDEFINED_PPS = 255
      };
  }

  namespace RADAR
  {
    enum RadarMdl
      {
        MACOM_MASRAU0025,
        UNDEFINED_RADAR = 255
      };
  }
	
  // BASIL status messages convey information from BASIL components as standard
  // character strings.
  namespace BASIL_STATUS
  {
    enum BasilStatusType
      {
        DEBUG,      // Debug data
        INFO,		// Informational message
        WARNING,	// Warning that may or may not effect performance
        ERROR,		// Error indicates some component is not performing
        UNKNOWN_STATUS = 255
      };

    enum Destination
      {
        CERR,
        SUBSCRIBERS,
        ALL_DESTINATIONS
      };
  }

  // BASIL command messages allow high level control of BASIL components from remote
  // components.  Command messages received on a BASIL interface and passed to subscribing
  // components for handling.  Not all components will handle all commands.
  namespace BASIL_COMMAND
  {
    enum BasilCommandType
      {
        START,				        // Start BASIL
        STOP,				        // Stop BASIL
        RESET,				        // Reset BASIL
        SET_TIME,			        // Set System Time (SAGE-only)
        SET_NEXT_GPS_TIME, 	        // Set System Time to next GPS time (SAGE-only)
        PAUSE,
        UNPAUSE,
        REBOOT,				        // Restart SAGE (SAGE-only)
        TOGGLE_VISUAL_ODOMETRY,     // Turn visual odometry on/off (Deprecated)
        TOGGLE_KINEMATICS,          // Turn kinematic updates on/off (Deprecated)
        TOGGLE_LIDAR_ODOMETRY,      // Turn lidar odometry on/off (Deprecated)
        ON,                         // Turn on the indicated MessageType enum in the payload
        OFF,                        // Turn off the indicated MessageType enum in the payload
        FAST_RESET,                 // Reset BASIL w/ faster responses where possible, e.g. INS alignment
        SOFT_RESET,                 // Reset BASIL w/ minimal interruption where possible
        FAST_SOFT_RESET,            // Reset BASIL w/ mininal interruption and faster responses
        UNKNOWN_COMMAND = 255
      };
  }

  // Internal BASIL time.
  namespace TIME
  {
    // Time message types
    enum TimeType
      {
        KERNEL_RECEIVE_TIME
      };

    // Status of associated time struct
    enum TimeStatus
      {
        FREE_RUNNING=0x00,
        EXTERNAL_DISCIPLINED=0x01,
        RTC_INIT=0x02,
        GPS_INIT_GPS_DISCIPLINED=0x03,
        NETWORK_INIT=0x04,
        EXTERNAL_INIT_EXTERNAL_DISCIPLINED=0x05
      };

    struct Time
    {
      // BASIL time status
      uint8_t timeStatus;

      // GPS week number, 0 = Jan. 1, 1980
      BasilEndian<uint16_t> gpsWeek;

      // Seconds since start of current GPS week
      BasilEndian<uint32_t> seconds;

      // Nano-seconds since start of current second
      BasilEndian<uint32_t> nanoseconds;

      // Get GPS week
      inline unsigned int getWeek() const
      {
        return gpsWeek;
      }

      // Get exact seconds since start of GPS week (ns resolution)
      inline double getSeconds() const
      {
        return seconds + nanoseconds*1E-9;
      }

      // Get exact seconds since start of GPS week (ns resolution)
      inline double getEpochSeconds() const
      {
        return gpsWeek * 604800 + 315964800 + getSeconds();
      }

      // Get the Time Status
      inline TimeStatus getTimeStatus() const
      {
        return static_cast<TimeStatus>(timeStatus);
      }

      // Set exact seconds since start of GPS week (ns resolution)
      inline void setTime(const TimeStatus status, const uint16_t week, const double seconds)
      {
        this->timeStatus = status;
        this->gpsWeek = week;
        this->seconds = static_cast<uint32_t>(seconds);
        this->nanoseconds = static_cast<uint32_t>((seconds - this->seconds) * 1E9);
      }

      // Set time from epoch
      inline void setTime(const TimeStatus status, const double epoch)
      {
        uint16_t week = static_cast<uint16_t>(std::floor((epoch - 315964800) / 604800));
        double seconds = std::fmod(epoch - 315964800, 604800);
        setTime(status, week, seconds);
      }

      // Set time from yy-mm-dd, hh-mm-ss
      inline void setTime(const TimeStatus status,
                          uint16_t year, uint8_t month, const uint8_t day,
                          const uint8_t hour, const uint8_t minute, const double second)
      {
        if (month <= 2)
          {
            year -= 1;
            month += 12;
          }
        double dh = hour + minute/60.0 + second/3600.0;
        double jd = std::floor(365.25*year) + floor(30.6001*(month+1)) + day + dh/24.0 + 1720981.5;
        double mjd = jd - 2400000.5;
        double week = std::floor((mjd-2444244.5)/7.0);
        double seconds = ((mjd-2444244.5)/7.0 - week)*7*86400.0;
        double dweek = std::floor(seconds/86400.0 + 0.5);
        seconds = dweek*86400.0 + dh*3600.0;
        setTime(status, static_cast<uint16_t>(week), seconds);
      }

      // serialization of this struct
      template<class Archive>
      void save(Archive& ar, const uint32_t version) const
      {
        const uint16_t wk = gpsWeek;
        const uint32_t sec = seconds;
        const uint32_t nsec = nanoseconds;
        ar
          & BOOST_SERIALIZATION_NVP(timeStatus)
          & BOOST_SERIALIZATION_NVP(wk)
          & BOOST_SERIALIZATION_NVP(sec)
          & BOOST_SERIALIZATION_NVP(nsec);
      }

      
      template<class Archive>
      void load(Archive& ar, const uint32_t version)
      {
        uint16_t wk;
        uint32_t sec, nsec;

        ar
          & BOOST_SERIALIZATION_NVP(timeStatus)
          & BOOST_SERIALIZATION_NVP(wk)
          & BOOST_SERIALIZATION_NVP(sec)
          & BOOST_SERIALIZATION_NVP(nsec);

        gpsWeek = wk;
        seconds = sec;
        nanoseconds = nsec;
      }


      BOOST_SERIALIZATION_SPLIT_MEMBER();

    } __attribute__((packed));

  }

  // Basil header format.  Attached to all BASIL messages that are serialized for transmission
  // to BASIL users.  Internally BASIL passes references to message classes.  Classes are
  // constructed by deserializing sensor data or by direct construction in code.  BASIL messages
  // are formed by serializing from classes before being transmitted to BASIL users.
  struct BasilHeader
  {
    // 3 Sync Bytes (Ascii CMU)
    int8_t sync[3];

    // Size of this header in bytes (22).
    uint8_t headerSize;

    // ID of the originating hardware, i.e. the hardware interfacing
    // directly to the sensor associated with the message body.
    uint8_t hardwareId;

    // Serialized BASIL::MESSAGE::MessageType enum.  There is a one-to-one
    // correspondence between messageType and BASIL classes for internal
    // communication and callbacks.
    uint8_t messageType;

    // Message version, depends on messageType
    // e.g for messageType = IMU, messageVersion will be one of
    // BASIL::IMU::ImuMdl's enums.  One messageType class can serialize
    // to or deserialize from many messageVersions.
    uint8_t messageVersion;

    // Unique ID of the device associate with the message, in case
    // there are multiple identical devices, e.g. HG1930 IMU #0 &
    // HG1930 IMU #1.
    uint8_t deviceId;

    // Size of the data packet attached to this header
    // Not including header or CRC, this is a property of messageVersion
    BasilEndian<uint16_t> dataSize;

    // Reserved and packing
    uint8_t reserved;

    // BASIL time stamp
    TIME::Time time;

    // serialization of this struct
    template<class Archive>
    void save(Archive& ar, const uint32_t version) const
    {
      const uint16_t ds(dataSize);

      ar 
        & BOOST_SERIALIZATION_NVP(sync)
        & BOOST_SERIALIZATION_NVP(headerSize)
        & BOOST_SERIALIZATION_NVP(hardwareId)
        & BOOST_SERIALIZATION_NVP(messageType)
        & BOOST_SERIALIZATION_NVP(messageVersion)
        & BOOST_SERIALIZATION_NVP(deviceId)
        & BOOST_SERIALIZATION_NVP(ds)
        & BOOST_SERIALIZATION_NVP(reserved)
        & BOOST_SERIALIZATION_NVP(time);
    }

    // serialization of this struct
    template<class Archive>
    void load(Archive& ar, const uint32_t version)
    {
      uint16_t ds;

      ar 
        & BOOST_SERIALIZATION_NVP(sync)
        & BOOST_SERIALIZATION_NVP(headerSize)
        & BOOST_SERIALIZATION_NVP(hardwareId)
        & BOOST_SERIALIZATION_NVP(messageType)
        & BOOST_SERIALIZATION_NVP(messageVersion)
        & BOOST_SERIALIZATION_NVP(deviceId)
        & BOOST_SERIALIZATION_NVP(ds)
        & BOOST_SERIALIZATION_NVP(reserved)
        & BOOST_SERIALIZATION_NVP(time);

      dataSize = ds;
    }

    BOOST_SERIALIZATION_SPLIT_MEMBER();

  } __attribute__((packed));

  // BASIL CRC is a 32 bit value using the Boost CRC library.
  typedef BasilEndian<uint32_t> BasilCrc;

  // serializable types defined 
  typedef BasilEndian<uint16_t> BasilUint16tType;
  typedef BasilEndian<uint32_t> BasilUint32tType;
  typedef BasilEndian<double> BasilDoubleType;

}

BOOST_CLASS_IMPLEMENTATION(BASIL::BasilUint16tType, boost::serialization::object_serializable);
BOOST_CLASS_TRACKING(BASIL::BasilUint16tType, boost::serialization::track_never);

BOOST_CLASS_IMPLEMENTATION(BASIL::BasilUint32tType, boost::serialization::object_serializable);
BOOST_CLASS_TRACKING(BASIL::BasilUint32tType, boost::serialization::track_never);

BOOST_CLASS_IMPLEMENTATION(BASIL::BasilDoubleType, boost::serialization::object_serializable);
BOOST_CLASS_TRACKING(BASIL::BasilDoubleType, boost::serialization::track_never);

BOOST_CLASS_IMPLEMENTATION(BASIL::TIME::Time, boost::serialization::object_serializable);
BOOST_CLASS_TRACKING(BASIL::TIME::Time, boost::serialization::track_never);

BOOST_CLASS_IMPLEMENTATION(BASIL::BasilHeader, boost::serialization::object_serializable);
BOOST_CLASS_TRACKING(BASIL::BasilHeader, boost::serialization::track_never);

#endif //BASIL_H
