/**

The following license terms and conditions apply, unless a redistribution
agreement or other license is obtained by KUKA Roboter GmbH, Augsburg, Germany.

SCOPE

The software "KUKA Sunrise.Connectivity FRI Client SDK" is targeted to work in
conjunction with the "KUKA Sunrise.Connectivity FastRobotInterface" toolkit.
In the following, the term "software" refers to all material directly
belonging to the provided SDK "Software development kit", particularly source
code, libraries, binaries, manuals and technical documentation.

COPYRIGHT

All Rights Reserved
Copyright (C)  2014-2019 
KUKA Roboter GmbH
Augsburg, Germany

LICENSE 

Redistribution and use of the software in source and binary forms, with or
without modification, are permitted provided that the following conditions are
met:
a) The software is used in conjunction with KUKA products only. 
b) Redistributions of source code must retain the above copyright notice, this
list of conditions and the disclaimer.
c) Redistributions in binary form must reproduce the above copyright notice,
this list of conditions and the disclaimer in the documentation and/or other
materials provided with the distribution. Altered source code of the
redistribution must be made available upon request with the distribution.
d) Modification and contributions to the original software provided by KUKA
must be clearly marked and the authorship must be stated.
e) Neither the name of KUKA nor the trademarks owned by KUKA may be used to
endorse or promote products derived from this software without specific prior
written permission.

DISCLAIMER OF WARRANTY

The Software is provided "AS IS" and "WITH ALL FAULTS," without warranty of
any kind, including without limitation the warranties of merchantability,
fitness for a particular purpose and non-infringement. 
KUKA makes no warranty that the Software is free of defects or is suitable for
any particular purpose. In no event shall KUKA be responsible for loss or
damages arising from the installation or use of the Software, including but
not limited to any indirect, punitive, special, incidental or consequential
damages of any character including, without limitation, damages for loss of
goodwill, work stoppage, computer failure or malfunction, or any and all other
commercial damages or losses. 
The entire risk to the quality and performance of the Software is not borne by
KUKA. Should the Software prove defective, KUKA is not liable for the entire
cost of any service and repair.



\file
\version {1.15}
*/
#ifndef _KUKA_FRI_LBR_STATE_H
#define _KUKA_FRI_LBR_STATE_H

#include <fri/friClientIf.h>

// forward declarations
typedef struct _FRIMonitoringMessage FRIMonitoringMessage;

/** Kuka namespace */
namespace KUKA
{
namespace FRI
{   
   
   /**
    * \brief Wrapper class for the FRI monitoring message for a KUKA LBR (leightweight) robot.
    */
   class LBRState
   {
      friend class LBRClient;
      
   public:

      enum
      {
          NUMBER_OF_JOINTS = 7      //!< number of axes of the KUKA LBR robot   
      };

      LBRState();
      
      /**
       * \brief Get the sample time in seconds. 
       * 
       * This is the period in which the KUKA Sunrise controller is sending 
       * FRI packets.
       * @return sample time in seconds
       */
      double getSampleTime() const; // sec
      
      /**
       * \brief Get the current FRI session state.
       * 
       * @return current FRI session state
       */
      ESessionState getSessionState() const;
      
      /**
       * \brief Get the current FRI connection quality.
       * 
       * @return current FRI connection quality
       */
      EConnectionQuality getConnectionQuality() const;
      
      /**
       * \brief Get the current safety state of the KUKA Sunrise controller.
       * 
       * @return current safety state
       */
      ESafetyState getSafetyState() const;
      
      /**
       * \brief Get the current operation mode of the KUKA Sunrise controller.
       * 
       * @return current operation mode
       */
      EOperationMode getOperationMode() const; 
      
      /**
       * \brief Get the accumulated drive state over all drives of the KUKA LBR controller.
       * 
       * If the drive states differ between drives, the following rule applies:
       * 1) The drive state is OFF if all drives are OFF.
       * 2) The drive state is ACTIVE if all drives are ACTIVE.
       * 3) otherwise the drive state is TRANSITIONING.
       * @return accumulated drive state
       */
      EDriveState getDriveState() const;
            
      /**
      * \brief Get the client command mode specified by the client.
      * 
      * @return the client command mode specified by the client.
      */
      EClientCommandMode getClientCommandMode() const;
      
      /**
       * \brief Get the overlay type specified by the client.
       * 
       * @return the overlay type specified by the client.
       */
      EOverlayType getOverlayType() const;
            
      /**
       * \brief Get the control mode of the KUKA LBR robot.
       * 
       * @return current control mode of the KUKA LBR robot.
       */
      EControlMode getControlMode() const;
      
      /** 
       * \brief Get the timestamp of the current robot state in Unix time. 
       * 
       * This method returns the seconds since 0:00, January 1st, 1970 (UTC).
       * Use getTimestampNanoSec() to increase your timestamp resolution when
       * seconds are insufficient. 
       * @return timestamp encoded as Unix time (seconds)
       */
      unsigned int getTimestampSec() const;
      
      /** 
       * \brief Get the nanoseconds elapsed since the last second (in Unix time).
       * 
       * This method complements getTimestampSec() to get a high resolution 
       * timestamp. 
       * @return timestamp encoded as Unix time (nanoseconds part)
       */
      unsigned int getTimestampNanoSec() const;
      
      /**
       * \brief Get the currently measured joint positions of the robot.
       * 
       * @return array of the measured joint positions in radians
       */
      const double* getMeasuredJointPosition() const;
      
      /**
       * \brief Get the last commanded joint positions of the robot.
       * 
       * @return array of the commanded joint positions in radians
       */
      const double* getCommandedJointPosition() const;
      
      /**
       * \brief Get the currently measured joint torques of the robot.
       * 
       * @return array of the measured torques in Nm
       */
      const double* getMeasuredTorque() const;
      
      /**
       * \brief Get the last commanded joint torques of the robot.
       * 
       * @return array of the commanded torques in Nm
       */
      const double* getCommandedTorque() const;
      
      /**
       * \brief Get the currently measured external joint torques of the robot.
       * 
       * The external torques corresponds to the measured torques when removing
       * the torques induced by the robot itself. 
       * @return array of the external torques in Nm
       */
      const double* getExternalTorque() const;
      
      /**
       * \brief Get the joint positions commanded by the interpolator.
       * 
       * When commanding a motion overlay in your robot application, this method
       * will give access to the joint positions currently commanded by the
       * motion interpolator.
       * @throw FRIException This method will throw an FRIException during monitoring mode.
       * @return array of the ipo joint positions in radians 
       */
      const double* getIpoJointPosition() const;
      
      /**
       * \brief Get an indicator for the current tracking performance of the commanded robot.
       * 
       * The tracking performance is an indicator on how well the commanded robot
       * is able to follow the commands of the FRI client. The best possible value
       * 1.0 is reached when the robot executes the given commands instantaneously.
       * The tracking performance drops towards 0 when latencies are induced, 
       * e.g. when the commanded velocity, acceleration or jerk exceeds the 
       * capabilities of the robot.
       * The tracking performance will always be 0 when the session state does
       * not equal COMMANDING_ACTIVE.
       * @return current tracking performance
       */
      double getTrackingPerformance() const;
      
      /**
       * \brief Get boolean IO value.
       *
       * @throw FRIException May throw an FRIException if the IO is of wrong type or unknown.
       * @param name Full name of the IO (Syntax "IOGroupName.IOName"). 
       * @return Returns IO's boolean value.
       */
      bool getBooleanIOValue(const char* name) const;
      
      /**
       * \brief Get digital IO value.
       *
       * @throw FRIException May throw an FRIException if the IO is of wrong type or unknown.
       * @param name Full name of the IO (Syntax "IOGroupName.IOName"). 
       * @return Returns IO's digital value.
       */
      unsigned long long getDigitalIOValue(const char* name) const;
      
      /**
       * \brief Get analog IO value.
       *
       * @throw FRIException May throw an FRIException if the IO is of wrong type or unknown.
       * @param name Full name of the IO (Syntax "IOGroupName.IOName"). 
       * @return Returns IO's analog value.
       */
      double getAnalogIOValue(const char* name) const;
      
   protected:
      
      static const int LBRMONITORMESSAGEID  = 0x245142;  //!< type identifier for the FRI monitoring message corresponding to a KUKA LBR robot 
      FRIMonitoringMessage* _message;                    //!< FRI monitoring message (protobuf struct)
   };

}
}


#endif // _KUKA_FRI_LBR_STATE_H
