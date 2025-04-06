/**

 The following license terms and conditions apply, unless a redistribution
agreement or other license is obtained by KUKA Deutschland GmbH, Augsburg, Germany.

SCOPE

The software “KUKA Sunrise.FRI Client SDK” is targeted to work in
conjunction with the “KUKA Sunrise.FRI” toolkit.
In the following, the term “software” refers to all material directly
belonging to the provided SDK “Software development kit”, particularly source
code, libraries, binaries, manuals and technical documentation.

COPYRIGHT

All Rights Reserved
Copyright (C)  2014-2021
KUKA Deutschland GmbH
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
 \version {2.5}
 */
#ifndef _KUKA_FRI_LBR_COMMAND_H
#define _KUKA_FRI_LBR_COMMAND_H

#include <stddef.h>

// forward declarations
typedef struct _FRICommandMessage FRICommandMessage;

/** Kuka namespace */
namespace KUKA
{
namespace FRI
{

/**
 * \brief Wrapper class for the FRI command message for a KUKA LBR (lightweight) robot.
 */
class LBRCommand
{
   friend class LBRClient;

public:

   /**
    * \brief Set the joint positions for the current interpolation step.
    *
    * This method is only effective when the client is in a commanding state.
    * @param values Array with the new joint positions (in rad)
    */
   void setJointPosition(const double* values);

   /**
    * \brief Set the applied wrench vector of the current interpolation step.
    *
    * The wrench vector consists of:
    * [F_x, F_y, F_z, tau_A, tau_B, tau_C]
    *
    * F ... forces (in N) applied along the Cartesian axes of the
    * currently used motion center.
    * tau ... torques (in Nm) applied along the orientation angles
    * (Euler angles A, B, C) of the currently used motion center.
    *
    * This method is only effective when the client is in a commanding state.
    * The ControlMode of the robot has to be Cartesian impedance control mode. The
    * Client Command Mode has to be wrench.
    *
    * @param wrench Applied Cartesian wrench vector.
    */
   void setWrench(const double* wrench);

   /**
    * \brief Set the applied joint torques for the current interpolation step.
    *
    * This method is only effective when the client is in a commanding state.
    * The ControlMode of the robot has to be joint impedance control mode. The
    * Client Command Mode has to be torque.
    *
    * @param torques Array with the applied torque values (in Nm)
    */
   void setTorque(const double* torques);

   /**
    * \brief Set the Cartesian pose for the current interpolation step.
    *        The pose describes the configured TCP relative to the configured base frame.
    *
    * The quaternion vector consists of: [t_x, t_y, t_z, q_w, q_x, q_y, q_z],
    * where the first three values describe the translation t as a regular 3-dim
    * vector, while the last four values describe the rotation q as an unit quaternion.
    *
    * The unit quaternion q is a 4-dimensional vector, describing the orientation as
    * one rotation around the vector v = [v_x, v_y, v_z] about the angle phi.
    *
    * [ q_w ] = [ cos (phi/2) ]
    * [ q_x ] = [ sin (phi/2) * v_x ]
    * [ q_y ] = [ sin (phi/2) * v_y ]
    * [ q_z ] = [ sin (phi/2) * v_z ]
    *
    * Setting a redundancy value is optional. If no value is provided, the interpolated
    * redundancy value is used. So far, only the E1 redundancy strategy is provided.
    *
    * This method is only effective when the client is in a commanding state.
    *
    * @param cartesianPoseQuaternion Array with the new Cartesian pose
    * @param redundancyValue pointer to redundancy value, NULL for default behavior
    */
   void setCartesianPose(const double* cartesianPoseQuaternion,
         double const * const redundancyValue = NULL);

   /**
    * \brief Set the Cartesian pose for the current interpolation step as a 3x4 matrix.
    *        The pose describes the configured TCP relative to the configured base frame.
    *
    * The first 3 columns represent a rotational matrix and the 4th column a 3-dim
    * translation vector for directions x, y, z (in mm).
    *
    * Setting a redundancy value is optional. If no value is provided, the interpolated
    * redundancy value is used. So far, only the E1 redundancy strategy is provided.
    *
    * @param cartesianPoseAsMatrix 2-dim double array where the requested 3x4 matrix
    * should be stored
    * @param redundancyValue pointer to redundancy value, NULL for default behavior
    */
   void setCartesianPoseAsMatrix(const double(&cartesianPoseAsMatrix)[3][4],
         double const * const redundancyValue = NULL);

   /**
    * \brief Set boolean output value.
    *
    * @throw FRIException Throws a FRIException if more outputs are set than can be registered.
    * @throw FRIException May throw a FRIException if the IO is of wrong type, unknown or not an output.
    * @param name Full name of the IO (Syntax "IOGroupName.IOName").
    * @param value Boolean value to set.
    */
   void setBooleanIOValue(const char* name, const bool value);

   /**
    * \brief Set digital output value.
    *
    * @throw FRIException Throws a FRIException if more outputs are set than can be registered.
    * @throw FRIException May throw a FRIException if the IO is of wrong type, unknown or not an output.
    * @param name Full name of the IO (Syntax "IOGroupName.IOName").
    * @param value Digital value to set.
    */
   void setDigitalIOValue(const char* name, const unsigned long long value);

   /**
    * \brief Set analog output value.
    *
    * @throw FRIException Throws a FRIException if more outputs are set than can be registered.
    * @throw FRIException May throw a FRIException if the IO is of wrong type, unknown or not an output.
    * @param name Full name of the IO (Syntax "IOGroupName.IOName").
    * @param value Analog value to set.
    */
   void setAnalogIOValue(const char* name, const double value);

protected:

   static const int LBRCOMMANDMESSAGEID = 0x34001; //!< type identifier for the FRI command message corresponding to a KUKA LBR robot
   FRICommandMessage* _cmdMessage; //!< FRI command message (protobuf struct)
   FRIMonitoringMessage* _monMessage; //!< FRI monitoring message (protobuf struct)

};

}
}

#endif // _KUKA_FRI_LBR_COMMAND_H
