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
#ifndef _KUKA_FRI_CLIENT_H
#define _KUKA_FRI_CLIENT_H



/** Kuka namespace */
namespace KUKA
{
/** Fast Robot Interface (FRI) namespace */
namespace FRI
{

   // forward declarations
   struct ClientData;


   /** \brief Enumeration of the FRI session state. */
   enum ESessionState
   {
      IDLE = 0,               //!< no session available
      MONITORING_WAIT = 1,    //!< monitoring mode with insufficient connection quality
      MONITORING_READY = 2,   //!< monitoring mode with connection quality sufficient for command mode
      COMMANDING_WAIT = 3,    //!< command mode about to start (overlay motion queued)
      COMMANDING_ACTIVE = 4   //!< command mode active
   };

   /** \brief Enumeration of the FRI connection quality. */
   enum EConnectionQuality
   {
      POOR = 0,               //!< poor connection quality
      FAIR = 1,               //!< connection quality insufficient for command mode
      GOOD = 2,               //!< connection quality sufficient for command mode
      EXCELLENT = 3           //!< excellent connection quality
   };

   /** \brief Enumeration of the controller's safety state. */
   enum ESafetyState
   {
      NORMAL_OPERATION = 0,   //!< No safety stop request present.
      SAFETY_STOP_LEVEL_0 = 1,//!< Safety stop request STOP0 or STOP1 present.
      SAFETY_STOP_LEVEL_1 = 2,//!< Safety stop request STOP1 (on-path) present.
      SAFETY_STOP_LEVEL_2 = 3 //!< Safety stop request STOP2 present.
   };

   /** \brief Enumeration of the controller's current mode of operation. */
   enum EOperationMode
   {
      TEST_MODE_1 = 0,        //!< test mode 1 with reduced speed (T1)
      TEST_MODE_2 = 1,        //!< test mode 2 (T2)
      AUTOMATIC_MODE = 2      //!< automatic operation mode (AUT)
   };

   /** \brief Enumeration of a drive's state. */
   enum EDriveState
   {
      OFF = 0,                //!< drive is not being used
      TRANSITIONING = 1,      //!< drive is in a transitioning state (before or after motion)
      ACTIVE = 2              //!< drive is being actively commanded
   };

   /** \brief Enumeration of control mode. */
   enum EControlMode
   {
      POSITION_CONTROL_MODE = 0,  //!< position control mode
      CART_IMP_CONTROL_MODE = 1,  //!< cartesian impedance control mode
      JOINT_IMP_CONTROL_MODE = 2, //!< joint impedance control mode
      NO_CONTROL = 3              //!< drives are not used
   };


   /** \brief Enumeration of the client command mode. */
   enum EClientCommandMode
   {
      NO_COMMAND_MODE = 0, //!< no client command mode available
      JOINT_POSITION = 1,  //!< commanding joint positions by the client
      WRENCH = 2,          //!< commanding wrenches and joint positions by the client
      TORQUE = 3,          //!< commanding joint torques and joint positions by the client
      CARTESIAN_POSE = 4   //!< commanding Cartesian poses by the client
   };

   /** \brief Enumeration of the overlay type. */
   enum EOverlayType
   {
      NO_OVERLAY = 0,   //!< no overlay type available
      JOINT = 1,        //!< joint overlay
      CARTESIAN = 2     //!< cartesian overlay
   };

   /** \brief Enumeration of redundancy strategies. */
   enum ERedundancyStrategy
   {
      E1 = 0,          //!< E1 redundancy strategy
      NO_STRATEGY = 4  //!< No redundancy strategy
   };

   /**
    * \brief FRI client interface.
    *
    * This is the callback interface that should be implemented by FRI clients.
    * Callbacks are automatically called by the client application
    * (ClientApplication) whenever new FRI messages arrive.
    */
   class IClient
   {
      friend class ClientApplication;

   public:

      /** \brief Virtual destructor. */
      virtual ~IClient() {}

      /**
       * \brief Callback that is called whenever the FRI session state changes.
       *
       * @param oldState previous FRI session state
       * @param newState current FRI session state
       */
      virtual void onStateChange(ESessionState oldState, ESessionState newState) = 0;

      /**
       * \brief Callback for the FRI session states 'Monitoring Wait' and 'Monitoring Ready'.
       */
      virtual void monitor() = 0;

      /**
       * \brief Callback for the FRI session state 'Commanding Wait'.
       */
      virtual void waitForCommand() = 0;

      /**
       * \brief Callback for the FRI session state 'Commanding'.
       */
      virtual void command() = 0;

   protected:

      /**
       * \brief Method to create and initialize the client data structure (used internally).
       *
       * @return newly allocated client data structure
       */
      virtual ClientData* createData() = 0;


   };

}
}


#endif // _KUKA_FRI_CLIENT_H
