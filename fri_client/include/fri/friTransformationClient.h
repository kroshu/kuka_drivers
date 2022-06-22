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
#ifndef _KUKA_FRI_TRANSFORMATION_CLIENT_H
#define _KUKA_FRI_TRANSFORMATION_CLIENT_H

#include <vector>
#include <fri/friClientIf.h>

/** Kuka namespace */
namespace KUKA
{
namespace FRI
{

	// forward declaration
	struct ClientData;

   /**
    * \brief Abstract FRI transformation client. 
    * 
    * A transformation client enables the user to send transformation matrices cyclically to the 
    * KUKA Sunrise controller for manipulating the transformations of dynamic frames in the
    * Sunrise scenegraph.
    * Usually, these matrices will be provided by external sensors. 
    * <br>
    * Custom transformation clients have to be derived from this class and need to
    * implement the provide() callback. This callback is called once by the
    * client application whenever a new FRI message arrives.
    * 
    * <b>This element is an undocumented internal feature. It is not intended to be used by applications as it might change or be removed in future versions.</b> 
    */
   class TransformationClient
   {
      
      friend class ClientApplication;
     
   public:
      
      /**
       * <br>  <b>This element is an undocumented internal feature. It is not intended to be used by applications as it might change or be removed in future versions.</b>  <br> 
       * \brief Constructor.
       **/
      TransformationClient();
      
      /** <br> <b>This element is an undocumented internal feature. It is not intended to be used by applications as it might change or be removed in future versions.</b>   <br> 
       * \brief Virtual destructor. 
       **/
      virtual ~TransformationClient();
      
      /** 
       * <br>  <b>This element is an undocumented internal feature. It is not intended to be used by applications as it might change or be removed in future versions.</b>  <br>
       * \brief Callback which is called whenever a new FRI message arrives. 
       *
       *  In this callback all requested transformations have to be set.
       *  
       *  \see getRequestedTransformationIDs(), setTransformation()
       */
      virtual void provide() = 0;
      
      /**
       * \brief Get the sample time in seconds. 
       * 
       * This is the period in which the KUKA Sunrise controller is sending 
       * FRI packets.
       * @return sample time in seconds
       */
      double getSampleTime() const; // sec
      
      /**
       * \brief Get the current FRI connection quality.
       * 
       * @return current FRI connection quality
       */
      EConnectionQuality getConnectionQuality() const;      

      /** 
       * <br>  <b>This element is an undocumented internal feature. It is not intended to be used by applications as it might change or be removed in future versions.</b>  <br>
       * \brief  Returns a vector of identifiers of all requested transformation matrices.
       * 
       * The custom TransformationClient has to provide data for transformation matrices with these
       * identifiers. 
       * 
       * @return reference to vector of IDs of requested transformations
       */
      const std::vector<const char*>& getRequestedTransformationIDs() const;
      
      /** 
       * <br>  <b>This element is an undocumented internal feature. It is not intended to be used by applications as it might change or be removed in future versions.</b>  <br>
       * 
       * \brief Get the timestamp of the current received FRI monitor message in Unix time. 
       * 
       * This method returns the seconds since 0:00, January 1st, 1970 (UTC).
       * Use getTimestampNanoSec() to increase your timestamp resolution when
       * seconds are insufficient. 
       * 
       * @return timestamp encoded as Unix time (seconds)
       */
      const unsigned int getTimestampSec() const;
      
      /** 
       * <br>  <b>This element is an undocumented internal feature. It is not intended to be used by applications as it might change or be removed in future versions.</b>  <br>
       * \brief Get the nanoseconds elapsed since the last second (in Unix time).
       * 
       * This method complements getTimestampSec() to get a high resolution 
       * timestamp. 
       * 
       * @return timestamp encoded as Unix time (nanoseconds part)
       */
      const unsigned int getTimestampNanoSec() const;
      
      /**
       * <br>  <b>This element is an undocumented internal feature. It is not intended to be used by applications as it might change or be removed in future versions.</b>  <br>
       * \brief Provides a requested transformation matrix.
       * 
       * A transformation matrix has 3x4 elements. It consists of a rotational matrix (3x3 elements)
       * and a translational vector (3x1 elements). The complete transformation matrix has the
       * following structure: <br>
       * [Transformation(3x4)] = [Rotation(3x3) | Translation(3x1) ]
       * <p>
       * All provided transformation matrices need a timestamp that corresponds to their
       * time of acquisiton. This timestamp must be synchronized to the timestamp
       * provided by the KUKA Sunrise controller (see getTimestampSec(), getTimestampNanoSec()).
       * <p>
       * If an update to the last transformation is not yet available when the provide()
       * callback is executed, the last transformation (including its timestamp) should be
       * repeated until a new transformation is available. 
       *
       * @throw FRIException Throws a FRIException if the maximum number of transformations is exceeded.  
       * @param transformationID Identifier string of the transformation matrix
       * @param transformationMatrix Provided transformation matrix
       * @param timeSec Timestamp encoded as Unix time (seconds)
       * @param timeNanoSec Timestamp encoded as Unix time (nanoseconds part)
       */
      void setTransformation(const char* transformationID, const double transformationMatrix[3][4],
            unsigned int timeSec, unsigned int timeNanoSec);
      
      /**
       * \brief Set boolean output value.
       * 
       * @throw FRIException Throws a FRIException if more outputs are set than can be registered. 
       * @throw FRIException May throw an FRIException if the IO is of wrong type, unknown or not an output.
       * @param name Full name of the IO (Syntax "IOGroupName.IOName").
       * @param value Boolean value to set. 
       */
      void setBooleanIOValue(const char* name, const bool value);
      
      /**
       * \brief Set digital output value.
       * 
       * @throw FRIException Throws a FRIException if more outputs are set than can be registered. 
       * @throw FRIException May throw an FRIException if the IO is of wrong type, unknown or not an output.
       * @param name Full name of the IO (Syntax "IOGroupName.IOName").
       * @param value Digital value to set. 
       */
      void setDigitalIOValue(const char* name, const unsigned long long value);
      
      /**
       * \brief Set analog output value.
       * 
       * @throw FRIException Throws a FRIException if more outputs are set than can be registered. 
       * @throw FRIException May throw an FRIException if the IO is of wrong type, unknown or not an output.
       * @param name Full name of the IO (Syntax "IOGroupName.IOName").
       * @param value Analog value to set. 
       */
      void setAnalogIOValue(const char* name, const double value);  
      
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
      
   private: 
      
      ClientData* _data;   //!< the client data structure
            
      /**
       * \brief Method to link the client data structure (used internally).
       * 
       * @param clientData the client data structure
       */
      void linkData(ClientData* clientData);
      
   };
   
}
}

#endif // _KUKA_FRI_TRANSFORMATION_CLIENT_H
