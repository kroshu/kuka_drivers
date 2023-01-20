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

#ifndef _KUKA_FRI_CLIENT_APPLICATION_H
#define _KUKA_FRI_CLIENT_APPLICATION_H

/** Kuka namespace */
namespace KUKA
{
/** Fast Robot Interface (FRI) namespace */
namespace FRI
{

   // forward declarations
   class IClient;
   class TransformationClient;
   class IConnection;
   struct ClientData;

   /**
    * \brief FRI client application class.
    * 
    * A client application takes an instance of the IConnection interface and
    * an instance of an IClient interface to provide the functionality
    * needed to set up an FRI client application. It can be used to easily 
    * integrate the FRI client code within other applications.
    * The algorithmic functionality of an FRI client application is implemented
    * using the IClient interface.
    */
   class ClientApplication
   {
   
   public:
   
      /**
       * \brief Constructor without transformation client.
       * 
       * This constructor takes an instance of the IConnection interface and
       * an instance of the IClient interface as parameters.
       * @param connection FRI connection class
       * @param client FRI client class
       */
      ClientApplication(IConnection& connection, IClient& client);
     
      /**
       * \brief Constructor with transformation client.
       * 
       * This constructor takes an instance of the IConnection interface and
       * an instance of the IClient interface and an instance of a
       * TransformationClient as parameters.
       * @param connection FRI connection class
       * @param client FRI client class
       * @param trafoClient FRI transformation client class
       */
      ClientApplication(IConnection& connection, IClient& client, TransformationClient& trafoClient);

      /** \brief Destructor. */
      ~ClientApplication();
   
      /**
       * \brief Connect the FRI client application with a KUKA Sunrise controller.
       * 
       * @param port The port ID
       * @param remoteHost The address of the remote host
       * @return True if connection was established
       */
      bool connect(int port, const char *remoteHost = NULL);
      
      /**
       * \brief Disconnect the FRI client application from a KUKA Sunrise controller.
       */
      void disconnect();
   
      /**
       * \brief Run a single processing step.
       * 
       * The processing step consists of receiving a new FRI monitoring message,
       * calling the corresponding client callback and sending the resulting 
       * FRI command message back to the KUKA Sunrise controller.
       * @return True if all of the substeps succeeded.
       */
      bool step();

   protected:
   
      IConnection&   _connection;          //!< connection interface
      IClient*       _robotClient;         //!< robot client interface
      TransformationClient* _trafoClient;  //!< transformation client interface
      ClientData*    _data;                //!< client data structure (for internal use)

   };

}
}


#endif // _KUKA_FRI_CLIENT_APPLICATION_H

