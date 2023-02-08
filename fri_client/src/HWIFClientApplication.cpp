#include <fri/HWIFClientApplication.hpp>


namespace KUKA {
    namespace FRI {

        HWIFClientApplication::HWIFClientApplication(IConnection& connection, IClient& client) 
            : ClientApplication(connection, client){}

        bool HWIFClientApplication::client_app_read() {
   if (!_connection.isOpen())
   {
      printf("Error: client application is not connected!\n");
      return false;
   }

   // **************************************************************************
   // Receive and decode new monitoring message
   // **************************************************************************
   size_ = _connection.receive(_data->receiveBuffer, FRI_MONITOR_MSG_MAX_SIZE);
   
   if (size_ <= 0)
   {  // TODO: size_ == 0 -> connection closed (maybe go to IDLE instead of stopping?)
      printf("Error: failed while trying to receive monitoring message!\n");
      return false;
   }
   
   if (!_data->decoder.decode(_data->receiveBuffer, size_))
   {
      return false;
   }
   
   // check message type (so that our wrappers match)
   if (_data->expectedMonitorMsgID != _data->monitoringMsg.header.messageIdentifier)
   {
      printf("Error: incompatible IDs for received message (got: %d expected %d)!\n",
            (int)_data->monitoringMsg.header.messageIdentifier,
            (int)_data->expectedMonitorMsgID);
      return false;
   }   

            return true;
        }

        void HWIFClientApplication::client_app_update() {
            // **************************************************************************
            // callbacks
            // **************************************************************************
            // reset commmand message before callbacks
   _data->resetCommandMessage();
   
   // callbacks for robot client
   ESessionState currentState = (ESessionState)_data->monitoringMsg.connectionInfo.sessionState;
   
   if (_data->lastState != currentState)
   {
      _robotClient->onStateChange(_data->lastState, currentState);
      _data->lastState = currentState;
   }
   
   switch (currentState)
   {
      case MONITORING_WAIT:
      case MONITORING_READY:
         _robotClient->monitor();
         break;
      case COMMANDING_WAIT:
         _robotClient->waitForCommand();
         break;
      case COMMANDING_ACTIVE:
         _robotClient->command();
         break;
      case IDLE:
      default:
         return; // nothing to send back
   }

   // callback for transformation client
   if(_trafoClient != NULL)
   {
      _trafoClient->provide();
   }
        }


        bool HWIFClientApplication::client_app_write() {
            // **************************************************************************
            // Encode and send command message
            // **************************************************************************

   _data->lastSendCounter++;
   // check if its time to send an answer
   if (_data->lastSendCounter >= _data->monitoringMsg.connectionInfo.receiveMultiplier)
   {
      _data->lastSendCounter = 0;
      
      // set sequence counters
      _data->commandMsg.header.sequenceCounter = _data->sequenceCounter++;
      _data->commandMsg.header.reflectedSequenceCounter = 
            _data->monitoringMsg.header.sequenceCounter;
      
      if (!_data->encoder.encode(_data->sendBuffer, size_))
      {
         return false;
      }
      
      if (!_connection.send(_data->sendBuffer, size_))
      {
         printf("Error: failed while trying to send command message!\n");
         return false;
      }
   }
   
   return true;
        }
    }
}  // namespace KUKA::FRI
