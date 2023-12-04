#include <omnimove/external_control_message.hpp>
#include <cstring>
#include <arpa/inet.h>
#include <boost/asio.hpp>
#include <rcl/logging.h>

namespace omnimove{

    ExternalControlMessage::ExternalControlMessage(const char* header, const char *message, const int& data_length){
        size_t header_length = strlen(header);
        msg_length_ = header_length + data_length + 2;
        msg_buffer_ = std::make_unique<char[]>(msg_length_);
        memset(msg_buffer_.get (),0, msg_length_);
        memcpy(msg_buffer_.get (), header, header_length);
        memcpy(msg_buffer_.get()+header_length, message, data_length);
        int16_t crc = calculateCRC();
        memcpy(msg_buffer_.get() + header_length+ data_length, &crc, sizeof(crc));
    }




    int16_t ExternalControlMessage::calculateCRC(){
        int crc = 0xFFFF; // initial value
        int polynomial = 0x1021;

        for (size_t i = 0; i < msg_length_-2; i++) {
            char b = msg_buffer_[i];
            for (int j = 0; j < 8; j++) {
                bool bit = ((b >> (7 - j) & 1) == 1);
                bool c15 = ((crc >> 15 & 1) == 1);
                crc <<= 1;
                if (c15 ^ bit)
                    crc ^= polynomial;
            }
        }
        return (short) crc;
    }

    boost::asio::const_buffer ExternalControlMessage::getSerialisedData(){
        return boost::asio::buffer(msg_buffer_.get(), msg_length_);
    }

    ExternalControlData::ExternalControlData():ExternalControlMessage("","",0), is_valid_(false){

    }

    ExternalControlData::ExternalControlData(const char *msg_data)
        : ExternalControlMessage(
              ExternalControlData::EXTERNAL_CONTROL_DATA_HEADER, msg_data,
              EXTERNAL_CONTROL_DATA_LENGTH),
          is_valid_(true) {

      memcpy(&actual_speed_x_, msg_data + 1, 4);
      memcpy(&actual_speed_y_, msg_data + 5, 4);
      memcpy(&actual_speed_w_, msg_data + 9, 4);
      copyPillarData(pillar1_, msg_data + 82);
      copyPillarData(pillar2_, msg_data + 86);
      copyPillarData(pillar3_, msg_data + 90);
      copyPillarData(pillar4_, msg_data + 94);
      copyPillarData(schild_, msg_data  + 98);


    }

    void ExternalControlData::copyPillarData(PillarData &pillar_data, const char *msg_data){
        memcpy(&pillar_data.target_height_reached_, msg_data , 1);
        memcpy(&pillar_data.manual_active_, msg_data +1, 1);
        memcpy(&pillar_data.actual_pos_, msg_data + 2 , 1);
        memcpy(&pillar_data.actual_speed_, msg_data + 3, 1);

    }

    bool ExternalControlData::isDataValid() const{
        return is_valid_;
    }

     bool ExternalControlData::isMessageValid(const char *msg_data, size_t length) {

      if (length < totalMessageLength ()) {
        return false;
      }

      for (size_t i = 0; i < strlen(EXTERNAL_CONTROL_DATA_HEADER); ++i) {
        if (msg_data[i] != EXTERNAL_CONTROL_DATA_HEADER[i]) {
          return false;
        }
      }

      return true;
    }

     size_t ExternalControlData::totalMessageLength(){
         return EXTERNAL_CONTROL_DATA_LENGTH + strlen(EXTERNAL_CONTROL_DATA_HEADER) + 2;
     }

    int ExternalControlData::speedX() const{
        return actual_speed_x_;
    }

    int ExternalControlData::speedY() const{
        return actual_speed_y_;
    }

    int ExternalControlData::speedW() const{
        return actual_speed_w_;
    }

    int ExternalControlData::speedPillar1() const {
        return pillar1_.actual_speed_;
    }

    int ExternalControlData::posPillar1() const{
        return pillar1_.actual_pos_;
    }

    int ExternalControlData::speedPillar2() const {
        return pillar2_.actual_speed_;
    }
    int ExternalControlData::posPillar2() const{
        return pillar2_.actual_pos_;
    }


    int ExternalControlData::speedPillar3() const {
        return pillar3_.actual_speed_;
    }

    int ExternalControlData::posPillar3() const{
        return pillar3_.actual_pos_;
    }

    int ExternalControlData::speedPillar4() const {
        return pillar4_.actual_speed_;
    }
    int ExternalControlData::posPillar4() const{
        return pillar4_.actual_pos_;
    }

    int ExternalControlData::speedShield() const{
        return schild_.actual_speed_;
    }

    int ExternalControlData::posShield() const{
        return schild_.actual_pos_;
    }


    uint32_t ExternalControlCommand::alive_counter_= 0;

    ExternalControlCommand::ExternalControlCommand(const char *message):ExternalControlMessage(EXTERNAL_CONTROL_COMMAND_HEADER,
                                                                                               message,
                                                                                               EXTERNAL_CONTROL_COMMAND_LENGTH){

    }

    std::unique_ptr<char[]> ExternalControlCommand::getMessageBuffer(){
        auto msg_data = std::make_unique<char[]>(ExternalControlCommand::EXTERNAL_CONTROL_COMMAND_LENGTH);
        memset(msg_data.get (), 0, ExternalControlCommand::EXTERNAL_CONTROL_COMMAND_LENGTH);
        uint32_t counter = ++alive_counter_;
        memcpy(msg_data.get() + ExternalControlCommand::EXTERNAL_CONTROL_COMMAND_LENGTH - 4 , &counter, 4);
        return msg_data;
     }



    ExternalControlOmnimoveDriveCommand::ExternalControlOmnimoveDriveCommand(int speed_x,int speed_y, int speed_w):
        ExternalControlCommand(getMessageData(speed_x, speed_y, speed_w).get()){

    }


    std::unique_ptr<char[]> ExternalControlOmnimoveDriveCommand::getMessageData(int speed_x, int speed_y, int speed_w)
    {
        auto msg_data = getMessageBuffer();
        msg_data.get()[0] = 0;
        msg_data.get()[1] = 2; //set mode to AutoDrive

        *((int32_t *)(msg_data.get() + 2)) = speed_x;
        *((int32_t *)(msg_data.get() + 6)) = speed_y;
        *((int32_t *)(msg_data.get() + 10)) = speed_w;
        return msg_data;
    }
}
