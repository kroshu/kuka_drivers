#include <omnimove/external_control_message.hpp>
#include <cstring>
#include <arpa/inet.h>
#include <boost/asio.hpp>
#include <rcl/logging.h>

namespace omnimove{

    template <typename Buffer>
    ExternalControlMessage::ExternalControlMessage(const char* header, const Buffer &message, const int& data_length){
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

    template <typename Buffer>
    ExternalControlData::ExternalControlData(const Buffer& msg_data):ExternalControlMessage(ExternalControlData::EXTERNAL_CONTROL_DATA_HEADER,
                                                                                          msg_data,
                                                                                          EXTERNAL_CONTROL_DATA_LENGTH), is_valid_(true){

    }

    bool ExternalControlData::isDataValid() const{
        return is_valid_;
    }

    template <typename Buffer>
     bool ExternalControlData::isMessageValid(const Buffer &msg_data) {

      if (msg_data.size() < EXTERNAL_CONTROL_DATA_LENGTH) {
        return false;
      }

      for (size_t i = 0; i < strlen(EXTERNAL_CONTROL_DATA_HEADER); ++i) {
        if (msg_data[i] != EXTERNAL_CONTROL_DATA_HEADER[i]) {
          return false;
        }
      }

      return true;
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


    ExternalControlCommand::ExternalControlCommand(const char *message):ExternalControlMessage(EXTERNAL_CONTROL_COMMAND_HEADER,
                                                                                               message,
                                                                                               EXTERNAL_CONTROL_COMMAND_LENGTH){

    }

    std::unique_ptr<char[]> ExternalControlCommand::getMessageBuffer(){
        auto msg_data = std::make_unique<char[]>(ExternalControlCommand::EXTERNAL_CONTROL_COMMAND_LENGTH);
        memset(msg_data.get (), 0, ExternalControlCommand::EXTERNAL_CONTROL_COMMAND_LENGTH);
        return msg_data;
     }


    /*
ExternalControlCommand::ExternalControlCommand():u8_forced_bit_(0),
    u8_mode_(0),
    s32_speed_x_(0),
    s32_speed_y_(0),
    s32_speed_w_(0),
    s32_deviation_x_(0),
    s32_deviation_y_(0),
    s32_deviation_w_(0),
    u8_deviation_start_(0),
    u8_odometry_reset_(0),
    u8_activate_path_planning_(0),
    u8_max_speed_of_allowed_(0),
    s32_min_deviation_xy_(0),
    s32_min_deviation_w_(0),
    u32_stop_position_(0),
    u32_stopping_distance_(0),
    s32_lift_height_(0),
    s32_lift_velocity_(0),
    u8_reserve_0_(0),
    u8_reserve_1_(0),
    u8_reserve_2_(0),
    u8_reserve_3_(0),
    u8_reserve_4_(0),
    u8_reserve_5_(0),
    u8_reserve_6_(0),
    u8_reserve_7_(0),
    u8_reserve_8_(0),
    u8_reserve_9_(0),
    u32_counter_received_(0),
{

}
*/
    ExternalControlOmnimoveDriveCommand::ExternalControlOmnimoveDriveCommand(int speed_x,int speed_y, int speed_w):
        ExternalControlCommand(getMessageData(speed_x, speed_y, speed_w).get()){

    }


    std::unique_ptr<char[]> ExternalControlOmnimoveDriveCommand::getMessageData(int speed_x, int speed_y, int speed_w)
    {
        auto msg_data = getMessageBuffer();
        msg_data.get()[0] = 0; // setting forced bit to 1
        msg_data.get()[1] = 2; //set mode to AutoDrive

        *((int32_t *)(msg_data.get() + 2)) = htonl(speed_x);
        *((int32_t *)(msg_data.get() + 6)) = htonl(speed_y);
        *((int32_t *)(msg_data.get() + 10)) = htonl(speed_w);
        return msg_data;
    }
}
