#ifndef EXTERNAL_CONTROL_MESSAGE_H
#define EXTERNAL_CONTROL_MESSAGE_H
#include <boost/asio.hpp>
#include <cstdint>
#include <memory>

namespace omnimove
{

class ExternalControlMessage
{
private:
  std::unique_ptr<char[]> msg_buffer_;
  size_t msg_length_;

protected:
  const char * header_;
  const char * message_;
  const char * crc_;
  ExternalControlMessage(const char * header, const char * message, const int & msg_length);

public:
  int16_t calculateCRC();
  boost::asio::const_buffer getSerialisedData();
};

class ExternalControlData : public ExternalControlMessage
{
private:
  const bool is_valid_;

protected:
  static constexpr const char * EXTERNAL_CONTROL_DATA_HEADER = "KMRUTV03";
  static constexpr int EXTERNAL_CONTROL_DATA_LENGTH = 106;  // without the header and crc bytes
  int32_t actual_speed_x_;
  int32_t actual_speed_y_;
  int32_t actual_speed_w_;

  struct PillarData
  {
    uint8_t manual_active_;
    uint8_t target_height_reached_;
    int8_t actual_speed_;
    uint8_t actual_pos_;
  };
  PillarData pillars_[4];
  PillarData shield_;

  void copyPillarData(PillarData & pillar_data, const char * msg_data);

public:
  ExternalControlData();
  ExternalControlData(const char * msg_data);
  static bool isMessageValid(const char * buffer, size_t length);
  static size_t totalMessageLength();
  int actualMode() const;
  int speedX() const;
  int speedY() const;
  int speedW() const;
  int speedPillar1() const;
  int speedPillar2() const;
  int speedPillar3() const;
  int speedPillar4() const;
  int speedShield() const;
  int posPillar1() const;
  int posPillar2() const;
  int posPillar3() const;
  int posPillar4() const;
  int posShield() const;
  bool isDataValid() const;
};

class ExternalControlCommand : public ExternalControlMessage
{
protected:
  ExternalControlCommand(const char * message);
  static std::unique_ptr<char[]> getMessageBuffer();
  static uint32_t alive_counter_;

public:
  static constexpr const char * EXTERNAL_CONTROL_COMMAND_HEADER = "CTRL2KMR";
  static constexpr int EXTERNAL_CONTROL_COMMAND_LENGTH = 68;
};

class ExternalControlOmnimoveDriveCommand : public ExternalControlCommand
{
private:
  std::unique_ptr<char[]> getMessageData(int speed_x, int speed_y, int speed_w);

public:
  ExternalControlOmnimoveDriveCommand(int speed_x, int speed_y, int speed_w);
};

class ExternalControlStopCommand : public ExternalControlCommand
{
public:
  ExternalControlStopCommand();
};

class ExternalControlCaterpillarDriveCommand : public ExternalControlCommand
{
private:
  std::unique_ptr<char[]> getMessageData(
    int speed_x, int speed_w, int pillar1_pos, int pillar2_pos, int pillar3_pos, int pillar4_pos,
    int shield_pos);

public:
  ExternalControlCaterpillarDriveCommand(
    int speed_x, int speed_w, int pillar1_pos, int pillar2_pos, int pillar3_pos, int pillar4_pos,
    int shield_pos);
};

}  // namespace omnimove
#endif  // EXTERNAL_CONTROL_MESSAGE_H
