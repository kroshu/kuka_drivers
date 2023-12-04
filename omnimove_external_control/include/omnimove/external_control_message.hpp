#ifndef EXTERNAL_CONTROL_MESSAGE_H
#define EXTERNAL_CONTROL_MESSAGE_H
#include <cstdint>
#include <memory>
#include <boost/asio.hpp>

namespace omnimove{

    class ExternalControlMessage{
        private:
            std::unique_ptr<char[]> msg_buffer_;
            size_t msg_length_;
        protected:
            const char *header_;
            const char *message_;
            const char *crc_;
            ExternalControlMessage(const char *header, const char *message, const int &msg_length);
        public:
            int16_t calculateCRC();
            boost::asio::const_buffer getSerialisedData();
    };

    class ExternalControlData : public ExternalControlMessage{
        private:
            const bool is_valid_;

        protected:
            static constexpr const char* EXTERNAL_CONTROL_DATA_HEADER = "KMRUTV03";
            static constexpr int EXTERNAL_CONTROL_DATA_LENGTH = 106; //without the header and crc bytes
            uint8_t actual_mode_;
            int32_t actual_speed_x_;
            int32_t actual_speed_y_;
            int32_t actual_speed_w_;
            uint8_t agv_is_moving_;
            int32_t max_agv_speed_x_;
            int32_t max_agv_speed_y_;
            int32_t max_agv_speed_w_;
            int32_t deviation_x_;
            int32_t deviation_y_;
            int32_t deviation_w_;
            uint8_t dev_driving_;
            uint8_t dev_target_reached_;
            uint8_t scanning_fields_;
            uint8_t driving_direction_;
            uint8_t current_state_;
            uint32_t dataMatrix_FrontCamera;
            uint32_t dataMatrix_RearCamera_;
            uint32_t dataMatrix_LeftCamera_;
            uint32_t dataMatrix_RightCamera;
            uint32_t status_optical_track_guiding_;
            uint8_t warning_optical_track_guiding_;
            uint8_t error_optical_track_guiding_;
            int32_t track_deviation_x_;
            int32_t track_deviation_y_;
            int32_t track_deviation_w_;
            uint8_t tag_number_front_camera_;
            uint8_t tag_number_rear_camera_;
            uint8_t tag_number_left_camera_;
            uint8_t tag_number_right_camera_;
            struct PillarData{
                uint8_t manual_active_;
                uint8_t target_height_reached_;
                uint8_t actual_speed_;
                uint8_t actual_pos_;
            };

            PillarData pillar1_;
            PillarData pillar2_;
            PillarData pillar3_;
            PillarData pillar4_;
            PillarData schild_;
            uint32_t alive_signal_;

            void copyPillarData(PillarData &pillar_data, const char *msg_data);
        public:
            ExternalControlData();
            ExternalControlData(const char *msg_data);
            static bool isMessageValid(const char *buffer, size_t length);
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

    class ExternalControlCommand : public ExternalControlMessage{
        protected:
            ExternalControlCommand(const char *message);
            static std::unique_ptr<char[]> getMessageBuffer();
            static uint32_t alive_counter_;
        public:
            static constexpr const char* EXTERNAL_CONTROL_COMMAND_HEADER = "CTRL2KMR";
            static constexpr int EXTERNAL_CONTROL_COMMAND_LENGTH = 68;
/*
            This is the current structure of the the message.
            uint8_t u8_forced_bit_;
            uint8_t u8_mode_;
            int32_t s32_speed_x_;
            int32_t s32_speed_y_;
            int32_t s32_speed_w_;
            int32_t s32_deviation_x_;
            int32_t s32_deviation_y_;
            int32_t s32_deviation_w_;
            uint8_t u8_deviation_start_;
            uint8_t u8_odometry_reset_;
            uint8_t u8_activate_path_planning_;
            uint8_t u8_max_speed_of_allowed_;
            int32_t s32_min_deviation_xy_;
            int32_t s32_min_deviation_w_;
            uint32_t u32_stop_position_;
            uint32_t u32_stopping_distance_;
            int32_t s32_lift_height_;
            int32_t s32_lift_velocity_;
            uint8_t u8_reserve_0_;
            uint8_t u8_reserve_1_;
            uint8_t u8_reserve_2_;
            uint8_t u8_reserve_3_;
            uint8_t u8_reserve_4_;
            uint8_t u8_reserve_5_;
            uint8_t u8_reserve_6_;
            uint8_t u8_reserve_7_;
            uint8_t u8_reserve_8_;
            uint8_t u8_reserve_9_;
            uint32_t u32_counter_received_;
            int16_t s16_crc;
            */
    };

    class ExternalControlOmnimoveDriveCommand : public ExternalControlCommand {
        private:
            std::unique_ptr<char[]> getMessageData(int speed_x, int speed_y, int speed_w);
        public:
            ExternalControlOmnimoveDriveCommand(int speed_x, int speed_y, int speed_w);
    };
} // namespace omnimove
#endif // EXTERNAL_CONTROL_MESSAGE_H
