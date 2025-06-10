#ifndef BEEX_TO_XTF_CONVERTER_H
#define BEEX_TO_XTF_CONVERTER_H

#include <bx_msgs/SurveyorInfoPayload.h>
#include <bx_msgs/VehicleState.h>
#include <rosbag/bag.h>
#include <rosbag/view.h>
#include <GeographicLib/UTMUPS.hpp>
#include <chrono>
#include <cmath>
#include <cstdint>
#include <cstring>
#include <ctime>
#include <fstream>
#include <iostream>
#include <stdexcept>
#include <vector>


class BeexToXtfConverter {
public:
    // Constructor
    BeexToXtfConverter(
            const std::string &beex_file,
            const std::string &save_file,
            float orientation             = -45.0,
            const std::string &sonar_type = "R2Sonic");

    // Main processing function
    void run();

private:
    struct Params {
        std::string beex_file;
        std::string save_file;
        float orientation;
        std::string sonar_type;

        struct Offsets {
            float x, y, z;
        } nav_offset, sonar_offset;
    };
    std::vector<uint8_t> multibeam_buffer_;
    std::vector<uint8_t> navigation_buffer_;
    std::vector<uint8_t> attitude_buffer_;

    Params params_;

    std::vector<std::vector<uint8_t>> save_file_byte_array_;
    uint64_t save_file_current_size_;
    int file_counter_;
    std::string base_save_file_;
    double roll, pitch, yaw;


    // Static ROS Topics
    static const std::string MULTIBEAM_TOPIC;
    static const std::string SONAR_INFO_TOPIC;
    static const std::string VEHICLE_LATLON;
    static const uint64_t SAVE_FILE_MAX_SIZE;

    rosbag::Bag bag;

    // Function declarations
    void quaternion_to_euler_angle(double w, double x, double y, double z);
    // std::vector<uint8_t> repack(const void *data, size_t size);
    std::vector<uint8_t> repack(const void *data, size_t size, size_t length);
    std::vector<uint8_t> prepare_beam_bytes(const bx_msgs::SurveyorInfoPayload &data, int i);
    std::pair<std::vector<uint8_t>, std::vector<uint8_t>> prepare_nav_file_packet(const bx_msgs::VehicleState &data);
    std::vector<uint8_t> prepare_header_file_packet();
    std::vector<uint8_t> prepare_multibeam_bytes(const bx_msgs::SurveyorInfoPayload &data, int ping_number);
    void save_current_file();
};

#endif  // BEEX_TO_XTF_CONVERTER_H
