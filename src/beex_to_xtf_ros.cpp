// Intially have issue with using buffer. If switch back to old method of saving everything in the the bytes array
// then
// 1-shot save in xtf file, it might work //
#include "beex_to_xtf_converter.h"

// Static member definitions
const std::string BeexToXtfConverter::MULTIBEAM_TOPIC  = "/ikan/profiling_sonar/data";
const std::string BeexToXtfConverter::SONAR_INFO_TOPIC = "/ikan/profiling_sonar/info";
const std::string BeexToXtfConverter::VEHICLE_LATLON   = "/ikan/nav/world_ned_msl";
const uint64_t BeexToXtfConverter::SAVE_FILE_MAX_SIZE  = 1'024ULL * 1'024ULL * 1'024ULL;

// Constructor (unchanged)
BeexToXtfConverter::BeexToXtfConverter(
        const std::string &beex_file,
        const std::string &save_file,
        float orientation,
        const std::string &sonar_type)
        : save_file_current_size_(0),
          file_counter_(0) {
    params_.beex_file      = beex_file;
    params_.save_file      = save_file;
    params_.orientation    = orientation;
    params_.sonar_type     = sonar_type;
    params_.nav_offset.x   = 0.096f;
    params_.nav_offset.y   = -0.189f;
    params_.nav_offset.z   = 0.0f;
    params_.sonar_offset.x = -0.012;
    params_.sonar_offset.y = 0.146;
    params_.sonar_offset.z = 0.568;
    if (sonar_type != "R2Sonic") {
        throw std::runtime_error("Unsupported sonar type. Only 'R2Sonic' is supported.");
    }

    if (save_file.find(".xtf") == std::string::npos || save_file.substr(save_file.length() - 4) != ".xtf") {
        throw std::runtime_error("Save file must have .xtf extension.");
    }

    base_save_file_   = save_file.substr(0, save_file.find(".xtf"));
    params_.save_file = base_save_file_ + "_test10.xtf";
}

// Quaternion to Euler (unchanged)
void BeexToXtfConverter::quaternion_to_euler_angle(double w, double x, double y, double z) {
    double ysqr = y * y;
    double t0   = 2.0 * (w * x + y * z);
    double t1   = 1.0 - 2.0 * (x * x + ysqr);
    roll        = std::atan2(t0, t1) * 180.0 / M_PI;

    double t2 = 2.0 * (w * y - z * x);
    t2        = std::clamp(t2, -1.0, 1.0);
    pitch     = std::asin(t2) * 180.0 / M_PI;

    double t3 = 2.0 * (w * z + x * y);
    double t4 = 1.0 - 2.0 * (ysqr + z * z);
    yaw       = std::atan2(t3, t4) * 180.0 / M_PI;
}

std::vector<uint8_t> BeexToXtfConverter::repack(const void *data, size_t size, size_t length) {
    std::vector<uint8_t> byte_data(length, 0);
    std::cout << "repacking" << std::endl;
    memcpy(byte_data.data(), data, std::min(size, length));
    std::cout << "repacking success" << std::endl;
    return byte_data;
}

// Prepare beam bytes (unchanged)
std::vector<uint8_t> BeexToXtfConverter::prepare_beam_bytes(const bx_msgs::SurveyorInfoPayload &data, int i) {
    std::vector<uint8_t> buffer(64, 0);  // Pre-allocate exact 64 bytes
    uint8_t *ptr = buffer.data();

    uint32_t Id             = static_cast<uint32_t>(i);                   // 4 bytes (int)
    double Intensity        = 0.0;                                        // 8 bytes (double)
    uint32_t Quality        = 0;                                          // 4 bytes (int)
    double TwoWayTravelTime = static_cast<double>(data.bathy_twoway[i]);  // 8 bytes (double)
    double DeltaTime        = 0.0;                                        // 8 bytes (double)
    double BeamAngle        = static_cast<double>(data.beam_angle[i]);    // 8 bytes (double)
    double TiltAngle        = 0.0;                                        // 8 bytes (double)
    float Reserved[4]       = {0};                                        // 16 bytes (float[4])

    memcpy(ptr, &Id, sizeof(Id));
    ptr += sizeof(Id);
    std::cout << "After Id: " << (ptr - buffer.data()) << std::endl;
    memcpy(ptr, &Intensity, sizeof(Intensity));
    ptr += sizeof(Intensity);
    std::cout << "After Intensity: " << (ptr - buffer.data()) << std::endl;
    memcpy(ptr, &Quality, sizeof(Quality));
    ptr += sizeof(Quality);
    std::cout << "After Quality: " << (ptr - buffer.data()) << std::endl;
    memcpy(ptr, &TwoWayTravelTime, sizeof(TwoWayTravelTime));
    ptr += sizeof(TwoWayTravelTime);
    std::cout << "After TwoWayTravelTime: " << (ptr - buffer.data()) << std::endl;
    memcpy(ptr, &DeltaTime, sizeof(DeltaTime));
    ptr += sizeof(DeltaTime);
    std::cout << "After DeltaTime: " << (ptr - buffer.data()) << std::endl;
    memcpy(ptr, &BeamAngle, sizeof(BeamAngle));
    ptr += sizeof(BeamAngle);
    std::cout << "After BeamAngle: " << (ptr - buffer.data()) << std::endl;
    memcpy(ptr, &TiltAngle, sizeof(TiltAngle));
    ptr += sizeof(TiltAngle);
    std::cout << "After TiltAngle: " << (ptr - buffer.data()) << std::endl;

    // Use repack for Reserved field
    std::vector<uint8_t> reserved_bytes = repack(Reserved, sizeof(Reserved), 16);
    memcpy(ptr, reserved_bytes.data(), reserved_bytes.size());
    ptr += reserved_bytes.size();
    std::cout << "After Reserved: " << (ptr - buffer.data()) << std::endl;


    if (ptr - buffer.data() != 64) {
        std::cerr << "Error: Beam size mismatch: " << (ptr - buffer.data()) << " != 64" << std::endl;
        throw std::runtime_error("Beam size mismatch");
    }

    return buffer;
}

std::pair<std::vector<uint8_t>, std::vector<uint8_t>> BeexToXtfConverter::prepare_nav_file_packet(
        const bx_msgs::VehicleState &data) {
    std::vector<uint8_t> nav_bytes, alt_bytes;

    // Convert NED to UTM
    double utm_easting  = data.utm_easting + data.pose_ned.position.y;
    double utm_northing = data.utm_northing + data.pose_ned.position.x;
    double altitude     = data.altitude;    // Down in NED (negative)
    float heave         = -data.depth_msl;  // Heave is negative depth

    int utm_zone_number = data.utm_zone_number;
    bool northern       = true;

    // Convert UTM to Latitude/Longitude
    double latitude_deg, longitude_deg;
    try {
        GeographicLib::UTMUPS::Reverse(
                utm_zone_number,
                northern,
                utm_easting,
                utm_northing,
                latitude_deg,
                longitude_deg);
    } catch (const GeographicLib::GeographicErr &e) {
        std::cerr << "GeographicLib error: " << e.what() << std::endl;
        latitude_deg  = 0.0;
        longitude_deg = 0.0;
    }

    // Convert quaternion to RPY
    quaternion_to_euler_angle(
            data.pose_ned.orientation.w,
            data.pose_ned.orientation.x,
            data.pose_ned.orientation.y,
            data.pose_ned.orientation.z);

    float Heading = static_cast<float>(yaw);

    // Convert Unix timestamp
    uint64_t unix_time_ms = data.unix_time_ms;
    time_t unix_timestamp = static_cast<time_t>(unix_time_ms / 1'000);
    struct tm *dt         = gmtime(&unix_timestamp);
    uint32_t microseconds = (unix_time_ms % 1'000) * 1'000;

    // ---- NAVIGATION PACKET ----
    uint16_t MagicNumber        = 64'206;  // "0xFACE"
    uint8_t HeaderType          = 42;      // XTF_HEADER_NAVIGATION
    uint8_t Reserved[7]         = {0};     // Reserved, 7 bytes
    uint32_t NumBytesThisRecord = 64;      // Confirmed 64 bytes
    uint16_t Year               = dt->tm_year + 1'900;
    uint8_t Month               = dt->tm_mon + 1;
    uint8_t Day                 = dt->tm_mday;
    uint8_t Hour                = dt->tm_hour;
    uint8_t Minute              = dt->tm_min;
    uint8_t Second              = dt->tm_sec;
    uint32_t SourceEpoch        = unix_timestamp;
    uint32_t TimeTag            = 0;  // Time since boot in ms
    double Raw_Y_Coordinate     = longitude_deg;
    double Raw_X_Coordinate     = latitude_deg;
    double Raw_Altitude         = data.pose_ned.position.z;
    uint8_t TimeFlag            = 3;
    uint8_t Reserved1[6]        = {0};  // Reserved, 6 bytes

    // Pack into byte vector
    nav_bytes.insert(
            nav_bytes.end(),
            reinterpret_cast<uint8_t *>(&MagicNumber),
            reinterpret_cast<uint8_t *>(&MagicNumber) + sizeof(MagicNumber));
    nav_bytes.insert(
            nav_bytes.end(),
            reinterpret_cast<uint8_t *>(&HeaderType),
            reinterpret_cast<uint8_t *>(&HeaderType) + sizeof(HeaderType));
    nav_bytes.insert(nav_bytes.end(), Reserved, Reserved + sizeof(Reserved));
    nav_bytes.insert(
            nav_bytes.end(),
            reinterpret_cast<uint8_t *>(&NumBytesThisRecord),
            reinterpret_cast<uint8_t *>(&NumBytesThisRecord) + sizeof(NumBytesThisRecord));
    nav_bytes.insert(
            nav_bytes.end(),
            reinterpret_cast<uint8_t *>(&Year),
            reinterpret_cast<uint8_t *>(&Year) + sizeof(Year));
    nav_bytes.insert(
            nav_bytes.end(),
            reinterpret_cast<uint8_t *>(&Month),
            reinterpret_cast<uint8_t *>(&Month) + sizeof(Month));
    nav_bytes.insert(
            nav_bytes.end(),
            reinterpret_cast<uint8_t *>(&Day),
            reinterpret_cast<uint8_t *>(&Day) + sizeof(Day));
    nav_bytes.insert(
            nav_bytes.end(),
            reinterpret_cast<uint8_t *>(&Hour),
            reinterpret_cast<uint8_t *>(&Hour) + sizeof(Hour));
    nav_bytes.insert(
            nav_bytes.end(),
            reinterpret_cast<uint8_t *>(&Minute),
            reinterpret_cast<uint8_t *>(&Minute) + sizeof(Minute));
    nav_bytes.insert(
            nav_bytes.end(),
            reinterpret_cast<uint8_t *>(&Second),
            reinterpret_cast<uint8_t *>(&Second) + sizeof(Second));
    nav_bytes.insert(
            nav_bytes.end(),
            reinterpret_cast<uint8_t *>(&microseconds),
            reinterpret_cast<uint8_t *>(&microseconds) + sizeof(microseconds));
    nav_bytes.insert(
            nav_bytes.end(),
            reinterpret_cast<uint8_t *>(&SourceEpoch),
            reinterpret_cast<uint8_t *>(&SourceEpoch) + sizeof(SourceEpoch));
    nav_bytes.insert(
            nav_bytes.end(),
            reinterpret_cast<uint8_t *>(&TimeTag),
            reinterpret_cast<uint8_t *>(&TimeTag) + sizeof(TimeTag));
    nav_bytes.insert(
            nav_bytes.end(),
            reinterpret_cast<uint8_t *>(&Raw_Y_Coordinate),
            reinterpret_cast<uint8_t *>(&Raw_Y_Coordinate) + sizeof(Raw_Y_Coordinate));
    nav_bytes.insert(
            nav_bytes.end(),
            reinterpret_cast<uint8_t *>(&Raw_X_Coordinate),
            reinterpret_cast<uint8_t *>(&Raw_X_Coordinate) + sizeof(Raw_X_Coordinate));
    nav_bytes.insert(
            nav_bytes.end(),
            reinterpret_cast<uint8_t *>(&Raw_Altitude),
            reinterpret_cast<uint8_t *>(&Raw_Altitude) + sizeof(Raw_Altitude));
    nav_bytes.insert(
            nav_bytes.end(),
            reinterpret_cast<uint8_t *>(&TimeFlag),
            reinterpret_cast<uint8_t *>(&TimeFlag) + sizeof(TimeFlag));
    nav_bytes.insert(nav_bytes.end(), Reserved1, Reserved1 + sizeof(Reserved1));

    if (nav_bytes.size() != 64) {
        std::cerr << "Error: Navigation packet size mismatch!" << std::endl;
        nav_bytes.resize(64, 0);
    }

    // Attitude packet
    HeaderType                 = 3;
    uint8_t SubChannelNumber   = 0;
    uint16_t NumChansToFollow  = 0;
    uint16_t Reserved1_Att[2]  = {0};
    NumBytesThisRecord         = 64;
    uint32_t Reserved2_Att[2]  = {0};
    uint32_t EpochMicroSeconds = microseconds;
    SourceEpoch                = unix_timestamp;
    float Pitch                = static_cast<float>(pitch);
    float Roll                 = static_cast<float>(roll);
    float Yaw_Att              = 0.0f;
    uint16_t Milliseconds      = static_cast<uint16_t>(microseconds / 1'000);
    uint8_t Reserved3          = 0;

    alt_bytes.insert(
            alt_bytes.end(),
            reinterpret_cast<uint8_t *>(&MagicNumber),
            reinterpret_cast<uint8_t *>(&MagicNumber) + sizeof(MagicNumber));
    alt_bytes.insert(
            alt_bytes.end(),
            reinterpret_cast<uint8_t *>(&HeaderType),
            reinterpret_cast<uint8_t *>(&HeaderType) + sizeof(HeaderType));
    alt_bytes.insert(
            alt_bytes.end(),
            reinterpret_cast<uint8_t *>(&SubChannelNumber),
            reinterpret_cast<uint8_t *>(&SubChannelNumber) + sizeof(SubChannelNumber));
    alt_bytes.insert(
            alt_bytes.end(),
            reinterpret_cast<uint8_t *>(&NumChansToFollow),
            reinterpret_cast<uint8_t *>(&NumChansToFollow) + sizeof(NumChansToFollow));
    alt_bytes.insert(
            alt_bytes.end(),
            reinterpret_cast<uint8_t *>(Reserved1_Att),
            reinterpret_cast<uint8_t *>(Reserved1_Att) + sizeof(Reserved1_Att));
    alt_bytes.insert(
            alt_bytes.end(),
            reinterpret_cast<uint8_t *>(&NumBytesThisRecord),
            reinterpret_cast<uint8_t *>(&NumBytesThisRecord) + sizeof(NumBytesThisRecord));
    alt_bytes.insert(
            alt_bytes.end(),
            reinterpret_cast<uint8_t *>(Reserved2_Att),
            reinterpret_cast<uint8_t *>(Reserved2_Att) + sizeof(Reserved2_Att));
    alt_bytes.insert(
            alt_bytes.end(),
            reinterpret_cast<uint8_t *>(&EpochMicroSeconds),
            reinterpret_cast<uint8_t *>(&EpochMicroSeconds) + sizeof(EpochMicroSeconds));
    alt_bytes.insert(
            alt_bytes.end(),
            reinterpret_cast<uint8_t *>(&SourceEpoch),
            reinterpret_cast<uint8_t *>(&SourceEpoch) + sizeof(SourceEpoch));
    alt_bytes.insert(
            alt_bytes.end(),
            reinterpret_cast<uint8_t *>(&Pitch),
            reinterpret_cast<uint8_t *>(&Pitch) + sizeof(Pitch));
    alt_bytes.insert(
            alt_bytes.end(),
            reinterpret_cast<uint8_t *>(&Roll),
            reinterpret_cast<uint8_t *>(&Roll) + sizeof(Roll));
    alt_bytes.insert(
            alt_bytes.end(),
            reinterpret_cast<uint8_t *>(&heave),
            reinterpret_cast<uint8_t *>(&heave) + sizeof(heave));
    alt_bytes.insert(
            alt_bytes.end(),
            reinterpret_cast<uint8_t *>(&Yaw_Att),
            reinterpret_cast<uint8_t *>(&Yaw_Att) + sizeof(Yaw_Att));
    alt_bytes.insert(
            alt_bytes.end(),
            reinterpret_cast<uint8_t *>(&TimeTag),
            reinterpret_cast<uint8_t *>(&TimeTag) + sizeof(TimeTag));
    alt_bytes.insert(
            alt_bytes.end(),
            reinterpret_cast<uint8_t *>(&Heading),
            reinterpret_cast<uint8_t *>(&Heading) + sizeof(Heading));
    alt_bytes.insert(
            alt_bytes.end(),
            reinterpret_cast<uint8_t *>(&Year),
            reinterpret_cast<uint8_t *>(&Year) + sizeof(Year));
    alt_bytes.insert(
            alt_bytes.end(),
            reinterpret_cast<uint8_t *>(&Month),
            reinterpret_cast<uint8_t *>(&Month) + sizeof(Month));
    alt_bytes.insert(
            alt_bytes.end(),
            reinterpret_cast<uint8_t *>(&Day),
            reinterpret_cast<uint8_t *>(&Day) + sizeof(Day));
    alt_bytes.insert(
            alt_bytes.end(),
            reinterpret_cast<uint8_t *>(&Hour),
            reinterpret_cast<uint8_t *>(&Hour) + sizeof(Hour));
    alt_bytes.insert(
            alt_bytes.end(),
            reinterpret_cast<uint8_t *>(&Minute),
            reinterpret_cast<uint8_t *>(&Minute) + sizeof(Minute));
    alt_bytes.insert(
            alt_bytes.end(),
            reinterpret_cast<uint8_t *>(&Second),
            reinterpret_cast<uint8_t *>(&Second) + sizeof(Second));
    alt_bytes.insert(
            alt_bytes.end(),
            reinterpret_cast<uint8_t *>(&Milliseconds),
            reinterpret_cast<uint8_t *>(&Milliseconds) + sizeof(Milliseconds));
    alt_bytes.insert(
            alt_bytes.end(),
            reinterpret_cast<uint8_t *>(&Reserved3),
            reinterpret_cast<uint8_t *>(&Reserved3) + sizeof(Reserved3));

    if (alt_bytes.size() != 64) {
        std::cerr << "Attitude packet size mismatch: " << alt_bytes.size() << " != 64" << std::endl;
        alt_bytes.resize(64, 0);
    }

    return {nav_bytes, alt_bytes};
}

// // Prepare navigation file packet (unchanged)
// std::pair<std::vector<uint8_t>, std::vector<uint8_t>> BeexToXtfConverter::prepare_nav_file_packet(
//         const nav_msgs::Odometry &data) {
//     std::vector<uint8_t> nav_bytes, alt_bytes;

//     double utm_northing = data.pose.pose.position.y;
//     double utm_easting  = data.pose.pose.position.x;
//     int utm_zone_number = 32;
//     bool northern       = true;

//     /* MAYBE NEED TO REMOVE THIS */
//     utm_northing += data.pose.pose.position.y;
//     utm_easting += data.pose.pose.position.x;

//     double latitude_deg, longitude_deg;
//     GeographicLib::UTMUPS::Reverse(utm_zone_number, northern, utm_easting, utm_northing, latitude_deg,
//     longitude_deg);

//     double roll, pitch, yaw;
//     quaternion_to_euler_angle(
//             data.pose.pose.orientation.w,
//             data.pose.pose.orientation.x,
//             data.pose.pose.orientation.y,
//             data.pose.pose.orientation.z,
//             roll,
//             pitch,
//             yaw);

//     uint64_t unix_time_ms = data.header.stamp.toNSec() / 1'000'000;
//     time_t unix_timestamp = static_cast<time_t>(unix_time_ms / 1'000);
//     struct tm *dt         = gmtime(&unix_timestamp);
//     uint32_t microseconds = (unix_time_ms % 1'000) * 1'000;

//     uint16_t MagicNumber        = 64'206;
//     uint8_t HeaderType          = 42;
//     uint8_t Reserved[7]         = {0};
//     uint32_t NumBytesThisRecord = 64;
//     uint16_t Year               = dt->tm_year + 1'900;
//     uint8_t Month               = dt->tm_mon + 1;
//     uint8_t Day                 = dt->tm_mday;
//     uint8_t Hour                = dt->tm_hour;
//     uint8_t Minute              = dt->tm_min;
//     uint8_t Second              = dt->tm_sec;
//     uint32_t Microseconds       = microseconds;
//     uint32_t SourceEpoch        = unix_timestamp;
//     uint32_t TimeTag            = 0;
//     double Raw_Y_Coordinate     = longitude_deg;
//     double Raw_X_Coordinate     = latitude_deg;
//     double Raw_Altitude         = data.pose.pose.position.z;
//     uint8_t TimeFlag            = 3;
//     uint8_t Reserved1[6]        = {0};

//     nav_bytes.insert(
//             nav_bytes.end(),
//             reinterpret_cast<uint8_t *>(&MagicNumber),
//             reinterpret_cast<uint8_t *>(&MagicNumber) + sizeof(MagicNumber));
//     nav_bytes.insert(
//             nav_bytes.end(),
//             reinterpret_cast<uint8_t *>(&HeaderType),
//             reinterpret_cast<uint8_t *>(&HeaderType) + sizeof(HeaderType));
//     nav_bytes.insert(nav_bytes.end(), Reserved, Reserved + sizeof(Reserved));
//     nav_bytes.insert(
//             nav_bytes.end(),
//             reinterpret_cast<uint8_t *>(&NumBytesThisRecord),
//             reinterpret_cast<uint8_t *>(&NumBytesThisRecord) + sizeof(NumBytesThisRecord));
//     nav_bytes.insert(
//             nav_bytes.end(),
//             reinterpret_cast<uint8_t *>(&Year),
//             reinterpret_cast<uint8_t *>(&Year) + sizeof(Year));
//     nav_bytes.insert(
//             nav_bytes.end(),
//             reinterpret_cast<uint8_t *>(&Month),
//             reinterpret_cast<uint8_t *>(&Month) + sizeof(Month));
//     nav_bytes.insert(
//             nav_bytes.end(),
//             reinterpret_cast<uint8_t *>(&Day),
//             reinterpret_cast<uint8_t *>(&Day) + sizeof(Day));
//     nav_bytes.insert(
//             nav_bytes.end(),
//             reinterpret_cast<uint8_t *>(&Hour),
//             reinterpret_cast<uint8_t *>(&Hour) + sizeof(Hour));
//     nav_bytes.insert(
//             nav_bytes.end(),
//             reinterpret_cast<uint8_t *>(&Minute),
//             reinterpret_cast<uint8_t *>(&Minute) + sizeof(Minute));
//     nav_bytes.insert(
//             nav_bytes.end(),
//             reinterpret_cast<uint8_t *>(&Second),
//             reinterpret_cast<uint8_t *>(&Second) + sizeof(Second));
//     nav_bytes.insert(
//             nav_bytes.end(),
//             reinterpret_cast<uint8_t *>(&Microseconds),
//             reinterpret_cast<uint8_t *>(&Microseconds) + sizeof(Microseconds));
//     nav_bytes.insert(
//             nav_bytes.end(),
//             reinterpret_cast<uint8_t *>(&SourceEpoch),
//             reinterpret_cast<uint8_t *>(&SourceEpoch) + sizeof(SourceEpoch));
//     nav_bytes.insert(
//             nav_bytes.end(),
//             reinterpret_cast<uint8_t *>(&TimeTag),
//             reinterpret_cast<uint8_t *>(&TimeTag) + sizeof(TimeTag));
//     nav_bytes.insert(
//             nav_bytes.end(),
//             reinterpret_cast<uint8_t *>(&Raw_Y_Coordinate),
//             reinterpret_cast<uint8_t *>(&Raw_Y_Coordinate) + sizeof(Raw_Y_Coordinate));
//     nav_bytes.insert(
//             nav_bytes.end(),
//             reinterpret_cast<uint8_t *>(&Raw_X_Coordinate),
//             reinterpret_cast<uint8_t *>(&Raw_X_Coordinate) + sizeof(Raw_X_Coordinate));
//     nav_bytes.insert(
//             nav_bytes.end(),
//             reinterpret_cast<uint8_t *>(&Raw_Altitude),
//             reinterpret_cast<uint8_t *>(&Raw_Altitude) + sizeof(Raw_Altitude));
//     nav_bytes.insert(
//             nav_bytes.end(),
//             reinterpret_cast<uint8_t *>(&TimeFlag),
//             reinterpret_cast<uint8_t *>(&TimeFlag) + sizeof(TimeFlag));
//     nav_bytes.insert(nav_bytes.end(), Reserved1, Reserved1 + sizeof(Reserved1));

//     if (nav_bytes.size() != 64) {
//         std::cerr << "Nav packet size mismatch: " << nav_bytes.size() << " != 64" << std::endl;
//         nav_bytes.resize(64, 0);
//     }

//     HeaderType                 = 3;
//     uint8_t SubChannelNumber   = 0;
//     uint16_t NumChansToFollow  = 0;
//     uint16_t Reserved1_Att[2]  = {0};
//     NumBytesThisRecord         = 64;
//     uint32_t Reserved2_Att[2]  = {0};
//     uint32_t EpochMicroSeconds = microseconds;
//     SourceEpoch                = unix_timestamp;
//     float Pitch                = static_cast<float>(pitch);
//     float Roll                 = static_cast<float>(roll);
//     float Heave                = -data.pose.pose.position.z;
//     float Yaw_Att              = 0.0f;
//     float Heading              = static_cast<float>(yaw);
//     uint16_t Milliseconds      = microseconds / 1'000;
//     uint8_t Reserved3          = 0;

//     alt_bytes.insert(
//             alt_bytes.end(),
//             reinterpret_cast<uint8_t *>(&MagicNumber),
//             reinterpret_cast<uint8_t *>(&MagicNumber) + sizeof(MagicNumber));
//     alt_bytes.insert(
//             alt_bytes.end(),
//             reinterpret_cast<uint8_t *>(&HeaderType),
//             reinterpret_cast<uint8_t *>(&HeaderType) + sizeof(HeaderType));
//     alt_bytes.insert(
//             alt_bytes.end(),
//             reinterpret_cast<uint8_t *>(&SubChannelNumber),
//             reinterpret_cast<uint8_t *>(&SubChannelNumber) + sizeof(SubChannelNumber));
//     alt_bytes.insert(
//             alt_bytes.end(),
//             reinterpret_cast<uint8_t *>(&NumChansToFollow),
//             reinterpret_cast<uint8_t *>(&NumChansToFollow) + sizeof(NumChansToFollow));
//     alt_bytes.insert(
//             alt_bytes.end(),
//             reinterpret_cast<uint8_t *>(Reserved1_Att),
//             reinterpret_cast<uint8_t *>(Reserved1_Att) + sizeof(Reserved1_Att));
//     alt_bytes.insert(
//             alt_bytes.end(),
//             reinterpret_cast<uint8_t *>(&NumBytesThisRecord),
//             reinterpret_cast<uint8_t *>(&NumBytesThisRecord) + sizeof(NumBytesThisRecord));
//     alt_bytes.insert(
//             alt_bytes.end(),
//             reinterpret_cast<uint8_t *>(Reserved2_Att),
//             reinterpret_cast<uint8_t *>(Reserved2_Att) + sizeof(Reserved2_Att));
//     alt_bytes.insert(
//             alt_bytes.end(),
//             reinterpret_cast<uint8_t *>(&EpochMicroSeconds),
//             reinterpret_cast<uint8_t *>(&EpochMicroSeconds) + sizeof(EpochMicroSeconds));
//     alt_bytes.insert(
//             alt_bytes.end(),
//             reinterpret_cast<uint8_t *>(&SourceEpoch),
//             reinterpret_cast<uint8_t *>(&SourceEpoch) + sizeof(SourceEpoch));
//     alt_bytes.insert(
//             alt_bytes.end(),
//             reinterpret_cast<uint8_t *>(&Pitch),
//             reinterpret_cast<uint8_t *>(&Pitch) + sizeof(Pitch));
//     alt_bytes.insert(
//             alt_bytes.end(),
//             reinterpret_cast<uint8_t *>(&Roll),
//             reinterpret_cast<uint8_t *>(&Roll) + sizeof(Roll));
//     alt_bytes.insert(
//             alt_bytes.end(),
//             reinterpret_cast<uint8_t *>(&Heave),
//             reinterpret_cast<uint8_t *>(&Heave) + sizeof(Heave));
//     alt_bytes.insert(
//             alt_bytes.end(),
//             reinterpret_cast<uint8_t *>(&Yaw_Att),
//             reinterpret_cast<uint8_t *>(&Yaw_Att) + sizeof(Yaw_Att));
//     alt_bytes.insert(
//             alt_bytes.end(),
//             reinterpret_cast<uint8_t *>(&TimeTag),
//             reinterpret_cast<uint8_t *>(&TimeTag) + sizeof(TimeTag));
//     alt_bytes.insert(
//             alt_bytes.end(),
//             reinterpret_cast<uint8_t *>(&Heading),
//             reinterpret_cast<uint8_t *>(&Heading) + sizeof(Heading));
//     alt_bytes.insert(
//             alt_bytes.end(),
//             reinterpret_cast<uint8_t *>(&Year),
//             reinterpret_cast<uint8_t *>(&Year) + sizeof(Year));
//     alt_bytes.insert(
//             alt_bytes.end(),
//             reinterpret_cast<uint8_t *>(&Month),
//             reinterpret_cast<uint8_t *>(&Month) + sizeof(Month));
//     alt_bytes.insert(
//             alt_bytes.end(),
//             reinterpret_cast<uint8_t *>(&Day),
//             reinterpret_cast<uint8_t *>(&Day) + sizeof(Day));
//     alt_bytes.insert(
//             alt_bytes.end(),
//             reinterpret_cast<uint8_t *>(&Hour),
//             reinterpret_cast<uint8_t *>(&Hour) + sizeof(Hour));
//     alt_bytes.insert(
//             alt_bytes.end(),
//             reinterpret_cast<uint8_t *>(&Minute),
//             reinterpret_cast<uint8_t *>(&Minute) + sizeof(Minute));
//     alt_bytes.insert(
//             alt_bytes.end(),
//             reinterpret_cast<uint8_t *>(&Second),
//             reinterpret_cast<uint8_t *>(&Second) + sizeof(Second));
//     alt_bytes.insert(
//             alt_bytes.end(),
//             reinterpret_cast<uint8_t *>(&Milliseconds),
//             reinterpret_cast<uint8_t *>(&Milliseconds) + sizeof(Milliseconds));
//     alt_bytes.insert(
//             alt_bytes.end(),
//             reinterpret_cast<uint8_t *>(&Reserved3),
//             reinterpret_cast<uint8_t *>(&Reserved3) + sizeof(Reserved3));

//     if (alt_bytes.size() != 64) {
//         std::cerr << "Attitude packet size mismatch: " << alt_bytes.size() << " != 64" << std::endl;
//         alt_bytes.resize(64, 0);
//     }

//     return {nav_bytes, alt_bytes};
// }

// // Prepare header file packet (unchanged)
// std::vector<uint8_t> BeexToXtfConverter::prepare_header_file_packet() {
//     std::vector<uint8_t> buffer;

//     uint8_t FileFormat                     = 123;
//     uint8_t SystemType                     = 1;
//     std::string RecordingProgramName       = params_.beex_file.substr(0, 8);
//     uint8_t RecordingProgramVersion[8]     = {1, 0, 0, 0, 0, 0, 0, 0};
//     std::string SonarName                  = params_.sonar_type.substr(0, 16);
//     uint16_t SensorsType                   = 0;
//     std::string NoteString                 = "";
//     std::string ThisFileName               = params_.save_file.substr(0, 64);
//     uint16_t NavUnits                      = 3;
//     uint16_t NumberofSonarChannels         = 0;
//     uint16_t NumberofBathymetryChannels    = 1;
//     uint8_t NumberofSnippetsChannels       = 0;
//     uint8_t NumberofForwardLookArrays      = 0;
//     uint16_t NumberofEchoStrengthChannels  = 0;
//     uint8_t NumberofInterferometryChannels = 0;

//     uint8_t Reserved1          = 0;
//     uint8_t Reserved2          = 0;
//     uint8_t Reserved3          = 0;
//     float ReferencePointHeight = 0.0f;

//     uint8_t ProjectionType[12] = {0};
//     uint8_t SpheroidType[10]   = {0};
//     int32_t NavigationLatency  = 0;

//     float OriginX        = 0.0;
//     float OriginY        = 0.0;
//     float NavOffsetY     = params_.nav_offset.y;
//     float NavOffsetX     = params_.nav_offset.x;
//     float NavOffsetZ     = params_.nav_offset.z;
//     float NavOffsetYaw   = -90.0f;
//     float MRUOffsetY     = 0.0f;
//     float MRUOffsetX     = 0.0f;
//     float MRUOffsetZ     = 0.0f;
//     float MRUOffsetYaw   = 0.0f;
//     float MRUOffsetPitch = 0.0f;
//     float MRUOffsetRoll  = 0.0f;

//     buffer.insert(
//             buffer.end(),
//             reinterpret_cast<uint8_t *>(&FileFormat),
//             reinterpret_cast<uint8_t *>(&FileFormat) + sizeof(FileFormat));
//     buffer.insert(
//             buffer.end(),
//             reinterpret_cast<uint8_t *>(&SystemType),
//             reinterpret_cast<uint8_t *>(&SystemType) + sizeof(SystemType));
//     buffer.insert(
//             buffer.end(),
//             repack(RecordingProgramName.c_str(), RecordingProgramName.size(), 8).begin(),
//             repack(RecordingProgramName.c_str(), RecordingProgramName.size(), 8).end());
//     buffer.insert(
//             buffer.end(),
//             repack(RecordingProgramVersion, sizeof(RecordingProgramVersion), 8).begin(),
//             repack(RecordingProgramVersion, sizeof(RecordingProgramVersion), 8).end());
//     buffer.insert(
//             buffer.end(),
//             repack(SonarName.c_str(), SonarName.size(), 16).begin(),
//             repack(SonarName.c_str(), SonarName.size(), 16).end());
//     buffer.insert(
//             buffer.end(),
//             reinterpret_cast<uint8_t *>(&SensorsType),
//             reinterpret_cast<uint8_t *>(&SensorsType) + sizeof(SensorsType));
//     buffer.insert(
//             buffer.end(),
//             repack(NoteString.c_str(), NoteString.size(), 64).begin(),
//             repack(NoteString.c_str(), NoteString.size(), 64).end());
//     buffer.insert(
//             buffer.end(),
//             repack(ThisFileName.c_str(), ThisFileName.size(), 64).begin(),
//             repack(ThisFileName.c_str(), ThisFileName.size(), 64).end());
//     buffer.insert(
//             buffer.end(),
//             reinterpret_cast<uint8_t *>(&NavUnits),
//             reinterpret_cast<uint8_t *>(&NavUnits) + sizeof(NavUnits));
//     buffer.insert(
//             buffer.end(),
//             reinterpret_cast<uint8_t *>(&NumberofSonarChannels),
//             reinterpret_cast<uint8_t *>(&NumberofSonarChannels) + sizeof(NumberofSonarChannels));
//     buffer.insert(
//             buffer.end(),
//             reinterpret_cast<uint8_t *>(&NumberofBathymetryChannels),
//             reinterpret_cast<uint8_t *>(&NumberofBathymetryChannels) + sizeof(NumberofBathymetryChannels));
//     buffer.insert(
//             buffer.end(),
//             reinterpret_cast<uint8_t *>(&NumberofSnippetsChannels),
//             reinterpret_cast<uint8_t *>(&NumberofSnippetsChannels) + sizeof(NumberofSnippetsChannels));
//     buffer.insert(
//             buffer.end(),
//             reinterpret_cast<uint8_t *>(&NumberofForwardLookArrays),
//             reinterpret_cast<uint8_t *>(&NumberofForwardLookArrays) + sizeof(NumberofForwardLookArrays));
//     buffer.insert(
//             buffer.end(),
//             reinterpret_cast<uint8_t *>(&NumberofEchoStrengthChannels),
//             reinterpret_cast<uint8_t *>(&NumberofEchoStrengthChannels) + sizeof(NumberofEchoStrengthChannels));
//     buffer.insert(
//             buffer.end(),
//             reinterpret_cast<uint8_t *>(&NumberofInterferometryChannels),
//             reinterpret_cast<uint8_t *>(&NumberofInterferometryChannels) +
// sizeof(NumberofInterferometryChannels));
//     buffer.insert(
//             buffer.end(),
//             reinterpret_cast<uint8_t *>(&Reserved1),
//             reinterpret_cast<uint8_t *>(&Reserved1) + sizeof(Reserved1));
//     buffer.insert(
//             buffer.end(),
//             reinterpret_cast<uint8_t *>(&Reserved2),
//             reinterpret_cast<uint8_t *>(&Reserved2) + sizeof(Reserved2));
//     buffer.insert(
//             buffer.end(),
//             reinterpret_cast<uint8_t *>(&Reserved3),
//             reinterpret_cast<uint8_t *>(&Reserved3) + sizeof(Reserved3));
//     buffer.insert(
//             buffer.end(),
//             reinterpret_cast<uint8_t *>(&ReferencePointHeight),
//             reinterpret_cast<uint8_t *>(&ReferencePointHeight) + sizeof(ReferencePointHeight));
//     buffer.insert(
//             buffer.end(),
//             repack(ProjectionType, sizeof(ProjectionType), 12).begin(),
//             repack(ProjectionType, sizeof(ProjectionType), 12).end());
//     buffer.insert(
//             buffer.end(),
//             repack(SpheroidType, sizeof(SpheroidType), 10).begin(),
//             repack(SpheroidType, sizeof(SpheroidType), 10).end());
//     buffer.insert(
//             buffer.end(),
//             reinterpret_cast<uint8_t *>(&NavigationLatency),
//             reinterpret_cast<uint8_t *>(&NavigationLatency) + sizeof(NavigationLatency));
//     buffer.insert(
//             buffer.end(),
//             reinterpret_cast<uint8_t *>(&OriginY),
//             reinterpret_cast<uint8_t *>(&OriginY) + sizeof(OriginY));
//     buffer.insert(
//             buffer.end(),
//             reinterpret_cast<uint8_t *>(&OriginX),
//             reinterpret_cast<uint8_t *>(&OriginX) + sizeof(OriginX));
//     buffer.insert(
//             buffer.end(),
//             reinterpret_cast<uint8_t *>(&NavOffsetY),
//             reinterpret_cast<uint8_t *>(&NavOffsetY) + sizeof(NavOffsetY));
//     buffer.insert(
//             buffer.end(),
//             reinterpret_cast<uint8_t *>(&NavOffsetX),
//             reinterpret_cast<uint8_t *>(&NavOffsetX) + sizeof(NavOffsetX));
//     buffer.insert(
//             buffer.end(),
//             reinterpret_cast<uint8_t *>(&NavOffsetZ),
//             reinterpret_cast<uint8_t *>(&NavOffsetZ) + sizeof(NavOffsetZ));
//     buffer.insert(
//             buffer.end(),
//             reinterpret_cast<uint8_t *>(&NavOffsetYaw),
//             reinterpret_cast<uint8_t *>(&NavOffsetYaw) + sizeof(NavOffsetYaw));
//     buffer.insert(
//             buffer.end(),
//             reinterpret_cast<uint8_t *>(&MRUOffsetY),
//             reinterpret_cast<uint8_t *>(&MRUOffsetY) + sizeof(MRUOffsetY));
//     buffer.insert(
//             buffer.end(),
//             reinterpret_cast<uint8_t *>(&MRUOffsetX),
//             reinterpret_cast<uint8_t *>(&MRUOffsetX) + sizeof(MRUOffsetX));
//     buffer.insert(
//             buffer.end(),
//             reinterpret_cast<uint8_t *>(&MRUOffsetZ),
//             reinterpret_cast<uint8_t *>(&MRUOffsetZ) + sizeof(MRUOffsetZ));
//     buffer.insert(
//             buffer.end(),
//             reinterpret_cast<uint8_t *>(&MRUOffsetYaw),
//             reinterpret_cast<uint8_t *>(&MRUOffsetYaw) + sizeof(MRUOffsetYaw));
//     buffer.insert(
//             buffer.end(),
//             reinterpret_cast<uint8_t *>(&MRUOffsetPitch),
//             reinterpret_cast<uint8_t *>(&MRUOffsetPitch) + sizeof(MRUOffsetPitch));
//     buffer.insert(
//             buffer.end(),
//             reinterpret_cast<uint8_t *>(&MRUOffsetRoll),
//             reinterpret_cast<uint8_t *>(&MRUOffsetRoll) + sizeof(MRUOffsetRoll));

//     buffer.resize(1'024, 0);
//     return buffer;
// }


// std::vector<uint8_t> BeexToXtfConverter::prepare_header_file_packet() {
//     std::vector<uint8_t> buffer;

//     // Header fields
//     uint8_t FileFormat               = 123;                             // Unsigned integer of 8 bits = 1 byte
//     uint8_t SystemType               = 53;                              // Updated to 53 for R2Sonic
//     std::string RecordingProgramName = params_.beex_file.substr(0, 8);  // Mandatory, padded to 8
//     char RecordingProgramVersion[8]  = "100";  // Fixed: 5 chars + null terminator = 6 bytes, fits in 8
//     std::string SonarName            = params_.sonar_type.substr(0, 16);  // Recommended
//     uint16_t SensorsType             = 0;                                 // Mandatory, default to 0
//     std::string NoteString   = "";  // cfm again. 64 bytes                                // Recommended, padded to
//     64 std::string ThisFileName = params_.save_file.substr(0, 64);  // Recommended, padded to 64 uint16_t NavUnits =
//     3;                                // Mandatory, 3 for Lat/Long uint16_t NumberofSonarChannels         = 0; //
//     Mandatory uint16_t NumberofBathymetryChannels    = 1;                  // Mandatory uint8_t
//     NumberofSnippetsChannels       = 0;                  // Mandatory uint8_t NumberofForwardLookArrays      = 0; //
//     Mandatory uint16_t NumberofEchoStrengthChannels  = 0;                  // Mandatory uint8_t
//     NumberofInterferometryChannels = 0;                  // Mandatory

//     uint8_t Reserved1          = 0;     // Unused
//     uint8_t Reserved2          = 0;     // Unused
//     uint8_t Reserved3          = 0;     // Unused
//     float ReferencePointHeight = 0.0f;  // Optional

//     uint8_t ProjectionType[12] = {0};  // Unused
//     uint8_t SpheroidType[10]   = {0};  // Unused
//     uint32_t NavigationLatency = 0;    // Optional
//     // params_.nav_offset.x       = 0.096f;
//     // params_.nav_offset.y       = -0.189f;
//     // params_.nav_offset.z       = 0.0f;
//     // params_.sonar_offset.x     = -0.012;
//     // params_.sonar_offset.y     = 0.146;
//     // params_.sonar_offset.z     = 0.568;

//     float OriginX        = 0.0;                   // Unused
//     float OriginY        = 0.0;                   // Unused
//     float NavOffsetY     = params_.nav_offset.y;  // Optional
//     float NavOffsetX     = params_.nav_offset.x;  // Optional
//     float NavOffsetZ     = params_.nav_offset.z;  // Optional
//     float NavOffsetYaw   = -90.0f;                // Optional
//     float MRUOffsetY     = 0.0f;                  // Optional
//     float MRUOffsetX     = 0.0f;                  // Optional
//     float MRUOffsetZ     = 0.0f;                  // Optional
//     float MRUOffsetYaw   = 0.0f;                  // Optional
//     float MRUOffsetPitch = 0.0f;                  // Optional
//     float MRUOffsetRoll  = 0.0f;                  // Optional
//     std::cout << "Buffer size after inserting: " << buffer.size() << std::endl;

//     // Insert header fields into buffer
//     buffer.insert(
//             buffer.end(),
//             reinterpret_cast<uint8_t *>(&FileFormat),
//             reinterpret_cast<uint8_t *>(&FileFormat) + sizeof(FileFormat));
//     std::cout << "Buffer size after inserting FileFormat: " << buffer.size() << std::endl;

//     buffer.insert(
//             buffer.end(),
//             reinterpret_cast<uint8_t *>(&SystemType),
//             reinterpret_cast<uint8_t *>(&SystemType) + sizeof(SystemType));
//     std::cout << "Buffer size after inserting SystemType: " << buffer.size() << std::endl;

//     buffer.insert(
//             buffer.end(),
//             repack(RecordingProgramName.c_str(), RecordingProgramName.size(), 8).begin(),
//             repack(RecordingProgramName.c_str(), RecordingProgramName.size(), 8).end());
//     std::cout << "Buffer size after inserting SystemType: " << buffer.size() << std::endl;

//     buffer.insert(
//             buffer.end(),
//             repack(RecordingProgramVersion, sizeof(RecordingProgramVersion), 8).begin(),
//             repack(RecordingProgramVersion, sizeof(RecordingProgramVersion), 8).end());
//     std::cout << "Buffer size after inserting SystemType: " << buffer.size() << std::endl;

//     buffer.insert(
//             buffer.end(),
//             repack(SonarName.c_str(), SonarName.size(), 8).begin(),
//             repack(SonarName.c_str(), SonarName.size(), 8).end());  /// sixteen
//     std::cout << "Buffer size after inserting SystemType: " << buffer.size() << std::endl;

//     std::vector<uint8_t> SensorsType_bytes = repack(&SensorsType, sizeof(SensorsType), 2);
//     buffer.insert(buffer.end(), SensorsType_bytes.begin(), SensorsType_bytes.begin());
//     std::cout << "Buffer size after inserting SystemType: " << buffer.size() << std::endl;

//     buffer.insert(
//             buffer.end(),
//             repack(NoteString.c_str(), NoteString.size(), 64).begin(),
//             repack(NoteString.c_str(), NoteString.size(), 64).end());
//     buffer.insert(
//             buffer.end(),
//             repack(ThisFileName.c_str(), ThisFileName.size(), 64).begin(),
//             repack(ThisFileName.c_str(), ThisFileName.size(), 64).end());
//     buffer.insert(
//             buffer.end(),
//             reinterpret_cast<uint8_t *>(&NavUnits),
//             reinterpret_cast<uint8_t *>(&NavUnits) + sizeof(NavUnits));
//     buffer.insert(
//             buffer.end(),
//             reinterpret_cast<uint8_t *>(&NumberofSonarChannels),
//             reinterpret_cast<uint8_t *>(&NumberofSonarChannels) + sizeof(NumberofSonarChannels));
//     buffer.insert(
//             buffer.end(),
//             reinterpret_cast<uint8_t *>(&NumberofBathymetryChannels),
//             reinterpret_cast<uint8_t *>(&NumberofBathymetryChannels) + sizeof(NumberofBathymetryChannels));
//     buffer.insert(
//             buffer.end(),
//             reinterpret_cast<uint8_t *>(&NumberofSnippetsChannels),
//             reinterpret_cast<uint8_t *>(&NumberofSnippetsChannels) + sizeof(NumberofSnippetsChannels));
//     buffer.insert(
//             buffer.end(),
//             reinterpret_cast<uint8_t *>(&NumberofForwardLookArrays),
//             reinterpret_cast<uint8_t *>(&NumberofForwardLookArrays) + sizeof(NumberofForwardLookArrays));
//     buffer.insert(
//             buffer.end(),
//             reinterpret_cast<uint8_t *>(&NumberofEchoStrengthChannels),
//             reinterpret_cast<uint8_t *>(&NumberofEchoStrengthChannels) + sizeof(NumberofEchoStrengthChannels));
//     buffer.insert(
//             buffer.end(),
//             reinterpret_cast<uint8_t *>(&NumberofInterferometryChannels),
//             reinterpret_cast<uint8_t *>(&NumberofInterferometryChannels) + sizeof(NumberofInterferometryChannels));
//     buffer.insert(
//             buffer.end(),
//             reinterpret_cast<uint8_t *>(&Reserved1),
//             reinterpret_cast<uint8_t *>(&Reserved1) + sizeof(Reserved1));
//     buffer.insert(
//             buffer.end(),
//             reinterpret_cast<uint8_t *>(&Reserved2),
//             reinterpret_cast<uint8_t *>(&Reserved2) + sizeof(Reserved2));
//     buffer.insert(
//             buffer.end(),
//             reinterpret_cast<uint8_t *>(&Reserved3),
//             reinterpret_cast<uint8_t *>(&Reserved3) + sizeof(Reserved3));
//     buffer.insert(
//             buffer.end(),
//             reinterpret_cast<uint8_t *>(&ReferencePointHeight),
//             reinterpret_cast<uint8_t *>(&ReferencePointHeight) + sizeof(ReferencePointHeight));
//     buffer.insert(
//             buffer.end(),
//             repack(ProjectionType, sizeof(ProjectionType), 12).begin(),
//             repack(ProjectionType, sizeof(ProjectionType), 12).end());
//     buffer.insert(
//             buffer.end(),
//             repack(SpheroidType, sizeof(SpheroidType), 10).begin(),
//             repack(SpheroidType, sizeof(SpheroidType), 10).end());
//     buffer.insert(
//             buffer.end(),
//             reinterpret_cast<uint8_t *>(&NavigationLatency),
//             reinterpret_cast<uint8_t *>(&NavigationLatency) + sizeof(NavigationLatency));
//     buffer.insert(
//             buffer.end(),
//             reinterpret_cast<uint8_t *>(&OriginY),
//             reinterpret_cast<uint8_t *>(&OriginY) + sizeof(OriginY));
//     buffer.insert(
//             buffer.end(),
//             reinterpret_cast<uint8_t *>(&OriginX),
//             reinterpret_cast<uint8_t *>(&OriginX) + sizeof(OriginX));
//     buffer.insert(
//             buffer.end(),
//             reinterpret_cast<uint8_t *>(&NavOffsetY),
//             reinterpret_cast<uint8_t *>(&NavOffsetY) + sizeof(NavOffsetY));
//     buffer.insert(
//             buffer.end(),
//             reinterpret_cast<uint8_t *>(&NavOffsetX),
//             reinterpret_cast<uint8_t *>(&NavOffsetX) + sizeof(NavOffsetX));
//     buffer.insert(
//             buffer.end(),
//             reinterpret_cast<uint8_t *>(&NavOffsetZ),
//             reinterpret_cast<uint8_t *>(&NavOffsetZ) + sizeof(NavOffsetZ));
//     buffer.insert(
//             buffer.end(),
//             reinterpret_cast<uint8_t *>(&NavOffsetYaw),
//             reinterpret_cast<uint8_t *>(&NavOffsetYaw) + sizeof(NavOffsetYaw));
//     buffer.insert(
//             buffer.end(),
//             reinterpret_cast<uint8_t *>(&MRUOffsetY),
//             reinterpret_cast<uint8_t *>(&MRUOffsetY) + sizeof(MRUOffsetY));
//     buffer.insert(
//             buffer.end(),
//             reinterpret_cast<uint8_t *>(&MRUOffsetX),
//             reinterpret_cast<uint8_t *>(&MRUOffsetX) + sizeof(MRUOffsetX));
//     buffer.insert(
//             buffer.end(),
//             reinterpret_cast<uint8_t *>(&MRUOffsetZ),
//             reinterpret_cast<uint8_t *>(&MRUOffsetZ) + sizeof(MRUOffsetZ));
//     buffer.insert(
//             buffer.end(),
//             reinterpret_cast<uint8_t *>(&MRUOffsetYaw),
//             reinterpret_cast<uint8_t *>(&MRUOffsetYaw) + sizeof(MRUOffsetYaw));
//     buffer.insert(
//             buffer.end(),
//             reinterpret_cast<uint8_t *>(&MRUOffsetPitch),
//             reinterpret_cast<uint8_t *>(&MRUOffsetPitch) + sizeof(MRUOffsetPitch));
//     buffer.insert(
//             buffer.end(),
//             reinterpret_cast<uint8_t *>(&MRUOffsetRoll),
//             reinterpret_cast<uint8_t *>(&MRUOffsetRoll) + sizeof(MRUOffsetRoll));

//     // Define CHANINFO structure
//     struct XTFChanInfo {
//         uint8_t TypeOfChannel    = 3;          // 1 byte (3 for bathymetry)
//         uint8_t SubChannelNumber = 1;          // 1 byte
//         uint16_t CorrectionFlags = 1;          // 2 bytes
//         uint16_t UniPolar        = 1;          // 2 bytes
//         uint16_t BytesPerSample  = 2;          // 2 bytes
//         uint32_t Reserved        = 0;          // 4 bytes
//         char ChannelName[16]     = "R2Sonic";  // 16 bytes
//         float VoltScale          = 5.0f;       // 4 bytes
//         float Frequency          = 0.0f;       // 4 bytes
//         float HorizBeamAngle     = 0.0f;       // 4 bytes
//         float TiltAngle          = 0.0f;       // 4 bytes
//         float BeamWidth          = 0.0f;       // 4 bytes
//         float OffsetX_pack;
//         float OffsetY_pack;
//         float OffsetZ_pack;
//         float OffsetYaw        = 0.0;
//         float OffsetPitch      = 0.0;
//         float OffsetRoll       = 0.0;
//         uint16_t BeamsPerArray = 0;
//         uint8_t SampleFormat   = 3;
//     };

//     // Add CHANINFO for the bathymetry channel
//     XTFChanInfo chanInfo;
//     chanInfo.OffsetX_pack = params_.sonar_offset.x;
//     chanInfo.OffsetY_pack = params_.sonar_offset.y;
//     chanInfo.OffsetZ_pack = params_.sonar_offset.z;
//     buffer.insert(
//             buffer.end(),
//             reinterpret_cast<uint8_t *>(&chanInfo),
//             reinterpret_cast<uint8_t *>(&chanInfo) + sizeof(chanInfo));

//     char ReservedArea2[53]                   = {0};  // 53 bytes
//     std::vector<uint8_t> reserved_area_bytes = repack(ReservedArea2, sizeof(ReservedArea2), 53);
//     buffer.insert(buffer.end(), reserved_area_bytes.begin(), reserved_area_bytes.end());

//     if (buffer.size() != 384) {
//         std::cout << "buffer bytes" << buffer.size() << std::endl;
//         throw std::runtime_error("Header size mismatch");
//     }

//     // Ensure total size is 1024 bytes
//     buffer.resize(1'024, 0);

//     return buffer;
// }

std::vector<uint8_t> BeexToXtfConverter::prepare_header_file_packet() {
    std::vector<uint8_t> buffer;

    // Header fields
    uint8_t FileFormat                     = 123;                               // 1 byte
    uint8_t SystemType                     = 53;                                // 1 byte
    std::string RecordingProgramName       = params_.beex_file.substr(0, 8);    // 8 bytes
    char RecordingProgramVersion[8]        = "100";                             // 8 bytes
    std::string SonarName                  = params_.sonar_type.substr(0, 16);  // 16 bytes
    uint16_t SensorsType                   = 0;                                 // 2 bytes
    std::string NoteString                 = "";                                // 64 bytes
    std::string ThisFileName               = params_.save_file.substr(0, 64);   // 64 bytes
    uint16_t NavUnits                      = 3;                                 // 2 bytes
    uint16_t NumberofSonarChannels         = 0;                                 // 2 bytes
    uint16_t NumberofBathymetryChannels    = 1;                                 // 2 bytes
    uint8_t NumberofSnippetsChannels       = 0;                                 // 1 byte
    uint8_t NumberofForwardLookArrays      = 0;                                 // 1 byte
    uint16_t NumberofEchoStrengthChannels  = 0;                                 // 2 bytes
    uint8_t NumberofInterferometryChannels = 0;                                 // 1 byte
    uint8_t Reserved1                      = 0;                                 // 1 byte
    uint8_t Reserved2                      = 0;                                 // 1 byte
    uint8_t Reserved3                      = 0;                                 // 1 byte
    float ReferencePointHeight             = 0.0f;                              // 4 bytes
    uint8_t ProjectionType[12]             = {0};                               // 12 bytes
    uint8_t SpheroidType[10]               = {0};                               // 10 bytes
    uint32_t NavigationLatency             = 0;                                 // 4 bytes
    float OriginX                          = 0.0;                               // 4 bytes
    float OriginY                          = 0.0;                               // 4 bytes
    float NavOffsetY                       = params_.nav_offset.y;              // 4 bytes
    float NavOffsetX                       = params_.nav_offset.x;              // 4 bytes
    float NavOffsetZ                       = params_.nav_offset.z;              // 4 bytes
    float NavOffsetYaw                     = -90.0f;                            // 4 bytes
    float MRUOffsetY                       = 0.0f;                              // 4 bytes
    float MRUOffsetX                       = 0.0f;                              // 4 bytes
    float MRUOffsetZ                       = 0.0f;                              // 4 bytes
    float MRUOffsetYaw                     = 0.0f;                              // 4 bytes
    float MRUOffsetPitch                   = 0.0f;                              // 4 bytes
    float MRUOffsetRoll                    = 0.0f;                              // 4 bytes

    std::cout << "Buffer size after inserting: " << buffer.size() << std::endl;

    // Insert header fields into buffer
    buffer.insert(
            buffer.end(),
            reinterpret_cast<uint8_t *>(&FileFormat),
            reinterpret_cast<uint8_t *>(&FileFormat) + sizeof(FileFormat));
    std::cout << "Buffer size after inserting FileFormat: " << buffer.size() << std::endl;

    buffer.insert(
            buffer.end(),
            reinterpret_cast<uint8_t *>(&SystemType),
            reinterpret_cast<uint8_t *>(&SystemType) + sizeof(SystemType));
    std::cout << "Buffer size after inserting SystemType: " << buffer.size() << std::endl;

    // Prepare RecordingProgramName (8 bytes)
    char recordingProgramNameArray[8] = {0};
    strncpy(recordingProgramNameArray, RecordingProgramName.c_str(), sizeof(recordingProgramNameArray));
    buffer.insert(
            buffer.end(),
            reinterpret_cast<uint8_t *>(recordingProgramNameArray),
            reinterpret_cast<uint8_t *>(recordingProgramNameArray) + sizeof(recordingProgramNameArray));
    std::cout << "Buffer size after inserting RecordingProgramName: " << buffer.size() << std::endl;

    // Insert RecordingProgramVersion (already 8 bytes)
    buffer.insert(
            buffer.end(),
            reinterpret_cast<uint8_t *>(RecordingProgramVersion),
            reinterpret_cast<uint8_t *>(RecordingProgramVersion) + sizeof(RecordingProgramVersion));
    std::cout << "Buffer size after inserting RecordingProgramVersion: " << buffer.size() << std::endl;

    // Prepare SonarName (16 bytes)
    char sonarNameArray[16] = {0};
    strncpy(sonarNameArray, SonarName.c_str(), sizeof(sonarNameArray));
    buffer.insert(
            buffer.end(),
            reinterpret_cast<uint8_t *>(sonarNameArray),
            reinterpret_cast<uint8_t *>(sonarNameArray) + sizeof(sonarNameArray));
    std::cout << "Buffer size after inserting SonarName: " << buffer.size() << std::endl;

    // Insert SensorsType (2 bytes)
    buffer.insert(
            buffer.end(),
            reinterpret_cast<uint8_t *>(&SensorsType),
            reinterpret_cast<uint8_t *>(&SensorsType) + sizeof(SensorsType));
    std::cout << "Buffer size after inserting SensorsType: " << buffer.size() << std::endl;

    // Prepare NoteString (64 bytes)
    char noteStringArray[64] = {0};
    strncpy(noteStringArray, NoteString.c_str(), sizeof(noteStringArray));
    buffer.insert(
            buffer.end(),
            reinterpret_cast<uint8_t *>(noteStringArray),
            reinterpret_cast<uint8_t *>(noteStringArray) + sizeof(noteStringArray));
    std::cout << "Buffer size after inserting NoteString: " << buffer.size() << std::endl;

    // Prepare ThisFileName (64 bytes)
    char thisFileNameArray[64] = {0};
    strncpy(thisFileNameArray, ThisFileName.c_str(), sizeof(thisFileNameArray));
    buffer.insert(
            buffer.end(),
            reinterpret_cast<uint8_t *>(thisFileNameArray),
            reinterpret_cast<uint8_t *>(thisFileNameArray) + sizeof(thisFileNameArray));
    std::cout << "Buffer size after inserting ThisFileName: " << buffer.size() << std::endl;

    buffer.insert(
            buffer.end(),
            reinterpret_cast<uint8_t *>(&NavUnits),
            reinterpret_cast<uint8_t *>(&NavUnits) + sizeof(NavUnits));
    buffer.insert(
            buffer.end(),
            reinterpret_cast<uint8_t *>(&NumberofSonarChannels),
            reinterpret_cast<uint8_t *>(&NumberofSonarChannels) + sizeof(NumberofSonarChannels));
    buffer.insert(
            buffer.end(),
            reinterpret_cast<uint8_t *>(&NumberofBathymetryChannels),
            reinterpret_cast<uint8_t *>(&NumberofBathymetryChannels) + sizeof(NumberofBathymetryChannels));
    buffer.insert(
            buffer.end(),
            reinterpret_cast<uint8_t *>(&NumberofSnippetsChannels),
            reinterpret_cast<uint8_t *>(&NumberofSnippetsChannels) + sizeof(NumberofSnippetsChannels));
    buffer.insert(
            buffer.end(),
            reinterpret_cast<uint8_t *>(&NumberofForwardLookArrays),
            reinterpret_cast<uint8_t *>(&NumberofForwardLookArrays) + sizeof(NumberofForwardLookArrays));
    buffer.insert(
            buffer.end(),
            reinterpret_cast<uint8_t *>(&NumberofEchoStrengthChannels),
            reinterpret_cast<uint8_t *>(&NumberofEchoStrengthChannels) + sizeof(NumberofEchoStrengthChannels));
    buffer.insert(
            buffer.end(),
            reinterpret_cast<uint8_t *>(&NumberofInterferometryChannels),
            reinterpret_cast<uint8_t *>(&NumberofInterferometryChannels) + sizeof(NumberofInterferometryChannels));
    buffer.insert(
            buffer.end(),
            reinterpret_cast<uint8_t *>(&Reserved1),
            reinterpret_cast<uint8_t *>(&Reserved1) + sizeof(Reserved1));
    buffer.insert(
            buffer.end(),
            reinterpret_cast<uint8_t *>(&Reserved2),
            reinterpret_cast<uint8_t *>(&Reserved2) + sizeof(Reserved2));
    buffer.insert(
            buffer.end(),
            reinterpret_cast<uint8_t *>(&Reserved3),
            reinterpret_cast<uint8_t *>(&Reserved3) + sizeof(Reserved3));
    buffer.insert(
            buffer.end(),
            reinterpret_cast<uint8_t *>(&ReferencePointHeight),
            reinterpret_cast<uint8_t *>(&ReferencePointHeight) + sizeof(ReferencePointHeight));
    std::cout << "Buffer size after inserting up to ReferencePointHeight: " << buffer.size() << std::endl;

    // Insert ProjectionType (12 bytes)
    buffer.insert(
            buffer.end(),
            reinterpret_cast<uint8_t *>(ProjectionType),
            reinterpret_cast<uint8_t *>(ProjectionType) + sizeof(ProjectionType));
    std::cout << "Buffer size after inserting ProjectionType: " << buffer.size() << std::endl;

    // Insert SpheroidType (10 bytes)
    buffer.insert(
            buffer.end(),
            reinterpret_cast<uint8_t *>(SpheroidType),
            reinterpret_cast<uint8_t *>(SpheroidType) + sizeof(SpheroidType));
    std::cout << "Buffer size after inserting SpheroidType: " << buffer.size() << std::endl;

    buffer.insert(
            buffer.end(),
            reinterpret_cast<uint8_t *>(&NavigationLatency),
            reinterpret_cast<uint8_t *>(&NavigationLatency) + sizeof(NavigationLatency));
    buffer.insert(
            buffer.end(),
            reinterpret_cast<uint8_t *>(&OriginY),
            reinterpret_cast<uint8_t *>(&OriginY) + sizeof(OriginY));
    buffer.insert(
            buffer.end(),
            reinterpret_cast<uint8_t *>(&OriginX),
            reinterpret_cast<uint8_t *>(&OriginX) + sizeof(OriginX));
    buffer.insert(
            buffer.end(),
            reinterpret_cast<uint8_t *>(&NavOffsetY),
            reinterpret_cast<uint8_t *>(&NavOffsetX) + sizeof(NavOffsetX));
    buffer.insert(
            buffer.end(),
            reinterpret_cast<uint8_t *>(&NavOffsetZ),
            reinterpret_cast<uint8_t *>(&NavOffsetZ) + sizeof(NavOffsetZ));
    buffer.insert(
            buffer.end(),
            reinterpret_cast<uint8_t *>(&NavOffsetYaw),
            reinterpret_cast<uint8_t *>(&NavOffsetYaw) + sizeof(NavOffsetYaw));
    buffer.insert(
            buffer.end(),
            reinterpret_cast<uint8_t *>(&MRUOffsetY),
            reinterpret_cast<uint8_t *>(&MRUOffsetY) + sizeof(MRUOffsetY));
    buffer.insert(
            buffer.end(),
            reinterpret_cast<uint8_t *>(&MRUOffsetX),
            reinterpret_cast<uint8_t *>(&MRUOffsetX) + sizeof(MRUOffsetX));
    buffer.insert(
            buffer.end(),
            reinterpret_cast<uint8_t *>(&MRUOffsetZ),
            reinterpret_cast<uint8_t *>(&MRUOffsetZ) + sizeof(MRUOffsetZ));
    buffer.insert(
            buffer.end(),
            reinterpret_cast<uint8_t *>(&MRUOffsetYaw),
            reinterpret_cast<uint8_t *>(&MRUOffsetYaw) + sizeof(MRUOffsetYaw));
    buffer.insert(
            buffer.end(),
            reinterpret_cast<uint8_t *>(&MRUOffsetPitch),
            reinterpret_cast<uint8_t *>(&MRUOffsetPitch) + sizeof(MRUOffsetPitch));
    buffer.insert(
            buffer.end(),
            reinterpret_cast<uint8_t *>(&MRUOffsetRoll),
            reinterpret_cast<uint8_t *>(&MRUOffsetRoll) + sizeof(MRUOffsetRoll));
    std::cout << "Buffer size after inserting up to MRUOffsetRoll: " << buffer.size() << std::endl;

    // Define CHANINFO structure
    struct XTFChanInfo {
        uint8_t TypeOfChannel    = 3;          // 1 byte (3 for bathymetry) 0
        uint8_t SubChannelNumber = 1;          // 1 byte  1
        uint16_t CorrectionFlags = 1;          // 2 bytes  2
        uint16_t UniPolar        = 1;          // 2 bytes  4
        uint16_t BytesPerSample  = 2;          // 2 bytes  6
        uint32_t Reserved        = 0;          // 4 bytes  8
        char ChannelName[16]     = "R2Sonic";  // 16 bytes  12
        float VoltScale          = 5.0f;       // 4 bytes  28
        float Frequency          = 0.0f;       // 4 bytes  32
        float HorizBeamAngle     = 0.0f;       // 4 bytes  36
        float TiltAngle          = 0.0f;       // 4 bytes  40
        float BeamWidth          = 0.0f;       // 4 bytes  44
        float OffsetX_pack;                    // 48
        float OffsetY_pack;                    // 52
        float OffsetZ_pack;                    // 56
        float OffsetYaw        = 0.0;          // 60
        float OffsetPitch      = 0.0;          // 64
        float OffsetRoll       = 0.0;          // 68
        uint16_t BeamsPerArray = 0;            // 72
        uint8_t SampleFormat   = 3;            // 74 // 75
    } __attribute__((packed));

    // Add CHANINFO for the bathymetry channel
    XTFChanInfo chanInfo;
    chanInfo.OffsetX_pack = params_.sonar_offset.x;
    chanInfo.OffsetY_pack = params_.sonar_offset.y;
    chanInfo.OffsetZ_pack = params_.sonar_offset.z;
    buffer.insert(
            buffer.end(),
            reinterpret_cast<uint8_t *>(&chanInfo),
            reinterpret_cast<uint8_t *>(&chanInfo) + sizeof(chanInfo));
    std::cout << "Buffer size after inserting XTFChanInfo: " << buffer.size() << std::endl;

    // Insert ReservedArea2 (53 bytes)
    char ReservedArea2[53] = {0};
    buffer.insert(
            buffer.end(),
            reinterpret_cast<uint8_t *>(ReservedArea2),
            reinterpret_cast<uint8_t *>(ReservedArea2) + sizeof(ReservedArea2));
    std::cout << "Buffer size after inserting ReservedArea2: " << buffer.size() << std::endl;

    // Check buffer size
    if (buffer.size() != 384) {  // Updated expected size, as calculated below
        std::cout << "buffer bytes: " << buffer.size() << std::endl;
        throw std::runtime_error("Header size mismatch");
    }

    // Ensure total size is 1024 bytes
    buffer.resize(1'024, 0);
    std::cout << "Final buffer size: " << buffer.size() << std::endl;

    return buffer;
}


// Prepare multibeam bytes (reserve capacity)
std::vector<uint8_t> BeexToXtfConverter::prepare_multibeam_bytes(
        const bx_msgs::SurveyorInfoPayload &data,
        int ping_number) {
    // Calculate total size: header (256 bytes) + beams (64 bytes each)
    size_t total_size = 256 + data.beam_ranges.size() * 64;
    std::vector<uint8_t> buffer(total_size, 0);  // Pre-allocate exact size
    uint8_t *ptr = buffer.data();                // Direct pointer access

    std::cout << "Beam ranges size: " << data.beam_ranges.size() << ", bathy_twoway size: " << data.bathy_twoway.size()
              << std::endl;
    std::cout << "Buffer allocated, size: " << buffer.size() << ", capacity: " << buffer.capacity() << std::endl;

    time_t unix_timestamp = static_cast<time_t>(data.unix_time_ms / 1'000);
    struct tm *dt         = gmtime(&unix_timestamp);
    std::cout << "sonar type " << params_.sonar_type.substr(0, 16) << std::endl;

    // Header fields
    uint16_t MagicNumber         = 64'206;  // Corrected to match XTF spec
    uint8_t HeaderType           = 28;
    uint8_t SubChannelNumber     = 0;
    uint16_t NumberChansToFollow = 0;
    uint16_t Reserved1[2]        = {0};  // 4 bytes total
    uint32_t NumBytesThisRecord  = static_cast<uint32_t>(total_size);
    uint16_t Year                = dt->tm_year + 1'900;
    uint8_t Month                = dt->tm_mon + 1;
    uint8_t Day                  = dt->tm_mday;
    uint8_t Hour                 = dt->tm_hour;
    uint8_t Minute               = dt->tm_min;
    uint8_t Second               = dt->tm_sec;
    uint8_t HSeconds             = (data.unix_time_ms % 1'000) / 10;
    uint16_t JulianDay           = 0;
    uint32_t EventNumber         = 0;
    uint32_t PingNumber          = ping_number;

    float SoundVelocity         = 1542.0f;
    float OceanTide             = 0.0f;
    uint32_t Reserved2          = 0;
    float ConductivityFreq      = 0.0f;
    float TemperatureFreq       = 0.0f;
    float PressureFreq          = 0.0f;
    float PressureTemp          = 0.0f;
    float Conductivity          = 0.0f;
    float WaterTemperature      = 0.0f;
    float Pressure              = 0.0f;
    float ComputerSoundVelocity = 0.0f;
    float MagX = 0.0f, MagY = 0.0f, MagZ = 0.0f;
    float AuxVal1 = 0.0f, AuxVal2 = 0.0f, AuxVal3 = 0.0f;
    float Reserved3 = 0.0f, Reserved4 = 0.0f, Reserved5 = 0.0f;
    float SpeedLog = 0.0f, Turbidity = 0.0f;
    float ShipSpeed = 0.0f, ShipGyro = 0.0f;
    double ShipYCoordinate = 0.0, ShipXCoordinate = 0.0;
    uint16_t ShipAltitude = 0, ShipDepth = 0;
    uint8_t FixTimeHour = 0, FixTimeMinute = 0, FixTimeSecond = 0, FixTimeHSecond = 0;
    float SensorSpeed = 0.0f, KP = 0.0f;
    double SensorYCoordinate = 0.0, SensorXCoordinate = 0.0;
    uint16_t SonarStatus = 0, RangeToFish = 0, BearingToFish = 0, CableOut = 0;
    float Layback = 0.0f, CableTension = 0.0f;
    float SensorDepth = 0.0f, SensorPrimaryAltitude = 0.0f, SensorAuxAltitude = 0.0f;
    float SensorPitch = 0.0f, SensorRoll = 0.0f, SensorHeading = 0.0f;
    float Heave = 0.0f, Yaw = 0.0f;
    uint32_t AttitudeTimeTag = 0, NavFixMilliseconds = 0;
    float DOT                 = 0.0f;
    uint8_t ComputerClockHour = 0, ComputerClockMinute = 0, ComputerClockSecond = 0, ComputerClockHsec = 0;
    int16_t FishPositionDeltaX = 0, FishPositionDeltaY = 0;
    uint8_t FishPositionErrorCode = 0;
    uint32_t OptionalOffset       = 0;
    uint8_t CableOutHundredths    = 0;
    uint8_t ReservedSpace2[6]     = {0};  // 6 bytes


    // Pack header (256 bytes total)
    memcpy(ptr, &MagicNumber, sizeof(MagicNumber));
    ptr += sizeof(MagicNumber);
    memcpy(ptr, &HeaderType, sizeof(HeaderType));
    ptr += sizeof(HeaderType);
    memcpy(ptr, &SubChannelNumber, sizeof(SubChannelNumber));
    ptr += sizeof(SubChannelNumber);
    memcpy(ptr, &NumberChansToFollow, sizeof(NumberChansToFollow));
    ptr += sizeof(NumberChansToFollow);
    // Use repack for field
    std::vector<uint8_t> Reserved1_bytes = repack(Reserved1, sizeof(Reserved1), 4);
    memcpy(ptr, Reserved1_bytes.data(), Reserved1_bytes.size());
    ptr += Reserved1_bytes.size();
    // memcpy(ptr, Reserved1, sizeof(Reserved1));
    // ptr += sizeof(Reserved1);
    // Use repack for field
    std::vector<uint8_t> NumBytesThisRecord_bytes = repack(&NumBytesThisRecord, sizeof(NumBytesThisRecord), 4);
    memcpy(ptr, NumBytesThisRecord_bytes.data(), NumBytesThisRecord_bytes.size());
    ptr += NumBytesThisRecord_bytes.size();
    memcpy(ptr, &Year, sizeof(Year));
    ptr += sizeof(Year);
    memcpy(ptr, &Month, sizeof(Month));
    ptr += sizeof(Month);
    memcpy(ptr, &Day, sizeof(Day));
    ptr += sizeof(Day);
    memcpy(ptr, &Hour, sizeof(Hour));
    ptr += sizeof(Hour);
    memcpy(ptr, &Minute, sizeof(Minute));
    ptr += sizeof(Minute);
    memcpy(ptr, &Second, sizeof(Second));
    ptr += sizeof(Second);
    memcpy(ptr, &HSeconds, sizeof(HSeconds));
    ptr += sizeof(HSeconds);
    memcpy(ptr, &JulianDay, sizeof(JulianDay));
    ptr += sizeof(JulianDay);
    memcpy(ptr, &EventNumber, sizeof(EventNumber));
    ptr += sizeof(EventNumber);
    memcpy(ptr, &PingNumber, sizeof(PingNumber));
    ptr += sizeof(PingNumber);
    memcpy(ptr, &SoundVelocity, sizeof(SoundVelocity));
    ptr += sizeof(SoundVelocity);
    memcpy(ptr, &OceanTide, sizeof(OceanTide));
    ptr += sizeof(OceanTide);
    memcpy(ptr, &Reserved2, sizeof(Reserved2));
    ptr += sizeof(Reserved2);
    memcpy(ptr, &ConductivityFreq, sizeof(ConductivityFreq));
    ptr += sizeof(ConductivityFreq);
    memcpy(ptr, &TemperatureFreq, sizeof(TemperatureFreq));
    ptr += sizeof(TemperatureFreq);
    memcpy(ptr, &PressureFreq, sizeof(PressureFreq));
    ptr += sizeof(PressureFreq);
    memcpy(ptr, &PressureTemp, sizeof(PressureTemp));
    ptr += sizeof(PressureTemp);
    memcpy(ptr, &Conductivity, sizeof(Conductivity));
    ptr += sizeof(Conductivity);
    memcpy(ptr, &WaterTemperature, sizeof(WaterTemperature));
    ptr += sizeof(WaterTemperature);
    memcpy(ptr, &Pressure, sizeof(Pressure));
    ptr += sizeof(Pressure);
    memcpy(ptr, &ComputerSoundVelocity, sizeof(ComputerSoundVelocity));
    ptr += sizeof(ComputerSoundVelocity);
    memcpy(ptr, &MagX, sizeof(MagX));
    ptr += sizeof(MagX);
    memcpy(ptr, &MagY, sizeof(MagY));
    ptr += sizeof(MagY);
    memcpy(ptr, &MagZ, sizeof(MagZ));
    ptr += sizeof(MagZ);
    memcpy(ptr, &AuxVal1, sizeof(AuxVal1));
    ptr += sizeof(AuxVal1);
    memcpy(ptr, &AuxVal2, sizeof(AuxVal2));
    ptr += sizeof(AuxVal2);
    memcpy(ptr, &AuxVal3, sizeof(AuxVal3));
    ptr += sizeof(AuxVal3);
    memcpy(ptr, &Reserved3, sizeof(Reserved3));
    ptr += sizeof(Reserved3);
    memcpy(ptr, &Reserved4, sizeof(Reserved4));
    ptr += sizeof(Reserved4);
    memcpy(ptr, &Reserved5, sizeof(Reserved5));
    ptr += sizeof(Reserved5);
    memcpy(ptr, &SpeedLog, sizeof(SpeedLog));
    ptr += sizeof(SpeedLog);
    memcpy(ptr, &Turbidity, sizeof(Turbidity));
    ptr += sizeof(Turbidity);
    memcpy(ptr, &ShipSpeed, sizeof(ShipSpeed));
    ptr += sizeof(ShipSpeed);
    memcpy(ptr, &ShipGyro, sizeof(ShipGyro));
    ptr += sizeof(ShipGyro);
    memcpy(ptr, &ShipYCoordinate, sizeof(ShipYCoordinate));
    ptr += sizeof(ShipYCoordinate);
    memcpy(ptr, &ShipXCoordinate, sizeof(ShipXCoordinate));
    ptr += sizeof(ShipXCoordinate);
    memcpy(ptr, &ShipAltitude, sizeof(ShipAltitude));
    ptr += sizeof(ShipAltitude);
    memcpy(ptr, &ShipDepth, sizeof(ShipDepth));
    ptr += sizeof(ShipDepth);
    memcpy(ptr, &FixTimeHour, sizeof(FixTimeHour));
    ptr += sizeof(FixTimeHour);
    memcpy(ptr, &FixTimeMinute, sizeof(FixTimeMinute));
    ptr += sizeof(FixTimeMinute);
    memcpy(ptr, &FixTimeSecond, sizeof(FixTimeSecond));
    ptr += sizeof(FixTimeSecond);
    memcpy(ptr, &FixTimeHSecond, sizeof(FixTimeHSecond));
    ptr += sizeof(FixTimeHSecond);
    memcpy(ptr, &SensorSpeed, sizeof(SensorSpeed));
    ptr += sizeof(SensorSpeed);
    memcpy(ptr, &KP, sizeof(KP));
    ptr += sizeof(KP);
    memcpy(ptr, &SensorYCoordinate, sizeof(SensorYCoordinate));
    ptr += sizeof(SensorYCoordinate);
    memcpy(ptr, &SensorXCoordinate, sizeof(SensorXCoordinate));
    ptr += sizeof(SensorXCoordinate);
    memcpy(ptr, &SonarStatus, sizeof(SonarStatus));
    ptr += sizeof(SonarStatus);
    memcpy(ptr, &RangeToFish, sizeof(RangeToFish));
    ptr += sizeof(RangeToFish);
    memcpy(ptr, &BearingToFish, sizeof(BearingToFish));
    ptr += sizeof(BearingToFish);
    memcpy(ptr, &CableOut, sizeof(CableOut));
    ptr += sizeof(CableOut);
    memcpy(ptr, &Layback, sizeof(Layback));
    ptr += sizeof(Layback);
    memcpy(ptr, &CableTension, sizeof(CableTension));
    ptr += sizeof(CableTension);
    memcpy(ptr, &SensorDepth, sizeof(SensorDepth));
    ptr += sizeof(SensorDepth);
    memcpy(ptr, &SensorPrimaryAltitude, sizeof(SensorPrimaryAltitude));
    ptr += sizeof(SensorPrimaryAltitude);
    memcpy(ptr, &SensorAuxAltitude, sizeof(SensorAuxAltitude));
    ptr += sizeof(SensorAuxAltitude);
    memcpy(ptr, &SensorPitch, sizeof(SensorPitch));
    ptr += sizeof(SensorPitch);
    memcpy(ptr, &SensorRoll, sizeof(SensorRoll));
    ptr += sizeof(SensorRoll);
    memcpy(ptr, &SensorHeading, sizeof(SensorHeading));
    ptr += sizeof(SensorHeading);
    memcpy(ptr, &Heave, sizeof(Heave));
    ptr += sizeof(Heave);
    memcpy(ptr, &Yaw, sizeof(Yaw));
    ptr += sizeof(Yaw);
    memcpy(ptr, &AttitudeTimeTag, sizeof(AttitudeTimeTag));
    ptr += sizeof(AttitudeTimeTag);
    memcpy(ptr, &DOT, sizeof(DOT));
    ptr += sizeof(DOT);
    memcpy(ptr, &NavFixMilliseconds, sizeof(NavFixMilliseconds));
    ptr += sizeof(NavFixMilliseconds);
    memcpy(ptr, &ComputerClockHour, sizeof(ComputerClockHour));
    ptr += sizeof(ComputerClockHour);
    memcpy(ptr, &ComputerClockMinute, sizeof(ComputerClockMinute));
    ptr += sizeof(ComputerClockMinute);
    memcpy(ptr, &ComputerClockSecond, sizeof(ComputerClockSecond));
    ptr += sizeof(ComputerClockSecond);
    memcpy(ptr, &ComputerClockHsec, sizeof(ComputerClockHsec));
    ptr += sizeof(ComputerClockHsec);
    memcpy(ptr, &FishPositionDeltaX, sizeof(FishPositionDeltaX));
    ptr += sizeof(FishPositionDeltaX);
    memcpy(ptr, &FishPositionDeltaY, sizeof(FishPositionDeltaY));
    ptr += sizeof(FishPositionDeltaY);
    memcpy(ptr, &FishPositionErrorCode, sizeof(FishPositionErrorCode));
    ptr += sizeof(FishPositionErrorCode);
    memcpy(ptr, &OptionalOffset, sizeof(OptionalOffset));
    ptr += sizeof(OptionalOffset);
    memcpy(ptr, &CableOutHundredths, sizeof(CableOutHundredths));
    ptr += sizeof(CableOutHundredths);
    memcpy(ptr, ReservedSpace2, sizeof(ReservedSpace2));
    ptr += sizeof(ReservedSpace2);

    std::cout << "Header packed, offset: " << (ptr - buffer.data()) << " (should be 256)" << std::endl;
    if (ptr - buffer.data() != 256) {
        std::cerr << "Error: Header size mismatch: " << (ptr - buffer.data()) << " != 256" << std::endl;
        throw std::runtime_error("Header size mismatch");
    }

    // Pack beams
    for (size_t i = 0; i < data.bathy_twoway.size(); ++i) {
        auto beam_bytes = prepare_beam_bytes(data, i);
        std::cout << 'a' << std::endl;
        memcpy(ptr, beam_bytes.data(), beam_bytes.size());
        ptr += beam_bytes.size();
        std::cout << "Beam " << i << " packed, offset: " << (ptr - buffer.data()) << std::endl;
    }

    std::cout << "Final buffer size: " << (ptr - buffer.data()) << ", expected: " << total_size << std::endl;
    if (ptr - buffer.data() != total_size) {
        std::cerr << "Error: Final size mismatch: " << (ptr - buffer.data()) << " != " << total_size << std::endl;
        throw std::runtime_error("Final size mismatch");
    }

    return buffer;
}

// Save current file (append mode)
void BeexToXtfConverter::save_current_file() {
    std::ofstream file(params_.save_file, std::ios::binary | std::ios::app);
    if (!file.is_open()) {
        std::cerr << "Failed to open file for writing: " << params_.save_file << std::endl;
        return;
    }
    for (const auto &bytes : save_file_byte_array_) {
        file.write(reinterpret_cast<const char *>(bytes.data()), bytes.size());
    }
    file.close();
    std::cout << "Saved " << save_file_byte_array_.size() << " packets to " << params_.save_file << std::endl;
}

// // Updated run() to save after every message
// void BeexToXtfConverter::run() {
//     std::cout << "Starting Conversion..." << std::endl;

//     // Write initial header
//     auto header_bytes = prepare_header_file_packet();
//     save_file_byte_array_.push_back(header_bytes);
//     save_file_current_size_ += header_bytes.size();
//     save_current_file();
//     save_file_byte_array_.clear();
//     save_file_current_size_ = 0;

//     rosbag::Bag bag;
//     try {
//         bag.open(params_.beex_file, rosbag::bagmode::Read);
//     } catch (const rosbag::BagException &e) {
//         std::cerr << "Error opening bag file: " << e.what() << std::endl;
//         return;
//     }
//     std::cout << "Bag opened..." << std::endl;

//     std::vector<std::string> topics = {MULTIBEAM_TOPIC, VEHICLE_LATLON};
//     rosbag::View view(bag, rosbag::TopicQuery(topics));
//     int ping_number = 1;
//     std::cout << "Processing bag msg..." << std::endl;

//     for (const rosbag::MessageInstance &m : view) {
//         if (m.getTopic() == MULTIBEAM_TOPIC) {
//             bx_msgs::SurveyorInfoPayload::ConstPtr msg = m.instantiate<bx_msgs::SurveyorInfoPayload>();
//             if (msg) {
//                 std::cout << "Processing multibeam msg..." << std::endl;

//                 auto multibeam_bytes = prepare_multibeam_bytes(*msg, ping_number++);
//                 std::cout << "prepped multibeam msg..." << std::endl;

//                 save_file_byte_array_.push_back(multibeam_bytes);
//                 std::cout << "appending multibeam msg..." << std::endl;

//                 save_file_current_size_ += multibeam_bytes.size();
//                 std::cout << "inc multibeam list size..." << std::endl;

//                 // Save and clear immediately
//                 save_current_file();
//                 save_file_byte_array_.clear();
//                 save_file_current_size_ = 0;
//             }
//         } else if (m.getTopic() == VEHICLE_LATLON) {
//             bx_msgs::VehicleState::ConstPtr msg = m.instantiate<bx_msgs::VehicleState>();
//             if (msg) {
//                 std::cout << "Processing odom msg..." << std::endl;

//                 auto [nav_bytes, alt_bytes] = prepare_nav_file_packet(*msg);
//                 save_file_byte_array_.push_back(nav_bytes);
//                 save_file_byte_array_.push_back(alt_bytes);
//                 save_file_current_size_ += nav_bytes.size() + alt_bytes.size();
//                 std::cout << "inc nav list size..." << std::endl;

//                 // Save and clear immediately
//                 save_current_file();
//                 save_file_byte_array_.clear();
//                 save_file_current_size_ = 0;
//             }
//         }
//     }

//     // Final save (if anything remains)
//     if (!save_file_byte_array_.empty()) {
//         save_current_file();
//     }
//     bag.close();
//     std::cout << "Conversion completed." << std::endl;
// }

void BeexToXtfConverter::run() {
    std::cout << "Starting Conversion..." << std::endl;

    auto header_bytes = prepare_header_file_packet();
    save_file_byte_array_.push_back(header_bytes);
    save_file_current_size_ += header_bytes.size();

    rosbag::Bag bag;
    try {
        bag.open(params_.beex_file, rosbag::bagmode::Read);
    } catch (const rosbag::BagException &e) {
        std::cerr << "Error opening bag file: " << e.what() << std::endl;
        return;
    }
    std::cout << "Bag opened..." << std::endl;

    std::vector<std::string> topics = {MULTIBEAM_TOPIC, VEHICLE_LATLON};
    rosbag::View view(bag, rosbag::TopicQuery(topics));
    int ping_number = 1;
    std::cout << "Processing bag msg..." << std::endl;

    for (const rosbag::MessageInstance &m : view) {
        if (m.getTopic() == MULTIBEAM_TOPIC) {
            bx_msgs::SurveyorInfoPayload::ConstPtr msg = m.instantiate<bx_msgs::SurveyorInfoPayload>();
            if (msg) {
                std::cout << "Processing multibeam msg..." << std::endl;

                // auto multibeam_bytes = prepare_multibeam_bytes(*msg, ping_number++);
                // std::cout << "prepped multibeam msg..." << std::endl;

                // save_file_byte_array_.push_back(multibeam_bytes);
                // std::cout << "appending mutlibeam msg..." << std::endl;

                // save_file_current_size_ += multibeam_bytes.size();
                // std::cout << "inc multibeam list size..." << std::endl;
            }
        } else if (m.getTopic() == VEHICLE_LATLON) {
            bx_msgs::VehicleState::ConstPtr msg = m.instantiate<bx_msgs::VehicleState>();
            if (msg) {
                std::cout << "Processing odom msg..." << std::endl;

                // auto [nav_bytes, alt_bytes] = prepare_nav_file_packet(*msg);
                // save_file_byte_array_.push_back(nav_bytes);
                // save_file_byte_array_.push_back(alt_bytes);
                // save_file_current_size_ += nav_bytes.size() + alt_bytes.size();
                // std::cout << "inc nav list size..." << std::endl;
            }
        }

        if (save_file_current_size_ >= SAVE_FILE_MAX_SIZE) {
            save_current_file();
            file_counter_++;
            params_.save_file = base_save_file_ + "_" + std::to_string(file_counter_) + ".xtf";
            save_file_byte_array_.clear();
            save_file_current_size_ = 0;

            auto new_header = prepare_header_file_packet();
            save_file_byte_array_.push_back(new_header);
            save_file_current_size_ += new_header.size();
        }
    }

    save_current_file();
    bag.close();
}

int main(int argc, char *argv[]) {
    if (argc != 3) {
        std::cerr << "Usage: " << argv[0] << " <input_beex_file> <output_xtf_file>" << std::endl;
        return 1;
    }
    BeexToXtfConverter converter(argv[1], argv[2]);
    converter.run();
    return 0;
}



// // Intially have issue with using buffer. If switch back to old method of saving everything in the the bytes array
// then
// // 1-shot save in xtf file, it might work //
// #include "beex_to_xtf_converter.h"

// // Static member definitions
// const std::string BeexToXtfConverter::MULTIBEAM_TOPIC  = "/ikan/profiling_sonar/data";
// const std::string BeexToXtfConverter::SONAR_INFO_TOPIC = "/ikan/profiling_sonar/info";
// const std::string BeexToXtfConverter::VEHICLE_LATLON   = "/ikan/nav/world_ned_msl";
// const uint64_t BeexToXtfConverter::SAVE_FILE_MAX_SIZE  = 1'024ULL * 1'024ULL * 1'024ULL;

// // Constructor (unchanged)
// BeexToXtfConverter::BeexToXtfConverter(
//         const std::string &beex_file,
//         const std::string &save_file,
//         float orientation,
//         const std::string &sonar_type)
//         : save_file_current_size_(0),
//           file_counter_(0) {
//     params_.beex_file      = beex_file;
//     params_.save_file      = save_file;
//     params_.orientation    = orientation;
//     params_.sonar_type     = sonar_type;
//     params_.nav_offset.x   = 0.096f;
//     params_.nav_offset.y   = -0.189f;
//     params_.nav_offset.z   = 0.0f;
//     params_.sonar_offset.x = -0.012;
//     params_.sonar_offset.y = 0.146;
//     params_.sonar_offset.z = 0.568;
//     if (sonar_type != "R2Sonic") {
//         throw std::runtime_error("Unsupported sonar type. Only 'R2Sonic' is supported.");
//     }

//     if (save_file.find(".xtf") == std::string::npos || save_file.substr(save_file.length() - 4) != ".xtf") {
//         throw std::runtime_error("Save file must have .xtf extension.");
//     }

//     base_save_file_   = save_file.substr(0, save_file.find(".xtf"));
//     params_.save_file = base_save_file_ + "_test5.xtf";
// }

// // Quaternion to Euler (unchanged)
// void BeexToXtfConverter::quaternion_to_euler_angle(double w, double x, double y, double z) {
//     double ysqr = y * y;
//     double t0   = 2.0 * (w * x + y * z);
//     double t1   = 1.0 - 2.0 * (x * x + ysqr);
//     roll        = std::atan2(t0, t1) * 180.0 / M_PI;

//     double t2 = 2.0 * (w * y - z * x);
//     t2        = std::clamp(t2, -1.0, 1.0);
//     pitch     = std::asin(t2) * 180.0 / M_PI;

//     double t3 = 2.0 * (w * z + x * y);
//     double t4 = 1.0 - 2.0 * (ysqr + z * z);
//     yaw       = std::atan2(t3, t4) * 180.0 / M_PI;
// }

// std::vector<uint8_t> BeexToXtfConverter::repack(const void *data, size_t size, size_t length) {
//     std::vector<uint8_t> byte_data(length, 0);
//     std::cout << "repacking" << std::endl;
//     memcpy(byte_data.data(), data, std::min(size, length));
//     std::cout << "repacking success" << std::endl;
//     return byte_data;
// }

// // Prepare beam bytes (unchanged)
// std::vector<uint8_t> BeexToXtfConverter::prepare_beam_bytes(const bx_msgs::SurveyorInfoPayload &data, int i) {
//     std::vector<uint8_t> buffer(64, 0);  // Pre-allocate exact 64 bytes
//     uint8_t *ptr = buffer.data();

//     uint32_t Id             = static_cast<uint32_t>(i);                   // 4 bytes (int)
//     double Intensity        = 0.0;                                        // 8 bytes (double)
//     uint32_t Quality        = 0;                                          // 4 bytes (int)
//     double TwoWayTravelTime = static_cast<double>(data.bathy_twoway[i]);  // 8 bytes (double)
//     double DeltaTime        = 0.0;                                        // 8 bytes (double)
//     double BeamAngle        = static_cast<double>(data.beam_angle[i]);    // 8 bytes (double)
//     double TiltAngle        = 0.0;                                        // 8 bytes (double)
//     float Reserved[4]       = {0};                                        // 16 bytes (float[4])

//     memcpy(ptr, &Id, sizeof(Id));
//     ptr = sizeof(Id);
//     std::cout << "After Id: " << (ptr - buffer.data()) << std::endl;
//     memcpy(ptr, &Intensity, sizeof(Intensity));
//     ptr = sizeof(Intensity);
//     std::cout << "After Intensity: " << (ptr - buffer.data()) << std::endl;
//     memcpy(ptr, &Quality, sizeof(Quality));
//     ptr = sizeof(Quality);
//     std::cout << "After Quality: " << (ptr - buffer.data()) << std::endl;
//     memcpy(ptr, &TwoWayTravelTime, sizeof(TwoWayTravelTime));
//     ptr = sizeof(TwoWayTravelTime);
//     std::cout << "After TwoWayTravelTime: " << (ptr - buffer.data()) << std::endl;
//     memcpy(ptr, &DeltaTime, sizeof(DeltaTime));
//     ptr = sizeof(DeltaTime);
//     std::cout << "After DeltaTime: " << (ptr - buffer.data()) << std::endl;
//     memcpy(ptr, &BeamAngle, sizeof(BeamAngle));
//     ptr = sizeof(BeamAngle);
//     std::cout << "After BeamAngle: " << (ptr - buffer.data()) << std::endl;
//     memcpy(ptr, &TiltAngle, sizeof(TiltAngle));
//     ptr = sizeof(TiltAngle);
//     std::cout << "After TiltAngle: " << (ptr - buffer.data()) << std::endl;

//     // Use repack for Reserved field
//     std::vector<uint8_t> reserved_bytes = repack(Reserved, sizeof(Reserved), 16);
//     memcpy(ptr, reserved_bytes.data(), reserved_bytes.size());
//     ptr += reserved_bytes.size();
//     std::cout << "After Reserved: " << (ptr - buffer.data()) << std::endl;


//     if (ptr - buffer.data() != 64) {
//         std::cerr << "Error: Beam size mismatch: " << (ptr - buffer.data()) << " != 64" << std::endl;
//         throw std::runtime_error("Beam size mismatch");
//     }

//     return buffer;
// }

// std::pair<std::vector<uint8_t>, std::vector<uint8_t>> BeexToXtfConverter::prepare_nav_file_packet(
//         const bx_msgs::VehicleState &data) {
//     std::vector<uint8_t> nav_bytes, alt_bytes;

//     // Convert NED to UTM
//     double utm_easting  = data.utm_easting + data.pose_ned.position.y;
//     double utm_northing = data.utm_northing + data.pose_ned.position.x;
//     double altitude     = data.altitude;    // Down in NED (negative)
//     float heave         = -data.depth_msl;  // Heave is negative depth

//     int utm_zone_number = data.utm_zone_number;
//     bool northern       = true;

//     // Convert UTM to Latitude/Longitude
//     double latitude_deg, longitude_deg;
//     try {
//         GeographicLib::UTMUPS::Reverse(
//                 utm_zone_number,
//                 northern,
//                 utm_easting,
//                 utm_northing,
//                 latitude_deg,
//                 longitude_deg);
//     } catch (const GeographicLib::GeographicErr &e) {
//         std::cerr << "GeographicLib error: " << e.what() << std::endl;
//         latitude_deg  = 0.0;
//         longitude_deg = 0.0;
//     }

//     // Convert quaternion to RPY
//     quaternion_to_euler_angle(
//             data.pose_ned.orientation.w,
//             data.pose_ned.orientation.x,
//             data.pose_ned.orientation.y,
//             data.pose_ned.orientation.z);

//     float Heading = static_cast<float>(yaw);

//     // Convert Unix timestamp
//     uint64_t unix_time_ms = data.unix_time_ms;
//     time_t unix_timestamp = static_cast<time_t>(unix_time_ms / 1'000);
//     struct tm *dt         = gmtime(&unix_timestamp);
//     uint32_t microseconds = (unix_time_ms % 1'000) * 1'000;

//     // ---- NAVIGATION PACKET ----
//     uint16_t MagicNumber        = 64'206;  // "0xFACE"
//     uint8_t HeaderType          = 42;      // XTF_HEADER_NAVIGATION
//     uint8_t Reserved[7]         = {0};     // Reserved, 7 bytes
//     uint32_t NumBytesThisRecord = 64;      // Confirmed 64 bytes
//     uint16_t Year               = dt->tm_year + 1'900;
//     uint8_t Month               = dt->tm_mon + 1;
//     uint8_t Day                 = dt->tm_mday;
//     uint8_t Hour                = dt->tm_hour;
//     uint8_t Minute              = dt->tm_min;
//     uint8_t Second              = dt->tm_sec;
//     uint32_t SourceEpoch        = unix_timestamp;
//     uint32_t TimeTag            = 0;  // Time since boot in ms
//     double Raw_Y_Coordinate     = longitude_deg;
//     double Raw_X_Coordinate     = latitude_deg;
//     double Raw_Altitude         = data.pose_ned.position.z;
//     uint8_t TimeFlag            = 3;
//     uint8_t Reserved1[6]        = {0};  // Reserved, 6 bytes

//     // Pack into byte vector
//     nav_bytes.insert(
//             nav_bytes.end(),
//             reinterpret_cast<uint8_t *>(&MagicNumber),
//             reinterpret_cast<uint8_t *>(&MagicNumber) + sizeof(MagicNumber));
//     nav_bytes.insert(
//             nav_bytes.end(),
//             reinterpret_cast<uint8_t *>(&HeaderType),
//             reinterpret_cast<uint8_t *>(&HeaderType) + sizeof(HeaderType));
//     nav_bytes.insert(nav_bytes.end(), Reserved, Reserved + sizeof(Reserved));
//     nav_bytes.insert(
//             nav_bytes.end(),
//             reinterpret_cast<uint8_t *>(&NumBytesThisRecord),
//             reinterpret_cast<uint8_t *>(&NumBytesThisRecord) + sizeof(NumBytesThisRecord));
//     nav_bytes.insert(
//             nav_bytes.end(),
//             reinterpret_cast<uint8_t *>(&Year),
//             reinterpret_cast<uint8_t *>(&Year) + sizeof(Year));
//     nav_bytes.insert(
//             nav_bytes.end(),
//             reinterpret_cast<uint8_t *>(&Month),
//             reinterpret_cast<uint8_t *>(&Month) + sizeof(Month));
//     nav_bytes.insert(
//             nav_bytes.end(),
//             reinterpret_cast<uint8_t *>(&Day),
//             reinterpret_cast<uint8_t *>(&Day) + sizeof(Day));
//     nav_bytes.insert(
//             nav_bytes.end(),
//             reinterpret_cast<uint8_t *>(&Hour),
//             reinterpret_cast<uint8_t *>(&Hour) + sizeof(Hour));
//     nav_bytes.insert(
//             nav_bytes.end(),
//             reinterpret_cast<uint8_t *>(&Minute),
//             reinterpret_cast<uint8_t *>(&Minute) + sizeof(Minute));
//     nav_bytes.insert(
//             nav_bytes.end(),
//             reinterpret_cast<uint8_t *>(&Second),
//             reinterpret_cast<uint8_t *>(&Second) + sizeof(Second));
//     nav_bytes.insert(
//             nav_bytes.end(),
//             reinterpret_cast<uint8_t *>(&microseconds),
//             reinterpret_cast<uint8_t *>(&microseconds) + sizeof(microseconds));
//     nav_bytes.insert(
//             nav_bytes.end(),
//             reinterpret_cast<uint8_t *>(&SourceEpoch),
//             reinterpret_cast<uint8_t *>(&SourceEpoch) + sizeof(SourceEpoch));
//     nav_bytes.insert(
//             nav_bytes.end(),
//             reinterpret_cast<uint8_t *>(&TimeTag),
//             reinterpret_cast<uint8_t *>(&TimeTag) + sizeof(TimeTag));
//     nav_bytes.insert(
//             nav_bytes.end(),
//             reinterpret_cast<uint8_t *>(&Raw_Y_Coordinate),
//             reinterpret_cast<uint8_t *>(&Raw_Y_Coordinate) + sizeof(Raw_Y_Coordinate));
//     nav_bytes.insert(
//             nav_bytes.end(),
//             reinterpret_cast<uint8_t *>(&Raw_X_Coordinate),
//             reinterpret_cast<uint8_t *>(&Raw_X_Coordinate) + sizeof(Raw_X_Coordinate));
//     nav_bytes.insert(
//             nav_bytes.end(),
//             reinterpret_cast<uint8_t *>(&Raw_Altitude),
//             reinterpret_cast<uint8_t *>(&Raw_Altitude) + sizeof(Raw_Altitude));
//     nav_bytes.insert(
//             nav_bytes.end(),
//             reinterpret_cast<uint8_t *>(&TimeFlag),
//             reinterpret_cast<uint8_t *>(&TimeFlag) + sizeof(TimeFlag));
//     nav_bytes.insert(nav_bytes.end(), Reserved1, Reserved1 + sizeof(Reserved1));

//     if (nav_bytes.size() != 64) {
//         std::cerr << "Error: Navigation packet size mismatch!" << std::endl;
//         nav_bytes.resize(64, 0);
//     }

//     // Attitude packet
//     HeaderType                 = 3;
//     uint8_t SubChannelNumber   = 0;
//     uint16_t NumChansToFollow  = 0;
//     uint16_t Reserved1_Att[2]  = {0};
//     NumBytesThisRecord         = 64;
//     uint32_t Reserved2_Att[2]  = {0};
//     uint32_t EpochMicroSeconds = microseconds;
//     SourceEpoch                = unix_timestamp;
//     float Pitch                = static_cast<float>(pitch);
//     float Roll                 = static_cast<float>(roll);
//     float Yaw_Att              = 0.0f;
//     uint16_t Milliseconds      = static_cast<uint16_t>(microseconds / 1'000);
//     uint8_t Reserved3          = 0;

//     alt_bytes.insert(
//             alt_bytes.end(),
//             reinterpret_cast<uint8_t *>(&MagicNumber),
//             reinterpret_cast<uint8_t *>(&MagicNumber) + sizeof(MagicNumber));
//     alt_bytes.insert(
//             alt_bytes.end(),
//             reinterpret_cast<uint8_t *>(&HeaderType),
//             reinterpret_cast<uint8_t *>(&HeaderType) + sizeof(HeaderType));
//     alt_bytes.insert(
//             alt_bytes.end(),
//             reinterpret_cast<uint8_t *>(&SubChannelNumber),
//             reinterpret_cast<uint8_t *>(&SubChannelNumber) + sizeof(SubChannelNumber));
//     alt_bytes.insert(
//             alt_bytes.end(),
//             reinterpret_cast<uint8_t *>(&NumChansToFollow),
//             reinterpret_cast<uint8_t *>(&NumChansToFollow) + sizeof(NumChansToFollow));
//     alt_bytes.insert(
//             alt_bytes.end(),
//             reinterpret_cast<uint8_t *>(Reserved1_Att),
//             reinterpret_cast<uint8_t *>(Reserved1_Att) + sizeof(Reserved1_Att));
//     alt_bytes.insert(
//             alt_bytes.end(),
//             reinterpret_cast<uint8_t *>(&NumBytesThisRecord),
//             reinterpret_cast<uint8_t *>(&NumBytesThisRecord) + sizeof(NumBytesThisRecord));
//     alt_bytes.insert(
//             alt_bytes.end(),
//             reinterpret_cast<uint8_t *>(Reserved2_Att),
//             reinterpret_cast<uint8_t *>(Reserved2_Att) + sizeof(Reserved2_Att));
//     alt_bytes.insert(
//             alt_bytes.end(),
//             reinterpret_cast<uint8_t *>(&EpochMicroSeconds),
//             reinterpret_cast<uint8_t *>(&EpochMicroSeconds) + sizeof(EpochMicroSeconds));
//     alt_bytes.insert(
//             alt_bytes.end(),
//             reinterpret_cast<uint8_t *>(&SourceEpoch),
//             reinterpret_cast<uint8_t *>(&SourceEpoch) + sizeof(SourceEpoch));
//     alt_bytes.insert(
//             alt_bytes.end(),
//             reinterpret_cast<uint8_t *>(&Pitch),
//             reinterpret_cast<uint8_t *>(&Pitch) + sizeof(Pitch));
//     alt_bytes.insert(
//             alt_bytes.end(),
//             reinterpret_cast<uint8_t *>(&Roll),
//             reinterpret_cast<uint8_t *>(&Roll) + sizeof(Roll));
//     alt_bytes.insert(
//             alt_bytes.end(),
//             reinterpret_cast<uint8_t *>(&heave),
//             reinterpret_cast<uint8_t *>(&heave) + sizeof(heave));
//     alt_bytes.insert(
//             alt_bytes.end(),
//             reinterpret_cast<uint8_t *>(&Yaw_Att),
//             reinterpret_cast<uint8_t *>(&Yaw_Att) + sizeof(Yaw_Att));
//     alt_bytes.insert(
//             alt_bytes.end(),
//             reinterpret_cast<uint8_t *>(&TimeTag),
//             reinterpret_cast<uint8_t *>(&TimeTag) + sizeof(TimeTag));
//     alt_bytes.insert(
//             alt_bytes.end(),
//             reinterpret_cast<uint8_t *>(&Heading),
//             reinterpret_cast<uint8_t *>(&Heading) + sizeof(Heading));
//     alt_bytes.insert(
//             alt_bytes.end(),
//             reinterpret_cast<uint8_t *>(&Year),
//             reinterpret_cast<uint8_t *>(&Year) + sizeof(Year));
//     alt_bytes.insert(
//             alt_bytes.end(),
//             reinterpret_cast<uint8_t *>(&Month),
//             reinterpret_cast<uint8_t *>(&Month) + sizeof(Month));
//     alt_bytes.insert(
//             alt_bytes.end(),
//             reinterpret_cast<uint8_t *>(&Day),
//             reinterpret_cast<uint8_t *>(&Day) + sizeof(Day));
//     alt_bytes.insert(
//             alt_bytes.end(),
//             reinterpret_cast<uint8_t *>(&Hour),
//             reinterpret_cast<uint8_t *>(&Hour) + sizeof(Hour));
//     alt_bytes.insert(
//             alt_bytes.end(),
//             reinterpret_cast<uint8_t *>(&Minute),
//             reinterpret_cast<uint8_t *>(&Minute) + sizeof(Minute));
//     alt_bytes.insert(
//             alt_bytes.end(),
//             reinterpret_cast<uint8_t *>(&Second),
//             reinterpret_cast<uint8_t *>(&Second) + sizeof(Second));
//     alt_bytes.insert(
//             alt_bytes.end(),
//             reinterpret_cast<uint8_t *>(&Milliseconds),
//             reinterpret_cast<uint8_t *>(&Milliseconds) + sizeof(Milliseconds));
//     alt_bytes.insert(
//             alt_bytes.end(),
//             reinterpret_cast<uint8_t *>(&Reserved3),
//             reinterpret_cast<uint8_t *>(&Reserved3) + sizeof(Reserved3));

//     if (alt_bytes.size() != 64) {
//         std::cerr << "Attitude packet size mismatch: " << alt_bytes.size() << " != 64" << std::endl;
//         alt_bytes.resize(64, 0);
//     }

//     return {nav_bytes, alt_bytes};
// }

// // // Prepare navigation file packet (unchanged)
// // std::pair<std::vector<uint8_t>, std::vector<uint8_t>> BeexToXtfConverter::prepare_nav_file_packet(
// //         const nav_msgs::Odometry &data) {
// //     std::vector<uint8_t> nav_bytes, alt_bytes;

// //     double utm_northing = data.pose.pose.position.y;
// //     double utm_easting  = data.pose.pose.position.x;
// //     int utm_zone_number = 32;
// //     bool northern       = true;

// //     /* MAYBE NEED TO REMOVE THIS */
// //     utm_northing += data.pose.pose.position.y;
// //     utm_easting += data.pose.pose.position.x;

// //     double latitude_deg, longitude_deg;
// //     GeographicLib::UTMUPS::Reverse(utm_zone_number, northern, utm_easting, utm_northing, latitude_deg,
// //     longitude_deg);

// //     double roll, pitch, yaw;
// //     quaternion_to_euler_angle(
// //             data.pose.pose.orientation.w,
// //             data.pose.pose.orientation.x,
// //             data.pose.pose.orientation.y,
// //             data.pose.pose.orientation.z,
// //             roll,
// //             pitch,
// //             yaw);

// //     uint64_t unix_time_ms = data.header.stamp.toNSec() / 1'000'000;
// //     time_t unix_timestamp = static_cast<time_t>(unix_time_ms / 1'000);
// //     struct tm *dt         = gmtime(&unix_timestamp);
// //     uint32_t microseconds = (unix_time_ms % 1'000) * 1'000;

// //     uint16_t MagicNumber        = 64'206;
// //     uint8_t HeaderType          = 42;
// //     uint8_t Reserved[7]         = {0};
// //     uint32_t NumBytesThisRecord = 64;
// //     uint16_t Year               = dt->tm_year + 1'900;
// //     uint8_t Month               = dt->tm_mon + 1;
// //     uint8_t Day                 = dt->tm_mday;
// //     uint8_t Hour                = dt->tm_hour;
// //     uint8_t Minute              = dt->tm_min;
// //     uint8_t Second              = dt->tm_sec;
// //     uint32_t Microseconds       = microseconds;
// //     uint32_t SourceEpoch        = unix_timestamp;
// //     uint32_t TimeTag            = 0;
// //     double Raw_Y_Coordinate     = longitude_deg;
// //     double Raw_X_Coordinate     = latitude_deg;
// //     double Raw_Altitude         = data.pose.pose.position.z;
// //     uint8_t TimeFlag            = 3;
// //     uint8_t Reserved1[6]        = {0};

// //     nav_bytes.insert(
// //             nav_bytes.end(),
// //             reinterpret_cast<uint8_t *>(&MagicNumber),
// //             reinterpret_cast<uint8_t *>(&MagicNumber) + sizeof(MagicNumber));
// //     nav_bytes.insert(
// //             nav_bytes.end(),
// //             reinterpret_cast<uint8_t *>(&HeaderType),
// //             reinterpret_cast<uint8_t *>(&HeaderType) + sizeof(HeaderType));
// //     nav_bytes.insert(nav_bytes.end(), Reserved, Reserved + sizeof(Reserved));
// //     nav_bytes.insert(
// //             nav_bytes.end(),
// //             reinterpret_cast<uint8_t *>(&NumBytesThisRecord),
// //             reinterpret_cast<uint8_t *>(&NumBytesThisRecord) + sizeof(NumBytesThisRecord));
// //     nav_bytes.insert(
// //             nav_bytes.end(),
// //             reinterpret_cast<uint8_t *>(&Year),
// //             reinterpret_cast<uint8_t *>(&Year) + sizeof(Year));
// //     nav_bytes.insert(
// //             nav_bytes.end(),
// //             reinterpret_cast<uint8_t *>(&Month),
// //             reinterpret_cast<uint8_t *>(&Month) + sizeof(Month));
// //     nav_bytes.insert(
// //             nav_bytes.end(),
// //             reinterpret_cast<uint8_t *>(&Day),
// //             reinterpret_cast<uint8_t *>(&Day) + sizeof(Day));
// //     nav_bytes.insert(
// //             nav_bytes.end(),
// //             reinterpret_cast<uint8_t *>(&Hour),
// //             reinterpret_cast<uint8_t *>(&Hour) + sizeof(Hour));
// //     nav_bytes.insert(
// //             nav_bytes.end(),
// //             reinterpret_cast<uint8_t *>(&Minute),
// //             reinterpret_cast<uint8_t *>(&Minute) + sizeof(Minute));
// //     nav_bytes.insert(
// //             nav_bytes.end(),
// //             reinterpret_cast<uint8_t *>(&Second),
// //             reinterpret_cast<uint8_t *>(&Second) + sizeof(Second));
// //     nav_bytes.insert(
// //             nav_bytes.end(),
// //             reinterpret_cast<uint8_t *>(&Microseconds),
// //             reinterpret_cast<uint8_t *>(&Microseconds) + sizeof(Microseconds));
// //     nav_bytes.insert(
// //             nav_bytes.end(),
// //             reinterpret_cast<uint8_t *>(&SourceEpoch),
// //             reinterpret_cast<uint8_t *>(&SourceEpoch) + sizeof(SourceEpoch));
// //     nav_bytes.insert(
// //             nav_bytes.end(),
// //             reinterpret_cast<uint8_t *>(&TimeTag),
// //             reinterpret_cast<uint8_t *>(&TimeTag) + sizeof(TimeTag));
// //     nav_bytes.insert(
// //             nav_bytes.end(),
// //             reinterpret_cast<uint8_t *>(&Raw_Y_Coordinate),
// //             reinterpret_cast<uint8_t *>(&Raw_Y_Coordinate) + sizeof(Raw_Y_Coordinate));
// //     nav_bytes.insert(
// //             nav_bytes.end(),
// //             reinterpret_cast<uint8_t *>(&Raw_X_Coordinate),
// //             reinterpret_cast<uint8_t *>(&Raw_X_Coordinate) + sizeof(Raw_X_Coordinate));
// //     nav_bytes.insert(
// //             nav_bytes.end(),
// //             reinterpret_cast<uint8_t *>(&Raw_Altitude),
// //             reinterpret_cast<uint8_t *>(&Raw_Altitude) + sizeof(Raw_Altitude));
// //     nav_bytes.insert(
// //             nav_bytes.end(),
// //             reinterpret_cast<uint8_t *>(&TimeFlag),
// //             reinterpret_cast<uint8_t *>(&TimeFlag) + sizeof(TimeFlag));
// //     nav_bytes.insert(nav_bytes.end(), Reserved1, Reserved1 + sizeof(Reserved1));

// //     if (nav_bytes.size() != 64) {
// //         std::cerr << "Nav packet size mismatch: " << nav_bytes.size() << " != 64" << std::endl;
// //         nav_bytes.resize(64, 0);
// //     }

// //     HeaderType                 = 3;
// //     uint8_t SubChannelNumber   = 0;
// //     uint16_t NumChansToFollow  = 0;
// //     uint16_t Reserved1_Att[2]  = {0};
// //     NumBytesThisRecord         = 64;
// //     uint32_t Reserved2_Att[2]  = {0};
// //     uint32_t EpochMicroSeconds = microseconds;
// //     SourceEpoch                = unix_timestamp;
// //     float Pitch                = static_cast<float>(pitch);
// //     float Roll                 = static_cast<float>(roll);
// //     float Heave                = -data.pose.pose.position.z;
// //     float Yaw_Att              = 0.0f;
// //     float Heading              = static_cast<float>(yaw);
// //     uint16_t Milliseconds      = microseconds / 1'000;
// //     uint8_t Reserved3          = 0;

// //     alt_bytes.insert(
// //             alt_bytes.end(),
// //             reinterpret_cast<uint8_t *>(&MagicNumber),
// //             reinterpret_cast<uint8_t *>(&MagicNumber) + sizeof(MagicNumber));
// //     alt_bytes.insert(
// //             alt_bytes.end(),
// //             reinterpret_cast<uint8_t *>(&HeaderType),
// //             reinterpret_cast<uint8_t *>(&HeaderType) + sizeof(HeaderType));
// //     alt_bytes.insert(
// //             alt_bytes.end(),
// //             reinterpret_cast<uint8_t *>(&SubChannelNumber),
// //             reinterpret_cast<uint8_t *>(&SubChannelNumber) + sizeof(SubChannelNumber));
// //     alt_bytes.insert(
// //             alt_bytes.end(),
// //             reinterpret_cast<uint8_t *>(&NumChansToFollow),
// //             reinterpret_cast<uint8_t *>(&NumChansToFollow) + sizeof(NumChansToFollow));
// //     alt_bytes.insert(
// //             alt_bytes.end(),
// //             reinterpret_cast<uint8_t *>(Reserved1_Att),
// //             reinterpret_cast<uint8_t *>(Reserved1_Att) + sizeof(Reserved1_Att));
// //     alt_bytes.insert(
// //             alt_bytes.end(),
// //             reinterpret_cast<uint8_t *>(&NumBytesThisRecord),
// //             reinterpret_cast<uint8_t *>(&NumBytesThisRecord) + sizeof(NumBytesThisRecord));
// //     alt_bytes.insert(
// //             alt_bytes.end(),
// //             reinterpret_cast<uint8_t *>(Reserved2_Att),
// //             reinterpret_cast<uint8_t *>(Reserved2_Att) + sizeof(Reserved2_Att));
// //     alt_bytes.insert(
// //             alt_bytes.end(),
// //             reinterpret_cast<uint8_t *>(&EpochMicroSeconds),
// //             reinterpret_cast<uint8_t *>(&EpochMicroSeconds) + sizeof(EpochMicroSeconds));
// //     alt_bytes.insert(
// //             alt_bytes.end(),
// //             reinterpret_cast<uint8_t *>(&SourceEpoch),
// //             reinterpret_cast<uint8_t *>(&SourceEpoch) + sizeof(SourceEpoch));
// //     alt_bytes.insert(
// //             alt_bytes.end(),
// //             reinterpret_cast<uint8_t *>(&Pitch),
// //             reinterpret_cast<uint8_t *>(&Pitch) + sizeof(Pitch));
// //     alt_bytes.insert(
// //             alt_bytes.end(),
// //             reinterpret_cast<uint8_t *>(&Roll),
// //             reinterpret_cast<uint8_t *>(&Roll) + sizeof(Roll));
// //     alt_bytes.insert(
// //             alt_bytes.end(),
// //             reinterpret_cast<uint8_t *>(&Heave),
// //             reinterpret_cast<uint8_t *>(&Heave) + sizeof(Heave));
// //     alt_bytes.insert(
// //             alt_bytes.end(),
// //             reinterpret_cast<uint8_t *>(&Yaw_Att),
// //             reinterpret_cast<uint8_t *>(&Yaw_Att) + sizeof(Yaw_Att));
// //     alt_bytes.insert(
// //             alt_bytes.end(),
// //             reinterpret_cast<uint8_t *>(&TimeTag),
// //             reinterpret_cast<uint8_t *>(&TimeTag) + sizeof(TimeTag));
// //     alt_bytes.insert(
// //             alt_bytes.end(),
// //             reinterpret_cast<uint8_t *>(&Heading),
// //             reinterpret_cast<uint8_t *>(&Heading) + sizeof(Heading));
// //     alt_bytes.insert(
// //             alt_bytes.end(),
// //             reinterpret_cast<uint8_t *>(&Year),
// //             reinterpret_cast<uint8_t *>(&Year) + sizeof(Year));
// //     alt_bytes.insert(
// //             alt_bytes.end(),
// //             reinterpret_cast<uint8_t *>(&Month),
// //             reinterpret_cast<uint8_t *>(&Month) + sizeof(Month));
// //     alt_bytes.insert(
// //             alt_bytes.end(),
// //             reinterpret_cast<uint8_t *>(&Day),
// //             reinterpret_cast<uint8_t *>(&Day) + sizeof(Day));
// //     alt_bytes.insert(
// //             alt_bytes.end(),
// //             reinterpret_cast<uint8_t *>(&Hour),
// //             reinterpret_cast<uint8_t *>(&Hour) + sizeof(Hour));
// //     alt_bytes.insert(
// //             alt_bytes.end(),
// //             reinterpret_cast<uint8_t *>(&Minute),
// //             reinterpret_cast<uint8_t *>(&Minute) + sizeof(Minute));
// //     alt_bytes.insert(
// //             alt_bytes.end(),
// //             reinterpret_cast<uint8_t *>(&Second),
// //             reinterpret_cast<uint8_t *>(&Second) + sizeof(Second));
// //     alt_bytes.insert(
// //             alt_bytes.end(),
// //             reinterpret_cast<uint8_t *>(&Milliseconds),
// //             reinterpret_cast<uint8_t *>(&Milliseconds) + sizeof(Milliseconds));
// //     alt_bytes.insert(
// //             alt_bytes.end(),
// //             reinterpret_cast<uint8_t *>(&Reserved3),
// //             reinterpret_cast<uint8_t *>(&Reserved3) + sizeof(Reserved3));

// //     if (alt_bytes.size() != 64) {
// //         std::cerr << "Attitude packet size mismatch: " << alt_bytes.size() << " != 64" << std::endl;
// //         alt_bytes.resize(64, 0);
// //     }

// //     return {nav_bytes, alt_bytes};
// // }

// // // Prepare header file packet (unchanged)
// // std::vector<uint8_t> BeexToXtfConverter::prepare_header_file_packet() {
// //     std::vector<uint8_t> buffer;

// //     uint8_t FileFormat                     = 123;
// //     uint8_t SystemType                     = 1;
// //     std::string RecordingProgramName       = params_.beex_file.substr(0, 8);
// //     uint8_t RecordingProgramVersion[8]     = {1, 0, 0, 0, 0, 0, 0, 0};
// //     std::string SonarName                  = params_.sonar_type.substr(0, 16);
// //     uint16_t SensorsType                   = 0;
// //     std::string NoteString                 = "";
// //     std::string ThisFileName               = params_.save_file.substr(0, 64);
// //     uint16_t NavUnits                      = 3;
// //     uint16_t NumberofSonarChannels         = 0;
// //     uint16_t NumberofBathymetryChannels    = 1;
// //     uint8_t NumberofSnippetsChannels       = 0;
// //     uint8_t NumberofForwardLookArrays      = 0;
// //     uint16_t NumberofEchoStrengthChannels  = 0;
// //     uint8_t NumberofInterferometryChannels = 0;

// //     uint8_t Reserved1          = 0;
// //     uint8_t Reserved2          = 0;
// //     uint8_t Reserved3          = 0;
// //     float ReferencePointHeight = 0.0f;

// //     uint8_t ProjectionType[12] = {0};
// //     uint8_t SpheroidType[10]   = {0};
// //     int32_t NavigationLatency  = 0;

// //     float OriginX        = 0.0;
// //     float OriginY        = 0.0;
// //     float NavOffsetY     = params_.nav_offset.y;
// //     float NavOffsetX     = params_.nav_offset.x;
// //     float NavOffsetZ     = params_.nav_offset.z;
// //     float NavOffsetYaw   = -90.0f;
// //     float MRUOffsetY     = 0.0f;
// //     float MRUOffsetX     = 0.0f;
// //     float MRUOffsetZ     = 0.0f;
// //     float MRUOffsetYaw   = 0.0f;
// //     float MRUOffsetPitch = 0.0f;
// //     float MRUOffsetRoll  = 0.0f;

// //     buffer.insert(
// //             buffer.end(),
// //             reinterpret_cast<uint8_t *>(&FileFormat),
// //             reinterpret_cast<uint8_t *>(&FileFormat) + sizeof(FileFormat));
// //     buffer.insert(
// //             buffer.end(),
// //             reinterpret_cast<uint8_t *>(&SystemType),
// //             reinterpret_cast<uint8_t *>(&SystemType) + sizeof(SystemType));
// //     buffer.insert(
// //             buffer.end(),
// //             repack(RecordingProgramName.c_str(), RecordingProgramName.size(), 8).begin(),
// //             repack(RecordingProgramName.c_str(), RecordingProgramName.size(), 8).end());
// //     buffer.insert(
// //             buffer.end(),
// //             repack(RecordingProgramVersion, sizeof(RecordingProgramVersion), 8).begin(),
// //             repack(RecordingProgramVersion, sizeof(RecordingProgramVersion), 8).end());
// //     buffer.insert(
// //             buffer.end(),
// //             repack(SonarName.c_str(), SonarName.size(), 16).begin(),
// //             repack(SonarName.c_str(), SonarName.size(), 16).end());
// //     buffer.insert(
// //             buffer.end(),
// //             reinterpret_cast<uint8_t *>(&SensorsType),
// //             reinterpret_cast<uint8_t *>(&SensorsType) + sizeof(SensorsType));
// //     buffer.insert(
// //             buffer.end(),
// //             repack(NoteString.c_str(), NoteString.size(), 64).begin(),
// //             repack(NoteString.c_str(), NoteString.size(), 64).end());
// //     buffer.insert(
// //             buffer.end(),
// //             repack(ThisFileName.c_str(), ThisFileName.size(), 64).begin(),
// //             repack(ThisFileName.c_str(), ThisFileName.size(), 64).end());
// //     buffer.insert(
// //             buffer.end(),
// //             reinterpret_cast<uint8_t *>(&NavUnits),
// //             reinterpret_cast<uint8_t *>(&NavUnits) + sizeof(NavUnits));
// //     buffer.insert(
// //             buffer.end(),
// //             reinterpret_cast<uint8_t *>(&NumberofSonarChannels),
// //             reinterpret_cast<uint8_t *>(&NumberofSonarChannels) + sizeof(NumberofSonarChannels));
// //     buffer.insert(
// //             buffer.end(),
// //             reinterpret_cast<uint8_t *>(&NumberofBathymetryChannels),
// //             reinterpret_cast<uint8_t *>(&NumberofBathymetryChannels) + sizeof(NumberofBathymetryChannels));
// //     buffer.insert(
// //             buffer.end(),
// //             reinterpret_cast<uint8_t *>(&NumberofSnippetsChannels),
// //             reinterpret_cast<uint8_t *>(&NumberofSnippetsChannels) + sizeof(NumberofSnippetsChannels));
// //     buffer.insert(
// //             buffer.end(),
// //             reinterpret_cast<uint8_t *>(&NumberofForwardLookArrays),
// //             reinterpret_cast<uint8_t *>(&NumberofForwardLookArrays) + sizeof(NumberofForwardLookArrays));
// //     buffer.insert(
// //             buffer.end(),
// //             reinterpret_cast<uint8_t *>(&NumberofEchoStrengthChannels),
// //             reinterpret_cast<uint8_t *>(&NumberofEchoStrengthChannels) + sizeof(NumberofEchoStrengthChannels));
// //     buffer.insert(
// //             buffer.end(),
// //             reinterpret_cast<uint8_t *>(&NumberofInterferometryChannels),
// //             reinterpret_cast<uint8_t *>(&NumberofInterferometryChannels) +
// // sizeof(NumberofInterferometryChannels));
// //     buffer.insert(
// //             buffer.end(),
// //             reinterpret_cast<uint8_t *>(&Reserved1),
// //             reinterpret_cast<uint8_t *>(&Reserved1) + sizeof(Reserved1));
// //     buffer.insert(
// //             buffer.end(),
// //             reinterpret_cast<uint8_t *>(&Reserved2),
// //             reinterpret_cast<uint8_t *>(&Reserved2) + sizeof(Reserved2));
// //     buffer.insert(
// //             buffer.end(),
// //             reinterpret_cast<uint8_t *>(&Reserved3),
// //             reinterpret_cast<uint8_t *>(&Reserved3) + sizeof(Reserved3));
// //     buffer.insert(
// //             buffer.end(),
// //             reinterpret_cast<uint8_t *>(&ReferencePointHeight),
// //             reinterpret_cast<uint8_t *>(&ReferencePointHeight) + sizeof(ReferencePointHeight));
// //     buffer.insert(
// //             buffer.end(),
// //             repack(ProjectionType, sizeof(ProjectionType), 12).begin(),
// //             repack(ProjectionType, sizeof(ProjectionType), 12).end());
// //     buffer.insert(
// //             buffer.end(),
// //             repack(SpheroidType, sizeof(SpheroidType), 10).begin(),
// //             repack(SpheroidType, sizeof(SpheroidType), 10).end());
// //     buffer.insert(
// //             buffer.end(),
// //             reinterpret_cast<uint8_t *>(&NavigationLatency),
// //             reinterpret_cast<uint8_t *>(&NavigationLatency) + sizeof(NavigationLatency));
// //     buffer.insert(
// //             buffer.end(),
// //             reinterpret_cast<uint8_t *>(&OriginY),
// //             reinterpret_cast<uint8_t *>(&OriginY) + sizeof(OriginY));
// //     buffer.insert(
// //             buffer.end(),
// //             reinterpret_cast<uint8_t *>(&OriginX),
// //             reinterpret_cast<uint8_t *>(&OriginX) + sizeof(OriginX));
// //     buffer.insert(
// //             buffer.end(),
// //             reinterpret_cast<uint8_t *>(&NavOffsetY),
// //             reinterpret_cast<uint8_t *>(&NavOffsetY) + sizeof(NavOffsetY));
// //     buffer.insert(
// //             buffer.end(),
// //             reinterpret_cast<uint8_t *>(&NavOffsetX),
// //             reinterpret_cast<uint8_t *>(&NavOffsetX) + sizeof(NavOffsetX));
// //     buffer.insert(
// //             buffer.end(),
// //             reinterpret_cast<uint8_t *>(&NavOffsetZ),
// //             reinterpret_cast<uint8_t *>(&NavOffsetZ) + sizeof(NavOffsetZ));
// //     buffer.insert(
// //             buffer.end(),
// //             reinterpret_cast<uint8_t *>(&NavOffsetYaw),
// //             reinterpret_cast<uint8_t *>(&NavOffsetYaw) + sizeof(NavOffsetYaw));
// //     buffer.insert(
// //             buffer.end(),
// //             reinterpret_cast<uint8_t *>(&MRUOffsetY),
// //             reinterpret_cast<uint8_t *>(&MRUOffsetY) + sizeof(MRUOffsetY));
// //     buffer.insert(
// //             buffer.end(),
// //             reinterpret_cast<uint8_t *>(&MRUOffsetX),
// //             reinterpret_cast<uint8_t *>(&MRUOffsetX) + sizeof(MRUOffsetX));
// //     buffer.insert(
// //             buffer.end(),
// //             reinterpret_cast<uint8_t *>(&MRUOffsetZ),
// //             reinterpret_cast<uint8_t *>(&MRUOffsetZ) + sizeof(MRUOffsetZ));
// //     buffer.insert(
// //             buffer.end(),
// //             reinterpret_cast<uint8_t *>(&MRUOffsetYaw),
// //             reinterpret_cast<uint8_t *>(&MRUOffsetYaw) + sizeof(MRUOffsetYaw));
// //     buffer.insert(
// //             buffer.end(),
// //             reinterpret_cast<uint8_t *>(&MRUOffsetPitch),
// //             reinterpret_cast<uint8_t *>(&MRUOffsetPitch) + sizeof(MRUOffsetPitch));
// //     buffer.insert(
// //             buffer.end(),
// //             reinterpret_cast<uint8_t *>(&MRUOffsetRoll),
// //             reinterpret_cast<uint8_t *>(&MRUOffsetRoll) + sizeof(MRUOffsetRoll));

// //     buffer.resize(1'024, 0);
// //     return buffer;
// // }


// std::vector<uint8_t> BeexToXtfConverter::prepare_header_file_packet() {
//     std::vector<uint8_t> buffer;

//     // Header fields
//     uint8_t FileFormat                     = 123;                             // Mandatory
//     uint8_t SystemType                     = 53;                              // Updated to 53 for R2Sonic
//     std::string RecordingProgramName       = params_.beex_file.substr(0, 8);  // Mandatory, padded to 8
//     char RecordingProgramVersion[8]        = "223";  // Fixed: 5 chars + null terminator = 6 bytes, fits in 8
//     std::string SonarName                  = params_.sonar_type.substr(0, 8);  // Recommended
//     uint16_t SensorsType                   = 0;                                // Mandatory, default to 0
//     std::string NoteString                 = "";                               // Recommended, padded to 64
//     std::string ThisFileName               = params_.save_file.substr(0, 64);  // Recommended, padded to 64
//     uint16_t NavUnits                      = 3;                                // Mandatory, 3 for Lat/Long
//     uint16_t NumberofSonarChannels         = 0;                                // Mandatory
//     uint16_t NumberofBathymetryChannels    = 1;                                // Mandatory
//     uint8_t NumberofSnippetsChannels       = 0;                                // Mandatory
//     uint8_t NumberofForwardLookArrays      = 0;                                // Mandatory
//     uint16_t NumberofEchoStrengthChannels  = 0;                                // Mandatory
//     uint8_t NumberofInterferometryChannels = 0;                                // Mandatory

//     uint8_t Reserved1          = 0;     // Unused
//     uint8_t Reserved2          = 0;     // Unused
//     uint8_t Reserved3          = 0;     // Unused
//     float ReferencePointHeight = 0.0f;  // Optional

//     uint8_t ProjectionType[12] = {0};  // Unused
//     uint8_t SpheroidType[10]   = {0};  // Unused
//     int32_t NavigationLatency  = 0;    // Optional
//     // params_.nav_offset.x       = 0.096f;
//     // params_.nav_offset.y       = -0.189f;
//     // params_.nav_offset.z       = 0.0f;
//     // params_.sonar_offset.x     = -0.012;
//     // params_.sonar_offset.y     = 0.146;
//     // params_.sonar_offset.z     = 0.568;

//     float OriginX        = 0.0;                   // Unused
//     float OriginY        = 0.0;                   // Unused
//     float NavOffsetY     = params_.nav_offset.y;  // Optional
//     float NavOffsetX     = params_.nav_offset.x;  // Optional
//     float NavOffsetZ     = params_.nav_offset.z;  // Optional
//     float NavOffsetYaw   = -90.0f;                // Optional
//     float MRUOffsetY     = 0.0f;                  // Optional
//     float MRUOffsetX     = 0.0f;                  // Optional
//     float MRUOffsetZ     = 0.0f;                  // Optional
//     float MRUOffsetYaw   = 0.0f;                  // Optional
//     float MRUOffsetPitch = 0.0f;                  // Optional
//     float MRUOffsetRoll  = 0.0f;                  // Optional

//     // Insert header fields into buffer
//     buffer.insert(
//             buffer.end(),
//             reinterpret_cast<uint8_t *>(&FileFormat),
//             reinterpret_cast<uint8_t *>(&FileFormat) + sizeof(FileFormat));
//     buffer.insert(
//             buffer.end(),
//             reinterpret_cast<uint8_t *>(&SystemType),
//             reinterpret_cast<uint8_t *>(&SystemType) + sizeof(SystemType));
//     buffer.insert(
//             buffer.end(),
//             repack(RecordingProgramName.c_str(), RecordingProgramName.size(), 8).begin(),
//             repack(RecordingProgramName.c_str(), RecordingProgramName.size(), 8).end());
//     buffer.insert(
//             buffer.end(),
//             repack(RecordingProgramVersion, sizeof(RecordingProgramVersion), 8).begin(),
//             repack(RecordingProgramVersion, sizeof(RecordingProgramVersion), 8).end());
//     buffer.insert(
//             buffer.end(),
//             repack(SonarName.c_str(), SonarName.size(), 8).begin(),
//             repack(SonarName.c_str(), SonarName.size(), 8).end());  /// sixteen

//     std::vector<uint8_t> SensorsType_bytes = repack(&SensorsType, sizeof(SensorsType), 2);
//     buffer.insert(buffer.end(), SensorsType_bytes.begin(), SensorsType_bytes.begin());
//     buffer.insert(
//             buffer.end(),
//             repack(NoteString.c_str(), NoteString.size(), 64).begin(),
//             repack(NoteString.c_str(), NoteString.size(), 64).end());
//     buffer.insert(
//             buffer.end(),
//             repack(ThisFileName.c_str(), ThisFileName.size(), 64).begin(),
//             repack(ThisFileName.c_str(), ThisFileName.size(), 64).end());
//     buffer.insert(
//             buffer.end(),
//             reinterpret_cast<uint8_t *>(&NavUnits),
//             reinterpret_cast<uint8_t *>(&NavUnits) + sizeof(NavUnits));
//     buffer.insert(
//             buffer.end(),
//             reinterpret_cast<uint8_t *>(&NumberofSonarChannels),
//             reinterpret_cast<uint8_t *>(&NumberofSonarChannels) + sizeof(NumberofSonarChannels));
//     buffer.insert(
//             buffer.end(),
//             reinterpret_cast<uint8_t *>(&NumberofBathymetryChannels),
//             reinterpret_cast<uint8_t *>(&NumberofBathymetryChannels) + sizeof(NumberofBathymetryChannels));
//     buffer.insert(
//             buffer.end(),
//             reinterpret_cast<uint8_t *>(&NumberofSnippetsChannels),
//             reinterpret_cast<uint8_t *>(&NumberofSnippetsChannels) + sizeof(NumberofSnippetsChannels));
//     buffer.insert(
//             buffer.end(),
//             reinterpret_cast<uint8_t *>(&NumberofForwardLookArrays),
//             reinterpret_cast<uint8_t *>(&NumberofForwardLookArrays) + sizeof(NumberofForwardLookArrays));
//     buffer.insert(
//             buffer.end(),
//             reinterpret_cast<uint8_t *>(&NumberofEchoStrengthChannels),
//             reinterpret_cast<uint8_t *>(&NumberofEchoStrengthChannels) + sizeof(NumberofEchoStrengthChannels));
//     buffer.insert(
//             buffer.end(),
//             reinterpret_cast<uint8_t *>(&NumberofInterferometryChannels),
//             reinterpret_cast<uint8_t *>(&NumberofInterferometryChannels) + sizeof(NumberofInterferometryChannels));
//     buffer.insert(
//             buffer.end(),
//             reinterpret_cast<uint8_t *>(&Reserved1),
//             reinterpret_cast<uint8_t *>(&Reserved1) + sizeof(Reserved1));
//     buffer.insert(
//             buffer.end(),
//             reinterpret_cast<uint8_t *>(&Reserved2),
//             reinterpret_cast<uint8_t *>(&Reserved2) + sizeof(Reserved2));
//     buffer.insert(
//             buffer.end(),
//             reinterpret_cast<uint8_t *>(&Reserved3),
//             reinterpret_cast<uint8_t *>(&Reserved3) + sizeof(Reserved3));
//     buffer.insert(
//             buffer.end(),
//             reinterpret_cast<uint8_t *>(&ReferencePointHeight),
//             reinterpret_cast<uint8_t *>(&ReferencePointHeight) + sizeof(ReferencePointHeight));
//     buffer.insert(
//             buffer.end(),
//             repack(ProjectionType, sizeof(ProjectionType), 12).begin(),
//             repack(ProjectionType, sizeof(ProjectionType), 12).end());
//     buffer.insert(
//             buffer.end(),
//             repack(SpheroidType, sizeof(SpheroidType), 10).begin(),
//             repack(SpheroidType, sizeof(SpheroidType), 10).end());
//     buffer.insert(
//             buffer.end(),
//             reinterpret_cast<uint8_t *>(&NavigationLatency),
//             reinterpret_cast<uint8_t *>(&NavigationLatency) + sizeof(NavigationLatency));
//     buffer.insert(
//             buffer.end(),
//             reinterpret_cast<uint8_t *>(&OriginY),
//             reinterpret_cast<uint8_t *>(&OriginY) + sizeof(OriginY));
//     buffer.insert(
//             buffer.end(),
//             reinterpret_cast<uint8_t *>(&OriginX),
//             reinterpret_cast<uint8_t *>(&OriginX) + sizeof(OriginX));
//     buffer.insert(
//             buffer.end(),
//             reinterpret_cast<uint8_t *>(&NavOffsetY),
//             reinterpret_cast<uint8_t *>(&NavOffsetY) + sizeof(NavOffsetY));
//     buffer.insert(
//             buffer.end(),
//             reinterpret_cast<uint8_t *>(&NavOffsetX),
//             reinterpret_cast<uint8_t *>(&NavOffsetX) + sizeof(NavOffsetX));
//     buffer.insert(
//             buffer.end(),
//             reinterpret_cast<uint8_t *>(&NavOffsetZ),
//             reinterpret_cast<uint8_t *>(&NavOffsetZ) + sizeof(NavOffsetZ));
//     buffer.insert(
//             buffer.end(),
//             reinterpret_cast<uint8_t *>(&NavOffsetYaw),
//             reinterpret_cast<uint8_t *>(&NavOffsetYaw) + sizeof(NavOffsetYaw));
//     buffer.insert(
//             buffer.end(),
//             reinterpret_cast<uint8_t *>(&MRUOffsetY),
//             reinterpret_cast<uint8_t *>(&MRUOffsetY) + sizeof(MRUOffsetY));
//     buffer.insert(
//             buffer.end(),
//             reinterpret_cast<uint8_t *>(&MRUOffsetX),
//             reinterpret_cast<uint8_t *>(&MRUOffsetX) + sizeof(MRUOffsetX));
//     buffer.insert(
//             buffer.end(),
//             reinterpret_cast<uint8_t *>(&MRUOffsetZ),
//             reinterpret_cast<uint8_t *>(&MRUOffsetZ) + sizeof(MRUOffsetZ));
//     buffer.insert(
//             buffer.end(),
//             reinterpret_cast<uint8_t *>(&MRUOffsetYaw),
//             reinterpret_cast<uint8_t *>(&MRUOffsetYaw) + sizeof(MRUOffsetYaw));
//     buffer.insert(
//             buffer.end(),
//             reinterpret_cast<uint8_t *>(&MRUOffsetPitch),
//             reinterpret_cast<uint8_t *>(&MRUOffsetPitch) + sizeof(MRUOffsetPitch));
//     buffer.insert(
//             buffer.end(),
//             reinterpret_cast<uint8_t *>(&MRUOffsetRoll),
//             reinterpret_cast<uint8_t *>(&MRUOffsetRoll) + sizeof(MRUOffsetRoll));

//     // Define CHANINFO structure
//     struct XTFChanInfo {
//         uint8_t TypeOfChannel    = 3;          // 1 byte (3 for bathymetry)
//         uint8_t SubChannelNumber = 1;          // 1 byte
//         uint16_t CorrectionFlags = 1;          // 2 bytes
//         uint16_t UniPolar        = 1;          // 2 bytes
//         uint16_t BytesPerSample  = 2;          // 2 bytes
//         uint32_t Reserved        = 0;          // 4 bytes
//         char ChannelName[16]     = "R2Sonic";  // 16 bytes
//         float VoltScale          = 5.0f;       // 4 bytes
//         float Frequency          = 0.0f;       // 4 bytes
//         float HorizBeamAngle     = 0.0f;       // 4 bytes
//         float TiltAngle          = 0.0f;       // 4 bytes
//         float BeamWidth          = 0.0f;       // 4 bytes
//         float OffsetX_pack;
//         float OffsetY_pack;
//         float OffsetZ_pack;
//         float OffsetYaw        = 0.0;
//         float OffsetPitch      = 0.0;
//         float OffsetRoll       = 0.0;
//         uint16_t BeamsPerArray = 0;
//         uint8_t SampleFormat   = 3;
//     };

//     // Add CHANINFO for the bathymetry channel
//     XTFChanInfo chanInfo;
//     chanInfo.OffsetX_pack = params_.sonar_offset.x;
//     chanInfo.OffsetY_pack = params_.sonar_offset.y;
//     chanInfo.OffsetZ_pack = params_.sonar_offset.z;
//     buffer.insert(
//             buffer.end(),
//             reinterpret_cast<uint8_t *>(&chanInfo),
//             reinterpret_cast<uint8_t *>(&chanInfo) + sizeof(chanInfo));

//     char ReservedArea2[53]                   = {0};  // 53 bytes
//     std::vector<uint8_t> reserved_area_bytes = repack(ReservedArea2, sizeof(ReservedArea2), 53);
//     buffer.insert(buffer.end(), reserved_area_bytes.begin(), reserved_area_bytes.end());

//     // Ensure total size is 1024 bytes
//     buffer.resize(1'024, 0);

//     return buffer;
// }


// // Prepare multibeam bytes (reserve capacity)
// std::vector<uint8_t> BeexToXtfConverter::prepare_multibeam_bytes(
//         const bx_msgs::SurveyorInfoPayload &data,
//         int ping_number) {
//     // Calculate total size: header (256 bytes) + beams (64 bytes each)
//     size_t total_size = 256 + data.beam_ranges.size() * 64;
//     std::vector<uint8_t> buffer(total_size, 0);  // Pre-allocate exact size
//     uint8_t *ptr = buffer.data();                // Direct pointer access

//     std::cout << "Beam ranges size: " << data.beam_ranges.size() << ", bathy_twoway size: " <<
//     data.bathy_twoway.size()
//               << std::endl;
//     std::cout << "Buffer allocated, size: " << buffer.size() << ", capacity: " << buffer.capacity() << std::endl;

//     time_t unix_timestamp = static_cast<time_t>(data.unix_time_ms / 1'000);
//     struct tm *dt         = gmtime(&unix_timestamp);
//     std::cout << "sonar type " << params_.sonar_type.substr(0, 16) << std::endl;

//     // Header fields
//     uint16_t MagicNumber         = 64'206;  // Corrected to match XTF spec
//     uint8_t HeaderType           = 28;
//     uint8_t SubChannelNumber     = 0;
//     uint16_t NumberChansToFollow = 0;
//     uint16_t Reserved1[2]        = {0};  // 4 bytes total
//     uint32_t NumBytesThisRecord  = static_cast<uint32_t>(total_size);
//     uint16_t Year                = dt->tm_year + 1'900;
//     uint8_t Month                = dt->tm_mon + 1;
//     uint8_t Day                  = dt->tm_mday;
//     uint8_t Hour                 = dt->tm_hour;
//     uint8_t Minute               = dt->tm_min;
//     uint8_t Second               = dt->tm_sec;
//     uint8_t HSeconds             = (data.unix_time_ms % 1'000) / 10;
//     uint16_t JulianDay           = 0;
//     uint32_t EventNumber         = 0;
//     uint32_t PingNumber          = ping_number;

//     float SoundVelocity         = 1542.0f;
//     float OceanTide             = 0.0f;
//     uint32_t Reserved2          = 0;
//     float ConductivityFreq      = 0.0f;
//     float TemperatureFreq       = 0.0f;
//     float PressureFreq          = 0.0f;
//     float PressureTemp          = 0.0f;
//     float Conductivity          = 0.0f;
//     float WaterTemperature      = 0.0f;
//     float Pressure              = 0.0f;
//     float ComputerSoundVelocity = 0.0f;
//     float MagX = 0.0f, MagY = 0.0f, MagZ = 0.0f;
//     float AuxVal1 = 0.0f, AuxVal2 = 0.0f, AuxVal3 = 0.0f;
//     float Reserved3 = 0.0f, Reserved4 = 0.0f, Reserved5 = 0.0f;
//     float SpeedLog = 0.0f, Turbidity = 0.0f;
//     float ShipSpeed = 0.0f, ShipGyro = 0.0f;
//     double ShipYCoordinate = 0.0, ShipXCoordinate = 0.0;
//     uint16_t ShipAltitude = 0, ShipDepth = 0;
//     uint8_t FixTimeHour = 0, FixTimeMinute = 0, FixTimeSecond = 0, FixTimeHSecond = 0;
//     float SensorSpeed = 0.0f, KP = 0.0f;
//     double SensorYCoordinate = 0.0, SensorXCoordinate = 0.0;
//     uint16_t SonarStatus = 0, RangeToFish = 0, BearingToFish = 0, CableOut = 0;
//     float Layback = 0.0f, CableTension = 0.0f;
//     float SensorDepth = 0.0f, SensorPrimaryAltitude = 0.0f, SensorAuxAltitude = 0.0f;
//     float SensorPitch = 0.0f, SensorRoll = 0.0f, SensorHeading = 0.0f;
//     float Heave = 0.0f, Yaw = 0.0f;
//     uint32_t AttitudeTimeTag = 0, NavFixMilliseconds = 0;
//     float DOT                 = 0.0f;
//     uint8_t ComputerClockHour = 0, ComputerClockMinute = 0, ComputerClockSecond = 0, ComputerClockHsec = 0;
//     int16_t FishPositionDeltaX = 0, FishPositionDeltaY = 0;
//     uint8_t FishPositionErrorCode = 0;
//     uint32_t OptionalOffset       = 0;
//     uint8_t CableOutHundredths    = 0;
//     uint8_t ReservedSpace2[6]     = {0};  // 6 bytes


//     // Pack header (256 bytes total)
//     memcpy(ptr, &MagicNumber, sizeof(MagicNumber));
//     ptr = sizeof(MagicNumber);
//     memcpy(ptr, &HeaderType, sizeof(HeaderType));
//     ptr = sizeof(HeaderType);
//     memcpy(ptr, &SubChannelNumber, sizeof(SubChannelNumber));
//     ptr = sizeof(SubChannelNumber);
//     memcpy(ptr, &NumberChansToFollow, sizeof(NumberChansToFollow));
//     ptr = sizeof(NumberChansToFollow);
//     // Use repack for field
//     std::vector<uint8_t> Reserved1_bytes = repack(Reserved1, sizeof(Reserved1), 4);
//     memcpy(ptr, Reserved1_bytes.data(), Reserved1_bytes.size());
//     ptr = Reserved1_bytes.size();
//     // memcpy(ptr, Reserved1, sizeof(Reserved1));
//     // ptr += sizeof(Reserved1);
//     // Use repack for field
//     std::vector<uint8_t> NumBytesThisRecord_bytes = repack(&NumBytesThisRecord, sizeof(NumBytesThisRecord), 4);
//     memcpy(ptr, NumBytesThisRecord_bytes.data(), NumBytesThisRecord_bytes.size());
//     ptr = NumBytesThisRecord_bytes.size();
//     memcpy(ptr, &Year, sizeof(Year));
//     ptr = sizeof(Year);
//     memcpy(ptr, &Month, sizeof(Month));
//     ptr = sizeof(Month);
//     memcpy(ptr, &Day, sizeof(Day));
//     ptr = sizeof(Day);
//     memcpy(ptr, &Hour, sizeof(Hour));
//     ptr = sizeof(Hour);
//     memcpy(ptr, &Minute, sizeof(Minute));
//     ptr = sizeof(Minute);
//     memcpy(ptr, &Second, sizeof(Second));
//     ptr = sizeof(Second);
//     memcpy(ptr, &HSeconds, sizeof(HSeconds));
//     ptr = sizeof(HSeconds);
//     memcpy(ptr, &JulianDay, sizeof(JulianDay));
//     ptr = sizeof(JulianDay);
//     memcpy(ptr, &EventNumber, sizeof(EventNumber));
//     ptr = sizeof(EventNumber);
//     memcpy(ptr, &PingNumber, sizeof(PingNumber));
//     ptr = sizeof(PingNumber);
//     memcpy(ptr, &SoundVelocity, sizeof(SoundVelocity));
//     ptr = sizeof(SoundVelocity);
//     memcpy(ptr, &OceanTide, sizeof(OceanTide));
//     ptr = sizeof(OceanTide);
//     memcpy(ptr, &Reserved2, sizeof(Reserved2));
//     ptr = sizeof(Reserved2);
//     memcpy(ptr, &ConductivityFreq, sizeof(ConductivityFreq));
//     ptr = sizeof(ConductivityFreq);
//     memcpy(ptr, &TemperatureFreq, sizeof(TemperatureFreq));
//     ptr = sizeof(TemperatureFreq);
//     memcpy(ptr, &PressureFreq, sizeof(PressureFreq));
//     ptr = sizeof(PressureFreq);
//     memcpy(ptr, &PressureTemp, sizeof(PressureTemp));
//     ptr = sizeof(PressureTemp);
//     memcpy(ptr, &Conductivity, sizeof(Conductivity));
//     ptr = sizeof(Conductivity);
//     memcpy(ptr, &WaterTemperature, sizeof(WaterTemperature));
//     ptr = sizeof(WaterTemperature);
//     memcpy(ptr, &Pressure, sizeof(Pressure));
//     ptr = sizeof(Pressure);
//     memcpy(ptr, &ComputerSoundVelocity, sizeof(ComputerSoundVelocity));
//     ptr = sizeof(ComputerSoundVelocity);
//     memcpy(ptr, &MagX, sizeof(MagX));
//     ptr = sizeof(MagX);
//     memcpy(ptr, &MagY, sizeof(MagY));
//     ptr = sizeof(MagY);
//     memcpy(ptr, &MagZ, sizeof(MagZ));
//     ptr = sizeof(MagZ);
//     memcpy(ptr, &AuxVal1, sizeof(AuxVal1));
//     ptr = sizeof(AuxVal1);
//     memcpy(ptr, &AuxVal2, sizeof(AuxVal2));
//     ptr = sizeof(AuxVal2);
//     memcpy(ptr, &AuxVal3, sizeof(AuxVal3));
//     ptr = sizeof(AuxVal3);
//     memcpy(ptr, &Reserved3, sizeof(Reserved3));
//     ptr = sizeof(Reserved3);
//     memcpy(ptr, &Reserved4, sizeof(Reserved4));
//     ptr = sizeof(Reserved4);
//     memcpy(ptr, &Reserved5, sizeof(Reserved5));
//     ptr = sizeof(Reserved5);
//     memcpy(ptr, &SpeedLog, sizeof(SpeedLog));
//     ptr = sizeof(SpeedLog);
//     memcpy(ptr, &Turbidity, sizeof(Turbidity));
//     ptr = sizeof(Turbidity);
//     memcpy(ptr, &ShipSpeed, sizeof(ShipSpeed));
//     ptr = sizeof(ShipSpeed);
//     memcpy(ptr, &ShipGyro, sizeof(ShipGyro));
//     ptr = sizeof(ShipGyro);
//     memcpy(ptr, &ShipYCoordinate, sizeof(ShipYCoordinate));
//     ptr = sizeof(ShipYCoordinate);
//     memcpy(ptr, &ShipXCoordinate, sizeof(ShipXCoordinate));
//     ptr = sizeof(ShipXCoordinate);
//     memcpy(ptr, &ShipAltitude, sizeof(ShipAltitude));
//     ptr = sizeof(ShipAltitude);
//     memcpy(ptr, &ShipDepth, sizeof(ShipDepth));
//     ptr = sizeof(ShipDepth);
//     memcpy(ptr, &FixTimeHour, sizeof(FixTimeHour));
//     ptr = sizeof(FixTimeHour);
//     memcpy(ptr, &FixTimeMinute, sizeof(FixTimeMinute));
//     ptr = sizeof(FixTimeMinute);
//     memcpy(ptr, &FixTimeSecond, sizeof(FixTimeSecond));
//     ptr = sizeof(FixTimeSecond);
//     memcpy(ptr, &FixTimeHSecond, sizeof(FixTimeHSecond));
//     ptr = sizeof(FixTimeHSecond);
//     memcpy(ptr, &SensorSpeed, sizeof(SensorSpeed));
//     ptr = sizeof(SensorSpeed);
//     memcpy(ptr, &KP, sizeof(KP));
//     ptr = sizeof(KP);
//     memcpy(ptr, &SensorYCoordinate, sizeof(SensorYCoordinate));
//     ptr = sizeof(SensorYCoordinate);
//     memcpy(ptr, &SensorXCoordinate, sizeof(SensorXCoordinate));
//     ptr = sizeof(SensorXCoordinate);
//     memcpy(ptr, &SonarStatus, sizeof(SonarStatus));
//     ptr = sizeof(SonarStatus);
//     memcpy(ptr, &RangeToFish, sizeof(RangeToFish));
//     ptr = sizeof(RangeToFish);
//     memcpy(ptr, &BearingToFish, sizeof(BearingToFish));
//     ptr = sizeof(BearingToFish);
//     memcpy(ptr, &CableOut, sizeof(CableOut));
//     ptr = sizeof(CableOut);
//     memcpy(ptr, &Layback, sizeof(Layback));
//     ptr = sizeof(Layback);
//     memcpy(ptr, &CableTension, sizeof(CableTension));
//     ptr = sizeof(CableTension);
//     memcpy(ptr, &SensorDepth, sizeof(SensorDepth));
//     ptr = sizeof(SensorDepth);
//     memcpy(ptr, &SensorPrimaryAltitude, sizeof(SensorPrimaryAltitude));
//     ptr = sizeof(SensorPrimaryAltitude);
//     memcpy(ptr, &SensorAuxAltitude, sizeof(SensorAuxAltitude));
//     ptr = sizeof(SensorAuxAltitude);
//     memcpy(ptr, &SensorPitch, sizeof(SensorPitch));
//     ptr = sizeof(SensorPitch);
//     memcpy(ptr, &SensorRoll, sizeof(SensorRoll));
//     ptr = sizeof(SensorRoll);
//     memcpy(ptr, &SensorHeading, sizeof(SensorHeading));
//     ptr = sizeof(SensorHeading);
//     memcpy(ptr, &Heave, sizeof(Heave));
//     ptr = sizeof(Heave);
//     memcpy(ptr, &Yaw, sizeof(Yaw));
//     ptr = sizeof(Yaw);
//     memcpy(ptr, &AttitudeTimeTag, sizeof(AttitudeTimeTag));
//     ptr = sizeof(AttitudeTimeTag);
//     memcpy(ptr, &DOT, sizeof(DOT));
//     ptr = sizeof(DOT);
//     memcpy(ptr, &NavFixMilliseconds, sizeof(NavFixMilliseconds));
//     ptr = sizeof(NavFixMilliseconds);
//     memcpy(ptr, &ComputerClockHour, sizeof(ComputerClockHour));
//     ptr = sizeof(ComputerClockHour);
//     memcpy(ptr, &ComputerClockMinute, sizeof(ComputerClockMinute));
//     ptr = sizeof(ComputerClockMinute);
//     memcpy(ptr, &ComputerClockSecond, sizeof(ComputerClockSecond));
//     ptr = sizeof(ComputerClockSecond);
//     memcpy(ptr, &ComputerClockHsec, sizeof(ComputerClockHsec));
//     ptr = sizeof(ComputerClockHsec);
//     memcpy(ptr, &FishPositionDeltaX, sizeof(FishPositionDeltaX));
//     ptr = sizeof(FishPositionDeltaX);
//     memcpy(ptr, &FishPositionDeltaY, sizeof(FishPositionDeltaY));
//     ptr = sizeof(FishPositionDeltaY);
//     memcpy(ptr, &FishPositionErrorCode, sizeof(FishPositionErrorCode));
//     ptr = sizeof(FishPositionErrorCode);
//     memcpy(ptr, &OptionalOffset, sizeof(OptionalOffset));
//     ptr = sizeof(OptionalOffset);
//     memcpy(ptr, &CableOutHundredths, sizeof(CableOutHundredths));
//     ptr = sizeof(CableOutHundredths);
//     memcpy(ptr, ReservedSpace2, sizeof(ReservedSpace2));
//     ptr = sizeof(ReservedSpace2);

//     std::cout << "Header packed, offset: " << (ptr - buffer.data()) << " (should be 256)" << std::endl;
//     if (ptr - buffer.data() != 256) {
//         std::cerr << "Error: Header size mismatch: " << (ptr - buffer.data()) << " != 256" << std::endl;
//         throw std::runtime_error("Header size mismatch");
//     }

//     // Pack beams
//     for (size_t i = 0; i < data.bathy_twoway.size(); ++i) {
//         auto beam_bytes = prepare_beam_bytes(data, i);
//         std::cout << 'a' << std::endl;
//         memcpy(ptr, beam_bytes.data(), beam_bytes.size());
//         ptr += beam_bytes.size();
//         std::cout << "Beam " << i << " packed, offset: " << (ptr - buffer.data()) << std::endl;
//     }

//     std::cout << "Final buffer size: " << (ptr - buffer.data()) << ", expected: " << total_size << std::endl;
//     if (ptr - buffer.data() != total_size) {
//         std::cerr << "Error: Final size mismatch: " << (ptr - buffer.data()) << " != " << total_size << std::endl;
//         throw std::runtime_error("Final size mismatch");
//     }

//     return buffer;
// }

// // Save current file (append mode)
// void BeexToXtfConverter::save_current_file() {
//     std::ofstream file(params_.save_file, std::ios::binary | std::ios::app);
//     if (!file.is_open()) {
//         std::cerr << "Failed to open file for writing: " << params_.save_file << std::endl;
//         return;
//     }
//     for (const auto &bytes : save_file_byte_array_) {
//         file.write(reinterpret_cast<const char *>(bytes.data()), bytes.size());
//     }
//     file.close();
//     std::cout << "Saved " << save_file_byte_array_.size() << " packets to " << params_.save_file << std::endl;
// }

// // // Updated run() to save after every message
// void BeexToXtfConverter::run() {
//     std::cout << "Starting Conversion..." << std::endl;

//     // Write initial header
//     auto header_bytes = prepare_header_file_packet();
//     save_file_byte_array_.push_back(header_bytes);
//     save_file_current_size_ += header_bytes.size();
//     save_current_file();
//     save_file_byte_array_.clear();
//     save_file_current_size_ = 0;

//     rosbag::Bag bag;
//     try {
//         bag.open(params_.beex_file, rosbag::bagmode::Read);
//     } catch (const rosbag::BagException &e) {
//         std::cerr << "Error opening bag file: " << e.what() << std::endl;
//         return;
//     }
//     std::cout << "Bag opened..." << std::endl;

//     std::vector<std::string> topics = {MULTIBEAM_TOPIC, VEHICLE_LATLON};
//     rosbag::View view(bag, rosbag::TopicQuery(topics));
//     int ping_number = 1;
//     std::cout << "Processing bag msg..." << std::endl;

//     for (const rosbag::MessageInstance &m : view) {
//         if (m.getTopic() == MULTIBEAM_TOPIC) {
//             bx_msgs::SurveyorInfoPayload::ConstPtr msg = m.instantiate<bx_msgs::SurveyorInfoPayload>();
//             if (msg) {
//                 std::cout << "Processing multibeam msg..." << std::endl;

//                 // auto multibeam_bytes = prepare_multibeam_bytes(*msg, ping_number++);
//                 // std::cout << "prepped multibeam msg..." << std::endl;

//                 // save_file_byte_array_.push_back(multibeam_bytes);
//                 // std::cout << "appending multibeam msg..." << std::endl;

//                 // save_file_current_size_ += multibeam_bytes.size();
//                 // std::cout << "inc multibeam list size..." << std::endl;

//                 // // Save and clear immediately
//                 // save_current_file();
//                 // save_file_byte_array_.clear();
//                 // save_file_current_size_ = 0;
//             }
//         } else if (m.getTopic() == VEHICLE_LATLON) {
//             bx_msgs::VehicleState::ConstPtr msg = m.instantiate<bx_msgs::VehicleState>();
//             if (msg) {
//                 std::cout << "Processing odom msg..." << std::endl;

//                 auto [nav_bytes, alt_bytes] = prepare_nav_file_packet(*msg);
//                 save_file_byte_array_.push_back(nav_bytes);
//                 save_file_byte_array_.push_back(alt_bytes);
//                 save_file_current_size_ += nav_bytes.size() + alt_bytes.size();
//                 std::cout << "inc nav list size..." << std::endl;

//                 // Save and clear immediately
//                 save_current_file();
//                 save_file_byte_array_.clear();
//                 save_file_current_size_ = 0;
//             }
//         }
//     }

//     // Final save (if anything remains)
//     if (!save_file_byte_array_.empty()) {
//         save_current_file();
//     }
//     bag.close();
//     std::cout << "Conversion completed." << std::endl;
// }

// // void BeexToXtfConverter::run() {
// //     std::cout << "Starting Conversion..." << std::endl;

// //     auto header_bytes = prepare_header_file_packet();
// //     save_file_byte_array_.push_back(header_bytes);
// //     save_file_current_size_ += header_bytes.size();

// //     rosbag::Bag bag;
// //     try {
// //         bag.open(params_.beex_file, rosbag::bagmode::Read);
// //     } catch (const rosbag::BagException &e) {
// //         std::cerr << "Error opening bag file: " << e.what() << std::endl;
// //         return;
// //     }
// //     std::cout << "Bag opened..." << std::endl;

// //     std::vector<std::string> topics = {MULTIBEAM_TOPIC, VEHICLE_LATLON};
// //     rosbag::View view(bag, rosbag::TopicQuery(topics));
// //     int ping_number = 1;
// //     std::cout << "Processing bag msg..." << std::endl;

// //     for (const rosbag::MessageInstance &m : view) {
// //         if (m.getTopic() == MULTIBEAM_TOPIC) {
// //             bx_msgs::SurveyorInfoPayload::ConstPtr msg = m.instantiate<bx_msgs::SurveyorInfoPayload>();
// //             if (msg) {
// //                 std::cout << "Processing multibeam msg..." << std::endl;

// //                 auto multibeam_bytes = prepare_multibeam_bytes(*msg, ping_number++);
// //                 std::cout << "prepped multibeam msg..." << std::endl;

// //                 save_file_byte_array_.push_back(multibeam_bytes);
// //                 std::cout << "appending mutlibeam msg..." << std::endl;

// //                 save_file_current_size_ += multibeam_bytes.size();
// //                 std::cout << "inc multibeam list size..." << std::endl;
// //             }
// //         } else if (m.getTopic() == VEHICLE_LATLON) {
// //             bx_msgs::VehicleState::ConstPtr msg = m.instantiate<bx_msgs::VehicleState>();
// //             if (msg) {
// //                 std::cout << "Processing odom msg..." << std::endl;

// //                 auto [nav_bytes, alt_bytes] = prepare_nav_file_packet(*msg);
// //                 save_file_byte_array_.push_back(nav_bytes);
// //                 save_file_byte_array_.push_back(alt_bytes);
// //                 save_file_current_size_ += nav_bytes.size() + alt_bytes.size();
// //                 std::cout << "inc nav list size..." << std::endl;
// //             }
// //             std::cout << "Processing odom msg..." << std::endl;
// //         }

// //         if (save_file_current_size_ >= SAVE_FILE_MAX_SIZE) {
// //             save_current_file();
// //             file_counter_++;
// //             params_.save_file = base_save_file_ + "_" + std::to_string(file_counter_) + ".xtf";
// //             save_file_byte_array_.clear();
// //             save_file_current_size_ = 0;

// //             auto new_header = prepare_header_file_packet();
// //             save_file_byte_array_.push_back(new_header);
// //             save_file_current_size_ += new_header.size();
// //         }
// //     }

// //     save_current_file();
// //     bag.close();
// // }

// int main(int argc, char *argv[]) {
//     if (argc != 3) {
//         std::cerr << "Usage: " << argv[0] << " <input_beex_file> <output_xtf_file>" << std::endl;
//         return 1;
//     }
//     BeexToXtfConverter converter(argv[1], argv[2]);
//     converter.run();
//     return 0;
// }