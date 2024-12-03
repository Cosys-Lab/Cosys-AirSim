// Copyright (c) Microsoft Corporation. All rights reserved.
// Licensed under the MIT License.

#ifndef msr_airlib_CommonStructs_hpp
#define msr_airlib_CommonStructs_hpp

#include "common/Common.hpp"
#include <ostream>

namespace msr
{
namespace airlib
{

    //velocity
    struct Twist
    {
        Vector3r linear, angular;

        Twist()
        {
        }

        Twist(const Vector3r& linear_val, const Vector3r& angular_val)
            : linear(linear_val), angular(angular_val)
        {
        }

        static const Twist zero()
        {
            static const Twist zero_twist(Vector3r::Zero(), Vector3r::Zero());
            return zero_twist;
        }
    };

    //force & torque
    struct Wrench
    {
        Vector3r force, torque;

        Wrench()
        {
        }

        Wrench(const Vector3r& force_val, const Vector3r& torque_val)
            : force(force_val), torque(torque_val)
        {
        }

        //support basic arithmatic
        Wrench operator+(const Wrench& other) const
        {
            Wrench result;
            result.force = this->force + other.force;
            result.torque = this->torque + other.torque;
            return result;
        }
        Wrench operator+=(const Wrench& other)
        {
            force += other.force;
            torque += other.torque;
            return *this;
        }
        Wrench operator-(const Wrench& other) const
        {
            Wrench result;
            result.force = this->force - other.force;
            result.torque = this->torque - other.torque;
            return result;
        }
        Wrench operator-=(const Wrench& other)
        {
            force -= other.force;
            torque -= other.torque;
            return *this;
        }

        static const Wrench zero()
        {
            static const Wrench zero_wrench(Vector3r::Zero(), Vector3r::Zero());
            return zero_wrench;
        }
    };

    struct Momentums
    {
        Vector3r linear;
        Vector3r angular;

        Momentums()
        {
        }

        Momentums(const Vector3r& linear_val, const Vector3r& angular_val)
            : linear(linear_val), angular(angular_val)
        {
        }

        static const Momentums zero()
        {
            static const Momentums zero_val(Vector3r::Zero(), Vector3r::Zero());
            return zero_val;
        }
    };

    struct Accelerations
    {
        Vector3r linear;
        Vector3r angular;

        Accelerations()
        {
        }

        Accelerations(const Vector3r& linear_val, const Vector3r& angular_val)
            : linear(linear_val), angular(angular_val)
        {
        }

        static const Accelerations zero()
        {
            static const Accelerations zero_val(Vector3r::Zero(), Vector3r::Zero());
            return zero_val;
        }
    };

    struct PoseWithCovariance
    {
        VectorMath::Pose pose;
        vector<real_T> covariance; //36 elements, 6x6 matrix

        PoseWithCovariance()
            : covariance(36, 0)
        {
        }
    };

    struct PowerSupply
    {
        vector<real_T> voltage, current;
    };

    struct TwistWithCovariance
    {
        Twist twist;
        vector<real_T> covariance; //36 elements, 6x6 matrix

        TwistWithCovariance()
            : covariance(36, 0)
        {
        }
    };

    struct Joystick
    {
        vector<float> axes;
        vector<int> buttons;
    };

    struct Odometry
    {
        PoseWithCovariance pose;
        TwistWithCovariance twist;
    };

    struct GeoPoint
    {
        double latitude = 0, longitude = 0;
        float altitude = 0;

        GeoPoint()
        {
        }

        GeoPoint(double latitude_val, double longitude_val, float altitude_val)
        {
            set(latitude_val, longitude_val, altitude_val);
        }

        void set(double latitude_val, double longitude_val, float altitude_val)
        {
            latitude = latitude_val, longitude = longitude_val;
            altitude = altitude_val;
        }

        friend std::ostream& operator<<(std::ostream& os, GeoPoint const& g)
        {
            return os << "[" << g.latitude << ", " << g.longitude << ", " << g.altitude << "]";
        }

        std::string to_string() const
        {
            return std::to_string(latitude) + string(", ") + std::to_string(longitude) + string(", ") + std::to_string(altitude);
        }
    };

    struct HomeGeoPoint
    {
        GeoPoint home_geo_point;
        double lat_rad, lon_rad;
        double cos_lat, sin_lat;

        HomeGeoPoint()
        {
        }
        HomeGeoPoint(const GeoPoint& home_geo_point_val)
        {
            initialize(home_geo_point_val);
        }
        void initialize(const GeoPoint& home_geo_point_val)
        {
            home_geo_point = home_geo_point_val;
            lat_rad = Utils::degreesToRadians(home_geo_point.latitude);
            lon_rad = Utils::degreesToRadians(home_geo_point.longitude);
            cos_lat = cos(lat_rad);
            sin_lat = sin(lat_rad);
        }
    };

    struct ProjectionMatrix
    {
        float matrix[4][4];

        void setTo(float val)
        {
            for (auto i = 0; i < 4; ++i)
                for (auto j = 0; j < 4; ++j)
                    matrix[i][j] = val;
        }
    };

    struct CollisionInfo
    {
        bool has_collided = false;
        Vector3r normal = Vector3r::Zero();
        Vector3r impact_point = Vector3r::Zero();
        Vector3r position = Vector3r::Zero();
        real_T penetration_depth = 0;
        TTimePoint time_stamp = 0;
        unsigned int collision_count = 0;
        std::string object_name;
        int object_id = -1;

        CollisionInfo()
        {
        }

        CollisionInfo(bool has_collided_val, const Vector3r& normal_val,
                      const Vector3r& impact_point_val, const Vector3r& position_val,
                      real_T penetration_depth_val, TTimePoint time_stamp_val,
                      const std::string& object_name_val, int object_id_val)
            : has_collided(has_collided_val), normal(normal_val), impact_point(impact_point_val), position(position_val), penetration_depth(penetration_depth_val), time_stamp(time_stamp_val), object_name(object_name_val), object_id(object_id_val)
        {
        }
    };

    struct CameraInfo
    {
        Pose pose;
        float fov;
        ProjectionMatrix proj_mat;

        CameraInfo()
        {
        }

        CameraInfo(const Pose& pose_val, float fov_val, const ProjectionMatrix& proj_mat_val)
            : pose(pose_val), fov(fov_val), proj_mat(proj_mat_val)
        {
        }
    };

    struct Box2D
    {
        Vector2r min;
        Vector2r max;

        Box2D()
        {
        }

        Box2D(Vector2r min_val, Vector2r max_val)
            : min(min_val), max(max_val)
        {
        }
    };

    struct Box3D
    {
        Vector3r min;
        Vector3r max;

        Box3D()
        {
        }

        Box3D(Vector3r min_val, Vector3r max_val)
            : min(min_val), max(max_val)
        {
        }
    };

    struct DetectionInfo
    {
        std::string name = "";
        GeoPoint geo_point = GeoPoint();
        Box2D box2D = Box2D();
        Box3D box3D = Box3D();
        Pose relative_pose = Pose();
        std::vector<std::string> tags;

        DetectionInfo()
        {
        }

        DetectionInfo(const std::string& name_val, const GeoPoint& geo_point_val, const Box2D& box2D_val, const Box3D& box3D_val, const Pose& relative_pose_val, const std::vector<std::string>& tags_val)
            : name(name_val), geo_point(geo_point_val), box2D(box2D_val), box3D(box3D_val), relative_pose(relative_pose_val), tags(tags_val)
        {
        }
    };

    struct CollisionResponse
    {
        unsigned int collision_count_raw = 0;
        unsigned int collision_count_non_resting = 0;
        TTimePoint collision_time_stamp = 0;
    };

    struct GeoPose
    {
        EIGEN_MAKE_ALIGNED_OPERATOR_NEW
        Quaternionr orientation;
        GeoPoint position;
    };

    struct RCData
    {
        TTimePoint timestamp = 0;
        //pitch, roll, yaw should be in range -1 to 1
        //switches should be integer value indicating its state, 0=on, 1=off for example.
        float pitch = 0, roll = 0, throttle = 0, yaw = 0;
        float left_z = 0, right_z = 0;
        uint16_t switches = 0;
        std::string vendor_id = "";
        bool is_initialized = false; //is RC connected?
        bool is_valid = false; //must be true for data to be valid

        unsigned int getSwitch(uint16_t index) const
        {
            return switches & (1 << index) ? 1 : 0;
        }

        void add(const RCData& other)
        {
            pitch += other.pitch;
            roll += other.roll;
            throttle += other.throttle;
            yaw += other.yaw;
        }
        void subtract(const RCData& other)
        {
            pitch -= other.pitch;
            roll -= other.roll;
            throttle -= other.throttle;
            yaw -= other.yaw;
        }
        void divideBy(float k)
        {
            pitch /= k;
            roll /= k;
            throttle /= k;
            yaw /= k;
        }
        bool isAnyMoreThan(float k)
        {
            using std::abs;
            return abs(pitch) > k || abs(roll) > k || abs(throttle) > k || abs(yaw) > k;
        }
        string toString()
        {
            return Utils::stringf("RCData[pitch=%f, roll=%f, throttle=%f, yaw=%f]", pitch, roll, throttle, yaw);
        }
    };

    struct LidarData {
        TTimePoint time_stamp = 0;
        vector<real_T> point_cloud;
        vector<std::string> groundtruth;
        Pose pose;

        LidarData()
        {
        }
    };

    struct DistanceSensorData
    {
        TTimePoint time_stamp;
        real_T distance; //meters
        real_T min_distance; //m
        real_T max_distance; //m
        Pose relative_pose;

        DistanceSensorData()
        {
        }
    };

    struct MeshPositionVertexBuffersResponse
    {
        Vector3r position;
        Quaternionr orientation;

        std::vector<float> vertices;
        std::vector<uint32_t> indices;
        std::string name;
    };

    // This is a small helper struct to keep camera details together
    // Not currently exposed to the client, just for cleaner codebase internally
    struct CameraDetails
    {
        std::string camera_name;
        std::string vehicle_name;

        CameraDetails(const std::string& camera_name_val, const std::string& vehicle_name_val)
            : camera_name(camera_name_val), vehicle_name(vehicle_name_val)
        {
        }

        std::string to_string() const
        {
            return Utils::stringf("CameraDetails: camera_name=%s, vehicle_name=%s",
                                  camera_name.c_str(),
                                  vehicle_name.c_str());
        }
    };

    struct GPULidarData {

        TTimePoint time_stamp = 0;
        vector<real_T> point_cloud;
        Pose pose;

        GPULidarData()
        {}
    };

    struct EchoData {

        TTimePoint time_stamp = 0;
        vector<real_T> point_cloud;
        vector<std::string> groundtruth;
        Pose pose;
        vector<std::string> passive_beacons_groundtruth;
        vector<real_T> passive_beacons_point_cloud;

        EchoData()
        {}
    };

    struct SensorTemplateData {

        TTimePoint time_stamp = 0;
        vector<real_T> point_cloud;
        Pose pose;

        SensorTemplateData()
        {}
    };

    struct UWBHit
    {
        TTimePoint time_stamp = 0;
        std::string beaconID;
        float beaconPosX, beaconPosY, beaconPosZ;
        bool isValid;
        float distance;
        float rssi;
    };


    struct MarLocUwbSensorData {

        TTimePoint time_stamp = 0;
        //vector<real_T> point_cloud;
        Pose pose;
        vector<std::string> beaconsActiveID;
        vector<float> beaconsActiveRssi;
        vector<float> beaconsActivePosX;
        vector<float> beaconsActivePosY;
        vector<float> beaconsActivePosZ;
        vector<float> beaconsActiveDistance;

        vector<float> allBeaconsId, allBeaconsX, allBeaconsY, allBeaconsZ;

        MarLocUwbSensorData()
        {}
    };


    struct MarLocUwbRange {
        TTimePoint time_stamp = 0;
        std::string anchorId;
        float anchorPosX, anchorPosY, anchorPosZ;
        bool valid_range;
        float distance;
        float rssi;

        MarLocUwbRange()
        {}
    };


    struct MarLocUwbRangeArray {
        std::string tagId;
        float tagPosX, tagPosY, tagPosZ;
        vector<int> ranges;

        MarLocUwbRangeArray()
        {}
    };

    struct MarLocUwbReturnMessage {
        //MarLocUwbRange
        vector <TTimePoint> mur_time_stamp;
        vector<std::string> mur_anchorId;
        vector<float> mur_anchorPosX, mur_anchorPosY, mur_anchorPosZ;
        vector<bool> mur_valid_range;
        vector<float> mur_distance;
        vector<float> mur_rssi;

        //MarLocUwbRangeArray
        vector<std::string> mura_tagId;
        vector<float> mura_tagPosX, mura_tagPosY, mura_tagPosZ;
        vector <vector<int>> mura_ranges;

        MarLocUwbReturnMessage()
        {}
    };

    struct WifiHit
    {
        TTimePoint time_stamp = 0;
        std::string beaconID;
        float beaconPosX, beaconPosY, beaconPosZ;
        bool isValid;
        float distance;
        float rssi;
    };

    struct WifiSensorData {

        TTimePoint time_stamp = 0;
        Pose pose;
        vector<std::string> beaconsActiveID;
        vector<float> beaconsActiveRssi;
        vector<float> beaconsActivePosX;
        vector<float> beaconsActivePosY;
        vector<float> beaconsActivePosZ;
        vector<float> beaconsActiveDistance;

        vector<float> allBeaconsId, allBeaconsX, allBeaconsY, allBeaconsZ;

        WifiSensorData()
        {}
    };

    struct WifiRange {
        TTimePoint time_stamp = 0;
        std::string anchorId;
        float anchorPosX, anchorPosY, anchorPosZ;
        bool valid_range;
        float distance;
        float rssi;

        WifiRange()
        {}
    };

    struct WifiRangeArray {
        std::string tagId;
        float tagPosX, tagPosY, tagPosZ;
        vector<int> ranges;

        WifiRangeArray()
        {}
    };

    struct WifiReturnMessage {
        //WifiRange
        vector <TTimePoint> wr_time_stamp;
        vector<std::string> wr_anchorId;
        vector<float> wr_anchorPosX, wr_anchorPosY, wr_anchorPosZ;
        vector<bool> wr_valid_range;
        vector<float> wr_distance;
        vector<float> wr_rssi;

        //WifiRangeArray
        vector<std::string> wra_tagId;
        vector<float> wra_tagPosX, wra_tagPosY, wra_tagPosZ;
        vector <vector<int>> wra_ranges;

        WifiReturnMessage()
        {}
    };

}} //namespace
#endif
