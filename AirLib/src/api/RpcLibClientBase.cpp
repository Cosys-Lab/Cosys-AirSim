// Copyright (c) Microsoft Corporation. All rights reserved.
// Licensed under the MIT License.

//in header only mode, control library is not available
#ifndef AIRLIB_HEADER_ONLY
//RPC code requires C++14. If build system like Unreal doesn't support it then use compiled binaries
#ifndef AIRLIB_NO_RPC
//if using Unreal Build system then include precompiled header file first

#include "api/RpcLibClientBase.hpp"

#include "common/Common.hpp"
#include "common/ClockFactory.hpp"
#include <functional>
#include <vector>
#include <thread>
STRICT_MODE_OFF

#ifndef RPCLIB_MSGPACK
#define RPCLIB_MSGPACK clmdep_msgpack
#endif // !RPCLIB_MSGPACK

#ifdef nil
#undef nil
#endif // nil

#include "common/common_utils/WindowsApisCommonPre.hpp"
#undef FLOAT
#undef check
#include "rpc/client.h"
//TODO: HACK: UE4 defines macro with stupid names like "check" that conflicts with msgpack library
#ifndef check
#define check(expr) (static_cast<void>((expr)))
#endif
#include "common/common_utils/WindowsApisCommonPost.hpp"

#include "api/RpcLibAdapatorsBase.hpp"


STRICT_MODE_ON
#ifdef _MSC_VER
__pragma(warning( disable : 4239))
#endif			  


namespace msr { namespace airlib {

struct RpcLibClientBase::impl {
    impl(const string&  ip_address, uint16_t port, float timeout_sec)
        : client(ip_address, port)
    {
        // some long flight path commands can take a while, so we give it up to 1 hour max.
        client.set_timeout(static_cast<int64_t>(timeout_sec * 1.0E3));
    }

    rpc::client client;
};

typedef msr::airlib_rpclib::RpcLibAdapatorsBase RpcLibAdapatorsBase;

RpcLibClientBase::RpcLibClientBase(const string&  ip_address, uint16_t port, float timeout_sec)
{
    pimpl_.reset(new impl(ip_address, port, timeout_sec));
}

RpcLibClientBase::~RpcLibClientBase()
{}

bool RpcLibClientBase::ping()
{
    return pimpl_->client.call("ping").as<bool>();
}
RpcLibClientBase::ConnectionState RpcLibClientBase::getConnectionState()
{
    switch (pimpl_->client.get_connection_state()) {
    case rpc::client::connection_state::connected: return ConnectionState::Connected;
    case rpc::client::connection_state::disconnected: return ConnectionState::Disconnected;
    case rpc::client::connection_state::initial: return ConnectionState::Initial;
    case rpc::client::connection_state::reset: return ConnectionState::Reset;
    default:
        return ConnectionState::Unknown;
    }
}
void RpcLibClientBase::enableApiControl(bool is_enabled, const std::string& vehicle_name)
{
    pimpl_->client.call("enableApiControl", is_enabled, vehicle_name);
}
bool RpcLibClientBase::isApiControlEnabled(const std::string& vehicle_name) const
{
    return pimpl_->client.call("isApiControlEnabled", vehicle_name).as<bool>();
}
int RpcLibClientBase::getClientVersion() const
{
    return 1; //sync with Python client
}
int RpcLibClientBase::getMinRequiredServerVersion() const
{
    return 1; //sync with Python client
}
int RpcLibClientBase::getMinRequiredClientVersion() const
{
    return pimpl_->client.call("getMinRequiredClientVersion").as<int>();
}
int RpcLibClientBase::getServerVersion() const
{
	return pimpl_->client.call("getServerVersion").as<int>();
}

void RpcLibClientBase::reset()
{
    pimpl_->client.call("reset");
}

void RpcLibClientBase::confirmConnection()
{
    ClockBase* clock = ClockFactory::get();

    // make sure we can talk to the DroneServer
    //std::cout << "Contacting DroneServer..." << std::flush;
    //command_context.client.ping();
    //std::cout << "DroneServer is responding." << std::endl;

    std::cout << "Waiting for connection - " << std::flush;
    const TTimeDelta pause_time = 1;
    while (getConnectionState() != RpcLibClientBase::ConnectionState::Connected)
    {
        std::cout << "X" << std::flush;
        clock->sleep_for(pause_time); 
    }
    std::cout << std::endl << "Connected!" << std::endl;

    auto server_ver = getServerVersion();
    auto client_ver = getClientVersion();
    auto server_min_ver = getMinRequiredServerVersion();
    auto client_min_ver = getMinRequiredClientVersion();
    
    std::string ver_info = Utils::stringf("Client Ver:%i (Min Req:%i), Server Ver:%i (Min Req:%i)",
        client_ver, client_min_ver, server_ver, server_min_ver);

    if (server_ver < server_min_ver) {
        std::cerr << std::endl << ver_info << std::endl;
        std::cerr << std::endl << "AirSim server is of older version and not supported by this client. Please upgrade!" << std::endl;
    }
    else if (client_ver < client_min_ver) {
        std::cerr << std::endl << ver_info << std::endl;
        std::cerr << std::endl << "AirSim client is of older version and not supported by this server. Please upgrade!" << std::endl;
    }
    else
        std::cout << std::endl << ver_info << std::endl;
}

bool RpcLibClientBase::armDisarm(bool arm, const std::string& vehicle_name)
{
    return pimpl_->client.call("armDisarm", arm, vehicle_name).as<bool>();
}

msr::airlib::GeoPoint RpcLibClientBase::getHomeGeoPoint(const std::string& vehicle_name) const
{
    return pimpl_->client.call("getHomeGeoPoint", vehicle_name).as<RpcLibAdapatorsBase::GeoPoint>().to();
}

msr::airlib::LidarData RpcLibClientBase::getLidarData(const std::string& lidar_name, const std::string& vehicle_name) const
{
    return pimpl_->client.call("getLidarData", lidar_name, vehicle_name).as<RpcLibAdapatorsBase::LidarData>().to();
}

bool RpcLibClientBase::simSetSegmentationObjectID(const std::string& mesh_name, int object_id, bool is_name_regex)
{
    return pimpl_->client.call("simSetSegmentationObjectID", mesh_name, object_id, is_name_regex).as<bool>();
}
int RpcLibClientBase::simGetSegmentationObjectID(const std::string& mesh_name) const
{
    return pimpl_->client.call("simGetSegmentationObjectID", mesh_name).as<int>();
}

CollisionInfo RpcLibClientBase::simGetCollisionInfo(const std::string& vehicle_name) const
{
    return pimpl_->client.call("simGetCollisionInfo", vehicle_name).as<RpcLibAdapatorsBase::CollisionInfo>().to();
}


//sim only
Pose RpcLibClientBase::simGetVehiclePose(const std::string& vehicle_name) const
{
    return pimpl_->client.call("simGetVehiclePose", vehicle_name).as<RpcLibAdapatorsBase::Pose>().to();
}
void RpcLibClientBase::simSetVehiclePose(const Pose& pose, bool ignore_collision, const std::string& vehicle_name)
{
    pimpl_->client.call("simSetVehiclePose", RpcLibAdapatorsBase::Pose(pose), ignore_collision, vehicle_name);
}

vector<msr::airlib::GeoPoint> RpcLibClientBase::xyzToGeoPoints(const vector<msr::airlib::Vector3r> &xyz_points, const std::string& vehicle_name)
{
    // TODO: a lot of copying...
    vector<RpcLibAdapatorsBase::Vector3r> input;
    input.reserve(xyz_points.size());
    for (const auto &point : xyz_points)
    {
        input.emplace_back(RpcLibAdapatorsBase::Vector3r(point));
    }

    vector<RpcLibAdapatorsBase::GeoPoint> result = pimpl_->client.call("simXyzToGeoPoints", input, vehicle_name).as<vector<RpcLibAdapatorsBase::GeoPoint>>();

    vector<msr::airlib::GeoPoint> output;
    output.reserve(result.size());
    for (const auto &point : result)
    {
        output.emplace_back(point.to());
    }
    
    return output;
}

vector<ImageCaptureBase::ImageResponse> RpcLibClientBase::simGetImages(vector<ImageCaptureBase::ImageRequest> request, const std::string& vehicle_name)
{
    const auto& response_adaptor = pimpl_->client.call("simGetImages", 
        RpcLibAdapatorsBase::ImageRequest::from(request), vehicle_name)
        .as<vector<RpcLibAdapatorsBase::ImageResponse>>();

    return RpcLibAdapatorsBase::ImageResponse::to(response_adaptor);
}
vector<uint8_t> RpcLibClientBase::simGetImage(const std::string& camera_name, ImageCaptureBase::ImageType type, const std::string& vehicle_name)
{
    vector<uint8_t> result = pimpl_->client.call("simGetImage", camera_name, type, vehicle_name).as<vector<uint8_t>>();
    if (result.size() == 1) {
        // rpclib has a bug with serializing empty vectors, so we return a 1 byte vector instead.
        result.clear();
    }
    return result;
}

void RpcLibClientBase::simSetCameraPose(const CameraPose camera_pose, const std::string& vehicle_name)
{
    pimpl_->client.call("simSetCameraPose", msr::airlib_rpclib::RpcLibAdapatorsBase::CameraPose(camera_pose), vehicle_name);
}

void RpcLibClientBase::simPrintLogMessage(const std::string& message, std::string message_param, unsigned char  severity)
{
    pimpl_->client.call("simPrintLogMessage", message, message_param, severity);
}

msr::airlib::RayCastResponse RpcLibClientBase::simRayCast(const msr::airlib::RayCastRequest request, const std::string vehicle_name)
{
    return pimpl_->client.call("simRayCast", msr::airlib_rpclib::RpcLibAdapatorsBase::RayCastRequest(request), vehicle_name).as<msr::airlib_rpclib::RpcLibAdapatorsBase::RayCastResponse>().to();
}

void RpcLibClientBase::simSetDrawableShapes(const msr::airlib::DrawableShapeRequest request, const std::string vehicle_name)
{
    pimpl_->client.call("simSetDrawableShapes", msr::airlib_rpclib::RpcLibAdapatorsBase::DrawableShapeRequest(request), vehicle_name);
}

void RpcLibClientBase::addDrawableShapePoint(msr::airlib::DrawableShapeRequest &request, const std::string &shape_name, const std::string &reference_frame_link, float x, float y, float z, float size, int color_r, int color_g, int color_b, int color_a)
{
    msr::airlib::DrawableShape shape;
    shape.reference_frame_link = reference_frame_link;
    shape.type = 0;

    shape.shape_params = std::vector<float>
    {
        x,
        y,
        z,
        size,
        static_cast<float>(color_r),
        static_cast<float>(color_g),
        static_cast<float>(color_b),
        static_cast<float>(color_a)
    };

    request.shapes.emplace(shape_name, shape);
}

void RpcLibClientBase::addDrawableShapeSphere(msr::airlib::DrawableShapeRequest &request, const std::string &shape_name, const std::string &reference_frame_link, float x, float y, float z, float radius, float thickness, int number_of_segments, int color_r, int color_g, int color_b, int color_a)
{
    msr::airlib::DrawableShape shape;
    shape.reference_frame_link = reference_frame_link;
    shape.type = 1;

    shape.shape_params = std::vector<float>
    {
        x,
        y,
        z,
        radius,
        thickness,
        static_cast<float>(number_of_segments),
        static_cast<float>(color_r),
        static_cast<float>(color_g),
        static_cast<float>(color_b),
        static_cast<float>(color_a)
    };

    request.shapes.emplace(shape_name, shape);
}

void RpcLibClientBase::addDrawableShapeCircle(msr::airlib::DrawableShapeRequest &request, const std::string &shape_name, const std::string &reference_frame_link, float x, float y, float z, float normal_x, float normal_y, float normal_z, float radius, float thickness, int number_of_segments, int color_r, int color_g, int color_b, int color_a)
{
    msr::airlib::DrawableShape shape;
    shape.reference_frame_link = reference_frame_link;
    shape.type = 2;

    shape.shape_params = std::vector<float>
    {
        x,
        y,
        z,
        normal_x,
        normal_y,
        normal_z,
        radius,
        thickness,
        static_cast<float>(number_of_segments),
        static_cast<float>(color_r),
        static_cast<float>(color_g),
        static_cast<float>(color_b),
        static_cast<float>(color_a)
    };

    request.shapes.emplace(shape_name, shape);
}

void RpcLibClientBase::addDrawableShapeBox(msr::airlib::DrawableShapeRequest &request, const std::string &shape_name, const std::string &reference_frame_link, float x, float y, float z, float extents_x, float extents_y, float extents_z, float thickness, int color_r, int color_g, int color_b, int color_a)
{
    msr::airlib::DrawableShape shape;
    shape.reference_frame_link = reference_frame_link;
    shape.type = 3;

    shape.shape_params = std::vector<float>
    {
        x,
        y,
        z,
        extents_x,
        extents_y,
        extents_z,
        thickness,
        static_cast<float>(color_r),
        static_cast<float>(color_g),
        static_cast<float>(color_b),
        static_cast<float>(color_a)
    };

    request.shapes.emplace(shape_name, shape);
}

void RpcLibClientBase::addDrawableShapeLine(msr::airlib::DrawableShapeRequest &request, const std::string &shape_name, const std::string &reference_frame_link, float start_x, float start_y, float start_z, float end_x, float end_y, float end_z, float thickness, int color_r, int color_g, int color_b, int color_a)
{
    msr::airlib::DrawableShape shape;
    shape.reference_frame_link = reference_frame_link;
    shape.type = 4;

    shape.shape_params = std::vector<float>
    {
        start_x,
        start_y,
        start_z,
        end_x,
        end_y,
        end_z,
        thickness,
        static_cast<float>(color_r),
        static_cast<float>(color_g),
        static_cast<float>(color_b),
        static_cast<float>(color_a)
    };

    request.shapes.emplace(shape_name, shape);
}

bool RpcLibClientBase::simIsPaused() const
{
    return pimpl_->client.call("simIsPaused").as<bool>();
}

void RpcLibClientBase::simPause(bool is_paused)
{
    pimpl_->client.call("simPause", is_paused);
}

void RpcLibClientBase::simContinueForTime(double seconds)
{
    pimpl_->client.call("simContinueForTime", seconds);
}

msr::airlib::Pose RpcLibClientBase::simGetObjectPose(const std::string& object_name) const
{
    return pimpl_->client.call("simGetObjectPose", object_name).as<RpcLibAdapatorsBase::Pose>().to();
}
bool RpcLibClientBase::simSetObjectPose(const std::string& object_name, const msr::airlib::Pose& pose, bool teleport)
{
    return pimpl_->client.call("simSetObjectPose", object_name, RpcLibAdapatorsBase::Pose(pose), teleport).as<bool>();
}
bool RpcLibClientBase::simSpawnStaticMeshObject(const std::string& object_class_name, const std::string& object_name, const msr::airlib::Pose& pose)
{
    return pimpl_->client.call("simSpawnStaticMeshObject", object_class_name, object_name, RpcLibAdapatorsBase::Pose(pose)).as<bool>();
}
bool RpcLibClientBase::simDeleteObject(const std::string& object_name)
{
    return pimpl_->client.call("simDeleteObject", object_name).as<bool>();
}

CameraInfo RpcLibClientBase::simGetCameraInfo(const std::string& camera_name, const std::string& vehicle_name) const
{
    return pimpl_->client.call("simGetCameraInfo", camera_name, vehicle_name).as<RpcLibAdapatorsBase::CameraInfo>().to();
}
void RpcLibClientBase::simSetCameraOrientation(const std::string& camera_name, const Quaternionr& orientation, const std::string& vehicle_name)
{
    pimpl_->client.call("simSetCameraOrientation", camera_name, RpcLibAdapatorsBase::Quaternionr(orientation), vehicle_name);
}

msr::airlib::Kinematics::State RpcLibClientBase::simGetGroundTruthKinematics(const std::string& vehicle_name) const
{
    return pimpl_->client.call("simGetGroundTruthKinematics", vehicle_name).as<RpcLibAdapatorsBase::KinematicsState>().to();
}
msr::airlib::Environment::State RpcLibClientBase::simGetGroundTruthEnvironment(const std::string& vehicle_name) const
{
    return pimpl_->client.call("simGetGroundTruthEnvironment", vehicle_name).as<RpcLibAdapatorsBase::EnvironmentState>().to();;
}

void RpcLibClientBase::cancelLastTask(const std::string& vehicle_name)
{
    pimpl_->client.call("cancelLastTask", vehicle_name);
}

//return value of last task. It should be true if task completed without
//cancellation or timeout
RpcLibClientBase* RpcLibClientBase::waitOnLastTask(bool* task_result, float timeout_sec)
{
    //should be implemented by derived class if it supports async task,
    //for example using futures
    unused(timeout_sec);
    unused(task_result);

    return this;
}

void* RpcLibClientBase::getClient()
{
    return &pimpl_->client;
}
const void* RpcLibClientBase::getClient() const
{
    return &pimpl_->client;
}

//----------- APIs to control ACharacter in scene ----------/
void RpcLibClientBase::simCharSetFaceExpression(const std::string& expression_name, float value, const std::string& character_name)
{
    pimpl_->client.call("simCharSetFaceExpression", expression_name, value, character_name);
}

float RpcLibClientBase::simCharGetFaceExpression(const std::string& expression_name, const std::string& character_name) const
{
    return pimpl_->client.call("simCharGetFaceExpression", expression_name, character_name).as<float>();
}

std::vector<std::string> RpcLibClientBase::simCharGetAvailableFaceExpressions()
{
    return pimpl_->client.call("simCharGetAvailableFaceExpressions").as<std::vector<std::string>>();
}

void RpcLibClientBase::simCharSetSkinDarkness(float value, const std::string& character_name)
{
    pimpl_->client.call("simCharSetSkinDarkness", character_name, value);
}

float RpcLibClientBase::simCharGetSkinDarkness(const std::string& character_name) const
{
    return pimpl_->client.call("simCharGetSkinDarkness", character_name).as<float>();
}

void RpcLibClientBase::simCharSetSkinAgeing(float value, const std::string& character_name)
{
    pimpl_->client.call("simCharSetSkinAgeing", character_name, value);
}

float RpcLibClientBase::simCharGetSkinAgeing(const std::string& character_name) const
{
    return pimpl_->client.call("simCharGetSkinAgeing", character_name).as<float>();
}

void RpcLibClientBase::simCharSetHeadRotation(const msr::airlib::Quaternionr& q, const std::string& character_name)
{
    pimpl_->client.call("simCharSetHeadRotation", RpcLibAdapatorsBase::Quaternionr(q), character_name);
}

msr::airlib::Quaternionr RpcLibClientBase::simCharGetHeadRotation(const std::string& character_name) const
{
    return pimpl_->client.call("simCharGetHeadRotation", character_name)
        .as<RpcLibAdapatorsBase::Quaternionr>().to();
}

void RpcLibClientBase::simCharSetBonePose(const std::string& bone_name, const msr::airlib::Pose& pose, const std::string& character_name)
{
    pimpl_->client.call("simCharSetBonePose", bone_name, RpcLibAdapatorsBase::Pose(pose), character_name);
}

msr::airlib::Pose RpcLibClientBase::simCharGetBonePose(const std::string& bone_name, const std::string& character_name) const
{
    return pimpl_->client.call("simCharGetBonePose", bone_name, character_name)
        .as<RpcLibAdapatorsBase::Pose>().to();
}

void RpcLibClientBase::simCharResetBonePose(const std::string& bone_name, const std::string& character_name)
{
    pimpl_->client.call("simCharResetBonePose", bone_name, character_name);
}

void RpcLibClientBase::simCharSetFacePreset(const std::string& preset_name, float value, const std::string& character_name)
{
    pimpl_->client.call("simCharSetFacePreset", preset_name, value, character_name);
}
void RpcLibClientBase::simSetFacePresets(const std::unordered_map<std::string, float>& presets, const std::string& character_name)
{
    pimpl_->client.call("simSetFacePresets", presets, character_name);
}
void RpcLibClientBase::simSetBonePoses(const std::unordered_map<std::string, msr::airlib::Pose>& poses, const std::string& character_name)
{
    std::unordered_map<std::string, RpcLibAdapatorsBase::Pose> r;
    for (const auto& p : poses)
        r[p.first] = RpcLibAdapatorsBase::Pose(p.second);

    pimpl_->client.call("simSetBonePoses", r, character_name);
}
std::unordered_map<std::string, msr::airlib::Pose> RpcLibClientBase::simGetBonePoses(const std::vector<std::string>& bone_names, const std::string& character_name) const
{
    std::unordered_map<std::string, RpcLibAdapatorsBase::Pose> t =
        pimpl_->client.call("simGetBonePoses", bone_names, character_name)
            .as<std::unordered_map<std::string, RpcLibAdapatorsBase::Pose>>();

    std::unordered_map<std::string, msr::airlib::Pose> r;
    for (const auto& p : t)
        r[p.first] = p.second.to();

    return r;
}


}} //namespace

#endif
#endif
