// Copyright (c) Microsoft Corporation. All rights reserved.
// Licensed under the MIT License.

//in header only mode, control library is not available
#ifndef AIRLIB_HEADER_ONLY
//RPC code requires C++14. If build system like Unreal doesn't support it then use compiled binaries
#ifndef AIRLIB_NO_RPC
//if using Unreal Build system then include precompiled header file first

#include "api/RpcLibServerBase.hpp"

#include "common/Common.hpp"
STRICT_MODE_OFF

#ifndef RPCLIB_MSGPACK
#define RPCLIB_MSGPACK clmdep_msgpack
#endif // !RPCLIB_MSGPACK
#include "common/common_utils/MinWinDefines.hpp"
#undef NOUSER

#include "common/common_utils/WindowsApisCommonPre.hpp"
#undef FLOAT
#undef check
#include "rpc/server.h"
//TODO: HACK: UE4 defines macro with stupid names like "check" that conflicts with msgpack library
#ifndef check
#define check(expr) (static_cast<void>((expr)))
#endif
#include "common/common_utils/WindowsApisCommonPost.hpp"

#include "api/RpcLibAdaptorsBase.hpp"
#include <functional>
#include <thread>

STRICT_MODE_ON

namespace msr
{
namespace airlib
{

    struct RpcLibServerBase::impl
    {
        impl(string server_address, uint16_t port)
            : server(server_address, port)
        {
        }

        impl(uint16_t port)
            : server(port)
        {
        }

        ~impl()
        {
        }

        void stop()
        {
            server.close_sessions();
            if (!is_async_) {
                // this deadlocks UI thread if async_run was called while there are pending rpc calls.
                server.stop();
            }
        }

        void run(bool block, std::size_t thread_count)
        {
            if (block) {
                server.run();
            }
            else {
                is_async_ = true;
                server.async_run(thread_count); //4 threads
            }
        }

        rpc::server server;
        bool is_async_ = false;
    };

    typedef msr::airlib_rpclib::RpcLibAdaptorsBase RpcLibAdaptorsBase;

    RpcLibServerBase::RpcLibServerBase(ApiProvider* api_provider, const std::string& server_address, uint16_t port)
        : api_provider_(api_provider)
    {

        if (server_address == "")
            pimpl_.reset(new impl(port));
        else
            pimpl_.reset(new impl(server_address, port));

        pimpl_->server.bind("ping", [&]() -> bool { return true; });

        pimpl_->server.bind("getServerVersion", []() -> int {
            return 3;
        });

        pimpl_->server.bind("getMinRequiredClientVersion", []() -> int {
            return 3;
        });

        pimpl_->server.bind("simPause", [&](bool is_paused) -> void {
            getWorldSimApi()->pause(is_paused);
        });

        pimpl_->server.bind("simIsPaused", [&]() -> bool {
            return getWorldSimApi()->isPaused();
        });

        pimpl_->server.bind("simContinueForTime", [&](double seconds) -> void {
            getWorldSimApi()->continueForTime(seconds);
        });

        pimpl_->server.bind("simContinueForFrames", [&](uint32_t frames) -> void {
            getWorldSimApi()->continueForFrames(frames);
        });

        pimpl_->server.bind("simSetTimeOfDay", [&](bool is_enabled, const string& start_datetime, bool is_start_datetime_dst, float celestial_clock_speed, float update_interval_secs, bool move_sun) -> void {
            getWorldSimApi()->setTimeOfDay(is_enabled, start_datetime, is_start_datetime_dst, celestial_clock_speed, update_interval_secs, move_sun);
        });

        pimpl_->server.bind("simEnableWeather", [&](bool enable) -> void {
            getWorldSimApi()->enableWeather(enable);
        });

        pimpl_->server.bind("simSetWeatherParameter", [&](WorldSimApiBase::WeatherParameter param, float val) -> void {
            getWorldSimApi()->setWeatherParameter(param, val);
        });

        pimpl_->server.bind("enableApiControl", [&](bool is_enabled, const std::string& vehicle_name) -> void {
            getVehicleApi(vehicle_name)->enableApiControl(is_enabled);
        });

        pimpl_->server.bind("isApiControlEnabled", [&](const std::string& vehicle_name) -> bool {
            return getVehicleApi(vehicle_name)->isApiControlEnabled();
        });

        pimpl_->server.bind("armDisarm", [&](bool arm, const std::string& vehicle_name) -> bool {
            return getVehicleApi(vehicle_name)->armDisarm(arm);
        });

        pimpl_->server.bind("simRunConsoleCommand", [&](const std::string& command) -> bool {
            return getWorldSimApi()->runConsoleCommand(command);
        });

        pimpl_->server.bind("simGetImages", [&](const std::vector<RpcLibAdaptorsBase::ImageRequest>& request_adapter, const std::string& vehicle_name) -> vector<RpcLibAdaptorsBase::ImageResponse> {
            const auto& response = getWorldSimApi()->getImages(RpcLibAdaptorsBase::ImageRequest::to(request_adapter), vehicle_name);
            return RpcLibAdaptorsBase::ImageResponse::from(response);
        });

        pimpl_->server.bind("simGetImage", [&](const std::string& camera_name, ImageCaptureBase::ImageType type, const std::string& vehicle_name, const std::string& annotation_name) -> vector<uint8_t> {
            return getWorldSimApi()->getImage(type, CameraDetails(camera_name, vehicle_name), annotation_name);
        });

        //CinemAirSim
        pimpl_->server.bind("simGetPresetLensSettings", [&](const std::string& camera_name, const std::string& vehicle_name) -> vector<string> {
            return getWorldSimApi()->getPresetLensSettings(CameraDetails(camera_name, vehicle_name));
        });

        pimpl_->server.bind("simGetLensSettings", [&](const std::string& camera_name, const std::string& vehicle_name) -> string {
            return getWorldSimApi()->getLensSettings(CameraDetails(camera_name, vehicle_name));
        });

        pimpl_->server.bind("simSetPresetLensSettings", [&](const std::string preset_lens_settings, const std::string& camera_name, const std::string& vehicle_name) -> void {
            getWorldSimApi()->setPresetLensSettings(preset_lens_settings, CameraDetails(camera_name, vehicle_name));
        });

        pimpl_->server.bind("simGetPresetFilmbackSettings", [&](const std::string& camera_name, const std::string& vehicle_name) -> vector<string> {
            return getWorldSimApi()->getPresetFilmbackSettings(CameraDetails(camera_name, vehicle_name));
        });

        pimpl_->server.bind("simSetPresetFilmbackSettings", [&](const std::string preset_filmback_settings, const std::string& camera_name, const std::string& vehicle_name) -> void {
            getWorldSimApi()->setPresetFilmbackSettings(preset_filmback_settings, CameraDetails(camera_name, vehicle_name));
        });

        pimpl_->server.bind("simGetFilmbackSettings", [&](const std::string& camera_name, const std::string& vehicle_name) -> string {
            return getWorldSimApi()->getFilmbackSettings(CameraDetails(camera_name, vehicle_name));
        });

        pimpl_->server.bind("simSetFilmbackSettings", [&](const float width, const float heigth, const std::string& camera_name, const std::string& vehicle_name) -> float {
            return getWorldSimApi()->setFilmbackSettings(width, heigth, CameraDetails(camera_name, vehicle_name));
            ;
        });

        pimpl_->server.bind("simGetFocalLength", [&](const std::string& camera_name, const std::string& vehicle_name) -> float {
            return getWorldSimApi()->getFocalLength(CameraDetails(camera_name, vehicle_name));
        });

        pimpl_->server.bind("simSetFocalLength", [&](const float focal_lenght, const std::string& camera_name, const std::string& vehicle_name) -> void {
            getWorldSimApi()->setFocalLength(focal_lenght, CameraDetails(camera_name, vehicle_name));
        });

        pimpl_->server.bind("simEnableManualFocus", [&](const bool enable, const std::string& camera_name, const std::string& vehicle_name) -> void {
            getWorldSimApi()->enableManualFocus(enable, CameraDetails(camera_name, vehicle_name));
        });

        pimpl_->server.bind("simGetFocusDistance", [&](const std::string& camera_name, const std::string& vehicle_name) -> float {
            return getWorldSimApi()->getFocusDistance(CameraDetails(camera_name, vehicle_name));
        });

        pimpl_->server.bind("simSetFocusDistance", [&](const float focus_distance, const std::string& camera_name, const std::string& vehicle_name) -> void {
            getWorldSimApi()->setFocusDistance(focus_distance, CameraDetails(camera_name, vehicle_name));
        });

        pimpl_->server.bind("simGetFocusAperture", [&](const std::string& camera_name, const std::string& vehicle_name) -> float {
            return getWorldSimApi()->getFocusAperture(CameraDetails(camera_name, vehicle_name));
        });

        pimpl_->server.bind("simSetFocusAperture", [&](const float focus_aperture, const std::string& camera_name, const std::string& vehicle_name) -> void {
            getWorldSimApi()->setFocusAperture(focus_aperture, CameraDetails(camera_name, vehicle_name));
        });

        pimpl_->server.bind("simEnableFocusPlane", [&](const bool enable, const std::string& camera_name, const std::string& vehicle_name) -> void {
            getWorldSimApi()->enableFocusPlane(enable, CameraDetails(camera_name, vehicle_name));
        });

        pimpl_->server.bind("simGetCurrentFieldOfView", [&](const std::string& camera_name, const std::string& vehicle_name) -> string {
            return getWorldSimApi()->getCurrentFieldOfView(CameraDetails(camera_name, vehicle_name));
        });
        //end CinemAirSim

        pimpl_->server.bind("simTestLineOfSightToPoint", [&](const RpcLibAdaptorsBase::GeoPoint& point, const std::string& vehicle_name) -> bool {
            return getVehicleSimApi(vehicle_name)->testLineOfSightToPoint(point.to());
        });

        pimpl_->server.bind("simTestLineOfSightBetweenPoints", [&](const RpcLibAdaptorsBase::GeoPoint& point1, const RpcLibAdaptorsBase::GeoPoint& point2) -> bool {
            return getWorldSimApi()->testLineOfSightBetweenPoints(point1.to(), point2.to());
        });

        pimpl_->server.bind("simGetWorldExtents", [&]() -> vector<RpcLibAdaptorsBase::GeoPoint> {
            std::vector<msr::airlib::GeoPoint> result = getWorldSimApi()->getWorldExtents(); // Returns vector with min, max
            std::vector<RpcLibAdaptorsBase::GeoPoint> conv_result;

            RpcLibAdaptorsBase::from(result, conv_result);
            return conv_result;
        });

        pimpl_->server.bind("simGetMeshPositionVertexBuffers", [&]() -> vector<RpcLibAdaptorsBase::MeshPositionVertexBuffersResponse> {
            const auto& response = getWorldSimApi()->getMeshPositionVertexBuffers();
            return RpcLibAdaptorsBase::MeshPositionVertexBuffersResponse::from(response);
        });

        pimpl_->server.bind("simAddVehicle", [&](const std::string& vehicle_name, const std::string& vehicle_type, const RpcLibAdaptorsBase::Pose& pose, const std::string& pawn_path) -> bool {
            return getWorldSimApi()->addVehicle(vehicle_name, vehicle_type, pose.to(), pawn_path);
        });

        pimpl_->server.bind("simSetVehiclePose", [&](const RpcLibAdaptorsBase::Pose& pose, bool ignore_collision, const std::string& vehicle_name) -> void {
            getVehicleSimApi(vehicle_name)->setPose(pose.to(), ignore_collision);
        });

        pimpl_->server.bind("simGetVehiclePose", [&](const std::string& vehicle_name) -> RpcLibAdaptorsBase::Pose {
            const auto& pose = getVehicleSimApi(vehicle_name)->getPose();
            return RpcLibAdaptorsBase::Pose(pose);
        });

        pimpl_->server.bind("simSetTraceLine", [&](const std::vector<float>& color_rgba, float thickness, const std::string& vehicle_name) -> void {
            getVehicleSimApi(vehicle_name)->setTraceLine(color_rgba, thickness);
        });

        pimpl_->server.bind("simSetSegmentationObjectID", [&](const std::string& mesh_name, int object_id, bool is_name_regex) -> bool {
            return getWorldSimApi()->setSegmentationObjectID(mesh_name, object_id, is_name_regex);
        });

        pimpl_->server.bind("simGetSegmentationObjectID", [&](const std::string& mesh_name) -> int {
            return getWorldSimApi()->getSegmentationObjectID(mesh_name);
        });

        pimpl_->server.bind("simListAnnotationObjects", [&](const std::string& annotation_name) -> std::vector<string> {
            return getWorldSimApi()->listAnnotationObjects(annotation_name);
        });

        pimpl_->server.bind("simListAnnotationPoses", [&](const std::string& annotation_name, bool ned, bool only_visible) -> std::vector<RpcLibAdaptorsBase::Pose> {
            return RpcLibAdaptorsBase::Pose::from(getWorldSimApi()->listAnnotationPoses(annotation_name, ned, only_visible));
        });

        pimpl_->server.bind("simSetAnnotationObjectID", [&](const std::string& annotation_name, const std::string& mesh_name, int object_id, bool is_name_regex) -> bool {
            return getWorldSimApi()->setAnnotationObjectID(annotation_name, mesh_name, object_id, is_name_regex);
        });

        pimpl_->server.bind("simGetAnnotationObjectID", [&](const std::string& annotation_name, const std::string& mesh_name) -> int {
            return getWorldSimApi()->getAnnotationObjectID(annotation_name, mesh_name);
        });

        pimpl_->server.bind("simSetAnnotationObjectColor", [&](const std::string& annotation_name, const std::string& mesh_name, int r, int g, int b, bool is_name_regex) -> bool {
            return getWorldSimApi()->setAnnotationObjectColor(annotation_name, mesh_name, r, g, b, is_name_regex);
        });

        pimpl_->server.bind("simGetAnnotationObjectColor", [&](const std::string& annotation_name, const std::string& mesh_name) -> std::string {
            return getWorldSimApi()->getAnnotationObjectColor(annotation_name, mesh_name);
        });

        pimpl_->server.bind("simSetAnnotationObjectValue", [&](const std::string& annotation_name, const std::string& mesh_name, float greyscale_value, bool is_name_regex) -> bool {
            return getWorldSimApi()->setAnnotationObjectValue(annotation_name, mesh_name, greyscale_value, is_name_regex);
        });

        pimpl_->server.bind("simGetAnnotationObjectValue", [&](const std::string& annotation_name, const std::string& mesh_name) -> float {
            return getWorldSimApi()->getAnnotationObjectValue(annotation_name, mesh_name);
        });

        pimpl_->server.bind("simSetAnnotationObjectTextureByPath", [&](const std::string& annotation_name, const std::string& mesh_name, const std::string& texture_path, bool is_name_regex) -> bool {
            return getWorldSimApi()->setAnnotationObjectTextureByPath(annotation_name, mesh_name, texture_path, is_name_regex);
        });

        pimpl_->server.bind("simEnableAnnotationObjectTextureByPath", [&](const std::string& annotation_name, const std::string& mesh_name, bool is_name_regex) -> bool {
            return getWorldSimApi()->enableAnnotationObjectTextureByPath(annotation_name, mesh_name, is_name_regex);
        });

        pimpl_->server.bind("simGetAnnotationObjectTexturePath", [&](const std::string& annotation_name, const std::string& mesh_name) -> std::string {
            return getWorldSimApi()->getAnnotationObjectTexturePath(annotation_name, mesh_name);
        });

        pimpl_->server.bind("simAddDetectionFilterMeshName", [&](const std::string& camera_name, ImageCaptureBase::ImageType type, const std::string& mesh_name, const std::string& vehicle_name, const std::string& annotation_name) -> void {
            getWorldSimApi()->addDetectionFilterMeshName(type, mesh_name, CameraDetails(camera_name, vehicle_name), annotation_name);
        });
        pimpl_->server.bind("simSetDetectionFilterRadius", [&](const std::string& camera_name, ImageCaptureBase::ImageType type, const float radius_cm, const std::string& vehicle_name, const std::string& annotation_name) -> void {
            getWorldSimApi()->setDetectionFilterRadius(type, radius_cm, CameraDetails(camera_name, vehicle_name), annotation_name);
        });
        pimpl_->server.bind("simClearDetectionMeshNames", [&](const std::string& camera_name, ImageCaptureBase::ImageType type, const std::string& vehicle_name, const std::string& annotation_name) -> void {
            getWorldSimApi()->clearDetectionMeshNames(type, CameraDetails(camera_name, vehicle_name), annotation_name);
        });
        pimpl_->server.bind("simGetDetections", [&](const std::string& camera_name, ImageCaptureBase::ImageType type, const std::string& vehicle_name, const std::string& annotation_name) -> vector<RpcLibAdaptorsBase::DetectionInfo> {
            const auto& response = getWorldSimApi()->getDetections(type, CameraDetails(camera_name, vehicle_name), annotation_name);
            return RpcLibAdaptorsBase::DetectionInfo::from(response);
        });
        pimpl_->server.bind("reset", [&]() -> void {
            //Exit if already resetting.
            static bool resetInProgress;
            if (resetInProgress)
                return;

            //Reset
            resetInProgress = true;
            auto* sim_world_api = getWorldSimApi();
            if (sim_world_api)
                sim_world_api->reset();
            else
                getVehicleApi("")->reset();

            resetInProgress = false;
        });

        pimpl_->server.bind("simPrintLogMessage", [&](const std::string& message, const std::string& message_param, unsigned char severity) -> void {
            getWorldSimApi()->printLogMessage(message, message_param, severity);
        });

        pimpl_->server.bind("getHomeGeoPoint", [&](const std::string& vehicle_name) -> RpcLibAdaptorsBase::GeoPoint {
            const auto& geo_point = getVehicleApi(vehicle_name)->getHomeGeoPoint();
            return RpcLibAdaptorsBase::GeoPoint(geo_point);
        });

        pimpl_->server.bind("getLidarData", [&](const std::string& lidar_name, const std::string& vehicle_name) -> RpcLibAdaptorsBase::LidarData {
            const auto& lidar_data = getVehicleApi(vehicle_name)->getLidarData(lidar_name);
            return RpcLibAdaptorsBase::LidarData(lidar_data);
        });

        pimpl_->server.bind("getImuData", [&](const std::string& imu_name, const std::string& vehicle_name) -> RpcLibAdaptorsBase::ImuData {
            const auto& imu_data = getVehicleApi(vehicle_name)->getImuData(imu_name);
            return RpcLibAdaptorsBase::ImuData(imu_data);
        });

        pimpl_->server.bind("getGPULidarData", [&](const std::string& lidar_name, const std::string& vehicle_name) -> RpcLibAdaptorsBase::GPULidarData {
		const auto& lidar_data = getVehicleApi(vehicle_name)->getGPULidarData(lidar_name);
		return RpcLibAdaptorsBase::GPULidarData(lidar_data);
        });

        pimpl_->server.bind("getEchoData", [&](const std::string& echo_name, const std::string& vehicle_name) -> RpcLibAdaptorsBase::EchoData {
            const auto& echo_data = getVehicleApi(vehicle_name)->getEchoData(echo_name);
            return RpcLibAdaptorsBase::EchoData(echo_data);
        });

        pimpl_->server.bind("setEchoData", [&](const std::string& echo_name, const std::string& vehicle_name, RpcLibAdaptorsBase::EchoData echo_data) -> void {
            getVehicleApi(vehicle_name)->setEchoData(echo_name, echo_data.to());
        });


        pimpl_->server.bind("getBarometerData", [&](const std::string& barometer_name, const std::string& vehicle_name) -> RpcLibAdaptorsBase::BarometerData {
            const auto& barometer_data = getVehicleApi(vehicle_name)->getBarometerData(barometer_name);
            return RpcLibAdaptorsBase::BarometerData(barometer_data);
        });

        pimpl_->server.bind("getMagnetometerData", [&](const std::string& magnetometer_name, const std::string& vehicle_name) -> RpcLibAdaptorsBase::MagnetometerData {
            const auto& magnetometer_data = getVehicleApi(vehicle_name)->getMagnetometerData(magnetometer_name);
            return RpcLibAdaptorsBase::MagnetometerData(magnetometer_data);
        });

        pimpl_->server.bind("getGpsData", [&](const std::string& gps_name, const std::string& vehicle_name) -> RpcLibAdaptorsBase::GpsData {
            const auto& gps_data = getVehicleApi(vehicle_name)->getGpsData(gps_name);
            return RpcLibAdaptorsBase::GpsData(gps_data);
        });

        pimpl_->server.bind("getDistanceSensorData", [&](const std::string& distance_sensor_name, const std::string& vehicle_name) -> RpcLibAdaptorsBase::DistanceSensorData {
            const auto& distance_sensor_data = getVehicleApi(vehicle_name)->getDistanceSensorData(distance_sensor_name);
            return RpcLibAdaptorsBase::DistanceSensorData(distance_sensor_data);
        });

        pimpl_->server.bind("simGetCameraInfo", [&](const std::string& camera_name, const std::string& vehicle_name) -> RpcLibAdaptorsBase::CameraInfo {
            const auto& camera_info = getWorldSimApi()->getCameraInfo(CameraDetails(camera_name, vehicle_name));
            return RpcLibAdaptorsBase::CameraInfo(camera_info);
        });

        pimpl_->server.bind("simSetDistortionParam", [&](const std::string& camera_name, const std::string& param_name, float value, const std::string& vehicle_name) -> void {
            getWorldSimApi()->setDistortionParam(param_name, value, CameraDetails(camera_name, vehicle_name));
        });

        pimpl_->server.bind("simGetDistortionParams", [&](const std::string& camera_name, const std::string& vehicle_name) -> std::vector<float> {
            return getWorldSimApi()->getDistortionParams(CameraDetails(camera_name, vehicle_name));
        });

        pimpl_->server.bind("simSetCameraPose", [&](const std::string& camera_name, const RpcLibAdaptorsBase::Pose& pose, const std::string& vehicle_name) -> void {
            getWorldSimApi()->setCameraPose(pose.to(), CameraDetails(camera_name, vehicle_name));
        });

        pimpl_->server.bind("simSetCameraFov", [&](const std::string& camera_name, float fov_degrees, const std::string& vehicle_name) -> void {
            getWorldSimApi()->setCameraFoV(fov_degrees, CameraDetails(camera_name, vehicle_name));
        });

        pimpl_->server.bind("simGetCollisionInfo", [&](const std::string& vehicle_name) -> RpcLibAdaptorsBase::CollisionInfo {
            const auto& collision_info = getVehicleSimApi(vehicle_name)->getCollisionInfoAndReset();
            return RpcLibAdaptorsBase::CollisionInfo(collision_info);
        });

        pimpl_->server.bind("simListSceneObjects", [&](const std::string& name_regex) -> std::vector<string> {
            return getWorldSimApi()->listSceneObjects(name_regex);
        });

        pimpl_->server.bind("simListSceneObjectsTags", [&](const std::string& name_regex) -> std::vector<std::pair<std::string, std::string>> {
            return getWorldSimApi()->listSceneObjectsTags(name_regex);
        });

        pimpl_->server.bind("simLoadLevel", [&](const std::string& level_name) -> bool {
            return getWorldSimApi()->loadLevel(level_name);
        });

        pimpl_->server.bind("simSpawnObject", [&](string& object_name, const string& load_component, const RpcLibAdaptorsBase::Pose& pose, const RpcLibAdaptorsBase::Vector3r& scale, bool physics_enabled, bool is_blueprint) -> string {
            return getWorldSimApi()->spawnObject(object_name, load_component, pose.to(), scale.to(), physics_enabled, is_blueprint);
        });

        pimpl_->server.bind("simDestroyObject", [&](const string& object_name) -> bool {
            return getWorldSimApi()->destroyObject(object_name);
        });

        pimpl_->server.bind("simListAssets", [&]() -> std::vector<std::string> {
            return getWorldSimApi()->listAssets();
        });

        pimpl_->server.bind("simListInstanceSegmentationObjects", [&]() -> std::vector<string> {
            return getWorldSimApi()->listInstanceSegmentationObjects();
        });

        pimpl_->server.bind("simGetInstanceSegmentationColorMap", [&]() -> std::vector<RpcLibAdaptorsBase::Vector3r> {
            return RpcLibAdaptorsBase::Vector3r::from(getWorldSimApi()->getInstanceSegmentationColorMap());
        });

        pimpl_->server.bind("simListInstanceSegmentationPoses", [&](bool ned, bool only_visible) -> std::vector<RpcLibAdaptorsBase::Pose> {
            return RpcLibAdaptorsBase::Pose::from(getWorldSimApi()->listInstanceSegmentationPoses(ned, only_visible));
        });

        pimpl_->server.bind("simGetObjectPose", [&](const std::string& object_name, bool ned) -> RpcLibAdaptorsBase::Pose {
            const auto& pose = getWorldSimApi()->getObjectPose(object_name, ned);
            return RpcLibAdaptorsBase::Pose(pose);
        });


        pimpl_->server.bind("simGetObjectScale", [&](const std::string& object_name) -> RpcLibAdaptorsBase::Vector3r {
            const auto& scale = getWorldSimApi()->getObjectScale(object_name);
            return RpcLibAdaptorsBase::Vector3r(scale);
        });

        pimpl_->server.bind("simSetObjectPose", [&](const std::string& object_name, const RpcLibAdaptorsBase::Pose& pose, bool teleport) -> bool {
            return getWorldSimApi()->setObjectPose(object_name, pose.to(), teleport);
        });

        pimpl_->server.bind("simSetObjectScale", [&](const std::string& object_name, const RpcLibAdaptorsBase::Vector3r& scale) -> bool {
            return getWorldSimApi()->setObjectScale(object_name, scale.to());
        });

        pimpl_->server.bind("simFlushPersistentMarkers", [&]() -> void {
            getWorldSimApi()->simFlushPersistentMarkers();
        });

        pimpl_->server.bind("simPlotPoints", [&](const std::vector<RpcLibAdaptorsBase::Vector3r>& points, const vector<float>& color_rgba, float size, float duration, bool is_persistent) -> void {
            vector<Vector3r> conv_points;
            RpcLibAdaptorsBase::to(points, conv_points);
            getWorldSimApi()->simPlotPoints(conv_points, color_rgba, size, duration, is_persistent);
        });

        pimpl_->server.bind("simPlotLineStrip", [&](const std::vector<RpcLibAdaptorsBase::Vector3r>& points, const vector<float>& color_rgba, float thickness, float duration, bool is_persistent) -> void {
            vector<Vector3r> conv_points;
            RpcLibAdaptorsBase::to(points, conv_points);
            getWorldSimApi()->simPlotLineStrip(conv_points, color_rgba, thickness, duration, is_persistent);
        });

        pimpl_->server.bind("simPlotLineList", [&](const std::vector<RpcLibAdaptorsBase::Vector3r>& points, const vector<float>& color_rgba, float thickness, float duration, bool is_persistent) -> void {
            vector<Vector3r> conv_points;
            RpcLibAdaptorsBase::to(points, conv_points);
            getWorldSimApi()->simPlotLineList(conv_points, color_rgba, thickness, duration, is_persistent);
        });

        pimpl_->server.bind("simPlotArrows", [&](const std::vector<RpcLibAdaptorsBase::Vector3r>& points_start, const std::vector<RpcLibAdaptorsBase::Vector3r>& points_end, const vector<float>& color_rgba, float thickness, float arrow_size, float duration, bool is_persistent) -> void {
            vector<Vector3r> conv_points_start;
            RpcLibAdaptorsBase::to(points_start, conv_points_start);
            vector<Vector3r> conv_points_end;
            RpcLibAdaptorsBase::to(points_end, conv_points_end);
            getWorldSimApi()->simPlotArrows(conv_points_start, conv_points_end, color_rgba, thickness, arrow_size, duration, is_persistent);
        });

        pimpl_->server.bind("simPlotStrings", [&](const std::vector<std::string> strings, const std::vector<RpcLibAdaptorsBase::Vector3r>& positions, float scale, const vector<float>& color_rgba, float duration) -> void {
            vector<Vector3r> conv_positions;
            RpcLibAdaptorsBase::to(positions, conv_positions);
            getWorldSimApi()->simPlotStrings(strings, conv_positions, scale, color_rgba, duration);
        });

        pimpl_->server.bind("simPlotTransforms", [&](const std::vector<RpcLibAdaptorsBase::Pose>& poses, float scale, float thickness, float duration, bool is_persistent) -> void {
            vector<Pose> conv_poses;
            RpcLibAdaptorsBase::to(poses, conv_poses);
            getWorldSimApi()->simPlotTransforms(conv_poses, scale, thickness, duration, is_persistent);
        });

        pimpl_->server.bind("simPlotTransformsWithNames", [&](const std::vector<RpcLibAdaptorsBase::Pose>& poses, const std::vector<std::string> names, float tf_scale, float tf_thickness, float text_scale, const vector<float>& text_color_rgba, float duration) -> void {
            vector<Pose> conv_poses;
            RpcLibAdaptorsBase::to(poses, conv_poses);
            getWorldSimApi()->simPlotTransformsWithNames(conv_poses, names, tf_scale, tf_thickness, text_scale, text_color_rgba, duration);
        });

        pimpl_->server.bind("simGetGroundTruthKinematics", [&](const std::string& vehicle_name) -> RpcLibAdaptorsBase::KinematicsState {
            const Kinematics::State& result = *getVehicleSimApi(vehicle_name)->getGroundTruthKinematics();
            return RpcLibAdaptorsBase::KinematicsState(result);
        });

        pimpl_->server.bind("simSetKinematics", [&](const RpcLibAdaptorsBase::KinematicsState& state, bool ignore_collision, const std::string& vehicle_name) {
            getVehicleSimApi(vehicle_name)->setKinematics(state.to(), ignore_collision);
        });

        pimpl_->server.bind("simGetPhysicsRawKinematics", [&](const std::string& vehicle_name) -> RpcLibAdaptorsBase::KinematicsState {
            const Kinematics::State result = getVehicleSimApi(vehicle_name)->getPhysicsRawKinematics();
            return RpcLibAdaptorsBase::KinematicsState(result);
        });

        pimpl_->server.bind("simSetPhysicsRawKinematics", [&](const RpcLibAdaptorsBase::KinematicsState& state, const std::string& vehicle_name) {
            getVehicleSimApi(vehicle_name)->setPhysicsRawKinematics(state.to());
        });

        pimpl_->server.bind("simGetGroundTruthEnvironment", [&](const std::string& vehicle_name) -> RpcLibAdaptorsBase::EnvironmentState {
            const Environment::State& result = (*getVehicleSimApi(vehicle_name)->getGroundTruthEnvironment()).getState();
            return RpcLibAdaptorsBase::EnvironmentState(result);
        });

        pimpl_->server.bind("simCreateVoxelGrid", [&](const RpcLibAdaptorsBase::Vector3r& position, const int& x, const int& y, const int& z, const float& res, const std::string& output_file) -> bool {
            return getWorldSimApi()->createVoxelGrid(position.to(), x, y, z, res, output_file);
        });

        pimpl_->server.bind("simSetLightIntensity", [&](const std::string& light_name, float intensity) -> bool {
            return getWorldSimApi()->setLightIntensity(light_name, intensity);
        });

        pimpl_->server.bind("getUWBData", [&](const std::string& sensor_name, const std::string& vehicle_name) -> RpcLibAdaptorsBase::MarLocUwbReturnMessage {
            const auto& marLocUwbReturnMessage = getVehicleApi(vehicle_name)->getUWBData(sensor_name);
            return RpcLibAdaptorsBase::MarLocUwbReturnMessage(marLocUwbReturnMessage);
        });

        pimpl_->server.bind("getUWBSensorData", [&](const std::string& sensor_name, const std::string& vehicle_name) -> RpcLibAdaptorsBase::MarLocUwbSensorData {
            const auto& marLocUwbSensorData = getVehicleApi(vehicle_name)->getUWBSensorData(sensor_name);
            return RpcLibAdaptorsBase::MarLocUwbSensorData(marLocUwbSensorData);
        });

        pimpl_->server.bind("getWifiData", [&](const std::string& sensor_name, const std::string& vehicle_name) -> RpcLibAdaptorsBase::WifiReturnMessage {
            const auto& wifiReturnMessage = getVehicleApi(vehicle_name)->getWifiData(sensor_name);
            return RpcLibAdaptorsBase::WifiReturnMessage(wifiReturnMessage);
        });

        pimpl_->server.bind("getWifiSensorData", [&](const std::string& sensor_name, const std::string& vehicle_name) -> RpcLibAdaptorsBase::WifiSensorData {
            const auto& wifiSensorData = getVehicleApi(vehicle_name)->getWifiSensorData(sensor_name);
            return RpcLibAdaptorsBase::WifiSensorData(wifiSensorData);
        });

        pimpl_->server.bind("cancelLastTask", [&](const std::string& vehicle_name) -> void {
            getVehicleApi(vehicle_name)->cancelLastTask();
        });

        pimpl_->server.bind("simSwapTextures", [&](const std::string tag, int tex_id, int component_id, int material_id) -> std::vector<string> {
            return *getWorldSimApi()->swapTextures(tag, tex_id, component_id, material_id);
        });

        pimpl_->server.bind("simSetObjectMaterial", [&](const std::string& object_name, const std::string& material_name, const int component_id) -> bool {
            return getWorldSimApi()->setObjectMaterial(object_name, material_name, component_id);
        });

        pimpl_->server.bind("simSetObjectMaterialFromTexture", [&](const std::string& object_name, const std::string& texture_path, const int component_id) -> bool {
            return getWorldSimApi()->setObjectMaterialFromTexture(object_name, texture_path, component_id);
        });

        pimpl_->server.bind("startRecording", [&]() -> void {
            getWorldSimApi()->startRecording();
        });

        pimpl_->server.bind("stopRecording", [&]() -> void {
            getWorldSimApi()->stopRecording();
        });

        pimpl_->server.bind("isRecording", [&]() -> bool {
            return getWorldSimApi()->isRecording();
        });

        pimpl_->server.bind("simSetWind", [&](const RpcLibAdaptorsBase::Vector3r& wind) -> void {
            getWorldSimApi()->setWind(wind.to());
        });

        pimpl_->server.bind("simSetExtForce", [&](const RpcLibAdaptorsBase::Vector3r& ext_force) -> void {
            getWorldSimApi()->setExtForce(ext_force.to());
        });

        pimpl_->server.bind("listVehicles", [&]() -> vector<string> {
            return getWorldSimApi()->listVehicles();
        });

        pimpl_->server.bind("getSettingsString", [&]() -> std::string {
            return getWorldSimApi()->getSettingsString();
        });

        //if we don't suppress then server will bomb out for exceptions raised by any method
        pimpl_->server.suppress_exceptions(true);
    }

    //required for pimpl
    RpcLibServerBase::~RpcLibServerBase()
    {
        stop();
    }

    void RpcLibServerBase::start(bool block, std::size_t thread_count)
    {
        pimpl_->run(block, thread_count);
    }

    void RpcLibServerBase::stop()
    {
        pimpl_->stop();
    }

    void* RpcLibServerBase::getServer() const
    {
        return &pimpl_->server;
    }
}
} //namespace
#endif
#endif
