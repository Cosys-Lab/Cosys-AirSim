// Copyright (c) Microsoft Corporation. All rights reserved.
// Licensed under the MIT License.

// Splits AirSimSettings into individual structure headers for better organization and faster compile times

#ifndef airsim_core_AirSimSettings_structs_hpp
#define airsim_core_AirSimSettings_structs_hpp

#include "CommonStructs.hpp"
#include "ImageCaptureBase.hpp"
#include "common_utils/Utils.hpp"
#include <map>
#include <string>
#include <vector>

namespace msr {
namespace airlib {

struct AirSimSettings
{
    static constexpr int kSubwindowCount = 3;
    static constexpr char const* kVehicleTypePX4 = "px4multirotor";
    static constexpr char const* kVehicleTypeArduCopterSolo = "arducoptersolo";
    static constexpr char const* kVehicleTypeSimpleFlight = "simpleflight";
    static constexpr char const* kVehicleTypeArduCopter = "arducopter";
    static constexpr char const* kVehicleTypePhysXCar = "physxcar";
    static constexpr char const* kVehicleTypeBoxCar = "boxcar";
    static constexpr char const* kVehicleTypeCPHusky = "cphusky";
    static constexpr char const* kVehicleTypePioneer = "pioneer";
    static constexpr char const* kVehicleTypeArduRover = "ardurover";
    static constexpr char const* kVehicleTypeComputerVision = "computervision";

    static constexpr char const* kVehicleInertialFrame = "VehicleInertialFrame";
    static constexpr char const* kSensorLocalFrame = "SensorLocalFrame";
    static constexpr char const* kBeaconTypeTemplate = "templateBeacon";
    static constexpr char const* kSimModeTypeMultirotor = "Multirotor";
    static constexpr char const* kSimModeTypeCar = "Car";
    static constexpr char const* kSimModeTypeSkidVehicle = "SkidVehicle";
    static constexpr char const* kSimModeTypeComputerVision = "ComputerVision";

    typedef ImageCaptureBase::ImageType ImageType;

    struct SubwindowSetting
    {
        int window_index;
        ImageType image_type;
        bool visible;
        std::string camera_name;
        std::string vehicle_name;
        std::string annotation_name;

        SubwindowSetting(int window_index_val = 0, ImageType image_type_val = ImageType::Scene, bool visible_val = false,
                         const std::string& camera_name_val = "", const std::string& vehicle_name_val = "", const std::string& annotation_name_val = "")
            : window_index(window_index_val)
            , image_type(image_type_val)
            , visible(visible_val)
            , camera_name(camera_name_val)
            , vehicle_name(vehicle_name_val)
            , annotation_name(annotation_name_val)
        {
        }
    };

    struct AnnotatorSetting
    {
        int annotator_index;
        int type;
        bool show_by_default;
        std::string name;
        bool set_direct;
        std::string texture_path;
        std::string texture_prefix;
        float max_view_distance;

        AnnotatorSetting(int annotator_index_val = 0, int type_val = 0, bool show_by_default_val = true,
            const std::string& name_val = "", bool set_direct_val = false, std::string texture_path_val = "", std::string texture_prefix_val = "", float max_view_distance_val = -1.0f)
            : annotator_index(annotator_index_val)
            , type(type_val)
            , show_by_default(show_by_default_val)
            , name(name_val)
            , set_direct(set_direct_val)
            , texture_path(texture_path_val)
            , texture_prefix(texture_prefix_val)
            , max_view_distance(max_view_distance_val)
        {
        }
    };

    struct RecordingSetting
    {
        bool record_on_move = false;
        float record_interval = 0.05f;
        std::string folder = "";
        bool enabled = false;

        std::map<std::string, std::vector<ImageCaptureBase::ImageRequest>> requests;

        RecordingSetting()
        {
        }

        RecordingSetting(bool record_on_move_val, float record_interval_val, const std::string& folder_val, bool enabled_val)
            : record_on_move(record_on_move_val), record_interval(record_interval_val), folder(folder_val), enabled(enabled_val)
        {
        }
    };

    struct PawnPath
    {
        std::string pawn_bp;
        std::string slippery_mat;
        std::string non_slippery_mat;

        PawnPath(const std::string& pawn_bp_val = "",
                 const std::string& slippery_mat_val = "/AirSim/VehicleAdv/PhysicsMaterials/Slippery.Slippery",
                 const std::string& non_slippery_mat_val = "/AirSim/VehicleAdv/PhysicsMaterials/NonSlippery.NonSlippery")
            : pawn_bp(pawn_bp_val), slippery_mat(slippery_mat_val), non_slippery_mat(non_slippery_mat_val)
        {
        }
    };

    struct RCSettings
    {
        int remote_control_id = -1;
        bool allow_api_when_disconnected = false;
    };

    struct Rotation
    {
        float yaw = 0;
        float pitch = 0;
        float roll = 0;

        Rotation()
        {
        }

        Rotation(float yaw_val, float pitch_val, float roll_val)
            : yaw(yaw_val), pitch(pitch_val), roll(roll_val)
        {
        }

        static Rotation nanRotation() noexcept;
    };

    struct GimbalSetting
    {
        float stabilization = 0;
        Rotation rotation = Rotation::nanRotation();
    };

    struct CaptureSetting
    {
        static constexpr float kSceneTargetGamma = 1.4f;

        int image_type = 0;
        bool force_update = false;
        unsigned int width = 256, height = 144;
        float fov_degrees = Utils::nan<float>();
        float target_gamma = Utils::nan<float>();
        int projection_mode = 0;
        float ortho_width = Utils::nan<float>();
        float motion_blur_amount = Utils::nan<float>();
        float motion_blur_max = Utils::nan<float>();
        float motion_blur_target_fps = Utils::nan<float>();
        float bloom_intensity = Utils::nan<float>();
        float bloom_threshold = Utils::nan<float>();
        int auto_exposure_method = -1;
        float auto_exposure_bias = Utils::nan<float>();
        bool auto_exposure_apply_physical_camera_exposure = true;
        float auto_exposure_min_brightness = Utils::nan<float>();
        float auto_exposure_max_brightness = Utils::nan<float>();
        float auto_exposure_speed_up = Utils::nan<float>();
        float auto_exposure_speed_down = Utils::nan<float>();
        float auto_exposure_low_percent = Utils::nan<float>();
    };

    struct VideoCaptureSetting
    {
        bool enabled = false;
        std::string filepath = "";
        unsigned int fps = 20, bitrate = 400000000;
    };

    struct ClockSetting
    {
        static constexpr char const* kSystemClockMode = "SystemClock";
        static constexpr char const* kStepableClockMode = "StepableClock";
        static constexpr char const* kVariableClockMode = "VariableClockWithCustomStartTime";

        float time_scale = 1.0f;
        bool speed_as_realtime = 0;
    };

    struct NotificationSetting
    {
        std::string message = "";
        std::string message_type = "Line";
        bool enabled = true;
    };

    struct SegmentationSetting
    {
        static constexpr int kDefaultSingleChannel = 0;
        static constexpr int kInstanced = 1;
        static constexpr int kRGBColor = 2;

        int init_method = kDefaultSingleChannel;
        int updated_physical_material = 0;
        int black_value = 0;
        float start_normalized = 0.0f;
        float height = 0.0f;
    };

    struct ClockSpeed
    {
        double explicit_time = 0;
        double multiplier = 1.0f;
        std::string custom_time = "";
    };
};

}
}

#endif