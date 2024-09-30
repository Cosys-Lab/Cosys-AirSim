// Copyright (c) Microsoft Corporation. All rights reserved.
// Licensed under the MIT License.

#ifndef airsim_core_AirSimSettings_hpp
#define airsim_core_AirSimSettings_hpp

#include "CommonStructs.hpp"
#include "ImageCaptureBase.hpp"
#include "Settings.hpp"
#include "common_utils/Utils.hpp"
#include "sensors/SensorBase.hpp"
#include <exception>
#include <functional>
#include <map>
#include <string>
#include <vector>

namespace msr
{
namespace airlib
{

    struct AirSimSettings
    {
    private:
        typedef common_utils::Utils Utils;
        typedef ImageCaptureBase::ImageType ImageType;

    public: //types
        static constexpr int kSubwindowCount = 3; //must be >= 3 for now
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

            static Rotation nanRotation() noexcept
            {
                static const Rotation val(Utils::nan<float>(), Utils::nan<float>(), Utils::nan<float>());
                return val;
            }
        };

        struct GimbalSetting
        {
            float stabilization = 0;
            //bool is_world_frame = false;
            Rotation rotation = Rotation::nanRotation();
        };

        struct CaptureSetting
        {
            //below settings_json are obtained by using Unreal console command (press ~):
            // ShowFlag.VisualizeHDR 1.
            //to replicate camera settings_json to SceneCapture2D
            //TODO: should we use UAirBlueprintLib::GetDisplayGamma()?
            static constexpr float kSceneTargetGamma = 1.4f;

            int image_type = 0;

            unsigned int width = 256, height = 144; //960 X 540
            float fov_degrees = Utils::nan<float>(); //90.0f
            int auto_exposure_method = -1; //histogram
            float auto_exposure_speed = Utils::nan<float>(); // 100.0f;
            float auto_exposure_bias = Utils::nan<float>(); // 0;
            float auto_exposure_max_brightness = Utils::nan<float>(); // 0.64f;
            float auto_exposure_min_brightness = Utils::nan<float>(); // 0.03f;
            float auto_exposure_low_percent = Utils::nan<float>(); // 80.0f;
            float auto_exposure_high_percent = Utils::nan<float>(); // 98.3f;
            float auto_exposure_histogram_log_min = Utils::nan<float>(); // -8;
            float auto_exposure_histogram_log_max = Utils::nan<float>(); // 4;
            float motion_blur_amount = Utils::nan<float>();
            float motion_blur_max = Utils::nan<float>(); // 5f;
            float target_gamma = Utils::nan<float>(); //1.0f; //This would be reset to kSceneTargetGamma for scene as default
            int projection_mode = 0; // ECameraProjectionMode::Perspective
            float ortho_width = Utils::nan<float>();
            float chromatic_aberration_scale = Utils::nan<float>(); // 0f;
		    bool ignore_marked = false;

            // Settings only available for scene camera
            bool lumen_gi_enabled = false;
            bool lumen_reflections_enabled = false;
            float lumen_final_quality = 1;
            float lumen_scene_detail = 1.0f;
            float lumen_scene_lightning_quality = 1;
        };

        struct NoiseSetting
        {
            int ImageType = 0;

            bool Enabled = false;

            float RandContrib = 0.2f;
            float RandSpeed = 100000.0f;
            float RandSize = 500.0f;
            float RandDensity = 2.0f;

            float HorzWaveContrib = 0.03f;
            float HorzWaveStrength = 0.08f;
            float HorzWaveVertSize = 1.0f;
            float HorzWaveScreenSize = 1.0f;

            float HorzNoiseLinesContrib = 1.0f;
            float HorzNoiseLinesDensityY = 0.01f;
            float HorzNoiseLinesDensityXY = 0.5f;

            float HorzDistortionContrib = 1.0f;
            float HorzDistortionStrength = 0.002f;

            bool LensDistortionEnable = false;
            float LensDistortionAreaFalloff = 1.0f;
            float LensDistortionAreaRadius = 1.0f;
            float LensDistortionIntensity = 0.5f;
            bool LensDistortionInvert = false;
        };

        struct PixelFormatOverrideSetting
        {
            int pixel_format = 0;
        };

        struct UnrealEngineSetting
        {
            std::map<int, PixelFormatOverrideSetting> pixel_format_override_settings;
        };

        using CaptureSettingsMap = std::map<int, CaptureSetting>;
        using NoiseSettingsMap = std::map<int, NoiseSetting>;
        struct CameraSetting
        {
            //nan means keep the default values set in components
            Vector3r position = VectorMath::nanVector();
            Rotation rotation = Rotation::nanRotation();

            bool external = false;                            // define if a sensor is attached to the vehicle itself(false), or to the world and is an external sensor (true)
            bool external_ned = true;                         // define if the external sensor coordinates should be reported back by the API in local NED or Unreal coordinates
            bool draw_sensor = false;

            GimbalSetting gimbal;
            CaptureSettingsMap capture_settings;
            NoiseSettingsMap noise_settings;

            UnrealEngineSetting ue_setting;

            CameraSetting()
            {
                initializeCaptureSettings(capture_settings);
                initializeNoiseSettings(noise_settings);
            }
        };
        using CameraSettingMap = std::map<std::string, CameraSetting>;

        struct CameraDirectorSetting
        {
            Vector3r position = VectorMath::nanVector();
            Rotation rotation = Rotation::nanRotation();
            float follow_distance = Utils::nan<float>();
        };

        struct SensorSetting
        {
            SensorBase::SensorType sensor_type;
            std::string sensor_name;
            bool enabled = true;
            Settings settings; // imported json data that needs to be parsed by specific sensors.
        };

        struct BarometerSetting : SensorSetting
        {
        };

        struct ImuSetting : SensorSetting
        {
        };

        struct GpsSetting : SensorSetting
        {
        };

        struct MagnetometerSetting : SensorSetting
        {
        };

        struct DistanceSetting : SensorSetting
        {
        };

        struct LidarSetting : SensorSetting
        {
        };

        struct GPULidarSetting : SensorSetting
        {
        };

	    struct EchoSetting : SensorSetting
        {
        };

        struct SensorTemplateSetting : SensorSetting
        {
        };

        struct MarLocUwbSetting : SensorSetting
        {
        };

        struct WifiSetting : SensorSetting
        {
        };

        struct VehicleSetting
        {
            //required
            std::string vehicle_name;
            std::string vehicle_type;

            //optional
            std::string default_vehicle_state;
            std::string pawn_path;
            bool allow_api_always = true;
            bool auto_create = true;
            bool enable_collision_passthrough = false;
            bool enable_trace = false;
            bool enable_collisions = true;
            bool is_fpv_vehicle = false;

            //nan means use player start
            Vector3r position = VectorMath::nanVector(); //in global NED
            Rotation rotation = Rotation::nanRotation();

            CameraSettingMap cameras;
            std::map<std::string, std::shared_ptr<SensorSetting>> sensors;

            RCSettings rc;

            VehicleSetting()
            {
            }

            VehicleSetting(const std::string& vehicle_name_val, const std::string& vehicle_type_val)
                : vehicle_name(vehicle_name_val), vehicle_type(vehicle_type_val)
            {
            }
    };

    struct BeaconSetting {
        //required
        std::string beacon_name;
        std::string beacon_type;
        std::string beacon_pawn_name;

        //optional
        std::string default_beacon_state;
        std::string pawn_path;
        bool allow_api_always = true;
        bool auto_create = true;
        bool enable_collision_passthrough = false;
        bool enable_trace = false;
        bool enable_collisions = true;
        bool is_fpv_vehicle = false;
        float debug_symbol_scale = 0.0f;

        //nan means use player start
        Vector3r position = VectorMath::nanVector(); //in global NED
        Rotation rotation = Rotation::nanRotation();

        std::map<std::string, CameraSetting> cameras;
        std::map<std::string, std::unique_ptr<SensorSetting>> sensors;
        std::vector<std::pair<std::string, std::string>> collision_blacklist;
        RCSettings rc;

        float scale = 1.0f;
    };

    struct PassiveEchoBeaconSetting {
        //required
        std::string name;
        bool enable = true;
        int initial_directions = 1000;
        float initial_lower_azimuth_limit = -90;
        float initial_upper_azimuth_limit = 90;
        float initial_lower_elevation_limit = -90;
        float initial_upper_elevation_limit = 90;
        float attenuation_limit = -100;
        float reflection_distance_limit = 0.4f;
        bool reflection_only_final = false;
        float attenuation_per_distance = 0;
        float attenuation_per_reflection = 0;
        float distance_limit = 3;
        int reflection_limit = 3;
        bool draw_debug_location = false;
        bool draw_debug_all_points = false;
        bool draw_debug_all_lines = false;
        float draw_debug_duration = -1.f;

        //nan means use player start
        Vector3r position = VectorMath::nanVector(); //in global NED
        Rotation rotation = Rotation::nanRotation();
    };

        struct MavLinkConnectionInfo
        {
            /* Default values are requires so uninitialized instance doesn't have random values */

            bool use_serial = true; // false means use UDP or TCP instead

            //Used to connect via HITL: needed only if use_serial = true
            std::string serial_port = "*";
            int baud_rate = 115200;

            // Used to connect to drone over UDP: needed only if use_serial = false and use_tcp == false
            std::string udp_address = "127.0.0.1";
            int udp_port = 14560;

            // Used to accept connections from drone over TCP: needed only if use_tcp = true
            bool lock_step = true;
            bool use_tcp = false;
            int tcp_port = 4560;

            // The PX4 SITL app requires receiving drone commands over a different mavlink channel called
            // the "ground control station" (or GCS) channel.
            std::string control_ip_address = "127.0.0.1";
            int control_port_local = 14540;
            int control_port_remote = 14580;

            // The log viewer can be on a different machine, so you can configure it's ip address and port here.
            int logviewer_ip_port = 14388;
            int logviewer_ip_sport = 14389; // for logging all messages we send to the vehicle.
            std::string logviewer_ip_address = "";

            // The QGroundControl app can be on a different machine, and AirSim can act as a proxy to forward
            // the mavlink stream over to that machine if you configure it's ip address and port here.
            int qgc_ip_port = 0;
            std::string qgc_ip_address = "";

            // mavlink vehicle identifiers
            uint8_t sim_sysid = 142;
            int sim_compid = 42;
            uint8_t offboard_sysid = 134;
            int offboard_compid = 1;
            uint8_t vehicle_sysid = 135;
            int vehicle_compid = 1;

            // if you want to select a specific local network adapter so you can reach certain remote machines (e.g. wifi versus ethernet)
            // then you will want to change the LocalHostIp accordingly.  This default only works when log viewer and QGC are also on the
            // same machine.  Whatever network you choose it has to be the same one for external
            std::string local_host_ip = "127.0.0.1";

            std::string model = "Generic";

            std::map<std::string, float> params;
            std::string logs;
        };

        struct MavLinkVehicleSetting : public VehicleSetting
        {
            MavLinkConnectionInfo connection_info;
        };

        struct TimeOfDaySetting
        {
            bool enabled = false;
            std::string start_datetime = ""; //format: %Y-%m-%d %H:%M:%S
            bool is_start_datetime_dst = false;
            float celestial_clock_speed = 1;
            float update_interval_secs = 60;
            bool move_sun = true;
        };

    private: //fields
        float settings_version_actual;
        float settings_version_minimum = 2.0f;

    public: //fields
        std::string simmode_name = "";
        std::string level_name = "";

        std::vector<SubwindowSetting> subwindow_settings;
        RecordingSetting recording_setting;
        TimeOfDaySetting tod_setting;
        std::vector<AnnotatorSetting> annotator_settings;

        std::vector<std::string> warning_messages;
        std::vector<std::string> error_messages;

        bool is_record_ui_visible = false;
        int initial_view_mode = 2; //ECameraDirectorMode::CAMERA_DIRECTOR_MODE_FLY_WITH_ME
        bool enable_rpc = true;
        std::string api_server_address = "";
        int api_port = RpcLibPort;
        std::string physics_engine_name = "";

        std::string clock_type = "";
        float clock_speed = 1.0f;
        bool engine_sound = false;
        bool move_world_origin = false;
        bool initial_instance_segmentation = true;
        bool log_messages_visible = true;
        bool show_los_debug_lines_ = false;
        HomeGeoPoint origin_geopoint{ GeoPoint(47.641468, -122.140165, 122) }; //The geo-coordinate assigned to Unreal coordinate 0,0,0
        std::map<std::string, PawnPath> pawn_paths; //path for pawn blueprint
        std::map<std::string, std::unique_ptr<VehicleSetting>> vehicles;
        CameraSetting camera_defaults;
        CameraDirectorSetting camera_director;
        std::map<std::string, std::unique_ptr<BeaconSetting>> beacons;
        std::map<std::string, std::unique_ptr<PassiveEchoBeaconSetting>> passive_echo_beacons;
        float speed_unit_factor = 1.0f;
        std::string speed_unit_label = "m\\s";
        std::map<std::string, std::shared_ptr<SensorSetting>> sensor_defaults;
        Vector3r wind = Vector3r::Zero();
        Vector3r ext_force = Vector3r::Zero();
        std::string material_list_file = "";
        std::string settings_text_ = "";

    public: //methods
        static AirSimSettings& singleton()
        {
            static AirSimSettings instance;
            return instance;
        }

        AirSimSettings()
        {
            initializeAnnotatorSettings(annotator_settings);
            initializeSubwindowSettings(subwindow_settings);
            initializePawnPaths(pawn_paths);
        }

        //returns number of warnings
        void load(std::function<std::string(void)> simmode_getter)
        {
            warning_messages.clear();
            error_messages.clear();
            const Settings& settings_json = Settings::singleton();
            checkSettingsVersion(settings_json);

            loadCoreSimModeSettings(settings_json, simmode_getter);
            loadLevelSettings(settings_json);
            loadDefaultCameraSetting(settings_json, camera_defaults);
            loadCameraDirectorSetting(settings_json, camera_director, simmode_name);
            loadSubWindowsSettings(settings_json, subwindow_settings);
            loadAnnotatorSettings(settings_json, annotator_settings);
            loadViewModeSettings(settings_json);
            loadPawnPaths(settings_json, pawn_paths);
            loadOtherSettings(settings_json);
            loadDefaultSensorSettings(simmode_name, settings_json, sensor_defaults);
            loadVehicleSettings(simmode_name, settings_json, vehicles, sensor_defaults, camera_defaults);
            loadBeaconSettings(simmode_name, settings_json, beacons);
            loadPassiveEchoBeaconSettings(settings_json, passive_echo_beacons);

            //this should be done last because it depends on vehicles (and/or their type) we have
            loadRecordingSetting(settings_json);
            loadClockSettings(settings_json);
        }

        static void initializeSettings(const std::string& json_settings_text)
        {
            singleton().settings_text_ = json_settings_text;
            Settings& settings_json = Settings::loadJSonString(json_settings_text);
            if (!settings_json.isLoadSuccess())
                throw std::invalid_argument("Cannot parse JSON settings_json string.");
        }

        static void createDefaultSettingsFile()
        {
            initializeSettings("{}");

            Settings& settings_json = Settings::singleton();
            //write some settings_json in new file otherwise the string "null" is written if all settings_json are empty
            settings_json.setString("SeeDocsAt", "https://cosys-lab.github.io/settings/");
            settings_json.setDouble("SettingsVersion", 2.0);

            std::string settings_filename = Settings::getUserDirectoryFullPath("settings.json");
            //TODO: there is a crash in Linux due to settings_json.saveJSonString(). Remove this workaround after we only support Unreal 4.17
            //https://answers.unrealengine.com/questions/664905/unreal-crashes-on-two-lines-of-extremely-simple-st.html
            settings_json.saveJSonFile(settings_filename);
        }

        // This is for the case when a new vehicle is made on the fly, at runtime
        void addVehicleSetting(const std::string& vehicle_name, const std::string& vehicle_type, const Pose& pose, const std::string& pawn_path = "")
        {
            // No Mavlink-type vehicles currently
            auto vehicle_setting = std::unique_ptr<VehicleSetting>(new VehicleSetting(vehicle_name, vehicle_type));
            vehicle_setting->position = pose.position;
            vehicle_setting->pawn_path = pawn_path;

            vehicle_setting->sensors = sensor_defaults;

            VectorMath::toEulerianAngle(pose.orientation, vehicle_setting->rotation.pitch, vehicle_setting->rotation.roll, vehicle_setting->rotation.yaw);

            vehicles[vehicle_name] = std::move(vehicle_setting);
        }

        const VehicleSetting* getVehicleSetting(const std::string& vehicle_name) const
        {
            auto it = vehicles.find(vehicle_name);
            if (it == vehicles.end())
                // pre-existing flying pawns in Unreal Engine don't have name 'SimpleFlight'
                it = vehicles.find("SimpleFlight");
            return it->second.get();
        }

        static Vector3r createVectorSetting(const Settings& settings_json, const Vector3r& default_vec)
        {
            return Vector3r(settings_json.getFloat("X", default_vec.x()),
                            settings_json.getFloat("Y", default_vec.y()),
                            settings_json.getFloat("Z", default_vec.z()));
        }
        static Rotation createRotationSetting(const Settings& settings_json, const Rotation& default_rot)
        {
            return Rotation(settings_json.getFloat("Yaw", default_rot.yaw),
                            settings_json.getFloat("Pitch", default_rot.pitch),
                            settings_json.getFloat("Roll", default_rot.roll));
        }

    private:
        void checkSettingsVersion(const Settings& settings_json)
        {
            bool has_default_settings = hasDefaultSettings(settings_json, settings_version_actual);
            bool upgrade_required = settings_version_actual < settings_version_minimum;
            if (upgrade_required) {
                bool auto_upgrade = false;

                //if we have default setting file not modified by user then we will
                //just auto-upgrade it
                if (has_default_settings) {
                    auto_upgrade = true;
                }
                else {
                    //check if auto-upgrade is possible
                    if (settings_version_actual == 1) {
                        const std::vector<std::string> all_changed_keys = {
                            "AdditionalCameras", "CaptureSettings", "NoiseSettings", "UsageScenario", "SimpleFlight", "PX4"
                        };
                        std::stringstream detected_keys_ss;
                        for (const auto& changed_key : all_changed_keys) {
                            if (settings_json.hasKey(changed_key))
                                detected_keys_ss << changed_key << ",";
                        }
                        std::string detected_keys = detected_keys_ss.str();
                        if (detected_keys.length()) {
                            std::string error_message =
                                "You are using newer version of AirSim with older version of settings.json. "
                                "You can either delete your settings.json and restart AirSim or use the upgrade "
                                "instructions at https://git.io/vjefh. \n\n"
                                "Following keys in your settings.json needs updating: ";

                            error_messages.push_back(error_message + detected_keys);
                        }
                        else
                            auto_upgrade = true;
                    }
                    else
                        auto_upgrade = true;
                }

                if (auto_upgrade) {
                    warning_messages.push_back(
                        "You are using newer version of AirSim with older version of settings.json. "
                        "You should delete your settings.json file and restart AirSim.");
                }
            }
            //else no action necessary
        }

        bool hasDefaultSettings(const Settings& settings_json, float& version)
        {
            //if empty settings file
            bool has_default = settings_json.size() == 0;

            bool has_docs = settings_json.getString("SeeDocsAt", "") != "" || settings_json.getString("see_docs_at", "") != "";
            //we had spelling mistake so we are currently supporting SettingsVersion or SettingdVersion :(
            version = settings_json.getFloat("SettingsVersion", settings_json.getFloat("SettingdVersion", 0));

            //If we have pre-V1 settings and only element is docs link
            has_default |= settings_json.size() == 1 && has_docs;

            //if we have V1 settings and only elements are docs link and version
            has_default |= settings_json.size() == 2 && has_docs && version > 0;

            return has_default;
        }

        void loadCoreSimModeSettings(const Settings& settings_json, std::function<std::string(void)> simmode_getter)
        {
            //get the simmode from user if not specified
            simmode_name = settings_json.getString("SimMode", "");
            if (simmode_name == "") {
                msr::airlib::Settings vehicles_child;
                if (simmode_getter)
                    simmode_name = simmode_getter();
                else
                    throw std::invalid_argument("simmode_name is not expected empty in SimModeBase");
            }

            physics_engine_name = settings_json.getString("PhysicsEngineName", "");
            if (physics_engine_name == "") {
                if (simmode_name == kSimModeTypeMultirotor)
                    physics_engine_name = "FastPhysicsEngine";
                else
                    physics_engine_name = "PhysX"; //this value is only informational for now
            }

            material_list_file = getMaterialListFile();
        }

        std::string getMaterialListFile()
        {

            if (FILE* file = fopen(msr::airlib::Settings::getExecutableFullPath("materials.csv").c_str(), "r")) {
                fclose(file);
                return material_list_file = msr::airlib::Settings::getExecutableFullPath("materials.csv");
            }
            else {
                return material_list_file = msr::airlib::Settings::Settings::getUserDirectoryFullPath("materials.csv");
            }
        }

        void loadLevelSettings(const Settings& settings_json)
        {
            level_name = settings_json.getString("Default Environment", "");
        }

        void loadViewModeSettings(const Settings& settings_json)
        {
            std::string view_mode_string = settings_json.getString("ViewMode", "");

            if (view_mode_string == "") {
                if (simmode_name == kSimModeTypeMultirotor)
                    view_mode_string = "FlyWithMe";
                else if (simmode_name == kSimModeTypeComputerVision)
                    view_mode_string = "Fpv";
                else
                    view_mode_string = "SpringArmChase";
            }

            if (view_mode_string == "Fpv")
                initial_view_mode = 0; // ECameraDirectorMode::CAMERA_DIRECTOR_MODE_FPV;
            else if (view_mode_string == "GroundObserver")
                initial_view_mode = 1; // ECameraDirectorMode::CAMERA_DIRECTOR_MODE_GROUND_OBSERVER;
            else if (view_mode_string == "FlyWithMe")
                initial_view_mode = 2; //ECameraDirectorMode::CAMERA_DIRECTOR_MODE_FLY_WITH_ME;
            else if (view_mode_string == "Manual")
                initial_view_mode = 3; // ECameraDirectorMode::CAMERA_DIRECTOR_MODE_MANUAL;
            else if (view_mode_string == "SpringArmChase")
                initial_view_mode = 4; // ECameraDirectorMode::CAMERA_DIRECTOR_MODE_SPRINGARM_CHASE;
            else if (view_mode_string == "Backup")
                initial_view_mode = 5; // ECameraDirectorMode::CAMREA_DIRECTOR_MODE_BACKUP;
            else if (view_mode_string == "NoDisplay")
                initial_view_mode = 6; // ECameraDirectorMode::CAMREA_DIRECTOR_MODE_NODISPLAY;
            else if (view_mode_string == "Front")
                initial_view_mode = 7; // ECameraDirectorMode::CAMREA_DIRECTOR_MODE_FRONT;
            else
                error_messages.push_back("ViewMode setting is not recognized: " + view_mode_string);
        }

        static void loadRCSetting(const std::string& simmode_name, const Settings& settings_json, RCSettings& rc_setting)
        {
            Settings rc_json;
            if (settings_json.getChild("RC", rc_json)) {
                rc_setting.remote_control_id = rc_json.getInt("RemoteControlID",
                                                              simmode_name == kSimModeTypeMultirotor ? 0 : -1);
                rc_setting.allow_api_when_disconnected = rc_json.getBool("AllowAPIWhenDisconnected",
                                                                         rc_setting.allow_api_when_disconnected);
            }
        }

        static std::string getCameraName(const Settings& settings_json)
        {
            return settings_json.getString("CameraName",
                                           //TODO: below exist only due to legacy reason and can be replaced by "" in future
                                           std::to_string(settings_json.getInt("CameraID", 0)));
        }

        void loadDefaultRecordingSettings()
        {
            recording_setting.requests.clear();
            // Add Scene image for each vehicle
            for (const auto& vehicle : vehicles) {
                recording_setting.requests[vehicle.first].push_back(ImageCaptureBase::ImageRequest(
                    "", ImageType::Scene, false, true));
            }
        }

        void loadRecordingSetting(const Settings& settings_json)
        {
            loadDefaultRecordingSettings();

            Settings recording_json;
            if (settings_json.getChild("Recording", recording_json)) {
                recording_setting.record_on_move = recording_json.getBool("RecordOnMove", recording_setting.record_on_move);
                recording_setting.record_interval = recording_json.getFloat("RecordInterval", recording_setting.record_interval);
                recording_setting.folder = recording_json.getString("Folder", recording_setting.folder);
                recording_setting.enabled = recording_json.getBool("Enabled", recording_setting.enabled);

                Settings req_cameras_settings;
                if (recording_json.getChild("Cameras", req_cameras_settings)) {
                    // If 'Cameras' field is present, clear defaults
                    recording_setting.requests.clear();
                    // Get name of the default vehicle to be used if "VehicleName" isn't specified
                    // Map contains a default vehicle if vehicles haven't been specified
                    std::string default_vehicle_name = vehicles.begin()->first;

                    for (size_t child_index = 0; child_index < req_cameras_settings.size(); ++child_index) {
                        Settings req_camera_settings;

                        if (req_cameras_settings.getChild(child_index, req_camera_settings)) {
                            std::string camera_name = getCameraName(req_camera_settings);
                            ImageType image_type = Utils::toEnum<ImageType>(
                                req_camera_settings.getInt("ImageType", 0));
                            bool compress = req_camera_settings.getBool("Compress", true);
                            bool pixels_as_float = req_camera_settings.getBool("PixelsAsFloat", false);
                            std::string vehicle_name = req_camera_settings.getString("VehicleName", default_vehicle_name);
                            std::string annotation_name = req_camera_settings.getString("Annotation", "");
                            recording_setting.requests[vehicle_name].push_back(ImageCaptureBase::ImageRequest(
                                camera_name, image_type, pixels_as_float, compress, annotation_name));
                        }
                    }
                }
            }
        }

        static void initializeCaptureSettings(CaptureSettingsMap& capture_settings)
        {
            capture_settings.clear();
            for (int i = -1; i < Utils::toNumeric(ImageType::Count); ++i) {
                capture_settings[i] = CaptureSetting();
            }
            capture_settings.at(Utils::toNumeric(ImageType::Scene)).target_gamma = CaptureSetting::kSceneTargetGamma;
        }

        static void loadCaptureSettings(const Settings& settings_json, CaptureSettingsMap& capture_settings)
        {
            // We don't call initializeCaptureSettings here since it's already called in CameraSettings constructor
            // And to avoid overwriting any defaults already set from CameraDefaults

            Settings json_parent;
            if (settings_json.getChild("CaptureSettings", json_parent)) {
                for (size_t child_index = 0; child_index < json_parent.size(); ++child_index) {
                    Settings json_settings_child;
                    if (json_parent.getChild(child_index, json_settings_child)) {
                        CaptureSetting capture_setting;
                        createCaptureSettings(json_settings_child, capture_setting);
                        capture_settings[capture_setting.image_type] = capture_setting;
                    }
                }
            }
        }

        static std::unique_ptr<VehicleSetting> createMavLinkVehicleSetting(const Settings& settings_json)
        {
            //these settings_json are expected in same section, not in another child
            std::unique_ptr<VehicleSetting> vehicle_setting_p = std::unique_ptr<VehicleSetting>(new MavLinkVehicleSetting());
            MavLinkVehicleSetting* vehicle_setting = static_cast<MavLinkVehicleSetting*>(vehicle_setting_p.get());

            //TODO: we should be selecting remote if available else keyboard
            //currently keyboard is not supported so use rc as default
            vehicle_setting->rc.remote_control_id = 0;

            MavLinkConnectionInfo& connection_info = vehicle_setting->connection_info;
            connection_info.sim_sysid = static_cast<uint8_t>(settings_json.getInt("SimSysID", connection_info.sim_sysid));
            connection_info.sim_compid = settings_json.getInt("SimCompID", connection_info.sim_compid);

            connection_info.vehicle_sysid = static_cast<uint8_t>(settings_json.getInt("VehicleSysID", connection_info.vehicle_sysid));
            connection_info.vehicle_compid = settings_json.getInt("VehicleCompID", connection_info.vehicle_compid);

            connection_info.offboard_sysid = static_cast<uint8_t>(settings_json.getInt("OffboardSysID", connection_info.offboard_sysid));
            connection_info.offboard_compid = settings_json.getInt("OffboardCompID", connection_info.offboard_compid);

            connection_info.logviewer_ip_address = settings_json.getString("LogViewerHostIp", connection_info.logviewer_ip_address);
            connection_info.logviewer_ip_port = settings_json.getInt("LogViewerPort", connection_info.logviewer_ip_port);
            connection_info.logviewer_ip_sport = settings_json.getInt("LogViewerSendPort", connection_info.logviewer_ip_sport);

            connection_info.qgc_ip_address = settings_json.getString("QgcHostIp", connection_info.qgc_ip_address);
            connection_info.qgc_ip_port = settings_json.getInt("QgcPort", connection_info.qgc_ip_port);

            connection_info.control_ip_address = settings_json.getString("ControlIp", connection_info.control_ip_address);
            connection_info.control_port_local = settings_json.getInt("ControlPort", connection_info.control_port_local); // legacy
            connection_info.control_port_local = settings_json.getInt("ControlPortLocal", connection_info.control_port_local);
            connection_info.control_port_remote = settings_json.getInt("ControlPortRemote", connection_info.control_port_remote);

            std::string sitlip = settings_json.getString("SitlIp", connection_info.control_ip_address);
            if (sitlip.size() > 0 && connection_info.control_ip_address.size() == 0) {
                // backwards compat
                connection_info.control_ip_address = sitlip;
            }
            if (settings_json.hasKey("SitlPort")) {
                // backwards compat
                connection_info.control_port_local = settings_json.getInt("SitlPort", connection_info.control_port_local);
            }

            connection_info.local_host_ip = settings_json.getString("LocalHostIp", connection_info.local_host_ip);

            connection_info.use_serial = settings_json.getBool("UseSerial", connection_info.use_serial);
            connection_info.udp_address = settings_json.getString("UdpIp", connection_info.udp_address);
            connection_info.udp_port = settings_json.getInt("UdpPort", connection_info.udp_port);
            connection_info.use_tcp = settings_json.getBool("UseTcp", connection_info.use_tcp);
            connection_info.lock_step = settings_json.getBool("LockStep", connection_info.lock_step);
            connection_info.tcp_port = settings_json.getInt("TcpPort", connection_info.tcp_port);
            connection_info.serial_port = settings_json.getString("SerialPort", connection_info.serial_port);
            connection_info.baud_rate = settings_json.getInt("SerialBaudRate", connection_info.baud_rate);
            connection_info.model = settings_json.getString("Model", connection_info.model);
            connection_info.logs = settings_json.getString("Logs", connection_info.logs);

            Settings params;
            if (settings_json.getChild("Parameters", params)) {
                std::vector<std::string> keys;
                params.getChildNames(keys);
                for (auto key : keys) {
                    connection_info.params[key] = params.getFloat(key, 0);
                }
            }

            return vehicle_setting_p;
        }

        static std::unique_ptr<VehicleSetting> createVehicleSetting(const std::string& simmode_name, const Settings& settings_json,
                                                                    const std::string vehicle_name,
                                                                    std::map<std::string, std::shared_ptr<SensorSetting>>& sensor_defaults,
                                                                    const CameraSetting& camera_defaults)
        {
            auto vehicle_type = Utils::toLower(settings_json.getString("VehicleType", ""));

            std::unique_ptr<VehicleSetting> vehicle_setting;
            if (vehicle_type == kVehicleTypePX4 || vehicle_type == kVehicleTypeArduCopterSolo || vehicle_type == kVehicleTypeArduCopter || vehicle_type == kVehicleTypeArduRover)
                vehicle_setting = createMavLinkVehicleSetting(settings_json);
            //for everything else we don't need derived class yet
            else {
                vehicle_setting = std::unique_ptr<VehicleSetting>(new VehicleSetting());
                if (vehicle_type == kVehicleTypeSimpleFlight) {
                    //TODO: we should be selecting remote if available else keyboard
                    //currently keyboard is not supported so use rc as default
                    vehicle_setting->rc.remote_control_id = 0;
                }
            }
            vehicle_setting->vehicle_name = vehicle_name;

            //required settings_json
            vehicle_setting->vehicle_type = vehicle_type;

            //optional settings_json
            vehicle_setting->pawn_path = settings_json.getString("PawnPath", "");
            vehicle_setting->default_vehicle_state = settings_json.getString("DefaultVehicleState", "");
            vehicle_setting->allow_api_always = settings_json.getBool("AllowAPIAlways",
                                                                      vehicle_setting->allow_api_always);
            vehicle_setting->auto_create = settings_json.getBool("AutoCreate",
                                                                 vehicle_setting->auto_create);
            vehicle_setting->enable_collision_passthrough = settings_json.getBool("EnableCollisionPassthrough",
                                                                                  vehicle_setting->enable_collision_passthrough);
            vehicle_setting->enable_trace = settings_json.getBool("EnableTrace",
                                                                  vehicle_setting->enable_trace);
            vehicle_setting->enable_collisions = settings_json.getBool("EnableCollisions",
                                                                       vehicle_setting->enable_collisions);
            vehicle_setting->is_fpv_vehicle = settings_json.getBool("IsFpvVehicle",
                                                                    vehicle_setting->is_fpv_vehicle);

            loadRCSetting(simmode_name, settings_json, vehicle_setting->rc);

            vehicle_setting->position = createVectorSetting(settings_json, vehicle_setting->position);
            vehicle_setting->rotation = createRotationSetting(settings_json, vehicle_setting->rotation);

            loadCameraSettings(settings_json, vehicle_setting->cameras, camera_defaults);
            loadSensorSettings(settings_json, "Sensors", vehicle_setting->sensors, sensor_defaults, simmode_name);

            return vehicle_setting;
        }

         static std::unique_ptr<BeaconSetting> createBeaconSetting(const std::string& simmode_name, const Settings& settings_json,
        const std::string beacon_name)
    {
        auto beacon_type = Utils::toLower(settings_json.getString("BeaconType", ""));
        auto beacon_pawn_name = settings_json.getString("BeaconPawnName", "");

        std::unique_ptr<BeaconSetting> beacon_setting;
        beacon_setting = std::unique_ptr<BeaconSetting>(new BeaconSetting());
        beacon_setting->beacon_name = beacon_name;

        //required settings_json
        beacon_setting->beacon_type = beacon_type;

        beacon_setting->beacon_pawn_name = beacon_pawn_name;

        //optional settings_json
        beacon_setting->scale = settings_json.getFloat("Scale", beacon_setting->scale);
        beacon_setting->pawn_path = settings_json.getString("PawnPath", "");
        beacon_setting->default_beacon_state = settings_json.getString("DefaultbeaconState", "");
        beacon_setting->allow_api_always = settings_json.getBool("AllowAPIAlways",
            beacon_setting->allow_api_always);
        beacon_setting->auto_create = settings_json.getBool("AutoCreate",
            beacon_setting->auto_create);
        beacon_setting->enable_collision_passthrough = settings_json.getBool("EnableCollisionPassthrough",
            beacon_setting->enable_collision_passthrough);
        beacon_setting->enable_trace = settings_json.getBool("EnableTrace",
            beacon_setting->enable_trace);
        beacon_setting->enable_collisions = settings_json.getBool("EnableCollisions",
            beacon_setting->enable_collisions);
        //beacon_setting->is_fpv_beacon = settings_json.getBool("IsFpvbeacon",
            //beacon_setting->is_fpv_beacon);

        Settings rc_json;
        if (settings_json.getChild("RC", rc_json)) {
            loadRCSetting(simmode_name, rc_json, beacon_setting->rc);
        }

        beacon_setting->position = createVectorSetting(settings_json, beacon_setting->position);
        beacon_setting->rotation = createRotationSetting(settings_json, beacon_setting->rotation);

        //loadCameraSettings(settings_json, beacon_setting->cameras);
        //loadSensorSettings(settings_json, "Sensors", beacon_setting->sensors  simmode_name);

        return beacon_setting;
    }

    static std::unique_ptr<PassiveEchoBeaconSetting> createPassiveEchoBeaconSetting(const Settings& settings_json, const std::string passive_echo_beacon_name)
    {
        std::unique_ptr<PassiveEchoBeaconSetting> passive_echo_beacon_setting;
        passive_echo_beacon_setting = std::unique_ptr<PassiveEchoBeaconSetting>(new PassiveEchoBeaconSetting());
        passive_echo_beacon_setting->name = passive_echo_beacon_name;


        passive_echo_beacon_setting->enable = settings_json.getBool("Enable", passive_echo_beacon_setting->enable);
        passive_echo_beacon_setting->initial_directions = settings_json.getInt("InitialDirections", passive_echo_beacon_setting->initial_directions);
        passive_echo_beacon_setting->initial_lower_azimuth_limit = settings_json.getFloat("SensorLowerAzimuthLimit", passive_echo_beacon_setting->initial_lower_azimuth_limit);
        passive_echo_beacon_setting->initial_upper_azimuth_limit = settings_json.getFloat("SensorUpperAzimuthLimit", passive_echo_beacon_setting->initial_upper_azimuth_limit);
        passive_echo_beacon_setting->initial_lower_elevation_limit = settings_json.getFloat("SensorLowerElevationLimit", passive_echo_beacon_setting->initial_lower_elevation_limit);
        passive_echo_beacon_setting->initial_upper_elevation_limit = settings_json.getFloat("SensorUpperElevationLimit", passive_echo_beacon_setting->initial_upper_elevation_limit);
        passive_echo_beacon_setting->attenuation_limit = settings_json.getFloat("AttenuationLimit", passive_echo_beacon_setting->attenuation_limit);
        passive_echo_beacon_setting->reflection_distance_limit = settings_json.getFloat("ReflectionDistanceLimit", passive_echo_beacon_setting->reflection_distance_limit);
        passive_echo_beacon_setting->reflection_only_final = settings_json.getBool("ReflectionOnlyFinal", passive_echo_beacon_setting->reflection_only_final);
        passive_echo_beacon_setting->attenuation_per_distance = settings_json.getFloat("AttenuationPerDistance", passive_echo_beacon_setting->attenuation_per_distance);
        passive_echo_beacon_setting->attenuation_per_reflection = settings_json.getFloat("AttenuationPerReflection", passive_echo_beacon_setting->attenuation_per_reflection);
        passive_echo_beacon_setting->distance_limit = settings_json.getFloat("DistanceLimit", passive_echo_beacon_setting->distance_limit);
        passive_echo_beacon_setting->reflection_limit = settings_json.getInt("ReflectionLimit", passive_echo_beacon_setting->reflection_limit);
        passive_echo_beacon_setting->draw_debug_location = settings_json.getBool("DrawDebugLocation", passive_echo_beacon_setting->draw_debug_location);
        passive_echo_beacon_setting->draw_debug_all_points = settings_json.getBool("DrawDebugAllPoints", passive_echo_beacon_setting->draw_debug_all_points);
        passive_echo_beacon_setting->draw_debug_all_lines = settings_json.getBool("DrawDebugAllLines", passive_echo_beacon_setting->draw_debug_all_lines);
        passive_echo_beacon_setting->draw_debug_duration = settings_json.getFloat("DrawDebugDuration", passive_echo_beacon_setting->draw_debug_duration);

        passive_echo_beacon_setting->position = createVectorSetting(settings_json, passive_echo_beacon_setting->position);
        passive_echo_beacon_setting->rotation = createRotationSetting(settings_json, passive_echo_beacon_setting->rotation);

        return passive_echo_beacon_setting;
    }

        static void createDefaultVehicle(const std::string& simmode_name, std::map<std::string, std::unique_ptr<VehicleSetting>>& vehicles,
                                         const std::map<std::string, std::shared_ptr<SensorSetting>>& sensor_defaults)
        {
            vehicles.clear();

            //NOTE: Do not set defaults for vehicle type here. If you do then make sure
            //to sync code in createVehicleSetting() as well.
            if (simmode_name == kSimModeTypeMultirotor) {
                // create simple flight as default multirotor
                auto simple_flight_setting = std::unique_ptr<VehicleSetting>(new VehicleSetting("SimpleFlight",
                                                                                                kVehicleTypeSimpleFlight));
                // TODO: we should be selecting remote if available else keyboard
                // currently keyboard is not supported so use rc as default
                simple_flight_setting->rc.remote_control_id = 0;
                simple_flight_setting->sensors = sensor_defaults;
                vehicles[simple_flight_setting->vehicle_name] = std::move(simple_flight_setting);
            }
            else if (simmode_name == kSimModeTypeCar) {
                // create PhysX as default car vehicle
                auto physx_car_setting = std::unique_ptr<VehicleSetting>(new VehicleSetting("PhysXCar", kVehicleTypePhysXCar));
                physx_car_setting->sensors = sensor_defaults;
                vehicles[physx_car_setting->vehicle_name] = std::move(physx_car_setting);
            }
            else if (simmode_name == kSimModeTypeSkidVehicle) {
                // create CPHusky as default skidvehicle vehicle
                auto cphusky_car_setting = std::unique_ptr<VehicleSetting>(new VehicleSetting("CPHusky", kVehicleTypeCPHusky));
                cphusky_car_setting->sensors = sensor_defaults;
                vehicles[cphusky_car_setting->vehicle_name] = std::move(cphusky_car_setting);
            }
            else if (simmode_name == kSimModeTypeComputerVision) {
                // create default computer vision vehicle
                auto cv_setting = std::unique_ptr<VehicleSetting>(new VehicleSetting("ComputerVision", kVehicleTypeComputerVision));
                cv_setting->sensors = sensor_defaults;
                vehicles[cv_setting->vehicle_name] = std::move(cv_setting);
            }
            else {
                throw std::invalid_argument(Utils::stringf(
                                                "Unknown SimMode: %s, failed to set default vehicle settings", simmode_name.c_str())
                                                .c_str());
            }
        }

        static void loadVehicleSettings(const std::string& simmode_name, const Settings& settings_json,
                                        std::map<std::string, std::unique_ptr<VehicleSetting>>& vehicles,
                                        std::map<std::string, std::shared_ptr<SensorSetting>>& sensor_defaults,
                                        const CameraSetting& camera_defaults)
        {
            createDefaultVehicle(simmode_name, vehicles, sensor_defaults);

            msr::airlib::Settings vehicles_child;
            if (settings_json.getChild("Vehicles", vehicles_child)) {
                std::vector<std::string> keys;
                vehicles_child.getChildNames(keys);

                //remove default vehicles, if values are specified in settings
                if (keys.size())
                    vehicles.clear();

                for (const auto& key : keys) {
                    msr::airlib::Settings child;
                    vehicles_child.getChild(key, child);
                    vehicles[key] = createVehicleSetting(simmode_name, child, key, sensor_defaults, camera_defaults);
                }
            }
        }

        static void loadBeaconSettings(const std::string& simmode_name, const Settings& settings_json,
                                       std::map<std::string, std::unique_ptr<BeaconSetting>>& beacons)
        {
            beacons.clear();

            msr::airlib::Settings beacons_child;
            if (settings_json.getChild("Beacons", beacons_child)) {
                std::vector<std::string> keys;
                beacons_child.getChildNames(keys);

                //remove default beacons, if values are specified in settings
                if (keys.size())
                    beacons.clear();

                for (const auto& key : keys) {
                    msr::airlib::Settings child;
                    beacons_child.getChild(key, child);
                    beacons[key] = createBeaconSetting(simmode_name, child, key);
                }
            }
        }

        static void loadPassiveEchoBeaconSettings(const Settings& settings_json,
            std::map<std::string, std::unique_ptr<PassiveEchoBeaconSetting>>& passive_echo_beacons)
        {
            passive_echo_beacons.clear();

            msr::airlib::Settings passive_echo_beacons_child;
            if (settings_json.getChild("PassiveEchoBeacons", passive_echo_beacons_child)) {
                std::vector<std::string> keys;
                passive_echo_beacons_child.getChildNames(keys);

                //remove default beacons, if values are specified in settings
                if (keys.size())
                    passive_echo_beacons.clear();

                for (const auto& key : keys) {
                    msr::airlib::Settings child;
                    passive_echo_beacons_child.getChild(key, child);
                    passive_echo_beacons[key] = createPassiveEchoBeaconSetting(child, key);
                }
            }
        }


        static void initializePawnPaths(std::map<std::string, PawnPath>& pawn_paths)
        {
            pawn_paths.clear();
            pawn_paths.emplace("BareboneCar",
                               PawnPath("Class'/AirSim/VehicleAdv/Vehicle/VehicleAdvPawn.VehicleAdvPawn_C'"));
            pawn_paths.emplace("DefaultCar",
                               PawnPath("Class'/AirSim/VehicleAdv/SUV/SuvCarPawn.SuvCarPawn_C'"));
		    pawn_paths.emplace("DefaultSkidVehicle",
                                PawnPath("Class'/AirSim/VehicleAdv/CPHusky/CPHuskyPawn.CPHuskyPawn_C'"));
            pawn_paths.emplace("DefaultQuadrotor",
                               PawnPath("Class'/AirSim/Blueprints/BP_FlyingPawn.BP_FlyingPawn_C'"));
            pawn_paths.emplace("Pioneer",
                PawnPath("Class'/AirSim/VehicleAdv/Pioneer/PioneerPawn.PioneerPawn_C'"));
            pawn_paths.emplace("BoxCar",
                PawnPath("Class'/AirSim/VehicleAdv/BoxCar/BoxCarPawn.BoxCarPawn_C'"));
            pawn_paths.emplace("DefaultComputerVision",
                               PawnPath("Class'/AirSim/Blueprints/BP_ComputerVisionPawn.BP_ComputerVisionPawn_C'"));
        }

        static void loadPawnPaths(const Settings& settings_json, std::map<std::string, PawnPath>& pawn_paths)
        {
            initializePawnPaths(pawn_paths);

            msr::airlib::Settings pawn_paths_child;
            if (settings_json.getChild("PawnPaths", pawn_paths_child)) {
                std::vector<std::string> keys;
                pawn_paths_child.getChildNames(keys);

                for (const auto& key : keys) {
                    msr::airlib::Settings child;
                    pawn_paths_child.getChild(key, child);
                    pawn_paths[key] = createPathPawn(child);
                }
            }
        }

        static PawnPath createPathPawn(const Settings& settings_json)
        {
            auto paths = PawnPath();
            paths.pawn_bp = settings_json.getString("PawnBP", "");
            auto slippery_mat = settings_json.getString("SlipperyMat", "");
            auto non_slippery_mat = settings_json.getString("NonSlipperyMat", "");

            if (slippery_mat != "")
                paths.slippery_mat = slippery_mat;
            if (non_slippery_mat != "")
                paths.non_slippery_mat = non_slippery_mat;

            return paths;
        }

        static void initializeNoiseSettings(NoiseSettingsMap& noise_settings)
        {
            const int image_count = Utils::toNumeric(ImageType::Count);
            noise_settings.clear();
            for (int i = -1; i < image_count - 1; ++i)
                noise_settings[i] = NoiseSetting();
        }

        static void loadNoiseSettings(const Settings& settings_json, NoiseSettingsMap& noise_settings)
        {
            // We don't call initializeNoiseSettings here since it's already called in CameraSettings constructor
            // And to avoid overwriting any defaults already set from CameraDefaults

            Settings json_parent;
            if (settings_json.getChild("NoiseSettings", json_parent)) {
                for (size_t child_index = 0; child_index < json_parent.size(); ++child_index) {
                    Settings json_settings_child;
                    if (json_parent.getChild(child_index, json_settings_child)) {
                        NoiseSetting noise_setting;
                        loadNoiseSetting(json_settings_child, noise_setting);
                        noise_settings[noise_setting.ImageType] = noise_setting;
                    }
                }
            }
        }

        static void loadNoiseSetting(const Settings& settings_json, NoiseSetting& noise_setting)
        {
            noise_setting.Enabled = settings_json.getBool("Enabled", noise_setting.Enabled);
            noise_setting.ImageType = settings_json.getInt("ImageType", noise_setting.ImageType);

            noise_setting.HorzWaveStrength = settings_json.getFloat("HorzWaveStrength", noise_setting.HorzWaveStrength);
            noise_setting.RandSpeed = settings_json.getFloat("RandSpeed", noise_setting.RandSpeed);
            noise_setting.RandSize = settings_json.getFloat("RandSize", noise_setting.RandSize);
            noise_setting.RandDensity = settings_json.getFloat("RandDensity", noise_setting.RandDensity);
            noise_setting.RandContrib = settings_json.getFloat("RandContrib", noise_setting.RandContrib);
            noise_setting.HorzWaveContrib = settings_json.getFloat("HorzWaveContrib", noise_setting.HorzWaveContrib);
            noise_setting.HorzWaveVertSize = settings_json.getFloat("HorzWaveVertSize", noise_setting.HorzWaveVertSize);
            noise_setting.HorzWaveScreenSize = settings_json.getFloat("HorzWaveScreenSize", noise_setting.HorzWaveScreenSize);
            noise_setting.HorzNoiseLinesContrib = settings_json.getFloat("HorzNoiseLinesContrib", noise_setting.HorzNoiseLinesContrib);
            noise_setting.HorzNoiseLinesDensityY = settings_json.getFloat("HorzNoiseLinesDensityY", noise_setting.HorzNoiseLinesDensityY);
            noise_setting.HorzNoiseLinesDensityXY = settings_json.getFloat("HorzNoiseLinesDensityXY", noise_setting.HorzNoiseLinesDensityXY);
            noise_setting.HorzDistortionStrength = settings_json.getFloat("HorzDistortionStrength", noise_setting.HorzDistortionStrength);
            noise_setting.HorzDistortionContrib = settings_json.getFloat("HorzDistortionContrib", noise_setting.HorzDistortionContrib);
        	noise_setting.LensDistortionEnable = settings_json.getBool("LensDistortionEnable", noise_setting.LensDistortionEnable);
            noise_setting.LensDistortionAreaFalloff = settings_json.getFloat("LensDistortionAreaFalloff", noise_setting.LensDistortionAreaFalloff);
            noise_setting.LensDistortionAreaRadius = settings_json.getFloat("LensDistortionAreaRadius", noise_setting.LensDistortionAreaRadius);
            noise_setting.LensDistortionIntensity = settings_json.getFloat("LensDistortionIntensity", noise_setting.LensDistortionIntensity);
            noise_setting.LensDistortionInvert = settings_json.getBool("LensDistortionInvert", noise_setting.LensDistortionInvert);
        }

        static GimbalSetting createGimbalSetting(const Settings& settings_json)
        {
            GimbalSetting gimbal;
            //capture_setting.gimbal.is_world_frame = settings_json.getBool("IsWorldFrame", false);
            gimbal.stabilization = settings_json.getFloat("Stabilization", false);
            gimbal.rotation = createRotationSetting(settings_json, gimbal.rotation);
            return gimbal;
        }

        static void loadUnrealEngineSetting(const msr::airlib::Settings& settings_json, UnrealEngineSetting& ue_setting)
        {
            Settings ue_settings_json;
            if (settings_json.getChild("UnrealEngine", ue_settings_json)) {
                Settings pixel_format_override_settings_json;
                ue_setting.pixel_format_override_settings.clear();

                for (int i = 0; i < Utils::toNumeric(ImageType::Count); i++) {
                    PixelFormatOverrideSetting pixel_format_setting;
                    pixel_format_setting.pixel_format = 0; // EXPixelformat::PF_Unknown
                    ue_setting.pixel_format_override_settings[i] = pixel_format_setting;
                }

                if (ue_settings_json.getChild("PixelFormatOverride", pixel_format_override_settings_json)) {
                    for (size_t child_index = 0; child_index < pixel_format_override_settings_json.size(); ++child_index) {
                        Settings pixel_format_child_json;
                        if (pixel_format_override_settings_json.getChild(child_index, pixel_format_child_json)) {
                            int image_type = pixel_format_child_json.getInt("ImageType", 0);
                            PixelFormatOverrideSetting& pixel_format_setting = ue_setting.pixel_format_override_settings.at(image_type);
                            pixel_format_setting.pixel_format = pixel_format_child_json.getInt("PixelFormat", 0); // default to EXPixelformat::PF_Unknown
                        }
                    }
                }
            }
        }

        static CameraSetting createCameraSetting(const Settings& settings_json, const CameraSetting& camera_defaults)
        {
            CameraSetting setting = camera_defaults;

            setting.position = createVectorSetting(settings_json, setting.position);
            setting.rotation = createRotationSetting(settings_json, setting.rotation);

            setting.external = settings_json.getBool("External", setting.external);
            setting.external_ned = settings_json.getBool("ExternalLocal", setting.external_ned);
            setting.draw_sensor = settings_json.getBool("DrawSensor", setting.draw_sensor);

            loadCaptureSettings(settings_json, setting.capture_settings);
            loadNoiseSettings(settings_json, setting.noise_settings);
            Settings json_gimbal;
            if (settings_json.getChild("Gimbal", json_gimbal))
                setting.gimbal = createGimbalSetting(json_gimbal);

            loadUnrealEngineSetting(settings_json, setting.ue_setting);

            return setting;
        }

        static void loadCameraSettings(const Settings& settings_json, CameraSettingMap& cameras,
                                       const CameraSetting& camera_defaults)
        {
            cameras.clear();

            Settings json_parent;
            if (settings_json.getChild("Cameras", json_parent)) {
                std::vector<std::string> keys;
                json_parent.getChildNames(keys);

                for (const auto& key : keys) {
                    msr::airlib::Settings child;
                    json_parent.getChild(key, child);
                    cameras[key] = createCameraSetting(child, camera_defaults);
                }
            }
        }

        static void createCaptureSettings(const msr::airlib::Settings& settings_json, CaptureSetting& capture_setting)
        {
            capture_setting.width = settings_json.getInt("Width", capture_setting.width);
            capture_setting.height = settings_json.getInt("Height", capture_setting.height);
            capture_setting.fov_degrees = settings_json.getFloat("FOV_Degrees", capture_setting.fov_degrees);
            capture_setting.auto_exposure_speed = settings_json.getFloat("AutoExposureSpeed", capture_setting.auto_exposure_speed);
            capture_setting.auto_exposure_bias = settings_json.getFloat("AutoExposureBias", capture_setting.auto_exposure_bias);
            capture_setting.auto_exposure_max_brightness = settings_json.getFloat("AutoExposureMaxBrightness", capture_setting.auto_exposure_max_brightness);
            capture_setting.auto_exposure_min_brightness = settings_json.getFloat("AutoExposureMinBrightness", capture_setting.auto_exposure_min_brightness);
            capture_setting.motion_blur_amount = settings_json.getFloat("MotionBlurAmount", capture_setting.motion_blur_amount);
            capture_setting.motion_blur_max = settings_json.getFloat("MotionBlurMax", capture_setting.motion_blur_max);
            capture_setting.image_type = settings_json.getInt("ImageType", 0);
            capture_setting.target_gamma = settings_json.getFloat("TargetGamma",
                                                                  capture_setting.image_type == 0 ? CaptureSetting::kSceneTargetGamma : Utils::nan<float>());
            capture_setting.chromatic_aberration_scale = settings_json.getFloat("ChromaticAberrationScale", capture_setting.chromatic_aberration_scale);
		    capture_setting.ignore_marked = settings_json.getBool("IgnoreMarked", capture_setting.ignore_marked);
            std::string projection_mode = Utils::toLower(settings_json.getString("ProjectionMode", ""));
            if (projection_mode == "" || projection_mode == "perspective")
                capture_setting.projection_mode = 0; // Perspective
            else if (projection_mode == "orthographic")
                capture_setting.projection_mode = 1; // Orthographic
            else
                throw std::invalid_argument(std::string("CaptureSettings projection_mode has invalid value in settings_json ") + projection_mode);

            capture_setting.ortho_width = settings_json.getFloat("OrthoWidth", capture_setting.ortho_width);

            capture_setting.lumen_gi_enabled = settings_json.getBool("LumenGIEnable", capture_setting.lumen_gi_enabled);
            capture_setting.lumen_reflections_enabled = settings_json.getBool("LumenReflectionEnable", capture_setting.lumen_reflections_enabled);
            capture_setting.lumen_final_quality = settings_json.getFloat("LumenFinalQuality", capture_setting.lumen_final_quality);
            capture_setting.lumen_scene_detail = settings_json.getFloat("LumenSceneDetail", capture_setting.lumen_scene_detail);
            capture_setting.lumen_scene_lightning_quality = settings_json.getFloat("LumenSceneLightningDetail", capture_setting.lumen_scene_lightning_quality);
        }

        static void loadSubWindowsSettings(const Settings& settings_json, std::vector<SubwindowSetting>& subwindow_settings)
        {
            //load default subwindows
            initializeSubwindowSettings(subwindow_settings);

            Settings json_parent;
            if (settings_json.getChild("SubWindows", json_parent)) {
                for (size_t child_index = 0; child_index < json_parent.size(); ++child_index) {
                    Settings json_settings_child;
                    if (json_parent.getChild(child_index, json_settings_child)) {
                        int window_index = json_settings_child.getInt("WindowID", 0);
                        SubwindowSetting& subwindow_setting = subwindow_settings.at(window_index);
                        subwindow_setting.window_index = window_index;
                        subwindow_setting.image_type = Utils::toEnum<ImageType>(
                            json_settings_child.getInt("ImageType", 0));
                        subwindow_setting.annotation_name = json_settings_child.getString("Annotation", "");
                        subwindow_setting.visible = json_settings_child.getBool("Visible", false);
                        subwindow_setting.camera_name = getCameraName(json_settings_child);
                        subwindow_setting.vehicle_name = json_settings_child.getString("VehicleName", "");
                    }
                }
            }
        }

        static void loadAnnotatorSettings(const Settings& settings_json, std::vector<AnnotatorSetting>& annotator_settings)
        {

            //load default annotator layers
            initializeAnnotatorSettings(annotator_settings);

            Settings json_parent;
            if (settings_json.getChild("Annotation", json_parent)) {
                for (size_t child_index = 0; child_index < json_parent.size(); ++child_index) {
                    Settings json_settings_child;
                    if (json_parent.getChild(child_index, json_settings_child)) {
                        AnnotatorSetting annotator_setting;
                        annotator_setting.annotator_index = (int)child_index;
                        annotator_setting.type = json_settings_child.getInt("Type", 0);
                        annotator_setting.show_by_default = json_settings_child.getBool("Default", true);
                        annotator_setting.name = json_settings_child.getString("Name", "");
                        annotator_setting.set_direct = json_settings_child.getBool("SetDirect", false);
                        annotator_setting.texture_path = json_settings_child.getString("TexturePath", "");
                        annotator_setting.texture_prefix = json_settings_child.getString("TexturePrefix", "");
                        annotator_setting.max_view_distance = json_settings_child.getFloat("ViewDistance", -1.0f);
                        annotator_settings.push_back(annotator_setting);
                    }
                }
            }
        }

        static void initializeAnnotatorSettings(std::vector<AnnotatorSetting>& annotator_settings)
        {
            annotator_settings.clear();
        }


        static void initializeSubwindowSettings(std::vector<SubwindowSetting>& subwindow_settings)
        {
            subwindow_settings.clear();
            subwindow_settings.push_back(SubwindowSetting(0, ImageType::DepthVis, false, "", "", "")); //depth
            subwindow_settings.push_back(SubwindowSetting(1, ImageType::Segmentation, false, "", "", "")); //seg
            subwindow_settings.push_back(SubwindowSetting(2, ImageType::Scene, false, "", "", "")); //vis
        }

        void loadOtherSettings(const Settings& settings_json)
        {
            //by default we spawn server at local endpoint. Do not use 127.0.0.1 as default below
            //because for docker container default is 0.0.0.0 and people get really confused why things
            //don't work
            api_server_address = settings_json.getString("LocalHostIp", "");
            api_port = settings_json.getInt("ApiServerPort", RpcLibPort);
            is_record_ui_visible = settings_json.getBool("RecordUIVisible", true);
            engine_sound = settings_json.getBool("EngineSound", false);
            enable_rpc = settings_json.getBool("EnableRpc", enable_rpc);
            speed_unit_factor = settings_json.getFloat("SpeedUnitFactor", 1.0f);
            speed_unit_label = settings_json.getString("SpeedUnitLabel", "m\\s");
            move_world_origin = settings_json.getBool("MoveWorldOrigin", false);
            initial_instance_segmentation = settings_json.getBool("InitialInstanceSegmentation", true);
            log_messages_visible = settings_json.getBool("LogMessagesVisible", true);
            show_los_debug_lines_ = settings_json.getBool("ShowLosDebugLines", false);

            { //load origin geopoint
                Settings origin_geopoint_json;
                if (settings_json.getChild("OriginGeopoint", origin_geopoint_json)) {
                    GeoPoint origin = origin_geopoint.home_geo_point;
                    origin.latitude = origin_geopoint_json.getDouble("Latitude", origin.latitude);
                    origin.longitude = origin_geopoint_json.getDouble("Longitude", origin.longitude);
                    origin.altitude = origin_geopoint_json.getFloat("Altitude", origin.altitude);
                    origin_geopoint.initialize(origin);
                }
            }

            { //time of day settings_json
                Settings tod_settings_json;
                if (settings_json.getChild("TimeOfDay", tod_settings_json)) {
                    tod_setting.enabled = tod_settings_json.getBool("Enabled", tod_setting.enabled);
                    tod_setting.start_datetime = tod_settings_json.getString("StartDateTime", tod_setting.start_datetime);
                    tod_setting.celestial_clock_speed = tod_settings_json.getFloat("CelestialClockSpeed", tod_setting.celestial_clock_speed);
                    tod_setting.is_start_datetime_dst = tod_settings_json.getBool("StartDateTimeDst", tod_setting.is_start_datetime_dst);
                    tod_setting.update_interval_secs = tod_settings_json.getFloat("UpdateIntervalSecs", tod_setting.update_interval_secs);
                    tod_setting.move_sun = tod_settings_json.getBool("MoveSun", tod_setting.move_sun);
                }
            }

            {
                // Wind Settings
                Settings child_json;
                if (settings_json.getChild("Wind", child_json)) {
                    wind = createVectorSetting(child_json, wind);
                }
            }
            {
                // External Force Settings
                Settings child_json;
                if (settings_json.getChild("ExternalForce", child_json)) {
                    ext_force = createVectorSetting(child_json, ext_force);
                }
            }
        }

        static void loadDefaultCameraSetting(const Settings& settings_json, CameraSetting& camera_defaults)
        {
            Settings child_json;
            if (settings_json.getChild("CameraDefaults", child_json)) {
                camera_defaults = createCameraSetting(child_json, camera_defaults);
            }
        }
        static void loadCameraDirectorSetting(const Settings& settings_json,
                                              CameraDirectorSetting& camera_director, const std::string& simmode_name)
        {
            camera_director = CameraDirectorSetting();

            Settings child_json;
            if (settings_json.getChild("CameraDirector", child_json)) {
                camera_director.position = createVectorSetting(child_json, camera_director.position);
                camera_director.rotation = createRotationSetting(child_json, camera_director.rotation);
                camera_director.follow_distance = child_json.getFloat("FollowDistance", camera_director.follow_distance);
            }

            if (std::isnan(camera_director.follow_distance)) {
                if (simmode_name == kSimModeTypeCar)
                    camera_director.follow_distance = -8;
                else if(simmode_name == kSimModeTypeSkidVehicle)
				    camera_director.follow_distance = -2;
                else
                    camera_director.follow_distance = -3;
            }
            if (std::isnan(camera_director.position.x()))
                camera_director.position.x() = camera_director.follow_distance;
            if (std::isnan(camera_director.position.y()))
                camera_director.position.y() = 0;
            if (std::isnan(camera_director.position.z())) {
                if (simmode_name == kSimModeTypeCar)
                    camera_director.position.z() = -3;
                else
                    camera_director.position.z() = -2;
            }
        }

        void loadClockSettings(const Settings& settings_json)
        {
            clock_type = settings_json.getString("ClockType", "");

            if (clock_type == "") {
                //default value
                clock_type = "ScalableClock";

                //override if multirotor simmode with simple_flight
                if (simmode_name == kSimModeTypeMultirotor) {
                    //TODO: this won't work if simple_flight and PX4 is combined together!

                    //for multirotors we select steppable fixed interval clock unless we have
                    //PX4 enabled vehicle
                    clock_type = "SteppableClock";
                    for (auto const& vehicle : vehicles) {
                        if (vehicle.second->auto_create &&
                            vehicle.second->vehicle_type == kVehicleTypePX4) {
                            clock_type = "ScalableClock";
                            break;
                        }
                    }
                }
            }

            clock_speed = settings_json.getFloat("ClockSpeed", 1.0f);
        }

        static std::shared_ptr<SensorSetting> createSensorSetting(
            SensorBase::SensorType sensor_type, const std::string& sensor_name,
            bool enabled)
        {
            std::shared_ptr<SensorSetting> sensor_setting;

            switch (sensor_type) {
            case SensorBase::SensorType::Barometer:
                sensor_setting = std::shared_ptr<SensorSetting>(new BarometerSetting());
                break;
            case SensorBase::SensorType::Imu:
                sensor_setting = std::shared_ptr<SensorSetting>(new ImuSetting());
                break;
            case SensorBase::SensorType::Gps:
                sensor_setting = std::shared_ptr<SensorSetting>(new GpsSetting());
                break;
            case SensorBase::SensorType::Magnetometer:
                sensor_setting = std::shared_ptr<SensorSetting>(new MagnetometerSetting());
                break;
            case SensorBase::SensorType::Distance:
                sensor_setting = std::shared_ptr<SensorSetting>(new DistanceSetting());
                break;
            case SensorBase::SensorType::Lidar:
                sensor_setting = std::shared_ptr<SensorSetting>(new LidarSetting());
                break;
            case SensorBase::SensorType::GPULidar:
                sensor_setting = std::unique_ptr<SensorSetting>(new GPULidarSetting());
                break;
            case SensorBase::SensorType::Echo:
                sensor_setting = std::unique_ptr<SensorSetting>(new EchoSetting());
                break;
            case SensorBase::SensorType::SensorTemplate:
                sensor_setting = std::unique_ptr<SensorSetting>(new SensorTemplateSetting());
                break;
            case SensorBase::SensorType::MarlocUwb:
                sensor_setting = std::unique_ptr<SensorSetting>(new MarLocUwbSetting());
                break;
            case SensorBase::SensorType::Wifi:
                sensor_setting = std::unique_ptr<SensorSetting>(new MarLocUwbSetting());
                break;
            default:
                throw std::invalid_argument("Unexpected sensor type");
            }

            sensor_setting->sensor_type = sensor_type;
            sensor_setting->sensor_name = sensor_name;
            sensor_setting->enabled = enabled;

            return sensor_setting;
        }

        static void initializeSensorSetting(SensorSetting* sensor_setting, const Settings& settings_json)
        {
            sensor_setting->enabled = settings_json.getBool("Enabled", sensor_setting->enabled);

            // pass the json Settings property bag through to the specific sensor params object where it is
            // extracted there.  This way default values can be kept in one place.  For example, see the
            // BarometerSimpleParams::initializeFromSettings method.
            sensor_setting->settings = settings_json;
        }

        // creates and intializes sensor settings from json
        static void loadSensorSettings(const Settings& settings_json, const std::string& collectionName,
                                       std::map<std::string, std::shared_ptr<SensorSetting>>& sensors,
                                       std::map<std::string, std::shared_ptr<SensorSetting>>& sensor_defaults,
                                       const std::string& simmode_name)

        {
            // NOTE: Increase type if number of sensors goes above 8
            uint8_t present_sensors_bitmask = 0;

            msr::airlib::Settings sensors_child;
            if (settings_json.getChild(collectionName, sensors_child)) {
                std::vector<std::string> keys;
                sensors_child.getChildNames(keys);

                for (const auto& key : keys) {
                    msr::airlib::Settings child;
                    sensors_child.getChild(key, child);

                    auto sensor_type = Utils::toEnum<SensorBase::SensorType>(child.getInt("SensorType", 0));
                    auto enabled = child.getBool("Enabled", false);

                    if (simmode_name == kSimModeTypeMultirotor  && sensor_type == SensorBase::SensorType::GPULidar && enabled) {
                        throw std::invalid_argument(std::string("GPULiDAR sensor from MultiRotor vehicle as this combination is not supported. Please remove or disable."));
                    }else{
                        sensors[key] = createSensorSetting(sensor_type, key, enabled);
                        initializeSensorSetting(sensors[key].get(), child);

                        // Mark sensor types already added
                        present_sensors_bitmask |= 1U << Utils::toNumeric(sensor_type);
                    }
                }
            }

            // Only add default sensors which are not present
            for (const auto& p : sensor_defaults) {
                auto type = Utils::toNumeric(p.second->sensor_type);

                if ((present_sensors_bitmask & (1U << type)) == 0)
                    sensors[p.first] = p.second;
            }
        }

        // creates default sensor list when none specified in json
        static void createDefaultSensorSettings(const std::string& simmode_name,
                                                std::map<std::string, std::shared_ptr<SensorSetting>>& sensors)
        {
            if (simmode_name == kSimModeTypeMultirotor) {
                sensors["imu"] = createSensorSetting(SensorBase::SensorType::Imu, "imu", true);
                sensors["magnetometer"] = createSensorSetting(SensorBase::SensorType::Magnetometer, "magnetometer", true);
                sensors["gps"] = createSensorSetting(SensorBase::SensorType::Gps, "gps", true);
                sensors["barometer"] = createSensorSetting(SensorBase::SensorType::Barometer, "barometer", true);
            }
            else if (simmode_name == kSimModeTypeCar) {
                sensors["gps"] = createSensorSetting(SensorBase::SensorType::Gps, "gps", true);
            }
            else {
                // no sensors added for other modes
            }
        }

        // loads or creates default sensor list
        static void loadDefaultSensorSettings(const std::string& simmode_name,
                                              const Settings& settings_json,
                                              std::map<std::string, std::shared_ptr<SensorSetting>>& sensors)
        {
            msr::airlib::Settings sensors_child;
            if (settings_json.getChild("DefaultSensors", sensors_child))
                loadSensorSettings(settings_json, "DefaultSensors", sensors, sensors, simmode_name);
            else
                createDefaultSensorSettings(simmode_name, sensors);
        }
    };
}
} //namespace

#endif
