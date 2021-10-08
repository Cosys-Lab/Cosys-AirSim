// Copyright (c) Microsoft Corporation. All rights reserved.
// Licensed under the MIT License.

#ifndef airsim_core_AirSimSettings_hpp
#define airsim_core_AirSimSettings_hpp

#include <string>
#include <vector>
#include <exception>
#include <functional>
#include "Settings.hpp"
#include "CommonStructs.hpp"
#include "common_utils/Utils.hpp"
#include "ImageCaptureBase.hpp"
#include "sensors/SensorBase.hpp"

namespace msr { namespace airlib {

struct AirSimSettings {
private:
    typedef common_utils::Utils Utils;
    typedef ImageCaptureBase::ImageType ImageType;

public: //types
    static constexpr int kSubwindowCount = 3; //must be >= 3 for now
    static constexpr char const * kVehicleTypePX4 = "px4multirotor";
	static constexpr char const * kVehicleTypeArduCopterSolo = "arducoptersolo";
	static constexpr char const * kVehicleTypeSimpleFlight = "simpleflight";
    static constexpr char const * kVehicleTypePhysXCar = "physxcar";
	static constexpr char const * kVehicleTypeBoxCar = "boxcar";
	static constexpr char const * kVehicleTypeCPHusky = "cphusky";
	static constexpr char const * kVehicleTypePioneer = "pioneer";
    static constexpr char const * kVehicleTypeComputerVision = "computervision";
    static constexpr char const * kVehicleTypeUrdfBot = "urdfbot";
    static constexpr char const * kBeaconTypeTemplate = "templateBeacon";
    static constexpr char const * kVehicleInertialFrame = "VehicleInertialFrame";
    static constexpr char const * kSensorLocalFrame = "SensorLocalFrame";

    struct SubwindowSetting {
        int window_index;
        ImageType image_type;
        bool visible;
        std::string camera_name;

        SubwindowSetting(int window_index_val = 0, ImageType image_type_val = ImageType::Scene, bool visible_val = false, const std::string& camera_name_val = "")
            : window_index(window_index_val), image_type(image_type_val), visible(visible_val), camera_name(camera_name_val)
        {
        }
    };

    struct RecordingSetting {
        bool record_on_move;
        float record_interval;

        std::vector<msr::airlib::ImageCaptureBase::ImageRequest> requests;

        RecordingSetting(bool record_on_move_val = false, float record_interval_val = 0.05f)
            : record_on_move(record_on_move_val), record_interval(record_interval_val)
        {
        }
    };

    struct PawnPath {
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

    struct RCSettings {
        int remote_control_id = -1;
        bool allow_api_when_disconnected = false;
    };

    struct Rotation {
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

        bool hasNan()
        {
            return std::isnan(yaw) || std::isnan(pitch) || std::isnan(roll);
        }

        static Rotation nanRotation()
        {
            static const Rotation val(Utils::nan<float>(), Utils::nan<float>(), Utils::nan<float>());
            return val;
        }
    };


    struct GimbalSetting {
        float stabilization = 0;
        //bool is_world_frame = false;
        Rotation rotation = Rotation::nanRotation();
    };

    struct CaptureSetting {
        //below settings_json are obtained by using Unreal console command (press ~):
        // ShowFlag.VisualizeHDR 1.
        //to replicate camera settings_json to SceneCapture2D
        //TODO: should we use UAirBlueprintLib::GetDisplayGamma()?
        typedef msr::airlib::Utils Utils;
        static constexpr float kSceneTargetGamma = 1.4f;

        int image_type = 0;

        unsigned int width = 960, height = 540; //960 X 540
        float fov_degrees = Utils::nan<float>(); //90.0f
        int auto_exposure_method = -1;   //histogram
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
    };

    struct NoiseSetting {
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

    struct CameraSetting {
        //nan means keep the default values set in components
        Vector3r position = VectorMath::nanVector();
        Rotation rotation = Rotation::nanRotation();

        GimbalSetting gimbal;
        std::map<int, CaptureSetting> capture_settings;
        std::map<int, NoiseSetting>  noise_settings;

        CameraSetting()
        {
            initializeCaptureSettings(capture_settings);
            initializeNoiseSettings(noise_settings);
        }
    };

    struct CameraDirectorSetting {
        Vector3r position = VectorMath::nanVector();
        Rotation rotation = Rotation::nanRotation();
        float follow_distance = Utils::nan<float>();
    };

    struct SensorSetting {
        SensorBase::SensorType sensor_type;
        std::string sensor_name;
        bool enabled;
    };

    struct BarometerSetting : SensorSetting {
    };

    struct ImuSetting : SensorSetting {
    };

    struct GpsSetting : SensorSetting {
    };

    struct MagnetometerSetting : SensorSetting {
    };

    struct DistanceSetting : SensorSetting {
    };

    struct LidarSetting : SensorSetting {

        // shared defaults
        uint number_of_channels = 16;
        real_T range = 100.0f;                            // meters
        uint measurement_per_cycle = 512;
        uint horizontal_rotation_frequency = 10;          // rotations/sec
        float horizontal_FOV_start = 0;                   // degrees
        float horizontal_FOV_end = 359;                   // degrees
		float update_frequency = 10;				      // how frequently to update the data in Hz
		bool limit_points = true;						  // limit the amount of points per measurement to 100000
		bool pause_after_measurement = false;			  // Pause the simulation after each measurement. Useful for API interaction to be synced
												          // If true, the time passed in-engine will be used (when performance doesn't allow real-time operation)

		bool generate_noise = false;					  // Toggle range based noise
		real_T min_noise_standard_deviation = 0;		  // Minimum noise standard deviation
		real_T noise_distance_scale = 1;			      // Factor to scale noise based on distance

        // defaults specific to a mode
        float vertical_FOV_upper = Utils::nan<float>();   // drones -15, car +10
        float vertical_FOV_lower = Utils::nan<float>();   // drones -45, car -10
        Vector3r position = VectorMath::nanVector(); 
        Rotation rotation = Rotation::nanRotation();

        bool draw_debug_points = false;

        std::string data_frame = AirSimSettings::kVehicleInertialFrame;
    };


	struct GPULidarSetting : SensorSetting {

		// shared defaults
		uint number_of_channels = 64;
		real_T range = 100.0f;                            // meters
        float range_max_lambertian_percentage = 80;       // Lambertian reflectivity percentage to max out on. Will act linear to 0% for below.

        float rain_max_intensity = 70;                    // Rain intensity maximum to scale from in mm/hour.
        float rain_constant_a = 0.01f;                     // Two constants to calculate the extinction coefficient in rain
        float rain_constant_b = 0.6f;

		uint measurement_per_cycle = 2048;
		float horizontal_rotation_frequency = 10;         // rotations/sec
		float horizontal_FOV_start = 0;                   // degrees
		float horizontal_FOV_end = 359;                   // degrees
		float update_frequency = 10;				      // how frequently to update the data in Hz
		uint resolution = 512;
		bool ground_truth = false;                        // Generate ground truth segmentation color values
		bool generate_noise = false;					  // Toggle range based noise

		bool ignore_marked = false;
		real_T min_noise_standard_deviation = 0;		  // Minimum noise standard deviation
		real_T noise_distance_scale = 1;			      // Factor to scale noise based on distance

		// defaults specific to a mode
		float vertical_FOV_upper = Utils::nan<float>();   // drones -15, car +10
		float vertical_FOV_lower = Utils::nan<float>();   // drones -45, car -10
		Vector3r position = VectorMath::nanVector();
		Rotation rotation = Rotation::nanRotation();

		bool generate_intensity = false;                  // Toggle intensity calculation on or off
        std::string material_list_file = "";              // String holding all material data

		bool draw_debug_points = false;
		uint draw_mode = 0;								  // 0 = no coloring, 1 = instance segmentation, 2 = material, 3 = intensity
	};

	struct EchoSetting : SensorSetting {

		// Signal propagation settings
		uint number_of_traces = 1000;					// Amount of traces (rays) being cast
		float reflection_opening_angle = 10.0f;			// Beam width of the scattered traces
		float attenuation_per_distance = 0.0f;			// Attenuation of signal wrt distance traveled (dB/m)
		float attenuation_per_reflection = 0.0f;        // Attenuation of signal wrt reflections (dB)
		float attenuation_limit = -100.0f;              // Attenuation at which the signal is considered dissipated (dB)
		float distance_limit = 10.0f;					// Maximum distance the signal can travel.
		int reflection_limit = 3;						// Maximum times the signal can reflect.
		float reflection_distance_limit = 0.4f;			// Maximum distance between reflection locations.
		float sensor_opening_angle = 180.0f;			// The opening angle in which rays will be cast from the sensor

		// Sensor settings
		float measurement_frequency = 10;				// The frequency of the sensor (measurements/s)
		float sensor_diameter = 0.5;					// The diameter of the sensor plane used to capture the reflecting traces (meter)

		// Engine & timing settings
		bool pause_after_measurement = false;			// Pause the simulation after each measurement. Useful for API interaction to be synced
														// If true, the time passed in-engine will be used (when performance doesn't allow real-time operation)
		// Debug settings
		bool draw_initial_points = false;		     	// Draw the points of the initial half sphere where the traces (rays) are cast
		bool draw_bounce_lines = false;					// Draw lines of all bouncing reflections of the traces with their color depending on attenuation
		bool draw_reflected_points = true;				// Draw debug points in world where reflected points are captured by the sensor
		bool draw_reflected_lines = false;				// Draw debug lines in world from reflected points to the sensor
		bool draw_reflected_paths = false;				// Draw debug lines for the full path of reflected points to the sensor
		bool draw_sensor = false;						// Draw the physical sensor in the world on the vehicle
		bool draw_external_points = false;				// Draw points from an external source (e.g. MATLAB clustered pointcloud)

		// Misc
		std::string data_frame = AirSimSettings::kSensorLocalFrame;
		Vector3r position = VectorMath::nanVector();
		Rotation rotation = Rotation::nanRotation();
		bool ignore_marked = false;
	};

    struct SensorTemplateSetting : SensorSetting {
        // Engine & timing settings
        float measurement_frequency = 10;				// The frequency of the sensor (measurements/s)
        bool pause_after_measurement = false;			// Pause the simulation after each measurement. Useful for API interaction to be synced
                                                        // If true, the time passed in-engine will be used (when performance doesn't allow real-time operation)

        std::string attach_link = "";

        // Misc
        std::string data_frame = AirSimSettings::kSensorLocalFrame;
        Vector3r position = VectorMath::nanVector();
        Rotation rotation = Rotation::nanRotation();
    };

    struct MarLocUwbSetting : SensorSetting {
        // Engine & timing settings
        uint number_of_traces = 5;	     				// Amount of traces (rays) being cast
        float sensor_opening_angle = 180.0f;			// The opening angle in which rays will be cast from the sensor
        float measurement_frequency = 10;				// The frequency of the sensor (measurements/s)
        bool pause_after_measurement = false;			// Pause the simulation after each measurement. Useful for API interaction to be synced
                                                        // If true, the time passed in-engine will be used (when performance doesn't allow real-time operation)

        std::string attach_link = "";

        // Misc
        std::string data_frame = AirSimSettings::kSensorLocalFrame;
        Vector3r position = VectorMath::nanVector();
        Rotation rotation = Rotation::nanRotation();
    };

    struct VehicleSetting {
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

        std::map<std::string, CameraSetting> cameras;
        std::map<std::string, std::unique_ptr<SensorSetting>> sensors;

        RCSettings rc;
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
    };

    struct MavLinkConnectionInfo {
        /* Default values are requires so uninitialized instance doesn't have random values */

        bool use_serial = true; // false means use UDP instead
                                //Used to connect via HITL: needed only if use_serial = true
        std::string serial_port = "*";
        int baud_rate = 115200;

        //Used to connect to drone over UDP: needed only if use_serial = false
        std::string ip_address = "127.0.0.1";
        int ip_port = 14560;

        // The PX4 SITL app requires receiving drone commands over a different mavlink channel.
        // So set this to empty string to disable this separate command channel.
        std::string sitl_ip_address = "127.0.0.1";
        int sitl_ip_port = 14556;

        // The log viewer can be on a different machine, so you can configure it's ip address and port here.
        int logviewer_ip_port = 14388;
        int logviewer_ip_sport = 14389; // for logging all messages we send to the vehicle.
        std::string logviewer_ip_address = "127.0.0.1";

        // The QGroundControl app can be on a different machine, so you can configure it's ip address and port here.
        int qgc_ip_port = 14550;
        std::string qgc_ip_address = "127.0.0.1";

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
    };

	struct MavLinkVehicleSetting : public VehicleSetting {
		MavLinkConnectionInfo connection_info;
	};

    struct TimeOfDaySetting {
        bool enabled = false;
        std::string start_datetime = "";    //format: %Y-%m-%d %H:%M:%S
        bool is_start_datetime_dst = false;
        float celestial_clock_speed = 1;
        float update_interval_secs = 60;
        bool move_sun = true;
    };

private: //fields
    float settings_version_actual;
    float settings_version_minimum = 1.2f;

public: //fields
    std::string simmode_name = "";

    std::vector<SubwindowSetting> subwindow_settings;
    RecordingSetting recording_setting;
    TimeOfDaySetting tod_setting;

    std::vector<std::string> warning_messages;
    std::vector<std::string> error_messages;
    
    bool is_record_ui_visible = false;
    int initial_view_mode = 2; //ECameraDirectorMode::CAMERA_DIRECTOR_MODE_FLY_WITH_ME
    bool enable_rpc = true;
    std::string api_server_address = "";
    std::string physics_engine_name = "";

    std::string clock_type = "";
    float clock_speed = 1.0f;
    bool engine_sound = false;
    bool log_messages_visible = true;
    HomeGeoPoint origin_geopoint{ GeoPoint(47.641468, -122.140165, 122) }; //The geo-coordinate assigned to Unreal coordinate 0,0,0
    std::map<std::string, PawnPath> pawn_paths; //path for pawn blueprint
    std::map<std::string, std::unique_ptr<VehicleSetting>> vehicles;
    std::map<std::string, std::unique_ptr<BeaconSetting>> beacons;
    CameraSetting camera_defaults;
    CameraDirectorSetting camera_director;
	float speed_unit_factor =  1.0f;
	std::string speed_unit_label = "m\\s";
    std::map<std::string, std::unique_ptr<SensorSetting>> sensor_defaults;

    std::string material_list_file = "";

public: //methods
    static AirSimSettings& singleton() 
    {
        static AirSimSettings instance;
        return instance;
    }

    AirSimSettings()
    {
        initializeSubwindowSettings(subwindow_settings);
        initializePawnPaths(pawn_paths);
        initializeVehicleSettings(vehicles);
    }

    //returns number of warnings
    void load(std::function<std::string(void)> simmode_getter)
    {
        warning_messages.clear();
        error_messages.clear();
        const Settings& settings_json = Settings::singleton();
        checkSettingsVersion(settings_json);

        loadCoreSimModeSettings(settings_json, simmode_getter);
        loadDefaultCameraSetting(settings_json, camera_defaults);
        loadCameraDirectorSetting(settings_json, camera_director, simmode_name);
        loadSubWindowsSettings(settings_json, subwindow_settings);
        loadViewModeSettings(settings_json);
        loadRecordingSetting(settings_json, recording_setting);
        loadPawnPaths(settings_json, pawn_paths);
        loadOtherSettings(settings_json);
        loadDefaultSensorSettings(simmode_name, settings_json, sensor_defaults);
        loadVehicleSettings(simmode_name, settings_json, vehicles);
        loadBeaconSettings(simmode_name, settings_json, beacons);

        //this should be done last because it depends on type of vehicles we have
        loadClockSettings(settings_json);
    }

    static void initializeSettings(const std::string& json_settings_text)
    {
        Settings& settings_json = Settings::loadJSonString(json_settings_text);
        if (! settings_json.isLoadSuccess())
            throw std::invalid_argument("Cannot parse JSON settings_json string.");
    }

    static void createDefaultSettingsFile()
    {
        std::string settings_filename = Settings::getUserDirectoryFullPath("settings.json");
        Settings& settings_json = Settings::loadJSonString("{}");
        //write some settings_json in new file otherwise the string "null" is written if all settings_json are empty
        settings_json.setString("SeeDocsAt", "https://cosysgit.uantwerpen.be/sensorsimulation/airsim/-/blob/master/docs/settings.md");
        settings_json.setDouble("SettingsVersion", 1.2);

        //TODO: there is a crash in Linux due to settings_json.saveJSonString(). Remove this workaround after we only support Unreal 4.17
        //https://answers.unrealengine.com/questions/664905/unreal-crashes-on-two-lines-of-extremely-simple-st.html
        settings_json.saveJSonFile(settings_filename);
    }

    const VehicleSetting* getVehicleSetting(const std::string& vehicle_name) const
    {
        auto it = vehicles.find(vehicle_name);
        if (it == vehicles.end())
            throw std::invalid_argument(Utils::stringf("VehicleSetting for vehicle name %s was requested but not found",
                vehicle_name.c_str()).c_str());
        else
            return it->second.get();
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
                        "AdditionalCameras", "CaptureSettings", "NoiseSettings",
                        "UsageScenario", "SimpleFlight", "PX4"
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
                            "Following keys in your settings.json needs updating: "
                            ;

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

        bool has_docs = settings_json.getString("SeeDocsAt", "") != ""
            || settings_json.getString("see_docs_at", "") != "";
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
            if (simmode_getter)
                simmode_name = simmode_getter();
            else
                throw std::invalid_argument("simmode_name is not expected empty in SimModeBase");
        }

        physics_engine_name = settings_json.getString("PhysicsEngineName", "");
        if (physics_engine_name == "") {
            if (simmode_name == "Multirotor")
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

    void loadViewModeSettings(const Settings& settings_json)
    {
        std::string view_mode_string = settings_json.getString("ViewMode", "");

        if (view_mode_string == "") {
            if (simmode_name == "Multirotor")
                view_mode_string = "FlyWithMe";
            else if (simmode_name == "ComputerVision")
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
                simmode_name == "Multirotor" ? 0 : -1);
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

    static void loadRecordingSetting(const Settings& settings_json, RecordingSetting& recording_setting)
    {
        Settings recording_json;
        if (settings_json.getChild("Recording", recording_json)) {
            recording_setting.record_on_move = recording_json.getBool("RecordOnMove", recording_setting.record_on_move);
            recording_setting.record_interval = recording_json.getFloat("RecordInterval", recording_setting.record_interval);

            Settings req_cameras_settings;
            if (recording_json.getChild("Cameras", req_cameras_settings)) {
                for (size_t child_index = 0; child_index < req_cameras_settings.size(); ++child_index) {
                    Settings req_camera_settings;
                    if (req_cameras_settings.getChild(child_index, req_camera_settings)) {
                        std::string camera_name = getCameraName(req_camera_settings);
                        ImageType image_type =
                            Utils::toEnum<ImageType>(
                                req_camera_settings.getInt("ImageType", 0));
                        bool compress = req_camera_settings.getBool("Compress", true);
                        bool pixels_as_float = req_camera_settings.getBool("PixelsAsFloat", false);

                        recording_setting.requests.push_back(msr::airlib::ImageCaptureBase::ImageRequest(
                            camera_name, image_type, pixels_as_float, compress));
                    }
                }
            }
        }
        if (recording_setting.requests.size() == 0)
            recording_setting.requests.push_back(msr::airlib::ImageCaptureBase::ImageRequest(
                "", ImageType::Scene, false, true));
    }

    static void initializeCaptureSettings(std::map<int, CaptureSetting>& capture_settings)
    {
        capture_settings.clear();
        for (int i = -1; i < Utils::toNumeric(ImageType::Count); ++i) {
            capture_settings[i] = CaptureSetting();
        }
        capture_settings.at(Utils::toNumeric(ImageType::Scene)).target_gamma = CaptureSetting::kSceneTargetGamma;
		capture_settings.at(Utils::toNumeric(ImageType::Segmentation)).target_gamma = 1;
    }

    static void loadCaptureSettings(const Settings& settings_json, std::map<int, CaptureSetting>& capture_settings)
    {
        initializeCaptureSettings(capture_settings);

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

        MavLinkConnectionInfo &connection_info = vehicle_setting->connection_info;
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

        connection_info.sitl_ip_address = settings_json.getString("SitlIp", connection_info.sitl_ip_address);
        connection_info.sitl_ip_port = settings_json.getInt("SitlPort", connection_info.sitl_ip_port);

        connection_info.local_host_ip = settings_json.getString("LocalHostIp", connection_info.local_host_ip);


        connection_info.use_serial = settings_json.getBool("UseSerial", connection_info.use_serial);
        connection_info.ip_address = settings_json.getString("UdpIp", connection_info.ip_address);
        connection_info.ip_port = settings_json.getInt("UdpPort", connection_info.ip_port);
        connection_info.serial_port = settings_json.getString("SerialPort", connection_info.serial_port);
        connection_info.baud_rate = settings_json.getInt("SerialBaudRate", connection_info.baud_rate);
        connection_info.model = settings_json.getString("Model", connection_info.model);

        return vehicle_setting_p;
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

    static std::unique_ptr<VehicleSetting> createVehicleSetting(const std::string& simmode_name,  const Settings& settings_json,
        const std::string vehicle_name)
    {
        auto vehicle_type = Utils::toLower(settings_json.getString("VehicleType", ""));

        std::unique_ptr<VehicleSetting> vehicle_setting;
        if (vehicle_type == kVehicleTypePX4 || vehicle_type == kVehicleTypeArduCopterSolo)
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
        vehicle_setting->enable_collision_passthrough = settings_json.getBool("EnableCollisionPassthrogh", 
            vehicle_setting->enable_collision_passthrough);
        vehicle_setting->enable_trace = settings_json.getBool("EnableTrace",
            vehicle_setting->enable_trace);
        vehicle_setting->enable_collisions = settings_json.getBool("EnableCollisions",
            vehicle_setting->enable_collisions);
        vehicle_setting->is_fpv_vehicle = settings_json.getBool("IsFpvVehicle",
            vehicle_setting->is_fpv_vehicle);

        Settings rc_json;
        if (settings_json.getChild("RC", rc_json)) {
            loadRCSetting(simmode_name, rc_json, vehicle_setting->rc);
        }

        vehicle_setting->position = createVectorSetting(settings_json, vehicle_setting->position);
        vehicle_setting->rotation = createRotationSetting(settings_json, vehicle_setting->rotation);

        loadCameraSettings(settings_json, vehicle_setting->cameras);
        loadSensorSettings(settings_json, "Sensors", vehicle_setting->sensors);
       
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
        beacon_setting->pawn_path = settings_json.getString("PawnPath", "");
        beacon_setting->default_beacon_state = settings_json.getString("DefaultbeaconState", "");
        beacon_setting->allow_api_always = settings_json.getBool("AllowAPIAlways",
            beacon_setting->allow_api_always);
        beacon_setting->auto_create = settings_json.getBool("AutoCreate",
            beacon_setting->auto_create);
        beacon_setting->enable_collision_passthrough = settings_json.getBool("EnableCollisionPassthrogh",
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
        //loadSensorSettings(settings_json, "Sensors", beacon_setting->sensors);

        return beacon_setting;
    }

    static void initializeVehicleSettings(std::map<std::string, std::unique_ptr<VehicleSetting>>& vehicles)
    {
        vehicles.clear();

        //NOTE: Do not set defaults for vehicle type here. If you do then make sure
        //to sync code in createVehicleSetting() as well.

        //create simple flight as default multirotor
        auto simple_flight_setting = std::unique_ptr<VehicleSetting>(new VehicleSetting());
        simple_flight_setting->vehicle_name = "SimpleFlight";
        simple_flight_setting->vehicle_type = kVehicleTypeSimpleFlight;
        //TODO: we should be selecting remote if available else keyboard
        //currently keyboard is not supported so use rc as default
        simple_flight_setting->rc.remote_control_id = 0;
        vehicles[simple_flight_setting->vehicle_name] = std::move(simple_flight_setting);

        //create default car vehicle
        auto physx_car_setting = std::unique_ptr<VehicleSetting>(new VehicleSetting());
        physx_car_setting->vehicle_name = "PhysXCar";
        physx_car_setting->vehicle_type = kVehicleTypePhysXCar;
        vehicles[physx_car_setting->vehicle_name] = std::move(physx_car_setting);

		//create default box vehicle
		auto box_car_setting = std::unique_ptr<VehicleSetting>(new VehicleSetting());
		box_car_setting->vehicle_name = "BoxCar";
		box_car_setting->vehicle_type = kVehicleTypeBoxCar;
		vehicles[box_car_setting->vehicle_name] = std::move(box_car_setting);

		//create default robot vehicle
		auto cphusky_setting = std::unique_ptr<VehicleSetting>(new VehicleSetting());
		cphusky_setting->vehicle_name = "CPHusky";
		cphusky_setting->vehicle_type = kVehicleTypeCPHusky;
		vehicles[cphusky_setting->vehicle_name] = std::move(cphusky_setting);

		//create default robot vehicle
		auto pioneer_setting = std::unique_ptr<VehicleSetting>(new VehicleSetting());
		pioneer_setting->vehicle_name = "Pioneer";
		pioneer_setting->vehicle_type = kVehicleTypePioneer;
		vehicles[pioneer_setting->vehicle_name] = std::move(pioneer_setting);

        //create default computer vision vehicle
        auto cv_setting = std::unique_ptr<VehicleSetting>(new VehicleSetting());
        cv_setting->vehicle_name = "ComputerVision";
        cv_setting->vehicle_type = kVehicleTypeComputerVision;
        vehicles[cv_setting->vehicle_name] = std::move(cv_setting);
    }

    static void loadVehicleSettings(const std::string& simmode_name, const Settings& settings_json,
        std::map<std::string, std::unique_ptr<VehicleSetting>>& vehicles)
    {
        initializeVehicleSettings(vehicles);

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
                vehicles[key] = createVehicleSetting(simmode_name, child, key);
            }
        }
    }

    static void loadBeaconSettings(const std::string& simmode_name, const Settings& settings_json,
        std::map<std::string, std::unique_ptr<BeaconSetting>>& beacons)
    {
        //initializeVehicleSettings(vehicles);

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

    static void initializePawnPaths(std::map<std::string, PawnPath>& pawn_paths)
    {
        pawn_paths.clear();
        pawn_paths.emplace("BareboneCar",
            PawnPath("Class'/AirSim/VehicleAdv/Vehicle/VehicleAdvPawn.VehicleAdvPawn_C'"));
        pawn_paths.emplace("DefaultCar",
            PawnPath("Class'/AirSim/VehicleAdv/SUV/SuvCarPawn.SuvCarPawn_C'"));
		pawn_paths.emplace("BoxCar",
			PawnPath("Class'/AirSim/VehicleAdv/BoxCar/BoxCarPawn.BoxCarPawn_C'"));
		pawn_paths.emplace("CPHusky",
			PawnPath("Class'/AirSim/VehicleAdv/CPHusky/CPHuskyPawn.CPHuskyPawn_C'"));
		pawn_paths.emplace("Pioneer",
			PawnPath("Class'/AirSim/VehicleAdv/Pioneer/PioneerPawn.PioneerPawn_C'"));
        pawn_paths.emplace("DefaultQuadrotor",
            PawnPath("Class'/AirSim/Blueprints/BP_FlyingPawn.BP_FlyingPawn_C'"));
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

    static void initializeNoiseSettings(std::map<int, NoiseSetting>&  noise_settings)
    {
        int image_count = Utils::toNumeric(ImageType::Count);
        noise_settings.clear();
        for (int i = -1; i < image_count; ++i)
            noise_settings[i] = NoiseSetting();
    }

    static void loadNoiseSettings(const Settings& settings_json, std::map<int, NoiseSetting>&  noise_settings)
    {
        initializeNoiseSettings(noise_settings);

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

    static void loadNoiseSetting(const msr::airlib::Settings& settings_json, NoiseSetting& noise_setting)
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

    static CameraSetting createCameraSetting(const Settings& settings_json)
    {
        CameraSetting setting;

        setting.position = createVectorSetting(settings_json, setting.position);
        setting.rotation = createRotationSetting(settings_json, setting.rotation);

        loadCaptureSettings(settings_json, setting.capture_settings);
        loadNoiseSettings(settings_json, setting.noise_settings);
        Settings json_gimbal;
        if (settings_json.getChild("Gimbal", json_gimbal))
            setting.gimbal = createGimbalSetting(json_gimbal);

        return setting;
    }

    static void loadCameraSettings(const Settings& settings_json, std::map<std::string, CameraSetting>& cameras)
    {
        cameras.clear();

        Settings json_parent;
        if (settings_json.getChild("Cameras", json_parent)) {
            std::vector<std::string> keys;
            json_parent.getChildNames(keys);

            for (const auto& key : keys) {
                msr::airlib::Settings child;
                json_parent.getChild(key, child);
                cameras[key] = createCameraSetting(child);
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
		capture_setting.chromatic_aberration_scale = settings_json.getFloat("ChromaticAberrationScale", capture_setting.chromatic_aberration_scale);
		capture_setting.ignore_marked = settings_json.getBool("IgnoreMarked", capture_setting.ignore_marked);


        capture_setting.target_gamma = settings_json.getFloat("TargetGamma",
            capture_setting.image_type == 0 ? CaptureSetting::kSceneTargetGamma : Utils::nan<float>());

        std::string projection_mode = Utils::toLower(settings_json.getString("ProjectionMode", ""));
        if (projection_mode == "" || projection_mode == "perspective")
            capture_setting.projection_mode = 0; // Perspective
        else if (projection_mode == "orthographic")
            capture_setting.projection_mode = 1; // Orthographic
        else
            throw std::invalid_argument(std::string("CaptureSettings projection_mode has invalid value in settings_json ") + projection_mode);

        capture_setting.ortho_width = settings_json.getFloat("OrthoWidth", capture_setting.ortho_width);
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
                    subwindow_setting.visible = json_settings_child.getBool("Visible", false);
                    subwindow_setting.camera_name = getCameraName(json_settings_child);
                }
            }
        }
    }

    static void initializeSubwindowSettings(std::vector<SubwindowSetting>& subwindow_settings)
    {
        subwindow_settings.clear();
        subwindow_settings.push_back(SubwindowSetting(0, ImageType::DepthVis, false, "")); //depth
        subwindow_settings.push_back(SubwindowSetting(0, ImageType::Segmentation, false, "")); //seg
        subwindow_settings.push_back(SubwindowSetting(0, ImageType::Scene, false, "")); //vis
    }

    void loadOtherSettings(const Settings& settings_json)
    {
        //by default we spawn server at local endpoint. Do not use 127.0.0.1 as default below
        //because for docker container default is 0.0.0.0 and people get really confused why things
        //don't work
        api_server_address = settings_json.getString("LocalHostIp", "");
        is_record_ui_visible = settings_json.getBool("RecordUIVisible", true);
        engine_sound = settings_json.getBool("EngineSound", false);
        enable_rpc = settings_json.getBool("EnableRpc", enable_rpc);
        speed_unit_factor = settings_json.getFloat("SpeedUnitFactor", 1.0f);
        speed_unit_label = settings_json.getString("SpeedUnitLabel", "m\\s");
        log_messages_visible = settings_json.getBool("LogMessagesVisible", true);

        {   //load origin geopoint
            Settings origin_geopoint_json;
            if (settings_json.getChild("OriginGeopoint", origin_geopoint_json)) {
                GeoPoint origin = origin_geopoint.home_geo_point;
                origin.latitude = origin_geopoint_json.getDouble("Latitude", origin.latitude);
                origin.longitude = origin_geopoint_json.getDouble("Longitude", origin.longitude);
                origin.altitude = origin_geopoint_json.getFloat("Altitude", origin.altitude);
                origin_geopoint.initialize(origin);
            }
        }

        {   //time of day settings_json
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
    }

    static void loadDefaultCameraSetting(const Settings& settings_json, CameraSetting& camera_defaults)
    {
        Settings child_json;
        if (settings_json.getChild("CameraDefaults", child_json)) {
            camera_defaults = createCameraSetting(child_json);
        }
    }

    static void loadCameraDirectorSetting(const Settings& settings_json, 
        CameraDirectorSetting& camera_director, const std::string& simmode_name)
    {
        camera_director = CameraDirectorSetting();

        Settings child_json;
        if (settings_json.getChild("CameraDirector", child_json)) {
            camera_director.position = createVectorSetting(settings_json, camera_director.position);
            camera_director.rotation = createRotationSetting(settings_json, camera_director.rotation);
            camera_director.follow_distance = child_json.getFloat("FollowDistance", camera_director.follow_distance);
        }

		msr::airlib::Settings vehicles_child;
		std::string vehicle_type;
		if (settings_json.getChild("Vehicles", vehicles_child)) {
			std::vector<std::string> keys;
			vehicles_child.getChildNames(keys);
			msr::airlib::Settings child;
			vehicles_child.getChild(keys.at(0), child);
			vehicle_type = child.getString("VehicleType", "");
		}

        if (std::isnan(camera_director.follow_distance)) {
            if (simmode_name == "Car")
				if (vehicle_type == "BoxCar") camera_director.follow_distance = -2;
				else camera_director.follow_distance = -8;
			else if(simmode_name == "SkidVehicle")
				camera_director.follow_distance = -2;
            else
                camera_director.follow_distance = -3;
        }
        if (std::isnan(camera_director.position.x()))
            camera_director.position.x() = camera_director.follow_distance;
        if (std::isnan(camera_director.position.y()))
            camera_director.position.y() = 0;
        if (std::isnan(camera_director.position.z())) {
            if (simmode_name == "Car")
				if (vehicle_type == "BoxCar")  camera_director.position.z() = -2;
				else camera_director.position.z() = -4;

			else if (simmode_name == "SkidVehicle")
				camera_director.position.z() = -2;
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
            if (simmode_name == "Multirotor") {
                //TODO: this won't work if simple_flight and PX4 is combined together!

                //for multirotors we select steppable fixed interval clock unless we have
                //PX4 enabled vehicle
                clock_type = "SteppableClock";
                for (auto const& vehicle : vehicles)
                {
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

    static void initializeBarometerSetting(BarometerSetting& barometer_setting, const Settings& settings_json)
    {
        unused(barometer_setting);
        unused(settings_json);

        //TODO: set from json as needed
    }

    static void initializeImuSetting(ImuSetting& imu_setting, const Settings& settings_json)
    {
        unused(imu_setting);
        unused(settings_json);

        //TODO: set from json as needed
    }

    static void initializeGpsSetting(GpsSetting& gps_setting, const Settings& settings_json)
    {
        unused(gps_setting);
        unused(settings_json);

        //TODO: set from json as needed
    }

    static void initializeMagnetometerSetting(MagnetometerSetting& magnetometer_setting, const Settings& settings_json)
    {
        unused(magnetometer_setting);
        unused(settings_json);

        //TODO: set from json as needed
    }

    static void initializeDistanceSetting(DistanceSetting& distance_setting, const Settings& settings_json)
    {
        unused(distance_setting);
        unused(settings_json);

        //TODO: set from json as needed
    }

    static void initializeLidarSetting(LidarSetting& lidar_setting, const Settings& settings_json)
    {
		lidar_setting.number_of_channels = settings_json.getInt("NumberOfChannels", lidar_setting.number_of_channels);
        lidar_setting.range = settings_json.getFloat("Range", lidar_setting.range);
        lidar_setting.measurement_per_cycle = settings_json.getInt("MeasurementsPerCycle", lidar_setting.measurement_per_cycle);
        lidar_setting.horizontal_rotation_frequency = settings_json.getInt("RotationsPerSecond", lidar_setting.horizontal_rotation_frequency);

		lidar_setting.generate_noise = settings_json.getBool("GenerateNoise", lidar_setting.generate_noise);
		lidar_setting.min_noise_standard_deviation = settings_json.getFloat("MinNoiseStandardDeviation", lidar_setting.min_noise_standard_deviation);
		lidar_setting.noise_distance_scale = settings_json.getFloat("NoiseDistanceScale", lidar_setting.noise_distance_scale);

		lidar_setting.update_frequency = settings_json.getFloat("UpdateFrequency", lidar_setting.update_frequency);
		lidar_setting.pause_after_measurement = settings_json.getBool("PauseAfterMeasurement", lidar_setting.pause_after_measurement);
		lidar_setting.limit_points = settings_json.getBool("LimitPoints", lidar_setting.limit_points);
        lidar_setting.draw_debug_points = settings_json.getBool("DrawDebugPoints", lidar_setting.draw_debug_points);

        lidar_setting.data_frame = settings_json.getString("DataFrame", lidar_setting.data_frame);

        lidar_setting.vertical_FOV_upper = settings_json.getFloat("VerticalFOVUpper", lidar_setting.vertical_FOV_upper);
        lidar_setting.vertical_FOV_lower = settings_json.getFloat("VerticalFOVLower", lidar_setting.vertical_FOV_lower);
        lidar_setting.horizontal_FOV_start = settings_json.getFloat("HorizontalFOVStart", lidar_setting.horizontal_FOV_start);
        lidar_setting.horizontal_FOV_end = settings_json.getFloat("HorizontalFOVEnd", lidar_setting.horizontal_FOV_end);

        lidar_setting.position = createVectorSetting(settings_json, lidar_setting.position);
        lidar_setting.rotation = createRotationSetting(settings_json, lidar_setting.rotation);

    }

	static void initializeGPULidarSetting(GPULidarSetting& lidar_setting, const Settings& settings_json)
	{
		lidar_setting.number_of_channels = settings_json.getInt("NumberOfChannels", lidar_setting.number_of_channels);
		lidar_setting.range = settings_json.getFloat("Range", lidar_setting.range);
		lidar_setting.measurement_per_cycle = settings_json.getInt("MeasurementsPerCycle", lidar_setting.measurement_per_cycle);
		lidar_setting.horizontal_rotation_frequency = settings_json.getFloat("RotationsPerSecond", lidar_setting.horizontal_rotation_frequency);
		lidar_setting.resolution = settings_json.getInt("Resolution", lidar_setting.resolution);
		lidar_setting.ground_truth = settings_json.getBool("GroundTruth", lidar_setting.ground_truth);
		lidar_setting.generate_noise = settings_json.getBool("GenerateNoise", lidar_setting.generate_noise);
		lidar_setting.generate_intensity = settings_json.getBool("GenerateIntensity", lidar_setting.generate_intensity);
		lidar_setting.min_noise_standard_deviation = settings_json.getFloat("MinNoiseStandardDeviation", lidar_setting.min_noise_standard_deviation);
		lidar_setting.update_frequency = settings_json.getFloat("UpdateFrequency", lidar_setting.update_frequency);
		lidar_setting.noise_distance_scale = settings_json.getFloat("NoiseDistanceScale", lidar_setting.noise_distance_scale);
		lidar_setting.draw_debug_points = settings_json.getBool("DrawDebugPoints", lidar_setting.draw_debug_points);
		lidar_setting.draw_mode = settings_json.getInt("DrawMode", lidar_setting.draw_mode);
		lidar_setting.vertical_FOV_upper = settings_json.getFloat("VerticalFOVUpper", lidar_setting.vertical_FOV_upper);
		lidar_setting.vertical_FOV_lower = settings_json.getFloat("VerticalFOVLower", lidar_setting.vertical_FOV_lower);
		lidar_setting.horizontal_FOV_start = settings_json.getFloat("HorizontalFOVStart", lidar_setting.horizontal_FOV_start);
		lidar_setting.horizontal_FOV_end = settings_json.getFloat("HorizontalFOVEnd", lidar_setting.horizontal_FOV_end);
		lidar_setting.ignore_marked = settings_json.getBool("IgnoreMarked", lidar_setting.ignore_marked);
        lidar_setting.range_max_lambertian_percentage = settings_json.getFloat("rangeMaxLambertianPercentage", lidar_setting.range_max_lambertian_percentage);
        lidar_setting.rain_max_intensity = settings_json.getFloat("rainMaxIntensity", lidar_setting.rain_max_intensity);
        lidar_setting.rain_constant_a = settings_json.getFloat("rainConstantA", lidar_setting.rain_constant_a);
        lidar_setting.rain_constant_b = settings_json.getFloat("rainConstantB", lidar_setting.rain_constant_b);

        if (FILE* file = fopen(msr::airlib::Settings::getExecutableFullPath("materials.csv").c_str(), "r")) {
            fclose(file);
            lidar_setting.material_list_file = msr::airlib::Settings::getExecutableFullPath("materials.csv");
        }
        else {
            lidar_setting.material_list_file = msr::airlib::Settings::Settings::getUserDirectoryFullPath("materials.csv");
        }



		lidar_setting.position = createVectorSetting(settings_json, lidar_setting.position);
		lidar_setting.rotation = createRotationSetting(settings_json, lidar_setting.rotation);
	}

    static void initializeEchoSetting(EchoSetting& echo_setting, const Settings& settings_json)
	{
		echo_setting.number_of_traces = settings_json.getInt("NumberOfTraces", echo_setting.number_of_traces);
		echo_setting.sensor_opening_angle = settings_json.getFloat("SensorOpeningAngle", echo_setting.sensor_opening_angle);
		echo_setting.reflection_opening_angle = settings_json.getFloat("ReflectionOpeningAngle", echo_setting.reflection_opening_angle);
		echo_setting.attenuation_per_distance = settings_json.getFloat("AttenuationPerDistance", echo_setting.attenuation_per_distance);
		echo_setting.attenuation_per_reflection = settings_json.getFloat("AttenuationPerReflection", echo_setting.attenuation_per_reflection);
		echo_setting.attenuation_limit = settings_json.getFloat("AttenuationLimit", echo_setting.attenuation_limit);
		echo_setting.distance_limit = settings_json.getFloat("DistanceLimit", echo_setting.distance_limit);
		echo_setting.reflection_limit = settings_json.getInt("ReflectionLimit", echo_setting.reflection_limit);
		echo_setting.reflection_distance_limit = settings_json.getFloat("ReflectionDistanceLimit", echo_setting.reflection_distance_limit);
		echo_setting.measurement_frequency = settings_json.getFloat("MeasurementFrequency", echo_setting.measurement_frequency);
		echo_setting.sensor_diameter = settings_json.getFloat("SensorDiameter", echo_setting.sensor_diameter);
		echo_setting.pause_after_measurement = settings_json.getBool("PauseAfterMeasurement", echo_setting.pause_after_measurement);

		echo_setting.draw_reflected_points = settings_json.getBool("DrawReflectedPoints", echo_setting.draw_reflected_points);
		echo_setting.draw_reflected_lines = settings_json.getBool("DrawReflectedLines", echo_setting.draw_reflected_lines);
		echo_setting.draw_reflected_paths = settings_json.getBool("DrawReflectedPaths", echo_setting.draw_reflected_paths);
		echo_setting.draw_initial_points = settings_json.getBool("DrawInitialPoints", echo_setting.draw_initial_points);
		echo_setting.draw_bounce_lines = settings_json.getBool("DrawBounceLines", echo_setting.draw_bounce_lines);
		echo_setting.draw_sensor = settings_json.getBool("DrawSensor", echo_setting.draw_sensor);
		echo_setting.draw_external_points = settings_json.getBool("DrawExternalPoints", echo_setting.draw_external_points);

		echo_setting.data_frame = settings_json.getString("DataFrame", echo_setting.data_frame);
		echo_setting.ignore_marked = settings_json.getBool("IgnoreMarked", echo_setting.ignore_marked);

		echo_setting.position = createVectorSetting(settings_json, echo_setting.position);
		echo_setting.rotation = createRotationSetting(settings_json, echo_setting.rotation);
	}

    static void initializeSensorTemplateSetting(SensorTemplateSetting& sensortemplate_setting, const Settings& settings_json)
    {
        sensortemplate_setting.measurement_frequency = settings_json.getFloat("MeasurementFrequency", sensortemplate_setting.measurement_frequency);
        sensortemplate_setting.pause_after_measurement = settings_json.getBool("PauseAfterMeasurement", sensortemplate_setting.pause_after_measurement);

        sensortemplate_setting.data_frame = settings_json.getString("DataFrame", sensortemplate_setting.data_frame);

        sensortemplate_setting.attach_link = settings_json.getString("AttachLink", "");

        sensortemplate_setting.position = createVectorSetting(settings_json, sensortemplate_setting.position);
        sensortemplate_setting.rotation = createRotationSetting(settings_json, sensortemplate_setting.rotation);
    }

    static void initializeMarLocUwbSensorSetting(MarLocUwbSetting& marlocUwb_setting, const Settings& settings_json)
    {
        marlocUwb_setting.measurement_frequency = settings_json.getFloat("MeasurementFrequency", marlocUwb_setting.measurement_frequency);
        marlocUwb_setting.pause_after_measurement = settings_json.getBool("PauseAfterMeasurement", marlocUwb_setting.pause_after_measurement);
        marlocUwb_setting.number_of_traces = settings_json.getInt("NumberOfTraces", marlocUwb_setting.number_of_traces);
        marlocUwb_setting.sensor_opening_angle = settings_json.getFloat("SensorOpeningAngle", marlocUwb_setting.sensor_opening_angle);

        marlocUwb_setting.data_frame = settings_json.getString("DataFrame", marlocUwb_setting.data_frame);

        marlocUwb_setting.attach_link = settings_json.getString("AttachLink", "");

        marlocUwb_setting.position = createVectorSetting(settings_json, marlocUwb_setting.position);
        marlocUwb_setting.rotation = createRotationSetting(settings_json, marlocUwb_setting.rotation);
    }

    static std::unique_ptr<SensorSetting> createSensorSetting(
        SensorBase::SensorType sensor_type, const std::string& sensor_name,
        bool enabled)
    {
        std::unique_ptr<SensorSetting> sensor_setting;

        switch (sensor_type) {
        case SensorBase::SensorType::Barometer:
            sensor_setting = std::unique_ptr<SensorSetting>(new BarometerSetting());
            break;
        case SensorBase::SensorType::Imu:
            sensor_setting = std::unique_ptr<SensorSetting>(new ImuSetting());
            break;
        case SensorBase::SensorType::Gps:
            sensor_setting = std::unique_ptr<SensorSetting>(new GpsSetting());
            break;
        case SensorBase::SensorType::Magnetometer:
            sensor_setting = std::unique_ptr<SensorSetting>(new MagnetometerSetting());
            break;
        case SensorBase::SensorType::Distance:
            sensor_setting = std::unique_ptr<SensorSetting>(new DistanceSetting());
            break;
        case SensorBase::SensorType::Lidar:
            sensor_setting = std::unique_ptr<SensorSetting>(new LidarSetting());
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

        switch (sensor_setting->sensor_type) {
        case SensorBase::SensorType::Barometer:
            initializeBarometerSetting(*static_cast<BarometerSetting*>(sensor_setting), settings_json);
            break;
        case SensorBase::SensorType::Imu:
            initializeImuSetting(*static_cast<ImuSetting*>(sensor_setting), settings_json);
            break;
        case SensorBase::SensorType::Gps:
            initializeGpsSetting(*static_cast<GpsSetting*>(sensor_setting), settings_json);
            break;
        case SensorBase::SensorType::Magnetometer:
            initializeMagnetometerSetting(*static_cast<MagnetometerSetting*>(sensor_setting), settings_json);
            break;
        case SensorBase::SensorType::Distance:
            initializeDistanceSetting(*static_cast<DistanceSetting*>(sensor_setting), settings_json);
            break;
        case SensorBase::SensorType::Lidar:
            initializeLidarSetting(*static_cast<LidarSetting*>(sensor_setting), settings_json);
            break;
		case SensorBase::SensorType::GPULidar:
			initializeGPULidarSetting(*static_cast<GPULidarSetting*>(sensor_setting), settings_json);
			break;
		case SensorBase::SensorType::Echo:
            initializeEchoSetting(*static_cast<EchoSetting*>(sensor_setting), settings_json);
            break;
        case SensorBase::SensorType::SensorTemplate:
            initializeSensorTemplateSetting(*static_cast<SensorTemplateSetting*>(sensor_setting), settings_json);
            break;
        case SensorBase::SensorType::MarlocUwb:
            initializeMarLocUwbSensorSetting(*static_cast<MarLocUwbSetting*>(sensor_setting), settings_json);
            break;
        default:
            throw std::invalid_argument("Unexpected sensor type");
        }
    }

    // creates and intializes sensor settings from json
    static void loadSensorSettings( const Settings& settings_json, const std::string& collectionName,
        std::map<std::string, std::unique_ptr<SensorSetting>>& sensors)
    {
        msr::airlib::Settings sensors_child;
        if (settings_json.getChild(collectionName, sensors_child)) {
            std::vector<std::string> keys;
            sensors_child.getChildNames(keys);

            for (const auto& key : keys) {
                msr::airlib::Settings child;
                sensors_child.getChild(key, child);

                auto sensor_type = Utils::toEnum<SensorBase::SensorType>(child.getInt("SensorType", 0));
                auto enabled = child.getBool("Enabled", false);
       
                sensors[key] = createSensorSetting(sensor_type, key, enabled);
                initializeSensorSetting(sensors[key].get(), child);
            }
        }
    }

    // creates default sensor list when none specified in json
    static void createDefaultSensorSettings(const std::string& simmode_name,
        std::map<std::string, std::unique_ptr<SensorSetting>>& sensors)
    {
        if (simmode_name == "Multirotor") {
            sensors["imu"] = createSensorSetting(SensorBase::SensorType::Imu, "imu", true);
            sensors["magnetometer"] = createSensorSetting(SensorBase::SensorType::Magnetometer, "magnetometer", true);
            sensors["gps"] = createSensorSetting(SensorBase::SensorType::Gps, "gps", true);
            sensors["barometer"] = createSensorSetting(SensorBase::SensorType::Barometer, "barometer", true);
        }
        else {
            // no sensors added for other modes
        }
    }

    // loads or creates default sensor list
    static void loadDefaultSensorSettings(const std::string& simmode_name, 
        const Settings& settings_json,
        std::map<std::string, std::unique_ptr<SensorSetting>>& sensors)
    {
        msr::airlib::Settings sensors_child;
        if (settings_json.getChild("DefaultSensors", sensors_child))
            loadSensorSettings(settings_json, "DefaultSensors", sensors);
        else
            createDefaultSensorSettings(simmode_name, sensors);
    }
};

}} //namespace
#endif
