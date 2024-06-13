classdef AirSimClient < handle
    %AIRSIMCLIENT a Matlab client for Cosys-AirSim API
    
    properties
        rpc_client;
        ip;
        port;
        is_drone;
        vehicle_name;   
        car_controls;
        drone_client;
        api_control;
    end
    
    properties (Constant)
        cur_python_path = py.sys.path;       
    end    
    
    methods(Static)        
        function [] = setupPython()
            try
                if exist(AirSimClient.cur_python_path, "file")
                    pyenv("Version", AirSimClient.cur_python_path);
                else
                    error("Python path '%s' not found, change to your local AirSim Python installion ", AirSimClient.cur_python_path);
                end
            catch matlabException
                if (strcmp(matlabException.identifier, "MATLAB:Pyenv:PythonLoadedInProcess"))
                   % Do nothing 
                else
                    rethrow(matlabException);
                end
            end
        end
        
        function [rpc_client] = setupRPC(ip, port)
            try
                rpc_client = py.msgpackrpc.Client(py.msgpackrpc.Address(ip, py.int(port)), ...
                                                pyargs("timeout", py.int(0.5), "pack_encoding", "utf-8", "unpack_encoding", "utf-8"));
                rpc_client.call("ping");
            catch matlabException
                if (strcmp(matlabException.identifier, "MATLAB:Python:PyException"))
                    warning("The simulator is probably not running.");
                end
                rethrow(matlabException);
            end
        end

        function carControls = getCarControls()
            carClient = py.airsim.CarClient();
            carClient.confirmConnection();
            carClient.enableApiControl(true);
            carControls = py.airsim.CarControls();
        end

        function droneClient = getDroneControls()
            droneClient = py.airsim.MultirotorClient();
            droneClient.confirmConnection();
            droneClient.enableApiControl(true);
        end

        function [pointCloud] = nedToLeftHandCoordinates(pointCloud)
            % AirSim uses NED coordinate system, i.e., +X is North, +Y is East and +Z is Down.
            % Convert to left-hand coordinates (invert Z axis)
            pointCloud(:, 3) = -pointCloud(:, 3);
        end

        function [pointCloud] = nedToRightHandCoordinates(pointCloud)
            % AirSim uses NED coordinate system, i.e., +X is North, +Y is East and +Z is Down.
            % Convert to right-hand coordinates (invert Z and Y axis)
            pointCloud(:, 3) = -pointCloud(:, 3);
            pointCloud(:, 2) = -pointCloud(:, 2);
        end

        function [colorMap] = getInstanceSegmentationColormap()
            % GET_INSTANCE_SEGMENTATION_COLORMAP Get full RGB colormap
            %
            % You can generate manually (see AirSimGenerateColorMap.m)
            % but this will save an hour.

            colorMap = uint8(readmatrix("colormap.csv"));            
        end
    end
    
    methods
        function obj = AirSimClient(varargin)
            argParser = inputParser();
            argParser.addOptional("IsDrone", false, @islogical);
            argParser.addOptional("ApiControl", false, @islogical);
            argParser.addOptional("IP", "127.0.0.1", @isstring);
            argParser.addOptional("Port", 41451, @isnumeric);
            argParser.addOptional("VehicleName", "airsimvehicle", @isstring);
            argParser.parse(varargin{:});

            obj.is_drone = argParser.Results.IsDrone;
            obj.api_control = argParser.Results.ApiControl;
            obj.ip = argParser.Results.IP;
            obj.port = argParser.Results.Port;
            obj.vehicle_name = argParser.Results.VehicleName;            

            obj.rpc_client = AirSimClient.setupRPC(obj.ip, obj.port);
            
            if obj.api_control
                if obj.is_drone
                    obj.drone_client = AirSimClient.getDroneClient();
                else
                    obj.car_controls = AirSimClient.getCarControls();
                end
            end 
        end                
        
        function [vehiclePose] = getVehiclePose(obj)
            vehicleState = obj.rpc_client.call("simGetVehiclePose", obj.vehicle_name);            
            vehiclePose.position = obj.nedToRightHandCoordinates(struct2array(struct(vehicleState{"position"})));
            vehiclePose.orientation = quatinv(struct2array(struct(vehicleState{"orientation"})));
        end
        
        function [kinematicsState] = getGroundTruthKinematics(obj)
            vehicleStateAirSim = obj.rpc_client.call("simGetGroundTruthKinematics", obj.vehicle_name);
            kinematicsState.position = obj.nedToRightHandCoordinates(struct2array(struct(vehicleStateAirSim{"position"})));
            kinematicsState.orientation = quatinv(struct2array(struct(vehicleStateAirSim{"orientation"})));
            kinematicsState.linear_velocity = obj.nedToRightHandCoordinates(struct2array(struct(vehicleStateAirSim{"linear_velocity"})));
            kinematicsState.angular_velocity = obj.nedToRightHandCoordinates(struct2array(struct(vehicleStateAirSim{"angular_velocity"})));
            kinematicsState.linear_acceleration = obj.nedToRightHandCoordinates(struct2array(struct(vehicleStateAirSim{"linear_acceleration"})));
            kinematicsState.angular_acceleration = obj.nedToRightHandCoordinates(struct2array(struct(vehicleStateAirSim{"angular_acceleration"})));
        end

        function simSetKinematics(obj, position, orientation, linear_velocity, angular_velocity, linear_acceleration, angular_acceleration)

            kinematicsState.position.x_val = position(1);
            kinematicsState.position.y_val = -position(2);
            kinematicsState.position.z_val = -position(3);
            orientation = quatinv(orientation);            
            kinematicsState.orientation.w_val = orientation(1);
            kinematicsState.orientation.x_val = orientation(2);
            kinematicsState.orientation.y_val = orientation(3);
            kinematicsState.orientation.z_val = orientation(4);
            kinematicsState.linear_velocity.x_val = linear_velocity(1);
            kinematicsState.linear_velocity.y_val = -linear_velocity(2);
            kinematicsState.linear_velocity.z_val = -linear_velocity(3);
            kinematicsState.angular_velocity.x_val = angular_velocity(1);
            kinematicsState.angular_velocity.y_val = -angular_velocity(2);
            kinematicsState.angular_velocity.z_val = -angular_velocity(3);
            kinematicsState.linear_acceleration.x_val = linear_acceleration(1);
            kinematicsState.linear_acceleration.y_val = -linear_acceleration(2);
            kinematicsState.linear_acceleration.z_val = -linear_acceleration(3);
            kinematicsState.angular_acceleration.x_val = angular_acceleration(1);
            kinematicsState.angular_acceleration.y_val = -angular_acceleration(2);
            kinematicsState.angular_acceleration.z_val = -angular_acceleration(3);
            obj.rpc_client.call("setKinematics", kinematicsState, ignore_collision, obj.vehicle_name);
        end

        function [EnvironmentState] = getGroundTruthEnvironment(obj)
            EnvironmentStateData = obj.rpc_client.call("simGetGroundTruthEnvironment", obj.vehicle_name);
            EnvironmentState.position = obj.nedToRightHandCoordinates(struct2array(struct(EnvironmentStateData{"position"})));
            EnvironmentState.gravity = obj.nedToRightHandCoordinates(struct2array(struct(EnvironmentStateData{"gravity"})));
            EnvironmentState.air_pressure = double(EnvironmentStateData{"air_pressure"});
            EnvironmentState.temperature = double(EnvironmentStateData{"temperature"});
            EnvironmentState.air_density = double(EnvironmentStateData{"air_density"});


            curGeoPointData = EnvironmentStateData{"geo_point"};
            geopoint.latitude = double(curGeoPointData{"latitude"});
            geopoint.longitude = double(curGeoPointData{"longitude"});
            geopoint.altitude = double(curGeoPointData{"altitude"});
            EnvironmentState.geo_point = geopoint;            
        end
        
        function [] = setVehiclePose(obj, position, orientation, ignoreCollisions)
            if nargin() < 4
                ignoreCollisions = true;
            end
            
            newPose.position.x_val = position(1);
            newPose.position.y_val = -position(2);
            newPose.position.z_val = -position(3);
            orientation = quatinv(orientation);
            
            newPose.orientation.w_val = orientation(1);
            newPose.orientation.x_val = orientation(2);
            newPose.orientation.y_val = orientation(3);
            newPose.orientation.z_val = orientation(4);
            
            obj.rpc_client.call("simSetVehiclePose", newPose, ignoreCollisions, obj.vehicle_name);
        end
        
        function [] = setVehicleControls(obj, throttle, steering)
            if ~obj.is_drone
                if throttle < 0
                    obj.car_controls.manual_gear = py.int(-1);
                    obj.car_controls.is_manual_gear = true;
                else
                    obj.car_controls.manual_gear = py.int(0);
                    obj.car_controls.is_manual_gear = false;
                end
            end
            
            obj.car_controls.throttle = throttle;
            obj.car_controls.steering = steering;
            
            obj.rpc_client.call("setCarControls", obj.car_controls, obj.vehicle_name);
        end

        % Enable disable or prompt api control status
        % Necessary to send speed control from matlab to the api
        function setEnableApiControl(obj)
           obj.rpc_client.call("enableApiControl", true, obj.vehicle_name);
        end

        function setDisableApiControl(obj)
            obj.rpc_client.call("enableApiControl", false, obj.vehicle_name);
        end   

        function isEnabled = getApiControlEnabled(obj)
            isEnabled = obj.rpc_client.call("isApiControlEnabled", obj.vehicle_name);
        end
        
        function setEnableDroneArm(obj)
           obj.rpc_client.call("armDisarm", true, obj.vehicle_name);
        end

        function setDisableDroneArm(obj)
           obj.rpc_client.call("armDisarm", false, obj.vehicle_name);
        end

        function pause(obj)
           obj.rpc_client.call("simPause", true);
        end

        function unpause(obj)
           obj.rpc_client.call("simPause", false);
        end

        function IsPaused = isPaused(obj)
            IsPaused = obj.rpc_client.call("simIsPaused");
        end

        function continueForFrames(obj, frames)
            obj.rpc_client.call("simContinueForFrames", frames);
        end

        function [imuData, timestamp] = getIMUData(obj, sensorName)
            % GET_IMU_INFO Get IMU sensor data
            %
            % The IMU is always at the center of the vehicle.

            data = obj.rpc_client.call("getImuData", sensorName, obj.vehicle_name);

            timestamp = floor(double(double(data{"time_stamp"}))/1e9);

            imuData.orientation = quatinv(struct2array(struct(data{"orientation"})));
            imuData.angularVelocity = obj.nedToRightHandCoordinates(struct2array(struct(data{"angular_velocity"})));
            imuData.linearAcceleration = obj.nedToRightHandCoordinates(struct2array(struct(data{"linear_acceleration"})));
        end

        function [barometerData, timestamp] = getBarometerData(obj, sensorName)

            data = obj.rpc_client.call("getBarometerData", sensorName, obj.vehicle_name);

            timestamp = floor(double(double(data{"time_stamp"}))/1e9);
            barometerData.altitude = quatinv(struct2array(struct(data{"altitude"})));
            barometerData.pressure = obj.nedToRightHandCoordinates(struct2array(struct(data{"pressure"})));
            barometerData.qnh = obj.nedToRightHandCoordinates(struct2array(struct(data{"qnh"})));
        end

        function [MagnetometerData, timestamp] = getMagnetometerData(obj, sensorName)

            data = obj.rpc_client.call("getMagnetometerData", sensorName, obj.vehicle_name);

            timestamp = floor(double(double(data{"time_stamp"}))/1e9);
            MagnetometerData.magnetic_field_body = obj.nedToRightHandCoordinates(struct2array(struct(data{"magnetic_field_body"})));
            MagnetometerData.magnetic_field_covariance = double(data{"magnetic_field_covariance"});
        end

        function [GpsData, timestamp] = getGpsData(obj, sensorName)

            data = obj.rpc_client.call("getGpsData", sensorName, obj.vehicle_name);

            timestamp = floor(double(double(data{"time_stamp"}))/1e9);

            gnssDataRaw = data{"gnss"};
            gnssData.eph = double(gnssDataRaw{"eph"});
            gnssData.epv = double(gnssDataRaw{"epv"});
            gnssData.velocity = struct2array(struct(gnssDataRaw{"velocity"}));

            curGeoPointData = gnssDataRaw{"geo_point"};
            geopoint.latitude = double(curGeoPointData{"latitude"});
            geopoint.longitude = double(curGeoPointData{"longitude"});
            geopoint.altitude = double(curGeoPointData{"altitude"});
            gnssData.geo_point = geopoint;            
            gnssData.fix_type = double(curGeoPointData{"fix_type"});
            gnssData.time_utc = floor(double(double(curGeoPointData{"time_utc"}))/1e9);
            GpsData.gnss = gnssData;      
            
            GpsData.is_valid = data{"is_valid"};
        end

        function [DistanceSensorData, timestamp] = getDistanceSensorData(obj, sensorName)

            data = obj.rpc_client.call("getDistanceSensorData", sensorName, obj.vehicle_name);

            timestamp = floor(double(double(data{"time_stamp"}))/1e9);
            curPoseData = data{"relative_pose"};
            DistanceSensorData.relative_pose.position = obj.nedToRightHandCoordinates(struct2array(struct(curPoseData{"position"})));
            DistanceSensorData.relative_pose.orientation = quatinv(struct2array(struct(curPoseData{"orientation"})));
            DistanceSensorData.distance = double(data{"distance"});
            DistanceSensorData.max_distance = double(data{"max_distance"});
            DistanceSensorData.min_distance = double(data{"min_distance"});
        end

        function [activePointCloud, activeData, passivePointCloud, passiveData, timestamp, sensorPose] = getEchoData(obj, sensorName, enablePassive)
            % GET_ECHO_DATA Get sensor data from an echo sensor
            %
            % The reflection direction for the passive pointcloud is saved 
            % in the normal field of the pointcloud.  
            
            echoData = obj.rpc_client.call("getEchoData", sensorName, obj.vehicle_name);

            timestamp = floor(double(double(echoData{"time_stamp"}))/1e9);

            % Get the sensor pose
            sensorPose.position = obj.nedToRightHandCoordinates(struct2array(struct(echoData{"pose"}{"position"})));
            sensorPose.orientation = quatinv(struct2array(struct(echoData{"pose"}{"orientation"})));
            
            % Get pointcloud data
            passivePointCloud = [];
            passiveData = {};
            activePointCloud = [];
            activeData = {};

            reflectorPointcloudRaw = cell2mat(cell(echoData{"point_cloud"}));
            reflectorPointcloudPassiveRaw = cell2mat(cell(echoData{"passive_beacons_point_cloud"}));
            
            if mod(numel(reflectorPointcloudRaw), 6) == 0 % Discard malformed point clouds
                activeData.labels = string(cell(echoData{"groundtruth"}))';
                reflectorPointcloudRaw = reshape(reflectorPointcloudRaw, 6, []).';
                reflectorPointcloudRaw = obj.nedToRightHandCoordinates(reflectorPointcloudRaw);
                activeData.attenuation = reflectorPointcloudRaw(:, 4); 
                activeData.distance = reflectorPointcloudRaw(:, 5); 
                activeData.reflections = reflectorPointcloudRaw(:, 6);
                activePointCloud = pointCloud(reflectorPointcloudRaw(:, 1:3));
            end
            
            if enablePassive && mod(numel(reflectorPointcloudPassiveRaw), 9) == 0
                allPassiveLabels = string(cell(echoData{"passive_beacons_groundtruth"}));
                passiveData.labels = allPassiveLabels(2:2:length(allPassiveLabels))';
                passiveData.reflectionLabels = allPassiveLabels(1:2:length(allPassiveLabels))';
                reflectorPointcloudPassiveRaw = reshape(reflectorPointcloudPassiveRaw, 9, []).';
                reflectorPointcloudPassiveRaw = obj.nedToRightHandCoordinates(reflectorPointcloudPassiveRaw);
                passiveData.attenuation = reflectorPointcloudPassiveRaw(:, 4); 
                passiveData.distance = reflectorPointcloudPassiveRaw(:, 5); 
                passiveData.reflections = reflectorPointcloudPassiveRaw(:, 6);
                normalData = reflectorPointcloudPassiveRaw(:, 7:9);
                normalData(:,2:3) = -normalData(:, 2:3);
                passivePointCloud = pointCloud(reflectorPointcloudPassiveRaw(:, 1:3), Normal=normalData);
            end               
        end   

        function [lidarPointCloud, lidarLabels, timestamp, sensorPose] = getLidarData(obj, sensorName, enableLabels)
            % GET_LIDAR_DATA Get sensor data from a lidar sensor
            
            lidarData = obj.rpc_client.call("getLidarData", sensorName, obj.vehicle_name);

            timestamp = floor(double(double(lidarData{"time_stamp"}))/1e9);

            % Get the sensor pose
            sensorPose.position = obj.nedToRightHandCoordinates(struct2array(struct(lidarData{"pose"}{"position"})));
            sensorPose.orientation = quatinv(struct2array(struct(lidarData{"pose"}{"orientation"})));
            
            % Get pointcloud data
            lidarPointCloud = [];
            lidarLabels = [];

            lidarPointcloudRaw = cell2mat(cell(lidarData{"point_cloud"}));
            
            if mod(numel(lidarPointcloudRaw), 3) == 0 % Discard malformed point clouds

                if enableLabels
                    lidarLabels = string(cell(lidarData{"groundtruth"}))';
                end
                lidarPointcloudRaw = reshape(lidarPointcloudRaw, 3, []).';
                lidarPointcloudRaw = obj.nedToRightHandCoordinates(lidarPointcloudRaw);
                lidarPointCloud = pointCloud(lidarPointcloudRaw); 
            end    
        end   

        function [lidarPointCloud, timestamp, sensorPose] = getGPULidarData(obj, sensorName)
            % GET_LIDAR_DATA Get sensor data from a GPU lidar sensor
            
            lidarData = obj.rpc_client.call("getGPULidarData", sensorName, obj.vehicle_name);

            timestamp = floor(double(double(lidarData{"time_stamp"}))/1e9);

            % Get the sensor pose
            sensorPose.position = obj.nedToRightHandCoordinates(struct2array(struct(lidarData{"pose"}{"position"})));
            sensorPose.orientation = quatinv(struct2array(struct(lidarData{"pose"}{"orientation"})));
            
            % Get pointcloud data
            lidarPointCloud = [];

            lidarPointcloudRaw = cell2mat(cell(lidarData{"point_cloud"}));
            
            if mod(numel(lidarPointcloudRaw), 5) == 0 % Discard malformed point clouds
                lidarPointcloudRaw = reshape(lidarPointcloudRaw, 5, []).';
                lidarPointcloudRaw = obj.nedToRightHandCoordinates(lidarPointcloudRaw);
                colorValues = uint32(lidarPointcloudRaw(:, 4));
                lidarLabelColors = zeros(size(lidarPointcloudRaw, 1), 3, 'uint8');
                lidarLabelColors(:, 1) = bitshift(bitand(colorValues, hex2dec('FF0000')), -16);
                lidarLabelColors(:, 2) = bitshift(bitand(colorValues, hex2dec('00FF00')), -8);
                lidarLabelColors(:, 3) = bitand(colorValues, hex2dec('0000FF'));
                % for index = 1:numel(colorValues)
                %     lidarLabelColors(index, 1) = bitshift(bitand(colorValues(index), hex2dec('FF0000')), -16);
                %     lidarLabelColors(index, 2) = bitshift(bitand(colorValues(index), hex2dec('00FF00')), -8);
                %     lidarLabelColors(index, 3) = bitand(colorValues(index), hex2dec('0000FF'));
                % end
                lidarPointCloud = pointCloud(lidarPointcloudRaw(:,1:3), Color=lidarLabelColors, Intensity=lidarPointcloudRaw(:, 5)); 
            end    
        end    

        function [image, timestamp] = getCameraImage(obj, sensorName, cameraType, annotationLayer)
            arguments
                obj AirSimClient
                sensorName string 
                cameraType uint32
                annotationLayer string = ""
            end
            % GET_CAMERA_IMAGE Get camera data from a camera sensor

            if cameraType == 1 || cameraType == 2 || cameraType == 3 || cameraType == 4
                image_request = py.airsim.ImageRequest(sensorName, int32(cameraType), true, false);
            elseif cameraType == 10
                image_request = py.airsim.ImageRequest(sensorName, int32(cameraType), false, false, annotationLayer);
            else
                image_request = py.airsim.ImageRequest(sensorName, int32(cameraType), false, false);
            end
            image_request_list = py.list({image_request});
            camera_image_response_request = obj.rpc_client.call("simGetImages", image_request_list, obj.vehicle_name);
            image_response = py.airsim.ImageResponse();
            camera_image = image_response.from_msgpack(camera_image_response_request{1});
            if cameraType == 1 || cameraType == 2 || cameraType == 3 || cameraType == 4
                image_bytes = single(camera_image.image_data_float);
                image_reshaped = reshape(image_bytes, camera_image.width.int32, camera_image.height.int32);
                image = permute(image_reshaped,[2 1]);
            else
                image_bytes = uint8(camera_image.image_data_uint8);
                image_reshaped = reshape(image_bytes, 3, camera_image.width.int32, camera_image.height.int32);
                image = permute(image_reshaped,[3 2 1]);
            end
            timestamp = floor(double(double(camera_image.time_stamp))/1e9);
        end

        function [images, timestamp] = getCameraImages(obj, sensorName, cameraTypes, annotationLayers)
            arguments
                obj AirSimClient
                sensorName string 
                cameraTypes uint32
                annotationLayers string = ""
            end
            % GET_CAMERA_IMAGES Get syncedcamera data from a camera sensor
            images = {};
            image_requests = [];
            for i = 1: numel(cameraTypes)
                if cameraTypes(i) == 1 || cameraTypes(i) == 2 || cameraTypes(i) == 3 || cameraTypes(i) == 4
                    image_requests{i} = py.airsim.ImageRequest(sensorName, int32(cameraTypes(i)), true, false);
                elseif cameraTypes(i) == 10
                    image_requests{i} = py.airsim.ImageRequest(sensorName, int32(cameraTypes(i)), false, false, annotationLayers(i));
                else
                    image_requests{i} = py.airsim.ImageRequest(sensorName, int32(cameraTypes(i)), false, false);
                end
            end
            image_request_list = py.list(image_requests);
            camera_image_response_request = obj.rpc_client.call("simGetImages", image_request_list, obj.vehicle_name);

            for i = 1: numel(cameraTypes)
                image_response = py.airsim.ImageResponse();
                camera_image = image_response.from_msgpack(camera_image_response_request{i});
                if cameraTypes(i) == 1 || cameraTypes(i) == 2 || cameraTypes(i) == 3 || cameraTypes(i) == 4
                    image_bytes = single(camera_image.image_data_float);
                    image_reshaped = reshape(image_bytes, camera_image.width.int32, camera_image.height.int32);
                    images{i} = permute(image_reshaped,[2 1]);
                else
                    image_bytes = uint8(camera_image.image_data_uint8);
                    image_reshaped = reshape(image_bytes, 3, camera_image.width.int32, camera_image.height.int32);
                    images{i} = rescale(permute(image_reshaped,[3 2 1]));
                end
            end
            timestamp = floor(double(double(camera_image.time_stamp))/1e9);
        end

        function [intrinsics, sensorPose] = getCameraInfo(obj, sensorName)
            % GET_CAMERA_INFO Get camera pose and intrinsics
            %
            % distortion is not implemented yet! 

            cameraData = obj.rpc_client.call("simGetCameraInfo", sensorName, obj.vehicle_name);
            sensorPose.position = obj.nedToRightHandCoordinates(struct2array(struct(cameraData{"pose"}{"position"})));
            sensorPose.orientation = quatinv(struct2array(struct(cameraData{"pose"}{"orientation"})));
            curFov = cameraData{"fov"};
            tempRequest = py.airsim.ImageRequest(sensorName, int32(1), true, false);
            tempRequestList = py.list({tempRequest});
            tempCameraResponseRequest = obj.rpc_client.call("simGetImages", tempRequestList, obj.vehicle_name);
            testImageResponse = py.airsim.ImageResponse();
            tempCameraImage = testImageResponse.from_msgpack(tempCameraResponseRequest{1});
            cameraWidth = tempCameraImage.width.int32;
            cameraHeight = tempCameraImage.height.int32;
            focalLength = (double(cameraWidth) / 2.0) / tan(curFov * pi / 360);
            intrinsics = cameraIntrinsics([focalLength, focalLength], [double(cameraWidth / 2), double(cameraHeight / 2)], [double(cameraWidth), double(cameraHeight)]);
        end

        function [wifiState] = getWifiState(obj, sensorName)
            wifiState = struct();
            wifiStateAirsim = obj.rpc_client.call("getWifiData", sensorName, obj.vehicle_name);
            wifiState.wr_time_stamp = [];
            wifiState.wr_anchorId = string([]);
            wifiState.wr_anchorPosX = [];
            wifiState.wr_anchorPosY = [];
            wifiState.wr_anchorPosZ = [];
            wifiState.wr_valid_range = [];
            wifiState.wr_distance = [];
            wifiState.wr_rssi = [];
            wifiState.wra_ranges = {};

            wr_time_stamp  = cellfun(@double,cell(wifiStateAirsim{'wr_time_stamp'}));
            wr_anchorId    = cellfun(@string,cell(wifiStateAirsim{'wr_anchorId'}));
            wr_anchorPosX  = cellfun(@double,cell(wifiStateAirsim{'wr_anchorPosX'}));
            wr_anchorPosY  = cellfun(@double,cell(wifiStateAirsim{'wr_anchorPosY'}));
            wr_anchorPosZ  = cellfun(@double,cell(wifiStateAirsim{'wr_anchorPosZ'}));
            wr_valid_range = cellfun(@double,cell(wifiStateAirsim{'wr_valid_range'}));
            wr_distance    = cellfun(@double,cell(wifiStateAirsim{'wr_distance'}));
            wr_rssi        = cellfun(@double,cell(wifiStateAirsim{'wr_rssi'}));
            wifiState.wra_tagId      = cellfun(@string,cell(wifiStateAirsim{'wra_tagId'}));
            wifiState.wra_tagPosX    = cellfun(@double,cell(wifiStateAirsim{'wra_tagPosX'}));
            wifiState.wra_tagPosY    = cellfun(@double,cell(wifiStateAirsim{'wra_tagPosY'}));
            wifiState.wra_tagPosZ    = cellfun(@double,cell(wifiStateAirsim{'wra_tagPosZ'}));
            wra_ranges = {};
            for i_ranges = 1:size(wifiStateAirsim{'wra_ranges'}, 2)
                wra_ranges{i_ranges, 1} = cellfun(@double,cell(wifiStateAirsim{'wra_ranges'}{i_ranges})); 
            end

            % Remove duplicates from ranges
            idx_offset = 0;
            for i_ranges = 1:size(wra_ranges, 1) % For all ranges (1 vector per rangeArray)
                ranges = cell2mat(wra_ranges(i_ranges)); % Find ranges for this range array
                [~, unqRangesId] = unique(wr_anchorId(ranges+1)); % Find unique indexes
                unqRangesId = unqRangesId' + idx_offset;
                
                wra_ranges_idx_start = size(wifiState.wr_distance, 2)+1;
                wifiState.wr_time_stamp (end+1:end+size(unqRangesId, 2)) = wr_time_stamp (unqRangesId);
                wifiState.wr_anchorId   (end+1:end+size(unqRangesId, 2)) = wr_anchorId   (unqRangesId);
                wifiState.wr_anchorPosX (end+1:end+size(unqRangesId, 2)) = wr_anchorPosX (unqRangesId);
                wifiState.wr_anchorPosY (end+1:end+size(unqRangesId, 2)) = wr_anchorPosY (unqRangesId);
                wifiState.wr_anchorPosZ (end+1:end+size(unqRangesId, 2)) = wr_anchorPosZ (unqRangesId);
                wifiState.wr_valid_range(end+1:end+size(unqRangesId, 2)) = wr_valid_range(unqRangesId);
                %wifiState.wr_distance   (end+1:end+size(unqRangesId, 2)) = wr_distance   (unqRangesId);
                wra_ranges_idx_stop = size(wifiState.wr_valid_range, 2);
                
                
                wifiState.wra_ranges{i_ranges, 1} = [wra_ranges_idx_start:wra_ranges_idx_stop];

                for i_range = 1:size(unqRangesId, 2)
                    currentRanges = wr_anchorId(ranges+1);
                    currentRssi = wr_rssi(ranges+1);
                    currentDistances = wr_distance(ranges+1);
                    [maxRssi, maxRssiIdx] = max(currentRssi .* (currentRanges == wr_anchorId(i_range+idx_offset)));
                    wifiState.wr_distance(end+1) = currentDistances(maxRssiIdx);
                    wifiState.wr_rssi(end+1) = maxRssi;
                end
                

                idx_offset = idx_offset+size(ranges, 2);
            end
        end

        function [uwbState] = getUWBState(obj)
            uwbState = struct();
            %uwbStateAirsim = obj.rpc_client.call("getUWBData", sensorName, obj.vehicle_name);
            uwbStateAirsim = obj.rpc_client.call("getUWBData", "", obj.vehicle_name);
            
            uwbState.mur_time_stamp = [];
            uwbState.mur_anchorId = string([]);
            uwbState.mur_anchorPosX = [];
            uwbState.mur_anchorPosY = [];
            uwbState.mur_anchorPosZ = [];
            uwbState.mur_valid_range = [];
            uwbState.mur_distance = [];
            uwbState.mur_rssi = [];
            uwbState.mura_ranges = {};

            mur_time_stamp  = cellfun(@double,cell(uwbStateAirsim{'mur_time_stamp'}));
            mur_anchorId    = cellfun(@string,cell(uwbStateAirsim{'mur_anchorId'}));
            mur_anchorPosX  = cellfun(@double,cell(uwbStateAirsim{'mur_anchorPosX'}));
            mur_anchorPosY  = cellfun(@double,cell(uwbStateAirsim{'mur_anchorPosY'}));
            mur_anchorPosZ  = cellfun(@double,cell(uwbStateAirsim{'mur_anchorPosZ'}));
            mur_valid_range = cellfun(@double,cell(uwbStateAirsim{'mur_valid_range'}));
            mur_distance    = cellfun(@double,cell(uwbStateAirsim{'mur_distance'}));
            mur_rssi        = cellfun(@double,cell(uwbStateAirsim{'mur_rssi'}));
            uwbState.mura_tagId      = cellfun(@string,cell(uwbStateAirsim{'mura_tagId'}));
            uwbState.mura_tagPosX    = cellfun(@double,cell(uwbStateAirsim{'mura_tagPosX'}));
            uwbState.mura_tagPosY    = cellfun(@double,cell(uwbStateAirsim{'mura_tagPosY'}));
            uwbState.mura_tagPosZ    = cellfun(@double,cell(uwbStateAirsim{'mura_tagPosZ'}));
            mura_ranges = {};
            for i_ranges = 1:size(uwbStateAirsim{'mura_ranges'}, 2)
                mura_ranges{i_ranges, 1} = cellfun(@double,cell(uwbStateAirsim{'mura_ranges'}{i_ranges})); 
            end

            % Remove duplicates from ranges
            idx_offset = 0;
            for i_ranges = 1:size(mura_ranges, 1) % For all ranges (1 vector per rangeArray)
                ranges = cell2mat(mura_ranges(i_ranges)); % Find ranges for this range array
                [~, unqRangesId] = unique(mur_anchorId(ranges+1)); % Find unique indexes
                unqRangesId = unqRangesId' + idx_offset;
                
                mura_ranges_idx_start = size(uwbState.mur_distance, 2)+1;
                uwbState.mur_time_stamp (end+1:end+size(unqRangesId, 2)) = mur_time_stamp (unqRangesId);
                uwbState.mur_anchorId   (end+1:end+size(unqRangesId, 2)) = mur_anchorId   (unqRangesId);
                uwbState.mur_anchorPosX (end+1:end+size(unqRangesId, 2)) = mur_anchorPosX (unqRangesId);
                uwbState.mur_anchorPosY (end+1:end+size(unqRangesId, 2)) = mur_anchorPosY (unqRangesId);
                uwbState.mur_anchorPosZ (end+1:end+size(unqRangesId, 2)) = mur_anchorPosZ (unqRangesId);
                uwbState.mur_valid_range(end+1:end+size(unqRangesId, 2)) = mur_valid_range(unqRangesId);
                %uwbState.mur_distance   (end+1:end+size(unqRangesId, 2)) = mur_distance   (unqRangesId);
                mura_ranges_idx_stop = size(uwbState.mur_valid_range, 2);
                
                
                uwbState.mura_ranges{i_ranges, 1} = [mura_ranges_idx_start:mura_ranges_idx_stop];

                for i_range = 1:size(unqRangesId, 2)
                    currentRanges = mur_anchorId(ranges+1);
                    currentRssi = mur_rssi(ranges+1);
                    currentDistances = mur_distance(ranges+1);
                    [maxRssi, maxRssiIdx] = max(currentRssi .* (currentRanges == mur_anchorId(i_range+idx_offset)));
                    uwbState.mur_distance(end+1) = currentDistances(maxRssiIdx);
                    uwbState.mur_rssi(end+1) = maxRssi;
                end
                

                idx_offset = idx_offset+size(ranges, 2);

                %uwbState.mur_rssi = max(mur_rssi(ranges+1) .* (mur_anchorId(ranges+1) ==  "testBeacon_25:2"))



                %uwbState.mur_rssi(end+1) = mur_rssi(unqRangesId);
            end
        end

        function [uwbState] = getUWBSensorState(obj, sensorName)
          uwbState = struct();
          %uwbStateAirsim = struct(obj.rpc_client.call("getWifiData", "wifi", obj.vehicle_names(idx)));
          %uwbStateAirsim = struct(obj.rpc_client.call("getUWBSensorData", "UnrealMarLocUwbSensor", obj.vehicle_names(idx)));
          uwbStateAirsim = obj.rpc_client.call("getUWBSensorData", sensorName, obj.vehicle_name);
          ts = uwbStateAirsim(1);
          ts = double(ts{1});
          uwbState.timestamp = ts;

          pos = uwbStateAirsim(2);
          pos = struct(pos{1});
          pos.position = struct(pos.position);
          pos.orientation = struct(pos.orientation);
          uwbState.position = pos;

          beaconID = uwbStateAirsim(3);
          beaconID = cellfun(@double,cell(beaconID{1}));
          beaconX = uwbStateAirsim(4);
          beaconX = cellfun(@double,cell(beaconX{1}));
          beaconY = uwbStateAirsim(5);
          beaconY = cellfun(@double,cell(beaconY{1}));
          beaconZ = uwbStateAirsim(6);
          beaconZ = cellfun(@double,cell(beaconZ{1}));

          uwbState.beaconPos = [beaconID', beaconX', beaconY', beaconZ'];
        end

        function [wifiState] = getWifiSensorState(obj, sensorName)
          wifiState = struct();
          %wifiStateAirsim = struct(obj.rpc_client.call("getWifiData", "wifi", obj.vehicle_names(idx)));
          %wifiStateAirsim = struct(obj.rpc_client.call("getwifiSensorData", "UnrealMarLocwifiSensor", obj.vehicle_names(idx)));
          wifiStateAirsim = obj.rpc_client.call("getWifiSensorData", sensorName, obj.vehicle_name);
          ts = wifiStateAirsim(1);
          ts = double(ts{1});
          wifiState.timestamp = ts;

          pos = wifiStateAirsim(2);
          pos = struct(pos{1});
          pos.position = struct(pos.position);
          pos.orientation = struct(pos.orientation);
          wifiState.position = pos;

          beaconID = wifiStateAirsim(3);
          beaconID = cellfun(@double,cell(beaconID{1}));
          beaconX = wifiStateAirsim(4);
          beaconX = cellfun(@double,cell(beaconX{1}));
          beaconY = wifiStateAirsim(5);
          beaconY = cellfun(@double,cell(beaconY{1}));
          beaconZ = wifiStateAirsim(6);
          beaconZ = cellfun(@double,cell(beaconZ{1}));

          wifiState.beaconPos = [beaconID', beaconX', beaconY', beaconZ'];
        end

        function geopoint = getHomeGeoPoint(obj)
            geopointData = obj.rpc_client.call("getHomeGeoPoint", obj.vehicle_name);
            geopoint.latitude = double(geopointData{"latitude"});
            geopoint.longitude = double(geopointData{"longitude"});
            geopoint.altitude = double(geopointData{"altitude"});
        end

        function setLightIntensity(obj, light_name, intensity)
            obj.rpc_client.call("simSetLightIntensity", light_name, intensity);
        end

        function swapTextures(obj, tags, tex_id, component_id, material_id)
            obj.rpc_client.call("simSwapTextures", tags, tex_id, component_id, material_id);
        end

        function setObjectMaterial(obj, object_name, material_name, component_id)
            obj.rpc_client.call("simSetObjectMaterial", object_name, material_name, component_id);
        end
        
        function setObjectMaterialFromTexture(obj, object_name, texture_path, component_id)
            obj.rpc_client.call("simSetObjectMaterialFromTexture", object_name, texture_path, component_id);
        end

        function setTimeOfDay(obj, is_enabled, start_datetime, is_start_datetime_dst, celestial_clock_speed,...
                              update_interval_secs, move_sun)
            obj.rpc_client.call("simSetTimeOfDay", is_enabled, start_datetime, is_start_datetime_dst, celestial_clock_speed,...
                                update_interval_secs, move_sun);
        end

        function flushPersistentMarkers(obj)
            obj.rpc_client.call('simFlushPersistentMarkers');
        end               
        
        function cancelLastTask(obj)
            obj.rpc_client.call('cancelLastTask', obj.vehicle_name);
        end
        
        function startRecording(obj)
            obj.rpc_client.call('startRecording');
        end
        
        function stopRecording(obj)
            obj.rpc_client.call('stopRecording');
        end
        
        function recording = isRecording(obj)
            recording = obj.rpc_client.call('isRecording');
        end
        
        function setWind(obj, wind)
            obj.rpc_client.call('simSetWind', wind);
        end
        
        function success = createVoxelGrid(obj, position, x, y, z, res, of)
            newPostiton.x_val = position(1);
            newPostiton.y_val  = -position(2);
            newPostiton.z_val  = -position(3);
            success = obj.rpc_client.call('simCreateVoxelGrid', newPostiton, x, y, z, res, of);
        end
        
        function success = addVehicle(obj, vehicle_name, vehicle_type, position, orientation, pawn_path)
            arguments
                obj AirSimClient
                vehicle_name string
                vehicle_type string
                position double
                orientation double
                pawn_path string = ""
            end
           
            newPose.position.x_val = position(1);
            newPose.position.y_val = -position(2);
            newPose.position.z_val = -position(3);
            orientation = quatinv(orientation);
            
            newPose.orientation.w_val = orientation(1);
            newPose.orientation.x_val = orientation(2);
            newPose.orientation.y_val = orientation(3);
            newPose.orientation.z_val = orientation(4);
            success = obj.rpc_client.call('simAddVehicle', vehicle_name, vehicle_type, newPose, pawn_path);
        end
        
        function vehicles = listVehicles(obj)
            vehicles = string(cell(obj.rpc_client.call("listVehicles")))';
        end
        
        function settings = getSettingsString(obj)
            settings = string(obj.rpc_client.call('getSettingsString'));
        end
        
        function simSetExtForce(obj, ext_force)
            newForce.x_val = ext_force(1);
            newForce.y_val  = -ext_force(2);
            newForce.z_val  = -ext_force(3);
            obj.rpc_client.call('simSetExtForce', newForce);
        end

        function takeoffAsync(obj, varargin)
            if nargin == 2
                t = varargin{1};
            else
                t = 20;
            end
            obj.drone_client.takeoffAsync(t, obj.vehicle_name);
        end

        function landAsync(obj, varargin)
            if nargin == 2
                t = varargin{1};
            else
                t = 60;
            end
            obj.drone_client.landAsync(t, obj.vehicle_name)
        end        

        function goHomeAsync(obj, varargin)
            if nargin == 2
                t = varargin{1};
            else
                t = 3e38;
            end
            obj.drone_client.goHomeAsync(t, obj.vehicle_name)
        end

        function moveByVelocityBodyFrameAsync(obj, vx, vy, vz, duration, drivetrain, yaw_mode_is_rate, yaw_mode_yaw_or_rate)
            yawMode.is_rate = yaw_mode_is_rate;
            yawMode.yaw_or_rate = yaw_mode_yaw_or_rate;
            obj.rpc_client.call("moveByVelocityBodyFrame", vx, vy, vz, duration, int32(drivetrain), yawMode, obj.vehicle_name);
        end

        function moveByVelocityZBodyFrameAsync(obj, vx, vy, z, duration, drivetrain, yaw_mode_is_rate, yaw_mode_yaw_or_rate)
            yawMode.is_rate = yaw_mode_is_rate;
            yawMode.yaw_or_rate = yaw_mode_yaw_or_rate;
            obj.rpc_client.call("moveByVelocityZBodyFrame", vx, vy, z, duration, int32(drivetrain), yawMode, obj.vehicle_name);
        end

        function moveByVelocityAsync(obj, vx, vy, vz, duration, drivetrain, yaw_mode_is_rate, yaw_mode_yaw_or_rate)
            yawMode.is_rate = yaw_mode_is_rate;
            yawMode.yaw_or_rate = yaw_mode_yaw_or_rate;
            obj.rpc_client.call("moveByVelocity", vx, vy, vz, duration, int32(drivetrain), yawMode, obj.vehicle_name);
        end

        function moveByVelocityZAsync(obj,  vx, vy, z, duration, drivetrain, yaw_mode_is_rate, yaw_mode_yaw_or_rate)
            yawMode.is_rate = yaw_mode_is_rate;
            yawMode.yaw_or_rate = yaw_mode_yaw_or_rate;
            obj.rpc_client.call("moveByVelocityZ", vx, vy, z, duration, int32(drivetrain), yawMode, obj.vehicle_name);
        end

        function moveOnPathAsync(obj, path, velocity, timeout_sec, drivetrain, yaw_mode_is_rate, yaw_mode_yaw_or_rate, lookahead, adaptive_lookahead)
            yawMode.is_rate = yaw_mode_is_rate;
            yawMode.yaw_or_rate = yaw_mode_yaw_or_rate;
            obj.rpc_client.call("moveOnPath", path, velocity, timeout_sec, int32(drivetrain), yawMode, lookahead, adaptive_lookahead, obj.vehicle_name);
        end

        function moveToPositionAsync(obj, x, y, z, velocity, timeout_sec, drivetrain, yaw_mode_is_rate, yaw_mode_yaw_or_rate, lookahead, adaptive_lookahead)
            yawMode.is_rate = yaw_mode_is_rate;
            yawMode.yaw_or_rate = yaw_mode_yaw_or_rate;
            obj.rpc_client.call("moveToPosition", x, y, z, velocity, timeout_sec, int32(drivetrain), yawMode, lookahead, adaptive_lookahead, obj.vehicle_name);
        end

        function moveToGPSAsync(obj, latitude, longitude, altitude, velocity, timeout_sec, drivetrain, yaw_mode_is_rate, yaw_mode_yaw_or_rate, lookahead, adaptive_lookahead)
            yawMode.is_rate = yaw_mode_is_rate;
            yawMode.yaw_or_rate = yaw_mode_yaw_or_rate;
            obj.rpc_client.call("moveToGPS", latitude, longitude, altitude, velocity, timeout_sec, int32(drivetrain), yawMode, lookahead, adaptive_lookahead, obj.vehicle_name);
        end

        function moveToZAsync(obj, z, velocity, timeout_sec, drivetrain, yaw_mode_is_rate, yaw_mode_yaw_or_rate, lookahead, adaptive_lookahead)
            yawMode.is_rate = yaw_mode_is_rate;
            yawMode.yaw_or_rate = yaw_mode_yaw_or_rate;
            obj.rpc_client.call("moveToZ", z, velocity, timeout_sec, int32(drivetrain), yawMode, lookahead, adaptive_lookahead, obj.vehicle_name);
        end

        function moveByManualAsync(obj, vx_max, vy_max, z_min, duration, drivetrain, yaw_mode_is_rate, yaw_mode_yaw_or_rate)
            yawMode.is_rate = yaw_mode_is_rate;
            yawMode.yaw_or_rate = yaw_mode_yaw_or_rate;
            obj.rpc_client.call("moveByManual", vx_max, vy_max, z_min, duration, int32(drivetrain), yawMode, obj.vehicle_name);
        end

        function rotateToYawAsync(obj, yaw, timeout_sec, margin)
            obj.rpc_client.call("rotateToYaw", yaw, timeout_sec, margin, obj.vehicle_name);
        end

        function rotateByYawRateAsync(obj, yaw_rate, duration)
            obj.rpc_client.call("rotateByYawRate", yaw_rate, duration, obj.vehicle_name);
        end

        function hoverAsync(obj)
            obj.rpc_client.call("hover", obj.vehicle_name);
        end

        function moveByMotorPWMsAsync(obj, front_right_pwm, rear_left_pwm, front_left_pwm, rear_right_pwm, duration)
            obj.rpc_client.call("moveByMotorPWMs", front_right_pwm, rear_left_pwm, front_left_pwm, rear_right_pwm, duration, obj.vehicle_name);
        end

        function moveByRollPitchYawZAsync(obj, roll, pitch, yaw, z, duration)
            obj.rpc_client.call("moveByRollPitchYawZ", roll, -pitch, -yaw, z, duration, obj.vehicle_name);
        end

        function moveByRollPitchYawThrottleAsync(obj, roll, pitch, yaw, throttle, duration)
            obj.rpc_client.call("moveByRollPitchYawThrottle", roll, -pitch, -yaw, throttle, duration, obj.vehicle_name);
        end

        function moveByRollPitchYawrateThrottleAsync(obj, roll, pitch, yaw_rate, throttle, duration)
            obj.rpc_client.call("moveByRollPitchYawrateThrottle", roll, -pitch, -yaw_rate, throttle, duration, obj.vehicle_name);
        end

        function moveByRollPitchYawrateZAsync(obj, roll, pitch, yaw_rate, z, duration)
            obj.rpc_client.call("moveByRollPitchYawrateZ", roll, -pitch, -yaw_rate, z, duration, obj.vehicle_name);
        end

        function moveByAngleRatesZAsync(obj, roll_rate, pitch_rate, yaw_rate, z, duration)
            obj.rpc_client.call("moveByRollPitchYawrateZ", roll_rate, -pitch_rate, -yaw_rate, z, duration, obj.vehicle_name);
        end

        function moveByAngleRatesThrottleAsync(obj, roll_rate, pitch_rate, yaw_rate, throttle, duration)
            obj.rpc_client.call("moveByAngleRatesThrottle", roll_rate, -pitch_rate, -yaw_rate, throttle, duration, obj.vehicle_name);
        end


        function [MultirotorState] = getMultirotorState(obj)
            vehicleStateAirSim = obj.rpc_client.call("getMultirotorState", obj.vehicle_name);

            collisionData = vehicleStateAirSim{"collision"};
            collisionInfo = struct(collisionData);
            collisionInfo.time_stamp = floor(double(double(collisionData{"time_stamp"}))/1e9);
            collisionInfo.object_name = string(collisionData{"object_name"});
            collisionInfo.object_id = double(collisionData{"object_id"});
            collisionInfo.position = struct2array(struct(collisionData{"position"}));
            collisionInfo.normal = struct2array(struct(collisionData{"normal"}));
            collisionInfo.impact_point = struct2array(struct(collisionData{"impact_point"}));
            MultirotorState.collision = collisionInfo;

            kinematicData = vehicleStateAirSim{"kinematics_estimated"};
            kinematicsState.position = obj.nedToRightHandCoordinates(struct2array(struct(kinematicData{"position"})));
            kinematicsState.orientation = quatinv(struct2array(struct(kinematicData{"orientation"})));
            kinematicsState.linear_velocity = obj.nedToRightHandCoordinates(struct2array(struct(kinematicData{"linear_velocity"})));
            kinematicsState.angular_velocity = obj.nedToRightHandCoordinates(struct2array(struct(kinematicData{"angular_velocity"})));
            kinematicsState.linear_acceleration = obj.nedToRightHandCoordinates(struct2array(struct(kinematicData{"linear_acceleration"})));
            kinematicsState.angular_acceleration = obj.nedToRightHandCoordinates(struct2array(struct(kinematicData{"angular_acceleration"})));
            MultirotorState.kinematics_estimated = kinematicsState;

            geopointData = vehicleStateAirSim{"gps_location"};
            geopoint.latitude = double(geopointData{"latitude"});
            geopoint.longitude = double(geopointData{"longitude"});
            geopoint.altitude = double(geopointData{"altitude"});
            MultirotorState.gps_location = geopoint;

            MultirotorState.timestamp = floor(double(double(vehicleStateAirSim{"timestamp"}))/1e9);
            MultirotorState.landed_state = vehicleStateAirSim{"landed_state"};
            MultirotorState.ready = vehicleStateAirSim{"ready"};
            MultirotorState.ready_message = string(vehicleStateAirSim{"ready_message"});
            MultirotorState.can_arm = vehicleStateAirSim{"can_arm"};
        end

        function [CarState] = getCarState(obj)
            vehicleStateAirSim = obj.rpc_client.call("getCarState", obj.vehicle_name);

            collisionData = vehicleStateAirSim{"collision"};
            collisionInfo = struct(collisionData);
            collisionInfo.time_stamp = floor(double(double(collisionData{"time_stamp"}))/1e9);
            collisionInfo.object_name = string(collisionData{"object_name"});
            collisionInfo.object_id = double(collisionData{"object_id"});
            collisionInfo.position = struct2array(struct(collisionData{"position"}));
            collisionInfo.normal = struct2array(struct(collisionData{"normal"}));
            collisionInfo.impact_point = struct2array(struct(collisionData{"impact_point"}));
            CarState.collision = collisionInfo;

            kinematicData = vehicleStateAirSim{"kinematics_estimated"};
            kinematicsState.position = obj.nedToRightHandCoordinates(struct2array(struct(kinematicData{"position"})));
            kinematicsState.orientation = quatinv(struct2array(struct(kinematicData{"orientation"})));
            kinematicsState.linear_velocity = obj.nedToRightHandCoordinates(struct2array(struct(kinematicData{"linear_velocity"})));
            kinematicsState.angular_velocity = obj.nedToRightHandCoordinates(struct2array(struct(kinematicData{"angular_velocity"})));
            kinematicsState.linear_acceleration = obj.nedToRightHandCoordinates(struct2array(struct(kinematicData{"linear_acceleration"})));
            kinematicsState.angular_acceleration = obj.nedToRightHandCoordinates(struct2array(struct(kinematicData{"angular_acceleration"})));
            CarState.kinematics_estimated = kinematicsState;

            CarState.timestamp = floor(double(double(vehicleStateAirSim{"timestamp"}))/1e9);
            CarState.speed = vehicleStateAirSim{"speed"};
            CarState.gear = vehicleStateAirSim{"gear"};
            CarState.rpm = vehicleStateAirSim{"rpm"};
            CarState.maxrpm = vehicleStateAirSim{"maxrpm"};
            CarState.handbrake = vehicleStateAirSim{"handbrake"};
        end

              
        function [] = pointcloudFeedback(obj, echoData, sensorName, pointCloud)
            pointCloud(:, 3) = -pointCloud(:, 3);
            echoData{"point_cloud"} = py.list(num2cell(reshape(pointCloud.', 1, [])));           
            obj.rpc_client.call("setEchoData", sensorName, obj.vehicle_name, echoData);
        end
        
        function reset(obj)
            obj.rpc_client.call("reset");
        end
        
        function resetVehicle(obj)
            obj.rpc_client.call("resetCar", obj.vehicle_name);
        end
        
        function [] = followTrajectory(obj, trajectoryPoses, elapsedTime)
            stepIdx = find(elapsedTime <= trajectoryPoses.timestamps, 1, "first");
    
            currentPose.position = trajectoryPoses.position(stepIdx, :);
            currentPose.orientation = trajectoryPoses.orientation(stepIdx, :);
            obj.setVehiclePose(currentPose);
        end
        
        function setWeather(obj, weatherType, weatherValue)
            if weatherType == AirSimWeather.Enabled
                obj.rpc_client.call("simEnableWeather", weatherValue);
            else
                obj.rpc_client.call("simSetWeatherParameter", uint32(weatherType), weatherValue);
            end
        end       

        function settings = getPresetLensSettings(obj, sensorName)
            returnData = obj.rpc_client.call("simGetPresetLensSettings", sensorName, obj.vehicle_name);
            settings = string(cell(returnData));
        end

        function settings = getLensSettings(obj, sensorName)
            returnData = obj.rpc_client.call("simGetLensSettings", sensorName, obj.vehicle_name);
            settings = string(returnData);
        end

        function settings = setPresetLensSettings(obj, preset_lens_settings, sensorName)
            returnData = obj.rpc_client.call("simSetPresetLensSettings", preset_lens_settings, sensorName, obj.vehicle_name);
            settings = string(returnData);
        end

        function settings = getPresetFilmbackSettings(obj, sensorName)
            returnData = obj.rpc_client.call("simGetPresetFilmbackSettings", sensorName, obj.vehicle_name);
            settings = string(cell(returnData));
        end

        function setPresetFilmbackSettings(obj, preset_filmback_settings, sensorName)
            obj.rpc_client.call("simGetPresetFilmbackSettings", preset_filmback_settings, sensorName, obj.vehicle_name);
        end

        function settings = getFilmbackSettings(obj, sensorName)
            returnData = obj.rpc_client.call("simGetFilmbackSettings", sensorName, obj.vehicle_name);
            settings = string(returnData);
        end

        function setFilmbackSettings(obj, sensor_width, sensor_height, sensorName)
            obj.rpc_client.call("simSetFilmbackSettings", sensor_width, sensor_height, sensorName, obj.vehicle_name);
        end

        function settings = getFocalLength(obj, sensorName)
            returnData = obj.rpc_client.call("simGetFocalLength", sensorName, obj.vehicle_name);
            settings = double(returnData);
        end

        function setFocalLength(obj, focal_length, sensorName)
            obj.rpc_client.call("simSetFocalLength", focal_length, sensorName, obj.vehicle_name);
        end

        function enableManualFocus(obj, enable, sensorName)
            obj.rpc_client.call("simEnableManualFocus", enable, sensorName, obj.vehicle_name);
        end

        function settings = getFocusDistance(obj, sensorName)
            returnData = obj.rpc_client.call("simGetFocusDistance", sensorName, obj.vehicle_name);
            settings = double(returnData);
        end

        function setFocusDistance(obj, focus_distance, sensorName)
            obj.rpc_client.call("simSetFocusDistance", focus_distance, sensorName, obj.vehicle_name);
        end

        function settings = getFocusAperture(obj, sensorName)
            returnData = obj.rpc_client.call("simGetFocusAperture", sensorName, obj.vehicle_name);
            settings = double(returnData);
        end

        function setFocusAperture(obj, focus_aperture, sensorName)
            obj.rpc_client.call("simSetFocusAperture", focus_aperture, sensorName, obj.vehicle_name);
        end

        function enableFocusPlane(obj, enable, sensorName)
            obj.rpc_client.call("simEnableFocusPlane", enable, sensorName, obj.vehicle_name);
        end

        function settings = getCurrentFieldOfView(obj, sensorName)
            returnData = obj.rpc_client.call("simGetCurrentFieldOfView", sensorName, obj.vehicle_name);
            settings = string(returnData);
        end

        function success = testLineOfSightToPoint(obj, point)
            success =  obj.rpc_client.call("simTestLineOfSightToPoint", point);
        end        

        function success = testLineOfSightBetweenPoints(obj, point1, point2)
            success = obj.rpc_client.call("simTestLineOfSightBetweenPoints", point1, point2);
        end

        function geoPoints = getWorldExtents(obj)
            geopointListData = obj.rpc_client.call("simGetWorldExtents");
            geoPointsList = cell(geopointListData);
            geoPoints = struct(geoPointsList{1});
            for cellIndex = 2 : numel(geoPointsList)
                geoPoints(cellIndex) = struct(geoPointsList{cellIndex});
            end
        end    

        function success = runConsoleCommand(obj, command)
            success = obj.rpc_client.call("simRunConsoleCommand", command);
        end

        function vertexData = getMeshPositionVertexBuffers(obj)
            vertexBuffersData = obj.rpc_client.call("simGetMeshPositionVertexBuffers");
            vertexBufferList = cell(vertexBuffersData);
            vertexDataCur = vertexBufferList{1};
            vertexData = struct(vertexDataCur);
            vertexData.position = struct2array(struct(vertexDataCur{"position"}));
            vertexData.orientation = struct2array(struct(vertexDataCur{"orientation"}));
            for cellIndex = 2 : numel(vertexBufferList)
                vertexDataCur = vertexBufferList{cellIndex};
                vertexData(cellIndex) = struct(vertexDataCur);
                vertexData(cellIndex).position = struct2array(struct(vertexDataCur{"position"}));
                vertexData(cellIndex).orientation = struct2array(struct(vertexDataCur{"orientation"}));
            end
        end

        function collisionInfo = getCollisionInfo(obj)
            collisionData = obj.rpc_client.call("simGetCollisionInfo", obj.vehicle_name);
            collisionInfo = struct(collisionData);
            collisionInfo.time_stamp = floor(double(double(collisionData{"time_stamp"}))/1e9);
            collisionInfo.object_name = string(collisionData{"object_name"});
            collisionInfo.object_id = double(collisionData{"object_id"});
            collisionInfo.position = struct2array(struct(collisionData{"position"}));
            collisionInfo.normal = struct2array(struct(collisionData{"normal"}));
            collisionInfo.impact_point = struct2array(struct(collisionData{"impact_point"}));
        end

        function setTraceLine(obj, color_rgba, thickness)
            obj.rpc_client.call("simSetTraceLine", py.list(color_rgba), thickness, obj.vehicle_name);
        end

        function success = setObjectPose(obj, objectName, position, orientation, teleport)
            newPose.position.x_val = position(1);
            newPose.position.y_val = -position(2);
            newPose.position.z_val = -position(3);
            orientation = quatinv(orientation);
            
            newPose.orientation.w_val = orientation(1);
            newPose.orientation.x_val = orientation(2);
            newPose.orientation.y_val = orientation(3);
            newPose.orientation.z_val = orientation(4);

            success = obj.rpc_client.call("simSetObjectPose", objectName, newPose, teleport);  
        end

        function scale = getObjectScale(obj, objectName)
            scale = struct2array(struct(obj.rpc_client.call("simGetObjectScale", objectName)));  
        end

        function setObjectScale(obj, objectName, scale)
            newScale.x_val = scale(1);
            newScale.y_val = scale(2);
            newScale.z_val = scale(3);
            obj.rpc_client.call("simSetObjectScale", objectName, newScale);  
        end

        function [objectList] = listSceneObjects(obj, name_regex)
            arguments
                obj AirSimClient
                name_regex string = ".*"
            end
            objectList = string(cell(obj.rpc_client.call("simListSceneObjects", name_regex)))';
        end

        function success = loadLevel(obj, level_name)
            success = obj.rpc_client.call("simLoadLevel", level_name);
        end

        function [objectList] = listAssets(obj)
            objectList = string(cell(obj.rpc_client.call("simListAssets")))';
        end

        function objectName = spawnObject(obj, object_name, asset_name, position, orientation, scale, physics_enabled, is_blueprint)
            newPose.position.x_val = position(1);
            newPose.position.y_val = -position(2);
            newPose.position.z_val = -position(3);
            orientation = quatinv(orientation);
            
            newPose.orientation.w_val = orientation(1);
            newPose.orientation.x_val = orientation(2);
            newPose.orientation.y_val = orientation(3);
            newPose.orientation.z_val = orientation(4);

            newScale.x_val = scale(1);
            newScale.y_val = scale(2);
            newScale.z_val = scale(3);
            returnData = obj.rpc_client.call("simSpawnObject", object_name, asset_name, newPose, newScale, physics_enabled, is_blueprint);
            objectName = string(returnData);
        end

        function success = destroyObject(obj, object_name)
            success = obj.rpc_client.call("simDestroyObject", object_name);
        end

        

        function [objectList] = getInstanceSegmentationObjectList(obj)
            % GET_INSTANCE_SEGMENTATION_OBJECT_LIST Get full list of all
            % objects in the scene.

            objectList = string(cell(obj.rpc_client.call("simListInstanceSegmentationObjects")))';
        end

        function [objectPose] = getObjectPose(obj, objectName, local)
            % GET_OBJECT_POSE Get 3D pose of a object by name.

            objectData = obj.rpc_client.call("simGetObjectPose", objectName, local);      
            objectPose.position = obj.nedToRightHandCoordinates(struct2array(struct(objectData{"position"})));
            objectPose.orientation = quatinv(struct2array(struct(objectData{"orientation"})));
        end

        function [instanceSegmentationTable] = getInstanceSegmentationLUT(obj)
            % GET_INSTANCE_SEGMENTATION_LUT Get full list of all
            % objects in the scene and their RGB segmentation values for
            % groundtruth labeling.

            name = obj.getInstanceSegmentationObjectList();
            colorMap = obj.getInstanceSegmentationColormap();
            r = colorMap(1:size(name, 1), 1);
            g = colorMap(1:size(name, 1), 2);
            b = colorMap(1:size(name, 1), 3);
            instanceSegmentationTable = table(name, r, g, b);
        end

        function [objectPosesTable] = getAllObjectPoses(obj, local)
            % GET_OBJECT_POSES Get 3D pose of every object in the scene.
            %
            % This can take a while for a large scene!

            name = obj.getInstanceSegmentationObjectList();
            objectPosesList = obj.rpc_client.call("simListInstanceSegmentationPoses", local);    
            objectPosesCells = cell(objectPosesList);
            objectPositions = zeros(numel(objectPosesCells), 3);
            objectOrientations = zeros(numel(objectPosesCells), 4);
            for cellIndex = 1 : numel(objectPosesCells)
                objectPoseStruct = struct(objectPosesCells{cellIndex});
                objectPositions(cellIndex, :) = obj.nedToRightHandCoordinates(struct2array(struct(objectPoseStruct.position)));
                objectOrientations(cellIndex, :)  = quatinv(struct2array(struct(objectPoseStruct.orientation)));
            end
            x = objectPositions(:, 1);
            y = objectPositions(:, 2);
            z = objectPositions(:, 3);
            quat_w = objectOrientations(:, 1);
            quat_x = objectOrientations(:, 2);
            quat_y = objectOrientations(:, 3);
            quat_z = objectOrientations(:, 4);
            objectPosesTable = table(name, x, y, z, quat_w, quat_x, quat_y, quat_z);
        end

        function success = setSegmentationObjectID(obj, mesh_name, object_id, is_name_regex)
            success = obj.rpc_client.call("simSetSegmentationObjectID", mesh_name, object_id, is_name_regex);
        end

        function objectID = getSegmentationObjectID(obj, mesh_name)
            objectID = obj.rpc_client.call("simGetSegmentationObjectID", mesh_name);
        end

        function [objectList] = simListAnnotationObjects(obj, annotation_name)
            objectList = string(cell(obj.rpc_client.call("simListAnnotationObjects", annotation_name)))';
        end

        function [objectPosesTable] = listAnnotationPoses(obj, annotation_name, local)
            name = obj.getInstanceSegmentationObjectList(annotation_name);
            objectPosesList = obj.rpc_client.call("simListAnnotationPoses", annotation_name, local);    
            objectPosesCells = cell(objectPosesList);
            objectPositions = zeros(numel(objectPosesCells), 3);
            objectOrientations = zeros(numel(objectPosesCells), 4);
            for cellIndex = 1 : numel(objectPosesCells)
                objectPoseStruct = struct(objectPosesCells{cellIndex});
                objectPositions(cellIndex, :) = obj.nedToRightHandCoordinates(struct2array(struct(objectPoseStruct.position)));
                objectOrientations(cellIndex, :)  = quatinv(struct2array(struct(objectPoseStruct.orientation)));
            end
            x = objectPositions(:, 1);
            y = objectPositions(:, 2);
            z = objectPositions(:, 3);
            quat_w = objectOrientations(:, 1);
            quat_x = objectOrientations(:, 2);
            quat_y = objectOrientations(:, 3);
            quat_z = objectOrientations(:, 4);
            objectPosesTable = table(name, x, y, z, quat_w, quat_x, quat_y, quat_z);
        end

        function success = setAnnotationObjectID(obj, annotation_name, mesh_name, object_id, is_name_regex)
            success = obj.rpc_client.call("simSetAnnotationObjectID", annotation_name, mesh_name, object_id, is_name_regex);
        end

        function objectID = getAnnotationObjectID(obj, mesh_name)
            objectID = obj.rpc_client.call("simGetAnnotationObjectID", mesh_name);
        end

        function success = setAnnotationObjectColor(obj, annotation_name, mesh_name, r, g, b, is_name_regex)
            success = obj.rpc_client.call("simSetAnnotationObjectColor", annotation_name, mesh_name, r, g, b, is_name_regex);
        end

        function objectColor = getAnnotationObjectColor(obj, mesh_name)
            objectColor = string(obj.rpc_client.call("simGetAnnotationObjectColor", mesh_name));
        end

        function success = setAnnotationObjectValue(obj, annotation_name, mesh_name, greyscale_value, is_name_regex)
            success = obj.rpc_client.call("simSetAnnotationObjectValue", annotation_name, mesh_name, greyscale_value, is_name_regex);
        end

        function greyscaleValue = getAnnotationObjectValue(obj, mesh_name)
            greyscaleValue = obj.rpc_client.call("simGetAnnotationObjectValue", mesh_name);
        end

        function success = setAnnotationObjectTextureByPath(obj, annotation_name, mesh_name, texture_path, is_name_regex)
            success = obj.rpc_client.call("simSetAnnotationObjectTextureByPath", annotation_name, mesh_name, texture_path, is_name_regex);
        end

        function success = enableAnnotationObjectTextureByPath(obj, annotation_name, mesh_name, is_name_regex)
            success = obj.rpc_client.call("simEnableAnnotationObjectTextureByPath", annotation_name, mesh_name, is_name_regex);
        end

        function texturePath = getAnnotationObjectTexturePath(obj, mesh_name)
            texturePath = string(obj.rpc_client.call("simGetAnnotationObjectTexturePath", mesh_name));
        end

        function addDetectionFilterMeshName(obj, camera_name, image_type, mesh_name, annotation_name)
            arguments
                obj AirSimClient
                camera_name string
                image_type uint32
                mesh_name string
                annotation_name string = ".*"
            end
            obj.rpc_client.call("simAddDetectionFilterMeshName", camera_name, image_type, mesh_name, obj.vehicle_name, annotation_name);
        end

        function setDetectionFilterRadius(obj, camera_name, image_type, radius_cm, annotation_name)
            arguments
                obj AirSimClient
                camera_name string
                image_type uint32
                radius_cm int32
                annotation_name string = ".*"
            end
            obj.rpc_client.call("simSetDetectionFilterRadius", camera_name, image_type, radius_cm, obj.vehicle_name, annotation_name);
        end

        function clearDetectionMeshNames(obj, camera_name, image_type, annotation_name)
            arguments
                obj AirSimClient
                camera_name string
                image_type uint32
                annotation_name string = ".*"
            end
            obj.rpc_client.call("simClearDetectionMeshNames", camera_name, image_type, obj.vehicle_name, annotation_name);
        end

        function detectionInfo = simGetDetections(obj)
            detectionData = obj.rpc_client.call("simGetDetections");
            detectionDataList = cell(detectionData);
            detectionDataCurrent = detectionDataList{1};
            detectionInfo = struct(detectionDataCurrent);
            detectionInfo.name = string(detectionInfo.name);
            curePoseData = detectionInfo.pose;
            curPose.position = struct2array(struct(curePoseData{"position"}));
            curPose.orientation = struct2array(struct(curePoseData{"orientation"}));
            detectionInfo.pose = curPose;
            curGeoPointData = detectionInfo.geo_point;
            geopoint.latitude = double(curGeoPointData{"latitude"});
            geopoint.longitude = double(curGeoPointData{"longitude"});
            geopoint.altitude = double(curGeoPointData{"altitude"});
            detectionInfo.geo_point = geopoint;
            curBox2dData = detectionInfo.box2D;
            curBox2D.min = struct2array(struct(curBox2dData{"min"}));
            curBox2D.max = struct2array(struct(curBox2dData{"max"}));
            detectionInfo.box2D = curBox2D;
            curBox3dData = detectionInfo.box3D;
            curBox3D.min = struct2array(struct(curBox3dData{"min"}));
            curBox3D.max = struct2array(struct(curBox3dData{"max"}));
            detectionInfo.box3D = curBox3D;

            for cellIndex = 2 : numel(detectionDataList)
                detectionDataCurrent = detectionDataList{cellIndex};
                detectionInfo(cellIndex) = struct(detectionDataCurrent);
                detectionInfo(cellIndex).name = string(detectionInfo(cellIndex).name);
                curePoseData = detectionInfo(cellIndex).pose;
                curPose.position = struct2array(struct(curePoseData{"position"}));
                curPose.orientation = struct2array(struct(curePoseData{"orientation"}));
                detectionInfo(cellIndex).pose = curPose;
                curGeoPointData = detectionInfo(cellIndex).geo_point;
                geopoint.latitude = double(curGeoPointData{"latitude"});
                geopoint.longitude = double(curGeoPointData{"longitude"});
                geopoint.altitude = double(curGeoPointData{"altitude"});
                detectionInfo(cellIndex).geo_point = geopoint;
                curBox2dData = detectionInfo(cellIndex).box2D;
                curBox2D.min = struct2array(struct(curBox2dData{"min"}));
                curBox2D.max = struct2array(struct(curBox2dData{"max"}));
                detectionInfo(cellIndex).box2D = curBox2D;
                curBox3dData = detectionInfo(cellIndex).box3D;
                curBox3D.min = struct2array(struct(curBox3dData{"min"}));
                curBox3D.max = struct2array(struct(curBox3dData{"max"}));
                detectionInfo(cellIndex).box3D = curBox3D;
            end
        end

        function logMessage = printLogMessage(obj, message, message_param, severity)
            logMessage = string(obj.rpc_client.call("simPrintLogMessage", message, message_param, severity));
        end

        function setDistortionParam(obj, camera_name, param_name, value)
            obj.rpc_client.call("simSetDistortionParam", camera_name, param_name, value, obj.vehicle_name);
        end

        function setCameraPose(obj, camera_name, position, orientation)
            newPose.position.x_val = position(1);
            newPose.position.y_val = -position(2);
            newPose.position.z_val = -position(3);
            orientation = quatinv(orientation);
            
            newPose.orientation.w_val = orientation(1);
            newPose.orientation.x_val = orientation(2);
            newPose.orientation.y_val = orientation(3);
            newPose.orientation.z_val = orientation(4);
            obj.rpc_client.call("simSetCameraPose", camera_name, newPose, obj.vehicle_name);
        end

        function simSetCameraFov(obj, camera_name, fov_degrees)
            obj.rpc_client.call("setCameraFov", camera_name, fov_degrees, obj.vehicle_name);
        end

    end
end



