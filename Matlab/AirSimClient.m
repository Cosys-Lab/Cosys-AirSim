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
        
        function [vehicleState] = getVehicleState(obj)
            vehicleStateAirSim = obj.rpc_client.call("simGetGroundTruthKinematics", obj.vehicle_name);
            vehicleState.pose.position = obj.nedToRightHandCoordinates(struct2array(struct(vehicleStateAirSim{"position"})));
            vehicleState.pose.orientation = quatinv(struct2array(struct(vehicleStateAirSim{"orientation"})));
            vehicleState.velocity.linear = simGetGroundTruthKinematics(struct2array(struct(vehicleStateAirSim{"linear_velocity"})));
            vehicleState.velocity.angular = simGetGroundTruthKinematics(struct2array(struct(vehicleStateAirSim{"angular_velocity"})));
            vehicleState.acceleration.linear = simGetGroundTruthKinematics(struct2array(struct(vehicleStateAirSim{"linear_acceleration"})));
            vehicleState.acceleration.angular = simGetGroundTruthKinematics(struct2array(struct(vehicleStateAirSim{"angular_acceleration"})));
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
                py.bool
                obj.rpc_client.call("simEnableWeather", weatherValue);
            else
                obj.rpc_client.call("simSetWeatherParameter", uint32(weatherType), weatherValue);
            end
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

        function [image, timestamp] = getCameraImage(obj, sensorName, cameraType)
            % GET_CAMERA_IMAGE Get camera data from a camera sensor

            if cameraType == 1 || cameraType == 2 || cameraType == 3
                image_request = py.airsim.ImageRequest(sensorName, int32(cameraType), true, false);
            else
                image_request = py.airsim.ImageRequest(sensorName, int32(cameraType), false, false);
            end
            image_request_list = py.list({image_request});
            camera_image_response_request = obj.rpc_client.call("simGetImages", image_request_list, obj.vehicle_name);
            image_response = py.airsim.ImageResponse();
            camera_image = image_response.from_msgpack(camera_image_response_request{1});
            if cameraType == 1 || cameraType == 2 || cameraType == 3
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

        function [images, timestamp] = getCameraImages(obj, sensorName, cameraTypes)
            % GET_CAMERA_IMAGES Get syncedcamera data from a camera sensor
            images = {};
            image_requests = [];
            for i = 1: numel(cameraTypes)
                if cameraTypes(i) == 1 || cameraTypes(i) == 2
                    image_requests{i} = py.airsim.ImageRequest(sensorName, int32(cameraTypes(i)), true, false);
                else
                    image_requests{i} = py.airsim.ImageRequest(sensorName, int32(cameraTypes(i)), false, false);
                end
            end
            image_request_list = py.list(image_requests);
            camera_image_response_request = obj.rpc_client.call("simGetImages", image_request_list, obj.vehicle_name);

            for i = 1: numel(cameraTypes)
                image_response = py.airsim.ImageResponse();
                camera_image = image_response.from_msgpack(camera_image_response_request{i});
                if cameraTypes(i) == 1 || cameraTypes(i) == 2
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
    end
end



