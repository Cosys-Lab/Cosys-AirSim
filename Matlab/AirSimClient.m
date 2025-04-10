classdef AirSimClient < handle
    %AIRSIMCLIENT a Matlab client for Cosys-AirSim API. 
    % Most functions are available but some were not possible to implement.
    
    properties
        rpc_client;
        ip;
        port;
        is_drone;
        vehicle_name;   
        car_controls;
        drone_client;
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
            carClient = py.cosysairsim.CarClient();
            carClient.confirmConnection();
            carControls = py.cosysairsim.CarControls();
        end

        function droneClient = getDroneControls()
            droneClient = py.cosysairsim.MultirotorClient();
            droneClient.confirmConnection();
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
            % AirSimClient Constructor for the AirSimClient class.
            %
            % Syntax:
            %   obj = AirSimClient(Name, Value)
            %
            % Description:
            %   Constructs an instance of the AirSimClient class.
            %
            % Inputs (Name-Value pairs):
            %   'IsDrone' - Logical flag indicating whether the vehicle is a drone (default: false).
            %   'IP' - String specifying the IP address of the AirSim server (default: "127.0.0.1").
            %   'Port' - Numeric value specifying the port of the AirSim server (default: 41451).
            %
            % Outputs:
            %   Instance of the AirSimClient class.

            argParser = inputParser();
            argParser.addOptional("IsDrone", false, @islogical);
            argParser.addOptional("IP", "127.0.0.1", @isstring);
            argParser.addOptional("Port", 41451, @isnumeric);
            argParser.parse(varargin{:});

            obj.is_drone = argParser.Results.IsDrone;
            obj.ip = argParser.Results.IP;
            obj.port = argParser.Results.Port;

            obj.rpc_client = AirSimClient.setupRPC(obj.ip, obj.port);
            
            if obj.is_drone
                obj.drone_client = AirSimClient.getDroneControls();
            else
                obj.car_controls = AirSimClient.getCarControls();
            end
        end     

           function [imuData, timestamp] = getIMUData(obj, sensorName, vehicleName)
            % GETIMUDATA Get IMU sensor data
            %
            % Description:
            %   Retrieves IMU sensor data from the specified sensor.
            %
            % Inputs:
            %   sensorName - Name of the IMU sensor.
            %   vehicleName - name of the vehicle.
            %
            % Outputs:
            %   imuData - Struct containing orientation(1x4), angular velocity(1x3), and linear acceleration(1x3).
            %   timestamp - Timestamp of the sensor data.

            data = obj.rpc_client.call("getImuData", sensorName, vehicleName);

            timestamp = double(double(data{"time_stamp"}))/1e9;

            imuData.orientation = quatinv(struct2array(struct(data{"orientation"})));
            imuData.angularVelocity = obj.nedToRightHandCoordinates(struct2array(struct(data{"angular_velocity"})));
            imuData.linearAcceleration = obj.nedToRightHandCoordinates(struct2array(struct(data{"linear_acceleration"})));
        end

        function [barometerData, timestamp] = getBarometerData(obj, sensorName, vehicleName)
            % GETBAROMETERDATA Get barometer sensor data
            %
            % Description:
            %   Retrieves barometer sensor data from the specified sensor.
            %
            % Inputs:
            %   sensorName - Name of the barometer sensor.
            %   vehicleName - name of the vehicle.
            %
            % Outputs:
            %   barometerData - Struct containing altitude, pressure, and qnh.
            %   timestamp - Timestamp of the sensor data.

            data = obj.rpc_client.call("getBarometerData", sensorName, vehicleName);

            timestamp = double(double(data{"time_stamp"}))/1e9;
            barometerData.altitude = quatinv(struct2array(struct(data{"altitude"})));
            barometerData.pressure = obj.nedToRightHandCoordinates(struct2array(struct(data{"pressure"})));
            barometerData.qnh = obj.nedToRightHandCoordinates(struct2array(struct(data{"qnh"})));
        end

        function [MagnetometerData, timestamp] = getMagnetometerData(obj, sensorName, vehicleName)
            % GETMAGNETOMETERDATA Get magnetometer sensor data
            %
            % Description:
            %   Retrieves magnetometer sensor data from the specified sensor.
            %
            % Inputs:
            %   sensorName - Name of the magnetometer sensor.
            %   vehicleName - name of the vehicle.
            %
            % Outputs:
            %   MagnetometerData - Struct containing magnetic field data.
            %   timestamp - Timestamp of the sensor data.

            data = obj.rpc_client.call("getMagnetometerData", sensorName, vehicleName);

            timestamp = double(double(data{"time_stamp"}))/1e9;
            MagnetometerData.magnetic_field_body = obj.nedToRightHandCoordinates(struct2array(struct(data{"magnetic_field_body"})));
            MagnetometerData.magnetic_field_covariance = double(data{"magnetic_field_covariance"});
        end

        function [gnssData, timestamp, isValid] = getGpsData(obj, sensorName, vehicleName)
            % GETGPSDATA Get GPS sensor data
            %
            % Description:
            %   Retrieves GPS sensor data from the specified sensor.
            %
            % Inputs:
            %   sensorName - Name of the GPS sensor.
            %   vehicleName - name of the vehicle.
            %
            % Outputs:
            %   gnssData - Struct containing GPS data (geo point, eph, epv, velocity, fix_type, time_utc)
            %   timestamp - Timestamp of the sensor data.
            %   isValid - boolean if the data is valid.            

            data = obj.rpc_client.call("getGpsData", sensorName, vehicleName);

            timestamp = double(double(data{"time_stamp"}))/1e9;

            gnssDataRaw = data{"gnss"};
            gnssData.eph = double(gnssDataRaw{"eph"});
            gnssData.epv = double(gnssDataRaw{"epv"});
            gnssData.velocity = struct2array(struct(gnssDataRaw{"velocity"}));

            curGeoPointData = gnssDataRaw{"geo_point"};
            geopoint.latitude = double(curGeoPointData{"latitude"});
            geopoint.longitude = double(curGeoPointData{"longitude"});
            geopoint.altitude = double(curGeoPointData{"altitude"});
            gnssData.geo_point = geopoint;            
            gnssData.fix_type = double(gnssDataRaw{"fix_type"});
            gnssData.time_utc = double(double(curGeoPointData{"time_utc"}))/1e9;   
            
            isValid = data{"is_valid"};
        end

        function [DistanceSensorData, timestamp] = getDistanceSensorData(obj, sensorName, vehicleName)
            % GETDISTANCESENSORDATA Get distance sensor data
            %
            % Description:
            %   Retrieves distance sensor data from the specified sensor.
            %
            % Inputs:
            %   sensorName - Name of the distance sensor.
            %   vehicleName - name of the vehicle.
            %
            % Outputs:
            %   DistanceSensorData - Struct containing distance data (relative pose, distance, min/max distance)
            %   timestamp - Timestamp of the sensor data.

            data = obj.rpc_client.call("getDistanceSensorData", sensorName, vehicleName);

            timestamp = double(double(data{"time_stamp"}))/1e9;
            curPoseData = data{"relative_pose"};
            DistanceSensorData.relative_pose.position = obj.nedToRightHandCoordinates(struct2array(struct(curPoseData{"position"})));
            DistanceSensorData.relative_pose.orientation = quatinv(struct2array(struct(curPoseData{"orientation"})));
            DistanceSensorData.distance = double(data{"distance"});
            DistanceSensorData.max_distance = double(data{"max_distance"});
            DistanceSensorData.min_distance = double(data{"min_distance"});
        end

        function [activePointCloud, activeData, passivePointCloud, passiveData, timestamp, sensorPose] = getEchoData(obj, sensorName, enablePassive, vehicleName)
            % GETECHODATA Get sensor data from an echo sensor
            %
            % Description:
            %   Retrieves data from an echo sensor, including point cloud data.
            %
            % Inputs:
            %   sensorName - Name of the echo sensor.
            %   enablePassive - Logical value to enable passive data retrieval.
            %   vehicleName - name of the vehicle.
            %
            % Outputs:
            %   activePointCloud - Active point cloud data.
            %   activeData - Active data struct (groundtruth labels, attenuation, distance, and reflection count for each point)
            %   passivePointCloud - Passive point cloud data. The reflection angle vector is contained within the Normal data field of the pointcloud
            %   passiveData - Passive data struct (groundtruth labels (original source), reflection labels(reflective surface), attenuation, distance, and reflection count for each point)
            %   timestamp - Timestamp of the sensor data.
            %   sensorPose - Sensor pose struct.
            
            echoData = obj.rpc_client.call("getEchoData", sensorName, vehicleName);

            timestamp = double(double(echoData{"time_stamp"}))/1e9;

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

        function [] = pointcloudFeedback(obj, echoData, sensorName, pointCloud, vehicleName)
            % POINTCLOUDFEEDBACK Send point cloud data as feedback to the
            % simulation for echo sensors.
            %
            % Description:
            %   Sends point cloud data received as feedback to the
            %   simulation via the AirSim API for echo sensors.
            %   vehicleName - name of the vehicle.
            %
            % Inputs:
            %   echoData - Echo data structure to update with point cloud.
            %   sensorName - Name of the echo sensor providing the point cloud.
            %   pointCloud - Point cloud data to send (N x 3 matrix of [x, y, z] coordinates).


            pointCloud(:, 3) = -pointCloud(:, 3);
            echoData{"point_cloud"} = py.list(num2cell(reshape(pointCloud.', 1, [])));           
            obj.rpc_client.call("setEchoData", sensorName, vehicleName, echoData);
        end        

        function [lidarPointCloud, lidarLabels, timestamp, sensorPose] = getLidarData(obj, sensorName, enableLabels, vehicleName)
            % GETLIDARDATA Get sensor data from a lidar sensor
            %
            % Description:
            %   Retrieves lidar sensor data from the specified sensor.
            %
            % Inputs:
            %   sensorName - Name of the lidar sensor.
            %   enableLabels - Logical value to enable label retrieval.
            %   vehicleName - name of the vehicle.
            %
            % Outputs:
            %   lidarPointCloud - Lidar point cloud data.
            %   lidarLabels - Lidar labels.
            %   timestamp - Timestamp of the sensor data.
            %   sensorPose - Sensor pose struct.            
            
            lidarData = obj.rpc_client.call("getLidarData", sensorName, vehicleName);

            timestamp = double(double(lidarData{"time_stamp"}))/1e9;

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

        function [lidarPointCloud, timestamp, sensorPose] = getGPULidarData(obj, sensorName, vehicleName)
            % GETGPULIDARDATA Get sensor data from a GPU lidar sensor
            %
            % Description:
            %   Retrieves GPU lidar sensor data from the specified sensor.
            %
            % Inputs:
            %   sensorName - Name of the GPU lidar sensor.
            %   vehicleName - name of the vehicle.            
            %
            % Outputs:
            %   lidarPointCloud - Lidar point cloud data.
            %   timestamp - Timestamp of the sensor data.
            %   sensorPose - Sensor pose struct.
            
            lidarData = obj.rpc_client.call("getGPULidarData", sensorName, vehicleName);

            timestamp = double(double(lidarData{"time_stamp"}))/1e9;

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

        function [image, timestamp] = getCameraImage(obj, sensorName, cameraType, vehicleName, annotationLayer)
            % GETCAMERAIMAGE Get camera data from a camera sensor
            %
            % Description:
            %   Retrieves camera sensor data from the specified sensor.
            %
            % Inputs:
            %   sensorName - Name of the camera sensor.
            %   cameraType - Type of the camera (use AirSimCameraTypes enum).
            %   vehicleName - name of the vehicle.
            %   annotationLayer - Optional annotation layer name if using annotation system (default '').
            %
            % Outputs:
            %   image - Captured image data.
            %   timestamp - Timestamp of when the image was captured.

            arguments
                obj AirSimClient
                sensorName string 
                cameraType uint32
                vehicleName string
                annotationLayer string = ""                
            end

            if cameraType == 1 || cameraType == 2 || cameraType == 3 || cameraType == 4
                image_request = py.cosysairsim.ImageRequest(sensorName, int32(cameraType), true, false);
            elseif cameraType == 10
                image_request = py.cosysairsim.ImageRequest(sensorName, int32(cameraType), false, false, annotationLayer);
            else
                image_request = py.cosysairsim.ImageRequest(sensorName, int32(cameraType), false, false);
            end
            image_request_list = py.list({image_request});
            camera_image_response_request = obj.rpc_client.call("simGetImages", image_request_list, vehicleName);
            image_response = py.cosysairsim.ImageResponse();
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
            timestamp = double(double(camera_image.time_stamp))/1e9;
        end

        function [images, timestamp] = getCameraImages(obj, sensorName, cameraTypes, vehicleName, annotationLayers)
            % GETCAMERAIMAGES Get synchronized camera data from multiple camera sensors
            %
            % Description:
            %   Retrieves synchronized camera data from multiple camera sensors.
            %
            % Inputs:
            %   sensorName - Name of the camera sensor.
            %   cameraTypes - Array of camera types (use AirSimCameraTypes enum).
            %   vehicleName - name of the vehicle.
            %   annotationLayers - Array of optional annotation layer name if using annotation system (default '').
            %
            % Outputs:
            %   images - Cell array of captured images from each camera type.
            %   timestamp - Timestamp of when the images were captured.

            arguments
                obj AirSimClient
                sensorName string 
                cameraTypes uint32
                vehicleName
                annotationLayers string = ""
            end
            images = {};
            image_requests = [];
            for i = 1: numel(cameraTypes)
                if cameraTypes(i) == 1 || cameraTypes(i) == 2 || cameraTypes(i) == 3 || cameraTypes(i) == 4
                    image_requests{i} = py.cosysairsim.ImageRequest(sensorName, int32(cameraTypes(i)), true, false);
                elseif cameraTypes(i) == 10
                    image_requests{i} = py.cosysairsim.ImageRequest(sensorName, int32(cameraTypes(i)), false, false, annotationLayers(i));
                else
                    image_requests{i} = py.cosysairsim.ImageRequest(sensorName, int32(cameraTypes(i)), false, false);
                end
            end
            image_request_list = py.list(image_requests);
            camera_image_response_request = obj.rpc_client.call("simGetImages", image_request_list, vehicleName);

            for i = 1: numel(cameraTypes)
                image_response = py.cosysairsim.ImageResponse();
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
            timestamp = double(double(camera_image.time_stamp))/1e9;
        end

        function [intrinsics, sensorPose] = getCameraInfo(obj, sensorName, vehicleName)
            % GETCAMERAINFO Get camera pose and intrinsics
            %
            % Description:
            %   Retrieves camera pose and intrinsics from the specified sensor.
            %   Do note that distortion is not implemented yet!
            %
            % Inputs:
            %   sensorName - Name of the camera sensor.
            %   vehicleName - name of the vehicle.
            %
            % Outputs:
            %   intrinsics - Camera intrinsics (focal length, principal point, image size).
            %   sensorPose - Sensor pose struct (position and orientation).

            cameraData = obj.rpc_client.call("simGetCameraInfo", sensorName, vehicleName);
            sensorPose.position = obj.nedToRightHandCoordinates(struct2array(struct(cameraData{"pose"}{"position"})));
            sensorPose.orientation = quatinv(struct2array(struct(cameraData{"pose"}{"orientation"})));
            curFov = cameraData{"fov"};
            tempRequest = py.cosysairsim.ImageRequest(sensorName, int32(1), true, false);
            tempRequestList = py.list({tempRequest});
            tempCameraResponseRequest = obj.rpc_client.call("simGetImages", tempRequestList, vehicleName);
            testImageResponse = py.cosysairsim.ImageResponse();
            tempCameraImage = testImageResponse.from_msgpack(tempCameraResponseRequest{1});
            cameraWidth = tempCameraImage.width.int32;
            cameraHeight = tempCameraImage.height.int32;
            focalLength = (double(cameraWidth) / 2.0) / tan(curFov * pi / 360);
            intrinsics = cameraIntrinsics([focalLength, focalLength], [double(cameraWidth / 2), double(cameraHeight / 2)], [double(cameraWidth), double(cameraHeight)]);
        end

        function [wifiState] = getWifiState(obj, sensorName, vehicleName)
            % GETWIFISTATE Get Wi-Fi state information from a sensor
            %
            % Description:
            %   Retrieves Wi-Fi state information from the specified sensor.
            %
            % Inputs:
            %   sensorName - Name of the Wi-Fi sensor.
            %   vehicleName - name of the vehicle.
            %
            % Outputs:
            %   wifiState - Struct containing Wi-Fi state information.

            wifiState = struct();
            wifiStateAirsim = obj.rpc_client.call("getWifiData", sensorName, vehicleName);
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

        function [uwbState] = getUWBState(obj, vehicleName)
            % GETUWBSTATE Get UWB (Ultra-Wideband) state information
            %
            % Description:
            %   Retrieves UWB (Ultra-Wideband) state information from the sensor.
            %
            % Inputs:
            %   vehicleName - name of the vehicle.            
            %
            % Outputs:
            %   uwbState - Struct containing UWB state information.

            uwbState = struct();
            %uwbStateAirsim = obj.rpc_client.call("getUWBData", sensorName, vehicleName);
            uwbStateAirsim = obj.rpc_client.call("getUWBData", "", vehicleName);
            
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

        function [uwbState] = getUWBSensorState(obj, sensorName, vehicleName)
            % GETUWBSENSORSTATE Get UWB sensor state information
            %
            % Description:
            %   Retrieves UWB sensor state information from the specified sensor.
            %
            % Inputs:
            %   sensorName - Name of the UWB sensor.
            %   vehicleName - name of the vehicle.         
            %
            % Outputs:
            %   uwbState - Struct containing UWB sensor state information.

            uwbState = struct();
            %uwbStateAirsim = struct(obj.rpc_client.call("getWifiData", "wifi", vehicleNames(idx)));
            %uwbStateAirsim = struct(obj.rpc_client.call("getUWBSensorData", "UnrealMarLocUwbSensor", vehicleNames(idx)));
            uwbStateAirsim = obj.rpc_client.call("getUWBSensorData", sensorName, vehicleName);
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

        function [wifiState] = getWifiSensorState(obj, sensorName, vehicleName)
            % GETWIFISENSORSTATE Get Wi-Fi sensor state information
            %
            % Description:
            %   Retrieves Wi-Fi sensor state information from the specified sensor.
            %
            % Inputs:
            %   sensorName - Name of the Wi-Fi sensor.
            %   vehicleName - name of the vehicle.         
            %
            % Outputs:
            %   wifiState - Struct containing Wi-Fi sensor state information.

            wifiState = struct();
            %wifiStateAirsim = struct(obj.rpc_client.call("getWifiData", "wifi", vehicleNames(idx)));
            %wifiStateAirsim = struct(obj.rpc_client.call("getwifiSensorData", "UnrealMarLocwifiSensor", vehicleNames(idx)));
            wifiStateAirsim = obj.rpc_client.call("getWifiSensorData", sensorName, vehicleName);
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
        
        function [vehiclePose] = getVehiclePose(obj, vehicleName)
            % GETVEHICLEPOSE Retrieve the current pose (position and orientation) of the vehicle.
            %
            % Description:
            %   Retrieves the current pose (position and orientation) of the vehicle.
            %
            % Inputs:
            %   vehicleName - name of the vehicle.                  
            %
            % Outputs:
            %   vehiclePose - A structure containing:
            %   position(1x3) - The position of the vehicle in right-hand coordinates.
            %   orientation(1x4) - The quaternion representing the orientation of the vehicle in right-hand coordinates.

            vehicleState = obj.rpc_client.call("simGetVehiclePose", vehicleName);            
            vehiclePose.position = obj.nedToRightHandCoordinates(struct2array(struct(vehicleState{"position"})));
            vehiclePose.orientation = quatinv(struct2array(struct(vehicleState{"orientation"})));
        end

        function [] = setVehiclePose(obj, position, orientation, ignoreCollisions, vehicleName)
            % SETVEHICLEPOSE Set the pose of the vehicle.
            %
            % Description:
            %   Sets the pose (position and orientation) of the vehicle.
            %
            % Inputs:
            %   position(1x3) - Desired position of the vehicle.
            %   orientation(1x4) - Desired quaternion orientation of the vehicle.
            %   ignoreCollisions - Boolean flag to ignore collisions (default: true).
            %   vehicleName - name of the vehicle.      
            
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
            
            obj.rpc_client.call("simSetVehiclePose", newPose, ignoreCollisions, vehicleName);
        end
        
        function [] = setVehicleControls(obj, throttle, steering, vehicleName)
            % SETVEHICLECONTROLS Set the controls of the vehicle.
            %
            % Description:
            %   Sets the controls (throttle and steering) of the vehicle.
            %
            % Inputs:
            %   throttle - Throttle value.
            %   steering - Steering value.
            %   vehicleName - name of the vehicle.      

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
            
            obj.rpc_client.call("setCarControls", obj.car_controls, vehicleName);
        end

        % Enable disable or prompt api control status
        % Necessary to send speed control from matlab to the api
        function setEnableApiControl(obj, vehicleName)
            % SETENABLEAPICONTROL Enable API control.
            %
            % Description:
            %   Enables API control for the vehicle.
            %
            % Inputs:
            %   vehicleName - name of the vehicle.                  

           obj.rpc_client.call("enableApiControl", true, vehicleName);
        end

        function setDisableApiControl(obj, vehicleName)
            % SETDISABLEAPICONTROL Disable API control.
            %
            % Description:
            %   Disables API control for the vehicle.
            %
            % Inputs:
            %   vehicleName - name of the vehicle.   

            obj.rpc_client.call("enableApiControl", false, vehicleName);
        end   

        function isEnabled = getApiControlEnabled(obj, vehicleName)
            % GETAPICONTROLENABLED Check if API control is enabled.
            %
            % Description:
            %   Returns a logical value indicating whether API control is enabled for the vehicle.
            %
            % Inputs:
            %   vehicleName - name of the vehicle.   
            %
            % Outputs:
            %   isEnabled - Logical value indicating the API control status.
            isEnabled = obj.rpc_client.call("isApiControlEnabled", vehicleName);
        end
        
        function setEnableDroneArm(obj, vehicleName)
            % SETENABLEDRONEARM Enable drone arm.
            %
            % Description:
            %   Enables the arm of the drone.
            %
            % Inputs:
            %   vehicleName - name of the vehicle.   

           obj.rpc_client.call("armDisarm", true, vehicleName);
        end

        function setDisableDroneArm(obj, vehicleName)
            % SETDISABLEDRONEARM Disable drone arm.
            %
            % Description:
            %   Disables the arm of the drone.
            %
            % Inputs:
            %   vehicleName - name of the vehicle.   
            
           obj.rpc_client.call("armDisarm", false, vehicleName);
        end

        function pause(obj)
            % PAUSE Pause the simulation.
            %
            % Description:
            %   Pauses the simulation.

           obj.rpc_client.call("simPause", true);
        end

        function unpause(obj)
            % UNPAUSE Unpause the simulation.
            % 
            % Description:
            %   Unpauses the simulation.

            obj.rpc_client.call("simPause", false);
        end

        function IsPaused = isPaused(obj)
            % ISPAUSED Check if the simulation is paused.
            %
            % Description:
            %   Returns a logical value indicating whether the simulation is paused.
            %
            % Outputs:
            %   IsPaused - Logical value indicating the pause status of the simulation.

            IsPaused = obj.rpc_client.call("simIsPaused");
        end

        function continueForFrames(obj, frames)
            % CONTINUEFORFRAMES Continue simulation for a specific number of frames.
            %
            % Description:
            %   Continues the simulation for the specified number of frames.
            %
            % Inputs:
            %   frames - Numeric value specifying the number of frames to continue the simulation.

            obj.rpc_client.call("simContinueForFrames", frames);
        end
        
        function [kinematicsState] = getGroundTruthKinematics(obj, vehicleName)
            % GETGROUNDTRUTHKINEMATICS Retrieve the ground truth kinematics of the vehicle.            
            %
            % Description:
            %   Retrieves the ground truth kinematics of the vehicle, including position, orientation, linear and angular velocities, and accelerations.
            %
            % Inputs:
            %   vehicleName - name of the vehicle.   
            %
            % Outputs:
            %   kinematicsState - A structure containing:
            %     position(1x3) - The position of the vehicle in right-hand coordinates.
            %     orientation(1x4) - The quaternion representing the orientation of the vehicle in right-hand coordinates.
            %     linear_velocity(1x3) - The linear velocity of the vehicle in right-hand coordinates.
            %     angular_velocity(1x3) - The angular velocity of the vehicle in right-hand coordinates.
            %     linear_acceleration(1x3) - The linear acceleration of the vehicle in right-hand coordinates.
            %     angular_acceleration(1x3) - The angular acceleration of the vehicle in right-hand coordinates.

            vehicleStateAirSim = obj.rpc_client.call("simGetGroundTruthKinematics", vehicleName);
            kinematicsState.position = obj.nedToRightHandCoordinates(struct2array(struct(vehicleStateAirSim{"position"})));
            kinematicsState.orientation = quatinv(struct2array(struct(vehicleStateAirSim{"orientation"})));
            kinematicsState.linear_velocity = obj.nedToRightHandCoordinates(struct2array(struct(vehicleStateAirSim{"linear_velocity"})));
            kinematicsState.angular_velocity = obj.nedToRightHandCoordinates(struct2array(struct(vehicleStateAirSim{"angular_velocity"})));
            kinematicsState.linear_acceleration = obj.nedToRightHandCoordinates(struct2array(struct(vehicleStateAirSim{"linear_acceleration"})));
            kinematicsState.angular_acceleration = obj.nedToRightHandCoordinates(struct2array(struct(vehicleStateAirSim{"angular_acceleration"})));
        end

        function setKinematics(obj, position, orientation, linear_velocity, angular_velocity, linear_acceleration, angular_acceleration, ignore_collision, vehicleName)
            % SETKINEMATICS Set the kinematic state of the vehicle.
            %
            % Description:
            %   Sets the kinematic state of the vehicle including position, orientation, linear and angular velocities, and accelerations.
            %
            % Inputs:
            %   position(1x3) - The position of the vehicle in right-hand coordinates.
            %   orientation(1x4) - The quaternion representing the orientation of the vehicle in right-hand coordinates.
            %   linear_velocity(1x3) - The linear velocity of the vehicle in right-hand coordinates.
            %   angular_velocity(1x3) - The angular velocity of the vehicle in right-hand coordinates.
            %   linear_acceleration(1x3) - The linear acceleration of the vehicle in right-hand coordinates.
            %   angular_acceleration(1x3) - The angular acceleration of the vehicle in right-hand coordinates.
            %   ignore_collision: Whether to ignore any collision or not.
            %   vehicleName - name of the vehicle.

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
            obj.rpc_client.call("setKinematics", kinematicsState, ignore_collision, vehicleName);
        end

        function [kinematicsState] = getGroundTruthKinematics(obj, vehicleName)
            % GETGROUNDTRUTHKINEMATICS Get Physics engine raw kinematics of the vehicle.
            %
            % Description:
            %   The position inside the returned KinematicsState is physics engine coordinate system, not Airsim.
            %   If you save it then you can load it back with simSetPhysicsRawKinematics.
            %   Accelaration is not used.
            %
            % Inputs:
            %   vehicleName - name of the vehicle.
            %
            % Outputs:
            %   kinematicsState - A structure containing:
            %     position(1x3) - The position of the vehicle in right-hand coordinates.
            %     orientation(1x4) - The quaternion representing the orientation of the vehicle in right-hand coordinates.
            %     linear_velocity(1x3) - The linear velocity of the vehicle in right-hand coordinates.
            %     angular_velocity(1x3) - The angular velocity of the vehicle in right-hand coordinates.
            %     linear_acceleration(1x3) - The linear acceleration of the vehicle in right-hand coordinates.
            %     angular_acceleration(1x3) - The angular acceleration of the vehicle in right-hand coordinates.

            vehicleStateAirSim = obj.rpc_client.call("simGetPhysicsRawKinematics", vehicleName);
            kinematicsState.position = obj.nedToRightHandCoordinates(struct2array(struct(vehicleStateAirSim{"position"})));
            kinematicsState.orientation = quatinv(struct2array(struct(vehicleStateAirSim{"orientation"})));
            kinematicsState.linear_velocity = obj.nedToRightHandCoordinates(struct2array(struct(vehicleStateAirSim{"linear_velocity"})));
            kinematicsState.angular_velocity = obj.nedToRightHandCoordinates(struct2array(struct(vehicleStateAirSim{"angular_velocity"})));
            kinematicsState.linear_acceleration = obj.nedToRightHandCoordinates(struct2array(struct(vehicleStateAirSim{"linear_acceleration"})));
            kinematicsState.angular_acceleration = obj.nedToRightHandCoordinates(struct2array(struct(vehicleStateAirSim{"angular_acceleration"})));
        end

        function setPhysicsRawKinematics(obj, position, orientation, linear_velocity, angular_velocity, linear_acceleration, angular_acceleration, vehicleName)
            % SETPHYSICSRAWKINEMATICS Set Physics engine raw kinematics of the vehicle.
            %
            % Description:
            %   Need to be simGetPhysicsRawKinematics value or same coordinate system.
            %   Accelaration is not used.
            %
            % Inputs:
            %   position(1x3) - The position of the vehicle in right-hand coordinates.
            %   orientation(1x4) - The quaternion representing the orientation of the vehicle in right-hand coordinates.
            %   linear_velocity(1x3) - The linear velocity of the vehicle in right-hand coordinates.
            %   angular_velocity(1x3) - The angular velocity of the vehicle in right-hand coordinates.
            %   linear_acceleration(1x3) - The linear acceleration of the vehicle in right-hand coordinates.
            %   angular_acceleration(1x3) - The angular acceleration of the vehicle in right-hand coordinates.
            %   vehicleName - name of the vehicle.

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
            kinematicsState.linear_acceleration.x_val = 0;
            kinematicsState.linear_acceleration.y_val = 0;
            kinematicsState.linear_acceleration.z_val = 0;
            kinematicsState.angular_acceleration.x_val = 0;
            kinematicsState.angular_acceleration.y_val = 0;
            kinematicsState.angular_acceleration.z_val = 0;
            obj.rpc_client.call("simSetPhysicsRawKinematics", kinematicsState, vehicleName);
        end

        function [EnvironmentState] = getGroundTruthEnvironment(obj, vehicleName)
            % GETGROUNDTRUTHENVIRONMENT Retrieve the ground truth environment data.
            %
            % Description:
            %   Retrieves the ground truth environment data including position, gravity, air pressure, temperature, air density, and geographic coordinates.
            %
            % Inputs:
            %   vehicleName - name of the vehicle.   
            %
            % Outputs:
            %   EnvironmentState - A structure containing:
            %     position(1x3) - The position of the vehicle in right-hand coordinates.
            %     gravity(1x3) - The gravity vector in right-hand coordinates.
            %     air_pressure - The air pressure.
            %     temperature - The temperature.
            %     air_density - The air density.
            %     geo_point - A structure containing:
            %       latitude - The latitude.
            %       longitude - The longitude.
            %       altitude - The altitude.

            EnvironmentStateData = obj.rpc_client.call("simGetGroundTruthEnvironment", vehicleName);
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


        function [MultirotorState] = getMultirotorState(obj)
            % GETMULTIROTORSTATE Get the current state of a multirotor vehicle
            %
            % Description:
            %   Retrieves and parses the current state of a multirotor vehicle from the AirSim API.
            %
            % Outputs:
            %   MultirotorState - Struct containing various state information:
            %     - collision: Collision information including time, object details, position, and impact.
            %     - kinematics_estimated: Estimated kinematic state including position, orientation, velocities,
            %                             and accelerations.
            %     - gps_location: GPS coordinates of the vehicle.
            %     - timestamp: Timestamp of the state.
            %     - landed_state: State indicating if the vehicle is landed.
            %     - ready: Ready state of the vehicle.
            %     - ready_message: Message indicating readiness status.
            %     - can_arm: Flag indicating if the vehicle can be armed.

            vehicleStateAirSim = obj.rpc_client.call("getMultirotorState", vehicleName);

            collisionData = vehicleStateAirSim{"collision"};
            collisionInfo = struct(collisionData);
            collisionInfo.time_stamp = double(double(collisionData{"time_stamp"}))/1e9;
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

            MultirotorState.timestamp = double(double(vehicleStateAirSim{"timestamp"}))/1e9;
            MultirotorState.landed_state = vehicleStateAirSim{"landed_state"};
            MultirotorState.ready = vehicleStateAirSim{"ready"};
            MultirotorState.ready_message = string(vehicleStateAirSim{"ready_message"});
            MultirotorState.can_arm = vehicleStateAirSim{"can_arm"};
        end

        function [CarState] = getCarState(obj, vehicleName)
            % GETCARSTATE Get the current state of a car vehicle
            %
            % Description:
            %   Retrieves and parses the current state of a car vehicle from the AirSim API.
            %
            % Inputs:
            %   vehicleName - name of the vehicle.   
            %
            % Outputs:
            %   CarState - Struct containing various state information:
            %     - collision: Collision information including time, object details, position, and impact.
            %     - kinematics_estimated: Estimated kinematic state including position, orientation, velocities,
            %                             and accelerations.
            %     - timestamp: Timestamp of the state.
            %     - speed: Current speed of the car.
            %     - gear: Current gear of the car.
            %     - rpm: Current RPM of the car engine.
            %     - maxrpm: Maximum RPM of the car engine.
            %     - handbrake: Handbrake status of the car.

            vehicleStateAirSim = obj.rpc_client.call("getCarState", vehicleName);

            collisionData = vehicleStateAirSim{"collision"};
            collisionInfo = struct(collisionData);
            collisionInfo.time_stamp = double(double(collisionData{"time_stamp"}))/1e9;
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

            CarState.timestamp = double(double(vehicleStateAirSim{"timestamp"}))/1e9;
            CarState.speed = vehicleStateAirSim{"speed"};
            CarState.gear = vehicleStateAirSim{"gear"};
            CarState.rpm = vehicleStateAirSim{"rpm"};
            CarState.maxrpm = vehicleStateAirSim{"maxrpm"};
            CarState.handbrake = vehicleStateAirSim{"handbrake"};
        end

        function [ComputerVisionState] = getComputerVisionState(obj, vehicleName)
            % GETCOMPUTERVISIONSTATE Get the current state of a computerVision vehicle
            %
            % Description:
            %   Retrieves and parses the current state of a computer vision vehicle from the AirSim API.
            %
            % Inputs:
            %   vehicleName - name of the vehicle.   
            %
            % Outputs:
            %   ComputerVisionState - Struct containing various state information:
            %     - kinematics_estimated: Estimated kinematic state including position, orientation, velocities,
            %                             and accelerations.
            %     - timestamp: Timestamp of the state.

            vehicleStateAirSim = obj.rpc_client.call("getComputerVisionState", vehicleName);

            kinematicData = vehicleStateAirSim{"kinematics_estimated"};
            kinematicsState.position = obj.nedToRightHandCoordinates(struct2array(struct(kinematicData{"position"})));
            kinematicsState.orientation = quatinv(struct2array(struct(kinematicData{"orientation"})));
            kinematicsState.linear_velocity = obj.nedToRightHandCoordinates(struct2array(struct(kinematicData{"linear_velocity"})));
            kinematicsState.angular_velocity = obj.nedToRightHandCoordinates(struct2array(struct(kinematicData{"angular_velocity"})));
            kinematicsState.linear_acceleration = obj.nedToRightHandCoordinates(struct2array(struct(kinematicData{"linear_acceleration"})));
            kinematicsState.angular_acceleration = obj.nedToRightHandCoordinates(struct2array(struct(kinematicData{"angular_acceleration"})));
            ComputerVisionState.kinematics_estimated = kinematicsState;

            ComputerVisionState.timestamp = double(double(vehicleStateAirSim{"timestamp"}))/1e9;
        end


        function reset(obj)
            % RESET Reset the simulation environment
            %
            % Description:
            %   Resets the entire simulation environment to its initial state.

            obj.rpc_client.call("reset");
        end
        
        function resetVehicle(obj, vehicleName)
            % RESETVEHICLE Reset a specific vehicle in the simulation
            %
            % Description:
            %   Resets a specific vehicle in the simulation environment to its initial state.
            %
            % Inputs:
            %   vehicleName - name of the vehicle.   

            obj.rpc_client.call("resetCar", vehicleName);
        end
        
        function [] = followTrajectory(obj, trajectoryPoses, elapsedTime, ignoreCollisions, vehicleName)
            % FOLLOWTRAJECTORY Move the vehicle to follow a trajectory
            %
            % Description:
            %   Sets the vehicle pose to follow a trajectory based on current elapsed time.
            %
            % Inputs:
            %   trajectoryPoses - Struct with trajectory poses including positions and orientations.
            %   elapsedTime - Current elapsed time to determine the trajectory step.
            %   ignoreCollisions - boolean toggle to ignore collisions or not.
            %   vehicleName - name of the vehicle.   

            stepIdx = find(elapsedTime <= trajectoryPoses.timestamps, 1, "first");
    
            position = trajectoryPoses.position(stepIdx, :);
            orientation = trajectoryPoses.orientation(stepIdx, :);
            obj.setVehiclePose(position, orientation, ignoreCollisions, vehicleName);
        end
        
        function setWeather(obj, weatherType, weatherValue)
            % SETWEATHER Set weather conditions in the simulation environment
            %
            % Description:
            %   Sets weather conditions such as rain, snow, or fog in the simulation.
            %
            % Inputs:
            %   weatherType - Type of weather condition to set (use AirSimWeather enum)
            %   weatherValue - Intensity or specific value of the weather condition.

            if weatherType == AirSimWeather.Enabled
                obj.rpc_client.call("simEnableWeather", weatherValue);
            else
                obj.rpc_client.call("simSetWeatherParameter", uint32(weatherType), weatherValue);
            end
        end 

         function success = testLineOfSightToPoint(obj, point)
            % TESTLINEOFSIGHTTOPOINT Test line of sight from the vehicle to a point in space
            %
            % Description:
            %   Checks if there is a line of sight from the vehicle to a specified point in space using the AirSim API.
            %
            % Inputs:
            %   point - 3-element vector [x, y, z] representing the point in space to test line of sight to.
            %
            % Outputs:
            %   success - Logical indicating whether there is line of sight (true) or not (false).

            success =  obj.rpc_client.call("simTestLineOfSightToPoint", point);
        end        

        function success = testLineOfSightBetweenPoints(obj, point1, point2)
            % TESTLINEOFSIGHTBETWEENPOINTS Test line of sight between two points in space
            %
            % Description:
            %   Checks if there is a direct line of sight between two specified points in space using the AirSim API.
            %
            % Inputs:
            %   point1 - 3-element vector [x, y, z] representing the first point in space.
            %   point2 - 3-element vector [x, y, z] representing the second point in space.
            %
            % Outputs:
            %   success - Logical indicating whether there is line of sight (true) or not (false).

            success = obj.rpc_client.call("simTestLineOfSightBetweenPoints", point1, point2);
        end

        function geoPoints = getWorldExtents(obj)
            % GETWORLDEXTENTS Get the world extents (bounding box) from the simulation
            %
            % Description:
            %   Retrieves the world extents (bounding box) from the simulation environment using the AirSim API.
            %
            % Outputs:
            %   geoPoints - Struct array containing the bounding box information with fields like latitude, longitude, and altitude.

            geopointListData = obj.rpc_client.call("simGetWorldExtents");
            geoPointsList = cell(geopointListData);
            geoPoints = struct(geoPointsList{1});
            for cellIndex = 2 : numel(geoPointsList)
                geoPoints(cellIndex) = struct(geoPointsList{cellIndex});
            end
        end    

        function success = runConsoleCommand(obj, command)
            % RUNCONSOLECOMMAND Run a console command in the simulation environment
            %
            % Description:
            %   Executes a console command within the simulation environment via the AirSim API.
            %
            % Inputs:
            %   command - String representing the console command to execute.
            %
            % Outputs:
            %   success - Logical indicating the success of executing the command (true/false).

            success = obj.rpc_client.call("simRunConsoleCommand", command);
        end

        function logMessage = printLogMessage(obj, message, message_param, severity)
            % PRINTLOGMESSAGE Print log message via AirSim API.
            %
            % Description:
            %   Sends a log message to AirSim for logging purposes.
            %
            % Inputs:
            %   message (string) - The message to log.
            %   message_param (string) - Additional message parameters (if any).
            %   severity (int) - Severity level of the log message.

            logMessage = string(obj.rpc_client.call("simPrintLogMessage", message, message_param, severity));
        end

        function vertexData = getMeshPositionVertexBuffers(obj)
            % GETMESHPOSITIONVERTEXBUFFERS Get vertex buffers of mesh position data
            %
            % Description:
            %   Retrieves vertex buffer data of mesh positions from the AirSim API.
            %
            % Outputs:
            %   vertexData - Struct array containing vertex data, including positions and orientations.

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

        function collisionInfo = getCollisionInfo(obj, vehicleName)
            % GETCOLLISIONINFO Get collision information for the vehicle
            %
            % Description:
            %   Retrieves collision information for the vehicle from the AirSim API.
            %
            % Inputs:
            %   vehicleName - name of the vehicle.   
            %
            % Outputs:
            %   collisionInfo - Struct containing collision information such as time stamp, object name, ID, position, normal, and impact point.

            collisionData = obj.rpc_client.call("simGetCollisionInfo", vehicleName);
            collisionInfo = struct(collisionData);
            collisionInfo.time_stamp = double(double(collisionData{"time_stamp"}))/1e9;
            collisionInfo.object_name = string(collisionData{"object_name"});
            collisionInfo.object_id = double(collisionData{"object_id"});
            collisionInfo.position = struct2array(struct(collisionData{"position"}));
            collisionInfo.normal = struct2array(struct(collisionData{"normal"}));
            collisionInfo.impact_point = struct2array(struct(collisionData{"impact_point"}));
        end


        function geopoint = getHomeGeoPoint(obj, vehicleName)
            % GETHOMEGEOPOINT Get the home geopoint of the vehicle
            %
            % Description:
            %   Retrieves the geopoint (latitude, longitude, altitude) of the vehicle's home position.
            %
            % Inputs:
            %   vehicleName - name of the vehicle.   
            %
            % Outputs:
            %   geopoint - Struct containing latitude, longitude, and altitude of the home position.

            geopointData = obj.rpc_client.call("getHomeGeoPoint", vehicleName);
            geopoint.latitude = double(geopointData{"latitude"});
            geopoint.longitude = double(geopointData{"longitude"});
            geopoint.altitude = double(geopointData{"altitude"});
        end

        function setLightIntensity(obj, light_name, intensity)
            % SETLIGHTINTENSITY Set the intensity of a light in the simulation
            %
            % Description:
            %   Sets the intensity of a light in the simulation.
            %
            % Inputs:
            %   light_name - Name of the light.
            %   intensity - Intensity value to set.            

            warning("simSetLightIntensity is deprecated, use the new Artificial Light API instead");
            obj.setWorldLightIntensity(light_name, intensity);
        end

        function swapTextures(obj, tags, tex_id, component_id, material_id)
            % SWAPTEXTURES Swap textures on a component of an object
            %
            % Description:
            %   Swaps textures on a specified component of an object in the simulation.
            %
            % Inputs:
            %   tags - Tags for the texture swap operation.
            %   tex_id - Texture ID.
            %   component_id - Component ID of the object.
            %   material_id - Material ID of the object.

            obj.rpc_client.call("simSwapTextures", tags, tex_id, component_id, material_id);
        end

        function setObjectMaterial(obj, object_name, material_name, component_id)
            % SETOBJECTMATERIAL Set material on a specific object component
            %
            % Description:
            %   Sets a material on a specific component of an object in the simulation.
            %
            % Inputs:
            %   object_name - Name of the object.
            %   material_name - Name of the material to set.
            %   component_id - Component ID of the object.

            obj.rpc_client.call("simSetObjectMaterial", object_name, material_name, component_id);
        end
        
        function setObjectMaterialFromTexture(obj, object_name, texture_path, component_id)
            % SETOBJECTMATERIALFROMTEXTURE Set material on an object component from a texture
            %
            % Description:
            %   Sets a material on a specific component of an object in the simulation using a texture.
            %
            % Inputs:
            %   object_name - Name of the object.
            %   texture_path - Path to the texture.
            %   component_id - Component ID of the object.

            obj.rpc_client.call("simSetObjectMaterialFromTexture", object_name, texture_path, component_id);
        end

        function setTimeOfDay(obj, is_enabled, start_datetime, is_start_datetime_dst, celestial_clock_speed,...
                              update_interval_secs, move_sun)
            % SETTIMEOFDAY Set the time of day in the simulation
            %
            % Description:
            %   Sets the time of day and related parameters in the simulation.
            %
            % Inputs:
            %   is_enabled - Logical value to enable time of day simulation.
            %   start_datetime - Start datetime for simulation.
            %   is_start_datetime_dst - Logical value indicating if start datetime is DST.
            %   celestial_clock_speed - Speed of the celestial clock.
            %   update_interval_secs - Update interval in seconds.
            %   move_sun - Logical value to move the sun based on simulation parameters.

            obj.rpc_client.call("simSetTimeOfDay", is_enabled, start_datetime, is_start_datetime_dst, celestial_clock_speed,...
                                update_interval_secs, move_sun);
        end

        function success = setVehicleLightIntensity(obj, vehicle_name, light_name, intensity)
            % SETVEHICLELIGHTINTENSITY Set the intensity of a vehicle light
            %
            % Description:
            %   Sets the intensity value of an artificial light associated with a specific vehicle.
            %
            % Inputs:
            %   vehicle_name - Name of the vehicle (string).
            %   light_name - Name of the light within the vehicle (string).
            %   intensity - Intensity value for the light (float). The unit depends on the simulation's intensity unit setting.
            %
            % Outputs:
            %   success - Logical value, true if the light intensity was changed successfully.
            %

            success = obj.rpc_client.call("simSetVehicleLightIntensity", vehicle_name, light_name, intensity);
        end

        function success = setWorldLightVisibility(obj, light_name, is_visible)
            % SETWORLDLIGHTVISIBILITY Enable or disable a static world light
            %
            % Description:
            %   Toggles the visibility of an artificial light set as a static world light.
            %
            % Inputs:
            %   light_name - Name of the world light (string).
            %   is_visible - Logical value to enable (true) or disable (false) the light.
            %
            % Outputs:
            %   success - Logical value, true if the light visibility was toggled successfully.
            %

            success = obj.rpc_client.call("simSetWorldLightVisibility", light_name, is_visible);
        end

        function success = setWorldLightIntensity(obj, light_name, intensity)
            % SETWORLDLIGHTINTENSITY Set the intensity of a static world light
            %
            % Description:
            %   Sets the intensity value of an artificial light set as a static world light.
            %
            % Inputs:
            %   light_name - Name of the world light (string).
            %   intensity - Intensity value for the light (float). The unit depends on the simulation's intensity unit setting.
            %
            % Outputs:
            %   success - Logical value, true if the light intensity was changed successfully.
            %

            success = obj.rpc_client.call("simSetWorldLightIntensity", light_name, intensity);
        end

        function success = setVehicleLightVisibility(obj, vehicle_name, light_name, is_visible)
            % SETVEHICLELIGHTVISIBILITY Enable or disable a vehicle light
            %
            % Description:
            %   Toggles the visibility of an artificial light set as a vehicle light.
            %
            % Inputs:
            %   vehicle_name - Name of the vehicle associated with the light (string).
            %   light_name - Name of the light within the vehicle (string).
            %   is_visible - Logical value to enable (true) or disable (false) the light.
            %
            % Outputs:
            %   success - Logical value, true if the vehicle light visibility was toggled successfully.
            %

            success = obj.rpc_client.call("simSetVehicleLightVisibility", vehicle_name, light_name, is_visible);
        end

        function flushPersistentMarkers(obj)
            % FLUSHPERSISTENTMARKERS Flush persistent markers in the simulation
            %
            % Description:
            %   Flushes all persistent markers in the simulation.

            obj.rpc_client.call('simFlushPersistentMarkers');
        end               
        
        function cancelLastTask(obj, vehicleName)
            % CANCELLASTTASK Cancel the last task in the simulation
            %
            % Description:
            %   Cancels the last task executed in the simulation.
            %
            % Inputs:
            %   vehicleName - name of the vehicle.   

            obj.rpc_client.call('cancelLastTask', vehicleName);
        end
        
        function startRecording(obj)
            % STARTRECORDING Start recording data in the simulation
            %
            % Description:
            %   Starts recording data (e.g., sensor data) in the simulation.

            obj.rpc_client.call('startRecording');
        end
        
        function stopRecording(obj)
            % STOPRECORDING Stop recording data in the simulation
            %
            % Description:
            %   Stops recording data in the simulation.

            obj.rpc_client.call('stopRecording');
        end
        
        function recording = isRecording(obj)
            % ISRECORDING Check if recording is currently active in the simulation
            %
            % Description:
            %   Checks if any recording (e.g., sensor data) is currently active in the simulation.
            %
            % Outputs:
            %   recording - Logical value indicating if recording is active.

            recording = obj.rpc_client.call('isRecording');
        end
        
        function setWind(obj, wind)
            % SETWIND Set the wind parameters in the simulation
            %
            % Description:
            %   Sets the wind parameters (direction and intensity) in the simulation.
            %
            % Inputs:
            %   wind - Wind parameters to set.

            obj.rpc_client.call('simSetWind', wind);
        end
        
        function success = createVoxelGrid(obj, position, x, y, z, res, of)
            % CREATEVOXELGRID Create a voxel grid in the simulation
            %
            % Description:
            %   Creates a voxel grid at the specified position with given dimensions and resolution in the simulation.
            %
            % Inputs:
            %   position - Position coordinates [x, y, z] of the voxel grid.
            %   x, y, z - Dimensions of the voxel grid.
            %   res - Resolution of the voxel grid.
            %   of - Name of output file to save voxel grid as
            %
            % Outputs:
            %   success - Logical value indicating if voxel grid creation was successful.

            newPostiton.x_val = position(1);
            newPostiton.y_val  = -position(2);
            newPostiton.z_val  = -position(3);
            success = obj.rpc_client.call('simCreateVoxelGrid', newPostiton, x, y, z, res, of);
        end
        
        function success = addVehicle(obj, vehicle_name, vehicle_type, position, orientation, pawn_path)
            % ADDVEHICLE Add a new vehicle to the simulation
            %
            % Description:
            %   Adds a new vehicle to the simulation environment with specified parameters.
            %
            % Inputs:
            %   vehicle_name - Name of the new vehicle.
            %   vehicle_type - Type of the vehicle.
            %   position - Position coordinates [x, y, z] of the vehicle.
            %   orientation - Orientation quaternion [w, x, y, z] of the vehicle.
            %   pawn_path - Optional path to pawn file for the vehicle (default "").
            %
            % Outputs:
            %   success - Logical value indicating if vehicle addition was successful.

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
            % LISTVEHICLES List all vehicles in the simulation
            %
            % Description:
            %   Retrieves a list of all vehicles currently present in the simulation.
            %
            % Outputs:
            %   vehicles - Array of strings containing names of all vehicles.

            vehicles = string(cell(obj.rpc_client.call("listVehicles")))';
        end
        
        function settings = getSettingsString(obj)
            % GETSETTINGSTRING Get settings string from the simulation
            %
            % Description:
            %
            % Outputs:
            %   settings - String containing the settings information.

            settings = string(obj.rpc_client.call('getSettingsString'));
        end
        
        function simSetExtForce(obj, ext_force)
            % SIMSETEXTFORCE Set an external force in the simulation
            %
            % Description:
            %   Sets an external force in the simulation environment.
            %
            % Inputs:
            %   ext_force - External force vector [x, y, z] to apply.

            newForce.x_val = ext_force(1);
            newForce.y_val  = -ext_force(2);
            newForce.z_val  = -ext_force(3);
            obj.rpc_client.call('simSetExtForce', newForce);
        end

        function setTraceLine(obj, color_rgba, thickness, vehicleName)
            % SETTRACELINE Set a trace line for visualization in the simulation
            %
            % Description:
            %   Sets a trace line for visualization purposes in the simulation via the AirSim API.
            %
            % Inputs:
            %   color_rgba - RGBA color array [R, G, B, A] for the trace line.
            %   thickness - Thickness of the trace line.
            %   vehicleName - name of the vehicle.   

            obj.rpc_client.call("simSetTraceLine", py.list(color_rgba), thickness, vehicleName);
        end

        function [objectPose] = getObjectPose(obj, objectName, local)
            % GETOBJECTPOSE Get 3D pose of an object
            %
            % Description:
            %   Retrieves the 3D pose (position and orientation) of a specified object from the AirSim API.
            %
            % Inputs:
            %   objectName - Name of the object to retrieve pose for.
            %   local - Logical indicating whether to retrieve pose in local coordinates (true/false).
            %
            % Outputs:
            %   objectPose - Struct containing 3D pose information (position and orientation) of the object.


            objectData = obj.rpc_client.call("simGetObjectPose", objectName, local);      
            objectPose.position = obj.nedToRightHandCoordinates(struct2array(struct(objectData{"position"})));
            objectPose.orientation = quatinv(struct2array(struct(objectData{"orientation"})));
        end

        function [objectPosesTable] = getAllObjectPoses(obj, local, only_visible)
            % GETALLOBJECTPOSES Get 3D pose of all objects in the scene
            %
            % Description:
            %   Retrieves the 3D pose (position and orientation) of all objects in the scene via the AirSim API.
            %
            % Inputs:
            %   local - Logical indicating whether to retrieve poses in local coordinates (true/false).
            %   only_visible - Logical indicating whether to only show visible objects (true/false).
            %
            % Outputs:
            %   objectPosesTable - Table containing names of objects and their corresponding 3D poses (position and orientation)

            name = obj.getInstanceSegmentationObjectList();
            objectPosesList = obj.rpc_client.call("simListInstanceSegmentationPoses", local, only_visible);    
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

        function success = setObjectPose(obj, objectName, position, orientation, teleport)
            % SETOBJECTPOSE Set pose (position and orientation) of an object
            %
            % Description:
            %   Sets the pose (position and orientation) of a specified object in the simulation via the AirSim API.
            %
            % Inputs:
            %   objectName - Name of the object to set pose for.
            %   position - 3-element vector [x, y, z] representing the position.
            %   orientation - Quaternion [w, x, y, z] representing the orientation.
            %   teleport - Logical indicating whether to teleport the object to the new pose (true/false).
            %
            % Outputs:
            %   success - Logical indicating the success of setting the object pose (true/false).

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
            % GETOBJECTSCALE Get scale of an object
            %
            % Description:
            %   Retrieves the scale of a specified object from the AirSim API.
            %
            % Inputs:
            %   objectName - Name of the object to retrieve scale for.
            %
            % Outputs:
            %   scale - 3-element vector [x, y, z] representing the scale of the object.

            scale = struct2array(struct(obj.rpc_client.call("simGetObjectScale", objectName)));  
        end

        function setObjectScale(obj, objectName, scale)
            % SETOBJECTSCALE Set scale of an object
            %
            % Description:
            %   Sets the scale of a specified object in the simulation via the AirSim API.
            %
            % Inputs:
            %   objectName - Name of the object to set scale for.
            %   scale - 3-element vector [x, y, z] representing the new scale.

            newScale.x_val = scale(1);
            newScale.y_val = scale(2);
            newScale.z_val = scale(3);
            obj.rpc_client.call("simSetObjectScale", objectName, newScale);  
        end

        function [objectList] = listSceneObjects(obj, name_regex)
            % LISTSCENEOBJECTS List all scene objects matching a name pattern
            %
            % Description:
            %   Retrieves a list of scene objects that match a specified name pattern from the AirSim API.
            %
            % Inputs:
            %   name_regex - Optional string pattern to filter objects by name.
            %
            % Outputs:
            %   objectList - String array containing names of scene objects matching the pattern.

            arguments
                obj AirSimClient
                name_regex string = ".*"
            end
            objectList = string(cell(obj.rpc_client.call("simListSceneObjects", name_regex)))';
        end

        function [objectList] = listSceneObjectsTags(obj, name_regex)
            % LISTSCENEOBJECTTAGS ists the objects present and the given tag matching the regular expression in the environment.
            %
            % Description:
            % The default behavior is to list all objects with first tag. A regex can be used to return a smaller list
            % of matching objects or actors.
            %
            % Inputs:
            %   name_regex - String to match actor tags against, e.g., "Gate.*".
            %
            % Outputs:
            %   objectList - String array containing names of scene objects matching the pattern.

            arguments
                obj AirSimClient
                name_regex string = ".*"
            end
            objectList = string(cell(obj.rpc_client.call("simListSceneObjectsTags", name_regex)))';
        end

        function success = loadLevel(obj, level_name)
            % LOADLEVEL Load a new level into the simulation
            %
            % Description:
            %   Loads a new level into the simulation environment via the AirSim API.
            %
            % Inputs:
            %   level_name - Name of the level to load.
            %
            % Outputs:
            %   success - Logical indicating the success of loading the level (true/false).

            success = obj.rpc_client.call("simLoadLevel", level_name);
        end


        function [objectList] = listAssets(obj)
            % LISTASSETS List all assets available in the simulation
            %
            % Description:
            %   Retrieves a list of all assets available in the simulation from the AirSim API.
            %
            % Outputs:
            %   objectList - String array containing names of all assets in the simulation.

            objectList = string(cell(obj.rpc_client.call("simListAssets")))';
        end

        function objectName = spawnObject(obj, object_name, asset_name, position, orientation, scale, physics_enabled, is_blueprint)
            % SPAWNOBJECT Spawn a new object in the simulation
            %
            % Description:
            %   Spawns a new object with specified parameters into the simulation via the AirSim API.
            %
            % Inputs:
            %   object_name - Name of the object to be spawned.
            %   asset_name - Name of the asset for the object.
            %   position - 3-element vector [x, y, z] representing the spawn position.
            %   orientation - Quaternion [w, x, y, z] representing the spawn orientation.
            %   scale - 3-element vector [x, y, z] representing the scale of the object.
            %   physics_enabled - Logical indicating whether physics is enabled for the object (true/false).
            %   is_blueprint - Logical indicating whether the object is a blueprint (true/false).
            %
            % Outputs:
            %   objectName - String representing the name of the spawned object.

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
            % DESTROYOBJECT Destroy an object in the simulation
            %
            % Description:
            %   Destroys a specified object in the simulation environment via the AirSim API.
            %
            % Inputs:
            %   object_name - Name of the object to be destroyed.
            %
            % Outputs:
            %   success - Logical indicating the success of destroying the object (true/false).

            success = obj.rpc_client.call("simDestroyObject", object_name);
        end     


        function [objectList] = getInstanceSegmentationObjectList(obj)
            % GETINSTANCESEGMENTATIONOBJECTLIST Get list of instance segmentation objects
            %
            % Description:
            %   Retrieves a list of all objects in the scene for instance segmentation via the AirSim API.
            %
            % Outputs:
            %   objectList - String array containing names of all instance segmentation objects.

            objectList = string(cell(obj.rpc_client.call("simListInstanceSegmentationObjects")))';
        end

        function [instanceSegmentationTable] = getInstanceSegmentationLUT(obj)
            % GETINSTANCESEGMENTATIONLUT Get instance segmentation lookup table (LUT)
            %
            % Description:
            %   Retrieves a lookup table (LUT) of all instance segmentation objects and their RGB values for groundtruth labeling from the AirSim API.
            %
            % Outputs:
            %   instanceSegmentationTable - Table containing names of objects and their corresponding RGB segmentation values.

            name = obj.getInstanceSegmentationObjectList();
            colorMap = obj.getInstanceSegmentationColormap();
            r = colorMap(1:size(name, 1), 1);
            g = colorMap(1:size(name, 1), 2);
            b = colorMap(1:size(name, 1), 3);
            instanceSegmentationTable = table(name, r, g, b);
        end

        function success = setSegmentationObjectID(obj, mesh_name, object_id, is_name_regex)
            % SETSEGMENTATIONOBJECTID Set segmentation ID for an object
            %
            % Description:
            %   Sets the segmentation ID for a specified object in the simulation via the AirSim API.
            %
            % Inputs:
            %   mesh_name - Name of the mesh object to set segmentation ID for.
            %   object_id - Numeric ID value to assign to the object for segmentation.
            %   is_name_regex - Logical indicating whether

            success = obj.rpc_client.call("simSetSegmentationObjectID", mesh_name, object_id, is_name_regex);
        end

        function objectID = getSegmentationObjectID(obj, mesh_name)
            % GETSEGMENTATIONOBJECTID Get segmentation ID of an object
            %
            % Description:
            %   Retrieves the segmentation ID of a specified object from the AirSim API.
            %
            % Inputs:
            %   mesh_name - Name of the mesh object to retrieve segmentation ID for.
            %
            % Outputs:
            %   objectID - Numeric ID value representing the segmentation ID of the object.

            objectID = obj.rpc_client.call("simGetSegmentationObjectID", mesh_name);
        end

        function [objectList] = listAnnotationObjects(obj, annotation_name)
            % SIMLISTANNOTATIONOBJECTS List annotation objects for a specific annotation layer.
            %
            % Description:
            %   Retrieves a list of annotation objects matching for a specific annotation layer from the AirSim API.
            %
            % Inputs:
            %   annotation_name - Name of the annotation layer to retrieve.
            %
            % Outputs:
            %   objectList - String array containing names of annotation objects matching the pattern.

            objectList = string(cell(obj.rpc_client.call("simListAnnotationObjects", annotation_name)))';
        end

        function [objectPosesTable] = listAnnotationPoses(obj, annotation_name, local, only_visible)
            % LISTANNOTATIONPOSES List poses of annotation objects for a specific annotation layer.
            %
            % Description:
            %   Retrieves the poses (position and orientation) of annotation objects for a specific annotation layer from the AirSim API.
            %
            % Inputs:
            %   annotation_name - Name of the annotation layer to retrieve poses for.
            %   local - Logical indicating whether to retrieve poses in local coordinates (true/false).
            %   only_visible - Logical indicating whether to only show visible objects (true/false).
            %
            % Outputs:
            %   objectPosesTable - Table containing names of annotation objects and their corresponding 3D poses (position and orientation).

            name = obj.listAnnotationObjects(annotation_name);
            objectPosesList = obj.rpc_client.call("simListAnnotationPoses", annotation_name, local, only_visible);    
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
            % SIMSETANNOTATIONOBJECTID Set ID for an annotation object. This works only for RGB layers.
            %
            % Description:
            %   Sets the ID for a specified annotation object in the simulation via the AirSim API.
            %   This works only for RGB layers.
            %
            % Inputs:
            %   annotation_name (char) - Name of the annotation layer to set ID for.
            %   mesh_name (char) - Name of the mesh object to set ID for.
            %   object_id (double) - Numeric ID value to assign to the annotation object.
            %   is_name_regex (logical, optional) - Whether mesh_name is a regex pattern (default is false).
            %
            % Outputs:
            %   success (logical) - Success status of setting the annotation object ID.

            success = obj.rpc_client.call("simSetAnnotationObjectID", annotation_name, mesh_name, object_id, is_name_regex);
        end

        function objectID = getAnnotationObjectID(obj, mesh_name)
            % SIMGETANNOTATIONOBJECTID Get ID of an annotation object. This works only for RGB layers.
            %
            % Description:
            %   Retrieves the ID of a specified annotation object from the AirSim API.
            %   This works only for RGB layers.
            %
            % Inputs:
            %   annotation_name (char) - Name of the annotation layer to retrieve ID for.
            %   mesh_name (char) - Name of the mesh object to retrieve ID for.
            %
            % Outputs:
            %   objectID (double) - Numeric ID value representing the ID of the annotation object.

            objectID = obj.rpc_client.call("simGetAnnotationObjectID", mesh_name);
        end

        function success = setAnnotationObjectColor(obj, annotation_name, mesh_name, r, g, b, is_name_regex)
            % SIMSETANNOTATIONOBJECTCOLOR Set color for an annotation object. This works only for RGB layers.
            %
            % Description:
            %   Sets the color for a specified annotation object in the simulation via the AirSim API.
            %   This works only for RGB layers.
            %
            % Inputs:
            %   annotation_name (char) - Name of the annotation layer to set color for.
            %   mesh_name (char) - Name of the mesh object to set color for.
            %   r (double) - Red component of the color (0-255).
            %   g (double) - Green component of the color (0-255).
            %   b (double) - Blue component of the color (0-255).
            %   is_name_regex (logical, optional) - Whether mesh_name is a regex pattern (default is false).
            %
            % Outputs:
            %   success (logical) - Success status of setting the annotation object color.

            success = obj.rpc_client.call("simSetAnnotationObjectColor", annotation_name, mesh_name, r, g, b, is_name_regex);
        end

        function objectColor = getAnnotationObjectColor(obj, mesh_name)
            % SIMGETANNOTATIONOBJECTCOLOR Get color of an annotation object. This works only for RGB layers.
            %
            % Description:
            %   Retrieves the color of a specified annotation object from the AirSim API. 
            %   This works only for RGB layers.
            %
            % Inputs:
            %   annotation_name (char) - Name of the annotation layer to retrieve color for.
            %   mesh_name (char) - Name of the mesh object to retrieve color for.
            %
            % Outputs:
            %   objectColor (double) - RGB color values representing the color of the annotation object.

            objectColor = string(obj.rpc_client.call("simGetAnnotationObjectColor", mesh_name));
        end

        function success = setAnnotationObjectValue(obj, annotation_name, mesh_name, greyscale_value, is_name_regex)
            % SIMSETANNOTATIONOBJECTVALUE Set value for an annotation object. This only works for greyscale layers.
            %
            % Description:
            %   Sets the greyscale value for a specified annotation object in the simulation via the AirSim API.
            %   This only works for greyscale layers.
            %
            % Inputs:
            %   annotation_name (char) - Name of the annotation layer to set value for.
            %   mesh_name (char) - Name of the mesh object to set value for.
            %   greyscale_value (double) - Greyscale value to assign to the annotation object.
            %   is_name_regex (logical, optional) - Whether mesh_name is a regex pattern (default is false).
            %
            % Outputs:
            %   success (logical) - Success status of setting the annotation object value.

            success = obj.rpc_client.call("simSetAnnotationObjectValue", annotation_name, mesh_name, greyscale_value, is_name_regex);
        end

        function greyscaleValue = getAnnotationObjectValue(obj, mesh_name)
            % SIMGETANNOTATIONOBJECTVALUE Get value of an annotation object. his only works for greyscale layers.
            %
            % Description:
            %   Retrieves the greyscale value of a specified annotation object from the AirSim API.
            %   This only works for greyscale layers.
            %
            % Inputs:
            %   annotation_name (char) - Name of the annotation layer to retrieve value for.
            %   mesh_name (char) - Name of the mesh object to retrieve value for.
            %
            % Outputs:
            %   greyscaleValue (double) - Greyscale value representing the value of the annotation object.

            greyscaleValue = obj.rpc_client.call("simGetAnnotationObjectValue", mesh_name);
        end

        function success = setAnnotationObjectTextureByPath(obj, annotation_name, mesh_name, texture_path, is_name_regex)
            % SIMSETANNOTATIONOBJECTTEXTUREBYPATH Set texture for an annotation object by path. This only works for texture layers.
            %
            % Description:
            %   Sets the texture for a specified annotation object by specifying the texture path via the AirSim API.
            %   This only works for texture layers. See documentation on annotation for texture path syntax.
            %
            % Inputs:
            %   annotation_name (char) - Name of the annotation layer to set texture for.
            %   mesh_name (char) - Name of the mesh object to set texture for.
            %   texture_path (char) - File path to the texture to assign to the annotation object.
            %   is_name_regex (logical, optional) - Whether mesh_name is a regex pattern (default is false).
            %
            % Outputs:
            %   success (logical) - Success status of setting the annotation object texture.

            success = obj.rpc_client.call("simSetAnnotationObjectTextureByPath", annotation_name, mesh_name, texture_path, is_name_regex);
        end

        function success = enableAnnotationObjectTextureByPath(obj, annotation_name, mesh_name, is_name_regex)
            % SIMENABLEANNOTATIONOBJECTTEXTUREBYPATH Enable texture for an annotation object by path. This only works for texture layers.
            %
            % Description:
            %   Enables the texture for a specified annotation object by specifying the texture path via the AirSim API.
            %   This only works for texture layers. This requires the texture to be present in the expected location. See annotation documentation for more info.
            %
            % Inputs:
            %   annotation_name (char) - Name of the annotation layer to enable texture for.
            %   mesh_name (char) - Name of the mesh object to enable texture for.
            %   is_name_regex (logical, optional) - Whether mesh_name is a regex pattern (default is false).
            %
            % Outputs:
            %   success (logical) - Success status of enabling the annotation object texture.

            success = obj.rpc_client.call("simEnableAnnotationObjectTextureByPath", annotation_name, mesh_name, is_name_regex);
        end

        function texturePath = getAnnotationObjectTexturePath(obj, mesh_name)
            % SIMGETANNOTATIONOBJECTTEXTUREPATH Get texture path of an annotation object. This only works for texture layers.
            %
            % Description:
            %   Retrieves the texture path assigned to a specified annotation object from the AirSim API.
            %   This only works for texture layers.
            %
            % Inputs:
            %   annotation_name (char) - Name of the annotation layer to retrieve texture path for.
            %   mesh_name (char) - Name of the mesh object to retrieve texture path for.
            %
            % Outputs:
            %   texturePath (char) - File path to the texture assigned to the annotation object.

            texturePath = string(obj.rpc_client.call("simGetAnnotationObjectTexturePath", mesh_name));
        end

        function addDetectionFilterMeshName(obj, camera_name, image_type, mesh_name, vehicleName, annotation_name)
            % ADDDETECTIONFILTERMESHNAME Add mesh name to detection filter.
            %
            % Description:
            %   Adds a mesh name to the detection filter for a specified camera and image type.
            %
            % Inputs:
            %   camera_name (string) - Name of the camera to add detection filter for.
            %   image_type (uint32) - Type of image to add detection filter for.
            %   mesh_name (string) - Name of the mesh object to add to the detection filter.
            %   vehicleName - name of the vehicle.   
            %   annotation_name (string, optional) - Name of the annotation to filter (default is ".*").

            arguments
                obj AirSimClient
                camera_name string
                image_type uint32
                mesh_name string
                vehicleName string
                annotation_name string = ".*"
            end
            obj.rpc_client.call("simAddDetectionFilterMeshName", camera_name, image_type, mesh_name, vehicleName, annotation_name);
        end

        function setDetectionFilterRadius(obj, camera_name, image_type, radius_cm, vehicleName, annotation_name)
            % SETDETECTIONFILTERRADIUS Set detection filter radius.
            %
            % Description:
            %   Sets the radius of detection filter for a specified camera and image type.
            %
            % Inputs:
            %   camera_name (string) - Name of the camera to set detection filter for.
            %   image_type (uint32) - Type of image to set detection filter for.
            %   radius_cm (int32) - Radius value in centimeters.
            %   vehicleName - name of the vehicle.   
            %   annotation_name (string, optional) - Name of the annotation to filter (default is ".*").

            arguments
                obj AirSimClient
                camera_name string
                image_type uint32
                radius_cm int32
                vehicleName string
                annotation_name string = ".*"
            end
            obj.rpc_client.call("simSetDetectionFilterRadius", camera_name, image_type, radius_cm, vehicleName, annotation_name);
        end

        function clearDetectionMeshNames(obj, camera_name, image_type, vehicleName, annotation_name)
            % CLEARDETECTIONMESHNAMES Clear detection mesh names from filter.
            %
            % Description:
            %   Clears all mesh names from the detection filter for a specified camera and image type.
            %
            % Inputs:
            %   camera_name (string) - Name of the camera to clear detection filter for.
            %   image_type (uint32) - Type of image to clear detection filter for.
            %   vehicleName - name of the vehicle.   
            %   annotation_name (string, optional) - Name of the annotation to clear (default is ".*").

            arguments
                obj AirSimClient
                camera_name string
                image_type uint32
                vehicleName string
                annotation_name string = ".*"
            end
            obj.rpc_client.call("simClearDetectionMeshNames", camera_name, image_type, vehicleName, annotation_name);
        end

        function detectionInfo = simGetDetections(obj)
            % SIMGETDETECTIONS Get detection information.
            %
            % Description:
            %   Retrieves information about detections from the AirSim API.
            %
            % Outputs:
            %   detectionInfo (struct array) - Array of structures containing detection information.

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

        function setDistortionParam(obj, camera_name, param_name, value, vehicleName)
            % SETDISTORTIONPARAM Set distortion parameter via AirSim API.
            %
            % Description:
            %   Sets a distortion parameter for a specified camera.
            %
            % Inputs:
            %   camera_name (string) - Name of the camera to set distortion parameter for.
            %   param_name (string) - Name of the distortion parameter to set.
            %   value - Value to set for the distortion parameter.
            %   vehicleName - name of the vehicle.   

            obj.rpc_client.call("simSetDistortionParam", camera_name, param_name, value, vehicleName);
        end

        function setCameraPose(obj, camera_name, position, orientation, vehicleName)
            % SETCAMERAPOSE Set camera pose via AirSim API.
            %
            % Description:
            %   Sets the pose (position and orientation) of a specified camera.
            %
            % Inputs:
            %   camera_name (string) - Name of the camera to set pose for.
            %   position (double array) - Position coordinates [x, y, z] of the camera.
            %   orientation (quaternion array) - Orientation quaternion [w, x, y, z] of the camera.
            %   vehicleName - name of the vehicle.   

            newPose.position.x_val = position(1);
            newPose.position.y_val = -position(2);
            newPose.position.z_val = -position(3);
            orientation = quatinv(orientation);
            
            newPose.orientation.w_val = orientation(1);
            newPose.orientation.x_val = orientation(2);
            newPose.orientation.y_val = orientation(3);
            newPose.orientation.z_val = orientation(4);
            obj.rpc_client.call("simSetCameraPose", camera_name, newPose, vehicleName);
        end

        function simSetCameraFov(obj, camera_name, fov_degrees, vehicleName)
            % SIMSETCAMERAFOV Set camera field of view (FOV) via AirSim API.
            %
            % Description:
            %   Sets the field of view (FOV) of a specified camera.
            %
            % Inputs:
            %   camera_name (string) - Name of the camera to set FOV for.
            %   fov_degrees (double) - Field of view angle in degrees.
            %   vehicleName - name of the vehicle.   

            obj.rpc_client.call("setCameraFov", camera_name, fov_degrees, vehicleName);
        end

        function settings = getPresetLensSettings(obj, sensorName, vehicleName)
            % GETPRESETLENSSETTINGS Get preset lens settings for a specific sensor
            %
            % Description:
            %   Retrieves preset lens settings (e.g., focal length) for a specified sensor from the AirSim API.
            %
            % Inputs:
            %   sensorName - Name of the sensor to retrieve settings for.
            %   vehicleName - name of the vehicle.   
            %
            % Outputs:
            %   settings - String representing the preset lens settings.

            returnData = obj.rpc_client.call("simGetPresetLensSettings", sensorName, vehicleName);
            settings = string(cell(returnData));
        end

        function settings = getLensSettings(obj, sensorName, vehicleName)
            % GETLENSSETTINGS Get current lens settings for a specific sensor
            %
            % Description:
            %   Retrieves current lens settings (e.g., focal length) for a specified sensor from the AirSim API.
            %
            % Inputs:
            %   sensorName - Name of the sensor to retrieve settings for.
            %   vehicleName - name of the vehicle.   
            %
            % Outputs:
            %   settings - String representing the current lens settings.

            returnData = obj.rpc_client.call("simGetLensSettings", sensorName, vehicleName);
            settings = string(returnData);
        end

        function settings = setPresetLensSettings(obj, preset_lens_settings, sensorName, vehicleName)
            % SETPRESETLENSSETTINGS Set preset lens settings for a specific sensor
            %
            % Description:
            %   Sets preset lens settings (e.g., focal length) for a specified sensor via the AirSim API.
            %
            % Inputs:
            %   preset_lens_settings - String representing the preset lens settings to set.
            %   sensorName - Name of the sensor to set settings for.
            %   vehicleName - name of the vehicle.   
            %
            % Outputs:
            %   settings - String representing the updated preset lens settings.

            returnData = obj.rpc_client.call("simSetPresetLensSettings", preset_lens_settings, sensorName, vehicleName);
            settings = string(returnData);
        end

        function settings = getPresetFilmbackSettings(obj, sensorName, vehicleName)
            % GETPRESETFILMBACKSETTINGS Get preset filmback settings for a specific sensor
            %
            % Description:
            %   Retrieves preset filmback settings (e.g., sensor dimensions) for a specified sensor from the AirSim API.
            %
            % Inputs:
            %   sensorName - Name of the sensor to retrieve settings for.
            %   vehicleName - name of the vehicle.   
            %
            % Outputs:
            %   settings - String representing the preset filmback settings.

            returnData = obj.rpc_client.call("simGetPresetFilmbackSettings", sensorName, vehicleName);
            settings = string(cell(returnData));
        end

        function setPresetFilmbackSettings(obj, preset_filmback_settings, sensorName, vehicleName)
            % SETPRESETFILMBACKSETTINGS Set preset filmback settings for a specific sensor
            %
            % Description:
            %   Sets preset filmback settings (e.g., sensor dimensions) for a specified sensor via the AirSim API.
            %
            % Inputs:
            %   preset_filmback_settings - String representing the preset filmback settings to set.
            %   sensorName - Name of the sensor to set settings for.
            %   vehicleName - name of the vehicle.   

            obj.rpc_client.call("simGetPresetFilmbackSettings", preset_filmback_settings, sensorName, vehicleName);
        end

        function settings = getFilmbackSettings(obj, sensorName, vehicleName)
            % GETFILMBACKSETTINGS Get current filmback settings for a specific sensor
            %
            % Description:
            %   Retrieves current filmback settings (e.g., sensor dimensions) for a specified sensor from the AirSim API.
            %
            % Inputs:
            %   sensorName - Name of the sensor to retrieve settings for.
            %   vehicleName - name of the vehicle.   
            %
            % Outputs:
            %   settings - String representing the current filmback settings.

            returnData = obj.rpc_client.call("simGetFilmbackSettings", sensorName, vehicleName);
            settings = string(returnData);
        end

        function setFilmbackSettings(obj, sensor_width, sensor_height, sensorName, vehicleName)
            % SETFILMBACKSETTINGS Set filmback settings for a specific sensor
            %
            % Description:
            %   Sets filmback settings (e.g., sensor dimensions) for a specified sensor via the AirSim API.
            %
            % Inputs:
            %   sensor_width - Width of the sensor.
            %   sensor_height - Height of the sensor.
            %   sensorName - Name of the sensor to set settings for.
            %   vehicleName - name of the vehicle.   

            obj.rpc_client.call("simSetFilmbackSettings", sensor_width, sensor_height, sensorName, vehicleName);
        end

        function settings = getFocalLength(obj, sensorName, vehicleName)
            % GETFOCALLENGTH Get current focal length for a specific sensor
            %
            % Description:
            %   Retrieves current focal length for a specified sensor from the AirSim API.
            %
            % Inputs:
            %   sensorName - Name of the sensor to retrieve focal length for.
            %   vehicleName - name of the vehicle.   
            %
            % Outputs:
            %   settings - Double representing the current focal length.

            returnData = obj.rpc_client.call("simGetFocalLength", sensorName, vehicleName);
            settings = double(returnData);
        end

        function setFocalLength(obj, focal_length, sensorName, vehicleName)
            % SETFOCALLENGTH Set focal length for a specific sensor
            %
            % Description:
            %   Sets focal length for a specified sensor via the AirSim API.
            %
            % Inputs:
            %   focal_length - Focal length value to set.
            %   sensorName - Name of the sensor to set focal length for.
            %   vehicleName - name of the vehicle.   

            obj.rpc_client.call("simSetFocalLength", focal_length, sensorName, vehicleName);
        end

        function enableManualFocus(obj, enable, sensorName, vehicleName)
            % ENABLEMANUALFOCUS Enable or disable manual focus for a specific sensor
            %
            % Description:
            %   Enables or disables manual focus for a specified sensor via the AirSim API.
            %
            % Inputs:
            %   enable - Logical value indicating whether to enable (true) or disable (false) manual focus.
            %   sensorName - Name of the sensor to enable/disable manual focus for.
            %   vehicleName - name of the vehicle.   

            obj.rpc_client.call("simEnableManualFocus", enable, sensorName, vehicleName);
        end

        function settings = getFocusDistance(obj, sensorName, vehicleName)
            % GETFOCUSDISTANCE Get current focus distance for a specific sensor
            %
            % Description:
            %   Retrieves current focus distance for a specified sensor from the AirSim API.
            %
            % Inputs:
            %   sensorName - Name of the sensor to retrieve focus distance for.
            %   vehicleName - name of the vehicle.   
            %
            % Outputs:
            %   settings - Double representing the current focus distance.

            returnData = obj.rpc_client.call("simGetFocusDistance", sensorName, vehicleName);
            settings = double(returnData);
        end

        function setFocusDistance(obj, focus_distance, sensorName, vehicleName)
            % SETFOCUSDISTANCE Set focus distance for a specific sensor
            %
            % Description:
            %   Sets focus distance for a specified sensor via the AirSim API.
            %
            % Inputs:
            %   focus_distance - Focus distance value to set.
            %   sensorName - Name of the sensor to set focus distance for.
            %   vehicleName - name of the vehicle.   

            obj.rpc_client.call("simSetFocusDistance", focus_distance, sensorName, vehicleName);
        end

        function settings = getFocusAperture(obj, sensorName, vehicleName)
            % GETFOCUSAPERTURE Get current focus aperture for a specific sensor
            %
            % Description:
            %   Retrieves current focus aperture for a specified sensor from the AirSim API.
            %
            % Inputs:
            %   sensorName - Name of the sensor to retrieve focus aperture for.
            %   vehicleName - name of the vehicle.   
            %
            % Outputs:
            %   settings - Double representing the current focus aperture.

            returnData = obj.rpc_client.call("simGetFocusAperture", sensorName, vehicleName);
            settings = double(returnData);
        end

        function setFocusAperture(obj, focus_aperture, sensorName, vehicleName)
            % SETFOCUSAPERTURE Set focus aperture for a specific sensor
            %
            % Description:
            %   Sets focus aperture for a specified sensor via the AirSim API.
            %
            % Inputs:
            %   focus_aperture - Focus aperture value to set.
            %   sensorName - Name of the sensor to set focus aperture for.
            %   vehicleName - name of the vehicle.   

            obj.rpc_client.call("simSetFocusAperture", focus_aperture, sensorName, vehicleName);
        end

        function enableFocusPlane(obj, enable, sensorName, vehicleName)
            % ENABLEFOCUSPLANE Enable or disable focus plane for a specific sensor
            %
            % Description:
            %   Enables or disables focus plane for a specified sensor via the AirSim API.
            %
            % Inputs:
            %   enable - Logical value indicating whether to enable (true) or disable (false) focus plane.
            %   sensorName - Name of the sensor to enable/disable focus plane for.
            %   vehicleName - name of the vehicle.   

            obj.rpc_client.call("simEnableFocusPlane", enable, sensorName, vehicleName);
        end

        function settings = getCurrentFieldOfView(obj, sensorName, vehicleName)
            % GETCURRENTFIELDOFVIEW Get current field of view for a specific sensor
            %
            % Description:
            %   Retrieves current field of view for a specified sensor from the AirSim API.
            %
            % Inputs:
            %   sensorName - Name of the sensor to retrieve field of view for.
            %   vehicleName - name of the vehicle.   
            %
            % Outputs:
            %   settings - String representing the current field of view.

            returnData = obj.rpc_client.call("simGetCurrentFieldOfView", sensorName, vehicleName);
            settings = string(returnData);
        end 

        function takeoffAsync(obj, vehicleName, t, join)
            % TAKEOFFASYNC Initiate asynchronous takeoff of the vehicle
            %
            % Description:
            %   Initiates asynchronous takeoff of the vehicle with optional timeout.
            %
            % Inputs:
            %   vehicleName - name of the vehicle.   
            %   t - timeout value.
            %   join - toggle on to wait for the method to finish. 

            if join
                obj.drone_client.takeoffAsync(t, vehicleName).join();
            else
                obj.drone_client.takeoffAsync(t, vehicleName);
            end
        end

        function landAsync(obj, vehicleName, t, join)
            % LANDASYNC Initiate asynchronous landing of the vehicle
            %
            % Description:
            %   Initiates asynchronous landing of the vehicle with optional timeout.
            %
            % Inputs:
            %   vehicleName - name of the vehicle. 
            %   t - timeout value.  
            %   join - toggle on to wait for the method to finish. 

            if join
                obj.drone_client.landAsync(t, vehicleName).join();
            else
                obj.drone_client.landAsync(t, vehicleName);
            end
        end        

        function goHomeAsync(obj, vehicleName, t, join)
            % GOHOMEASYNC Initiate asynchronous return to home of the vehicle
            %
            % Description:
            %   Initiates asynchronous return to home of the vehicle with optional timeout.
            %
            % Inputs:
            %   vehicleName - name of the vehicle.   
            %   t - timeout value.  
            %   join - toggle on to wait for the method to finish. 

            if join
                obj.drone_client.goHomeAsync(t, vehicleName).join();
            else
                obj.drone_client.goHomeAsync(t, vehicleName);
            end
        end

        function moveByVelocityZBodyFrameAsync(obj, vx, vy, z, duration, drivetrain, yaw_mode_is_rate, yaw_mode_yaw_or_rate, vehicleName, join)
            % MOVEBYVELOCITYZBODYFRAMEASYNC Move the vehicle by velocity and z in body frame asynchronously
            %
            % Description:
            %     Moves the vehicle by velocity and z in the body frame asynchronously.
            %
            % Inputs:
            %     vx, vy - Velocity components in body frame.
            %     z - Z position to move to.
            %     duration - Duration of the movement.
            %     drivetrain - Drivetrain type (Use AirSimDrivetrainTypes enum)
            %     yaw_mode_is_rate - Logical value indicating if yaw mode is rate.
            %     yaw_mode_yaw_or_rate - Yaw or rate value depending on mode.
            %     vehicleName - name of the vehicle.   
            %     join - toggle on to wait for the method to finish. 
        
            yawMode.is_rate = yaw_mode_is_rate;
            yawMode.yaw_or_rate = yaw_mode_yaw_or_rate;
            
            if join
                obj.drone_client.moveByVelocityZBodyFrameAsync(vx, vy, z, duration, int32(drivetrain), yawMode, vehicleName).join();
            else
                obj.drone_client.moveByVelocityZBodyFrameAsync(vx, vy, z, duration, int32(drivetrain), yawMode, vehicleName);
            end
        end

        function moveByVelocityAsync(obj, vx, vy, vz, duration, drivetrain, yaw_mode_is_rate, yaw_mode_yaw_or_rate, vehicleName, join)
            % MOVEBYVELOCITYASYNC Move the vehicle by velocity asynchronously
            %
            % Description:
            %     Moves the vehicle by velocity asynchronously.
            %
            % Inputs:
            %     vx, vy, vz - Velocity components.
            %     duration - Duration of the movement.
            %     drivetrain - Drivetrain type (Use AirSimDrivetrainTypes enum)
            %     yaw_mode_is_rate - Logical value indicating if yaw mode is rate.
            %     yaw_mode_yaw_or_rate - Yaw or rate value depending on mode.
            %     vehicleName - name of the vehicle.   
            %     join - toggle on to wait for the method to finish. 
        
            yawMode.is_rate = yaw_mode_is_rate;
            yawMode.yaw_or_rate = yaw_mode_yaw_or_rate;
            
            if join
                obj.drone_client.moveByVelocityAsync(vx, vy, vz, duration, int32(drivetrain), yawMode, vehicleName).join();
            else
                obj.drone_client.moveByVelocityAsync(vx, vy, vz, duration, int32(drivetrain), yawMode, vehicleName);
            end
        end

        function moveByVelocityZAsync(obj, vx, vy, z, duration, drivetrain, yaw_mode_is_rate, yaw_mode_yaw_or_rate, vehicleName, join)
            % MOVEBYVELOCITYZASYNC Move the vehicle by velocity and z asynchronously
            %
            % Description:
            %     Moves the vehicle by velocity and z asynchronously.
            %
            % Inputs:
            %     vx, vy - Velocity components.
            %     z - Z position to move to.
            %     duration - Duration of the movement.
            %     drivetrain - Drivetrain type (Use AirSimDrivetrainTypes enum)
            %     yaw_mode_is_rate - Logical value indicating if yaw mode is rate.
            %     yaw_mode_yaw_or_rate - Yaw or rate value depending on mode.
            %     vehicleName - name of the vehicle.   
            %     join - toggle on to wait for the method to finish. 
        
            yawMode.is_rate = yaw_mode_is_rate;
            yawMode.yaw_or_rate = yaw_mode_yaw_or_rate;
            
            if join
                obj.drone_client.moveByVelocityZAsync(vx, vy, z, duration, int32(drivetrain), yawMode, vehicleName).join();
            else
                obj.drone_client.moveByVelocityZAsync(vx, vy, z, duration, int32(drivetrain), yawMode, vehicleName);
            end
        end

        function moveOnPathAsync(obj, path, velocity, timeout_sec, drivetrain, yaw_mode_is_rate, yaw_mode_yaw_or_rate, lookahead, adaptive_lookahead, vehicleName, join)
            % MOVEONPATHASYNC Move the vehicle on a path asynchronously
            %
            % Description:
            %     Moves the vehicle along a specified path asynchronously.
            %
            % Inputs:
            %     path - Path to follow.
            %     velocity - Velocity of the vehicle.
            %     timeout_sec - Timeout duration in seconds.
            %     drivetrain - Drivetrain type (Use AirSimDrivetrainTypes enum)
            %     yaw_mode_is_rate - Logical value indicating if yaw mode is rate.
            %     yaw_mode_yaw_or_rate - Yaw or rate value depending on mode.
            %     lookahead - Lookahead distance.
            %     adaptive_lookahead - Adaptive lookahead distance.
            %     vehicleName - name of the vehicle.   
            %     join - toggle on to wait for the method to finish. 
        
            yawMode.is_rate = yaw_mode_is_rate;
            yawMode.yaw_or_rate = yaw_mode_yaw_or_rate;
            
            if join
                obj.drone_client.moveOnPathAsync(path, velocity, timeout_sec, int32(drivetrain), yawMode, lookahead, adaptive_lookahead, vehicleName).join();
            else
                obj.drone_client.moveOnPathAsync(path, velocity, timeout_sec, int32(drivetrain), yawMode, lookahead, adaptive_lookahead, vehicleName);
            end
        end

        function moveToPositionAsync(obj, x, y, z, velocity, timeout_sec, drivetrain, yaw_mode_is_rate, yaw_mode_yaw_or_rate, lookahead, adaptive_lookahead, vehicleName, join)
            % MOVETOPOSITIONASYNC Move the vehicle to a position asynchronously
            %
            % Description:
            %     Moves the vehicle to a specified position asynchronously.
            %
            % Inputs:
            %     x, y, z - Coordinates of the position to move to.
            %     velocity - Velocity of the vehicle.
            %     timeout_sec - Timeout duration in seconds.
            %     drivetrain - Drivetrain type (Use AirSimDrivetrainTypes enum)
            %     yaw_mode_is_rate - Logical value indicating if yaw mode is rate.
            %     yaw_mode_yaw_or_rate - Yaw or rate value depending on mode.
            %     lookahead - Lookahead distance.
            %     adaptive_lookahead - Adaptive lookahead distance.
            %     vehicleName - name of the vehicle.   
            %     join - toggle on to wait for the method to finish. 
        
            yawMode.is_rate = yaw_mode_is_rate;
            yawMode.yaw_or_rate = yaw_mode_yaw_or_rate;
            
            if join
                obj.drone_client.moveToPositionAsync(x, y, z, velocity, timeout_sec, int32(drivetrain), yawMode, lookahead, adaptive_lookahead, vehicleName).join();
            else
                obj.drone_client.moveToPositionAsync(x, y, z, velocity, timeout_sec, int32(drivetrain), yawMode, lookahead, adaptive_lookahead, vehicleName);
            end
        end

        function moveToGPSAsync(obj, latitude, longitude, altitude, velocity, timeout_sec, drivetrain, yaw_mode_is_rate, yaw_mode_yaw_or_rate, lookahead, adaptive_lookahead, vehicleName, join)
            % MOVETOGPSASYNC Move the vehicle to a GPS location asynchronously
            %
            % Description:
            %   Moves the vehicle to a specified GPS location asynchronously.
            %
            % Inputs:
            %   latitude, longitude, altitude - GPS coordinates.
            %   velocity - Velocity of the vehicle.
            %   timeout_sec - Timeout duration in seconds.
            %   drivetrain - Drivetrain type (Use AirSimDrivetrainTypes enum)
            %   yaw_mode_is_rate - Logical value indicating if yaw mode is rate.
            %   yaw_mode_yaw_or_rate - Yaw or rate value depending on mode.
            %   lookahead - Lookahead distance.
            %   adaptive_lookahead - Adaptive lookahead distance.
            %   vehicleName - name of the vehicle.   
            %   join - toggle on to wait for the method to finish. 

            yawMode.is_rate = yaw_mode_is_rate;
            yawMode.yaw_or_rate = yaw_mode_yaw_or_rate;
            
            if join
                obj.drone_client.moveToGPSAsync(latitude, longitude, altitude, velocity, timeout_sec, int32(drivetrain), yawMode, lookahead, adaptive_lookahead, vehicleName).join();
            else
                obj.drone_client.moveToGPSAsync(latitude, longitude, altitude, velocity, timeout_sec, int32(drivetrain), yawMode, lookahead, adaptive_lookahead, vehicleName);
            end
        end

        function moveToZAsync(obj, z, velocity, timeout_sec, drivetrain, yaw_mode_is_rate, yaw_mode_yaw_or_rate, lookahead, adaptive_lookahead, vehicleName, join)
            % MOVETOZASYNC Move the vehicle to a specific Z position asynchronously
            %
            % Description:
            %   Moves the vehicle to a specified Z position asynchronously.
            %
            % Inputs:
            %   z - Z position to move to.
            %   velocity - Velocity of the vehicle.
            %   timeout_sec - Timeout duration in seconds.
            %   drivetrain - Drivetrain type (Use AirSimDrivetrainTypes enum)
            %   yaw_mode_is_rate - Logical value indicating if yaw mode is rate.
            %   yaw_mode_yaw_or_rate - Yaw or rate value depending on mode.
            %   lookahead - Lookahead distance.
            %   adaptive_lookahead - Adaptive lookahead distance.
            %   vehicleName - name of the vehicle.   
            %   join - toggle on to wait for the method to finish. 

            yawMode.is_rate = yaw_mode_is_rate;
            yawMode.yaw_or_rate = yaw_mode_yaw_or_rate;

            if join
                obj.drone_client.moveToZAsync(z, velocity, timeout_sec, int32(drivetrain), yawMode, lookahead, adaptive_lookahead, vehicleName).join();
            else
                obj.drone_client.moveToZAsync(z, velocity, timeout_sec, int32(drivetrain), yawMode, lookahead, adaptive_lookahead, vehicleName);
            end
        end

        function moveByManualAsync(obj, vx_max, vy_max, z_min, duration, drivetrain, yaw_mode_is_rate, yaw_mode_yaw_or_rate, vehicleName)
            % MOVEBYMANUALASYNC Move the vehicle manually by specifying max velocities and minimum Z
            %
            % Description:
            %   Moves the vehicle manually by specifying maximum velocities and minimum Z position.
            %
            % Inputs:
            %   vx_max, vy_max - Maximum velocities in x and y directions.
            %   z_min - Minimum Z position.
            %   duration - Duration of the movement.
            %   drivetrain - Drivetrain type (Use AirSimDrivetrainTypes enum)
            %   yaw_mode_is_rate - Logical value indicating if yaw mode is rate.
            %   yaw_mode_yaw_or_rate - Yaw or rate value depending on mode.
            %   vehicleName - name of the vehicle.   

            yawMode.is_rate = yaw_mode_is_rate;
            yawMode.yaw_or_rate = yaw_mode_yaw_or_rate;
            if join
                obj.drone_client.moveByManualAsync(vx_max, vy_max, z_min, duration, int32(drivetrain), yawMode, vehicleName).join();
            else
                obj.drone_client.moveByManualAsync(vx_max, vy_max, z_min, duration, int32(drivetrain), yawMode, vehicleName);
            end
        end

        function rotateToYawAsync(obj, yaw, timeout_sec, margin, vehicleName, join)
            % ROTATETOYAWASYNC Rotate the vehicle to a specified yaw asynchronously
            %
            % Description:
            %     Rotates the vehicle to a specified yaw asynchronously.
            %
            % Inputs:
            %     yaw - Yaw angle to rotate to.
            %     timeout_sec - Timeout duration in seconds.
            %     margin - Margin for error in rotation.
            %     vehicleName - name of the vehicle.   
            %     join - toggle on to wait for the method to finish. 
        
            if join
                obj.drone_client.rotateToYawAsync(yaw, timeout_sec, margin, vehicleName).join();
            else
                obj.drone_client.rotateToYawAsync(yaw, timeout_sec, margin, vehicleName);
            end
        end
        
        function rotateByYawRateAsync(obj, yaw_rate, duration, vehicleName, join)
            % ROTATEBYYAWRATEASYNC Rotate the vehicle by a specified yaw rate asynchronously
            %
            % Description:
            %     Rotates the vehicle by a specified yaw rate asynchronously.
            %
            % Inputs:
            %     yaw_rate - Yaw rate to rotate by.
            %     duration - Duration of the rotation.
            %     vehicleName - name of the vehicle.   
            %     join - toggle on to wait for the method to finish. 
        
            if join
                obj.drone_client.rotateByYawRateAsync(yaw_rate, duration, vehicleName).join();
            else
                obj.drone_client.rotateByYawRateAsync(yaw_rate, duration, vehicleName);
            end
        end
        
        function hoverAsync(obj, vehicleName, join)
            % HOVERASYNC Make the vehicle hover at its current position asynchronously
            %
            % Inputs:
            %     vehicleName - name of the vehicle.   
            %     join - toggle on to wait for the method to finish. 
            %
            % Description:
            %     Makes the vehicle hover at its current position asynchronously.
        
            if join
                obj.drone_client.hoverAsync(vehicleName).join();
            else
                obj.drone_client.hoverAsync(vehicleName);
            end
        end
        
        function moveByMotorPWMsAsync(obj, front_right_pwm, rear_left_pwm, front_left_pwm, rear_right_pwm, duration, vehicleName, join)
            % MOVEBYMOTORPWMSASYNC Move the vehicle by specifying motor PWM values asynchronously
            %
            % Description:
            %     Moves the vehicle by specifying motor PWM values asynchronously.
            %
            % Inputs:
            %     front_right_pwm, rear_left_pwm, front_left_pwm, rear_right_pwm - PWM values for each motor.
            %     duration - Duration of the movement.
            %     vehicleName - name of the vehicle.   
            %     join - toggle on to wait for the method to finish. 
        
            if join
                obj.drone_client.moveByMotorPWMsAsync(front_right_pwm, rear_left_pwm, front_left_pwm, rear_right_pwm, duration, vehicleName).join();
            else
                obj.drone_client.moveByMotorPWMsAsync(front_right_pwm, rear_left_pwm, front_left_pwm, rear_right_pwm, duration, vehicleName);
            end
        end
        
        function moveByRollPitchYawZAsync(obj, roll, pitch, yaw, z, duration, vehicleName, join)
            % MOVEBYROLLPITCHYAWZASYNC Move the vehicle by specifying roll, pitch, yaw, and Z position asynchronously
            %
            % Description:
            %     Moves the vehicle by specifying roll, pitch, yaw, and Z position asynchronously.
            %
            % Inputs:
            %     roll, pitch, yaw - Roll, pitch, and yaw angles.
            %     z - Z position to move to.
            %     duration - Duration of the movement.
            %     vehicleName - name of the vehicle.   
            %     join - toggle on to wait for the method to finish. 
        
            if join
                obj.drone_client.moveByRollPitchYawZAsync(roll, -pitch, -yaw, z, duration, vehicleName).join();
            else
                obj.drone_client.moveByRollPitchYawZAsync(roll, -pitch, -yaw, z, duration, vehicleName);
            end
        end
        
        function moveByRollPitchYawThrottleAsync(obj, roll, pitch, yaw, throttle, duration, vehicleName, join)
            % MOVEBYROLLPITCHYAWTHROTTLEASYNC Move the vehicle by specifying roll, pitch, yaw, and throttle asynchronously
            %
            % Description:
            %     Moves the vehicle by specifying roll, pitch, yaw, and throttle asynchronously.
            %
            % Inputs:
            %     roll, pitch, yaw - Roll, pitch, and yaw angles.
            %     throttle - Throttle value.
            %     duration - Duration of the movement.
            %     vehicleName - name of the vehicle.   
            %     join - toggle on to wait for the method to finish. 
        
            if join
                obj.drone_client.moveByRollPitchYawThrottleAsync(roll, -pitch, -yaw, throttle, duration, vehicleName).join();
            else
                obj.drone_client.moveByRollPitchYawThrottleAsync(roll, -pitch, -yaw, throttle, duration, vehicleName);
            end
        end
        
        function moveByRollPitchYawrateThrottleAsync(obj, roll, pitch, yaw_rate, throttle, duration, vehicleName, join)
            % MOVEBYROLLPITCHYAWTHROTTLEASYNC Move the vehicle by specifying roll, pitch, yaw, and throttle asynchronously
            %
            % Description:
            %     Moves the vehicle by specifying roll, pitch, yaw, and throttle asynchronously.
            %
            % Inputs:
            %     roll, pitch, yaw - Roll, pitch, and yaw angles.
            %     throttle - Throttle value.
            %     duration - Duration of the movement.
            %     vehicleName - name of the vehicle.   
            %     join - toggle on to wait for the method to finish. 
        
            if join
                obj.drone_client.moveByRollPitchYawrateThrottleAsync(roll, -pitch, -yaw_rate, throttle, duration, vehicleName).join();
            else
                obj.drone_client.moveByRollPitchYawrateThrottleAsync(roll, -pitch, -yaw_rate, throttle, duration, vehicleName);
            end
        end
        
        function moveByRollPitchYawrateZAsync(obj, roll, pitch, yaw_rate, z, duration, vehicleName, join)
            % MOVEBYROLLPITCHYAWRATEZASYNC Move the vehicle by specifying roll, pitch, yaw rate, and Z position asynchronously
            %
            % Description:
            %     Moves the vehicle by specifying roll, pitch, yaw rate, and Z position asynchronously.
            %
            % Inputs:
            %     roll, pitch - Roll and pitch angles.
            %     yaw_rate - Yaw rate.
            %     z - Z position to move to.
            %     duration - Duration of the movement.
            %     vehicleName - name of the vehicle.   
            %     join - toggle on to wait for the method to finish. 
        
            if join
                obj.drone_client.moveByRollPitchYawrateZAsync(roll, -pitch, -yaw_rate, z, duration, vehicleName).join();
            else
                obj.drone_client.moveByRollPitchYawrateZAsync(roll, -pitch, -yaw_rate, z, duration, vehicleName);
            end
        end

        function moveByAngleRatesZAsync(obj, roll_rate, pitch_rate, yaw_rate, z, duration, vehicleName, join)
            % MOVEBYANGLERATESZASYNC Move the vehicle by specifying roll, pitch, yaw rates, and Z position asynchronously
            %
            % Description:
            %     Moves the vehicle by specifying roll, pitch, yaw rates, and Z position asynchronously.
            %
            % Inputs:
            %     roll_rate, pitch_rate, yaw_rate - Roll, pitch, and yaw rates.
            %     z - Z position to move to.
            %     duration - Duration of the movement.
            %     vehicleName - name of the vehicle.   
            %     join - toggle on to wait for the method to finish. 
        
            if join
                obj.drone_client.moveByAngleRatesZAsync(roll_rate, -pitch_rate, -yaw_rate, z, duration, vehicleName).join();
            else
                obj.drone_client.moveByAngleRatesZAsync(roll_rate, -pitch_rate, -yaw_rate, z, duration, vehicleName);
            end
        end

        function moveByAngleRatesThrottleAsync(obj, roll_rate, pitch_rate, yaw_rate, throttle, duration, vehicleName, join)
            % MOVEBYANGLERATESTHROTTLEASYNC Move the vehicle by specifying roll, pitch, yaw rates, and throttle asynchronously
            %
            % Description:
            %   Moves the vehicle by specifying roll, pitch, yaw rates, and throttle asynchronously.
            %
            % Inputs:
            %   roll_rate, pitch_rate, yaw_rate - Roll, pitch, and yaw rates.
            %   throttle - Throttle value.
            %   duration - Duration of the movement.
            %   vehicleName - name of the vehicle.   
            %   join - toggle on to wait for the method to finish. 

            if join
                obj.drone_client.moveByAngleRatesThrottleAsync(roll_rate, -pitch_rate, -yaw_rate, throttle, duration, vehicleName).join();
            else
                obj.drone_client.moveByAngleRatesThrottleAsync(roll_rate, -pitch_rate, -yaw_rate, throttle, duration, vehicleName);
            end
        end
    end
end

