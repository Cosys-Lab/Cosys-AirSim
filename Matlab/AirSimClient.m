classdef AirSimClient < handle
    %AIRSIMCLIENT a Matlab client for Cosys-AirSim API
    
    properties
        rpc_client;
        ip;
        port;
        is_drone;
        vehicle_name;   
        car_controls;
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
    end
    
    methods
        function obj = AirSimClient(varargin)
            argParser = inputParser();
            argParser.addOptional("IsDrone", false, @islogical);
            argParser.addOptional("ApiControl", false, @islogical);
            argParser.addOptional("IP", "127.0.0.1", @isstring);
            argParser.addOptional("Port", 41451, @isstring);
            argParser.addOptional("VehicleName", "airsimvehicle", @isstring);
            argParser.parse(varargin{:});

            obj.is_drone = argParser.Results.IsDrone;
            obj.api_control = argParser.Results.ApiControl;
            obj.ip = argParser.Results.IP;
            obj.port = argParser.Results.Port;
            obj.vehicle_name = argParser.Results.VehicleName;            

            obj.rpc_client = AirSimClient.setupRPC(obj.ip, obj.port);
            
            if obj.api_control
                obj.car_controls = AirSimClient.getCarControls();
            end 
        end
        
        function [sensorPose, timestamp, activePointCloud, activeData, activeLabels, passivePointCloud, passiveData, passiveLabels, passiveReflectionLabels] = getEchoData(obj, sensorName, enablePassive)
            %GET_ECHO_DATA Get sensor data from an echo sensor
            %
            % The activeData and passiveData contains the attenuation and distance
            % data saved for each point of the pointcloud.
            % The reflection direction for the passive pointcloud is saved
            % in the normal datafield.  
            
            echoData = obj.rpc_client.call("getEchoData", sensorName, obj.vehicle_name);

            timestamp = double(echoData{"time_stamp"});


            % Get the sensor pose
            sensorPose.position = obj.nedToRightHandCoordinates(struct2array(struct(echoData{"pose"}{"position"})));
            sensorPose.orientation = quatinv(struct2array(struct(echoData{"pose"}{"orientation"})));
            
            % Get pointcloud data
            passivePointCloud = [];
            passiveLabels = [];
            passiveData = [];
            passiveReflectionLabels = [];
            activePointCloud = [];
            activeData = [];
            activeLabels = [];

            reflectorPointcloudRaw = cell2mat(cell(echoData{"point_cloud"}));
            reflectorPointcloudPassiveRaw = cell2mat(cell(echoData{"passive_beacons_point_cloud"}));
            
            if mod(numel(reflectorPointcloudRaw), 5) == 0 % Discard malformed point clouds
                activeLabels = string(cell(echoData{"groundtruth"}));

                reflectorPointcloudRaw = reshape(reflectorPointcloudRaw, 5, []).';
                reflectorPointcloudRaw = obj.nedToRightHandCoordinates(reflectorPointcloudRaw);
                activeData = reflectorPointcloudRaw(:, 4:5); 
                activePointCloud = pointCloud(reflectorPointcloudRaw(:, 1:3), Intensity=reflectorPointcloudRaw(:, 4)); % attenuation is also saved in the intensity channel   
            end
            
            if enablePassive && mod(numel(reflectorPointcloudPassiveRaw), 8) == 0
                allPassiveLabels = string(cell(echoData{"passive_beacons_groundtruth"}));
                passiveLabels = allPassiveLabels(2:2:length(allPassiveLabels));
                passiveReflectionLabels = allPassiveLabels(1:2:length(allPassiveLabels));
                reflectorPointcloudPassiveRaw = reshape(reflectorPointcloudPassiveRaw, 8, []).';
                reflectorPointcloudPassiveRaw = obj.nedToRightHandCoordinates(reflectorPointcloudPassiveRaw);
                passiveData = reflectorPointcloudPassiveRaw(:, 4:5); 
                passivePointCloud = pointCloud(reflectorPointcloudPassiveRaw(:, 1:3), Intensity=reflectorPointcloudPassiveRaw(:, 4), Normal=reflectorPointcloudPassiveRaw(:, 6:8));
            end               
        end       
        
        function [vehiclePose] = getVehiclePose(obj)
            % Get car pose
            vehicleState = obj.rpc_client.call("simGetVehiclePose", obj.vehicle_name);            
            vehiclePose.position = obj.nedToRightHandCoordinates(struct2array(struct(vehicleState{"position"})));
            vehiclePose.orientation = quatinv(struct2array(struct(vehicleState{"orientation"})));
        end
        
        function [vehicleState] = getVehicleState(obj)
            % Get car pose
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
    end
end



