%% Example two drones
% 
% This works well with the settings below:
%
% {
%     "SeeDocsAt": "https://cosys-lab.github.io/settings/",
%     "SettingsVersion": 2,
%     "ClockSpeed": 1,
%     "LocalHostIp": "127.0.0.1",
%     "ApiServerPort": 41451,
%     "RpcEnabled": true,
%     "SimMode": "Multirotor",
%     "Vehicles": {
%         "Drone1": {
%             "VehicleType": "SimpleFlight",
%             "AllowAPIAlways": true,
%             "X": 0,
%             "Y": 0,
%             "Z": 0,
%             "Yaw": 0
%         },
%         "Drone2": {
%             "VehicleType": "SimpleFlight",
%             "AllowAPIAlways": true,
%             "X": 5,
%             "Y": 0,
%             "Z": 0,
%             "Yaw": 0
%         }
%     }
% }

airSimClient = AirSimClient(IsDrone=true, IP="127.0.0.1", port=41451);

airSimClient.setEnableApiControl("Drone1");
airSimClient.setEnableApiControl("Drone2");

airSimClient.setEnableDroneArm("Drone1");
airSimClient.setEnableDroneArm("Drone2");

airSimClient.takeoffAsync("Drone1", 20, true);
airSimClient.takeoffAsync("Drone2", 20, false);

airSimClient.moveToPositionAsync(10, 10, -5, 5, 3e+38, AirSimDrivetrainTypes.MaxDegreeOfFreedom, true, 0, -1, 1, "Drone1", true);
airSimClient.moveToPositionAsync(10, 14, -5, 5, 3e+38, AirSimDrivetrainTypes.MaxDegreeOfFreedom, true, 0, -1, 1, "Drone2", true); 

airSimClient.landAsync("Drone1", 60, true);
airSimClient.landAsync("Drone2", 60, false);

airSimClient.setDisableDroneArm("Drone1");
airSimClient.setDisableDroneArm("Drone2");

airSimClient.setDisableApiControl("Drone1");
airSimClient.setDisableApiControl("Drone2");