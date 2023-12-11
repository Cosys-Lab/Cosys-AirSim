clear ll
my_client = AirSimClient();
vehicle_pose = my_client.getVehiclePose;
[sensorPose, timestamp, activePointCloud, activeData, activeLabels, passivePointCloud, passiveData, passiveLabels, passiveReflectionLabels] = my_client.getEchoData("echo", true);

figure;
if ~isempty(passivePointCloud)
    pcshow(passivePointCloud, color="X", MarkerSize=50);
    hold on;
    quiver3(passivePointCloud.Location(:, 1), passivePointCloud.Location(:, 2), passivePointCloud.Location(:, 3),...
        passivePointCloud.Normal(:, 1), -passivePointCloud.Normal(:, 2), passivePointCloud.Normal(:, 3), 2);
    hold off
else
    pcshow(pointCloud([0, 0, 0]));
end
title('Passive')
xlabel("X (m)")
ylabel("Y (m)")
zlabel("Z (m)")
xlim([0 10])
ylim([-10 10])
zlim([-10 10])
