% Script to generate VIO masking images of a AirSim ROS Bag.

%% General setup

% Where the .bag and .csv files are stored
input_file_path = './input';

% Where the converted bag should be stored
output_file_path = './output';

% Filename of the ROS bag file
bag_file_name = 'merged_converted_merged_airsim_drone_data_masked.bag';

% Topic names
camera_one_topic_name = '/airsim/camera/right/rgb/image';
camera_one_out_topic_name = '/airsim/camera/right/rgba/image';

%% Read ROS bag

disp('Reading ROS bag...')

% Read in rosbags & extract data
bag = rosbagreader(fullfile(input_file_path, bag_file_name));

disp('Completed reading ROS bag!')

%% Initialize

disp('Initializing...')

% read ROS messages
camera_images_one_select = select(bag, 'Topic', camera_one_topic_name);
camera_images_one = readMessages(camera_images_one_select);
n_images = size(camera_images_one, 1);

% Create output folder if it doesn't exist yet
if ~exist(output_file_path, 'dir')
    mkdir(output_file_path);
end

% Check camera image size
first_camera_image = readImage(camera_images_one{1});

%% Generate converted images 

disp('Converting...')

parfor_progress(n_images);

for camera_idx = 1 : n_images
    camera_one_image = readImage(camera_images_one{camera_idx});
    camera_one_image(:,:,4) = zeros(size(camera_one_image, 1), size(camera_one_image, 2)) + 255;

    image_one_message = camera_images_one{camera_idx};
    image_one_message.Encoding = 'rgba8'; 
    writeImage(image_one_message, camera_one_image);
    camera_images_one{camera_idx} = image_one_message;
    parfor_progress;
end


%% Generate mask images 

disp('Write new rosbag...')
timestamps = camera_images_one_select.MessageList.Time;
bag_writer = rosbagwriter(fullfile(output_file_path, "converted_" + bag_file_name));
write(bag_writer, camera_one_out_topic_name, timestamps, camera_images_one);
delete(bag_writer)
clear bag_writer

disp('Done!')
disp(append('To merge you can run from current folder "rosrun airsim bagmerge.py ', strrep(fullfile(input_file_path, bag_file_name), "\", "/"), ' ', strrep(fullfile(output_file_path, "converted_" + bag_file_name), "\", "/"), ' -o ', strrep(fullfile(output_file_path, "merged_converted_" + bag_file_name), "\", "/"), ' -t ', camera_one_out_topic_name, '"'))
