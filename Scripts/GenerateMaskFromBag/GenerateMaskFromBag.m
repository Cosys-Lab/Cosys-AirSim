% Script to generate VIO masking images of a AirSim ROS Bag.

%% General setup

% Where the .bag and .csv files are stored
input_file_path = './input';

% Where the converted bag should be stored
output_file_path = './output';

% Filename of the ROS bag file
bag_file_name = 'airsim_drone_data.bag';

% Topic names
camera_topic_name = '/airsim/camera/left/rgb/image';
segmentation_topic_name = '/airsim/camera/left/segmentation/image';
mask_topic_name = '/camera/mask/image';

% Filename of the instance segmentation groundtruth map
groundtruth_rgb_conversion_csv_file_name = 'airsim_drone_groundtruth.csv';

% Filename of the object class list
object_types_csv_file_name = 'airsim_drone_object_types.csv';

% Mask values: unknown, static, semi-dynamic and dynamic
mask_values = [0 84 168 255]; 

% Enable plotting
plot_images = 0;

%% Read ROS bag

disp('Reading ROS bag...')

% Read in rosbags & extract data
bag = rosbagreader(fullfile(input_file_path, bag_file_name));

disp('Completed reading ROS bag!')

%% Initialize

disp('Initializing...')

% read ROS messages
camera_images_select = select(bag, 'Topic', camera_topic_name);
camera_images = readMessages(camera_images_select);
segmentation_images_select = select(bag, 'Topic', segmentation_topic_name);
segmentation_images = readMessages(segmentation_images_select);
n_images = size(camera_images, 1);

% Read object types
object_types = readtable(fullfile(input_file_path, object_types_csv_file_name), 'ReadVariableNames', false);
object_states = table2array(object_types(:,2));
object_types = string(table2array(object_types(:,1)));
n_classes = size(object_types,1);

% Read and convert groundtruth map file and prepare it for easy comparision
groundtruth_map = readtable(fullfile(input_file_path, groundtruth_rgb_conversion_csv_file_name));
groundtruth_map_values = table2array(groundtruth_map(:,2:4));
groundtruth_map = convertvars(groundtruth_map, 'R', 'string');
groundtruth_map = convertvars(groundtruth_map, 'G', 'string');
groundtruth_map = convertvars(groundtruth_map, 'B', 'string');
groundtruth_map = table2array(groundtruth_map);
for label_idx = 1:size(groundtruth_map,1)
    groundtruth_map(label_idx, 2) = strjoin(groundtruth_map(label_idx, 2:4), ',');
end
groundtruth_map(:,3:4) = [];
n_labels = size(groundtruth_map, 1);
groundtruth_map(n_labels + 1,:) = ["unknown","0,0,0"];
n_labels = n_labels + 1;

% Create output folder if it doesn't exist yet
if ~exist(output_file_path, 'dir')
    mkdir(output_file_path);
end

% Check camera image size
first_camera_image = readImage(camera_images{1});

%% Generate mask images 

disp('Masking...')

parfor_progress(n_images);
if plot_images
    figure;
end

% for camera_idx = 1 : n_images
parfor(camera_idx = 1 : n_images)
    cur_mask_image = uint8(zeros(size(first_camera_image, 1) * size(first_camera_image, 2), 1));
    cur_segmentation_image = readImage(segmentation_images{camera_idx});
    if plot_images
        subplot(1, 2, 1);
        hold on;
        imshow(cur_segmentation_image);
    end
    cur_segmentation_image = reshape(cur_segmentation_image, size(first_camera_image, 1) * size(first_camera_image, 2), 3);
    [b, found_indexes] = ismember(cur_segmentation_image, groundtruth_map_values, 'rows');
    found_indexes(found_indexes==0) = n_labels;
    labels = groundtruth_map(squeeze(found_indexes)', 1);
    for class_index = 1 : n_classes
        indexes_true = startsWith(labels, object_types(class_index), 'IgnoreCase', true);
        cur_mask_image(indexes_true) = mask_values(object_states(class_index) + 1);
    end
    cur_mask_image = reshape(cur_mask_image, size(first_camera_image, 1), size(first_camera_image, 2));
    image_message = segmentation_images{camera_idx};
    image_message.Encoding = 'mono8'; 
    writeImage(image_message, cur_mask_image)
    segmentation_images{camera_idx} = image_message;
    parfor_progress;
    if plot_images
        subplot(1, 2, 2);
        imshow(cur_mask_image);    
        drawnow;
    end
end


%% Generate mask images 

disp('Write new rosbag...')
timestamps = segmentation_images_select.MessageList.Time;
bag_writer = rosbagwriter(fullfile(output_file_path, "masked_" + bag_file_name));
write(bag_writer, mask_topic_name, timestamps, segmentation_images);
delete(bag_writer)
clear bag_writer

disp('Done!')
disp(append('To merge you can run from current folder "rosrun airsim bagmerge.py ', strrep(fullfile(input_file_path, bag_file_name), "\", "/"), ' ', strrep(fullfile(output_file_path, "masked_" + bag_file_name), "\", "/"), ' -o ', strrep(fullfile(input_file_path, "merged_masked_" + bag_file_name), "\", "/"), ' -t ', mask_topic_name, '"'))