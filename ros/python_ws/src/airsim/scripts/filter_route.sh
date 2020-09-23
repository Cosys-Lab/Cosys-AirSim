#/bin/bash
echo "Before..."
ls -lah airsim_hyslam_benchmark_2020-09-18*
rosbag filter airsim_hyslam_benchmark_2020-09-18.bag airsim_hyslam_benchmark_2020-09-18_filter.bag 'topic == "/airsim/gtpose" or topic == "/airsim/gtposekimera" or topic == "/airsim/imu" or topic == "/imu0kimera" or topic == "/rosout" or topic == "/rosout_agg" or topic == "/tf"'
echo "After..."
ls -lah airsim_hyslam_benchmark_2020-09-18*
