#/bin/bash
echo "Before..."
src=airsim_hyslam_benchmark_rb_0r_2020-09-18.bag
dst=airsim_hyslam_route_rb_0r_2020-09-18.bag

ls -lah $src
rosbag filter $src $dst 'topic == "/airsim/gtpose" or topic == "/airsim/gtposekimera" or topic == "/airsim/imu" or topic == "/imu0kimera" or topic == "/rosout" or topic == "/rosout_agg" or topic == "/tf"'
echo "After..."
ls -lah $dst
