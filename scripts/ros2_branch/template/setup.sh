#!/usr/bin/env bash
# Copyright (c) Cosys-Lab, University of Antwerp. 2026
#
# Downloads the two dependencies the Cosys-AirSim ROS2 workspace needs that are not in git:
# rpclib (RPC client library) into external/rpclib and Eigen (linear algebra headers) into AirLib/deps/eigen3.
# Run it once from the root of this branch, then build the workspace in ros2/ with colcon (see README.md).

set -e

SCRIPT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
cd "$SCRIPT_DIR"

for tool in wget unzip; do
    if ! command -v "$tool" >/dev/null; then
        echo "Please install $tool first: sudo apt-get install -y $tool"
        exit 1
    fi
done

if [ ! -d "external/rpclib/rpclib-2.3.1" ]; then
    echo "Downloading rpclib 2.3.1..."
    rm -rf external/rpclib
    mkdir -p external/rpclib
    wget -q -O rpclib.zip https://github.com/WouterJansen/rpclib/archive/refs/tags/v2.3.1.zip
    unzip -q rpclib.zip -d external/rpclib
    rm rpclib.zip
else
    echo "rpclib is already downloaded."
fi

if [ ! -d "AirLib/deps/eigen3/Eigen" ]; then
    echo "Downloading Eigen 3.4.1..."
    rm -rf temp_eigen AirLib/deps/eigen3
    wget -q -O eigen3.zip https://github.com/WouterJansen/eigen/archive/refs/tags/3.4.1r.zip
    unzip -q eigen3.zip -d temp_eigen
    mkdir -p AirLib/deps/eigen3
    mv temp_eigen/eigen*/Eigen AirLib/deps/eigen3
    rm -rf temp_eigen eigen3.zip
else
    echo "Eigen is already downloaded."
fi

echo ""
echo "Setup done. Build the ROS2 workspace:"
echo "  cd ros2"
echo "  rosdep install --from-paths src -y --ignore-src"
echo "  colcon build --cmake-args -DCMAKE_BUILD_TYPE=Release"
