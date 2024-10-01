#!/bin/bash
set -x

if ! which unzip; then
    sudo apt-get install unzip
fi

wget -c https://github.com/Cosys-Lab/Cosys-AirSim/releases/download/5.4-v3.1/Blocks_package_54_31.zip
unzip -q Blocks_package_54_31.zip
rm Blocks_package_54_31.zip
