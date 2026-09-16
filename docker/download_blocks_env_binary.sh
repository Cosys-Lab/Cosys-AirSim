#!/bin/bash
set -x

if ! which unzip; then
    sudo apt-get install unzip
fi

wget -c https://github.com/Cosys-Lab/Cosys-AirSim/releases/download/5.8-v3.5.0/Blocks_packaged_Linux_58_34.zip
unzip -q Blocks_packaged_Linux_56_34.zip
rm Blocks_packaged_Linux_56_34.zip
mv Blocks_packaged_Linux_56_34 LinuxBlocks
