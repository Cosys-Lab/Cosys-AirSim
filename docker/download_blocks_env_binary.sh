#!/bin/bash
set -x

if ! which unzip; then
    sudo apt-get install unzip
fi

wget -c https://github.com/Cosys-Lab/Cosys-AirSim/releases/download/5.8-v4.0.0/Blocks_packaged_Linux_58_400.zip
unzip -q Blocks_packaged_Linux_58_400.zip -d LinuxBlocks
rm Blocks_packaged_Linux_58_400.zip
