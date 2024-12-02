#!/bin/bash
set -x

if ! which unzip; then
    sudo apt-get install unzip
fi

wget -c https://github.com/Cosys-Lab/Cosys-AirSim/releases/download/5.2-v3.2/Blocks_packaged_Linux_52_32.zip
unzip -q Blocks_packaged_Linux_52_32.zip
rm Blocks_packaged_Linux_52_32.zip
