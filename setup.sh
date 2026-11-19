#! /bin/bash
set -x
set -e

SCRIPT_DIR="$( cd "$( dirname "${BASH_SOURCE[0]}" )" && pwd )"
pushd "$SCRIPT_DIR" >/dev/null

downloadHighPolySuv=true
MIN_CMAKE_VERSION=3.12.0
function version_less_than_equal_to() { test "$(printf '%s\n' "$@" | sort -V | head -n 1)" = "$1"; }

# brew gives error if package is already installed
function brew_install() { brew list $1 &>/dev/null || brew install $1; }

# Parse command line arguments
while [[ $# -gt 0 ]]
do
key="$1"

case $key in
    --no-full-poly-car)
    downloadHighPolySuv=false
    shift # past value
    ;;
esac
done

# llvm tools
if [ "$(uname)" == "Darwin" ]; then # osx
    brew update
    # Update below line for newer versions
    brew install llvm
fi

if ! which cmake; then
    # CMake not installed
    cmake_ver=0
else
    cmake_ver=$(cmake --version 2>&1 | head -n1 | cut -d ' ' -f3 | awk '{print $NF}')
fi

#give user perms to access USB port - this is not needed if not using PX4 HIL
#TODO: figure out how to do below in travis
# Install additional tools, CMake if required
if [ "$(uname)" == "Darwin" ]; then # osx
    if [[ ! -z "${whoami}" ]]; then #this happens when running in travis
        sudo dseditgroup -o edit -a `whoami` -t user dialout
    fi

    # MacOS 11 has new Python env management that breaks the Python 2-to-3
    # build process. We need to make sure brew updates before attempting to
    # install, since it will update packaages
    brew update
    brew_install wget
    brew_install coreutils

    if version_less_than_equal_to $cmake_ver $MIN_CMAKE_VERSION; then
        brew install cmake  # should get cmake 3.8
    else
        echo "Already have good version of cmake: $cmake_ver"
    fi

else #linux
    if [[ ! -z "${whoami}" ]]; then #this happens when running in travis
        sudo /usr/sbin/useradd -G dialout $USER
        sudo usermod -a -G dialout $USER
    fi

    # install additional tools
    sudo apt-get install -y unzip

fi # End USB setup


# Download rpclib
if [ ! -d "external/rpclib/rpclib-2.3.1" ]; then
    echo "*********************************************************************************************"
    echo "Downloading rpclib..."
    echo "*********************************************************************************************"

    wget https://github.com/WouterJansen/rpclib/archive/refs/tags/v2.3.1.zip

    # remove previous versions
    rm -rf "external/rpclib"

    mkdir -p "external/rpclib"
    unzip -q v2.3.1.zip -d external/rpclib
    rm v2.3.1.zip
fi

# Download high-polycount SUV model (not tracked in git; lives in the CosysAirSim pack folder under */Vehicles/SUV)
SUV_CONTENT="Unreal/Plugins/AirSim/Content"
SUV_MARKER="$SUV_CONTENT/CosysAirSim/Blueprints/Vehicles/SUV/BP_SuvCarPawn.uasset"
SUV_URL="https://github.com/Cosys-Lab/Cosys-AirSim/releases/download/carassets/cosys_car_assets_v2.zip"
if $downloadHighPolySuv; then
    if [ ! -f "$SUV_MARKER" ]; then
            echo "*********************************************************************************************"
            echo "Downloading high-poly car assets.... The download is ~54MB and can take some time."
            echo "To install without this assets, re-run setup.sh with the argument --no-full-poly-car"
            echo "*********************************************************************************************"

            rm -rf "suv_download_tmp"
            mkdir -p "suv_download_tmp"
            wget -O suv_download_tmp/car_assets.zip "$SUV_URL"
            # remove stale SUV copies: the per-type SUV folders and the pre-Fab VehicleAdv/SUV location
            rm -rf "$SUV_CONTENT"/CosysAirSim/*/Vehicles/SUV "$SUV_CONTENT/VehicleAdv/SUV"
            unzip -q -o suv_download_tmp/car_assets.zip -d "$SUV_CONTENT"
            rm -rf "suv_download_tmp"
            if [ ! -f "$SUV_MARKER" ]; then
                echo "Unable to download high-polycount SUV. The DefaultCar pawn will not be available."
            fi
    fi
else
    echo "### Not downloading high-poly car asset (--no-full-poly-car). The DefaultCar pawn will not be available."
fi

echo "Installing Eigen library..."

if [ ! -d "AirLib/deps/eigen3" ]; then
    echo "Downloading Eigen..."
    wget -O eigen3.zip https://github.com/WouterJansen/eigen/archive/refs/tags/3.4.1r.zip
    unzip -q eigen3.zip -d temp_eigen
    mkdir -p AirLib/deps/eigen3
    mv temp_eigen/eigen*/Eigen AirLib/deps/eigen3
    rm -rf temp_eigen
    rm eigen3.zip
else
    echo "Eigen is already installed."
fi

popd >/dev/null

set +x
echo ""
echo "******************************************"
echo "Cosys-AirSim setup completed successfully!"
echo "******************************************"
