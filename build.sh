#!/usr/bin/env bash

# get path of current script: https://stackoverflow.com/a/39340259/207661
SCRIPT_DIR="$( cd "$( dirname "${BASH_SOURCE[0]}" )" && pwd )"
pushd "$SCRIPT_DIR"  >/dev/null

set -e
set -x

debug=false
gcc=false
ue_root="${UE_ROOT:-}"
# Parse command line arguments
while [[ $# -gt 0 ]]
do
    key="$1"

    case $key in
    --debug)
        debug=true
        shift # past argument
        ;;
    --gcc)
        gcc=true
        shift # past argument
        ;;
    --ue-root)
        ue_root="$2"
        shift # past argument
        shift # past value
        ;;
    esac

done

if [[ -n "$ue_root" && $gcc == true ]]; then
    echo "ERROR: --ue-root and --gcc are mutually exclusive (Unreal Engine's bundled toolchain is Clang-only)."
    exit 1
fi

function version_less_than_equal_to() { test "$(printf '%s\n' "$@" | sort -V | head -n 1)" = "$1"; }

# check for rpclib
RPC_VERSION_FOLDER="rpclib-2.3.1"
if [ ! -d "./external/rpclib/$RPC_VERSION_FOLDER" ]; then
    echo "ERROR: new version of AirSim requires newer rpclib."
    echo "please run setup.sh first and then run build.sh again."
    exit 1
fi

# check for local cmake build created by setup.sh
if [ -d "./cmake_build" ]; then
    if [ "$(uname)" == "Darwin" ]; then
        CMAKE="$(greadlink -f cmake_build/bin/cmake)"
    else
        CMAKE="$(readlink -f cmake_build/bin/cmake)"
    fi
else
    CMAKE=$(which cmake)
fi

# variable for build output
if $debug; then
    build_dir=build_debug
else
    build_dir=build_release
fi 
if [ "$(uname)" == "Darwin" ]; then
    # llvm v8 is too old for Big Sur see
    # https://github.com/microsoft/AirSim/issues/3691
    #export CC=/usr/local/opt/llvm@8/bin/clang
    #export CXX=/usr/local/opt/llvm@8/bin/clang++
    #now pick up whatever setup.sh installs
    export CC="$(brew --prefix)/opt/llvm/bin/clang"
    export CXX="$(brew --prefix)/opt/llvm/bin/clang++"
elif [[ -n "$ue_root" ]]; then
    # Unreal Engine links its Linux targets with its own bundled Clang + sysroot, not the
    # system compiler/libc. Building AirLib/rpclib with the matching toolchain here avoids
    # ABI mismatches (e.g. "undefined symbol: __isoc23_strtol") that show up when a host with
    # a newer glibc (Ubuntu 24.04+, glibc >= 2.38) builds against UE's older bundled sysroot.
    ue_toolchain_dirs=("$ue_root"/Engine/Extras/ThirdPartyNotUE/SDKs/HostLinux/Linux_x64/*/x86_64-unknown-linux-gnu)
    if [[ ! -e "${ue_toolchain_dirs[0]}" ]]; then
        echo "ERROR: could not find Unreal Engine's bundled Linux toolchain under:"
        echo "  $ue_root/Engine/Extras/ThirdPartyNotUE/SDKs/HostLinux/Linux_x64/*/x86_64-unknown-linux-gnu"
        echo "Check that --ue-root points at a valid Unreal Engine install root."
        exit 1
    fi
    if [[ ${#ue_toolchain_dirs[@]} -gt 1 ]]; then
        echo "ERROR: found multiple Unreal Engine bundled Linux toolchains under $ue_root, expected one:"
        printf '  %s\n' "${ue_toolchain_dirs[@]}"
        exit 1
    fi
    UE_TOOLCHAIN_DIR="${ue_toolchain_dirs[0]}"
    if [[ ! -x "$UE_TOOLCHAIN_DIR/bin/clang++" ]]; then
        echo "ERROR: $UE_TOOLCHAIN_DIR/bin/clang++ not found or not executable."
        exit 1
    fi
    echo "Using Unreal Engine's bundled Linux toolchain at $UE_TOOLCHAIN_DIR"
    export CC="$UE_TOOLCHAIN_DIR/bin/clang"
    export CXX="$UE_TOOLCHAIN_DIR/bin/clang++"
    CMAKE_VARS="$CMAKE_VARS -DUSING_UE_TOOLCHAIN=ON -DCMAKE_CXX_FLAGS=--sysroot=$UE_TOOLCHAIN_DIR -DCMAKE_C_FLAGS=--sysroot=$UE_TOOLCHAIN_DIR -DCMAKE_EXE_LINKER_FLAGS=--sysroot=$UE_TOOLCHAIN_DIR -DCMAKE_SHARED_LINKER_FLAGS=--sysroot=$UE_TOOLCHAIN_DIR"
else
    if $gcc; then
        export CC="gcc"
        export CXX="g++"
    else
        export CC="clang"
        export CXX="clang++"
    fi
fi

#install EIGEN library
if [[ ! -d "./AirLib/deps/eigen3/Eigen" ]]; then
    echo "### Eigen is not installed. Please run setup.sh first."
    exit 1
fi

echo "putting build in $build_dir folder, to clean, just delete the directory..."

# this ensures the cmake files will be built in our $build_dir instead.
if [[ -f "./cmake/CMakeCache.txt" ]]; then
    rm "./cmake/CMakeCache.txt"
fi
if [[ -d "./cmake/CMakeFiles" ]]; then
    rm -rf "./cmake/CMakeFiles"
fi



if [[ ! -d $build_dir ]]; then
    mkdir -p $build_dir
fi

pushd $build_dir  >/dev/null
if $debug; then
    folder_name="Debug"
    "$CMAKE" ../cmake -DCMAKE_BUILD_TYPE=Debug $CMAKE_VARS \
        || (popd && rm -r $build_dir && exit 1)   
else
    folder_name="Release"
    "$CMAKE" ../cmake -DCMAKE_BUILD_TYPE=Release $CMAKE_VARS \
        || (popd && rm -r $build_dir && exit 1)
fi
popd >/dev/null


pushd $build_dir  >/dev/null
# final linking of the binaries can fail due to a missing libc++abi library
# (happens on Fedora, see https://bugzilla.redhat.com/show_bug.cgi?id=1332306).
# So we only build the libraries here for now
make -lc++fs -j"$(nproc)"
popd >/dev/null

mkdir -p AirLib/lib/x64/$folder_name
mkdir -p AirLib/deps/rpclib/lib
mkdir -p AirLib/deps/MavLinkCom/lib
cp $build_dir/output/lib/libAirLib.a AirLib/lib
cp $build_dir/output/lib/libMavLinkCom.a AirLib/deps/MavLinkCom/lib
cp $build_dir/output/lib/librpc.a AirLib/deps/rpclib/lib/librpc.a

# Update AirLib/lib, AirLib/deps staging folders with new binaries
<<<<<<< HEAD
rsync -rl --delete $build_dir/output/lib/ AirLib/lib/x64/$folder_name
rsync -rl --delete external/rpclib/$RPC_VERSION_FOLDER/include AirLib/deps/rpclib
rsync -rl --delete MavLinkCom/include AirLib/deps/MavLinkCom
=======
rsync -rlt --delete $build_dir/output/lib/ AirLib/lib/x64/$folder_name
rsync -rlt --delete external/rpclib/$RPC_VERSION_FOLDER/include AirLib/deps/rpclib
rsync -rlt --delete MavLinkCom/include AirLib/deps/MavLinkCom
>>>>>>> 896180faa49f42ce71cc7e5329f1cb95017c9b1a

# Stage everything into the Unreal plugin's Fab-compliant Source layout.
# AirLib's own headers ship; its .cpp sources do NOT (the module links the precompiled
# AirLib static lib instead - AIRLIB_HEADER_ONLY guards most of AirLib's own .cpp files
# down to nothing when compiled as part of the UE module anyway, so there is nothing to
# gain and only duplicate-symbol risk to lose by also shipping them).
AIRSIM_MODULE=Unreal/Plugins/AirSim/Source/AirSim
mkdir -p "$AIRSIM_MODULE/Public/AirLib" "$AIRSIM_MODULE/Lib/AirLib"
# the vendored third-party headers (json, ctpl) go to ThirdParty/AirSimVendor below instead
<<<<<<< HEAD
rsync -rl --delete --delete-excluded --exclude json.hpp --exclude ctpl_stl.h AirLib/include/ "$AIRSIM_MODULE/Public/AirLib/"
=======
rsync -rlt --delete --delete-excluded --exclude json.hpp --exclude ctpl_stl.h AirLib/include/ "$AIRSIM_MODULE/Public/AirLib/"
>>>>>>> 896180faa49f42ce71cc7e5329f1cb95017c9b1a
# the static libraries go to a Linux or Mac subfolder, so one plugin can hold the libraries of every platform
case "$(uname)" in Darwin) lib_platform=Mac ;; *) lib_platform=Linux ;; esac
mkdir -p "$AIRSIM_MODULE/Lib/AirLib/$lib_platform"
rm -f "$AIRSIM_MODULE/Lib/AirLib/libAirLib.a"
cp AirLib/lib/libAirLib.a "$AIRSIM_MODULE/Lib/AirLib/$lib_platform/libAirLib.a"

THIRDPARTY=Unreal/Plugins/AirSim/Source/ThirdParty
mkdir -p "$THIRDPARTY/AirSimEigen/Eigen" "$THIRDPARTY/AirSimRpclib" "$THIRDPARTY/AirSimMavLinkCom"
<<<<<<< HEAD
rsync -rl --delete AirLib/deps/eigen3/Eigen/ "$THIRDPARTY/AirSimEigen/Eigen/"
rsync -rl --delete AirLib/deps/rpclib/include "$THIRDPARTY/AirSimRpclib/"
mkdir -p "$THIRDPARTY/AirSimRpclib/lib/$lib_platform"
rm -f "$THIRDPARTY/AirSimRpclib/lib/librpc.a"
cp AirLib/deps/rpclib/lib/librpc.a "$THIRDPARTY/AirSimRpclib/lib/$lib_platform/librpc.a"
rsync -rl --delete AirLib/deps/MavLinkCom/include "$THIRDPARTY/AirSimMavLinkCom/"
=======
rsync -rlt --delete AirLib/deps/eigen3/Eigen/ "$THIRDPARTY/AirSimEigen/Eigen/"
rsync -rlt --delete AirLib/deps/rpclib/include "$THIRDPARTY/AirSimRpclib/"
mkdir -p "$THIRDPARTY/AirSimRpclib/lib/$lib_platform"
rm -f "$THIRDPARTY/AirSimRpclib/lib/librpc.a"
cp AirLib/deps/rpclib/lib/librpc.a "$THIRDPARTY/AirSimRpclib/lib/$lib_platform/librpc.a"
rsync -rlt --delete AirLib/deps/MavLinkCom/include "$THIRDPARTY/AirSimMavLinkCom/"
>>>>>>> 896180faa49f42ce71cc7e5329f1cb95017c9b1a
mkdir -p "$THIRDPARTY/AirSimMavLinkCom/lib/$lib_platform"
rm -f "$THIRDPARTY/AirSimMavLinkCom/lib/libMavLinkCom.a"
cp AirLib/deps/MavLinkCom/lib/libMavLinkCom.a "$THIRDPARTY/AirSimMavLinkCom/lib/$lib_platform/libMavLinkCom.a"
mkdir -p "$THIRDPARTY/AirSimVendor/include/common_utils"
cp AirLib/include/common/common_utils/json.hpp "$THIRDPARTY/AirSimVendor/include/common_utils/json.hpp"
cp AirLib/include/common/common_utils/ctpl_stl.h "$THIRDPARTY/AirSimVendor/include/ctpl_stl.h"

set +x

echo ""
echo ""
echo "=========================================="
echo " Cosys-AirSim airlib c++ plugin is built!."
echo "=========================================="
echo ""
echo "For further info see for installation see:"
echo "https://github.com/Cosys-Lab/Cosys-AirSim/tree/main/docs/install_linux.md"
echo "=================================================================="

popd >/dev/null
