// Copyright (c) Microsoft Corporation. All rights reserved.
// Licensed under the MIT License.

#ifndef air_ComputerVisionRpcLibAdapators_hpp
#define air_ComputerVisionRpcLibAdapators_hpp

#include "common/Common.hpp"
#include "common/CommonStructs.hpp"
#include "api/RpcLibAdaptorsBase.hpp"
#include "common/ImageCaptureBase.hpp"
#include "vehicles/computervision/api/ComputerVisionApiBase.hpp"

#include "common/common_utils/WindowsApisCommonPre.hpp"
#include "rpc/msgpack.hpp"
#include "common/common_utils/WindowsApisCommonPost.hpp"

namespace msr { namespace airlib_rpclib {

class ComputerVisionRpcLibAdapators : public RpcLibAdaptorsBase {
public:
    
    struct ComputerVisionState {
        KinematicsState kinematics_estimated;
        uint64_t timestamp;

        MSGPACK_DEFINE_MAP(kinematics_estimated, timestamp);

        ComputerVisionState()
        {}

        ComputerVisionState(const msr::airlib::ComputerVisionApiBase::ComputerVisionState& s)
        {
          
            timestamp = s.timestamp;
            kinematics_estimated = s.kinematics_estimated;
        }
        msr::airlib::ComputerVisionApiBase::ComputerVisionState to() const
        {
            return msr::airlib::ComputerVisionApiBase::ComputerVisionState(kinematics_estimated.to(), timestamp);
        }
    };
};

}} //namespace


#endif
