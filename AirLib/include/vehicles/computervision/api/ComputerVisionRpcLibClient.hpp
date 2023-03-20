// Copyright (c) Microsoft Corporation. All rights reserved.
// Licensed under the MIT License.

#ifndef air_ComputerVisionRpcLibClient_hpp
#define air_ComputerVisionRpcLibClient_hpp

#include "common/Common.hpp"
#include <functional>
#include "common/CommonStructs.hpp"
#include "vehicles/computervision/api/ComputerVisionApiBase.hpp"
#include "api/RpcLibClientBase.hpp"
#include "common/ImageCaptureBase.hpp"


namespace msr { namespace airlib {

class ComputerVisionRpcLibClient : public RpcLibClientBase {
public:
    ComputerVisionRpcLibClient(const string& ip_address = "localhost", uint16_t port = RpcLibPort, float timeout_sec = 60);

    ComputerVisionApiBase::ComputerVisionState getComputerVisionState(const std::string& vehicle_name = "");

    virtual ~ComputerVisionRpcLibClient();    //required for pimpl
};

}} //namespace
#endif
