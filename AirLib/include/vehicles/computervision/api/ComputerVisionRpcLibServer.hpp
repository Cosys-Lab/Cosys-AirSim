// Copyright (c) Microsoft Corporation. All rights reserved.
// Licensed under the MIT License.

#ifndef air_ComputerVisionRpcLibServer_hpp
#define air_ComputerVisionRpcLibServer_hpp

#ifndef AIRLIB_NO_RPC

#include "common/Common.hpp"
#include <functional>
#include "api/RpcLibServerBase.hpp"
#include "vehicles/computervision/api/ComputerVisionApiBase.hpp"

namespace msr
{
    namespace airlib
    {

        class ComputerVisionRpcLibServer : public RpcLibServerBase
        {
        public:
            ComputerVisionRpcLibServer(ApiProvider* api_provider, string server_address, uint16_t port = RpcLibPort);
            virtual ~ComputerVisionRpcLibServer();

        protected:
            virtual ComputerVisionApiBase* getVehicleApi(const std::string& vehicle_name) override
            {
                return static_cast<ComputerVisionApiBase*>(RpcLibServerBase::getVehicleApi(vehicle_name));
            }
        };

#endif
    }
} //namespace
#endif