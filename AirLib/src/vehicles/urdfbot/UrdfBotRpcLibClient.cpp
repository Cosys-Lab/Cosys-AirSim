//in header only mode, control library is not available
#ifndef AIRLIB_HEADER_ONLY
//RPC code requires C++14. If build system like Unreal doesn't support it then use compiled binaries
#ifndef AIRLIB_NO_RPC
//if using Unreal Build system then include precompiled header file first

#include "vehicles/urdfbot/api/UrdfBotRpcLibClient.hpp"

#include "common/Common.hpp"
#include "common/ClockFactory.hpp"
#include <thread>
STRICT_MODE_OFF
#ifndef RPCLIB_MSGPACK
#define RPCLIB_MSGPACK clmdep_msgpack
#endif // !RPCLIB_MSGPACK
#undef check
#ifdef nil
#undef nil
#endif // nil
#include "rpc/client.h"
#include "vehicles/urdfbot/api/UrdfBotRpcLibAdaptors.hpp"
STRICT_MODE_ON
#ifdef _MSC_VER
__pragma(warning(disable : 4239))
#endif	

namespace msr { namespace airlib {

UrdfBotRpcLibClient::UrdfBotRpcLibClient(const string& ip_address, uint16_t port, float timeout_sec)
    : RpcLibClientBase(ip_address, port, timeout_sec) {};

UrdfBotRpcLibClient::~UrdfBotRpcLibClient() {};

void UrdfBotRpcLibClient::addAngularForce(const UrdfBotApiBase::AddAngularForce& force, const std::string& vehicle_name)
{
    static_cast<rpc::client*>(getClient())->
        call("addAngularForce", msr::airlib_rpclib::UrdfBotRpcLibAdaptors::AddAngularForce(force), vehicle_name);
}

void UrdfBotRpcLibClient::addLinearForce(const UrdfBotApiBase::AddLinearForce& force, const std::string& vehicle_name)
{
    static_cast<rpc::client*>(getClient())->
        call("addLinearForce", msr::airlib_rpclib::UrdfBotRpcLibAdaptors::AddLinearForce(force), vehicle_name);
}

void UrdfBotRpcLibClient::updateForceMagnitude(const UrdfBotApiBase::UpdateForceMagnitude& update, const std::string& vehicle_name)
{
    static_cast<rpc::client*>(getClient())->
        call("updateForceMagnitude", msr::airlib_rpclib::UrdfBotRpcLibAdaptors::UpdateForceMagnitude(update), vehicle_name);
}

void UrdfBotRpcLibClient::updateControlledMotionComponentControlSignal(const UrdfBotApiBase::UrdfBotControlledMotionComponentControlSignal& updated_control_signal, const std::string& vehicle_name)
{
    static_cast<rpc::client*>(getClient())->
        call("updateControlledMotionComponentControlSignal", msr::airlib_rpclib::UrdfBotRpcLibAdaptors::UrdfBotControlledMotionComponentControlSignal(updated_control_signal), vehicle_name);
}

UrdfBotApiBase::UrdfBotState UrdfBotRpcLibClient::getUrdfBotState(const std::string& vehicle_name)
{
    return static_cast<rpc::client*>(getClient())->
        call("getUrdfBotState", vehicle_name).as<msr::airlib_rpclib::UrdfBotRpcLibAdaptors::UrdfBotState>().to();
}

}}

#endif
#endif