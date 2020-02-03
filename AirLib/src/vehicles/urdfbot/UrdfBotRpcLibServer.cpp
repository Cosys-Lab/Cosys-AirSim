//in header only mode, control library is not available
#ifndef AIRLIB_HEADER_ONLY
//RPC code requires C++14. If build system like Unreal doesn't support it then use compiled binaries
#ifndef AIRLIB_NO_RPC
//if using Unreal Build system then include precompiled header file first

#include "vehicles/urdfbot/api/UrdfBotRpcLibServer.hpp"


#include "common/Common.hpp"
STRICT_MODE_OFF
#ifndef RPCLIB_MSGPACK
#define RPCLIB_MSGPACK clmdep_msgpack
#endif // !RPCLIB_MSGPACK
#include "common/common_utils/MinWinDefines.hpp"
#undef NOUSER
//UE4 defines macro names that conflicts with msgpack library (e.g. "check")
//#undef check
#include "rpc/server.h"
#include "vehicles/urdfbot/api/UrdfBotRpcLibAdaptors.hpp"
//UE4 defines macro names that conflicts with msgpack library (e.g. "check")
#define check(expr) (static_cast<void>((expr)))
STRICT_MODE_ON


namespace msr { namespace airlib {

UrdfBotRpcLibServer::UrdfBotRpcLibServer(ApiProvider* api_provider, string server_address, uint16_t port)
    : RpcLibServerBase(api_provider, server_address, port)
{
    auto server = static_cast<rpc::server*>(getServer());

    server->
        bind("addAngularForce", [&](const msr::airlib_rpclib::UrdfBotRpcLibAdaptors::AddAngularForce& force, const std::string& vehicle_name) -> void {
        getVehicleApi(vehicle_name)->addAngularForce(force.to(), vehicle_name);
    });

    server->
        bind("addLinearForce", [&](const msr::airlib_rpclib::UrdfBotRpcLibAdaptors::AddLinearForce& force, const std::string& vehicle_name) -> void {
        getVehicleApi(vehicle_name)->addLinearForce(force.to(), vehicle_name);
    });

    server->
        bind("updateForceMagnitude", [&](const msr::airlib_rpclib::UrdfBotRpcLibAdaptors::UpdateForceMagnitude& update, const std::string& vehicle_name) -> void {
        getVehicleApi(vehicle_name)->updateForceMagnitude(update.to(), vehicle_name);
    });

    server->
        bind("getUrdfBotState", [&](const std::string& vehicle_name) -> msr::airlib_rpclib::UrdfBotRpcLibAdaptors::UrdfBotState {
        return msr::airlib_rpclib::UrdfBotRpcLibAdaptors::UrdfBotState(getVehicleApi(vehicle_name)->getBotState(vehicle_name));
    });

    server->
        bind("updateControlledMotionComponentControlSignal", [&](const msr::airlib_rpclib::UrdfBotRpcLibAdaptors::UrdfBotControlledMotionComponentControlSignal& signal, const std::string& vehicle_name) -> void {
        getVehicleApi(vehicle_name)->updateControlledMotionComponentControlSignal(signal.to(), vehicle_name);
    });

}

}} //namespace
#endif
#endif
