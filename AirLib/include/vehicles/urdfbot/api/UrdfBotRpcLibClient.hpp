#ifndef air_UrdfBotRpcLibClient_hpp
#define air_UrdfBotRpcLibClient_hpp

#include "common/Common.hpp"
#include <functional>
#include "common/CommonStructs.hpp"
#include "vehicles/urdfbot/api/UrdfBotApiBase.hpp"
#include "api/RpcLibClientBase.hpp"
#include "common/ImageCaptureBase.hpp"

namespace msr { namespace airlib {

class UrdfBotRpcLibClient : public RpcLibClientBase {
public:
    UrdfBotRpcLibClient(const string& ip_address = "localhost", uint16_t port = 41451, float timeout_sec = 60);

    void addAngularForce(const UrdfBotApiBase::AddAngularForce& force, const std::string& vehicle_name);
    void addLinearForce(const UrdfBotApiBase::AddLinearForce& force, const std::string& vehicle_name);
    void updateForceMagnitude(const UrdfBotApiBase::UpdateForceMagnitude& update, const std::string& vehicle_name);
    void updateControlledMotionComponentControlSignal(const UrdfBotApiBase::UrdfBotControlledMotionComponentControlSignal& updated_control_signal, const std::string& vehicle_name);
    UrdfBotApiBase::UrdfBotState getUrdfBotState(const std::string& vehicle_name);
    

    virtual ~UrdfBotRpcLibClient(); // required for pimpl
};

}}
#endif