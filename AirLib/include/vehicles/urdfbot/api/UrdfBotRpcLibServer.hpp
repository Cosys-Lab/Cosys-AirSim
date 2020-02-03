#ifndef air_UrdfBotRpcLibServer_hpp
#define air_UrdfBotRpcLibServer_hpp

#include "common/Common.hpp"
#include <functional>
#include "api/RpcLibServerBase.hpp"
#include "vehicles/urdfbot/api/UrdfBotApiBase.hpp"

namespace msr { namespace airlib {

class UrdfBotRpcLibServer : public RpcLibServerBase {
public:
	UrdfBotRpcLibServer(ApiProvider* api_provider, string server_address, uint16_t port = 41451);
	virtual ~UrdfBotRpcLibServer() {};

protected:
	virtual UrdfBotApiBase* getVehicleApi(const std::string& vehicle_name) override
	{
		return static_cast<UrdfBotApiBase*>(RpcLibServerBase::getVehicleApi(vehicle_name));
	}
};

}};
#endif