#ifndef air_UrdfBotRpcLibAdaptors_hpp
#define air_UrdfBotRpcLibAdaptors_hpp

#include "common/Common.hpp"
#include "common/CommonStructs.hpp"
#include "api/RpcLibAdapatorsBase.hpp"
#include "common/ImageCaptureBase.hpp"
#include "vehicles/urdfbot/api/UrdfBotApiBase.hpp"
#include "physics/Kinematics.hpp"
#include "rpc/msgpack.hpp"

#include <string>

namespace msr { namespace airlib_rpclib {

class UrdfBotRpcLibAdaptors : public RpcLibAdapatorsBase {
public:	
    struct AddLinearForce {
        std::string force_name;
        std::string link_name;
        msr::airlib_rpclib::RpcLibAdapatorsBase::Vector3r application_point;
        msr::airlib_rpclib::RpcLibAdapatorsBase::Vector3r axis;

        MSGPACK_DEFINE_MAP(force_name, link_name, application_point, axis)

        AddLinearForce() {};

        AddLinearForce(const msr::airlib::UrdfBotApiBase::AddLinearForce& alf)
        {
            force_name = alf.force_name;
            link_name = alf.link_name;
            application_point = msr::airlib_rpclib::RpcLibAdapatorsBase::Vector3r(alf.application_point);
            axis = msr::airlib_rpclib::RpcLibAdapatorsBase::Vector3r(alf.axis);
        }

        msr::airlib::UrdfBotApiBase::AddLinearForce to() const
        {
            return msr::airlib::UrdfBotApiBase::AddLinearForce(force_name, link_name, application_point.to(), axis.to());
        }
    };

    struct AddAngularForce {
        std::string force_name;
        std::string link_name;
        msr::airlib_rpclib::RpcLibAdapatorsBase::Vector3r axis;

        MSGPACK_DEFINE_MAP(force_name, link_name, axis)

        AddAngularForce() {};

        AddAngularForce(const msr::airlib::UrdfBotApiBase::AddAngularForce& aaf)
        {
            force_name = aaf.force_name;
            link_name = aaf.link_name;
            axis = msr::airlib_rpclib::RpcLibAdapatorsBase::Vector3r(aaf.axis);
        }

        msr::airlib::UrdfBotApiBase::AddAngularForce to() const
        {
            return msr::airlib::UrdfBotApiBase::AddAngularForce(force_name, link_name, axis.to());
        }
    };

    struct UpdateForceMagnitude {
        std::string force_name;
        float magnitude;

        MSGPACK_DEFINE_MAP(force_name, magnitude)

        UpdateForceMagnitude() {};

        UpdateForceMagnitude(const msr::airlib::UrdfBotApiBase::UpdateForceMagnitude& ufm)
        {
            force_name = ufm.force_name;
            magnitude = ufm.magnitude;
        }

        msr::airlib::UrdfBotApiBase::UpdateForceMagnitude to() const
        {
            return msr::airlib::UrdfBotApiBase::UpdateForceMagnitude(force_name, magnitude);
        }
    };

    struct LinkInformation {
        std::string link_name;
        msr::airlib_rpclib::RpcLibAdapatorsBase::Pose link_relative_pose;
        msr::airlib_rpclib::RpcLibAdapatorsBase::Twist link_relative_twist;

        MSGPACK_DEFINE_MAP(link_name, link_relative_pose, link_relative_twist)

        LinkInformation() {};

        LinkInformation(const msr::airlib::UrdfBotApiBase::LinkInformation& li)
        {
            link_name = li.link_name;
            link_relative_pose = msr::airlib_rpclib::RpcLibAdapatorsBase::Pose(li.link_relative_pose);
            link_relative_twist = msr::airlib_rpclib::RpcLibAdapatorsBase::Twist(li.link_relative_twist);
        }

        msr::airlib::UrdfBotApiBase::LinkInformation to() const
        {
            return msr::airlib::UrdfBotApiBase::LinkInformation(link_name, link_relative_pose.to(), link_relative_twist.to());
        }
    };

    struct UrdfBotState {
        std::vector<msr::airlib_rpclib::UrdfBotRpcLibAdaptors::LinkInformation> link_information;
        msr::airlib_rpclib::RpcLibAdapatorsBase::KinematicsState kinematics_estimated;
        std::map<std::string, std::map<std::string, std::string>> controlled_motion_component_states;

        MSGPACK_DEFINE_MAP(link_information, kinematics_estimated, controlled_motion_component_states)

        UrdfBotState() {};

        UrdfBotState(const msr::airlib::UrdfBotApiBase::UrdfBotState& s)
        {
            link_information.clear();
            controlled_motion_component_states.clear();

            for (auto it = s.link_information.begin(); it != s.link_information.end(); ++it)
            {
                LinkInformation li(*it);
                link_information.emplace_back(li);
            }

            kinematics_estimated = msr::airlib_rpclib::RpcLibAdapatorsBase::KinematicsState(s.kinematics_estimated);

            for (auto const &kvp : s.controlled_motion_component_states)
            {
                std::map<std::string, std::string> this_component_state;
                for (auto const &this_kvp : kvp.second)
                {
                    this_component_state[this_kvp.first] = this_kvp.second;
                }
                this->controlled_motion_component_states[kvp.first] = this_component_state;
            }
        }

        msr::airlib::UrdfBotApiBase::UrdfBotState to() const
        {
            std::vector<msr::airlib::UrdfBotApiBase::LinkInformation> converted_link_information;

            for (auto li : link_information)
            {
                converted_link_information.emplace_back(li.to());
            }

            return msr::airlib::UrdfBotApiBase::UrdfBotState(converted_link_information, kinematics_estimated.to(), controlled_motion_component_states);
        }
    };

    struct UrdfBotControlledMotionComponentControlSignal {
        std::string component_name;
        std::map<std::string, float> control_signal_values;

        MSGPACK_DEFINE_MAP(component_name, control_signal_values);

        UrdfBotControlledMotionComponentControlSignal() {};

        UrdfBotControlledMotionComponentControlSignal(const msr::airlib::UrdfBotApiBase::UrdfBotControlledMotionComponentControlSignal &s)
        {
            this->control_signal_values.clear();

            this->component_name = s.component_name;
            for (const auto &kvp : s.control_signal_values)
            {
                this->control_signal_values[kvp.first] = kvp.second;
            }
        }

        msr::airlib::UrdfBotApiBase::UrdfBotControlledMotionComponentControlSignal to() const
        {
            msr::airlib::UrdfBotApiBase::UrdfBotControlledMotionComponentControlSignal target;

            target.component_name = this->component_name;

            for (const auto &kvp : this->control_signal_values)
            {
                target.control_signal_values[kvp.first] = kvp.second;
            }

            return target;
        }
    };
};
}}
#endif