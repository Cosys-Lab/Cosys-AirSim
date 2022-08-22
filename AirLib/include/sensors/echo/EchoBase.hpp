// Copyright (c) Microsoft Corporation. All rights reserved.
// Licensed under the MIT License.

#ifndef msr_airlib_EchoBase_hpp
#define msr_airlib_EchoBase_hpp

#include "sensors/SensorBase.hpp"

namespace msr { namespace airlib {

class EchoBase : public SensorBase {
public:
    EchoBase(const std::string& sensor_name = "")
        : SensorBase(sensor_name)
    {}

public: //types
    struct Output { //fields to enable creation of ROS message PointCloud2 and LaserScan

        // header
        TTimePoint time_stamp;
        Pose relative_pose;

        // data
        // - array of floats that represent [x,y,z] coordinate for each point hit within the range
        //       x0, y0, z0, x1, y1, z1, ..., xn, yn, zn
        //       TODO: Do we need an intensity place-holder [x,y,z, intensity]?
        // - in echo local NED coordinates
        // - in meters
        vector<real_T> point_cloud;
    };

public:
    virtual void reportState(StateReporter& reporter) override
    {
        //call base
        UpdatableObject::reportState(reporter);

        reporter.writeValue("Echo-Timestamp", output_.time_stamp);
        reporter.writeValue("Echo-NumPoints", static_cast<int>(output_.point_cloud.size() / 3));
    }

	const EchoData& getOutput() const
	{
		return output_;
	}

	const EchoData& getInput() const
	{
		return input_;
	}

	void setInput(const EchoData& input) const
	{
		input_ = input;
	}

protected:
    void setOutput(const EchoData& output)
    {
        output_ = output;
    }

private:
    EchoData output_;
	mutable EchoData input_;
};

}} //namespace
#endif
