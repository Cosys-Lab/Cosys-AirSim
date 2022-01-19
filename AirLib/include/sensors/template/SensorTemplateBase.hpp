// Copyright (c) Microsoft Corporation. All rights reserved.
// Licensed under the MIT License.

#ifndef msr_airlib_SensorTemplateBase_hpp
#define msr_airlib_SensorTemplateBase_hpp

#include "sensors/SensorBase.hpp"

namespace msr { namespace airlib {

class SensorTemplateBase : public SensorBase {
public:
	/*SensorTemplateBase(const std::string& sensor_name = "", const std::string& attach_link = "")
        : SensorBase(sensor_name, attach_link)
    {}*/
    SensorTemplateBase(const std::string& sensor_name = "")
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

        reporter.writeValue("TemplateSensor-Timestamp", output_.time_stamp);
    }

	const SensorTemplateData& getOutput() const
	{
		return output_;
	}

	const SensorTemplateData& getInput() const
	{
		return input_;
	}

	void setInput(const SensorTemplateData& input) const
	{
		input_ = input;
	}

protected:
    void setOutput(const SensorTemplateData& output)
    {
        output_ = output;
    }

private:
	SensorTemplateData output_;
	mutable SensorTemplateData input_;
};

}} //namespace
#endif
