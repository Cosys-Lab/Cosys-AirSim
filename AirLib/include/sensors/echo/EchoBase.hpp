// Developed by Cosys-Lab, University of Antwerp

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
        vector<real_T> point_cloud;
    };

public:
    virtual void reportState(StateReporter& reporter) override
    {
        //call base
        UpdatableObject::reportState(reporter);

        reporter.writeValue("Echo-Timestamp", output_.time_stamp);
        reporter.writeValue("Echo-NumPoints", static_cast<int>(output_.point_cloud.size() / 5));
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
