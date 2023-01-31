// Developed by Cosys-Lab, University of Antwerp

#ifndef msr_airlib_MarLocUwbBase_hpp
#define msr_airlib_MarLocUwbBase_hpp

#include "sensors/SensorBase.hpp"

namespace msr { namespace airlib {

class MarLocUwbBase : public SensorBase {
public:
	/*MarLocUwbBase(const std::string& sensor_name = "", const std::string& attach_link = "")
        : SensorBase(sensor_name, attach_link)
    {}*/
    MarLocUwbBase(const std::string& sensor_name = "")
        : SensorBase(sensor_name){
        if (sensor_name.find("_") != std::string::npos) {
            id_ = stoi(sensor_name.substr(sensor_name.find_last_of("_") + 1));
        }else {
            id_ = 0;
        }
    }

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

        reporter.writeValue("MarLocUwbSensor-Timestamp", output_.time_stamp);
    }

	const MarLocUwbSensorData& getOutput() const
	{
		return output_;
	}

    const int& getID() const
    {
        return id_;
    }

	const MarLocUwbSensorData& getInput() const
	{
		return input_;
	}

	void setInput(const MarLocUwbSensorData& input) const
	{
		input_ = input;
	}

protected:
    void setOutput(const MarLocUwbSensorData& output)
    {
        output_ = output;
    }

private:
    MarLocUwbSensorData output_;
    int id_;
	mutable MarLocUwbSensorData input_;
};

}} //namespace
#endif
