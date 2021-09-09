// Copyright (c) Microsoft Corporation. All rights reserved.
// Licensed under the MIT License.

#ifndef msr_airlib_GPULidarBase_hpp
#define msr_airlib_GPULidarBase_hpp

#include "sensors/SensorBase.hpp"

namespace msr {
	namespace airlib {

		class GPULidarBase : public SensorBase {
		public:
			GPULidarBase(const std::string& sensor_name = "")
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
				// - in lidar local NED coordinates
				// - in meters
				vector<real_T> point_cloud;
			};

		public:
			virtual void reportState(StateReporter& reporter) override
			{
				//call base
				UpdatableObject::reportState(reporter);

				reporter.writeValue("Lidar-Timestamp", output_.time_stamp);
				reporter.writeValue("Lidar-NumPoints", static_cast<int>(output_.point_cloud.size() / 5));
			}

			const GPULidarData& getOutput() const
			{
				return output_;
			}

		protected:
			void setOutput(const GPULidarData& output)
			{
				output_ = output;
			}

		private:
			GPULidarData output_;
		};

	}
} //namespace
#endif
