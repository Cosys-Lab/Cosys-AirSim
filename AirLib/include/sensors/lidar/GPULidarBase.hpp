// Developed by Cosys-Lab, University of Antwerp

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
