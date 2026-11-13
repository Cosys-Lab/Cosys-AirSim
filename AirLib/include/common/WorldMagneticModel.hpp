// Copyright (c) Cosys-Lab, University of Antwerp. 2026
//
// World Magnetic Model lookup: declination, inclination and total intensity of the Earth's
// magnetic field from the tables in WorldMagneticModelTables.hpp (bilinear interpolation on a
// 10 degree grid). The lookup logic is ported from PX4 (src/lib/world_magnetic_model/
// geo_mag_declination.cpp) so AirLib's magnetometer reference matches what PX4's EKF uses;
// it is distributed under PX4's license:
//
// Copyright (c) 2020-2024 PX4 Development Team. All rights reserved.
//
// Redistribution and use in source and binary forms, with or without
// modification, are permitted provided that the following conditions
// are met:
//
// 1. Redistributions of source code must retain the above copyright
//    notice, this list of conditions and the following disclaimer.
// 2. Redistributions in binary form must reproduce the above copyright
//    notice, this list of conditions and the following disclaimer in
//    the documentation and/or other materials provided with the
//    distribution.
// 3. Neither the name PX4 nor the names of its contributors may be
//    used to endorse or promote products derived from this software
//    without specific prior written permission.
//
// THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS
// "AS IS" AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT
// LIMITED TO, THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS
// FOR A PARTICULAR PURPOSE ARE DISCLAIMED. IN NO EVENT SHALL THE
// COPYRIGHT OWNER OR CONTRIBUTORS BE LIABLE FOR ANY DIRECT, INDIRECT,
// INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING,
// BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES; LOSS
// OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER CAUSED
// AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT
// LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN
// ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE
// POSSIBILITY OF SUCH DAMAGE.

#ifndef msr_airlib_WorldMagneticModel_hpp
#define msr_airlib_WorldMagneticModel_hpp

#include <cmath>
#include <cstdint>
#include <algorithm>
#include "common/WorldMagneticModelTables.hpp"

namespace msr
{
namespace airlib
{
namespace wmm
{

    namespace detail
    {
        inline float constrain(float val, float min_val, float max_val)
        {
            return std::min(std::max(val, min_val), max_val);
        }

        inline unsigned getLookupTableIndex(float* val, float min_val, float max_val)
        {
            // limit to (table bounds - 1) because bilinear interpolation requires checking (index + 1)
            *val = constrain(*val, min_val, max_val - SAMPLING_RES);
            return static_cast<unsigned>((-(min_val) + *val) / SAMPLING_RES);
        }

        inline float getTableData(float latitude_deg, float longitude_deg, const int16_t table[LAT_DIM][LON_DIM])
        {
            latitude_deg = constrain(latitude_deg, SAMPLING_MIN_LAT, SAMPLING_MAX_LAT);

            if (longitude_deg > SAMPLING_MAX_LON) {
                longitude_deg -= 360.f;
            }
            if (longitude_deg < SAMPLING_MIN_LON) {
                longitude_deg += 360.f;
            }

            // round down to nearest sampling resolution
            float min_lat = std::floor(latitude_deg / SAMPLING_RES) * SAMPLING_RES;
            float min_lon = std::floor(longitude_deg / SAMPLING_RES) * SAMPLING_RES;

            // find index of nearest low sampling point
            const unsigned min_lat_index = getLookupTableIndex(&min_lat, SAMPLING_MIN_LAT, SAMPLING_MAX_LAT);
            const unsigned min_lon_index = getLookupTableIndex(&min_lon, SAMPLING_MIN_LON, SAMPLING_MAX_LON);

            const float data_sw = table[min_lat_index][min_lon_index];
            const float data_se = table[min_lat_index][min_lon_index + 1];
            const float data_ne = table[min_lat_index + 1][min_lon_index + 1];
            const float data_nw = table[min_lat_index + 1][min_lon_index];

            // bilinear interpolation on the four grid corners
            const float lat_scale = constrain((latitude_deg - min_lat) / SAMPLING_RES, 0.f, 1.f);
            const float lon_scale = constrain((longitude_deg - min_lon) / SAMPLING_RES, 0.f, 1.f);

            const float data_min = lon_scale * (data_se - data_sw) + data_sw;
            const float data_max = lon_scale * (data_ne - data_nw) + data_nw;

            return lat_scale * (data_max - data_min) + data_min;
        }
    }

    // Magnetic declination (degrees, positive east of true north)
    inline float getDeclinationDegrees(float latitude_deg, float longitude_deg)
    {
        return detail::getTableData(latitude_deg, longitude_deg, declination_table) * WMM_DECLINATION_SCALE_TO_DEGREES;
    }

    // Magnetic inclination (degrees, positive pointing down)
    inline float getInclinationDegrees(float latitude_deg, float longitude_deg)
    {
        return detail::getTableData(latitude_deg, longitude_deg, inclination_table) * WMM_INCLINATION_SCALE_TO_DEGREES;
    }

    // Total field strength (Tesla)
    inline float getStrengthTesla(float latitude_deg, float longitude_deg)
    {
        return detail::getTableData(latitude_deg, longitude_deg, totalintensity_table) * WMM_TOTALINTENSITY_SCALE_TO_NANOTESLA * 1e-9f;
    }

}
}
} //namespace
#endif
