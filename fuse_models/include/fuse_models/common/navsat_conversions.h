/*
* Copyright (C) 2010 Austin Robot Technology, and others
* All rights reserved.
*
* Redistribution and use in source and binary forms, with or without
* modification, are permitted provided that the following conditions
* are met:
*
* 1. Redistributions of source code must retain the above copyright
* notice, this list of conditions and the following disclaimer.
* 2. Redistributions in binary form must reproduce the above
* copyright notice, this list of conditions and the following
* disclaimer in the documentation and/or other materials provided
* with the distribution.
* 3. Neither the names of the University of Texas at Austin, nor
* Austin Robot Technology, nor the names of other contributors may
* be used to endorse or promote products derived from this
* software without specific prior written permission.
*
* THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS
* "AS IS" AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT
* LIMITED TO, THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS
* FOR A PARTICULAR PURPOSE ARE DISCLAIMED. IN NO EVENT SHALL THE
* COPYRIGHT OWNER OR CONTRIBUTORS BE LIABLE FOR ANY DIRECT, INDIRECT,
* INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING,
* BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES;
* LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER
* CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT
* LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN
* ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE
* POSSIBILITY OF SUCH DAMAGE.
*
* This file contains code from multiple files in the original
* source. The originals can be found here:
*
* https://github.com/austin-robot/utexas-art-ros-pkg/blob/afd147a1eb944fc3dbd138574c39699813f797bf/stacks/art_vehicle/art_common/include/art/UTM.h
* https://github.com/austin-robot/utexas-art-ros-pkg/blob/afd147a1eb944fc3dbd138574c39699813f797bf/stacks/art_vehicle/art_common/include/art/conversions.h
*/
#pragma once

#include <cmath>

#include <sensor_msgs/msg/nav_sat_fix.hpp>

namespace fuse_models
{
namespace common
{
// Helper consts
const double kRADIANS_PER_DEGREE = M_PI / 180.0;
const double kDEGREES_PER_RADIAN = 180.0 / M_PI;
// WGS params
const double kWGS84_A = 6378137.0;         // major axis
const double kWGS84_B = 6356752.31424518;  // minor axis
const double kWGS84_F = 0.0033528107;      // ellipsoid flattening
const double kWGS84_E = 0.0818191908;      // first eccentricity
const double kWGS84_EP = 0.0820944379;     // second eccentricity

// UTM
const double kUTM_K0 = 0.9996;                      // scale factor
const double kUTM_FE = 500000.0;                    // false easting
const double kUTM_FN_N = 0.0;                       // false northing, northern hemisphere
const double kUTM_FN_S = 10000000.0;                // false northing, southern hemisphere
const double kUTM_E2 = (kWGS84_E * kWGS84_E);       // e^2
const double kUTM_E4 = (kUTM_E2 * kUTM_E2);         // e^4
const double kUTM_E6 = (kUTM_E4 * kUTM_E2);         // e^6
const double kUTM_EP2 = (kUTM_E2 / (1 - kUTM_E2));  // e'^2
/**
 * Determine the correct UTM letter designator for the
 * given latitude
 *
 * @returns 'Z' if latitude is outside the UTM limits of 84N to 80S
 *
 * Written by Chuck Gantz- chuck.gantz@globalstar.com
 */
static inline char UTMLetterDesignator(double Lat)
{
  char LetterDesignator;

  if ((84 >= Lat) && (Lat >= 72))
  {
    LetterDesignator = 'X';
  }
  else if ((72 > Lat) && (Lat >= 64))
  {
    LetterDesignator = 'W';
  }
  else if ((64 > Lat) && (Lat >= 56))
  {
    LetterDesignator = 'V';
  }
  else if ((56 > Lat) && (Lat >= 48))
  {
    LetterDesignator = 'U';
  }
  else if ((48 > Lat) && (Lat >= 40))
  {
    LetterDesignator = 'T';
  }
  else if ((40 > Lat) && (Lat >= 32))
  {
    LetterDesignator = 'S';
  }
  else if ((32 > Lat) && (Lat >= 24))
  {
    LetterDesignator = 'R';
  }
  else if ((24 > Lat) && (Lat >= 16))
  {
    LetterDesignator = 'Q';
  }
  else if ((16 > Lat) && (Lat >= 8))
  {
    LetterDesignator = 'P';
  }
  else if ((8 > Lat) && (Lat >= 0))
  {
    LetterDesignator = 'N';
  }
  else if ((0 > Lat) && (Lat >= -8))
  {
    LetterDesignator = 'M';
  }
  else if ((-8 > Lat) && (Lat >= -16))
  {
    LetterDesignator = 'L';
  }
  else if ((-16 > Lat) && (Lat >= -24))
  {
    LetterDesignator = 'K';
  }
  else if ((-24 > Lat) && (Lat >= -32))
  {
    LetterDesignator = 'J';
  }
  else if ((-32 > Lat) && (Lat >= -40))
  {
    LetterDesignator = 'H';
  }
  else if ((-40 > Lat) && (Lat >= -48))
  {
    LetterDesignator = 'G';
  }
  else if ((-48 > Lat) && (Lat >= -56))
  {
    LetterDesignator = 'F';
  }
  else if ((-56 > Lat) && (Lat >= -64))
  {
    LetterDesignator = 'E';
  }
  else if ((-64 > Lat) && (Lat >= -72))
  {
    LetterDesignator = 'D';
  }
  else if ((-72 > Lat) && (Lat >= -80))
  {
    LetterDesignator = 'C';
  }
  else
  {
    // 'Z' is an error flag, the Latitude is outside the UTM limits
    LetterDesignator = 'Z';
  }
  return LetterDesignator;
}
/**
 * @brief Utility to convert geodetic coords into UTM
 *
 * @param[in] latitude the floating point latitude in degrees
 * @param[in] longitude the floating point longitude in degrees
 * @param[out] UTM_northing
 * @param[out] UTM_easting
 * @param[out] UTM_zone
 * @param[out] gamma
 */
static inline void LatitudeLongitudeToUTM(const double latitude, const double longitude,
                                          double& UTM_northing, double& UTM_easting,
                                          std::string& UTM_zone, double& gamma)
{
  // Bound longitude
  double bounded_longitude =
      (longitude + 180) - static_cast<int>((longitude + 180) / 360) * 360 - 180;

  // Convert to radians
  double latitude_rad = latitude * kRADIANS_PER_DEGREE;
  double longitude_rad = bounded_longitude * kRADIANS_PER_DEGREE;
  int zone_number = static_cast<int>((longitude + 180) / 6) + 1;

  // Handle Norway
  if (latitude >= 56.0 && latitude < 64.0 && bounded_longitude >= 3.0 && bounded_longitude < 12.0)
  {
    zone_number = 32;
  }
  if (latitude >= 72.0 && latitude < 84.0)
  {
    if (bounded_longitude >= 0.0 && bounded_longitude < 9.0)
    {
      zone_number = 31;
    }
    else if (bounded_longitude >= 9.0 && bounded_longitude < 21.0)
    {
      zone_number = 33;
    }
    else if (bounded_longitude >= 21.0 && bounded_longitude < 33.0)
    {
      zone_number = 35;
    }
    else if (bounded_longitude >= 33.0 && bounded_longitude < 42.0)
    {
      zone_number = 37;
    }
  }
  // +3 puts origin in middle of zone
  double longitude_origin = (zone_number - 1) * 6 - 180 + 3;
  double longitude_origin_rad = longitude_origin * kRADIANS_PER_DEGREE;
  // Compute the UTM Zone from the latitude and longitude
  char zone_buf[] = { 0, 0, 0, 0 };
  // We &0x3fU to let GCC know the size of zone_number. In this case, it's under
  // 63 (6bits)
  snprintf(zone_buf, sizeof(zone_buf), "%d%c", zone_number & 0x3fU, UTMLetterDesignator(latitude));
  UTM_zone = std::string(zone_buf);

  double eccentricity_prime_squared = (kUTM_E2) / (1 - kUTM_E2);
  double N, T, C, A, M;
  N = kWGS84_A / sqrt(1 - kUTM_E2 * sin(latitude_rad) * sin(latitude_rad));
  T = tan(latitude_rad) * tan(latitude_rad);
  C = eccentricity_prime_squared * cos(latitude_rad) * cos(latitude_rad);
  A = cos(latitude_rad) * (longitude_rad - longitude_origin_rad);

  M = kWGS84_A *
      ((1 - kUTM_E2 / 4 - 3 * kUTM_E4 / 64 - 5 * kUTM_E6 / 256) * latitude_rad -
       (3 * kUTM_E2 / 8 + 3 * kUTM_E4 / 32 + 45 * kUTM_E6 / 1024) * sin(2 * latitude_rad) +
       (15 * kUTM_E4 / 256 + 45 * kUTM_E6 / 1024) * sin(4 * latitude_rad) -
       (35 * kUTM_E6 / 3072) * sin(6 * latitude_rad));
  UTM_easting =
      static_cast<double>(kUTM_K0 * N *
                              (A + (1 - T + C) * A * A * A / 6 +
                               (5 - 18 * T + T * T + 72 * C - 58 * eccentricity_prime_squared) * A *
                                   A * A * A * A / 120) +
                          500000.0);
  UTM_northing = static_cast<double>(
      kUTM_K0 * (M + N * tan(latitude_rad) *
                         (A * A / 2 + (5 - T + 9 * C + 4 * C * C) * A * A * A * A / 24 +
                          (61 - 58 * T + T * T + 600 * C - 330 * eccentricity_prime_squared) * A *
                              A * A * A * A * A / 720)));
  gamma = atan(tan(longitude_rad - longitude_origin_rad) * sin(latitude_rad)) * kDEGREES_PER_RADIAN;

  if (latitude < 0)
  {
    // 10000000 meter offset for southern hemisphere
    UTM_northing += 10000000.0;
  }
}
static inline void LatitudeLongitudeToUTM(const double latitude, const double longitude,
                                          double& UTM_northing, double& UTM_easting,
                                          std::string& UTM_zone)
{
  double gamma;
  LatitudeLongitudeToUTM(latitude, longitude, UTM_northing, UTM_easting, UTM_zone, gamma);
}
}  // namespace common

}  // namespace fuse_models