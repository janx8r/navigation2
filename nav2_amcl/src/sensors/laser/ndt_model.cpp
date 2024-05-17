/*
 *  Player - One Hell of a Robot Server
 *  Copyright (C) 2000  Brian Gerkey   &  Kasper Stoy
 *                      gerkey@usc.edu    kaspers@robotics.usc.edu
 *
 *  This library is free software; you can redistribute it and/or
 *  modify it under the terms of the GNU Lesser General Public
 *  License as published by the Free Software Foundation; either
 *  version 2.1 of the License, or (at your option) any later version.
 *
 *  This library is distributed in the hope that it will be useful,
 *  but WITHOUT ANY WARRANTY; without even the implied warranty of
 *  MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the GNU
 *  Lesser General Public License for more details.
 *
 *  You should have received a copy of the GNU Lesser General Public
 *  License along with this library; if not, write to the Free Software
 *  Foundation, Inc., 59 Temple Place, Suite 330, Boston, MA  02111-1307  USA
 *
 */

#include <math.h>
#include <assert.h>

#include "nav2_amcl/sensors/laser/laser.hpp"

namespace nav2_amcl
{

NdtLaserModel::NdtLaserModel(size_t max_beams, NdtMap * map)
: Laser(max_beams, map)
{
}

double
NdtLaserModel::sensorFunction(LaserData * data, pf_sample_set_t * set)
{
  NdtLaserModel * self;
  int step;
  double p, pz;
  double obs_range, obs_bearing;
  double total_weight;
  pf_sample_t * sample;
  pf_vector_t pose;

  self = reinterpret_cast<NdtLaserModel *>(data->laser);

  total_weight = 0.0;

  // std::cout << "start computing total_weight. set->sample_count = " << set->sample_count << ", data->range_count = " << data->range_count << std::endl;



  // Compute the sample weights
  for (int j = 0; j < set->sample_count; j++) {
    sample = set->samples + j;
    pose = sample->pose;

    // Take account of the laser pose relative to the robot
    pose = pf_vector_coord_add(self->laser_pose_, pose);

    p = 1.0;

    step = (data->range_count - 1) / (self->max_beams_ - 1);

    // Step size must be at least 1
    if (step < 1) {
      step = 1;
    }

    for (int i = 0; i < data->range_count; i += step) {
      obs_range = data->ranges[i][0];
      obs_bearing = data->ranges[i][1];

      // This model ignores max range readings
      if (obs_range >= data->range_max) {
        continue;
      }

      // Check for NaN
      if (obs_range != obs_range) {
        continue;
      }

      // Compute the endpoint of the beam
      double pos_x = pose.v[0] + obs_range * cos(pose.v[2] + obs_bearing);
      double pos_y = pose.v[1] + obs_range * sin(pose.v[2] + obs_bearing);

      pz = self->ndt_map_->get_probability(pos_x, pos_y);
      
      if(pz != pz)
        std::cout << "pz is nan, i = " << i << " j = " << j << std::endl;
      else
        p += pz;
    }

    sample->weight *= p;
    total_weight += sample->weight;
  }

  std::cout << "total_weight = " << total_weight << std::endl;

  return total_weight;
}


bool
NdtLaserModel::sensorUpdate(pf_t * pf, LaserData * data)
{
  if (max_beams_ < 2) {
    return false;
  }
  pf_update_sensor(pf, (pf_sensor_model_fn_t) sensorFunction, data);

  return true;
}

}  // namespace nav2_amcl
