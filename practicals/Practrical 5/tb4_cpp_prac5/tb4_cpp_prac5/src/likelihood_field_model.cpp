#include <math.h>
#include <assert.h>

#include "iar_amcl/likelihood_field_model.hpp"

namespace iar_amcl
{
    LikelihoodFieldModel::LikelihoodFieldModel(
    double z_hit, double z_max, double z_rand, double sigma_hit,
    double max_occ_dist, size_t max_beams, map_t * map)
    : Laser(max_beams, map)
    {
        z_hit_ = z_hit;
        z_rand_ = z_rand;
        sigma_hit_ = sigma_hit;
        z_max_ = z_max;
        map_update_cspace(map, max_occ_dist);
    }

    // Determine the probability for the given pose
    double
    LikelihoodFieldModel::sensorFunction(nav2_amcl::LaserData * data, pf_sample_set_t * set)
    {
        LikelihoodFieldModel * self;
        int i, j, step;
        double pz; 
        double dist;
        double p; // probability (or weight) of indivdual particle
        double obs_range, obs_bearing; // the measurement values of one laser beam
        double total_weight;
        pf_sample_t * sample;
        pf_vector_t pose_robot; 

        // hit_x and hit_y are the horizontal position and vertical position of the endpoint of a laser beam
        double hit_x, hit_y;

        self = reinterpret_cast<LikelihoodFieldModel *>(data->laser);

        total_weight = 0.0;

        // Compute the sample weights
        for (j = 0; j < set->sample_count; j++) {
                sample = set->samples + j;
                pose_robot = sample->pose;

                /* TODO TASK - MILSTONE 1.1 
                    Compute the pose of the lidar sensor.
                */
                // Get the lidar-to-robot transform (Tsens)
                pf_vector_t lidar_pose_to_robot = self->laser_pose_;

                // Compute lidar's pose in the global frame
                double lidar_x, lidar_y, lidar_theta;
                lidar_x = pose_robot.v[0]
                        + lidar_pose_to_robot.v[0] * cos(pose_robot.v[2])
                        - lidar_pose_to_robot.v[1] * sin(pose_robot.v[2]);
                lidar_y = pose_robot.v[1]
                        + lidar_pose_to_robot.v[0] * sin(pose_robot.v[2])
                        + lidar_pose_to_robot.v[1] * cos(pose_robot.v[2]);
                lidar_theta = pose_robot.v[2] + lidar_pose_to_robot.v[2];
                // 1.1 End
                p = 1.0;

                step = (data->range_count - 1) / (self->max_beams_ - 1);
                // Step size normally larger than 1, but in case of error setting max number beams used 
                // to model the probability, we need to check the step and ensure it is larger or equal to 1
                if (step<1)
                    step = 1;

                for (i = 0; i < data->range_count; i += step) {
                    obs_range = data->ranges[i][0];
                    obs_bearing = data->ranges[i][1];

                    pz = 0.0;
                    /* TODO TASK - MILESTONE 1.2
                        Check whether the laser beam's value is infinity, if yes, skip this beam measurement
                    */
                    if (std::isinf(obs_range)) 
                    {
                    continue;
                    }
                    // 1.2 End
                    /* 
                        TODO TASK - MILESTONE 1.3
                        Check whether a failure measurement is detected, i.e., max range is detected. If yes, update the probability
                    */
                    if (obs_range >= data->range_max) {
                    pz += self->z_max_;
                    }
                    // 1.3 End

                    /* 
                        TODO TASK - MILESTONE 1.4
                        Process a beam with range measurement less than the maximum value.
                    */
                    } else {
                    // Random measurement component
                    pz += self->z_rand_ / data->range_max;

                    // Compute the endpoint of the beam in the global frame
                    hit_x = lidar_x + obs_range * cos(lidar_theta + obs_bearing);
                    hit_y = lidar_y + obs_range * sin(lidar_theta + obs_bearing);

                    // Convert to map grid coordinates
                    int m_x = MAP_GXWX(self->map_, hit_x);
                    int m_y = MAP_GYWY(self->map_, hit_y);

                    if (MAP_VALID(self->map_, m_x, m_y)) {
                        int index = MAP_INDEX(self->map_, m_x, m_y);
                        dist = self->map_->cells[index].occ_dist;
                    } else {
                        dist = self->map_->max_occ_dist;
                    }

                    // Gaussian ("hit") component
                    pz += self->z_hit_ * exp(-(dist * dist) / (2 * self->sigma_hit_ * self->sigma_hit_));
                }
                    // 1.4 End
                    
                    
                    assert(pz <= 1.0);
                    assert(pz >= 0.0);
                    //      p *= pz;
                    // here we have an ad-hoc weighting scheme for combining beam probs
                    // works well, though...
                    p += pz * pz * pz;
                }
            sample->weight *= p;
            total_weight += sample->weight;
        }
        return total_weight;
    }

    bool 
    LikelihoodFieldModel::sensorUpdate(pf_t * pf, nav2_amcl::LaserData * data)
    {
        if (max_beams_ < 2) {
            return false;
        }
        pf_update_sensor(pf, (pf_sensor_model_fn_t) sensorFunction, data);

        return true;
    }
}