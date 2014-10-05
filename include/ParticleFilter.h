#ifndef _PARTICLE_FILTER_H_
#define _PARTICLE_FILTER_H_

#include "LocalizationUtils.h"
#include "GeometryUtils.h"
#include "ImageUtils.h"
#include "FileUtils.h"

#include <vector>
#include <string>

#include <boost/random/uniform_real.hpp>
#include <boost/random/normal_distribution.hpp>
#include <boost/random/variate_generator.hpp>
#include <boost/math/distributions/normal.hpp>
#include <boost/random/mersenne_twister.hpp>

class ParticleFilter
{
public:

  /* *********************************************************************************** */
  explicit ParticleFilter();

  /* *********************************************************************************** */
  ~ParticleFilter();

  /* *********************************************************************************** */
  bool initialize(const std::vector<std::string>& args);

  /* *********************************************************************************** */
  void run();

private:

  /* *********************************************************************************** */
  void initializeParticlesUniform(const localization_utils::Map& map);

  /* *********************************************************************************** */
  void initializeParticlesLocation(const geometry_utils::Vec2& point);

  /* *********************************************************************************** */
  bool motionModel(const localization_utils::OdometryMessage::ConstPtr& msg);

  /* *********************************************************************************** */
  bool measurementModel(const localization_utils::LaserMessage::ConstPtr& msg,
                        std::vector<geometry_utils::Vector2>& laser_out);

  /* *********************************************************************************** */
  inline double beamIndexToAngle(const unsigned int& index) const
  {
    return geometry_utils::deg2rad(180.0 / 179.0 * static_cast<double>(index) - 90.0);
  }

  /* *********************************************************************************** */
  void visualize(const std::vector<geometry_utils::Vector2>& laser);

  /* *********************************************************************************** */
  // Members
  std::vector<localization_utils::Particle> particles, prop_particles;
  geometry_utils::Vector3 pose;
  localization_utils::OdometryMessage last_odometry;
  localization_utils::Map map;
  localization_utils::DataStream data;
  bool initialized;
  double max_weight;

  // Parameters
  file_utils::parameter_list_d params;
  std::vector<std::string> required_params;
  std::string map_identifier;
  std::string data_identifier;

  // Random sampling
  boost::mt19937 rng1, rng2, rng3, rng4, rng5, rng6;

};
#endif
