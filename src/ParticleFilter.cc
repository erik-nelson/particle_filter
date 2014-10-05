#include "../include/ParticleFilter.h"

#include <boost/uuid/seed_rng.hpp>
#include <opencv2/imgproc/imgproc.hpp>

namespace gu = geometry_utils;
namespace iu = image_utils;
namespace fu = file_utils;
namespace lu = localization_utils;

ParticleFilter::ParticleFilter() : initialized(false), max_weight(1.0)
{
  required_params.clear();
  required_params.push_back("particle_count");
  required_params.push_back("laser_hit");
  required_params.push_back("laser_random");
  required_params.push_back("laser_max");
  required_params.push_back("laser_sigma");
  required_params.push_back("translational_stddev");
  required_params.push_back("angular_stddev");
  required_params.push_back("pixel_radius");
  required_params.push_back("resample_coefficient");

  map_identifier = "map_file";
  data_identifier = "data_file";
}

ParticleFilter::~ParticleFilter()
{}

bool ParticleFilter::initialize(const std::vector<std::string>& args)
{
  // Load parameters from config file at args[1]
  if (!fu::loadParameters<double>(args[1], required_params, params))
  {
    printf("[ERROR: ParticleFilter::initialize]: Unable to load parameters\n");
    return false;
  }

  // Iterate over parameter map and print values to user
  for (fu::parameter_list_d::iterator it = params.begin(); it != params.end(); ++it)
    std::cout << "\tParameter: " << it->first << " => " << it->second() << "\n";

  // Load the map from config file at args[1]
  if (!fu::loadMap(args[1], map_identifier, map))
  {
    printf("[ERROR: ParticleFilter::initialize]: Unable to load map\n");
    return false;
  }

  // Initialize particles based on the map limits
  rng1.seed( boost::uuids::detail::seed_rng()() );
  rng2.seed( boost::uuids::detail::seed_rng()() );
  rng3.seed( boost::uuids::detail::seed_rng()() );
  rng4.seed( boost::uuids::detail::seed_rng()() );
  rng5.seed( boost::uuids::detail::seed_rng()() );
  rng6.seed( boost::uuids::detail::seed_rng()() );
  initializeParticlesUniform(map);
  //initializeParticlesLocation(gu::Vec2(350,420));

  // Load the laser scans from config file at args[1]
  if (!fu::loadData(args[1], data_identifier, data))
  {
    printf("[ERROR: ParticleFilter::initialize]: Unable to load sensor data\n");
    return false;
  }

  // Gaurantee member variables are initialized properly
  pose.zeros();
  last_odometry = lu::OdometryMessage();

  return true;
}

void ParticleFilter::run()
{
  lu::DataStream::sensor_type type;
  unsigned int index = 0;
  while (data.getNextPendingMessage(type, index))
  {
    if (index < 50)
      continue;

    std::vector<gu::Vector2> best_laser;
    switch (type)
    {
      case lu::DataStream::ODOMETRY:
        {
          const lu::TaggedMessage<lu::OdometryMessage>::ConstPtr& m =
          data.getPendingOdometryMessage(index);

          motionModel(m->msg);

          break;
        }
      case lu::DataStream::LASER:
        {
          const lu::TaggedMessage<lu::LaserMessage>::ConstPtr& m =
          data.getPendingLaserMessage(index);

          // Build an odometry message first, push state forward
          // using interpolated position value
          lu::OdometryMessage odometry;
          odometry.x = m->msg->xb;
          odometry.y = m->msg->yb;
          odometry.yaw = m->msg->yawb;
          odometry.time = m->msg->time;
          lu::OdometryMessage::ConstPtr odometry_ptr(new lu::OdometryMessage(odometry));

          motionModel(odometry_ptr);

          // Use laser frame position to update measurement model

          measurementModel(m->msg, best_laser);

          break;
        }
      default:
        {
          printf("[ERROR: ParticleFilter::run]: unhandled observation type\n");
          break;
        }
    }

    // After every motion or measurement input, the particles will have changed.
    // Visualize their new distribution over the map as well as the best laser scan
    visualize(best_laser);
  }

  data.clearPendingMessages();
}

void ParticleFilter::initializeParticlesUniform(const lu::Map& map)
{
  // Assume x, y, and theta directions are independent
  boost::uniform_real<double> uniform_x(map.min_x, map.max_x);
  boost::variate_generator<boost::mt19937, boost::uniform_real<double> > u_x(rng1, uniform_x);

  boost::uniform_real<double> uniform_y(map.min_y, map.max_y);
  boost::variate_generator<boost::mt19937, boost::uniform_real<double> > u_y(rng2, uniform_y);

  boost::uniform_real<double> uniform_t(-M_PI, M_PI);
  boost::variate_generator<boost::mt19937, boost::uniform_real<double> > u_t(rng3, uniform_t);

  // Resample until we get a particle that lies in the white space on the map
  for (unsigned int ii = 0; ii < params["particle_count"](); )
  {
    gu::Vector3 state( u_x(), u_y(), u_t() );
    if (map.map.at<float>(map.map.rows - state(1) - 1, state(0)) == 1.f)
    {
      lu::Particle p( state, 1.0 / static_cast<double>( params["particle_count"]() ) );
      particles.push_back(p);
      ++ii;
    }
  }
}

void ParticleFilter::initializeParticlesLocation(const gu::Vec2& point)
{
  boost::uniform_real<double> uniform_t(-M_PI, M_PI);
  boost::variate_generator<boost::mt19937, boost::uniform_real<double> > u_t(rng3, uniform_t);

  // Initialize all particles to the same location
  for (unsigned int ii = 0; ii < params["particle_count"](); ++ii)
  {
    gu::Vector3 state( point(0), point(1), u_t() );
    lu::Particle p( state, 1.0 / static_cast<double>( params["particle_count"]() ) );
    particles.push_back(p);
  }
}

bool ParticleFilter::motionModel(const lu::OdometryMessage::ConstPtr& msg)
{
  // Given new odometry, loop over all particles,
  // propagating them forward with some additive noise

  // Build normal distributions for error on motion and measurement models
  static boost::normal_distribution<double> t_normal( 0.0, params["translational_stddev"]() );
  static boost::variate_generator< boost::mt19937,
                                   boost::normal_distribution<double> > t_noise(rng4, t_normal);

  static boost::normal_distribution<double> r_normal( 0.0, params["angular_stddev"]() );
  static boost::variate_generator< boost::mt19937,
                                   boost::normal_distribution<double> > r_noise(rng5, r_normal);

  if (!initialized)
  {
    last_odometry = *msg;
    initialized = true;
    return false;
  }

  // Odometry is given in global frame, need to calculate delta to propagate uncertainty
  gu::Vector2 delta_trans =
    gu::Vector2(msg->x, msg->y) - gu::Vector2(last_odometry.x, last_odometry.y);

  double delta_rot = gu::shortest_angular_distance(msg->yaw, last_odometry.yaw);

  for (unsigned int ii = 0; ii < params["particle_count"](); ++ii)
  {
    // Calculate error in robot motion update with normally distributed noise
    double ds = gu::norm(delta_trans) * ( 1.0 + t_noise() ) / map.resolution;
    double da = delta_rot * ( 1.0 + r_noise() );

    // Calculate the new state of the particle after applying odometry
    double x_new = particles[ii].state(0) + ds * cos(particles[ii].state(2) + da);
    double y_new = particles[ii].state(1) + ds * sin(particles[ii].state(2) + da);

    // Make sure to do a boundary check on the map
    x_new = x_new > map.max_x ? map.max_x : x_new < map.min_x ? map.min_x : x_new;
    y_new = y_new > map.max_y ? map.max_y : y_new < map.min_y ? map.min_y : y_new;

    // Update the particle's state
    particles[ii].state(0) = x_new;
    particles[ii].state(1) = y_new;
    particles[ii].state(2) = gu::normalize(particles[ii].state(2) + da); // unroll from -PI to PI
  }

  last_odometry = *msg;

  return true;
}

bool ParticleFilter::measurementModel(const lu::LaserMessage::ConstPtr& msg,
                                      std::vector<gu::Vector2>& laser_out)
{
  // Use a beam-based measurement model to calculate particle weights
  // Rather than raycasting, we have already pre-computed the distance field for the map,
  // so get likelihoods by querying the distance field

  // Store params locally rather than inline lookup
  double z_hit = params["laser_hit"]();
  double z_random = params["laser_random"]();
  unsigned int particle_count = params["particle_count"]();
  double resample_coefficient = params["resample_coefficient"]();

  double w_sum = 0.0;

  //HACK
  unsigned int best = 0;
  double best_w = 0.0;
  bool test_max = false;
  std::vector< std::vector<gu::Vector2> > laser_spots_list;
  //HACK

  for (unsigned int ii = 0; ii < particle_count; ++ii)
  {
    //HACK
    test_max = false;
    //HACK

    // Laser is 25 cm forward from robot center
    double laser_x = particles[ii].state(0) + cos(particles[ii].state(2)) * 2.50;
    double laser_y = particles[ii].state(1) + sin(particles[ii].state(2)) * 2.50;

    double w = 0.0;

    //HACK
    std::vector< gu::Vector2 > laser_spots;
    //HACK

    for (unsigned int jj = 0; jj < msg->ranges.size(); ++jj)
    {
      double laser_angle = particles[ii].state(2) + beamIndexToAngle(jj);
      double zx = laser_x + msg->ranges[jj] * cos(laser_angle) / map.resolution;
      double zy = laser_y + msg->ranges[jj] * sin(laser_angle) / map.resolution;

      //HACK
      laser_spots.push_back(gu::Vector2(zx, zy));
      //HACK

      double z = lu::queryDistanceField(map.distance_field, zx, map.map.rows - zy - 1);
      double pz = 0.0;

      if (z >= 0)
        pz = z_hit / ( z + z_random );
      else // laser beam landed outside of map (i.e. unknown space)
        pz = 0.01 * z_random;

      w += pz;
    }

    //HACK
    laser_spots_list.push_back(laser_spots);

    if (w > best_w)
    {
    best_w = w;
    best = ii;
    test_max = true;
    }
    //HACK

    // Assign calculated weight to particle
    particles[ii].w = w;
    w_sum += w;
  }

  // Store the best laser scan
  laser_out = laser_spots_list[best];

  // TODO: Use w_sum to modulate particle count (w_sum = 1/eta)
  //printf("eta: %lf\n", w_sum);

  // Normalize particle weights
  max_weight = 0.0;
  for (unsigned int ii = 0; ii < particle_count; ++ii)
  {
    particles[ii].w /= w_sum;

    // Get max weight for coloring visualization
    if (particles[ii].w > max_weight)
      max_weight = particles[ii].w;
  }

  // Copy particles so that we can resample
  prop_particles.clear();
  prop_particles.assign(particles.begin(), particles.end());
  particles.clear();

  // Sample a small number of particles at random
  for (unsigned int ii = 0; ii < params["n_random"](); ++ii)
    particles.push_back(sampleRandom());


  // Sort particles by weight for resampling purposes
  std::sort(prop_particles.begin(), prop_particles.end(), lu::sortWeight);

  // Resample from propagated particles
  //std::vector<double> cumulative_weights;
  //double cumulative_weight = 0.0;
  //for (unsigned int ii = 0; ii < particle_count; ++ii)
  //{
  //  cumulative_weight += prop_particles[ii].w;
  //  cumulative_weights.push_back(cumulative_weight);
  //}

  //static boost::uniform_real<double> uniform(0.0, 1.0);
  //static boost::variate_generator<boost::mt19937, boost::uniform_real<double> > u(rng6, uniform);
  //for (unsigned int ii = 0; ii < particle_count; ++ii)
  //{
  //  double sampled = u();
  //  for (unsigned int jj = 0; jj < cumulative_weights.size(); ++jj)
  //  {
  //    if (sampled < cumulative_weights[jj])
  //    {
  //      particles.push_back(prop_particles[jj]);
  //      break;
  //    }
  //  }
  //}

  static boost::uniform_real<double> uniform(0.0, 1.0);
  static boost::variate_generator<boost::mt19937, boost::uniform_real<double> > u(rng6, uniform);
  for (unsigned int ii = particles.size(); ii < particle_count; ++ii)
  {
    double sampled = u();
    unsigned int fx = floor( pow( sampled, 1.0 / params["resample_coefficient"]() ) * particle_count);
    particles.push_back(prop_particles[fx]);
  }

  return true;
}

void ParticleFilter::visualize(const std::vector<gu::Vector2>& laser)
{
  // Create a copy of the map for visualization
  // convert it to 3 channel floating point color values
  cv::Mat display = map.map;
  cv::cvtColor(display, display, CV_GRAY2BGR);

  // Loop over particles, adding them to the display
  // Boundary checks are already handled in the motion model
  for (unsigned int ii = 0; ii < params["particle_count"](); ++ii)
  {
    int grid_px = static_cast<int>(particles[ii].state(0));
    int grid_py = static_cast<int>(particles[ii].state(1));

    float red = 1.f - static_cast<float>(particles[ii].w / max_weight);
    float green = static_cast<float>(particles[ii].w / max_weight);

    float arrow_len = 10.0;
    cv::Point start(grid_px, map.map.rows - 1 - grid_py);
    cv::Point tip = start + cv::Point(arrow_len * cos(particles[ii].state(2)),
                                      -arrow_len * sin(particles[ii].state(2)));

    cv::Point arrow_left = tip - cv::Point(arrow_len / 2.0 * cos(particles[ii].state(2) - M_PI/4),
                                           -arrow_len / 2.0 * sin(particles[ii].state(2) - M_PI/4));
    cv::Point arrow_right = tip - cv::Point(arrow_len / 2.0 * cos(particles[ii].state(2) + M_PI/4),
                                            -arrow_len / 2.0 * sin(particles[ii].state(2) + M_PI/4));
    cv::line(display, start, tip, cv::Scalar(0.f, green, red), 1, 8);
    cv::line(display, arrow_left, tip, cv::Scalar(0.f, green, red), 1, 8);
    cv::line(display, arrow_right, tip, cv::Scalar(0.f, green, red), 1, 8);

    cv::circle(display, // image
              cv::Point(grid_px, map.map.rows - 1 - grid_py), // center
              params["pixel_radius"](), // circle radius
              cv::Scalar(0.f, green, red), // color
              -1, // filled
              8); // line type
  }

  // Visualize the best laser scan
  for (unsigned int jj = 0; jj < laser.size(); ++jj)
  {
   int grid_px = static_cast<int>(laser[jj](0));
   int grid_py = static_cast<int>(laser[jj](1));

   cv::Scalar color(0.0f, 1.0f, 0.0f);
   cv::circle(display, // image
              cv::Point(grid_px, map.map.rows - 1 - grid_py), // center
              params["pixel_radius"](), // circle radius
              color, // color
              -1, // filled
              8); // line type
  }

  // Display the image
  cv::imshow("ParticleFilter.cc", display);
  cv::waitKey(1);
}
