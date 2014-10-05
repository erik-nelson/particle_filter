#ifndef _LOCALIZATION_H_
#define _LOCALIZATION_H_

#include "GeometryUtils.h"

#include <boost/shared_ptr.hpp>
#include <opencv2/core/core.hpp>
#include <opencv2/imgproc/imgproc.hpp>
#include <opencv2/highgui/highgui.hpp>
#include <stdio.h>

#include <vector>
#include <string>

namespace localization_utils
{
  /* *********************************************************************************** */
  struct LaserMessage
  {
    typedef ::boost::shared_ptr<LaserMessage> Ptr;
    typedef ::boost::shared_ptr<const LaserMessage> ConstPtr;

    LaserMessage() :
      xb(0.0), yb(0.0), yawb(0.0),
      xl(0.0), yl(0.0), yawl(0.0), time(0.0)
    { ranges.clear(); }

    double xb;
    double yb;
    double yawb;
    double xl;
    double yl;
    double yawl;
    double time;

    std::vector<double> ranges;

    inline void print(std::string prefix = std::string()) const
    {
      if (!prefix.empty())
        std::cout << prefix << std::endl;

      std::cout << "xb: " << xb << "\n";
      std::cout << "yb: " << yb << "\n";
      std::cout << "yawb: " << yawb << "\n";
      std::cout << "xb: " << xl << "\n";
      std::cout << "yb: " << yl << "\n";
      std::cout << "yawb: " << yawl << "\n";
      std::cout << "time: " << time << "\n";
      std::cout << "ranges: \n";
      for (unsigned int ii = 0; ii < ranges.size(); ++ii)
        std::cout << ranges[ii] << ", ";
      std::cout << "\n\n";
    }

  };

  /* *********************************************************************************** */
  struct OdometryMessage
  {
    typedef ::boost::shared_ptr<OdometryMessage> Ptr;
    typedef ::boost::shared_ptr<const OdometryMessage> ConstPtr;

    OdometryMessage() : x(0.0), y(0.0), yaw(0.0), time(0.0) {}
    double x;
    double y;
    double yaw;
    double time;

    inline void print(std::string prefix = std::string()) const
    {
      if (!prefix.empty())
        std::cout << prefix << std::endl;

      std::cout << "x: " << x << "\n";
      std::cout << "y: " << y << "\n";
      std::cout << "yaw: " << yaw << "\n";
      std::cout << "time: " << time << "\n\n";
    }
  };

  /* *********************************************************************************** */
  template<class T>
  struct TaggedMessage
  {
    typename T::ConstPtr msg;
    std::string tag;
    typedef ::boost::shared_ptr< TaggedMessage<T> > Ptr;
    typedef ::boost::shared_ptr< const TaggedMessage<T> > ConstPtr;
    TaggedMessage(const typename T::ConstPtr m, const std::string& t) : msg(m), tag(t) {}
  };

  /* *********************************************************************************** */
  class DataStream
  {
  public:
    DataStream() : pending_index(0) {}
    ~DataStream()
    {
      sensor_ordering.clear();
      pending_laser.clear();
      pending_odometry.clear();
    }

    typedef enum {
      LASER,
      ODOMETRY
    } sensor_type;

    void addLaserMessage(const LaserMessage::ConstPtr& msg,
                         const std::string& tag = std::string())
    {
      TaggedMessage<LaserMessage>::Ptr p =
      TaggedMessage<LaserMessage>::Ptr(new TaggedMessage<LaserMessage>(msg, tag));
      pending_laser.push_back(p);
    }
    void addOdometryMessage(const OdometryMessage::ConstPtr& msg,
                            const std::string& tag = std::string())
    {
      TaggedMessage<OdometryMessage>::Ptr p =
      TaggedMessage<OdometryMessage>::Ptr(new TaggedMessage<OdometryMessage>(msg, tag));
      pending_odometry.push_back(p);
    }

    void sortPendingMessages()
    {
      sensor_ordering.clear();

      // Sort the messages based on timestamps, across all input
      unsigned int i = 0;
      for (laser_queue::const_iterator it = pending_laser.begin();
           it != pending_laser.end(); ++it, ++i)
      {
        StampedSensorType::Ptr p =
        StampedSensorType::Ptr(new StampedSensorType((*it)->msg->time, LASER, i));
        sensor_ordering.push_back(p);
      }

      i = 0;
      for (odometry_queue::const_iterator it = pending_odometry.begin();
           it != pending_odometry.end(); ++it, ++i)
      {
        StampedSensorType::Ptr p =
        StampedSensorType::Ptr(new StampedSensorType((*it)->msg->time, ODOMETRY, i));
        sensor_ordering.push_back(p);
      }
      std::sort(sensor_ordering.begin(), sensor_ordering.end(),
                compareSensorTimestamps);

      pending_index = 0;

    }

    void clearPendingMessages()
    {
      sensor_ordering.clear();
      pending_laser.clear();
      pending_odometry.clear();
    }

    bool getNextPendingMessage(sensor_type& type, unsigned int& index)
    {
      if (pending_index >= sensor_ordering.size())
        return false;

      type = sensor_ordering[pending_index]->type;
      index = sensor_ordering[pending_index]->index;

      pending_index++;

      return true;
    }

    const TaggedMessage<OdometryMessage>::ConstPtr& getPendingOdometryMessage(unsigned int index)
    {
      return pending_odometry[index];
    }
    const TaggedMessage<LaserMessage>::ConstPtr& getPendingLaserMessage(unsigned int index)
    {
      return pending_laser[index];
    }

  private:
    struct StampedSensorType
    {
      double time;
      sensor_type type;
      unsigned int index;
      StampedSensorType(double t, sensor_type s, unsigned int i) :
        time(t), type(s), index(i) {}
      typedef ::boost::shared_ptr<StampedSensorType> Ptr;
      typedef ::boost::shared_ptr<const StampedSensorType> ConstPtr;
    };

    static bool compareSensorTimestamps(const StampedSensorType::ConstPtr& lhs,
                                        const StampedSensorType::ConstPtr& rhs)
    {
      return ((lhs->time < rhs->time) ||
              ((lhs->time == rhs->time) && (lhs->type < rhs->type)));
    }

    unsigned int pending_index;
    std::vector< StampedSensorType::ConstPtr > sensor_ordering;

    typedef std::vector< TaggedMessage<LaserMessage>::ConstPtr > laser_queue;
    typedef std::vector< TaggedMessage<OdometryMessage>::ConstPtr > odometry_queue;
    laser_queue pending_laser;
    odometry_queue pending_odometry;

  };

  /* *********************************************************************************** */
  struct Map
  {
    explicit Map() :
      max_x(0), min_x(0), max_y(0), min_y(0),
      origin_x(0.0), origin_y(0.0), resolution(0.0) {}

    ~Map() {}

    ::cv::Mat map;
    ::cv::Mat distance_field;
    unsigned int max_x;
    unsigned int min_x;
    unsigned int max_y;
    unsigned int min_y;
    double origin_x;
    double origin_y;
    double resolution;
  };

  /* *********************************************************************************** */
  struct Particle
  {
    explicit Particle() :
      w(0.0) { state.zeros(); }

    Particle(const geometry_utils::Vector3& state_, const double& w_) :
      state(state_), w(w_) {}

    ~Particle() {}

    ::geometry_utils::Vector3 state;
    double w;

    inline void print(const std::string& prefix = std::string()) const
    {
      state.print(prefix);
      std::cout << w << std::endl;
    }
  };

  /* *********************************************************************************** */
  static bool sortWeight(const Particle& lhs, const Particle& rhs)
  {
    return lhs.w < rhs.w;
  }

  /* *********************************************************************************** */
  static ::cv::Mat createDistanceField(const ::cv::Mat& map)
  {
    ::cv::Mat obstacles(map.rows, map.cols, CV_8UC1, 255);
    for (unsigned int rr = 0; rr < map.rows; ++rr)
      for (unsigned int cc = 0; cc < map.cols; ++cc)
        if (map.at<float>(rr, cc) == 0.f)
          obstacles.at<unsigned char>(rr, cc) = 0;

    ::cv::Mat distance_field(map.rows, map.cols, CV_32FC1, 1);
    ::cv::distanceTransform(obstacles,
                            distance_field,
                            CV_DIST_L2,
                            CV_DIST_MASK_PRECISE);

    return distance_field;
  }

  /* *********************************************************************************** */
  inline double queryDistanceField(const cv::Mat& distance_field,
                                   const double& x,
                                   const double& y)
  {
    int col = static_cast<int>(x);
    int row = static_cast<int>(y); // y is already in correct coordinate frame

    bool within_cols = col < distance_field.cols && col >= 0;
    bool within_rows = row < distance_field.rows && row >= 0;

    if (within_rows && within_cols)
    {
    //HACK
    //cv::Mat test = distance_field;
    //cv::cvtColor(test, test, CV_GRAY2BGR);

    //cv::Scalar color(1.0f, 1.0f, 0.0f);
    //cv::circle(test, // image
    //          cv::Point(col, row), // center
    //          5,
    //          color, // color
    //          -1, // filled
    //          8); // line type

    //cv::imshow("test2", test);
    //cv::waitKey(1);
    //HACK

      return static_cast<double>(distance_field.at<float>(row, col));
    }

    // If query is not within map, return -1.0
    return -1.0;
  }

};

#endif
