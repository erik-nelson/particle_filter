#ifndef _FILE_UTILS_H_
#define _FILE_UTILS_H_

#include <opencv2/core/core.hpp>
#include <boost/lexical_cast.hpp>
#include <boost/foreach.hpp>
#include <boost/tokenizer.hpp>

#include <map>
#include <stdio.h>
#include <fstream>
#include "LocalizationUtils.h"

namespace file_utils
{
  /* *********************************************************************************** */
  template <typename T>
  struct Parameter
  {
    T value;
    inline const T& operator()() const
    {
      return value;
    }
  };

  /* *********************************************************************************** */
  typedef std::map< std::string, Parameter<double> > parameter_list_d;

  /* *********************************************************************************** */
  template <typename T>
  static bool loadParameters(const std::string& config_file,
                             const std::vector<std::string>& params,
                             std::map< std::string, Parameter<T> > & out)
  {
    out.clear();

    printf("[FileUtils.h]: Loading parameters from file %s\n", config_file.c_str());

    // Open the parameter file
    ::cv::FileStorage fs;
    fs.open(config_file, ::cv::FileStorage::READ);
    if (!fs.isOpened())
    {
      printf("[ERROR: FileUtils.h]: Unable to load parameters from file %s\n",
             config_file.c_str());
      return false;
    }

    // Load parameters from the .yml file into the std::map
    for (std::vector<std::string>::const_iterator it = params.begin(); it != params.end(); ++it)
    {
      Parameter<T> p;
      fs[*it] >> p.value;
      out[*it] = p;
    }

    fs.release();

    printf("[FileUtils.h]: Successfully loaded parameters\n");
    return true;
  }

  /* *********************************************************************************** */
  static bool loadMap(const std::string& config_file,
                      const std::string& map_identifier,
                      localization_utils::Map& out)
  {
    out.map.release();
    out.distance_field.release();

    ::cv::FileStorage fs;
    fs.open(config_file, ::cv::FileStorage::READ);
    if (!fs.isOpened())
    {
      printf("[ERROR: FileUtils.h]: Unable to locate map filepath from file %s\n",
             config_file.c_str());
      return false;
    }

    // Load path to map file from .yml file
    std::string map_filename;
    fs[map_identifier] >> map_filename;

    fs.release();

    // Load the map file into a cv::Mat
    std::ifstream map_stream(map_filename.c_str());
    if (!map_stream.good())
    {
      printf("[ERROR: FileUtils.h]: Unable to read map from file %s\n",
             map_filename.c_str());
      return false;
    }

    printf("[FileUtils.h]: Reading map from file %s\n",
           map_filename.c_str());

    std::string line;
    unsigned int map_x, map_y, shift_x, shift_y;
    unsigned int min_x, max_x, min_y, max_y;
    double resolution;
    std::vector< std::vector< double > > pixels;

    while (!map_stream.eof())
    {
      std::getline(map_stream, line);
      if (line.find("robot_specifications->global_mapsize_x") != std::string::npos)
      {
        std::string::size_type s = line.find_last_of(' ');
      }
      else if (line.find("robot_specifications->global_mapsize_y") != std::string::npos)
      {
        std::string::size_type s = line.find_last_of(' ');
      }
      else if (line.find("robot_specifications->resolution") != std::string::npos)
      {
        std::string::size_type s = line.find_last_of(' ');
        resolution = boost::lexical_cast<double>(line.substr(s+1, line.size()-1));
      }
      else if (line.find("robot_specifications->autoshifted_x") != std::string::npos)
      {
        std::string::size_type s = line.find_last_of(' ');
        shift_x = boost::lexical_cast<unsigned int>(line.substr(s+1, line.size()-1));
      }
      else if (line.find("robot_specifications->autoshifted_y") != std::string::npos)
      {
        std::string::size_type s = line.find_last_of(' ');
        shift_y = boost::lexical_cast<unsigned int>(line.substr(s+1, line.size()-1));
      }
      else if (line.empty())
        ;
      else if (line.find("global_map[0]") != std::string::npos)
      {
        std::string::size_type s1 = line.find_last_of(": ");
        std::string::size_type s2 = line.find_last_of(" ");
        map_x = boost::lexical_cast<unsigned int>(line.substr(s1+1, s1+3));
        map_y = boost::lexical_cast<unsigned int>(line.substr(s2+1, s2+3));
        printf("[FileUtils.h]: Map size: %d %d\n", map_x, map_y);
      }
      else
      {
        boost::char_separator<char> sep(" ");
        boost::tokenizer< boost::char_separator<char> > tokens(line, sep);
        std::vector<double> pixel_row;
        BOOST_FOREACH(const std::string& t, tokens)
        {
          pixel_row.push_back(boost::lexical_cast<double>(t));
        }
        pixels.push_back(pixel_row);
      }
    }

    out.map.create(map_y, map_x, CV_32FC1);
    min_x = map_x;
    max_x = 0;
    min_y = map_y;
    max_y = 0;
    for (unsigned int xx = 0; xx < map_x; ++xx)
      for (unsigned int yy = 0; yy < map_y; ++yy)
      {
        if ((pixels[xx][yy] > 0.01 && pixels[xx][yy] < 0.99) || pixels[xx][yy] < 0.0)
          out.map.at<float>(map_y - yy - 1, xx) = 0.5;
        else
        {
          min_x = xx < min_x ? xx : min_x;
          min_y = yy < min_y ? yy : min_y;
          max_x = xx > max_x ? xx : max_x;
          max_y = yy > max_y ? yy : max_y;
          if (pixels[xx][yy] <= 0.01)
            out.map.at<float>(map_y - yy - 1, xx) = 0.0;
          else
            out.map.at<float>(map_y - yy - 1, xx) = 1.0;
        }
      }

    // We will flip X and Y when using particles
    out.max_x = max_x;
    out.min_x = min_x;
    out.max_y = max_y;
    out.min_y = min_y;
    out.origin_x = shift_x;
    out.origin_y = shift_y;
    out.resolution = resolution;

    // Create a distance field so for localization so that we
    // don't require expensive raycasting
    out.distance_field = localization_utils::createDistanceField(out.map);

    printf("[FileUtils.h]: Successfully loaded map\n");
    return true;

  }

    /* *********************************************************************************** */
  static bool loadData(const std::string& config_file,
                       const std::string& data_identifier,
                       localization_utils::DataStream& out)
  {
    ::cv::FileStorage fs;
    fs.open(config_file, ::cv::FileStorage::READ);
    if (!fs.isOpened())
    {
      printf("[ERROR: FileUtils.h]: Unable to locate data filepath from file %s\n",
             config_file.c_str());
      return false;
    }

    // Load path to data file from .yml file
    std::string data_filename;
    fs[data_identifier] >> data_filename;

    fs.release();

    // Load the data into a DataStream
    std::ifstream data_stream(data_filename.c_str());
    if (!data_stream.good())
    {
      printf("[ERROR: FileUtils.h]: Unable to read data from file %s\n",
             data_filename.c_str());
      return false;
    }

    printf("[FileUtils.h]: Reading data from file %s\n",
           data_filename.c_str());

    std::string line;
    while (!data_stream.eof())
    {
      std::getline(data_stream, line);

      // Tokenize the line
      boost::char_separator<char> sep(" ");
      boost::tokenizer< boost::char_separator<char> > tokens(line, sep);
      std::vector<double> values;
      BOOST_FOREACH(const std::string& t, tokens)
      {
        if (t[0] != 'L' && t[0] != 'O')
          values.push_back(boost::lexical_cast<double>(t));
      }

      if (line[0] == 'L')
      {
        localization_utils::LaserMessage msg;
        msg.xb = values[0];
        msg.yb = values[1];
        msg.yawb = values[2];
        msg.xl = values[3];
        msg.yl = values[4];
        msg.yawl = values[5];

        msg.ranges.clear();
        for (unsigned int ii = 6; ii < 186; ++ii)
          msg.ranges.push_back(values[ii]);

        msg.time = values[186];

        localization_utils::LaserMessage::ConstPtr p_msg(new localization_utils::LaserMessage(msg));
        out.addLaserMessage(p_msg);
      }
      if (line[0] == 'O')
      {
        localization_utils::OdometryMessage msg;
        msg.x = values[0];
        msg.y = values[1];
        msg.yaw = values[2];
        msg.time = values[3];

        localization_utils::OdometryMessage::ConstPtr p_msg(new localization_utils::OdometryMessage(msg));
        out.addOdometryMessage(p_msg);
      }
    }

    out.sortPendingMessages();

    printf("[FileUtils.h]: Successfully loaded data\n");

    return true;
  }
};

#endif
