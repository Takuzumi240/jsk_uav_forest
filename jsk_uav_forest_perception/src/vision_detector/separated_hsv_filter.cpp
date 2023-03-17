// -*- mode: c++ -*-
/*********************************************************************
 * Software License Agreement (BSD License)
 *
 *  Copyright (c) 2017, JSK Lab
 *  All rights reserved.
 *
 *  Redistribution and use in source and binary forms, with or without
 *  modification, are permitted provided that the following conditions
 *  are met:
 *
 *   * Redistributions of source code must retain the above copyright
 *     notice, this list of conditions and the following disclaimer.
 *   * Redistributions in binary form must reproduce the above
 *     copyright notice, this list of conditions and the following
 *     disclaimer in the documentation and/o2r other materials provided
 *     with the distribution.
 *   * Neither the name of the JSK Lab nor the names of its
 *     contributors may be used to endorse or promote products derived
 *     from this software without specific prior written permission.
 *
 *  THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS
 *  "AS IS" AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT
 *  LIMITED TO, THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS
 *  FOR A PARTICULAR PURPOSE ARE DISCLAIMED. IN NO EVENT SHALL THE
 *  COPYRIGHT OWNER OR CONTRIBUTORS BE LIABLE FOR ANY DIRECT, INDIRECT,
 *  INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING,
 *  BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES;
 *  LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER
 *  CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT
 *  LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN
 *  ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE
 *  POSSIBILITY OF SUCH DAMAGE.
 *********************************************************************/

/* base class */
#include <jsk_uav_forest_perception/vision_detector/separated_hsv_filter.h>

using namespace std;

namespace vision_detection
{
  void SeparatedHsvFilter::initialize(ros::NodeHandle nh, ros::NodeHandle pnh)
  {
    vision_detection::IndependentHsvFilter::initialize(nh, pnh);
    ROS_ERROR("separated HSV filter");
    pnh_.param("max_distance", max_distance_, 6.0);
    cnt_ = 0;
  }

  bool SeparatedHsvFilter::filter(const sensor_msgs::ImageConstPtr& image_msg, const sensor_msgs::LaserScanConstPtr& scan_msg, int& target_tree_index)
  {

    if(cnt_%5 == 0)
      {
        cv::Mat src_image = cv_bridge::toCvCopy(image_msg, image_msg->encoding)->image;

        /* general hsv filter */
        cv::Mat hsv_image_mask;
        hsvFilter(src_image, image_msg->encoding, hsv_image_mask);

        /* find countour */
        std::vector<std::vector<cv::Point> > contours;
        std::vector<cv::Vec4i> hierarchy;
        cv::findContours(hsv_image_mask, contours, hierarchy, CV_RETR_EXTERNAL, CV_CHAIN_APPROX_SIMPLE, cv::Point(0,0));

        if(contours.size() == 0) return false;

        /* find the biggest contour */
        float max_contour_area = 0;
        float max_contour = 0;
        for(int i = 0; i < contours.size(); ++i)
          {
            if (cv::contourArea(contours[i]) > max_contour_area)
              {
                max_contour_area = cv::contourArea(contours[i]);
                max_contour = i;
              }
          }

        cv::Moments contour_moments = cv::moments(contours[max_contour], true);

        /* publish the base information */
        geometry_msgs::PointStamped contour_center;
        contour_center.header = image_msg->header;
        contour_center.point.x = contour_moments.m10 / contour_moments.m00;
        contour_center.point.y = contour_moments.m01 / contour_moments.m00;
        pub_target_image_center_.publish(contour_center);

        /* draw */
        cv::circle(src_image, cv::Point(contour_center.point.x, contour_center.point.y), 100, cv::Scalar(0, 0, 255), 10);
        cv::drawContours(src_image, contours, max_contour, contour_color_, 10);

        /* publish */
        pub_target_image_.publish(cv_bridge::CvImage(image_msg->header,
                                                     image_msg->encoding,
                                                     src_image).toImageMsg());

        pub_mask_image_.publish(cv_bridge::CvImage(image_msg->header,
                                                   "mono8",
                                                   hsv_image_mask).toImageMsg());
      }

    vector<int> cluster_index;
    for (size_t i = 0; i < scan_msg->ranges.size(); ++i)
      {
        if(scan_msg->ranges[i] > 0)
            cluster_index.push_back(i);
      }

    if(cluster_index.empty())
      return false;

    float min_direction = 1e6;
    float target_tree_laser_distance = 0.0;
    for (auto it = cluster_index.begin(); it != cluster_index.end(); ++it)
      {
        float laser_direction = *it * scan_msg->angle_increment + scan_msg->angle_min;
        float laser_distance = scan_msg->ranges[*it];

        if(laser_distance < max_distance_)
          {
            if(fabs(laser_direction) < min_direction)
              {
                target_tree_index = *it;
                min_direction = fabs(laser_direction);
                target_tree_laser_distance = laser_distance;
              }
          }
      }

    cnt_ += 1;
    ROS_ERROR("find the tree %f, %f", min_direction, target_tree_laser_distance);
    return true;
  }

}

/* plugin registration */
#include <pluginlib/class_list_macros.h>
PLUGINLIB_EXPORT_CLASS(vision_detection::SeparatedHsvFilter, vision_detection::DetectorBase);
