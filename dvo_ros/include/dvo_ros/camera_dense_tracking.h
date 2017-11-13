/**
 *  This file is part of dvo.
 *
 *  Copyright 2012 Christian Kerl <christian.kerl@in.tum.de> (Technical
 *University of Munich)
 *  For more information see <http://vision.in.tum.de/data/software/dvo>.
 *
 *  dvo is free software: you can redistribute it and/or modify
 *  it under the terms of the GNU General Public License as published by
 *  the Free Software Foundation, either version 3 of the License, or
 *  (at your option) any later version.
 *
 *  dvo is distributed in the hope that it will be useful,
 *  but WITHOUT ANY WARRANTY; without even the implied warranty of
 *  MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
 *  GNU General Public License for more details.
 *
 *  You should have received a copy of the GNU General Public License
 *  along with dvo.  If not, see <http://www.gnu.org/licenses/>.
 */

#ifndef CAMERA_DENSE_TRACKING_H_
#define CAMERA_DENSE_TRACKING_H_

#include <boost/shared_ptr.hpp>
#include <boost/thread.hpp>

#include <ros/console.h>
#include <ros/ros.h>

#include <tf/transform_listener.h>

#include <geometry_msgs/PoseWithCovarianceStamped.h>
#include <sensor_msgs/CameraInfo.h>
#include <sensor_msgs/Image.h>

#include <image_transport/image_transport.h>

#include <Eigen/Geometry>

#include <dynamic_reconfigure/server.h>

//#include <dvo_ros/camera_base.h>
#include <dvo_ros/CameraDenseTrackerConfig.h>

#include <dvo/core/intrinsic_matrix.h>
#include <dvo/core/rgbd_image.h>
#include <dvo/dense_tracking.h>
#include <dvo/visualization/camera_trajectory_visualizer.h>

#include <message_filters/subscriber.h>
#include <message_filters/sync_policies/approximate_time.h>
#include <message_filters/synchronizer.h>

namespace dvo_ros
{

typedef message_filters::sync_policies::ApproximateTime<sensor_msgs::Image, sensor_msgs::Image> MyRGBDWithCameraInfoPolicy;

class CameraDenseTracker //: public CameraBase
{
private:
    typedef dynamic_reconfigure::Server<dvo_ros::CameraDenseTrackerConfig> ReconfigureServer;

    uint32_t width;
    uint32_t height;

    boost::shared_ptr<dvo::DenseTracker> tracker;
    dvo::DenseTracker::Config tracker_cfg;
    boost::shared_ptr<dvo::core::RgbdImagePyramid> current, reference;

    Eigen::Affine3d accumulated_transform, /*from_baselink_to_asus,*/ latest_absolute_transform_;

    size_t frames_since_last_success;

    tf::TransformListener tl;

    ros::Publisher pose_pub_;
    ros::Subscriber pose_sub_;

    ReconfigureServer reconfigure_server_;

    std::shared_ptr<dvo::visualization::CameraTrajectoryVisualizerInterface> vis_;

    bool use_dense_tracking_estimate_;
    boost::mutex tracker_mutex_;

    message_filters::Subscriber<sensor_msgs::Image>* rgb_image_subscriber_;
    message_filters::Subscriber<sensor_msgs::Image>* depth_image_subscriber_;
    message_filters::Subscriber<sensor_msgs::CameraInfo>* rgb_camera_info_subscriber_;
    message_filters::Subscriber<sensor_msgs::CameraInfo>* depth_camera_info_subscriber_;

    message_filters::Synchronizer<MyRGBDWithCameraInfoPolicy>* my_synchronizer_;

    bool hasChanged (const sensor_msgs::CameraInfo::ConstPtr& camera_info_msg);
    bool hasChanged (const cv::Mat& img);
    void reset (const sensor_msgs::CameraInfo::ConstPtr& camera_info_msg);
    void reset (const cv::Mat& img);

    void publishTransform (const std_msgs::Header& header,
                           const Eigen::Affine3d& transform,
                           const std::string frame);
    void publishPose (const std_msgs::Header& header,
                      const Eigen::Affine3d& transform,
                      const std::string frame);

    bool isSynchronizedImageStreamRunning ();

    void startSynchronizedImageStream ();
    void stopSynchronizedImageStream ();

    message_filters::Connection connection;
    bool connected;

    ros::NodeHandle nh_;

public:
    CameraDenseTracker (ros::NodeHandle& nh, ros::NodeHandle& nh_private);
    virtual ~CameraDenseTracker ();

    //    virtual void
    //    handleImages (const sensor_msgs::ImageConstPtr& rgb_image_msg,
    //                  const sensor_msgs::ImageConstPtr& depth_image_msg,
    //                  const sensor_msgs::CameraInfoConstPtr&
    //                  rgb_camera_info_msg,
    //                  const sensor_msgs::CameraInfoConstPtr&
    //                  depth_camera_info_msg);

    virtual void handleImages (const sensor_msgs::ImageConstPtr& rgb_image_msg,
                               const sensor_msgs::ImageConstPtr& depth_image_msg);

    void handlePose (const geometry_msgs::PoseWithCovarianceStampedConstPtr& pose);

    void handleConfig (dvo_ros::CameraDenseTrackerConfig& config, uint32_t level);
};

} /* namespace dvo_ros */
#endif /* CAMERA_DENSE_TRACKING_H_ */
