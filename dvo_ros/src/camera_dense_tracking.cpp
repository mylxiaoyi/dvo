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

#include <boost/bind.hpp>
#include <cv_bridge/cv_bridge.h>

#include <tf/transform_broadcaster.h>
#include <tf_conversions/tf_eigen.h>

#include <dvo/core/surface_pyramid.h>
#include <dvo/util/stopwatch.h>
#include <dvo/visualization/visualizer.h>
//#include <dvo/visualization/pcl_camera_trajectory_visualizer.h>

#include <dvo_ros/camera_dense_tracking.h>
#include <dvo_ros/util/configtools.h>
#include <dvo_ros/util/util.h>
#include <dvo_ros/visualization/ros_camera_trajectory_visualizer.h>

namespace dvo_ros
{

using namespace dvo;
using namespace dvo::core;
using namespace dvo::util;

CameraDenseTracker::CameraDenseTracker (ros::NodeHandle& nh, ros::NodeHandle& nh_private)
: /*CameraBase (nh, nh_private),*/ tracker_cfg (DenseTracker::getDefaultConfig ()),
  frames_since_last_success (0), reconfigure_server_ (nh_private),
  vis_ (new dvo::visualization::NoopCameraTrajectoryVisualizer ()),
  use_dense_tracking_estimate_ (false), connected (false), nh_ (nh)
{
    ROS_INFO ("CameraDenseTracker::ctor(...)");

    rgb_image_subscriber_ =
    new message_filters::Subscriber<sensor_msgs::Image> (nh,
                                                         "camera/rgb/image_raw", 1);
    depth_image_subscriber_ = new message_filters::Subscriber<sensor_msgs::Image> (nh, "camera/depth/image_raw",
                                                                                   1);

    my_synchronizer_ = new message_filters::Synchronizer<MyRGBDWithCameraInfoPolicy> (
    MyRGBDWithCameraInfoPolicy (10), *rgb_image_subscriber_, *depth_image_subscriber_);

    startSynchronizedImageStream();

    pose_pub_ = nh.advertise<geometry_msgs::PoseStamped> ("rgbd/pose", 1);

    ReconfigureServer::CallbackType reconfigure_server_callback =
    boost::bind (&CameraDenseTracker::handleConfig, this, _1, _2);
//    reconfigure_server_.setCallback (reconfigure_server_callback);

    //    dvo_ros::util::tryGetTransform(from_baselink_to_asus, tl, "base_link",
    //    "asus");

    //    from_baselink_to_asus.setIdentity ();
    //    ROS_INFO_STREAM ("transformation: base_link -> asus"
    //                     << std::endl
    //                     << from_baselink_to_asus.matrix ());

//    pose_sub_ = nh.subscribe<geometry_msgs::PoseWithCovarianceStamped> (
//    "pelican/pose", 1, &CameraDenseTracker::handlePose, this);

    latest_absolute_transform_.setIdentity ();
    accumulated_transform.setIdentity ();

    dvo::visualization::Visualizer::instance ()
    .enabled (true)
    .useExternalWaitKey (false)
    .save (false);

    vis_ =
    std::make_shared<dvo_ros::visualization::RosCameraTrajectoryVisualizer> (nh_);
}

CameraDenseTracker::~CameraDenseTracker ()
{
    //    delete vis_;
    vis_.reset ();
}

bool CameraDenseTracker::hasChanged (const sensor_msgs::CameraInfo::ConstPtr& camera_info_msg)
{
    return width != camera_info_msg->width || height != camera_info_msg->height;
}

bool CameraDenseTracker::hasChanged (const cv::Mat& img)
{
    return width != img.cols || height != img.rows;
}

void CameraDenseTracker::reset (const sensor_msgs::CameraInfo::ConstPtr& camera_info_msg)
{
    // IntrinsicMatrix intrinsics =
    // IntrinsicMatrix::create(camera_info_msg->K[0], camera_info_msg->K[4],
    // camera_info_msg->K[2], camera_info_msg->K[5]);
    IntrinsicMatrix intrinsics =
    IntrinsicMatrix::create (camera_info_msg->P[0], camera_info_msg->P[5],
                             camera_info_msg->P[2], camera_info_msg->P[6]);
    ROS_INFO_STREAM ("before reset tracker");
    tracker.reset (new DenseTracker (intrinsics, tracker_cfg));
    ROS_INFO_STREAM ("after reset tracker");

    static RgbdImagePyramid* const __null__ = 0;

    reference.reset (__null__);
    current.reset (__null__);

    width = camera_info_msg->width;
    height = camera_info_msg->height;

    vis_->reset ();
}

void CameraDenseTracker::reset (const cv::Mat& img)
{
    // IntrinsicMatrix intrinsics =
    // IntrinsicMatrix::create(camera_info_msg->K[0], camera_info_msg->K[4],
    // camera_info_msg->K[2], camera_info_msg->K[5]);
    IntrinsicMatrix intrinsics =
    IntrinsicMatrix::create (517.306408, 516.469215, 318.643040, 255.313989);

    tracker.reset (new DenseTracker (intrinsics, tracker_cfg));

    static RgbdImagePyramid* const __null__ = 0;

    reference.reset (__null__);
    current.reset (__null__);

    width = img.cols;
    height = img.rows;

    vis_->reset ();
}

void CameraDenseTracker::handleConfig (dvo_ros::CameraDenseTrackerConfig& config, uint32_t level)
{
    ROS_INFO_STREAM ("hello from handleConfig and level = " << level);
    if (level == 0) return;

    if (level & CameraDenseTracker_RunDenseTracking)
    {
        if (config.run_dense_tracking)
        {
            startSynchronizedImageStream ();
        }
        else
        {
            ROS_INFO_STREAM("stop synchronized image stream");
            stopSynchronizedImageStream ();

            // force reset of tracker
            width = 0;
            height = 0;
        }
    }

    if (!config.run_dense_tracking && config.use_dense_tracking_estimate)
    {
        config.use_dense_tracking_estimate = false;
    }

    use_dense_tracking_estimate_ = config.use_dense_tracking_estimate;

    if (level & CameraDenseTracker_ConfigParam)
    {
        // lock tracker so we don't reconfigure it while it is running
        boost::mutex::scoped_lock lock (tracker_mutex_);

        // fix config, so we don't die by accident
        if (config.coarsest_level < config.finest_level)
        {
            config.finest_level = config.coarsest_level;
        }

        dvo_ros::util::updateConfigFromDynamicReconfigure (config, tracker_cfg);

        // we are called in the ctor as well, but at this point we don't have a
        // tracker instance
        if (tracker)
        {
            tracker->configure ();
        }

        ROS_INFO_STREAM ("reconfigured tracker, config ( " << tracker_cfg
                                                           << " )");
    }

    if (level & CameraDenseTracker_MiscParam)
    {
        ROS_INFO_STREAM ("hello here");
        if (vis_)
        {
            vis_->reset ();
            //            delete vis_;
            vis_.reset ();
        }

        if (config.reconstruction)
        {
            //            vis_ = new
            //            dvo::visualization::PclCameraTrajectoryVisualizer ();
            vis_ =
            std::make_shared<dvo_ros::visualization::RosCameraTrajectoryVisualizer> (nh_);
            ROS_INFO_STREAM ("after new vis");
        }
        else
        {
            vis_ = std::make_shared<dvo::visualization::NoopCameraTrajectoryVisualizer> ();
        }
    }
}

void CameraDenseTracker::handlePose (const geometry_msgs::PoseWithCovarianceStampedConstPtr& pose)
{
    tf::Transform tmp;

    tf::poseMsgToTF (pose->pose.pose, tmp);
    tf::transformTFToEigen (tmp, latest_absolute_transform_);

    if (!use_dense_tracking_estimate_)
        publishPose (pose->header, latest_absolute_transform_,
                     "baselink_estimate");
}

// void CameraDenseTracker::handleImages (const sensor_msgs::ImageConstPtr&
// rgb_image_msg,
//                                       const sensor_msgs::ImageConstPtr&
//                                       depth_image_msg,
//                                       const sensor_msgs::CameraInfoConstPtr&
//                                       rgb_camera_info_msg,
//                                       const sensor_msgs::CameraInfoConstPtr&
//                                       depth_camera_info_msg)
//{
//    ROS_INFO_STREAM ("hello from CameraDenseTracker::handleImages");

//    static stopwatch sw_callback ("callback");
//    sw_callback.start ();

//    // lock tracker so no one can reconfigure it
//    boost::mutex::scoped_lock lock (tracker_mutex_);

//    // different size of rgb and depth image
//    if (depth_camera_info_msg->width != rgb_camera_info_msg->width ||
//        depth_camera_info_msg->height != rgb_camera_info_msg->height)
//    {
//        ROS_WARN ("RGB and depth image have different size!");

//        return;
//    }

//    // something has changed
//    if (hasChanged (rgb_camera_info_msg))
//    {
//        ROS_WARN ("RGB image size has changed, resetting tracker!");

//        reset (rgb_camera_info_msg);
//    }

//    cv::Mat intensity, depth;
//    cv::Mat rgb_in = cv_bridge::toCvShare (rgb_image_msg)->image;
//    if (rgb_in.channels () == 3)
//    {
//        cv::Mat tmp;
//        cv::cvtColor (rgb_in, tmp, CV_BGR2GRAY, 1);

//        tmp.convertTo (intensity, CV_32F);
//    }
//    else
//    {
//        rgb_in.convertTo (intensity, CV_32F);
//    }

//    cv::Mat depth_in = cv_bridge::toCvShare (depth_image_msg)->image;

//    if (depth_in.type () == CV_16UC1)
//    {
//        ROS_INFO_STREAM ("before SurfacePyramid::convertRawDepthImageSse");
//        SurfacePyramid::convertRawDepthImageSse (depth_in, depth, 0.001);
//        ROS_INFO_STREAM ("after SurfacePyramid::convertRawDepthImageSse");
//    }
//    else
//    {
//        depth = depth_in;
//    }

//    reference.swap (current);
//    ROS_INFO_STREAM ("before current.reset");
//    current.reset (new RgbdImagePyramid (intensity, depth));
//    ROS_INFO_STREAM ("after current.reset");

//    // time delay compensation TODO: use driver settings instead
//    std_msgs::Header h = rgb_image_msg->header;
//    // h.stamp -= ros::Duration(0.05);

//    static Eigen::Affine3d first;

//    if (!reference)
//    {
//        accumulated_transform = latest_absolute_transform_ *
//        from_baselink_to_asus;
//        first = accumulated_transform;

//        //        vis_->camera ("first")
//        //        ->color (dvo::visualization::Color::blue ())
//        //        .update (current->level (0), tracker->intrinsics (0),
//        //        accumulated_transform)
//        //        .show ();

//        return;
//    }

//    Eigen::Affine3d transform, last_transform;
//    last_transform.setIdentity ();
//    tracker->updateLastTransform (last_transform);

//    static stopwatch sw_match ("match", 100);
//    sw_match.start ();

//    ROS_INFO_STREAM ("before tracker->match");
//    bool success = tracker->match (*reference, *current, transform);
//    ROS_INFO_STREAM ("after tracker->match");

//    sw_match.stopAndPrint ();

//    if (success)
//    {
//        frames_since_last_success = 0;
//        accumulated_transform = accumulated_transform * transform;

//        Eigen::Matrix<double, 6, 6> covariance;

//        // tracker->getCovarianceEstimate(covariance);

//        // std::cerr << covariance << std::endl << std::endl;

//        //        vis_->trajectory ("estimate")->color
//        //        (dvo::visualization::Color::red ()).add (
//        //        accumulated_transform);

//        //        vis_->camera ("current")
//        //        ->color (dvo::visualization::Color::red ())
//        //        .update (current->level (0), tracker->intrinsics (0),
//        //        accumulated_transform)
//        //        .show ();
//    }
//    else
//    {
//        frames_since_last_success++;
//        reference.swap (current);
//        ROS_WARN ("fail");
//    }

//    publishTransform (h, accumulated_transform * from_baselink_to_asus.inverse
//    (),
//                      "base_link_estimate");
//    //  publishTransform(rgb_image_msg->header, first_transform.inverse() *
//    //  accumulated_transform, "asus_estimate");

//    if (use_dense_tracking_estimate_)
//    {
//        publishPose (h, accumulated_transform * from_baselink_to_asus.inverse
//        (),
//                     "baselink_estimate");
//    }

//    sw_callback.stopAndPrint ();
//}

void CameraDenseTracker::publishTransform (const std_msgs::Header& header,
                                           const Eigen::Affine3d& transform,
                                           const std::string frame)
{
    static tf::TransformBroadcaster tb;

    tf::StampedTransform tf_transform;
    tf_transform.frame_id_ = "world";
    tf_transform.child_frame_id_ = frame;
    tf_transform.stamp_ = header.stamp;

    tf::transformEigenToTF (transform, tf_transform);

    tb.sendTransform (tf_transform);
}

void CameraDenseTracker::publishPose (const std_msgs::Header& header,
                                      const Eigen::Affine3d& transform,
                                      const std::string frame)
{
    //    if (pose_pub_.getNumSubscribers () == 0) return;

    geometry_msgs::PoseStampedPtr msg (new geometry_msgs::PoseStamped);

    static int seq = 1;

    msg->header.seq = seq++;
    msg->header.frame_id = frame;
    msg->header.stamp = header.stamp;

    tf::Transform tmp;

    tf::transformEigenToTF (transform, tmp);
    tf::poseTFToMsg (tmp, msg->pose);

    //    msg->pose.covariance.assign (0.0);

    pose_pub_.publish (msg);
}

void CameraDenseTracker::handleImages (const sensor_msgs::ImageConstPtr& rgb_image_msg,
                                       const sensor_msgs::ImageConstPtr& depth_image_msg)
{
//    ROS_INFO_STREAM("hello from handleImages");
    cv::Mat rgb_in = cv_bridge::toCvCopy (rgb_image_msg)->image;
    cv::Mat depth_in = cv_bridge::toCvCopy (depth_image_msg)->image;

    //    ROS_INFO_STREAM ("rgb_in rows = " << rgb_in.rows << ", cols = " <<
    //    rgb_in.cols);
    //    ROS_INFO_STREAM ("depth_in rows = " << depth_in.rows
    //                                        << ", cols = " << depth_in.cols);

    static stopwatch sw_callback ("callback");
    sw_callback.start ();

    // lock tracker so no one can reconfigure it
    boost::mutex::scoped_lock lock (tracker_mutex_);

    // different size of rgb and depth image
    if (rgb_in.rows != depth_in.rows || rgb_in.cols != depth_in.cols)
    {
        ROS_WARN ("RGB and depth image have different size!");

        return;
    }

    // something has changed
    if (hasChanged (rgb_in))
    {
        ROS_WARN ("RGB image size has changed, resetting tracker!");

        reset (rgb_in);
    }

    cv::Mat intensity, depth;
    //    cv::Mat rgb_in = cv_bridge::toCvShare (rgb_image_msg)->image;
    if (rgb_in.channels () == 3)
    {
        cv::Mat tmp;
        cv::cvtColor (rgb_in, tmp, CV_BGR2GRAY, 1);

        tmp.convertTo (intensity, CV_32F);
    }
    else
    {
        rgb_in.convertTo (intensity, CV_32F);
    }

    //    cv::Mat depth_in = cv_bridge::toCvShare (depth_image_msg)->image;

    if (depth_in.type () == CV_16UC1)
    {
        SurfacePyramid::convertRawDepthImageSse (depth_in, depth, 0.001);
    }
    else
    {
        depth = depth_in;
    }

    reference.swap (current);
    current.reset (new RgbdImagePyramid (intensity, depth));

    // time delay compensation TODO: use driver settings instead
    std_msgs::Header h = rgb_image_msg->header;
    // h.stamp -= ros::Duration(0.05);

    static Eigen::Affine3d first;

    if (!reference)
    {
        accumulated_transform = latest_absolute_transform_ /** from_baselink_to_asus*/;
        first = accumulated_transform;

        ROS_INFO_STREAM("first camera");
        vis_->camera ("first")
        ->color (dvo::visualization::Color::blue ())
        .update (current->level (0), tracker->intrinsics (0), accumulated_transform)
        .show ();

        return;
    }

    Eigen::Affine3d transform, last_transform;
    last_transform.setIdentity ();
    tracker->updateLastTransform (last_transform);

    static stopwatch sw_match ("match", 100);
    sw_match.start ();

    bool success = tracker->match (*reference, *current, transform);

    sw_match.stopAndPrint ();

    if (success)
    {
        frames_since_last_success = 0;
        accumulated_transform = accumulated_transform * transform;

        Eigen::Matrix<double, 6, 6> covariance;

        // tracker->getCovarianceEstimate(covariance);

        // std::cerr << covariance << std::endl << std::endl;

        vis_->trajectory ("estimate")->color (dvo::visualization::Color::red ()).add (accumulated_transform);

        vis_->camera ("current")
        ->color (dvo::visualization::Color::red ())
        .update (current->level (0), tracker->intrinsics (0), accumulated_transform)
        .show ();
    }
    else
    {
        frames_since_last_success++;
        reference.swap (current);
        ROS_WARN ("fail");
    }

    publishTransform (h, accumulated_transform /* * from_baselink_to_asus.inverse ()*/,
                      "baselink_estimate");
    //  publishTransform(rgb_image_msg->header, first_transform.inverse()
    //    *
    //  accumulated_transform, "asus_estimate");

    //    if (use_dense_tracking_estimate_)
    if (success)
    {
        ROS_INFO_STREAM("publish pose");
        publishPose (h, accumulated_transform /* * from_baselink_to_asus.inverse ()*/, "world");
    }

    sw_callback.stopAndPrint ();
}

void CameraDenseTracker::startSynchronizedImageStream ()
{
    if (!connected)
    {
        //        connection = synchronizer_->registerCallback (
        //        boost::bind (&CameraBase::handleImages, this, _1, _2, _3,
        //        _4));
        //        connected = true;

        connection = my_synchronizer_->registerCallback (
        boost::bind (&CameraDenseTracker::handleImages, this, _1, _2));
        connected = true;
        ROS_INFO_STREAM ("image stream started");
    }
}

void CameraDenseTracker::stopSynchronizedImageStream ()
{
    if (connected)
    {
        connection.disconnect ();
        connected = false;
    }
}

bool CameraDenseTracker::isSynchronizedImageStreamRunning ()
{
    return connected;
}

} /* namespace dvo_ros */
