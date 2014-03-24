// This file is part of SVO - Semi-direct Visual Odometry.
//
// Copyright (C) 2014 Christian Forster <forster at ifi dot uzh dot ch>
// (Robotics and Perception Group, University of Zurich, Switzerland).
//
// SVO is free software: you can redistribute it and/or modify it under the
// terms of the GNU General Public License as published by the Free Software
// Foundation, either version 3 of the License, or any later version.
//
// SVO is distributed in the hope that it will be useful, but WITHOUT ANY
// WARRANTY; without even the implied warranty of MERCHANTABILITY or FITNESS
// FOR A PARTICULAR PURPOSE. See the GNU General Public License for more details.
//
// You should have received a copy of the GNU General Public License
// along with this program.  If not, see <http://www.gnu.org/licenses/>.

#include <svo_ros/visualizer.h>
#include <svo/frame_handler_mono.h>
#include <svo/frame.h>
#include <svo/point.h>
#include <svo/map.h>
#include <svo/feature.h>
#include <svo_msgs/Info.h>
#include <geometry_msgs/PoseWithCovarianceStamped.h>
#include <sensor_msgs/image_encodings.h>
#include <cv_bridge/cv_bridge.h>
#include <tf/tf.h>
#include <iostream>
#include <fstream>
#include <ros/package.h>
#include <vikit/timer.h>
#include <vikit/output_helper.h>
#include <vikit/params_helper.h>
#include <deque>
#include <algorithm>
#include <svo_msgs/DenseInput.h>

namespace svo {

Visualizer::
Visualizer() :
    pnh_("~"),
    trace_id_(0),
    img_pub_level_(vk::getParam<int>("svo/image_publishing_level", 0)),
    img_pub_nth_(vk::getParam<int>("svo/publish_every_nth_img", 1)),
    dense_pub_nth_(vk::getParam<int>("svo/publish_every_nth_dense_input", 1))
{
  // Init ROS Marker Publishers
  pub_frames_ = pnh_.advertise<visualization_msgs::Marker>("keyframes", 10);
  pub_points_ = pnh_.advertise<visualization_msgs::Marker>("points", 1000);
  pub_pose_ = pnh_.advertise<geometry_msgs::PoseWithCovarianceStamped>("pose",10);
  pub_info_ = pnh_.advertise<svo_msgs::Info>("info", 10);
  pub_dense_ = pnh_.advertise<svo_msgs::DenseInput>("dense_input",10);

  // create video publisher
  image_transport::ImageTransport it(pnh_);
  pub_images_ = it.advertise("image", 10);
}

void Visualizer::
visualizeMarkers(const FramePtr& frame, const set<FramePtr>& core_kfs, const Map& map)
{
  if(frame != NULL)
  {
    // publish camera pose tf
    vk::output_helper::publishTfTransform(frame->T_f_w_, ros::Time(frame->timestamp_), "cam_pos", "world", br_);
    if(pub_frames_.getNumSubscribers() > 0 || pub_points_.getNumSubscribers() > 0)
    {
      vk::output_helper::publishHexacopterMarker(pub_frames_, "cam_pos", "cams", ros::Time(frame->timestamp_), 1, 0, 0.3, Vector3d(0.,0.,1.));
      vk::output_helper::publishPointMarker(pub_points_, frame->pos(), "trajectory", ros::Time::now(), trace_id_, 0, 0.006, Vector3d(0.,0.,0.5));
      publishMapRegion(core_kfs);
      removeDeletedPts(map);
    }
  }
}

void Visualizer::
publishMinimal(const cv::Mat& img, const FramePtr& frame, const FrameHandlerMono& slam, const double timestamp)
{
  ++trace_id_;
  std_msgs::Header header_msg;
  header_msg.frame_id = "/cam";
  header_msg.seq = trace_id_;
  header_msg.stamp = ros::Time(timestamp);

  // publish svo info msgs
  if(pub_info_.getNumSubscribers() > 0)
  {
    svo_msgs::Info msg_info;
    msg_info.header = header_msg;
    msg_info.processing_time = slam.lastProcessingTime();
    msg_info.keyframes.resize(slam.map().keyframes_.size());
    for(list<FramePtr>::const_iterator it=slam.map().keyframes_.begin(); it!=slam.map().keyframes_.end(); ++it)
      msg_info.keyframes.push_back((*it)->id_);
    msg_info.stage = static_cast<int>(slam.stage());
    msg_info.tracking_quality = static_cast<int>(slam.trackingQuality());
    if(frame != NULL)
      msg_info.num_matches = slam.lastNumObservations();
    else
      msg_info.num_matches = 0;
    pub_info_.publish(msg_info);
  }

  // publish pyramid-image every nth-frame
  if(frame != NULL && img_pub_nth_ > 0
      && trace_id_%img_pub_nth_ == 0 && pub_images_.getNumSubscribers() > 0)
  {
    const int scale = (1<<img_pub_level_);
    cv::Mat img_rgb(frame->img_pyr_[img_pub_level_].size(), CV_8UC3);
    cv::cvtColor(frame->img_pyr_[img_pub_level_], img_rgb, CV_GRAY2RGB);

    // during initialization, draw lines
    if(slam.stage() == FrameHandlerBase::SECOND_FRAME)
    {
      const vector<cv::Point2f>& px_ref(slam.initFeatureTrackRefPx());
      const vector<cv::Point2f>& px_cur(slam.initFeatureTrackCurPx());
      for(vector<cv::Point2f>::const_iterator it_ref=px_ref.begin(), it_cur=px_cur.begin();
          it_ref != px_ref.end(); ++it_ref, ++it_cur)
        cv::line(img_rgb,
                 cv::Point2f(it_cur->x/scale, it_cur->y/scale),
                 cv::Point2f(it_ref->x/scale, it_ref->y/scale), cv::Scalar(0,255,0), 2);
    }

    // draw features on frame
    if(img_pub_level_ == 0)
      for(Features::iterator it=frame->fts_.begin(); it!=frame->fts_.end(); ++it)
        cv::rectangle(img_rgb,
                      cv::Point2f((*it)->px[0]-2, (*it)->px[1]-2),
                      cv::Point2f((*it)->px[0]+2, (*it)->px[1]+2),
                      cv::Scalar(0,255,0), CV_FILLED);
    if(img_pub_level_ == 1)
      for(Features::iterator it=frame->fts_.begin(); it!=frame->fts_.end(); ++it)
        cv::rectangle(img_rgb,
                      cv::Point2f((*it)->px[0]/scale-1, (*it)->px[1]/scale-1),
                      cv::Point2f((*it)->px[0]/scale+1, (*it)->px[1]/scale+1),
                      cv::Scalar(0,255,0), CV_FILLED);
    else
      for(Features::iterator it=frame->fts_.begin(); it!=frame->fts_.end(); ++it)
        cv::rectangle(img_rgb,
                      cv::Point2f((*it)->px[0]/scale, (*it)->px[1]/scale),
                      cv::Point2f((*it)->px[0]/scale, (*it)->px[1]/scale),
                      cv::Scalar(0,255,0), CV_FILLED);

    // publish image
    cv_bridge::CvImage img_msg;
    img_msg.header = header_msg;
    img_msg.image = img_rgb;
    img_msg.encoding = sensor_msgs::image_encodings::BGR8;
    pub_images_.publish(img_msg.toImageMsg());
  }
  else if(frame == NULL && pub_images_.getNumSubscribers() > 0 && slam.stage() == FrameHandlerBase::PAUSED)
  {
    // display image when slam is not running
    cv_bridge::CvImage img_msg;
    img_msg.header.stamp = ros::Time::now();
    img_msg.header.frame_id = "/image";
    img_msg.image = img;
    img_msg.encoding = sensor_msgs::image_encodings::MONO8;
    pub_images_.publish(img_msg.toImageMsg());
  }

  // publish pose (world in camera frame)
  if(pub_pose_.getNumSubscribers() > 0 && frame != NULL)
  {
    geometry_msgs::PoseWithCovarianceStampedPtr msg_pose(new geometry_msgs::PoseWithCovarianceStamped);
    Quaterniond q(frame->T_f_w_.rotation_matrix());
    Vector3d p(frame->T_f_w_.translation());
    msg_pose->header = header_msg;
    msg_pose->pose.pose.position.x = p[0];
    msg_pose->pose.pose.position.y = p[1];
    msg_pose->pose.pose.position.z = p[2];
    msg_pose->pose.pose.orientation.x = q.x();
    msg_pose->pose.pose.orientation.y = q.y();
    msg_pose->pose.pose.orientation.z = q.z();
    msg_pose->pose.pose.orientation.w = q.w();
    for(size_t i=0; i<36; ++i)
      msg_pose->pose.covariance[i] = frame->Cov_(i%6, i/6);
    pub_pose_.publish(msg_pose);
  }
}

void Visualizer::
publishMapRegion(set<FramePtr> frames)
{
  if(pub_points_.getNumSubscribers() > 0)
  {
    int ts = vk::Timer::getCurrentTime();
    for(set<FramePtr>::iterator it=frames.begin(); it!=frames.end(); ++it)
      displayKeyframeWithMps(*it, ts);
  }
}

void Visualizer::
removeDeletedPts(const Map& map)
{
  if(pub_points_.getNumSubscribers() > 0)
  {
    for(list<Point*>::const_iterator it=map.trash_points_.begin(); it!=map.trash_points_.end(); ++it)
      vk::output_helper::publishPointMarker(pub_points_, Vector3d(), "pts", ros::Time::now(), (*it)->id_, 2, 0.006, Vector3d());
  }
}

void Visualizer::
displayKeyframeWithMps(const FramePtr& frame, int ts)
{
  // publish keyframe
  vk::output_helper::publishFrameMarker(pub_frames_, frame->T_f_w_.inverse().rotation_matrix(), frame->pos(), "kfs", ros::Time::now(), frame->id_*10, 0, 0.015);

  // publish point cloud and links
  for(Features::iterator it=frame->fts_.begin(); it!=frame->fts_.end(); ++it)
  {
    if((*it)->point == NULL)
      continue;

    if((*it)->point->last_published_ts_ == ts)
      continue;

    vk::output_helper::publishPointMarker(pub_points_, (*it)->point->pos_, "pts", ros::Time::now(), (*it)->point->id_, 0, 0.005, Vector3d(1.0, 0., 1.0));
    (*it)->point->last_published_ts_ = ts;
  }
}

void Visualizer::
exportToDense(const FramePtr& frame)
{
  // publish air_ground_msgs
  if(frame != NULL && dense_pub_nth_ > 0
      && trace_id_%dense_pub_nth_ == 0 && pub_dense_.getNumSubscribers() > 0)
  {
    svo_msgs::DenseInput msg;
    msg.header.stamp = ros::Time(frame->timestamp_);
    msg.header.frame_id = "/world";
    msg.frame_id = frame->id_;

    cv_bridge::CvImage img_msg;
    img_msg.header.stamp = msg.header.stamp;
    img_msg.header.frame_id = "camera";
    img_msg.image = frame->img();
    img_msg.encoding = sensor_msgs::image_encodings::MONO8;
    msg.image = *img_msg.toImageMsg();

    double min_z = std::numeric_limits<double>::max();
    double max_z = std::numeric_limits<double>::min();
    for(Features::iterator it=frame->fts_.begin(); it!=frame->fts_.end(); ++it)
    {
      if((*it)->point==NULL)
        continue;
      Vector3d pos = frame->T_f_w_*(*it)->point->pos_;
      min_z = min(pos[2], min_z);
      max_z = max(pos[2], max_z);
    }
    msg.min_depth = (float) min_z;
    msg.max_depth = (float) max_z;

    Quaterniond q(frame->T_f_w_.inverse().rotation_matrix());
    Vector3d p(frame->T_f_w_.inverse().translation());
    msg.pose.position.x = p[0];
    msg.pose.position.y = p[1];
    msg.pose.position.z = p[2];
    msg.pose.orientation.w = q.w();
    msg.pose.orientation.x = q.x();
    msg.pose.orientation.y = q.y();
    msg.pose.orientation.z = q.z();
    pub_dense_.publish(msg);
  }
}

} // end namespace svo