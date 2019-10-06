/*
 * Copyright (c) 2010, Willow Garage, Inc.
 * All rights reserved.
 *
 * Redistribution and use in source and binary forms, with or without
 * modification, are permitted provided that the following conditions are met:
 *
 *     * Redistributions of source code must retain the above copyright
 *       notice, this list of conditions and the following disclaimer.
 *     * Redistributions in binary form must reproduce the above copyright
 *       notice, this list of conditions and the following disclaimer in the
 *       documentation and/or other materials provided with the distribution.
 *     * Neither the name of the Willow Garage, Inc. nor the names of its
 *       contributors may be used to endorse or promote products derived from
 *       this software without specific prior written permission.
 *
 * THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS"
 * AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE
 * IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE
 * ARE DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT OWNER OR CONTRIBUTORS BE
 * LIABLE FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR
 * CONSEQUENTIAL DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF
 * SUBSTITUTE GOODS OR SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS
 * INTERRUPTION) HOWEVER CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN
 * CONTRACT, STRICT LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE)
 * ARISING IN ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE
 * POSSIBILITY OF SUCH DAMAGE.
 */

#include <ros/ros.h>
#include <visualization_msgs/Marker.h>
#include "nav_msgs/Odometry.h"
#include <atomic>

ros::Publisher marker_pub;
ros::Timer cmdReqExpireTimer;

static constexpr float pos1_x = 4;
static constexpr float pos1_y = -3;

static constexpr float pos2_x = -2;
static constexpr float pos2_y = -3;

static constexpr float error = 0.4;

enum Mode
{
  Normal,
  PublishPickupZone,
  Hide,
  PublishDropZone,
  Done,
};

Mode currentMode = Normal;
bool timerMode = true;

void setMarker(int32_t action, int32_t shape, float x, float y)
{
  visualization_msgs::Marker marker;

  marker.header.frame_id = "map";
  marker.header.stamp = ros::Time::now();

  marker.ns = "add_markers";
  marker.id = 0;

  marker.type = shape;
  marker.action = action; //visualization_msgs::Marker::ADD;

  marker.pose.position.x = x;
  marker.pose.position.y = y;
  marker.pose.position.z = 0;
  marker.pose.orientation.x = 0.0;
  marker.pose.orientation.y = 0.0;
  marker.pose.orientation.z = 0.0;
  marker.pose.orientation.w = 1.0;

  marker.scale.x = 0.2;
  marker.scale.y = 0.2;
  marker.scale.z = 0.2;

  marker.color.r = 0.0f;
  marker.color.g = 12.0f;
  marker.color.b = 255.0f;
  marker.color.a = 1.0;

  marker.lifetime = ros::Duration();
  marker_pub.publish(marker);
}

void chatterCallback(const nav_msgs::Odometry::ConstPtr& msg)
{
  if(timerMode)
  {
    if((std::abs(msg->pose.pose.position.x) > error) || (std::abs(msg->pose.pose.position.y) > error))
    {
      timerMode = false;
      ROS_INFO("Add_markers works as home service");
    }
    return;
  }

  switch (currentMode)
  {
  case Normal:
    if((msg->pose.pose.position.x > (pos1_x - error))
        && (msg->pose.pose.position.x < (pos1_x + error)))
    {
      if((msg->pose.pose.position.y > (pos1_y - error))
          && (msg->pose.pose.position.y < (pos1_y + error)))
      {
        currentMode = PublishPickupZone;
      }
    }
    break;

  case PublishPickupZone:
    currentMode = Hide;
    setMarker(visualization_msgs::Marker::DELETE, visualization_msgs::Marker::CUBE, pos1_x, pos1_y);
    break;

  case Hide:
    if((msg->pose.pose.position.x > (pos2_x - error))
        && (msg->pose.pose.position.x < (pos2_x + error)))
    {
      if((msg->pose.pose.position.y > (pos2_y - error))
          && (msg->pose.pose.position.y < (pos2_y + error)))
      {
        currentMode = PublishDropZone;
      }
    }
    break;

  case PublishDropZone:
        currentMode = Done;
        setMarker(visualization_msgs::Marker::ADD, visualization_msgs::Marker::CUBE, pos2_x, pos2_y);
    break;  

  case Done:
    break;
  default:
    break;
  }
}


void timerCallback(const ros::TimerEvent& e)
{
  if(not timerMode)
  {
    return;
  }

  switch (currentMode)
  {
  case Normal:
  case PublishPickupZone:
    currentMode = Hide;
    ROS_INFO("Hide the marker");
    setMarker(visualization_msgs::Marker::DELETE, visualization_msgs::Marker::CUBE, pos1_x, pos1_y);
    break;

  case Hide:
  case PublishDropZone:
    currentMode = Done;
    ROS_INFO("Publish the marker at the drop off zone");
    setMarker(visualization_msgs::Marker::ADD, visualization_msgs::Marker::CUBE, pos2_x, pos2_y);
    break;  

  case Done:
    break;
  default:
    break;
  }
}

int main( int argc, char** argv )
{
  ros::init(argc, argv, "add_markers");
  ros::NodeHandle nAdv;
  ros::NodeHandle nTime;
  ros::NodeHandle nSub;
  marker_pub = nAdv.advertise<visualization_msgs::Marker>("visualization_marker", 1);
  ros::Subscriber sub = nSub.subscribe("odom", 1000, chatterCallback);

  while (marker_pub.getNumSubscribers() < 1)
  {
    if (!ros::ok())
    {
      return 0;
    }
    ROS_WARN_ONCE("Please create a subscriber to the marker");
    sleep(1);
  }

  ROS_INFO("Publish the marker at the pickup zone");
  setMarker(visualization_msgs::Marker::ADD, visualization_msgs::Marker::CUBE, pos1_x, pos1_y);
  cmdReqExpireTimer = nTime.createTimer(ros::Duration(5), timerCallback);
  
  ros::spin();
}