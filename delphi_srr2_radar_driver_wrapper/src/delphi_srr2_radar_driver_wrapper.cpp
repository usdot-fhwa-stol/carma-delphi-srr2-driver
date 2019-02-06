/*
 * Copyright (C) 2019 LEIDOS.
 *
 * Licensed under the Apache License, Version 2.0 (the "License"); you may not
 * use this file except in compliance with the License. You may obtain a copy of
 * the License at
 *
 * http://www.apache.org/licenses/LICENSE-2.0
 *
 * Unless required by applicable law or agreed to in writing, software
 * distributed under the License is distributed on an "AS IS" BASIS, WITHOUT
 * WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied. See the
 * License for the specific language governing permissions and limitations under
 * the License.
 */

#include "delphi_srr2_radar_driver_wrapper.h"
#include <radar_msgs/RadarTrackArray.h>
#include <radar_msgs/RadarStatus.h>

DelphiSrr2RadarDriverWrapper::DelphiSrr2RadarDriverWrapper(int argc, char **argv, const std::string &name) : DriverWrapper (argc, argv, name) {}

DelphiSrr2RadarDriverWrapper::~DelphiSrr2RadarDriverWrapper() {}

void DelphiSrr2RadarDriverWrapper::initialize() {

    // Override spin rate
    spin_rate_ = 50;

    // Messages timeout is 0.1 second in this spin rate:
    msg_timeout_ = 1 / spin_rate_ * 5;

    // Set driver type
    status_.sensor = true;

    // Initilize all subscribers for SRR driver
    detection_sub_ = nh_->subscribe("detections", 1, &DelphiSrr2RadarDriverWrapper::detection_cb, this);
    status1_sub_ = nh_->subscribe("srr_status1", 1, &DelphiSrr2RadarDriverWrapper::status1_cb, this);
    status2_sub_ = nh_->subscribe("srr_status2", 1, &DelphiSrr2RadarDriverWrapper::status2_cb, this);
    status5_sub_ = nh_->subscribe("srr_status5", 1, &DelphiSrr2RadarDriverWrapper::status5_cb, this);

    // Initilize all publishers for CARMA
    object_track_pub_ = nh_->advertise<radar_msgs::RadarTrackArray>("radar/tracks_raw", 1);
    radar_status_pub_ = nh_->advertise<radar_msgs::RadarStatus>("radar/status", 1);

    private_nh_->param<double>("bounding_box_size", bounding_box_size_, 1);
    private_nh_->param<double>("driver_timeout", driver_timeout_, 0.3);

}

void DelphiSrr2RadarDriverWrapper::pre_spin()
{
    checkRadarTimeout();
}

void DelphiSrr2RadarDriverWrapper::post_spin()
{
    publish_radar_status();
    publish_object_track();
}

void DelphiSrr2RadarDriverWrapper::shutdown() {}

void DelphiSrr2RadarDriverWrapper::status1_cb(const delphi_srr_msgs::SrrStatus1ConstPtr &msg)
{
    last_update_time_ = ros::Time::now();
    status1_msg_ = msg;
}

void DelphiSrr2RadarDriverWrapper::status2_cb(const delphi_srr_msgs::SrrStatus2ConstPtr &msg)
{
    last_update_time_ = ros::Time::now();
    status2_msg_ = msg;
}

void DelphiSrr2RadarDriverWrapper::status5_cb(const delphi_srr_msgs::SrrStatus5ConstPtr &msg)
{
    last_update_time_ = ros::Time::now();
    status_.status = worker_.healthCheck(msg);
}

void DelphiSrr2RadarDriverWrapper::detection_cb(const radar_msgs::RadarDetectionArrayConstPtr &msg) {
    track_msg_ = msg;
}

void DelphiSrr2RadarDriverWrapper::publish_object_track()
{
    if(track_msg_ == nullptr ||
       ros::Time::now() - track_msg_->header.stamp > ros::Duration(msg_timeout_))
    {
        return;
    }
    object_track_pub_.publish(worker_.compositeRadarTrack(track_msg_, bounding_box_size_));
}

void DelphiSrr2RadarDriverWrapper::publish_radar_status()
{
    if(status1_msg_ == nullptr ||
       status2_msg_ == nullptr ||
       ros::Time::now() - status1_msg_->header.stamp > ros::Duration(msg_timeout_) ||
       ros::Time::now() - status2_msg_->header.stamp > ros::Duration(msg_timeout_))
    {
        return;
    }
    radar_status_pub_.publish(worker_.compositeRadarStatus(status1_msg_, status2_msg_));
}

void DelphiSrr2RadarDriverWrapper::checkRadarTimeout()
{
    if(last_update_time_.isZero())
    {
        status_.status = cav_msgs::DriverStatus::OFF;
    }
    else if(ros::Time::now() - last_update_time_ > ros::Duration(driver_timeout_))
    {
        status_.status = cav_msgs::DriverStatus::FAULT;
    }
}
