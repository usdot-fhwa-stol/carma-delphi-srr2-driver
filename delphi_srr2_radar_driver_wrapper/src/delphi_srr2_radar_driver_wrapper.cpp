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

    // Set driver type
    status_.sensor = true;

    // Initilize all subscribers for SRR driver
    detection_sub_ = nh_->subscribe("detections", 10, &DelphiSrr2RadarDriverWrapper::detection_cb, this);
    status1_sub_ = nh_->subscribe("srr_status1", 10, &DelphiSrr2RadarDriverWrapper::status1_cb, this);
    status2_sub_ = nh_->subscribe("srr_status2", 10, &DelphiSrr2RadarDriverWrapper::status2_cb, this);
    status5_sub_ = nh_->subscribe("srr_status5", 10, &DelphiSrr2RadarDriverWrapper::status5_cb, this);

    // Initilize all publishers for CARMA
    object_track_pub_ = nh_->advertise<radar_msgs::RadarTrackArray>("radar/srr/tracks_raw", 1);
    radar_status_pub_ = nh_->advertise<radar_msgs::RadarStatus>("radar/srr/status", 1);

    private_nh_->param<std::string>("sensor_frame", sensor_frame_, "srr2");
    private_nh_->param<double>("bounding_box_size", bounding_box_size_, 5);
}

void DelphiSrr2RadarDriverWrapper::pre_spin()
{
    checkRadarTimeout();
    publish_radar_status();
    publish_object_track();
}

void DelphiSrr2RadarDriverWrapper::post_spin() {}

void DelphiSrr2RadarDriverWrapper::shutdown() {}

void DelphiSrr2RadarDriverWrapper::status1_cb(const delphi_srr_msgs::SrrStatus1ConstPtr &msg)
{
    last_update_time = ros::Time::now();
    status1_msg_ = msg;
}

void DelphiSrr2RadarDriverWrapper::status2_cb(const delphi_srr_msgs::SrrStatus2ConstPtr &msg)
{
    last_update_time = ros::Time::now();
    status2_msg_ = msg;
}

void DelphiSrr2RadarDriverWrapper::status5_cb(const delphi_srr_msgs::SrrStatus5ConstPtr &msg)
{
    last_update_time = ros::Time::now();
    status5_msg_ = msg;
    switch(msg->CAN_TX_SYSTEM_STATUS)
    {
        case delphi_srr_msgs::SrrStatus5::CAN_TX_SYSTEM_STATUS_Running:
            status_.status = cav_msgs::DriverStatus::OPERATIONAL;
            break;
        case delphi_srr_msgs::SrrStatus5::CAN_TX_SYSTEM_STATUS_Faulty:
            status_.status = cav_msgs::DriverStatus::FAULT;
            break;
        case delphi_srr_msgs::SrrStatus5::CAN_TX_SYSTEM_STATUS_Blocked:
        case delphi_srr_msgs::SrrStatus5::CAN_TX_SYSTEM_STATUS_Hot:
            status_.status = cav_msgs::DriverStatus::DEGRADED;
            break;
        default:
            status_.status = cav_msgs::DriverStatus::OFF;
            break;
    }
}

void DelphiSrr2RadarDriverWrapper::detection_cb(const radar_msgs::RadarDetectionArrayConstPtr &msg) {
    last_update_time = ros::Time::now();
    track_msg_ = msg;
}

void DelphiSrr2RadarDriverWrapper::publish_object_track()
{
    radar_msgs::RadarTrackArray tracks;
    tracks.header.stamp = ros::Time::now();
    tracks.header.frame_id = sensor_frame_;
    for(radar_msgs::RadarDetection object : track_msg_->detections)
    {
        radar_msgs::RadarTrack track;
        track.track_id = object.detection_id;
        geometry_msgs::Polygon bounding_box;
        geometry_msgs::Point32 point1, point2, point3, point4;
        point1.x = static_cast<float> (object.position.x + 0.5 * bounding_box_size_);
        point1.y = static_cast<float> (object.position.y - 0.5 * bounding_box_size_);
        point2.x = static_cast<float> (object.position.x - 0.5 * bounding_box_size_);
        point2.y = static_cast<float> (object.position.y - 0.5 * bounding_box_size_);
        point3.x = static_cast<float> (object.position.x + 0.5 * bounding_box_size_);
        point3.y = static_cast<float> (object.position.y + 0.5 * bounding_box_size_);
        point4.x = static_cast<float> (object.position.x - 0.5 * bounding_box_size_);
        point4.y = static_cast<float> (object.position.y + 0.5 * bounding_box_size_);
        std::vector<geometry_msgs::Point32> polygon_points{point1, point2, point3, point4};
        bounding_box.points = polygon_points;
        track.track_shape = bounding_box;
        track.linear_velocity = object.velocity;
        tracks.tracks.push_back(track);
    }
    object_track_pub_.publish(tracks);
}

void DelphiSrr2RadarDriverWrapper::publish_radar_status()
{
    radar_msgs::RadarStatus status;
    status.header.stamp = ros::Time::now();
    status.curvature = static_cast<short> (status1_msg_->CAN_TX_CURVATURE);
    status.yaw_rate = status1_msg_->CAN_TX_YAW_RATE_CALC;
    status.vehicle_speed = status1_msg_->CAN_TX_VEHICLE_SPEED_CALC;
    status.temperature = static_cast<char> (status2_msg_->CAN_TX_TEMPERATURE);
    radar_status_pub_.publish(status);
}

void DelphiSrr2RadarDriverWrapper::checkRadarTimeout()
{
    if(ros::Time::now() - last_update_time > ros::Duration(0.3))
    {
        status_.status = cav_msgs::DriverStatus::FAULT;
    }
}
