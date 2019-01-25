#pragma once

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

#include <driver_wrapper/driver_wrapper.h>
#include <delphi_srr_msgs/SrrStatus1.h>
#include <delphi_srr_msgs/SrrStatus2.h>
#include <delphi_srr_msgs/SrrStatus5.h>
#include <radar_msgs/RadarDetectionArray.h>

class DelphiSrr2RadarDriverWrapper : public cav::DriverWrapper
{
    ros::Subscriber status1_sub_;
    ros::Subscriber status2_sub_;
    ros::Subscriber status5_sub_;
    ros::Subscriber detection_sub_;
    ros::Publisher  object_track_pub_;
    ros::Publisher  radar_status_pub_;

    delphi_srr_msgs::SrrStatus1ConstPtr status1_msg_;
    delphi_srr_msgs::SrrStatus2ConstPtr status2_msg_;
    delphi_srr_msgs::SrrStatus5ConstPtr status5_msg_;
    radar_msgs::RadarDetectionArrayConstPtr track_msg_;

    ros::Time last_update_time_;
    double bounding_box_size_;
    double driver_timeout_;

public:
    DelphiSrr2RadarDriverWrapper(int argc, char **argv, const std::string &name = "delphi_srr2_radar_driver_wrapper");
    virtual ~DelphiSrr2RadarDriverWrapper();

private:

    /**
     * @brief Callback for handling srr_status message
     */
    void status1_cb(const delphi_srr_msgs::SrrStatus1ConstPtr& msg);
    void status2_cb(const delphi_srr_msgs::SrrStatus2ConstPtr& msg);
    void status5_cb(const delphi_srr_msgs::SrrStatus5ConstPtr& msg);

    /**
     * @brief Callback for handling radar detection message
     */
    void detection_cb(const radar_msgs::RadarDetectionArrayConstPtr& msg);

    /**
     * @brief Publishes radar updates received from the vendor driver
     */
    void publish_object_track();
    void publish_radar_status();

    /**
     * @brief Check radar data output timeout
     */
    void checkRadarTimeout();

    //cav::DriverWrapper members
    virtual void initialize();
    virtual void pre_spin();
    virtual void post_spin();
    virtual void shutdown();

};
