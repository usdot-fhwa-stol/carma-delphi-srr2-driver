/*
 * Copyright (C) 2019-2021 LEIDOS.
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

#include "delphi_srr2_radar_driver_wrapper_worker.h"

radar_msgs::RadarTrackArray DelphiSrr2RadarDriverWrapperWorker::compositeRadarTrack(const radar_msgs::RadarDetectionArrayConstPtr &msg, double bounding_box_size)
{
    radar_msgs::RadarTrackArray tracks;
    tracks.header.stamp = msg->header.stamp;
    tracks.header.frame_id = msg->header.frame_id;
    for(radar_msgs::RadarDetection object : msg->detections)
    {
        radar_msgs::RadarTrack track;
        track.track_id = object.detection_id;
        // An imaginary 2-D bounding box is put on the detected point.
        // Its boundaries are parallel with vehicle x or y axix and its size is configurable.
        geometry_msgs::Polygon bounding_box;
        geometry_msgs::Point32 point1, point2, point3, point4;
        point1.x = static_cast<float> (object.position.x + 0.5 * bounding_box_size);
        point1.y = static_cast<float> (object.position.y - 0.5 * bounding_box_size);
        point2.x = static_cast<float> (object.position.x - 0.5 * bounding_box_size);
        point2.y = static_cast<float> (object.position.y - 0.5 * bounding_box_size);
        point3.x = static_cast<float> (object.position.x + 0.5 * bounding_box_size);
        point3.y = static_cast<float> (object.position.y + 0.5 * bounding_box_size);
        point4.x = static_cast<float> (object.position.x - 0.5 * bounding_box_size);
        point4.y = static_cast<float> (object.position.y + 0.5 * bounding_box_size);
        std::vector<geometry_msgs::Point32> polygon_points{point1, point2, point3, point4};
        bounding_box.points = polygon_points;
        track.track_shape = bounding_box;
        track.linear_velocity = object.velocity;
        tracks.tracks.push_back(track);
    }
    return tracks;
}

radar_msgs::RadarStatus DelphiSrr2RadarDriverWrapperWorker::compositeRadarStatus(const delphi_srr_msgs::SrrStatus1ConstPtr &msg1, const delphi_srr_msgs::SrrStatus2ConstPtr &msg2)
{
    radar_msgs::RadarStatus status;
    status.header.stamp = msg1->header.stamp;
    status.header.frame_id = msg1->header.frame_id;
    status.curvature = static_cast<short> (msg1->can_tx_curvature);
    status.yaw_rate = msg1->can_tx_yaw_rate_calc;
    status.vehicle_speed = msg1->can_tx_vehicle_speed_calc;
    status.temperature = static_cast<char> (msg2->can_tx_temperature);
    return status;
}

unsigned char DelphiSrr2RadarDriverWrapperWorker::healthCheck(const delphi_srr_msgs::SrrStatus5ConstPtr &msg)
{
    switch(msg->can_tx_system_status)
    {
        case delphi_srr_msgs::SrrStatus5::CAN_TX_SYSTEM_STATUS_RUNNING:
            return cav_msgs::DriverStatus::OPERATIONAL;
        case delphi_srr_msgs::SrrStatus5::CAN_TX_SYSTEM_STATUS_FAULTY:
            return cav_msgs::DriverStatus::FAULT;
        case delphi_srr_msgs::SrrStatus5::CAN_TX_SYSTEM_STATUS_STARTUP:
        case delphi_srr_msgs::SrrStatus5::CAN_TX_SYSTEM_STATUS_CONFIGURATION:
        case delphi_srr_msgs::SrrStatus5::CAN_TX_SYSTEM_STATUS_BLOCKED:
        case delphi_srr_msgs::SrrStatus5::CAN_TX_SYSTEM_STATUS_HOT:
            return cav_msgs::DriverStatus::DEGRADED;
        default:
            return cav_msgs::DriverStatus::OFF;
    }
}
