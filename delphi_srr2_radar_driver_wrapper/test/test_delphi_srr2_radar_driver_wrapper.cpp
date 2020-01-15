/*
 * Copyright (C) 2019-2020 LEIDOS.
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
#include <gtest/gtest.h>

TEST(DelphiSrr2RadarTest, testTracksComposition)
{
    DelphiSrr2RadarDriverWrapperWorker worker;
    radar_msgs::RadarDetectionArray detections;
    detections.header.stamp.sec = 100;
    detections.header.stamp.nsec = 200;
    detections.header.frame_id = "srr2";
    radar_msgs::RadarDetection object1;
    object1.position.x = 4;
    object1.position.y = 6;
    object1.velocity.x = 0;
    object1.velocity.y = 0;
    object1.detection_id = 50;
    detections.detections.push_back(object1);
    radar_msgs::RadarDetectionArrayConstPtr detections_pointer(new radar_msgs::RadarDetectionArray(detections));
    radar_msgs::RadarTrackArray tracks = worker.compositeRadarTrack(detections_pointer, 1);
    EXPECT_EQ(100, tracks.header.stamp.sec);
    EXPECT_EQ(200, tracks.header.stamp.nsec);
    EXPECT_EQ("srr2", tracks.header.frame_id);
    EXPECT_EQ(1, tracks.tracks.size());
    EXPECT_EQ(50, tracks.tracks[0].track_id);
    EXPECT_EQ(0.0, tracks.tracks[0].linear_velocity.x);
    EXPECT_EQ(0.0, tracks.tracks[0].linear_velocity.y);
    EXPECT_EQ(4.5, tracks.tracks[0].track_shape.points[0].x);
    EXPECT_EQ(5.5, tracks.tracks[0].track_shape.points[0].y);
    EXPECT_EQ(3.5, tracks.tracks[0].track_shape.points[1].x);
    EXPECT_EQ(5.5, tracks.tracks[0].track_shape.points[1].y);
    EXPECT_EQ(4.5, tracks.tracks[0].track_shape.points[2].x);
    EXPECT_EQ(6.5, tracks.tracks[0].track_shape.points[2].y);
    EXPECT_EQ(3.5, tracks.tracks[0].track_shape.points[3].x);
    EXPECT_EQ(6.5, tracks.tracks[0].track_shape.points[3].y);
}

TEST(DelphiSrr2RadarTest, testStatusComposition)
{
    DelphiSrr2RadarDriverWrapperWorker worker;
    delphi_srr_msgs::SrrStatus1 status1;
    delphi_srr_msgs::SrrStatus2 status2;
    status1.header.stamp.sec = 1;
    status1.header.stamp.nsec = 2;
    status1.header.frame_id = "srr";
    status1.CAN_TX_CURVATURE = 3;
    status1.CAN_TX_YAW_RATE_CALC = 4;
    status1.CAN_TX_VEHICLE_SPEED_CALC = 5;
    status2.CAN_TX_TEMPERATURE = 6;
    delphi_srr_msgs::SrrStatus1ConstPtr status1_pointer(new delphi_srr_msgs::SrrStatus1(status1));
    delphi_srr_msgs::SrrStatus2ConstPtr status2_pointer(new delphi_srr_msgs::SrrStatus2(status2));
    radar_msgs::RadarStatus radar_status = worker.compositeRadarStatus(status1_pointer, status2_pointer);
    EXPECT_EQ(1, radar_status.header.stamp.sec);
    EXPECT_EQ(2, radar_status.header.stamp.nsec);
    EXPECT_EQ("srr", radar_status.header.frame_id);
    EXPECT_EQ(3, radar_status.curvature);
    EXPECT_EQ(4, radar_status.yaw_rate);
    EXPECT_EQ(5, radar_status.vehicle_speed);
    EXPECT_EQ(6, radar_status.temperature);
}

TEST(DelphiSrr2RadarTest, testHeathCheck)
{
    DelphiSrr2RadarDriverWrapperWorker worker;
    delphi_srr_msgs::SrrStatus5 status5;
    status5.CAN_TX_SYSTEM_STATUS = delphi_srr_msgs::SrrStatus5::CAN_TX_SYSTEM_STATUS_Running;
    delphi_srr_msgs::SrrStatus5ConstPtr status5_pointer(new delphi_srr_msgs::SrrStatus5(status5));
    EXPECT_EQ(1, worker.healthCheck(status5_pointer));
    status5.CAN_TX_SYSTEM_STATUS = delphi_srr_msgs::SrrStatus5::CAN_TX_SYSTEM_STATUS_Blocked;
    status5_pointer.reset(new delphi_srr_msgs::SrrStatus5(status5));
    EXPECT_EQ(2, worker.healthCheck(status5_pointer));
    status5.CAN_TX_SYSTEM_STATUS = delphi_srr_msgs::SrrStatus5::CAN_TX_SYSTEM_STATUS_Faulty;
    status5_pointer.reset(new delphi_srr_msgs::SrrStatus5(status5));
    EXPECT_EQ(3, worker.healthCheck(status5_pointer));
}

// Run all the tests
int main(int argc, char **argv) {
    testing::InitGoogleTest(&argc, argv);
    return RUN_ALL_TESTS();
}
