#pragma once

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

#include <radar_msgs/RadarTrackArray.h>
#include <radar_msgs/RadarStatus.h>
#include <radar_msgs/RadarDetectionArray.h>
#include <delphi_srr_msgs/SrrStatus1.h>
#include <delphi_srr_msgs/SrrStatus2.h>
#include <delphi_srr_msgs/SrrStatus5.h>
#include <cav_msgs/DriverStatus.h>

class DelphiSrr2RadarDriverWrapperWorker
{

public:

    DelphiSrr2RadarDriverWrapperWorker() {}
    ~DelphiSrr2RadarDriverWrapperWorker() {}

    radar_msgs::RadarTrackArray compositeRadarTrack(const radar_msgs::RadarDetectionArrayConstPtr &msg, double bounding_box_size);
    radar_msgs::RadarStatus compositeRadarStatus(const delphi_srr_msgs::SrrStatus1ConstPtr &msg1, const delphi_srr_msgs::SrrStatus2ConstPtr &msg2);
    unsigned char healthCheck(const delphi_srr_msgs::SrrStatus5ConstPtr &msg);
};
