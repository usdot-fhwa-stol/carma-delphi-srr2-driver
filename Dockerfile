#  Copyright (C) 2018-2021 LEIDOS.
# 
#  Licensed under the Apache License, Version 2.0 (the "License"); you may not
#  use this file except in compliance with the License. You may obtain a copy of
#  the License at
# 
#  http://www.apache.org/licenses/LICENSE-2.0
# 
#  Unless required by applicable law or agreed to in writing, software
#  distributed under the License is distributed on an "AS IS" BASIS, WITHOUT
#  WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied. See the
#  License for the specific language governing permissions and limitations under
#  the License.
FROM usdotfhwastoldev/carma-base:develop as setup

ARG ROS1_PACKAGES=""
ENV ROS1_PACKAGES=${ROS1_PACKAGES}
ARG ROS2_PACKAGES=""
ENV ROS2_PACKAGES=${ROS2_PACKAGES}

RUN mkdir ~/src
COPY --chown=carma . /home/carma/src/
RUN ~/src/docker/checkout.sh
RUN ~/src/docker/install.sh

FROM usdotfhwastoldev/carma-base:develop

ARG BUILD_DATE="NULL"
ARG VERSION="NULL"
ARG VCS_REF="NULL"

# Install AutonomousStuff Delphi SRR2 Driver Package
RUN sudo apt update && \
  sudo apt install ros-$ROS_DISTRO-delphi-srr

LABEL org.label-schema.schema-version="1.0"
LABEL org.label-schema.name="carma-delphi-srr2-driver"
LABEL org.label-schema.description="Delphi SRR2 radar driver for the CARMA Platform"
LABEL org.label-schema.vendor="Leidos"
LABEL org.label-schema.version=${VERSION}
LABEL org.label-schema.url="https://highways.dot.gov/research/research-programs/operations/CARMA"
LABEL org.label-schema.vcs-url="https://github.com/usdot-fhwa-stol/carma-delphi-srr2-driver/"
LABEL org.label-schema.vcs-ref=${VCS_REF}
LABEL org.label-schema.build-date=${BUILD_DATE}

COPY --from=setup /home/carma/install /opt/carma/install
RUN sudo chmod -R +x /opt/carma/install

CMD  [ "wait-for-it.sh", "localhost:11311", "--", "roslaunch", "delphi_srr2_radar_driver_wrapper", "delphi_srr2_radar_driver_wrapper.launch"]
