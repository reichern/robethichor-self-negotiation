FROM osrf/ros:humble-desktop

RUN apt-get update \
    && apt-get install -y python3-colcon-common-extensions \
    && rm -rf /var/lib/apt/lists/* \
    && rosdep update

WORKDIR /robethichor_ws

COPY . src/

RUN apt-get update && rosdep install --from-paths src -y --ignore-src
RUN . /opt/ros/humble/setup.sh && colcon build
RUN chmod +x src/launch.bash

ENTRYPOINT ["/bin/bash", "-c", "source /robethichor_ws/install/setup.bash && cd src && ./launch.bash", "--"]