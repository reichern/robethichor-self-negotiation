FROM osrf/ros:humble-desktop

RUN apt-get update && apt-get install -y \
    python3-colcon-common-extensions \
    && rm -rf /var/lib/apt/lists/*

WORKDIR /robethichor_ws

COPY . src/

RUN . /opt/ros/humble/setup.sh && colcon build
RUN chmod +x src/run/scalability_experiments/run_experiment.bash
RUN chmod +x src/run/simulation/run_usecase.bash

ENTRYPOINT ["/bin/bash", "-c", "source /robethichor_ws/install/setup.bash && exec \"$@\" || exec bash", "--"]