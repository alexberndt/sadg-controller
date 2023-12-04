ARG ROS2_VERSION=humble

FROM ros:${ROS2_VERSION}

# General preparations
RUN apt-get update
RUN apt-get install -y python3-rosdep python3-colcon-common-extensions python3-pip python3-tk
RUN rosdep update

# Prepare workspace
WORKDIR /ws
RUN mkdir -p src
WORKDIR /ws/src
RUN git clone --recurse-submodules https://github.com/alexberndt/sadg-controller.git
WORKDIR /ws
RUN rosdep install --from-paths src --ignore-src -r -y
RUN python3 -m pip install -r src/sadg-controller/requirements.txt
RUN colcon build

RUN rm -rf /var/lib/apt/lists/*

COPY entrypoint.sh /ws/entrypoint.sh
ENTRYPOINT [ "/ws/entrypoint.sh" ]
