FROM ros:galactic-ros-core-focal
SHELL ["/bin/bash", "-c"]

RUN apt-get update && apt-get install -y \
    python3-pip \
    g++ \
    doxygen \
    make \
    cmake \
    python3-colcon-common-extensions \
    python3-rosdep

RUN mkdir -p /root/sadg_ws/src
WORKDIR /root/sadg_ws

COPY third_party src/third_party
COPY sadg_controller src/sadg_controller
# COPY dist/*.whl dist/
# COPY pyproject.toml .
# COPY poetry.lock .

# RUN pip install --no-cache-dir dist/*.whl \
#     && rm -rf dist/*.whl

RUN source /opt/ros/galactic/setup.bash && rosdep init && rosdep update
RUN rosdep install --from-paths src --ignore-src -r -y
RUN colcon build --symlink-install


# CMD ["python3", "-c", "import sadg_controller; print('Hello', sadg_controller)"]

# RUN python3 src/sadg_controller/main.py
