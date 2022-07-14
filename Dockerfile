FROM ros:noetic-ros-core-focal

RUN apt-get update && apt-get install -y \
    python3-pip

WORKDIR /home

# COPY requirements.txt .
COPY third_party third_party
COPY src src
COPY data data
COPY pyproject.toml .
COPY poetry.lock .

RUN pip install .

RUN source /opt/ros/noetic/setup.bash

# RUN python3 src/sadg_controller/main.py
