FROM ros:noetic-ros-core-focal

RUN apt-get update && apt-get install -y \
    python3-pip \
    g++ \
    doxygen \
    make \
    cmake

WORKDIR /home
SHELL ["/bin/bash", "-c"]

COPY third_party third_party
COPY src src
COPY data data
COPY launch launch
COPY dist/*.whl dist/
COPY pyproject.toml .
COPY poetry.lock .

RUN pip install --no-cache-dir dist/*.whl \
    && rm -rf dist/*.whl

RUN source /opt/ros/noetic/setup.bash && catkin_make


# CMD ["python3", "-c", "import sadg_controller; print('Hello', sadg_controller)"]

# RUN python3 src/sadg_controller/main.py
