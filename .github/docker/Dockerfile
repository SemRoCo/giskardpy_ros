FROM osrf/ros:jazzy-desktop-full

ARG DOWNLOAD_URL

SHELL ["/bin/bash", "--login", "-c"]

ENV WORKON_HOME=/root/venvs

# Installing Dependencies
# tzdata install noninteractive, because of Timezone setup
RUN apt-get update
RUN DEBIAN_FRONTEND=noninteractive TZ=Europe/Berlin apt-get -y install tzdata
RUN apt-get install -y \
            curl \
            jq \
            gh \
            python3  \
            python3-pip  \
            python3-venv  \
            python3-virtualenvwrapper \
            python3-numpy \
            ros-jazzy-ros2-control \
            ros-jazzy-ros2-controllers \
            ros-jazzy-rclpy-message-converter \
            git-all \
            cmake \
            sudo \
            pkg-config

RUN echo "source /opt/ros/jazzy/setup.bash" >> ~/.bashrc && \
    echo "source /usr/share/colcon_cd/function/colcon_cd.sh" >> ~/.bashrc

# Install virtualenvwrapper \
# and create giskardpy venv
RUN mkdir -p $WORKON_HOME && \
    echo "export WORKON_HOME=$WORKON_HOME" >> ~/.bashrc && \
    echo "source /usr/share/virtualenvwrapper/virtualenvwrapper.sh" >> ~/.bashrc
# mkvirtualenv returns exit 1, but works fine \
# so overwriting with exit 0, so this step doesn't fail
RUN mkvirtualenv giskardpy --system-site-packages; exit 0
RUN echo "source /root/venvs/giskardpy/bin/activate" >> ~/.bashrc

# Download python-requirements
# and install in giskardpy venv
RUN curl -fsSL $DOWNLOAD_URL -o requirements.txt && \
    sed -i '/giskardpy/d' requirements.txt # remove giskardpy dependency, because it can't be fulfilled here
RUN workon giskardpy && \
    sudo apt-get install libeigen3-dev && \
    python3 -m pip install --upgrade pip && \
    pip3 install -r requirements.txt

# Install qpSWIFT
RUN workon giskardpy &&  \
    pip3 install pybind11 && \
    cd ~ && \
    git clone https://github.com/SemRoCo/qpSWIFT && \
    cd qpSWIFT && \
    cmake -S . -B build -DCMAKE_BUILD_TYPE=Release && \
    cmake --build build --config Release && \
    cmake --build build --target install && \
    cd python && \
    pip3 install .

# Install betterpybullet
RUN workon giskardpy && \
    mkdir -p ~/bpb && \
    cd ~/bpb && \
    git clone -b jazzy https://github.com/SemRoCo/bullet3.git && \
    cd bullet3 && \
    ./build_better_pybullet.sh \
