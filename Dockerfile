FROM nvidia/cuda:11.7.1-cudnn8-devel-ubuntu22.04

ENV USERNAME=user
ENV UID=1000
# ARG HOME=/home/${USERNAME}
ARG HOME=/root

# ビルド時にタイムゾーン選択に邪魔されないようにするため
RUN ln -sf /usr/share/zoneinfo/Asia/Tokyo /etc/localtime
ENV LANG en_US.UTF-8

# ros2
RUN apt update \
    && apt upgrade -y
RUN apt install -y \
    sudo \
    software-properties-common \
    && add-apt-repository universe
RUN apt update && apt install -y curl gnupg lsb-release \
    && curl -sSL https://raw.githubusercontent.com/ros/rosdistro/master/ros.key -o /usr/share/keyrings/ros-archive-keyring.gpg \
    && echo "deb [arch=$(dpkg --print-architecture) signed-by=/usr/share/keyrings/ros-archive-keyring.gpg] http://packages.ros.org/ros2/ubuntu $(. /etc/os-release && echo $UBUNTU_CODENAME) main" | sudo tee /etc/apt/sources.list.d/ros2.list > /dev/null
RUN apt update \
    && apt install -y \
    ros-humble-ros-base \
    ros-dev-tools \
    python3-colcon-common-extensions
RUN echo 'source /opt/ros/humble/setup.bash' >> ~/.bashrc \
    && echo 'export ROS_DOMAIN_ID=30 #TURTLEBOT3' >> ~/.bashrc \
    && echo "LOCAL_IP=`hostname -I | cut -d' ' -f1`" >> ~/.bashrc \
    && echo 'echo "LOCAL_IP=${LOCAL_IP}"' >> ~/.bashrc \
    && echo 'source /opt/ros/humble/setup.bash' >> ~/.bashrc \
    && echo 'export ROS_DOMAIN_ID=33 #TURTLEBOT3' >> ~/.bashrc \
    && echo 'export ROS_LOCALHOST_ONLY=0' >> ~/.bashrc \
    && echo 'env | grep ROS' >> ~/.bashrc \
    && echo 'alias colup=". ~/workspace/ros2_ws/install/local_setup.bash"' >> ~/.bashrc \
    && echo 'export OPENPOSE_MODEL_PATH="${HOME}/openpose/models"' >> ~/.bashrc

# openpose
RUN apt update && apt install -y git
WORKDIR ${HOME}
RUN git clone https://github.com/CMU-Perceptual-Computing-Lab/openpose
WORKDIR ${HOME}/openpose
RUN git submodule update --init --recursive --remote

RUN apt-get install -y cmake-qt-gui \
    && apt-get install -y libopencv-dev
RUN bash ./scripts/ubuntu/install_deps.sh

WORKDIR ${HOME}/openpose/models/pose/body_25
RUN wget -c https://www.dropbox.com/s/3x0xambj2rkyrap/pose_iter_584000.caffemodel
WORKDIR ${HOME}/openpose/models/face
RUN wget -c https://www.dropbox.com/s/d08srojpvwnk252/pose_iter_116000.caffemodel
WORKDIR ${HOME}/openpose/models/hand
RUN wget -c https://www.dropbox.com/s/gqgsme6sgoo0zxf/pose_iter_102000.caffemodel

WORKDIR ${HOME}/openpose/build
RUN cmake \
    -DBUILD_PYTHON=ON \
    -DDOWNLOAD_BODY_25_MODEL=OFF \
    -DDOWNLOAD_FACE_MODEL=OFF \
    -DDOWNLOAD_HAND_MODEL=OFF \
    -DUSE_CUDNN=OFF \
    .. \
    && make -j`nproc`
RUN echo 'export PYTHONPATH="${HOME}/openpose/build/python:$PYTHONPATH"' >> ~/.bashrc
RUN apt-get install -y libcanberra-gtk*
RUN pip3 install cv_bridge

WORKDIR ${HOME}
