# 基础镜像：ROS Noetic 桌面全功能版 (包含 Gazebo 等)
FROM osrf/ros:noetic-desktop-full

# 设置非交互前端，防止 apt 卡住
ENV DEBIAN_FRONTEND=noninteractive

# ==============================
# 1. 系统级依赖安装
# ==============================
RUN apt-get update && apt-get install -y \
    python3-pip \
    python3-tk \
    git \
    vim \
    nano \
    net-tools \
    iproute2 \
    can-utils \
    libgl1-mesa-dev \
    libgl1-mesa-glx \
    libosmesa6-dev \
    libglew-dev \
    patchelf \
    && rm -rf /var/lib/apt/lists/*

# ==============================
# 2. Python 环境配置
# ==============================
COPY requirements.txt /tmp/requirements.txt
RUN pip3 install --no-cache-dir -r /tmp/requirements.txt

# ==============================
# 3. MuJoCo 环境还原 (匹配你原有的 /root/mujoco-3.1.6)
# ==============================
WORKDIR /root
RUN wget https://github.com/google-deepmind/mujoco/releases/download/3.1.6/mujoco-3.1.6-linux-x86_64.tar.gz \
    && tar -xf mujoco-3.1.6-linux-x86_64.tar.gz \
    && rm mujoco-3.1.6-linux-x86_64.tar.gz
ENV PATH="/root/mujoco-3.1.6/bin:${PATH}"

# ==============================
# 4. 工作空间设置
# ==============================
# 自动 source ROS 环境变量
RUN echo "source /opt/ros/noetic/setup.bash" >> /root/.bashrc
# 如果 ros_ws 编译过，自动 source 它的 setup.bash
RUN echo "if [ -f /home/leslie/ros_ws/devel/setup.bash ]; then source /home/leslie/ros_ws/devel/setup.bash; fi" >> /root/.bashrc

# 设置工作目录
WORKDIR /home/leslie/ros_ws