# syntax=docker/dockerfile:1.7

FROM ros:humble-ros-base-jammy

SHELL ["/bin/bash", "-o", "pipefail", "-c"]

ARG DEBIAN_FRONTEND=noninteractive
ARG UBUNTU_APT_MIRROR=https://mirrors.ustc.edu.cn/ubuntu
ARG ROS2_APT_MIRROR=https://mirrors.ustc.edu.cn/ros2/ubuntu
ARG OPEN3D141_SHA256=ed370e9606202b2afba6d1790c048d975d7ca8caffd2c9f2595becd0be17a5e2
ARG KN_NAV_REF=main
ARG UNITREE_SDK2_REF=main
ARG LIVOX_SDK2_REF=master
ARG GIT_REFRESH=initial

RUN printf '%s\n' \
      "deb ${UBUNTU_APT_MIRROR} jammy main restricted universe multiverse" \
      "deb ${UBUNTU_APT_MIRROR} jammy-updates main restricted universe multiverse" \
      "deb ${UBUNTU_APT_MIRROR} jammy-security main restricted universe multiverse" \
      >/etc/apt/sources.list && \
    sed -i \
      "s|^Types: deb deb-src$|Types: deb|; \
       s|http://packages.ros.org/ros2/ubuntu|${ROS2_APT_MIRROR}|g; \
       s|https://packages.ros.org/ros2/ubuntu|${ROS2_APT_MIRROR}|g" \
      /etc/apt/sources.list.d/ros2.sources && \
    apt-get update && \
    apt-get install -y --no-install-recommends \
      build-essential \
      ca-certificates \
      cmake \
      git \
      libapr1-dev \
      libboost-all-dev \
      libc++-11-dev \
      libc++abi-11-dev \
      libeigen3-dev \
      libfmt-dev \
      libgl1-mesa-dev \
      libopencv-dev \
      libpcl-dev \
      libspdlog-dev \
      libtbb-dev \
      libx11-dev \
      libyaml-cpp-dev \
      pkg-config \
      python3-colcon-common-extensions \
      python3-dev \
      python3-numpy \
      python3-open3d \
      python3-scipy \
      ros-humble-ament-cmake-python \
      ros-humble-cv-bridge \
      ros-humble-eigen3-cmake-module \
      ros-humble-image-transport \
      ros-humble-pcl-ros \
      unzip \
      vim && \
    rm -rf /var/lib/apt/lists/*

RUN mkdir -p \
      /home/code/thirdparty \
      /home/code/work_space/ws_livox/src \
      /tmp/ros_logs

# 可针对实际open3d的目录进行修改
RUN --mount=type=bind,source=thirdparty/open3d141.zip,target=/tmp/open3d141.zip,readonly \
    echo "${OPEN3D141_SHA256}  /tmp/open3d141.zip" | sha256sum --check --strict - && \
    unzip -q /tmp/open3d141.zip -d /home/code/thirdparty && \
    test -f /home/code/thirdparty/open3d141/lib/cmake/Open3D/Open3DConfig.cmake

RUN printf '%s\n' "${GIT_REFRESH}" >/dev/null && \
    git clone --depth 1 --branch "${UNITREE_SDK2_REF}" \
      https://github.com/unitreerobotics/unitree_sdk2.git \
      /home/code/thirdparty/unitree_sdk2 && \
    git clone --depth 1 --branch "${LIVOX_SDK2_REF}" \
      https://github.com/Livox-SDK/Livox-SDK2.git \
      /home/code/thirdparty/Livox-SDK2 && \
    git clone --depth 1 --branch "${KN_NAV_REF}" \
      https://github.com/xuqiang656/kn_nav.git \
      /home/code/work_space/kn_nav

COPY work_space/ws_livox/src/ /home/code/work_space/ws_livox/src/

RUN cmake \
      -S /home/code/thirdparty/unitree_sdk2 \
      -B /home/code/thirdparty/unitree_sdk2/build \
      -DCMAKE_BUILD_TYPE=Release \
      -DCMAKE_INSTALL_PREFIX=/opt/unitree_robotics \
      -DBUILD_EXAMPLES=OFF && \
    cmake --build /home/code/thirdparty/unitree_sdk2/build --parallel 4 && \
    cmake --install /home/code/thirdparty/unitree_sdk2/build

RUN cmake \
      -S /home/code/thirdparty/Livox-SDK2 \
      -B /home/code/thirdparty/Livox-SDK2/build \
      -DCMAKE_BUILD_TYPE=Release && \
    cmake --build /home/code/thirdparty/Livox-SDK2/build --parallel 4 && \
    cmake --install /home/code/thirdparty/Livox-SDK2/build && \
    ldconfig

RUN source /opt/ros/humble/setup.bash && \
    cd /home/code/work_space/ws_livox && \
    colcon build \
      --symlink-install \
      --cmake-args \
        -DCMAKE_BUILD_TYPE=Release \
        -DBUILD_TESTING=OFF \
        -DROS_EDITION=ROS2 \
        -DDISTRO_ROS=humble


RUN cd /home/code/work_space/kn_nav/src/PCT_planner/planner && \
    bash build_thirdparty.sh && \
    bash build.sh

RUN source /opt/ros/humble/setup.bash && \
    source /home/code/work_space/ws_livox/install/setup.bash && \
    export CMAKE_PREFIX_PATH="/opt/unitree_robotics:${CMAKE_PREFIX_PATH:-}" && \
    export LD_LIBRARY_PATH="/opt/unitree_robotics/lib:/usr/local/lib:${LD_LIBRARY_PATH:-}" && \
    cd /home/code/work_space/kn_nav && \
    colcon build \
      --symlink-install \
      --cmake-args \
        -DCMAKE_BUILD_TYPE=Release \
        -DBUILD_TESTING=OFF \
        -DOpen3D_DIR=/home/code/thirdparty/open3d141/lib/cmake/Open3D

RUN printf '%s\n' \
      'source /opt/ros/humble/setup.bash' \
      'source /home/code/work_space/ws_livox/install/setup.bash' \
      'source /home/code/work_space/kn_nav/install/setup.bash' \
      'pct_root=/home/code/work_space/kn_nav/src/PCT_planner/planner/lib' \
      'pct_build_root=${pct_root}/build/src' \
      'export LD_LIBRARY_PATH="${pct_root}:${pct_build_root}/a_star:${pct_build_root}/trajectory_optimization:${pct_build_root}/ele_planner:${pct_build_root}/map_manager:${pct_build_root}/common/smoothing:/home/code/thirdparty/pct-install/gtsam-4.1.1/lib:/opt/unitree_robotics/lib:/usr/local/lib:${LD_LIBRARY_PATH:-}"' \
      'export PYTHONPATH="${pct_root}:${PYTHONPATH:-}"' \
      'export CMAKE_PREFIX_PATH="/opt/unitree_robotics:${CMAKE_PREFIX_PATH:-}"' \
      'export Open3D_DIR=/home/code/thirdparty/open3d141/lib/cmake/Open3D' \
      'export ROS_LOG_DIR="${ROS_LOG_DIR:-/tmp/ros_logs}"' \
      'mkdir -p "${ROS_LOG_DIR}"' \
      >>/root/.bashrc

WORKDIR /home/code/work_space/kn_nav

CMD ["sleep", "infinity"]
