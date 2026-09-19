# cuRobo MPC evaluation container (task plan-mpc-curobo-v1, option C3 of the
# human-dynamic-plan-s1 report). SIMULATION / EVALUATION ONLY.
#
# Why a container of our own: the workstation host has NO CUDA toolkit (no nvcc,
# RUNBOOK.md:188) and cuRobo compiles CUDA kernels (V2 does this at runtime via
# cuda.core/NVRTC, which still wants the CUDA headers a devel image provides).
# The cell's planner image may be used read-only as a base, but we build a clean
# image FROM an official CUDA *devel* base so the toolkit provenance is explicit
# and the recreate is reproducible (task rule: pulling a CUDA devel base is
# allowed).
#
# This single image carries three things so the ROS bridge can import cuRobo and
# publish ROS commands from one process:
#   1. CUDA 12.6 toolkit (devel: nvcc + headers + NVRTC)  -> cuRobo kernels
#   2. PyTorch cu124 + cuRobo V2 (Apache-2.0, MPC-capable) -> the MPC prototype
#   3. ROS 2 Humble + Universal Robots driver             -> the sim bridge
#
# Recreate: see mpc_curobo/build_container.sh (records size + build time).
FROM nvidia/cuda:12.6.2-devel-ubuntu22.04

ENV DEBIAN_FRONTEND=noninteractive \
    TZ=Etc/UTC \
    PYTHONUNBUFFERED=1 \
    PIP_NO_CACHE_DIR=1

# --- base OS + python -------------------------------------------------------
RUN apt-get update && apt-get install -y --no-install-recommends \
        ca-certificates curl gnupg2 lsb-release locales software-properties-common \
        git build-essential python3 python3-pip python3-dev python3-venv \
        libgl1 libglib2.0-0 && \
    locale-gen en_US en_US.UTF-8 && \
    rm -rf /var/lib/apt/lists/*
ENV LANG=en_US.UTF-8 LC_ALL=en_US.UTF-8

# --- ROS 2 Humble (Ubuntu 22.04) + Universal Robots driver ------------------
RUN add-apt-repository universe && \
    curl -sSL https://raw.githubusercontent.com/ros/rosdistro/master/ros.key \
        -o /usr/share/keyrings/ros-archive-keyring.gpg && \
    echo "deb [arch=$(dpkg --print-architecture) signed-by=/usr/share/keyrings/ros-archive-keyring.gpg] \
http://packages.ros.org/ros2/ubuntu $(lsb_release -cs) main" \
        > /etc/apt/sources.list.d/ros2.list && \
    apt-get update && apt-get install -y --no-install-recommends \
        ros-humble-ros-base \
        ros-humble-ur \
        ros-humble-ur-robot-driver \
        ros-humble-ur-controllers \
        ros-humble-ur-description \
        ros-humble-controller-manager \
        ros-humble-ros2-control \
        ros-humble-ros2-controllers \
        python3-colcon-common-extensions python3-rosdep && \
    rm -rf /var/lib/apt/lists/*

# --- PyTorch (CUDA 12.4 wheels; forward-compatible with the 12.6 toolkit and
#     the host's 580 driver) + cuRobo V2 (Apache-2.0) build deps -------------
RUN pip3 install --upgrade pip setuptools wheel && \
    pip3 install torch --index-url https://download.pytorch.org/whl/cu124

# cuRobo V2, pinned to the v0.8.0 tag (Apache-2.0). Cloned so the bundled
# examples/assets ship with the image; installed with the cu12-torch extra.
ARG CUROBO_REF=v0.8.0
RUN git clone --depth 1 --branch ${CUROBO_REF} https://github.com/NVlabs/curobo.git /opt/curobo && \
    pip3 install -e "/opt/curobo[cu12-torch]"

# The cell's measured robot description + our prototype are bind-mounted at run
# time (read-only), never baked in, so edits do not require a rebuild.
WORKDIR /work
ENV PYTHONPATH=/work
# Source ROS on every shell for interactive debugging.
RUN echo "source /opt/ros/humble/setup.bash" >> /root/.bashrc

CMD ["/bin/bash"]
