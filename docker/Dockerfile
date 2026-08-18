# Development image for the WebGL + WASM forward/inverse kinematics example.
#
# It carries three toolchains at once:
#   * a native C++ toolchain (gcc, cmake, Eigen, Sophus, GoogleTest) for debugging
#     the kinematics with gdb and running the unit tests,
#   * Emscripten for compiling the same sources to WebAssembly,
#   * Node.js for the webpack dev server that serves the frontend.
FROM ubuntu:22.04

ENV DEBIAN_FRONTEND=noninteractive

# Setup timezone
RUN echo 'Etc/UTC' > /etc/timezone \
    && ln -s /usr/share/zoneinfo/Etc/UTC /etc/localtime \
    && apt-get update \
    && apt-get -y -q --no-install-recommends install \
    tzdata \
    && apt-get clean && rm -rf /var/lib/apt/lists/*

# Shell tooling
RUN apt-get update \
    && apt-get install -y -q --no-install-recommends \
    apt-utils \
    bash-completion \
    ca-certificates \
    clang-format \
    dirmngr \
    gdb \
    gnupg2 \
    less \
    sudo \
    tmux \
    vim \
    wget \
    xz-utils \
    && apt-get clean && rm -rf /var/lib/apt/lists/*

# Build toolchain
RUN apt-get update && apt-get install -y -q --no-install-recommends \
    build-essential \
    cmake \
    curl \
    git \
    lsb-release \
    ninja-build \
    pkg-config \
    python3 \
    python3-dev \
    python3-pip \
    && apt-get clean && rm -rf /var/lib/apt/lists/*

# Dependency packages
RUN apt-get -q update \
    && apt-get -y -q --no-install-recommends install \
    libfmt-dev \
    libyaml-cpp-dev \
    && apt-get clean && rm -rf /var/lib/apt/lists/*

# Install CMake 3.27.4
RUN wget -q https://github.com/Kitware/CMake/releases/download/v3.27.4/cmake-3.27.4-linux-x86_64.tar.gz \
    && tar -xzf cmake-3.27.4-linux-x86_64.tar.gz \
    && cd cmake-3.27.4-linux-x86_64 \
    && rm -rf /usr/local/man \
    && cp -r * /usr/local/ \
    && cd .. \
    && rm -rf cmake-3.27.4-linux-x86_64 cmake-3.27.4-linux-x86_64.tar.gz

# Install Eigen 3.4.0
RUN git clone --depth 1 --branch 3.4.0 https://gitlab.com/libeigen/eigen.git /opt/eigen \
    && mkdir -p /opt/eigen/build && cd /opt/eigen/build \
    && cmake \
    -DCMAKE_BUILD_TYPE=Release \
    -DEIGEN_BUILD_DOC=OFF \
    -DBUILD_TESTING=OFF \
    .. \
    && make install \
    && cd /opt && rm -r /opt/eigen

# Install Sophus 1.24.6.
# SOPHUS_USE_BASIC_LOGGING keeps libfmt out of the installed CMake package, which
# matters because the same headers are compiled for WebAssembly where fmt is absent.
RUN git clone --depth 1 --branch 1.24.6 https://github.com/strasdat/Sophus.git /opt/Sophus \
    && mkdir -p /opt/Sophus/build && cd /opt/Sophus/build \
    && cmake \
    -DCMAKE_BUILD_TYPE=Release \
    -DBUILD_SOPHUS_TESTS=OFF \
    -DBUILD_SOPHUS_EXAMPLES=OFF \
    -DSOPHUS_USE_BASIC_LOGGING=ON \
    .. \
    && make install \
    && cd /opt && rm -r /opt/Sophus

# Install GoogleTest v1.16.0
RUN git clone --depth 1 --branch v1.16.0 https://github.com/google/googletest.git /opt/googletest \
    && mkdir -p /opt/googletest/build && cd /opt/googletest/build \
    && cmake \
    -DCMAKE_BUILD_TYPE=Release \
    -DBUILD_GMOCK=OFF \
    .. \
    && make install \
    && cd /opt && rm -r /opt/googletest

# Install Node.js 20 for the frontend dev server
RUN curl -fsSL https://deb.nodesource.com/setup_20.x | bash - \
    && apt-get install -y -q --no-install-recommends nodejs \
    && apt-get clean && rm -rf /var/lib/apt/lists/* \
    && npm install -g npm@10

# Install Emscripten.
# `emsdk activate` drops a versioned node into /opt/emsdk/node; the symlink below
# pins a stable PATH entry so ENV does not have to hard-code that version.
ARG EMSDK_VERSION=3.1.64
RUN git clone --depth 1 https://github.com/emscripten-core/emsdk.git /opt/emsdk \
    && cd /opt/emsdk \
    && ./emsdk install ${EMSDK_VERSION} \
    && ./emsdk activate ${EMSDK_VERSION} \
    && ln -s "$(dirname "$(find /opt/emsdk/node -maxdepth 3 -type f -name node | head -n 1)")" /opt/emsdk/node/current-bin

ENV EMSDK=/opt/emsdk
ENV EMSDK_NODE=/opt/emsdk/node/current-bin/node
ENV EM_CACHE=/opt/emsdk/upstream/emscripten/cache
ENV PATH="/opt/emsdk:/opt/emsdk/upstream/emscripten:/opt/emsdk/node/current-bin:${PATH}"

# Warm the Emscripten sysroot so the first in-container build is not a cold start,
# then make the whole SDK writable for the non-root user.
RUN embuilder build libc libc++ libc++abi libcompiler_rt libembind \
    && chmod -R a+rwX /opt/emsdk

ENV LANG=C.UTF-8
ENV LC_ALL=C.UTF-8

ARG USER_NAME=robotics
ARG GROUP_NAME=robotics
ARG PROJECT_NAME=modern-robotics-wasm

ARG USER_UID=1000
ARG USER_GID=1000
RUN groupadd --gid ${USER_GID} ${GROUP_NAME}

RUN useradd --create-home --shell /bin/bash \
    --uid ${USER_UID} --gid ${USER_GID} ${USER_NAME} \
    && echo "${USER_NAME}:${GROUP_NAME}" | chpasswd \
    && echo "${USER_NAME} ALL=(ALL) NOPASSWD:ALL" > "/etc/sudoers.d/${USER_NAME}"

RUN mkdir -p /home/${USER_NAME}/${PROJECT_NAME} \
    && chown -R ${USER_NAME}:${GROUP_NAME} /home/${USER_NAME}
ENV HOME=/home/${USER_NAME}

RUN echo "export USER=${USER_NAME}" >> ${HOME}/.bashrc \
    && echo "export GROUP=${GROUP_NAME}" >> ${HOME}/.bashrc \
    && echo "export LD_LIBRARY_PATH=/usr/local/lib:\${LD_LIBRARY_PATH}" >> ${HOME}/.bashrc \
    && echo "export PATH=/opt/emsdk:/opt/emsdk/upstream/emscripten:/opt/emsdk/node/current-bin:\${PATH}" >> ${HOME}/.bashrc

# npm needs a writable prefix for the mounted workspace
RUN mkdir -p ${HOME}/.npm && chown -R ${USER_NAME}:${GROUP_NAME} ${HOME}/.npm

USER ${USER_NAME}
WORKDIR /home/${USER_NAME}/${PROJECT_NAME}
ENV SHELL="/bin/bash"

EXPOSE 3000
