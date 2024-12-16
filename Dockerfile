# Use the Ubuntu 24.04 base image
FROM ubuntu:24.04

# Set environment variable for non-interactive apt installs
ENV DEBIAN_FRONTEND=noninteractive

# Update and install basic utilities
RUN apt update && apt install -y \
    software-properties-common \
    curl \
    git \
    neovim

# Add Python 3.10 repository and install Python 3.10 and pip
RUN add-apt-repository -y ppa:deadsnakes/ppa && \
    apt update && apt install -y \
    python3.10 \
    python3.10-distutils && \
    curl -sS https://bootstrap.pypa.io/get-pip.py | python3.10

# Install Python requirements
COPY requirements.txt /tmp/
RUN python3.10 -m pip install -r /tmp/requirements.txt

# Clone and install rSoccer
RUN git clone https://github.com/robocin/rSoccer.git /rSoccer && \
    cd /rSoccer && \
    python3.10 -m pip install .

# Set up work directory and copy user code
WORKDIR /workspace
COPY src/ /workspace/src
