FROM ros:humble

# Set the working directory
WORKDIR /ws

# Install OpenCV and OpenCV Contrib
RUN apt-get update && apt-get install -y libopencv-dev libopencv-contrib-dev
RUN apt-get update && apt-get -y install libcanberra-gtk-module

# Install ROS2 dependencies
RUN apt-get update && apt-get install -y \
    python3 \
    python3-pip \
    libboost-all-dev \
    libeigen3-dev \
    libompl-dev \
    ros-humble-tf-transformations \
    ros-humble-cv-bridge \
    ros-humble-octomap-msgs \
    ros-humble-octomap \
    ros-humble-dynamic-edt-3d 

# Install Dependencies for OMPL
RUN apt-get update && apt-get install -y \
    libeigen3-dev \
    libboost-all-dev \
    cmake \
    git \
    g++

# Install OpenCV Python Headless
RUN pip3 install \
    setuptools==58.2.0 \
    opencv-contrib-python-headless \
    cv_bridge \
    transforms3d

# Install GTK+ Development Libraries
RUN apt-get update && apt-get install -y libgtk-3-dev

# Install rviz2 and octomap rviz plugins
RUN apt-get update && apt-get install -y ros-humble-rviz2 ros-humble-octomap-rviz-plugins

# Install debugger
RUN apt-get update && apt-get install -y gdb

# Clone the OMPL repository to get the scripts
RUN git clone https://github.com/ompl/ompl.git /tmp/ompl

# Copy the entrypoint script into the container
COPY entrypoint.sh /entrypoint.sh

# Make the entrypoint script executable
RUN chmod +x /entrypoint.sh

# Add ros source to bashrc
RUN echo "source /opt/ros/humble/setup.bash" >> ~/.bashrc

ENTRYPOINT ["/entrypoint.sh"]