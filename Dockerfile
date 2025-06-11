FROM osrf/ros:noetic-desktop

# Set build-time variables for the user
ARG USERNAME=ubuntu
ARG USER_UID=1000
ARG USER_GID=$USER_UID
RUN if ! id -u $USER_UID >/dev/null 2>&1; then \
        groupadd --gid $USER_GID $USERNAME && \
        useradd -s /bin/bash --uid $USER_UID --gid $USER_GID -m $USERNAME; \
    fi

RUN rm -f /etc/apt/sources.list.d/ros1-latest.list \
    && rm -f /usr/share/keyrings/ros1-latest-archive-keyring.gpg

# Add sudo support for the non-root user
RUN apt-get update && \
    apt-get install -y sudo && \
    echo "$USERNAME ALL=(root) NOPASSWD:ALL" > /etc/sudoers.d/$USERNAME && \
    chmod 0440 /etc/sudoers.d/$USERNAME
RUN apt-get update && \
    apt-get install -y sudo python3-dev python3-pip && \
    # optional: make python3 the default
    update-alternatives --install /usr/bin/python python /usr/bin/python3 1 && \
    echo "$USERNAME ALL=(ALL) NOPASSWD:ALL" > /etc/sudoers.d/$USERNAME && \
    chmod 0440 /etc/sudoers.d/$USERNAME

# # Fix ROS GPG key issue
# RUN apt-get update && apt-get install -y curl gnupg && \
#     apt-key del F42ED6FBAB17C654 || true && \
#     curl -s https://raw.githubusercontent.com/ros/rosdistro/master/ros.asc | apt-key add - && \
#     apt-get update && \
#     apt-get install -y sudo python3-dev python3-pip && \
#     update-alternatives --install /usr/bin/python python /usr/bin/python3 1 && \
#     echo "ubuntu ALL=(ALL) NOPASSWD:ALL" > /etc/sudoers.d/ubuntu && \
#     chmod 0440 /etc/sudoers.d/ubuntu

# Fix ROS GPG key issue with updated method
# RUN apt-get install -y curl gnupg lsb-release && \
#     curl -sSL https://raw.githubusercontent.com/ros/rosdistro/master/ros.asc | \
#     gpg --dearmor -o /usr/share/keyrings/ros-archive-keyring.gpg && \
#     echo "deb [signed-by=/usr/share/keyrings/ros-archive-keyring.gpg] http://packages.ros.org/ros/ubuntu $(lsb_release -sc) main" > /etc/apt/sources.list.d/ros-latest.list && \
#     apt-get update

# RUN apt-get install -y sudo python3-dev python3-pip && \
#     update-alternatives --install /usr/bin/python python /usr/bin/python3 1 && \
#     echo "${USERNAME} ALL=(ALL) NOPASSWD:ALL" > /etc/sudoers.d/${USERNAME} && \
#     chmod 0440 /etc/sudoers.d/${USERNAME}

# Switch from root to user
USER $USERNAME 
#Add user to video group to allow access to webcam
RUN sudo usermod --append --groups video $USERNAME
# Update all packages
RUN sudo apt update && sudo apt upgrade -y
# Install Git and X11 applications (for xeyes) and ping utility
RUN sudo apt install -y git iputils-ping x11-apps sshfs sshpass net-tools \
    netcat openssh-server avahi-daemon libnss-mdns iproute2 tmux vim nano curl
# Rosdep update
RUN rosdep update 
#Source the ROS setup file
RUN echo "source /opt/ros/${ROS_DISTRO}/setup.bash" >> ~/.bashrc

### ADD ANY CUSTOME SETUP BELOW ###

# Setup environment variables
# Home directory
ARG HOME_DIR=/home/$USERNAME
# Based on DNS configuration: jetbot, jetbot.local, or jetbot.lan
ARG ROS_MASTER_URI=http://jetbot:11311
# The local device's wlan0 IP. ipconfig (Windows) or ifconfig (Linux)
ARG CLIENT_IP=192.168.8.142

COPY EVC/workshops/requirements.txt /tmp/requirements.txt
RUN pip3 install --no-cache-dir -r /tmp/requirements.txt

# Move copied directory from the jetson to the container
COPY EVC $HOME_DIR/EVC



# COPY talker.py $HOME_DIR/
# COPY listener.py $HOME_DIR/
RUN sudo chown -R $USERNAME:$USERNAME $HOME_DIR/EVC
# Setup ROS_MASTER_URI
RUN echo "export ROS_MASTER_URI=$ROS_MASTER_URI" >> ~/.bashrc
# Setup ROS_IP
# RUN echo "export ROS_IP=$(hostname -I | awk '{print $1}')" >> ~/.bashrc
RUN echo "export ROS_IP=$CLIENT_IP" >> ~/.bashrc
# Setup DISPLAY
RUN echo "export DISPLAY=$CLIENT_IP:0" >> ~/.bashrc