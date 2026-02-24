FROM nvcr.io/nvidia/pytorch:22.10-py3

ARG USERNAME=USERNAME
ARG USER_UID=1000
ARG USER_GID=$USER_UID

# Delete user if it exists in container (e.g Ubuntu Noble: ubuntu)
RUN if id -u $USER_UID ; then userdel `id -un $USER_UID` ; fi

# Create the user
RUN groupadd --gid $USER_GID $USERNAME \
    && useradd --uid $USER_UID --gid $USER_GID -m $USERNAME \
    && apt-get update \
    && apt-get install -y sudo \
    && echo $USERNAME ALL=\(root\) NOPASSWD:ALL > /etc/sudoers.d/$USERNAME \
    && chmod 0440 /etc/sudoers.d/$USERNAME

# Setup Locale
RUN apt-get update \
    && apt-get install -y locales \
    && locale-gen en_US en_US.UTF-8
ENV LC_ALL en_US.UTF-8
ENV LANG en_US.UTF-8
ENV LANGUAGE en_US:en
ENV TZ=Etc/UTC

# Set environment variable to handle interactive prompts
ARG DEBIAN_FRONTEND=noninteractive

# Install essential Ubuntu dependencies
RUN apt-get update  && apt-get upgrade -y && apt-get install -y \
    software-properties-common \
    build-essential \
    zlib1g-dev \
    libncurses5-dev \
    libgdbm-dev \
    libnss3-dev \
    libssl-dev \
    libreadline-dev \
    libffi-dev \
    libsqlite3-dev \
    wget \
    libbz2-dev curl \
    cython3 \
    ffmpeg \
    gfortran \
    libopenblas-dev \
    libopenblas64-dev \
    patchelf \
    pkg-config \
    portaudio19-dev \
    python3-testresources \
    python3-typing-extensions \
    # python3-tk \
    sox \
    sshpass \
    openssh-client \
    && rm -rf /var/lib/apt/lists/* 
    # && sshpass openssh-client 

# Install Python 3.10
RUN add-apt-repository ppa:savoury1/python \
    && apt-get update \
    && apt-get install -y python3.10-tk \
    && apt-get install -y python3.10 \
    && apt-get install -y python3.10-dev python3.10-distutils python3.10-venv python3-pip python3-empy \
    && pip3 install virtualenv

# Setup ROS sources list and keys
RUN sh -c 'echo "deb http://packages.ros.org/ros/ubuntu $(lsb_release -sc) main" > /etc/apt/sources.list.d/ros-latest.list' \
    && curl -s https://raw.githubusercontent.com/ros/rosdistro/master/ros.asc | sudo apt-key add -

# Install ROS Noetic
RUN apt-get update \
    && apt-get install -y ros-noetic-ros-base \
    && apt-get install -y python3-rosdep python3-rosinstall python3-rosinstall-generator python3-wstool \
    && rm -rf /var/lib/apt/lists/*

# Create Python3.10 virtual environments
RUN virtualenv -p python3.10 /opt/venv_speech_event \
    && virtualenv -p python3.10 /opt/venv_conversation_management \
    && virtualenv -p python3.10 /opt/venv_text_to_speech

# Install NaoQi
RUN apt-get update \
    && apt-get install -y python2 libpython2.7 libatlas3-base \
    && curl https://bootstrap.pypa.io/pip/2.7/get-pip.py --output /home/$USERNAME/get-pip.py \
    && python2 /home/$USERNAME/get-pip.py \
    && python2 -m pip install numpy \
    && wget -S -L -O /home/$USERNAME/pynaoqi-python2.7-2.5.5.5-linux64.tar.gz  https://community-static.aldebaran.com/resources/2.5.5/sdk-python/pynaoqi-python2.7-2.5.5.5-linux64.tar.gz \
    && tar -xvf /home/$USERNAME/pynaoqi-python2.7-2.5.5.5-linux64.tar.gz -C /home/$USERNAME/ \
    && chmod a+rx /home/$USERNAME/pynaoqi-python2.7-2.5.5.5-linux64/lib/python2.7/site-packages/ 
    # && rm /home/$USERNAME/pynaoqi.tar.gz /home/$USERNAME/get-pip.py

ENV SHELL /bin/bash

# Install Python dependencies for speechEvent
COPY speech_event_requirements.txt .
RUN bash -c "source /opt/venv_speech_event/bin/activate && pip install --no-cache-dir -U pip"
RUN bash -c "source /opt/venv_speech_event/bin/activate && pip install --no-cache-dir -r speech_event_requirements.txt"

# Install Python dependencies for conversationManagement
COPY conversation_management_requirements.txt .
RUN bash -c "source /opt/venv_conversation_management/bin/activate && pip install --no-cache-dir -U pip"
RUN bash -c "source /opt/venv_conversation_management/bin/activate && pip install --no-cache-dir -r conversation_management_requirements.txt"

# Install Python dependencies for textToSpeech
COPY tts_requirements.txt .
RUN bash -c "source /opt/venv_text_to_speech/bin/activate && pip install --no-cache-dir -U pip"
RUN bash -c "source /opt/venv_text_to_speech/bin/activate && pip install --no-cache-dir -r tts_requirements.txt"

# Initialize and update rosdep
RUN bash -c "source /opt/ros/noetic/setup.bash && rosdep init" || true \
    && bash -c "source /opt/ros/noetic/setup.bash && rosdep update"

# Final tourches to .bashrc for user (PYTHONPATH export needs to come before sourcing ROS and workspace setup.bash files)
RUN echo "export PATH=/usr/local/sbin:/usr/local/bin:/usr/sbin:/usr/bin:/sbin:/bin" >> /home/$USERNAME/.bashrc \
    && echo "export PYTHONPATH=${PYTHONPATH}:/home/$USERNAME/pynaoqi-python2.7-2.5.5.5-linux64/lib/python2.7/site-packages" >> /home/$USERNAME/.bashrc \
    && echo "source /opt/ros/noetic/setup.bash" >> /home/$USERNAME/.bashrc \
    && echo "source ./devel/setup.bash" >> /home/$USERNAME/.bashrc

# ********************************************************
# * Anything else you want to do like clean up goes here *
# ********************************************************

# [Optional] Set the default user. Omit if you want to keep the default as root.
USER $USERNAME
CMD ["/bin/bash"]

