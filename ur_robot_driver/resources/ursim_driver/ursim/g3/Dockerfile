# MIT License
#
# Original from https://github.com/ahobsonsayers/DockURSim
# Copyright (c) 2019 Arran Hobson Sayers
#
# Permission is hereby granted, free of charge, to any person obtaining a copy
# of this software and associated documentation files (the "Software"), to deal
# in the Software without restriction, including without limitation the rights
# to use, copy, modify, merge, publish, distribute, sublicense, and/or sell
# copies of the Software, and to permit persons to whom the Software is
# furnished to do so, subject to the following conditions:
#
# The above copyright notice and this permission notice shall be included in all
# copies or substantial portions of the Software.
#
# THE SOFTWARE IS PROVIDED "AS IS", WITHOUT WARRANTY OF ANY KIND, EXPRESS OR
# IMPLIED, INCLUDING BUT NOT LIMITED TO THE WARRANTIES OF MERCHANTABILITY,
# FITNESS FOR A PARTICULAR PURPOSE AND NONINFRINGEMENT. IN NO EVENT SHALL THE
# AUTHORS OR COPYRIGHT HOLDERS BE LIABLE FOR ANY CLAIM, DAMAGES OR OTHER
# LIABILITY, WHETHER IN AN ACTION OF CONTRACT, TORT OR OTHERWISE, ARISING FROM,
# OUT OF OR IN CONNECTION WITH THE SOFTWARE OR THE USE OR OTHER DEALINGS IN THE
# SOFTWARE.

FROM ubuntu:latest

# Set Version Information
ARG VERSION
ARG URSIM
LABEL build_version="URSim Version: ${VERSION}"
LABEL description="UR simulator cb series"

# Set Timezone
ENV TZ "Europe/Copenhagen"

# Set Home Directory
ENV HOME /ursim

# Set robot model - Can be UR3, UR5 or UR10
ENV ROBOT_MODEL UR5

# Set display
ENV DISPLAY :1

RUN \
    echo "**** Installing Dependencies ****" && \
    apt-get update && \
    apt-get install -qy --no-install-recommends openjdk-8-jre psmisc && \
    apt-get -y install curl && \
    apt-get install -y x11vnc xvfb && \
    # Change java alternatives so we use openjdk8 (required by URSim)
    update-alternatives --install /usr/bin/java java /usr/lib/jvm/java-8-openjdk-amd64/jre/bin/java 10000

# Setup JAVA_HOME
ENV JAVA_HOME /usr/lib/jvm/java-8-openjdk-amd64

RUN \
    echo "**** Downloading URSim ****" && \
    # Download URSim Linux tar.gz
    curl ${URSIM} -o URSim-Linux.tar.gz && \
    # Extract tarball
    tar xvzf URSim-Linux.tar.gz && \
    # Remove the tarball
    rm URSim-Linux.tar.gz && \
    # Rename the URSim folder to just ursim
    mv  /ursim* /ursim

RUN \
    echo "**** Installing URSim ****" && \
    # cd to ursim folder
    cd /ursim && \
    # Stop install of unnecessary packages and install required ones quietly
    sed -i 's|apt-get -y install|apt-get -qy install --no-install-recommends|g' ./install.sh && \
    # Skip xterm command. We don't have a desktop
    sed -i 's|tty -s|(exit 0)|g' install.sh && \
    # Skip Check of Java Version as we have the correct installed and the command will fail
    sed -i 's|needToInstallJava$|(exit 0)|g' install.sh && \
    # Skip install of desktop shortcuts - we don't have a desktop
    sed -i '/for TYPE in UR3 UR5 UR10/,$ d' ./install.sh  && \
    # Replace libcurl3 with libcurl4 else the installation will fail
    sed -i 's/libcurl3/libcurl4/g' ./install.sh  && \
    # Remove commands that are not relevant on docker as we are root user
    sed -i 's|pkexec ||g' ./install.sh && \
    sed -i 's|sudo ||g' ./install.sh && \
    # Install URSim
    ./install.sh && \
    echo "Installed URSim"

RUN \
    echo "**** Clean Up ****" && \
    rm -rf \
    /tmp/* \
    /var/lib/apt/lists/* \
    /var/tmp/*

# Copy entrypoint script
COPY g3/entrypoint.sh /entrypoint.sh
RUN chmod +x /entrypoint.sh

# VNC viewer
EXPOSE 5900
# Modbus Port
EXPOSE 502
# Interface Ports
EXPOSE 29999
EXPOSE 30001-30004

# Install the ExternalControl URCap
ADD ./aux/externalcontrol-1.0.5.urcap /ursim/GUI/bundle/com.fzi.externalcontrol-1.0.5.jar
# Install rs485 URCap
ADD ./aux/rs485-1.0.urcap /ursim/GUI/bundle/com.fzi.rs485-1.0.0.jar

ENTRYPOINT ["/entrypoint.sh"]
