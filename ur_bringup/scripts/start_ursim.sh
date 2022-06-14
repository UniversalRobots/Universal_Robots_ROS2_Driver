#!/bin/bash

# copyright 2022 Universal Robots A/S
#
# Redistribution and use in source and binary forms, with or without
# modification, are permitted provided that the following conditions are met:
#
#    * Redistributions of source code must retain the above copyright
#      notice, this list of conditions and the following disclaimer.
#
#    * Redistributions in binary form must reproduce the above copyright
#      notice, this list of conditions and the following disclaimer in the
#      documentation and/or other materials provided with the distribution.
#
#    * Neither the name of the {copyright_holder} nor the names of its
#      contributors may be used to endorse or promote products derived from
#      this software without specific prior written permission.
#
# THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS"
# AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE
# IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE
# ARE DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT HOLDER OR CONTRIBUTORS BE
# LIABLE FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR
# CONSEQUENTIAL DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF
# SUBSTITUTE GOODS OR SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS
# INTERRUPTION) HOWEVER CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN
# CONTRACT, STRICT LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE)
# ARISING IN ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE
# POSSIBILITY OF SUCH DAMAGE.

PERSISTENT_BASE="${HOME}/.ursim"
URCAP_VERSION="1.0.5"

echo -e "\033[0;31mDEPRECATION WARNING: " \
    "Launch files from the ur_bringup package are deprecated and will be removed from Iron " \
    "Irwini on. Please use the same launch files from the ur_robot_driver package." \
    "\033[0m"

help()
{
  # Display Help
  echo "Starts URSim inside a docker container"
  echo
  echo "Syntax: `basename "$0"` [-m|s|h]"
  echo "options:"
  echo "    -m <model>     Robot model. One of [ur3, ur3e, ur5, ur5e, ur10, ur10e, ur16e]. Defaults to ur5e."
  echo "    -h             Print this Help."
  echo
}

ROBOT_MODEL=UR5
ROBOT_SERIES=e-series

validate_model()
{
  case $ROBOT_MODEL in
    ur3|ur5|ur10)
      ROBOT_MODEL=${ROBOT_MODEL^^}
      ROBOT_SERIES=cb3
      ;;
    ur3e|ur5e|ur10e|ur16e)
      ROBOT_MODEL=${ROBOT_MODEL^^}
      ROBOT_MODEL=$(echo ${ROBOT_MODEL:0:$((${#ROBOT_MODEL}-1))})
      ROBOT_SERIES=e-series
      ;;
    *)
      echo "Not a valid robot model: $ROBOT_MODEL"
      exit
      ;;
  esac
}


while getopts ":hm:s:" option; do
  case $option in
    h) # display Help
      help
      exit;;
    m) # robot model
      ROBOT_MODEL=${OPTARG}
      validate_model
      ;;
    \?) # invalid option
      echo "Error: Invalid option"
      help
      exit;;
  esac
done

URCAP_STORAGE="${PERSISTENT_BASE}/${ROBOT_SERIES}/urcaps"
PROGRAM_STORAGE="${PERSISTENT_BASE}/${ROBOT_SERIES}/programs"

# Create local storage for programs and URCaps
mkdir -p "${URCAP_STORAGE}"
mkdir -p "${PROGRAM_STORAGE}"

# Download external_control URCap
if [[ ! -f "${URCAP_STORAGE}/externalcontrol-${URCAP_VERSION}.jar" ]]; then
  curl -L -o "${URCAP_STORAGE}/externalcontrol-${URCAP_VERSION}.jar" \
    "https://github.com/UniversalRobots/Universal_Robots_ExternalControl_URCap/releases/download/v${URCAP_VERSION}/externalcontrol-${URCAP_VERSION}.jar"
fi

# Check whether network already exists
docker network inspect ursim_net > /dev/null
if [ $? -eq 0 ]; then
  echo "ursim_net already exists"
else
  echo "Creating ursim_net"
  docker network create --subnet=192.168.56.0/24 ursim_net
fi

# run docker container
docker run --rm -d --net ursim_net --ip 192.168.56.101\
  -v "${URCAP_STORAGE}":/urcaps \
  -v "${PROGRAM_STORAGE}":/ursim/programs \
  -e ROBOT_MODEL="${ROBOT_MODEL}" \
  --name ursim \
  universalrobots/ursim_${ROBOT_SERIES} || exit

trap "echo killing; docker container kill ursim; exit" SIGINT SIGTERM

echo "Docker URSim is running"
printf "\nTo access Polyscope, open the following URL in a web browser.\n\thttp://192.168.56.101:6080/vnc.html\n\n"
echo "To exit, press CTRL+C"

while :
do
  sleep 1
done
