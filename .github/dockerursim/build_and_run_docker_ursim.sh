#!/bin/bash

DIR="$( cd "$( dirname "${BASH_SOURCE[0]}" )" >/dev/null 2>&1 && pwd )"

docker network create --subnet=192.168.0.0/16 static_test_net

docker build ${DIR} -t mydockerursim \
  --build-arg URSIM="https://s3-eu-west-1.amazonaws.com/ur-support-site/118926/URSim_Linux-5.11.1.108318.tar.gz" \
  --build-arg VERSION=5.11.1.108318
docker volume create dockerursim
docker run --name="mydockerursim" -d \
  -e ROBOT_MODEL=UR5 \
  -e DISPLAY=:1 \
  --net static_test_net \
  --ip 192.168.56.101 \
  -p 5900:5900 \
  -p 8080:8080 \
  -p 29999:29999 \
  -p 30001-30004:30001-30004 \
  -v "${DIR}/.vol":/ursim/programs \
  --privileged \
  --cpus=1 \
  mydockerursim
