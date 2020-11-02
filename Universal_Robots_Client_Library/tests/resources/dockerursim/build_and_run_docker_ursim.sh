#!/bin/bash

DIR="$( cd "$( dirname "${BASH_SOURCE[0]}" )" >/dev/null 2>&1 && pwd )"
URSIM_VERSION="${1:-5.8.0.10253}"

docker network create --subnet=192.168.56.0/24 static_test_net

docker build ${DIR} -t mydockerursim --build-arg VERSION="${URSIM_VERSION}"
docker volume create dockerursim
docker run --name="mydockerursim" -d \
  -e ROBOT_MODEL=UR5 \
  --net static_test_net \
  --ip 192.168.56.101 \
  -p 8080:8080 \
  -p 29999:29999 \
  -p 30001-30004:30001-30004 \
  -v "${DIR}/.vol":/ursim/programs \
  -v dockursim:/ursim \
  --privileged \
  --cpus=1 \
  mydockerursim
