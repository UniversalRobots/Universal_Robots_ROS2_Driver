#!/bin/bash

DIR="$( cd "$( dirname "${BASH_SOURCE[0]}" )" >/dev/null 2>&1 && pwd )"

docker stop mydockerursim && docker rm mydockerursim
docker network create rm static_test_net
