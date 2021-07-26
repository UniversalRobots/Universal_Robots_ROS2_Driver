#!/bin/bash

DIR="$( cd "$( dirname "${BASH_SOURCE[0]}" )" >/dev/null 2>&1 && pwd )"

docker-compose up -d
vncviewer -autoreconnect 30 0.0.0.0:5900 
