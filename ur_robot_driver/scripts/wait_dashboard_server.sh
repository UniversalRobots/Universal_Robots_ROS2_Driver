#!/bin/bash

ip="192.168.56.101"
if [[ ! -z "$1" ]]
then
	ip=$1
fi

netcat -z $ip 29999
while [ $? -eq 1 ]
do
    echo "Dashboard server not accepting connections..."
    sleep 3
    netcat -z $ip 29999
done
echo "Dashboard server connections are possible."
sleep 5
