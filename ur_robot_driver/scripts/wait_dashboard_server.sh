#!/bin/bash

netcat -z 192.168.56.101 29999
while [ $? -eq 1 ]
do
    echo "Dashboard server not accepting connections..."
    sleep 3
    netcat -z 192.168.56.101 29999
done
echo "Dashboard server connections are possible."
sleep 5
