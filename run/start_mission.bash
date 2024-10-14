#!/bin/bash

# Robot name
ROBOT_NAME="robassistant_1"

echo "Starting mission execution..."
ros2 service call /$ROBOT_NAME/start std_srvs/srv/Empty
echo "Complete"