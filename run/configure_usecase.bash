#!/bin/bash

# CONFIGURATION
ROBOT_BASE_NAME="robassistant_"

# JSON files paths
BASE_FOLDER="usecase/"
CONTEXT_FILE="context.json"
USER_STATUS_FILE="user_status.json"
ETHIC_PROFILES_FILE="ethic_profiles.json"
GOAL_FILE="goal.json"
ETHICAL_IMPLICATIONS_FILENAME="ethical_implications.json"
DISPOSITION_ACTIVATION_FILENAME="disposition_activation.json"

# Connector service information
PROFILE_LOAD_PATH="/loadEthicProfile"
USER_STATUS_PATH="/setUserStatus"
SET_GOAL_PATH="/setGoal"

ROS_WS_PATH="../../" # this directory is inside the ws/src folder

configure_robot() {

    local USER_LABEL=$1
    local ROBOT_NAME=$2
    local PORT=$3

    echo "------ Running and configuring $ROBOT_NAME ------"

    # JSON file read
    ETHIC_PROFILES=$(cat $BASE_FOLDER$1"/"$ETHIC_PROFILES_FILE)
    USER_STATUS=$(cat $BASE_FOLDER$1"/"$USER_STATUS_FILE)
    GOAL=$(cat $BASE_FOLDER$1"/"$GOAL_FILE)

    # Json sent as messages in topics should be escaped
    CONTEXT=$(cat $BASE_FOLDER$1"/"$CONTEXT_FILE | jq -c .)

    # Ros 2 launch (to launch the ros environment)
    FULL_PATH=$(pwd)
    INSTALL_PATH=$FULL_PATH"/"$ROS_WS_PATH"install/setup.bash"
    LAUNCH_COMMAND="ros2 launch robethichor robethichor_launch.py ns:=$ROBOT_NAME port:=$PORT ethical_implication_file:=$FULL_PATH/$BASE_FOLDER/$ETHICAL_IMPLICATIONS_FILENAME disposition_activation_file:=$FULL_PATH/$BASE_FOLDER/$DISPOSITION_ACTIVATION_FILENAME"
    echo "Launching command: $LAUNCH_COMMAND"
    gnome-terminal -- bash -c ". $INSTALL_PATH; $LAUNCH_COMMAND; exec bash"
    sleep 3

    # Connector service configuration
    CONNECTOR_BASEURL="http://localhost:$PORT"

    # Setup application through connector
    echo "Uploading profiles"
    curl -X POST $CONNECTOR_BASEURL$PROFILE_LOAD_PATH -H "Content-Type: application/json" -d "$ETHIC_PROFILES"

    echo "Uploading user status"
    curl -X POST $CONNECTOR_BASEURL$USER_STATUS_PATH -H "Content-Type: application/json" -d "$USER_STATUS"

    echo "Setting goal"
    curl -X POST $CONNECTOR_BASEURL$SET_GOAL_PATH -H "Content-Type: application/json" -d "$GOAL"

    # Publish base context (profile should be selected)
    echo "Publishing base context"
    ros2 topic pub --once /$ROBOT_NAME/current_context std_msgs/msg/String "{data: '$CONTEXT'}"

    echo -e "Configuration of $ROBOT_NAME complete\n"
}

start_mission() {

    local ROBOT_NAME=$1

    echo "Starting mission execution..."
    ros2 service call /$ROBOT_NAME/start std_srvs/srv/Empty
}

if [ "$#" -lt 2 ]; then
  echo "Usage: $0 <first_user> <second_user> [<start_mission>]"
  exit 1
fi

R1=$1
R2=$2
START=${3:-}

# First robot startup
configure_robot $1 $ROBOT_BASE_NAME$1 5000

# Second robot startup
configure_robot $2 $ROBOT_BASE_NAME$2 5001

sleep 3

# Start the mission if third parameter has been set to Y or y
if [ "$START" == "Y" ] || [ "$START" == "y" ]; then

    # First robot mission start
    start_mission $ROBOT_BASE_NAME$1

    # Second robot mission start
    start_mission $ROBOT_BASE_NAME$2
fi