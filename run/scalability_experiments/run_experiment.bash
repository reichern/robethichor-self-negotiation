#!/bin/bash

# Run this file as: ./run_usecase.bash <n_dispositions> <activated_dispositions_percentage>
# Example: ./run_usecase.bash 50 25

LOG_FILE="results/results.log"

# JSON files paths
BASE_FOLDER="test_cases/"
CONTEXT_FILE="context.json"
ETHICAL_IMPLICATIONS_FILENAME="ethical_implications.json"
DISPOSITION_ACTIVATION_FILENAME="disposition_activation.json"

# Connector service information
PROFILE_LOAD_PATH="/loadEthicProfile"
USER_STATUS_PATH="/setUserStatus"
SET_GOAL_PATH="/setGoal"

ROS_WS_PATH="../../../" # this directory is inside the ws/src folder

start_robot() {

    local ROBOT_NAME=$1
    local PORT=$2

    echo "------ Running $ROBOT_NAME ------"

    # Ros 2 launch (to launch the ros environment)
    FULL_PATH=$(pwd)
    INSTALL_PATH=$FULL_PATH"/"$ROS_WS_PATH"install/setup.bash"
    LAUNCH_COMMAND="ros2 launch robethichor robethichor_launch.py ns:=$ROBOT_NAME port:=$PORT ethical_implication_file:=$FULL_PATH/$BASE_FOLDER$ETHICAL_IMPLICATIONS_FILENAME disposition_activation_file:=$FULL_PATH/$BASE_FOLDER$DISPOSITION_ACTIVATION_FILENAME log_output_file:=$FULL_PATH/$LOG_FILE"
    echo "Launching command: $LAUNCH_COMMAND"
    gnome-terminal -- bash -c ". $INSTALL_PATH; $LAUNCH_COMMAND; exec bash"
    sleep 3
}

configure_robot() {
    local ROBOT_NAME=$1
    local PORT=$2
    local N=$3
    local P=$4
    local I=$5

    # JSON file read
    ETHIC_PROFILES=$(cat $BASE_FOLDER$N"/profiles/ethic_profiles_"$N"-"$I".json")
    USER_STATUS=$(cat $BASE_FOLDER$N"/statuses/"$P"/user_status_"$N"_"$P"-"$I".json")
    GOAL="{\"goal\": \""$N"_"$P"-"$I"\"}"

    # Json sent as messages in topics should be escaped
    CONTEXT=$(cat $BASE_FOLDER$CONTEXT_FILE | jq -c .)

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
    #ros2 service call /$ROBOT_NAME/start std_srvs/srv/Empty
    ros2 topic pub --once /start std_msgs/msg/Empty "{}"
}

if [ "$#" -lt 2 ]; then
  echo "Usage: $0 <n_dispositions> <activated_dispositions_percentage> <number_of_cases>"
  exit 1
fi

if ! [[ "$3" =~ ^[0-9]+$ ]]; then
  echo "Parameter <number_of_cases> must be an integer number!"
  exit 1
fi

N=$1
P=$2
C=$3

R1="robassistant_1"
R2="robassistant_2"

# First robot startup
start_robot $R1 5000

# Second robot startup
start_robot $R2 5001

for (( i=1; i<=C; i++ )) do
  for (( j=1; j<=C; j++ )) do
    echo "Running usecase with i=$i and j=$j"

    sleep 3

    # Configure the robots with N and P parameters, and run the usecases
    # Robot 1
    configure_robot $R1 5000 $N $P $i

    # Robot 2
    configure_robot $R2 5001 $N $P $j

    sleep 3

    # Start the mission
    start_mission

    sleep 5
  done
done