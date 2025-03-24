#!/bin/bash

# from repo: https://github.com/RoboChor/robethichor-ethics-based-negotiation

# Run this file as: ./run_usecase.bash [--force-config <N> <P>] [--wait <time>] [--names <robot1_name> <robot2_name>] [--hosts <host1> <host2>] [--ports <port1> <port2>] [--launch <true/false>]
# Example: ./run_usecase.bash --force-config "A B C D E F G H I J" "A B C D E F G H I J" --wait 10 --names "robassistant_1" "robassistant_2" --hosts "localhost" "localhost" --ports 5000 5001 --launch true

# CONFIGURATION
ROBOT_BASE_NAME="robassistant_"
LOG_FILE="results/results.log"

# JSON files paths
BASE_FOLDER="usecases/"
CONTEXT_FILE="context.json"
USER_STATUS_FILE="user_status.json"
ETHIC_PROFILES_FILE="ethic_profiles.json"
GOAL_FILE="goal.json"
ETHICAL_IMPLICATIONS_FILENAME="ethical_implications.json"
DISPOSITION_ACTIVATION_FILENAME="disposition_activation.json"

# Connector service information
PROFILE_LOAD_PATH="/profile"
USER_STATUS_PATH="/status"
SET_GOAL_PATH="/goal"
INT_PROFILE_LOAD_PATH="/interrupting/profile"
INT_USER_STATUS_PATH="/interrupting/status"
INT_SET_GOAL_PATH="/interrupting/goal"

ROS_WS_PATH="../../" # make the root directory as the workspace base folder
FULL_PATH=$(pwd)
INSTALL_PATH=$FULL_PATH"/"$ROS_WS_PATH"install/setup.bash"

FIRST=true

start_robot() {

    local ROBOT_NAME=$1
    local PORT=$2

    echo "------ Running $ROBOT_NAME ------"

    # Ros 2 launch (to launch the ros environment)
    LAUNCH_COMMAND="ros2 launch robethichor robethichor_launch.py ns:=$ROBOT_NAME port:=$PORT ethical_implication_file:=$FULL_PATH/$BASE_FOLDER$ETHICAL_IMPLICATIONS_FILENAME disposition_activation_file:=$FULL_PATH/$BASE_FOLDER$DISPOSITION_ACTIVATION_FILENAME log_output_file:=$FULL_PATH/$LOG_FILE"
    echo "Launching command: $LAUNCH_COMMAND"
    gnome-terminal -- bash -c ". $INSTALL_PATH; $LAUNCH_COMMAND; exec bash"
    sleep 3
}

configure_robot() {

    local ROBOT_NAME=$1
    local HOST=$2
    local PORT=$3

    local USER_LABEL=$4

    echo "------ Configuring $ROBOT_NAME for user $USER_LABEL------"

    # JSON file read
    ETHIC_PROFILES=$(cat $BASE_FOLDER$USER_LABEL"/"$ETHIC_PROFILES_FILE)
    USER_STATUS=$(cat $BASE_FOLDER$USER_LABEL"/"$USER_STATUS_FILE)

    # Json sent as messages in topics should be escaped
    CONTEXT=$(cat $BASE_FOLDER$USER_LABEL"/"$CONTEXT_FILE | jq -c .)

    # Connector service configuration
    CONNECTOR_BASEURL="http://$HOST:$PORT"

    # Setup application through connector
    echo "Uploading profiles"
    curl -X POST $CONNECTOR_BASEURL$PROFILE_LOAD_PATH -H "Content-Type: application/json" -d "$ETHIC_PROFILES"

    echo "Uploading user status"
    curl -X POST $CONNECTOR_BASEURL$USER_STATUS_PATH -H "Content-Type: application/json" -d "$USER_STATUS"

    # echo "Setting goal"
    # curl -X POST $CONNECTOR_BASEURL$SET_GOAL_PATH -H "Content-Type: application/json" -d "$GOAL"

    # Publish base context (profile should be selected)
    echo "Publishing base context"
    ros2 topic pub --once /$ROBOT_NAME/current_context std_msgs/msg/String "{data: '$CONTEXT'}"

    echo -e "Configuration of $ROBOT_NAME complete for user $USER_LABEL\n"
}

configure_interrupt() {

    local ROBOT_NAME=$1
    local HOST=$2
    local PORT=$3

    local USER_LABEL=$4
    local ACTIVE_USER=$5

    echo "------ User $USER_LABEL interrupting $ROBOT_NAME with active user $ACTIVE_USER------"

    # launch nodes for interruption
    echo "launch interupting nodes? $FIRST"
    if [ "$FIRST" = true ]; then
        LAUNCH_COMMAND="ros2 launch robethichor robethichor_interruption_launch.py ns:=$ROBOT_NAME/interrupting_user"
        echo "Launching command: $LAUNCH_COMMAND"
        gnome-terminal -- bash -c ". $INSTALL_PATH; $LAUNCH_COMMAND; exec bash"
        sleep 3
        FIRST=false
    fi

    # JSON file read
    ETHIC_PROFILES=$(cat $BASE_FOLDER$USER_LABEL"/"$ETHIC_PROFILES_FILE)
    USER_STATUS=$(cat $BASE_FOLDER$USER_LABEL"/"$USER_STATUS_FILE)

    # Json sent as messages in topics should be escaped
    CONTEXT=$(cat $BASE_FOLDER$USER_LABEL"/"$CONTEXT_FILE | jq -c .)

    # Connector service configuration
    CONNECTOR_BASEURL="http://$HOST:$PORT"


    # Setup application through connector
    echo "Uploading profiles"
    curl -X POST $CONNECTOR_BASEURL$INT_PROFILE_LOAD_PATH -H "Content-Type: application/json" -d "$ETHIC_PROFILES"

    echo "Uploading user status"
    curl -X POST $CONNECTOR_BASEURL$INT_USER_STATUS_PATH -H "Content-Type: application/json" -d "$USER_STATUS"

    # echo "Setting goal"
    # curl -X POST $CONNECTOR_BASEURL$SET_GOAL_PATH -H "Content-Type: application/json" -d "$GOAL"

    # Publish base context (profile should be selected)
    echo "Publishing base context"
    ros2 topic pub --once /$ROBOT_NAME/interrupting_user/current_context std_msgs/msg/String "{data: '$CONTEXT'}"

    echo -e "Configuration of interrupting user $USER_LABEL complete\n"

    sleep 3

    # send interrupt signal
    echo "Sending interrupt signal"
    ros2 topic pub --once /$ROBOT_NAME/interrupt std_msgs/msg/Bool "{data: True}"

}

start_mission() {

    local ROBOT_NAME=$1
    local HOST=$2
    local PORT=$3

    local USER_LABEL=$4
    local INTERRUPTED_BY=$5

    echo "Starting mission execution..."

    GOAL=$(cat $BASE_FOLDER$USER_LABEL"/"$GOAL_FILE | jq --arg user "$USER_LABEL" --arg interrupted "$INTERRUPTED_BY" '.goal += " (User: " + $user + " starting mission! interrupted by " + $interrupted)')

    curl -X POST http://$HOST:$PORT$SET_GOAL_PATH -H "Content-Type: application/json" -d "$GOAL"
}


USER_1=(A B C)
USER_2=(A B C)

WAIT_TIME=5

R1_NAME="robassistant_1"
# R2_NAME="robassistant_2"

R1_HOST="localhost"
# R2_HOST="localhost"

R1_PORT=5000
# R2_PORT=5001

LAUNCH=false

while [[ "$#" -gt 0 ]]; do
  case $1 in
    --force-config)
      shift
      USER_1=($1)
      shift
      USER_2=($1)
      shift
      ;;
    --wait)
      shift
      WAIT_TIME=$1
      shift
      ;;
    --names)
      shift
      R1_NAME=$1
      shift
    #   R2_NAME=$1
    #   shift
      ;;
    --hosts)
      shift
      R1_HOST=$1
      shift
    #   R2_HOST=$1
    #   shift
      ;;
    --ports)
      shift
      R1_PORT=$1
      shift
    #   R2_PORT=$1
    #   shift
      ;;
    --launch)
      shift
      LAUNCH=$1
      shift
      ;;
    *)
      echo "Unknown parameter passed: $1"
      exit 1
      ;;
  esac
done


if [ "$LAUNCH" = true ]; then
    # First robot startup
    start_robot $R1_NAME $R1_PORT

    # Second robot startup
    # start_robot $R2_NAME $R2_PORT
fi

for U1 in "${USER_1[@]}"; do
    for U2 in "${USER_2[@]}"; do
        if [ "$U1" != "$U2" ]; then
            echo "Running simulation for $U1 against $U2"

            # Robot configuration
            configure_robot $R1_NAME $R1_HOST $R1_PORT $U1

            # Second robot configuration
            # configure_robot $R2_NAME $R2_HOST $R2_PORT $U2

            sleep 2

            # Starts the mission for the robot
            start_mission $R1_NAME $R1_HOST $R1_PORT $U1 $U2

            sleep 2

            # Configure interrupting user and send mission request
            configure_interrupt $R1_NAME $R1_HOST $R1_PORT $U2 $U1
            # TODO start mission for interrupting user (has different path!! )
            # start_mission $R1_NAME $R1_HOST $R1_PORT $U2

            sleep $WAIT_TIME
        fi
    done
done
