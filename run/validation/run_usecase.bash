#!/bin/bash

# from repo: https://github.com/RoboChor/robethichor-ethics-based-negotiation

# Run this file as: ./run_usecase.bash [--force-config <N> <P>] [--wait <time>] [--host <host>] [--port <port>] [--launch <true/false>]
# Example: ./run_usecase.bash --force-config "A B C D E F G H I J" "A B C D E F G H I J" --wait 10 --host "localhost" --port 5000 --launch true

# CONFIGURATION
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
CONTEXT_PATH="/context"
INT_PROFILE_LOAD_PATH="/interrupting/profile"
INT_USER_STATUS_PATH="/interrupting/status"
INT_SET_GOAL_PATH="/interrupting/goal"
INT_CONTEXT_PATH="/interrupting/context"

ROS_WS_PATH="../../../" # make the root directory as the workspace base folder
FULL_PATH=$(pwd)
INSTALL_PATH=$FULL_PATH"/"$ROS_WS_PATH"install/setup.bash"

start_robot() {

    local PORT=$1
    local GAZEBO=$2

    echo "------ Launching robot ------"

    # launch the ros environment
    LAUNCH_COMMAND="ros2 launch robethichor robethichor.launch.py port:=$PORT ethical_implication_file:=$FULL_PATH/$BASE_FOLDER$ETHICAL_IMPLICATIONS_FILENAME disposition_activation_file:=$FULL_PATH/$BASE_FOLDER$DISPOSITION_ACTIVATION_FILENAME log_output_file:=$FULL_PATH/$LOG_FILE gazebo:=$GAZEBO"
    gnome-terminal -- bash -c ". $INSTALL_PATH; $LAUNCH_COMMAND; exec bash"
    sleep 3
}

start_gazebo(){

    echo "------ Start up Gazebo -----"

    # launch gazebo
    LAUNCH_GAZEBO_COMMAND="ros2 launch robethichor gazebo.launch.py"
    gnome-terminal -- bash -c ". $INSTALL_PATH; $LAUNCH_GAZEBO_COMMAND; exec bash"
    sleep 5
    ros2 topic pub --once /custom_text_marker visualization_msgs/msg/Marker "{'header': {'stamp': 'now','frame_id': 'map'},'id': 0,'type': 9,'action': 0,'pose': {'position': {'x': 5,'y':1, 'z':0},'orientation': {'x': 0, 'y':0, 'z':0, 'w':1}},'scale': {'x': 1, 'y':1, 'z':1},'color': {'r':0.0, 'g':0.0, 'b':0.0, 'a':1.0},'lifetime': {'sec':0},'text': '101'}"
    ros2 topic pub --once /custom_text_marker visualization_msgs/msg/Marker "{'header': {'stamp': 'now','frame_id': 'map'},'id': 1,'type': 9,'action': 0,'pose': {'position': {'x': 5,'y':-2, 'z':0},'orientation': {'x': 0, 'y':0, 'z':0, 'w':1}},'scale': {'x': 1, 'y':1, 'z':1},'color': {'r':0.0, 'g':0.0, 'b':0.0, 'a':1.0},'lifetime': {'sec':0},'text': '102'}"
    sleep 5
}

configure_robot() {

    local HOST=$1
    local PORT=$2
    local USER_LABEL=$3

    echo "------ Configuring robot for user $USER_LABEL------"

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

    # Publish base context (profile should be selected)
    echo "Uploading base context"
    curl -X POST $CONNECTOR_BASEURL$CONTEXT_PATH -H "Content-Type: application/json" -d "$CONTEXT"
    
    echo -e "Configuration of robot complete for user $USER_LABEL\n"
}

configure_interrupt() {

    local HOST=$1
    local PORT=$2
    local USER_LABEL=$3
    local ACTIVE_USER=$4

    echo "------ Configuring interrupt for  $USER_LABEL! ------"

    # JSON file read
    ETHIC_PROFILES=$(cat $BASE_FOLDER$USER_LABEL"/"$ETHIC_PROFILES_FILE)
    USER_STATUS=$(cat $BASE_FOLDER$USER_LABEL"/"$USER_STATUS_FILE)
    GOAL=$(cat $BASE_FOLDER$USER_LABEL"/"$GOAL_FILE  | jq --arg user "$USER_LABEL" '.goal  += " (User: " + $user + ")"')
    # GOAL=$(cat $BASE_FOLDER$USER_LABEL"/"$GOAL_FILE | jq '.goal' )

    # Json sent as messages in topics should be escaped
    CONTEXT=$(cat $BASE_FOLDER$USER_LABEL"/"$CONTEXT_FILE | jq -c .)

    # Connector service configuration
    CONNECTOR_BASEURL="http://$HOST:$PORT"


    # Setup application through connector

    echo "Setting goal"
    curl -X POST $CONNECTOR_BASEURL$INT_SET_GOAL_PATH -H "Content-Type: application/json" -d "$GOAL"

    # sleep 1

    echo "Uploading profiles"
    curl -X POST $CONNECTOR_BASEURL$INT_PROFILE_LOAD_PATH -H "Content-Type: application/json" -d "$ETHIC_PROFILES"

    echo "Uploading user status"
    curl -X POST $CONNECTOR_BASEURL$INT_USER_STATUS_PATH -H "Content-Type: application/json" -d "$USER_STATUS"


    # Publish base context (profile should be selected)
    echo "Uploading base context"
    curl -X POST $CONNECTOR_BASEURL$INT_CONTEXT_PATH -H "Content-Type: application/json" -d "$CONTEXT"

    echo -e "Configuration of interrupting user $USER_LABEL complete\n"

}

start_mission() {

    local HOST=$1
    local PORT=$2
    local USER_LABEL=$3
    local INTERRUPTED_BY=$4
    local GAZEBO=$5
    
    echo "Starting mission execution for user $USER_LABEL"
    GOAL=$(cat $BASE_FOLDER$USER_LABEL"/"$GOAL_FILE | jq --arg user "$USER_LABEL" '.goal  += " (User: " + $user + ")"')
    # GOAL=$(cat $BASE_FOLDER$USER_LABEL"/"$GOAL_FILE | jq '.goal') # 

    curl -X POST http://$HOST:$PORT$SET_GOAL_PATH -H "Content-Type: application/json" -d "$GOAL"
}


USER_1=(A B C D E F G H I J)
USER_2=(A B C D E F G H I J)

WAIT_TIME=10

GAZEBO=FALSE

HOST="localhost"

PORT=5000

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
    --gazebo)
      shift
      GAZEBO=$1
      shift
      ;;
    --host)
      shift
      HOST=$1
      shift
      ;;
    --port)
      shift
      PORT=$1
      shift
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
    if [ "$GAZEBO" = true ]; then
        # gazebo startup
        start_gazebo
    fi
    # First robot startup
    start_robot $PORT $GAZEBO
fi


for U1 in "${USER_1[@]}"; do
    # Robot configuration
    configure_robot $HOST $PORT $U1
    sleep 2
    for U2 in "${USER_2[@]}"; do
        if [ "$U1" != "$U2" ]; then
            echo "------ Running simulation $U1 interrupted by $U2 ------"

            if [ "$GAZEBO" = true ]; then
                sleep 10
            fi
            # Starts the mission for the robot
            start_mission $HOST $PORT $U1 $U2 $GAZEBO

            sleep 3
            if [ "$GAZEBO" = true ]; then
                sleep 3
            fi

            # Configure interrupting user and send mission request
            configure_interrupt $HOST $PORT $U2 $U1

            sleep $WAIT_TIME

            if [ "$GAZEBO" = true ]; then
              echo "Reset robot to original position"
              # ros2 topic pub --once /goal std_msgs/msg/String '{data: "Reset robot..."}'
              ros2 action send_goal /navigate_to_pose nav2_msgs/action/NavigateToPose "pose: {header: {frame_id: map}, pose: {position: {x: 0.0, y: 0.0, z: 0.0}, orientation:{x: 0.0, y: 0.0, z: 0, w: 1.0000000}}}"
              echo "waiting for robot to reach position ... "
            fi
        fi
    done
done
