#!/bin/bash

# from repo: https://github.com/RoboChor/robethichor-ethics-based-negotiation

# Run this file as: ./run_usecase.bash [--force-config <N> <P>] [--wait <time>] [--names <robot1_name> <robot2_name>] [--hosts <host1> <host2>] [--ports <port1> <port2>] [--launch <true/false>]
# Example: ./run_usecase.bash --force-config "A B C D E F G H I J" "A B C D E F G H I J" --wait 10 --names "robassistant_1" "robassistant_2" --hosts "localhost" "localhost" --ports 5000 5001 --launch true

# CONFIGURATION
LOG_FILE="results/results.log"

# JSON files paths
BASE_FOLDER="test_cases/"
CONTEXT_FILE="context.json"
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

ROS_WS_PATH="../../" # make the root directory as the workspace base folder
FULL_PATH=$(pwd)
INSTALL_PATH=$FULL_PATH"/"$ROS_WS_PATH"install/setup.bash"

start_robot() {

    local PORT=$1
    local GAZEBO=$2

    echo "------ Launching robot ------"

    # launch the ros environment
    LAUNCH_COMMAND="ros2 launch robethichor robethichor.launch.py port:=$PORT ethical_implication_file:=$FULL_PATH/$BASE_FOLDER$ETHICAL_IMPLICATIONS_FILENAME disposition_activation_file:=$FULL_PATH/$BASE_FOLDER$DISPOSITION_ACTIVATION_FILENAME log_output_file:=$FULL_PATH/$LOG_FILE gazebo:=$GAZEBO"
    # echo "Launching command: $LAUNCH_COMMAND"
    gnome-terminal -- bash -c ". $INSTALL_PATH; $LAUNCH_COMMAND; exec bash"
    sleep 3
}

start_gazebo(){

    echo "------ Start up Gazebo -----"

    # launch gazebo
    LAUNCH_GAZEBO_COMMAND="ros2 launch robethichor gazebo.launch.py"
    # echo "Launching command: $LAUNCH_GAZEBO_COMMAND"
    gnome-terminal -- bash -c ". $INSTALL_PATH; $LAUNCH_GAZEBO_COMMAND; exec bash"
    sleep 5
    ros2 topic pub --once /custom_text_marker visualization_msgs/msg/Marker "{'header': {'stamp': 'now','frame_id': 'map'},'id': 0,'type': 9,'action': 0,'pose': {'position': {'x': 5,'y':1, 'z':0},'orientation': {'x': 0, 'y':0, 'z':0, 'w':1}},'scale': {'x': 1, 'y':1, 'z':1},'color': {'r':0.0, 'g':0.0, 'b':0.0, 'a':1.0},'lifetime': {'sec':0},'text': '101'}"
    # sleep 1
    ros2 topic pub --once /custom_text_marker visualization_msgs/msg/Marker "{'header': {'stamp': 'now','frame_id': 'map'},'id': 1,'type': 9,'action': 0,'pose': {'position': {'x': 5,'y':-2, 'z':0},'orientation': {'x': 0, 'y':0, 'z':0, 'w':1}},'scale': {'x': 1, 'y':1, 'z':1},'color': {'r':0.0, 'g':0.0, 'b':0.0, 'a':1.0},'lifetime': {'sec':0},'text': '102'}"
    sleep 5
}

configure_robot() {

    local HOST=$1
    local PORT=$2
    local N=$3
    local P=$4
    local I=$5
    local C=$6

    echo "------ Configuring robot for user $I------"

    # JSON file read
    ETHIC_PROFILES=$(cat $BASE_FOLDER$N"/profiles/ethic_profiles_"$N"-"$I".json")
    USER_STATUS=$(cat $BASE_FOLDER$N"/statuses/"$P"/user_status_"$N"_"$P"-"$I".json")
 
    # Json sent as messages in topics should be escaped
    CONTEXT=$(cat $BASE_FOLDER$CONTEXT_FILE | jq -c .)
    
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
    echo "Uploading base context"
    curl -X POST $CONNECTOR_BASEURL$CONTEXT_PATH -H "Content-Type: application/json" -d "$CONTEXT"
    # echo "Publishing base context"
    # ros2 topic pub --once /current_context std_msgs/msg/String "{data: '$CONTEXT'}"

    echo -e "Configuration of robot complete\n"
}

configure_interrupt() {

    local HOST=$1
    local PORT=$2
    local N=$3
    local P=$4
    local I=$5
    local C=$6

    echo "------ Configuring interrupt for user $I! ------"

    # launch nodes for interruption
    # echo "launch interupting nodes? $FIRST"
    # if [ "$FIRST" = true ]; then
    #     LAUNCH_COMMAND="ros2 launch robethichor robethichor_interruption.launch.py ns:=interrupting_user"
    #     echo "Launching command: $LAUNCH_COMMAND"
    #     gnome-terminal -- bash -c ". $INSTALL_PATH; $LAUNCH_COMMAND; exec bash"
    #     sleep 3
    #     FIRST=false
    # fi

    # JSON file read
    ETHIC_PROFILES=$(cat $BASE_FOLDER$N"/profiles/ethic_profiles_"$N"-"$I".json")
    USER_STATUS=$(cat $BASE_FOLDER$N"/statuses/"$P"/user_status_"$N"_"$P"-"$I".json")
 
    # Json sent as messages in topics should be escaped
    CONTEXT=$(cat $BASE_FOLDER$CONTEXT_FILE | jq -c .)
  
    # Connector service configuration
    CONNECTOR_BASEURL="http://$HOST:$PORT"


    # Setup application through connector
    
    echo "Sending goal"
    GOAL="{\"goal\": \"Scalability test! "$N" dispositions, "$P" activated conditions, ethicprofile no. "$I". Test counter: "$C"\"}"
    curl -X POST http://$HOST:$PORT$INT_SET_GOAL_PATH -H "Content-Type: application/json" -d "$GOAL"

    # sleep 1

    echo "Uploading profiles"
    curl -X POST $CONNECTOR_BASEURL$INT_PROFILE_LOAD_PATH -H "Content-Type: application/json" -d "$ETHIC_PROFILES"

    echo "Uploading user status"
    curl -X POST $CONNECTOR_BASEURL$INT_USER_STATUS_PATH -H "Content-Type: application/json" -d "$USER_STATUS"


    # Publish base context (profile should be selected)
    echo "Uploading base context"
    curl -X POST $CONNECTOR_BASEURL$INT_CONTEXT_PATH -H "Content-Type: application/json" -d "$CONTEXT"

    # echo "Publishing base context"
    # ros2 topic pub --once /interrupting_user/current_context std_msgs/msg/String "{data: '$CONTEXT'}"

    echo -e "Configuration of interrupting user $USER_LABEL complete\n"

    # send interrupt 
    # echo "Sending interrupt signal"
    # ros2 topic pub --once /interrupt std_msgs/msg/Bool "{data: True}"

}

start_mission() {

    local HOST=$1
    local PORT=$2
    local N=$3
    local P=$4
    local I=$5
    local C=$6
    local GAZEBO=$7


    # if [ "$GAZEBO" = true ]; then
    #   echo "Reset robot to original position"
    #   ros2 action send_goal /navigate_to_pose nav2_msgs/action/NavigateToPose "pose: {header: {frame_id: map}, pose: {position: {x: 0.0, y: 0.0, z: 0.0}, orientation:{x: 0.0, y: 0.0, z: 0, w: 1.0000000}}}"
    #   echo "waiting for robot to reach position ... "
    #   sleep 10
    # fi
    
    echo "Starting mission execution for user $I"
    GOAL="{\"goal\": \"Scalability test! "$N" dispositions, "$P" activated conditions, ethicprofile no. "$I". Test counter: "$C"\"}"
 
    curl -X POST http://$HOST:$PORT$SET_GOAL_PATH -H "Content-Type: application/json" -d "$GOAL"
}

N_VALUES=(100 25 50 100) # 10 25 50 100 200 300
P_VALUES=(10 25 50 75 100)
C=10
 
WAIT_TIME=10

GAZEBO=FALSE

# R1_NAME="robassistant_1"
# R2_NAME="robassistant_2"

HOST="localhost"
# R2_HOST="localhost"

PORT=5000
# R2_PORT=5001

LAUNCH=false

STARTING_I=1
STARTING_J=1
STARTING_COUNTER=1

while [[ "$#" -gt 0 ]]; do
  case $1 in
    --force-config)
      shift
      N_VALUES=($1)
      shift
      P_VALUES=($1)
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
    --launch)
      shift
      LAUNCH=$1
      shift
      ;;
    --start-params)
      shift
      STARTING_I=$1
      shift
      STARTING_J=$1
      shift
      STARTING_COUNTER=$1
      shift
      ;;
    *)
      echo "Unknown parameter passed: $1"
      exit 1
      ;;
  esac
done



if [ "$GAZEBO" = true ]; then
    # gazebo startup
    start_gazebo
fi

if [ "$LAUNCH" = true ]; then
    # robot startup
    start_robot $PORT $GAZEBO
fi


for N in "${N_VALUES[@]}"; do
  for P in "${P_VALUES[@]}"; do
    counter=$STARTING_COUNTER
    for (( i=$STARTING_I; i<=C; i++ )) do

      # Robot configuration
      configure_robot $HOST $PORT $N $P $i $counter
      sleep 2

      for (( j=$STARTING_J; j<=C; j++ )) do
        if [ "$i" != "$j" ]; then
          echo "------ Running simulation U$i interrupted by U$j ------"

          # Starts the mission for the robot
          start_mission $HOST $PORT $N $P $i $counter $GAZEBO

          sleep 3

          # Configure interrupting user and send mission request
          configure_interrupt $HOST $PORT $N $P $j $counter

          sleep $WAIT_TIME
          counter=$((counter+1))
        fi
      done
    done
  done
done