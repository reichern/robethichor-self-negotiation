#!/bin/bash

# Run this file as: ./run_usecase.bash [--force-config <N> <P>] [--wait <time>] [--names <robot1_name> <robot2_name>] [--hosts <host1> <host2>] [--ports <port1> <port2>] [--launch <true/false>]
# Example: ./run_experiment.bash --force-config "50 100 200 300 400" "10 25 50 75 100" --wait 20 --names "robassistant_1" "robassistant_2" --hosts "localhost" "localhost" --ports 5000 5001 --launch true

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
    local HOST=$2
    local PORT=$3
    local N=$4
    local P=$5
    local I=$6
    local C=$7

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
    echo "Publishing base context"
    ros2 topic pub --once /$ROBOT_NAME/current_context std_msgs/msg/String "{data: '$CONTEXT'}"

    echo -e "Configuration of $ROBOT_NAME complete\n"
}

start_mission() {

    local ROBOT_NAME=$1
    local HOST=$2
    local PORT=$3
    local N=$4
    local P=$5
    local I=$6
    local C=$7

    echo "Starting mission execution..."

    GOAL="{\"goal\": \""$N" dispositions, "$P" activated conditions, ethicprofile no. "$I". Test counter: "$C"\"}"
  
    curl -X POST http://$HOST:$PORT$SET_GOAL_PATH -H "Content-Type: application/json" -d "$GOAL"
}

N_VALUES=(50 100 200 300 400)
P_VALUES=(10 25 50 75 100)
C=10

WAIT_TIME=20

R1_NAME="robassistant_1"
R2_NAME="robassistant_2"

R1_HOST="localhost"
R2_HOST="localhost"

R1_PORT=5000
R2_PORT=5001

LAUNCH=false

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
    --names)
      shift
      R1_NAME=$1
      shift
      R2_NAME=$1
      shift
      ;;
    --hosts)
      shift
      R1_HOST=$1
      shift
      R2_HOST=$1
      shift
      ;;
    --ports)
      shift
      R1_PORT=$1
      shift
      R2_PORT=$1
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
  # First robot startup
  start_robot $R1_NAME $R1_PORT

  # Second robot startup
  start_robot $R2_NAME $R2_PORT
fi

for N in "${N_VALUES[@]}"; do
  for P in "${P_VALUES[@]}"; do
    counter=1
    for (( i=1; i<=C; i++ )) do
      for (( j=1; j<=C; j++ )) do

        sleep 1
        echo "Running usecase no. $counter (i=$i, j=$j) with N=$N, P=$P"

        # Configure the robots with N and P parameters, and run the usecases
        # Robot 1
        configure_robot $R1_NAME $R1_HOST $R1_PORT $N $P $i $counter
        # Robot 2
        configure_robot $R2_NAME $R2_HOST $R2_PORT $N $P $j $counter

        sleep 2

        # Start the mission
        start_mission $R1_NAME $R1_HOST $R1_PORT $N $P $i $counter
        start_mission $R2_NAME $R2_HOST $R2_PORT $N $P $j $counter

        sleep $WAIT_TIME
        counter=$((counter+1))

      done
    done
  done
done