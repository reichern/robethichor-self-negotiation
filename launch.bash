#!/bin/bash

# Run this file as: ./launch.bash [--name <robot_name> ] [--port <port>]
# Example: ./launch.bash --name "robassistant" --port 5000

# CONFIGURATION
LOG_FILE="results/results.log"
ETHICAL_IMPLICATIONS_FILENAME="configuration/ethical_implications.json"
DISPOSITION_ACTIVATION_FILENAME="configuration/disposition_activation.json"

NAMESPACE="robassistant"
PORT=5000

start_robot() {

    local ROBOT_NAME=$1
    local PORT=$2

    # Ros 2 launch (to launch the ros environment)
    FULL_PATH=$(pwd)
    LAUNCH_COMMAND="ros2 launch robethichor robethichor_launch.py ns:=$ROBOT_NAME port:=$PORT ethical_implication_file:=$FULL_PATH/$BASE_FOLDER$ETHICAL_IMPLICATIONS_FILENAME disposition_activation_file:=$FULL_PATH/$BASE_FOLDER$DISPOSITION_ACTIVATION_FILENAME log_output_file:=$FULL_PATH/$LOG_FILE"
    $LAUNCH_COMMAND 2>&1

}

# configure_robot() {

#     local ROBOT_NAME=$1
#     local HOST=$2
#     local PORT=$3

#     local USER_LABEL=$4

#     echo "------ Configuring $ROBOT_NAME for user $USER_LABEL------"

#     # JSON file read
#     ETHIC_PROFILES=$(cat $BASE_FOLDER$USER_LABEL"/"$ETHIC_PROFILES_FILE)
#     USER_STATUS=$(cat $BASE_FOLDER$USER_LABEL"/"$USER_STATUS_FILE)

#     # Json sent as messages in topics should be escaped
#     CONTEXT=$(cat $BASE_FOLDER$USER_LABEL"/"$CONTEXT_FILE | jq -c .)

#     # Connector service configuration
#     CONNECTOR_BASEURL="http://$HOST:$PORT"

#     # Setup application through connector
#     echo "Uploading profiles"
#     curl -X POST $CONNECTOR_BASEURL$PROFILE_LOAD_PATH -H "Content-Type: application/json" -d "$ETHIC_PROFILES"

#     echo "Uploading user status"
#     curl -X POST $CONNECTOR_BASEURL$USER_STATUS_PATH -H "Content-Type: application/json" -d "$USER_STATUS"

#     # Publish base context (profile should be selected)
#     echo "Publishing base context"
#     ros2 topic pub --once /$ROBOT_NAME/current_context std_msgs/msg/String "{data: '$CONTEXT'}"

#     echo -e "Configuration of $ROBOT_NAME complete for user $USER_LABEL\n"
# }

# start_mission() {

#     local ROBOT_NAME=$1
#     local HOST=$2
#     local PORT=$3

#     local USER_LABEL=$4
#     local NEGOTIATING_AGAINST=$5

#     echo "Starting mission execution..."

#     GOAL=$(cat $BASE_FOLDER$USER_LABEL"/"$GOAL_FILE | jq --arg user "$USER_LABEL" --arg against "$NEGOTIATING_AGAINST" '.goal += " (User: " + $user + ", Negotiating against: " + $against + ")"')

#     curl -X POST http://$HOST:$PORT$SET_GOAL_PATH -H "Content-Type: application/json" -d "$GOAL"
# }


while [[ "$#" -gt 0 ]]; do
  case $1 in
    --namespace)
      shift
      NAMESPACE=$1
      shift
      ;;
    --port)
      shift
      PORT=$1
      shift
      ;;
    *)
      echo "Unknown parameter passed: $1"
      exit 1
      ;;
  esac
done

start_robot $NAMESPACE $PORT

#configure_robot $R1_NAME $R1_HOST $R1_PORT $U1
