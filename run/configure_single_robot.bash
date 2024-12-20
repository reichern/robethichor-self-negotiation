#!/bin/bash

# Robot name
ROBOT_NAME="robassistant_1"

# JSON files paths
BASE_CONTEXT_FILE="context_base.json"
UPDATE_CONTEXT_FILE="context_update.json"
USER_STATUS_FILE="user_status.json"
ETHIC_PROFILES_FILE="ethic_profiles.json"
GOAL_FILE="goal.json"

# Connector service configuration
CONNECTOR_BASEURL="http://localhost:5000"
PROFILE_LOAD_PATH="/loadEthicProfile"
USER_STATUS_PATH="/setUserStatus"
SET_GOAL_PATH="/setGoal"

# JSON file read
ETHIC_PROFILES=$(cat $ETHIC_PROFILES_FILE)
USER_STATUS=$(cat $USER_STATUS_FILE)
GOAL=$(cat $GOAL_FILE)

# Json sent as messages in topics should be escaped
BASE_CONTEXT=$(cat $BASE_CONTEXT_FILE | jq -c .)
UPDATE_CONTEXT=$(cat $UPDATE_CONTEXT_FILE | jq -c .)

# Setup application through connector
echo "Uploading profiles"
curl -X POST $CONNECTOR_BASEURL$PROFILE_LOAD_PATH -H "Content-Type: application/json" -d "$ETHIC_PROFILES"

echo "Uploading user status"
curl -X POST $CONNECTOR_BASEURL$USER_STATUS_PATH -H "Content-Type: application/json" -d "$USER_STATUS"


echo "Setting goal"
curl -X POST $CONNECTOR_BASEURL$SET_GOAL_PATH -H "Content-Type: application/json" -d "$GOAL"

# Publish base context (profile should be selected)
echo "Publishing base context"
ros2 topic pub --once /$ROBOT_NAME/current_context std_msgs/msg/String "{data: '$BASE_CONTEXT'}"
echo "Context published"

echo "Complete"