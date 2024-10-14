#!/bin/bash

# ID generator
generate_id() {
  echo $(uuidgen)
}

publish_message() {
  local id=$1
  local key=$2
  local content=$3
  local json="{\"id\":\"$id\", \"key\":\"$key\", \"content\":$content}"
  echo $json | ros2 topic pub --once /negotiation_msgs std_msgs/msg/String "{data: '$json'}"
}

id=$(generate_id)
echo "Robot negotiation uuid: $id"

# Main loop
while true; do
  echo "Select an option:"
  echo "1. Send dice to be receiver"
  echo "2. Send dice to be sender"
  echo "3. Send first offer"
  echo "4. Send second offer"
  echo "5. Send third offfer"
  echo "6. Send reject decision"
  echo "7. Send accept decision"
  echo "8. Send skip"
  echo "9. Send quit"
  echo "10. Exit"

  read -p "Select: " option


  case $option in
    1)
      publish_message $id "dice" 1
      ;;
    2)
      publish_message $id "dice" 100000
      ;;
    3)
      publish_message $id "offer" "{\"task\": [\"t1\"], \"conditions\": [\"crowd_anxiety\"]}"
      ;;
    4)
      publish_message $id "offer" "{\"task\": [\"t1\"], \"conditions\": [\"crowd_anxiety\", \"elderly\"]}"
      ;;
    5)
      publish_message $id "offer" "{\"task\": [\"t1\"], \"conditions\": [\"pregnant\", \"elderly\", \"crowd_anxiety\"]}"
      ;;
    6)
      publish_message $id "decision" "\"reject\""
      ;;
    7)
      publish_message $id "decision" "\"accept\""
      ;;
    8)
      publish_message $id "decision" "\"skip\""
      ;;
    9)
      publish_message $id "quit" "true"
      ;;
    10)
      echo "Loop exit."
      break
      ;;
    *)
      echo "Option not valid. Try again."
      ;;
  esac
done

echo "End."