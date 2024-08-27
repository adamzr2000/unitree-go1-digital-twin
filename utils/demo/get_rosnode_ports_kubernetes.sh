#!/bin/bash

# Define the mapping between ROS nodes and Kubernetes deployments
declare -A NODE_DEPLOYMENT_MAP
NODE_DEPLOYMENT_MAP["/rplidarNode"]="lidar-deployment"
NODE_DEPLOYMENT_MAP["/node_gesture_control"]="gesture-control-app-deployment"
NODE_DEPLOYMENT_MAP["/go1_controller"]="go1-base-deployment"
NODE_DEPLOYMENT_MAP["/OmniIsaacRosBridge"]="digital-twin-app-deployment"
NODE_DEPLOYMENT_MAP["/local_ekf/ekf_localization_with_odom"]="go1-base-deployment"
NODE_DEPLOYMENT_MAP["/cartographer_node"]="go1-navigation-deployment"
NODE_DEPLOYMENT_MAP["/monitoring_node_robot"]="monitoring-robot-deployment"
NODE_DEPLOYMENT_MAP["/monitoring_node_edge"]="monitoring-edge-deployment"

OUTPUT_FILE="output.csv"

echo "topic,sender,src-port,src-ip,receiver,dst-port,dst-ip" > $OUTPUT_FILE

# Get the pod name for roscore-edge-deployment
ROSCORE_POD_NAME=$(kubectl get pods --no-headers -o custom-columns=":metadata.name" | grep "^roscore-edge-deployment-")

if [ -z "$ROSCORE_POD_NAME" ]; then
    echo "No pod found for roscore-edge-deployment"
    exit 1
fi

# Define the list of ROS nodes
NODE_NAMES=(
    "/rplidarNode"
    "/node_gesture_control"
    "/go1_controller"
    "/OmniIsaacRosBridge"
    "/cartographer_node"
    "/local_ekf/ekf_localization_with_odom"
    "monitoring_node_edge"
    "monitoring_node_robot"
)

declare -A NODE_IP_MAP

for NODE_NAME in "${NODE_NAMES[@]}"; do
    DEPLOYMENT=${NODE_DEPLOYMENT_MAP[$NODE_NAME]}
    
    if [ -z "$DEPLOYMENT" ]; then
        echo "No deployment found for node: $NODE_NAME"
        continue
    fi
    
    # Get the pod names associated with the deployment
    POD_NAMES=$(kubectl get pods --no-headers -o custom-columns=":metadata.name" | grep "^${DEPLOYMENT}-")
    
    if [ -z "$POD_NAMES" ]; then
        echo "No pod found for deployment: $DEPLOYMENT"
    else
        for POD_NAME in $POD_NAMES; do
            # Get the IP address of the pod
            IP_ADDRESS=$(kubectl get pod $POD_NAME -o jsonpath="{.status.podIP}")
            NODE_IP_MAP[$NODE_NAME]=$IP_ADDRESS
        done
    fi
done

function extract_ports {
    ROS_TOPIC=$1
    NODE_SRC=$2
    NODE_DST=$3
    PORT_DIRECTION=$4

    SRC_IP=${NODE_IP_MAP[$NODE_SRC]}

    # Run the rosnode info command inside the roscore-edge-deployment pod and store the output
    OUTPUT=$(kubectl exec $ROSCORE_POD_NAME -- sh -c ". /opt/ros/noetic/setup.sh && rosnode info $NODE_SRC")
    
    if [ $? -ne 0 ]; then
        echo "Failed to exec rosnode info for $NODE_SRC in pod $ROSCORE_POD_NAME"
        echo "$ROS_TOPIC,$NODE_SRC,Not found,$SRC_IP,$NODE_DST,Not found,Not found" >> $OUTPUT_FILE
        return
    fi

    # Extract port numbers and destination IP
    while IFS= read -r line; do
        if [[ $line =~ "topic: $ROS_TOPIC" ]]; then
            read -r line
            if [[ $line =~ "to: $NODE_DST" ]]; then
                while IFS= read -r line; do
                    if [[ $line =~ "direction: $PORT_DIRECTION" ]]; then
                        SRC_PORT=$(echo "$line" | awk -F'[()]' '{print $2}' | awk '{print $1}')
                        DST_IP_PORT=$(echo "$line" | awk -F'[()]' '{print $2}' | awk '{print $3}')
                        DST_IP=$(echo "$DST_IP_PORT" | awk -F':' '{print $1}')
                        DST_PORT=$(echo "$DST_IP_PORT" | awk -F':' '{print $2}')
                        echo "$ROS_TOPIC,$NODE_SRC,$SRC_PORT,$SRC_IP,$NODE_DST,$DST_PORT,$DST_IP" >> $OUTPUT_FILE
                        return
                    fi
                done
            fi
        fi
    done <<< "$OUTPUT"

    echo "$ROS_TOPIC,$NODE_SRC,Not found,$SRC_IP,$NODE_DST,Not found,Not found" >> $OUTPUT_FILE
}

# Extract ports for specific nodes and topics
extract_ports "/scan" "/rplidarNode" "/cartographer_node" "outbound"
extract_ports "/go1_controller/cmd_vel" "/node_gesture_control" "/go1_controller" "outbound"
extract_ports "/joint_states" "/go1_controller" "/OmniIsaacRosBridge" "outbound"
extract_ports "/go1_controller/odom" "/go1_controller" "/monitoring_node_edge" "outbound"
extract_ports "/go1_controller/odom" "/go1_controller" "/cartographer_node" "outbound"
extract_ports "/odometry/filtered" "/local_ekf/ekf_localization_with_odom" "/cartographer_node" "outbound"

echo "Node and port information has been saved to $OUTPUT_FILE"
echo
echo "CSV Output:"
cat $OUTPUT_FILE
