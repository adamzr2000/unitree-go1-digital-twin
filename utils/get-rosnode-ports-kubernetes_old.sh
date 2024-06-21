#!/bin/bash

#!/bin/bash

# Define the mapping between ROS nodes and Kubernetes deployments
declare -A NODE_DEPLOYMENT_MAP
NODE_DEPLOYMENT_MAP["/rplidarNode"]="lidar-deployment"
NODE_DEPLOYMENT_MAP["/node_gesture_control"]="gesture-control-app-deployment"
NODE_DEPLOYMENT_MAP["/go1_controller"]="go1-base-deployment"
NODE_DEPLOYMENT_MAP["/OmniIsaacRosBridge"]="digital-twin-app-deployment"
NODE_DEPLOYMENT_MAP["/local_ekf/ekf_localization_with_odom"]="go1-navigation-deployment"
NODE_DEPLOYMENT_MAP["/cartographer_node"]="go1-navigation-deployment"

# Define the list of ROS nodes
NODE_NAMES=(
    "/rplidarNode"
    "/node_gesture_control"
    "/go1_controller"
    "/OmniIsaacRosBridge"
    "/local_ekf/ekf_localization_with_odom"
    "/cartographer_node"
)

OUTPUT_FILE="output.csv"

echo "ros-node,tcp-port,ip-address" > $OUTPUT_FILE

# Get the pod name for roscore-edge-deployment
ROSCORE_POD_NAME=$(kubectl get pods --no-headers -o custom-columns=":metadata.name" | grep "^roscore-edge-deployment-")

if [ -z "$ROSCORE_POD_NAME" ]; then
    echo "No pod found for roscore-edge-deployment"
    exit 1
fi

for NODE_NAME in "${NODE_NAMES[@]}"; do
    DEPLOYMENT=${NODE_DEPLOYMENT_MAP[$NODE_NAME]}
    
    if [ -z "$DEPLOYMENT" ]; then
        echo "No deployment found for node: $NODE_NAME"
        echo "$NODE_NAME,Not found,Not found" >> $OUTPUT_FILE
        continue
    fi
    
    # Get the pod names associated with the deployment
    POD_NAMES=$(kubectl get pods --no-headers -o custom-columns=":metadata.name" | grep "^${DEPLOYMENT}-")
    
    if [ -z "$POD_NAMES" ]; then
        echo "No pod found for deployment: $DEPLOYMENT"
        echo "$NODE_NAME,Not found,Not found" >> $OUTPUT_FILE
    else
        for POD_NAME in $POD_NAMES; do
            # Get the IP address of the pod
            IP_ADDRESS=$(kubectl get pod $POD_NAME -o jsonpath="{.status.podIP}")

            # Run the rosnode info command inside the roscore-edge-deployment pod and store the output
            OUTPUT=$(kubectl exec $ROSCORE_POD_NAME -- sh -c ". /opt/ros/noetic/setup.sh && rosnode info $NODE_NAME")
            
            if [ $? -ne 0 ]; then
                echo "Failed to exec rosnode info for $NODE_NAME in pod $ROSCORE_POD_NAME"
                echo "$NODE_NAME,Not found,$IP_ADDRESS" >> $OUTPUT_FILE
                continue
            fi

            # Extract the port number using grep and awk
            PORT=$(echo "$OUTPUT" | grep "contacting node" | awk -F':' '{print $3}' | awk -F'/' '{print $1}')
            

            if [ -n "$PORT" ]; then
                echo "$NODE_NAME,$PORT,$IP_ADDRESS" >> $OUTPUT_FILE
            else
                echo "$NODE_NAME,Not found,$IP_ADDRESS" >> $OUTPUT_FILE
            fi
        done
    fi
done

echo "Node and port information has been saved to $OUTPUT_FILE"
echo
echo "CSV Output:"
cat $OUTPUT_FILE



# # if [ "$#" -lt 1 ]; then
# #     echo "Usage: $0 <node_name1> [node_name2] ... [-o <output_file>]"
# #     exit 1
# # fi

# # OUTPUT_FILE="output.csv"

# # # Parse optional parameters
# # while [[ "$#" -gt 0 ]]; do
# #     case $1 in
# #         -o|--output)
# #             OUTPUT_FILE="$2"
# #             shift
# #             ;;
# #         *)
# #             NODE_NAMES+=("$1")
# #             ;;
# #     esac
# #     shift
# # done

# # if [ "${#NODE_NAMES[@]}" -eq 0 ]; then
# #     echo "Usage: $0 <node_name1> [node_name2] ... [-o <output_file>]"
# #     exit 1
# # fi

# # DEPLOYMENT_NAME='roscore-edge-deployment'

# # # Get the pod name associated with the deployment
# # POD_NAME=$(kubectl get pods --no-headers -o custom-columns=":metadata.name" | grep "^${DEPLOYMENT_NAME}-")

# # # Check if a pod name was found
# # if [ -z "$POD_NAME" ]; then
# #   echo "No pod found for deployment: $DEPLOYMENT_NAME"
# #   echo "Available deployments:"
# #   kubectl get deploy
# #   exit 1
# # fi

# # # Assuming there's only one container in the pod
# # CONTAINER_NAME=$(kubectl get pod $POD_NAME -o jsonpath="{.spec.containers[0].name}")

# # # Write the CSV header to the output file
# # echo "Node,Port" > $OUTPUT_FILE

# # # Loop through each node name
# # for NODE_NAME in "${NODE_NAMES[@]}"; do
# #     # Run the rosnode info command inside the Kubernetes pod and store the output
# #     OUTPUT=$(kubectl exec -it $POD_NAME -c $CONTAINER_NAME -- sh -c ". /opt/ros/noetic/setup.sh && rosnode info $NODE_NAME")

# #     # Extract the port number using grep and awk
# #     PORT=$(echo "$OUTPUT" | grep "contacting node" | awk -F':' '{print $3}' | awk -F'/' '{print $1}')

# #     if [ -n "$PORT" ]; then
# #         echo "$NODE_NAME,$PORT" >> $OUTPUT_FILE
# #     else
# #         echo "$NODE_NAME,Not found" >> $OUTPUT_FILE
# #     fi
# # done

# # echo "Node and port information has been saved to $OUTPUT_FILE"
# # echo
# # echo "CSV Output:"
# # cat $OUTPUT_FILE
