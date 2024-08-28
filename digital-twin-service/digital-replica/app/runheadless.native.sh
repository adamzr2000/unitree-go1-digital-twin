#! /bin/sh

# Define the path to the .kit file
KIT_FILE="/isaac-sim/apps/omni.isaac.sim.headless.native.kit"

# Check if the ROS1 bridge extension is already enabled in the .kit file
if ! grep -q 'isaac.startup.ros_bridge_extension = "omni.isaac.ros_bridge"' "$KIT_FILE"; then
  echo "Enabling ROS1 bridge extension in $KIT_FILE..."
  
  # Backup the original .kit file before modification
  cp "$KIT_FILE" "${KIT_FILE}.bak"
  
  # Add the ROS1 bridge extension to the .kit file
  echo '' >> "$KIT_FILE"
  echo '# Set the default ROS bridge to enable on startup' >> "$KIT_FILE"
  echo '[settings."filter:platform"."linux-x86_64"]' >> "$KIT_FILE"
  echo 'isaac.startup.ros_bridge_extension = "omni.isaac.ros_bridge"' >> "$KIT_FILE"
else
  echo "ROS1 bridge extension is already enabled."
fi

# Run the usual startup sequence
/isaac-sim/license.sh && /isaac-sim/privacy.sh && /isaac-sim/isaac-sim.headless.native.sh \
  --/persistent/isaac/asset_root/default="$OMNI_SERVER" \
  --merge-config="/isaac-sim/config/open_endpoint.toml" --allow-root "$@" 
