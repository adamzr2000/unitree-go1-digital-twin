#!/bin/bash

influxdb_url=${INFLUXDB_URL:-"http://192.168.40.4:8086"}
influxdb_token=${INFLUXDB_TOKEN:-"desire6g2024;"}
influxdb_org=${INFLUXDB_ORG:-"desire6g"}
influxdb_bucket=${INFLUXDB_BUCKET:-"ros-metrics"}
window_size=${WINDOW_SIZE:-"50"}
topics=${TOPICS:- }
manual_delay=${MANUAL_DELAY:-"false"}

echo "influxdb_url: $influxdb_url"
echo "influxdb_token: $influxdb_token"
echo "influxdb_org: $influxdb_org"
echo "influxdb_bucket: $influxdb_bucket"
echo "window_size: $window_size"
echo "topics: $topics"
echo "manual_delay: $manual_delay"

# Convert the topics string into an array
IFS=' ' read -r -a topics_array <<< "$topics"

# Construct the command to run the Python script
cmd=(python3 client.py "${topics_array[@]}" --influxdb_url "$influxdb_url" --influxdb_token "$influxdb_token" --influxdb_org "$influxdb_org" --influxdb_bucket "$influxdb_bucket" --window_size "$window_size")

# Add the --manual_delay flag if manual_delay is set to "true"
if [ "$manual_delay" == "true" ]; then
    cmd+=("--manual_delay")
fi

# Run the constructed command
echo "Running command: ${cmd[@]}"
"${cmd[@]}"


# Use "${topics_array[@]}" to expand the topics into separate arguments
# python3 client.py "${topics_array[@]}" --influxdb_url $influxdb_url --influxdb_token $influxdb_token --influxdb_org $influxdb_org --influxdb_bucket $influxdb_bucket --window_size $window_size
