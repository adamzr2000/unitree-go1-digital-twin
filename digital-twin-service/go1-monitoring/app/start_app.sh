#!/bin/bash

influxdb_url=${INFLUXDB_URL:-"http://192.168.40.4:8086"}
influxdb_token=${INFLUXDB_TOKEN:-"desire6g2024;"}
influxdb_org=${INFLUXDB_ORG:-"desire6g"}
influxdb_bucket=${INFLUXDB_BUCKET:-"ros-metrics"}
window_size=${WINDOW_SIZE:-"50"}
topics=${TOPICS:- }

echo "influxdb_url: $influxdb_url"
echo "influxdb_token: $influxdb_token"
echo "influxdb_org: $influxdb_org"
echo "influxdb_bucket: $influxdb_bucket"
echo "window_size: $window_size"
echo "topics: $topics"

# Convert the topics string into an array
IFS=' ' read -r -a topics_array <<< "$topics"

# Use "${topics_array[@]}" to expand the topics into separate arguments
python3 client.py "${topics_array[@]}" --influxdb_url $influxdb_url --influxdb_token $influxdb_token --influxdb_org $influxdb_org --influxdb_bucket $influxdb_bucket --window_size $window_size
