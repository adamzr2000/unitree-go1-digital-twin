#!/bin/bash

docker run -d \
    --name influxdb2 \
    -p 8087:8086 \
    --rm \
    -e DOCKER_INFLUXDB_INIT_MODE="setup" \
    -e DOCKER_INFLUXDB_INIT_USERNAME="desire6g" \
    -e DOCKER_INFLUXDB_INIT_PASSWORD="desire6g2024;" \
    -e DOCKER_INFLUXDB_INIT_ORG="desire6g" \
    -e DOCKER_INFLUXDB_INIT_BUCKET="infrastructure-monitoring" \
    -e DOCKER_INFLUXDB_INIT_RETENTION=1w \
    -e DOCKER_INFLUXDB_INIT_ADMIN_TOKEN="desire6g2024;" \
    influxdb:2
