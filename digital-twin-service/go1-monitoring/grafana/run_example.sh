#!/bin/bash

docker run -d \
    --net host \
    --name grafana \
    --rm \
    -v "$PWD/config/provisioning/datasources:/etc/grafana/provisioning/datasources" \
    -v "$PWD/config/provisioning/dashboards:/etc/grafana/provisioning/dashboards" \
    -e GF_SECURITY_ADMIN_USER="desire6g" \
    -e GF_SECURITY_ADMIN_PASSWORD="desire6g2024;" \
    grafana/grafana:latest
