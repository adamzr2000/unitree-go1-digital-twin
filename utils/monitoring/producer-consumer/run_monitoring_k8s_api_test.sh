#!/bin/bash

docker run -it --rm \
    --name "k8s-test" \
    --net host \
    -v "./config:/config" \
    -v "./app:/app/" \
    monitoring-agent:latest \
    kubernetes_api_test.py