#!/bin/bash

echo "Applying QoS policy to low-latency network slice 1 (robots)..."

curl -F file=@policies/demo_pol_3_perf_isolation_different_ns-slice_1-robot.yml http://localhost:5000/iml/update-policy
curl -F file=@policies/demo_pol_3_perf_isolation_different_ns-slice_1-bg.yml http://localhost:5000/iml/update-policy
curl -F file=@policies/demo_pol_3NEW_perf_isolation_same_pol_ns-slice_2.yml http://localhost:5000/iml/update-policy
