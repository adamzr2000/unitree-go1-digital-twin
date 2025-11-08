#!/bin/bash

echo "Applying same QoS policies for all users/services..."

curl -F file=@policies/demo_pol_2_perf_isolation_same_pol_ns-slice_1.yml http://localhost:5000/iml/update-policy
curl -F file=@policies/demo_pol_2_perf_isolation_same_pol_ns-slice_2.yml http://localhost:5000/iml/update-policy
