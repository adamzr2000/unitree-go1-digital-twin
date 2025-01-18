#!/bin/bash

# This script modifies resource usage policies in the robot network service.
# It ensures higher guaranteed bandwidth for the robot user and reduces the resource usage of the background (bg) flow.

echo "Applying higher guaranteed bandwidth policy to the critical robot traffic in network slice 1..."
curl -F file=@policies/demo_pol_3_perf_isolation_different_ns-slice_1-robot.yml http://localhost:5000/iml/update-policy

echo "Applying reduced resource usage policy to the non-important robot traffic in network slice 1..."
curl -F file=@policies/demo_pol_3_perf_isolation_different_ns-slice_1-bg.yml http://localhost:5000/iml/update-policy

echo "Applying reduced resource usage policy to the simulated traffic in network slice 2..."
curl -F file=@policies/demo_pol_3_perf_isolation_different_ns-slice_2.yml http://localhost:5000/iml/update-policy
