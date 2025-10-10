# How to use it?

```bash
./run_example.sh \
  --ros-master-uri http://127.0.0.1:11311 \
  --ros-ip 127.0.0.1 \
  --topics "/scan,/joint_states,/odometry/filtered,/go1_controller/cmd_vel" \
  --influxdb-url http://127.0.0.1:8087 \
  --window-size 50 \
  --wait-for-master true \
  --node-role edge
```