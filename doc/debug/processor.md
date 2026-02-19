### Build
```bash
cd /path/to/Lizhyr
colcon build
source install/setup.bash
```

### Computer A (Processor - LiDAR processing and TCP streaming)
```bash
# Edit config if needed: lizhyr_processor/config/processor.yaml
# Set: lidar_id, server_ip, server_port
ros2 launch lizhyr_processor processor.launch.py
```

In another terminal, play bag:
```bash
source install/setup.bash
ros2 bag play /path/to/os_lidar_data_0.mcap --loop
```

### Computer B (Restorer - TCP server and point cloud restoration)
```bash
# Edit config if needed: lizhyr_restorer/config/restorer.yaml
# Set: server_port, target_frame_id
ros2 launch lizhyr_restorer restorer.launch.py
```

Verify restored clouds:
```bash
ros2 topic list  # Should see /Lizhyr/points_1, /Lizhyr/points_2, etc.
ros2 topic echo /Lizhyr/points_1 --once
```

### Debug: Monitor raw TCP stream
```bash
nc -l 8080 > /tmp/lidar.bin
xxd -g 4 -l 16 /tmp/lidar.bin
# First 4 uint32: magic(LDAR), payload_size, lidar_id, sequence
```