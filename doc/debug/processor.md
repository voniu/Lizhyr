### Build
```
cd /path/to/Lizhyr
colcon build --packages-select lizhyr
source install/setup.bash
ros2 launch lizhyr processor.launch.py
```

### Play bag
```
ros2 bag play /path/to/os_lidar_data_0.mcap
```

### Listen 8080 port
```
nc -l 8080 > /tmp/lidar.bin
xxd -g 4 -l 16 /tmp/lidar.bin
```