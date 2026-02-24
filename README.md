ROS2 port

```bash
colcon build --packages-select orbslam3_ros2 \
  --cmake-args -G Ninja -DCMAKE_BUILD_TYPE=Release
```

## Third-party sources

- `third_party/ORB_SLAM3` is vendored from [`UZ-SLAMLab/ORB_SLAM3`](https://github.com/UZ-SLAMLab/ORB_SLAM3).
- We apply small local compatibility/build fixes on top of upstream.
