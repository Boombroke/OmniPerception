# usb相机帧率低

在启动无驱usb相机后，发现帧率极低，只有5-15 fps
将video0设置分辨率1920x1080 @ 30 fps 格式：MJPG
v4l2-ctl -d /dev/video0 --set-fmt-video=width=1920,height=1080,pixelformat=MJPG
v4l2-ctl -d /dev/video0 --set-parm=30

# 编译

colcon build --symlink-install 

# 运行多相机

ros2 launch usb_camera multi_camera.launch.py

# 运行单相机

ros2 launch usb_camera oneself_camera.launch.py