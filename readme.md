# OVERVIEW
  该项目是robomaster哨兵机器人全向感知的简易版，使用三个USB相机和rm_vision识别器魔改，将识别到的装甲板数字发送给下位机。可搭配rv食用，串口部分自行在rm_serial_driver中添加即可。
目前使用三个相机，对于相机调用需要在config中配置，添加udev规则确保相机位置与真实位置对应。

## 包含项目

usb相机模块 https://github.com/Boombroke/OmniPerception/tree/main/usb_camera

全向感知模块 https://github.com/Boombroke/OmniPerception/tree/main/rm_omni_perception

自定义话题消息 https://github.com/Boombroke/OmniPerception/tree/main/auto_aim_interfaces

## 源码编译

```
colcon build --symlink-install
```

## 调试及运行

单USB相机图像发布
```
ros2 launch usb_camera oneself_camera.launch.py
```

多USB相机图像发布
```
ros2 launch usb_camera three_cameras.launch.py
```

全向感知+USB相机节点
```
ros2 launch omni_perception_bringup rm_omni_perception.launch.py 
```

## 后续优化方向

加入PNP结算及TF，而并非直接发送数字。
加入识别优先级。
将识别器改为神经网络。

该版本是花了几个小时用现有轮子搓的，目前鲁棒性不高，后续会慢慢优化