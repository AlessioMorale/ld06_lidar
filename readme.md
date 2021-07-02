## 编译方法

使用catkin编译，执行如下操作

```sh
catkin_make

```



## 运行方法

```sh
source devel/setup.bash
roslaunch ldlidar LD06.launch 

or

rosrun ldlidar ldlidar 
```

rviz的配置在rviz文件夹下面。



## 测试

代码在ubuntun16.04 kinetic版本下测试，使用rviz可视化。
