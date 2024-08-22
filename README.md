# Simple move

Simple move_base sample

## 実行方法

export ROS_PACKAGE_PATH=$(pwd)/..:$ROS_PACKAGE_PATH

```bash
roslaunch simple_move run_base.launch run_gazebo:=true ## gazeboのlaunch
rosrun rviz rviz -d simple_move.rviz ##
```

- いい感じのところに 2D pose estimate (rviz上)
- 適度なところへ goal を設定 (rviz上)

### ダミーのオドメトリの出力 / Dummy odometry

twistを積算するダミーのオドメトリを使用する

```
roslaunch simple_move run_base.launch run_gazebo:=true use_dummy_odom:=true ## gazeboのlaunch, ダミーのオドメトリ
```

### Making map

マップを作る

```
roslaunch simple_move run_base.launch run_gazebo:=true making_map:=true ## gazeboのlaunch, マップを作る
```

cmd_velを出力するもの(twist_joyやtwist_keyboard)でロボットを動かす。

マップの保存
```
rosrun map_server map_saver -f map
```

### Real Robot

マップを作る

- ターミナル1
    ```
    roslaunch simple_mode urg_node.launch
    ```
- ターミナル2
    ```
    roslaunch simple_move run_base.launch run_gazebo:=false making_map:=true use_dummy_odom:=true
    ```

## インストールが必要なもの

他も必要かもしれない

``` bash
sudo apt install \
ros-noetic-gazebo-plugins \
ros-noetic-move-base \
ros-noetic-amcl \
ros-noetic-map-server \
ros-noetic-teb-local-planner \
ros-noetic-dwa-local-planner \
ros-noetic-global-planner  \
ros-noetic-slam-gmapping  \
ros-noetic-teleop-twist-keyboard

```

### URDF

URDF ファイルは修正の必要ないとは思うが

xacro base.urdf.xacro > simple_move.urdf ## aero_ros_pkg 直下で

libgazebo_force_move の所を修正


## OLD information

このディレクトリを /tmp/simple_move に置く (シンボリックリンクでもよい)
