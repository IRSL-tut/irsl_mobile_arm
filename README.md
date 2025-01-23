# Simple move

Simple mobile base and arm sample

## 実行方法

```bash
sudo mkdir -p /irsl_ws/src
sudo chown -R $USER:$USER /irsl_ws
cd /irsl_ws/src
git clone https://github.com/IRSL-tut/irsl_mobile_arm
cd /irsl_ws
rosdep update
rosdep install -r --ignore-src --from-path src
catkin build
source /irsl_ws/devel/setup.bash
```

```bash
roslaunch irsl_mobile_arm run_base.launch run_gazebo:=true ## gazeboのlaunch
rosrun rviz rviz -d simple_move.rviz ##
```

- いい感じのところに 2D pose estimate (rviz上)
- 適度なところへ goal を設定 (rviz上)

### ダミーのオドメトリの出力 / Dummy odometry

twistを積算するダミーのオドメトリを使用する

```bash
roslaunch irsl_mobile_arm run_base.launch run_gazebo:=true use_dummy_odom:=true ## gazeboのlaunch, ダミーのオドメトリ
```

### Making map

マップを作る

```bash
roslaunch irsl_mobile_arm run_base.launch run_gazebo:=true making_map:=true ## gazeboのlaunch, マップを作る
```

cmd_velを出力するもの(twist_joyやtwist_keyboard)でロボットを動かす。

マップの保存

```bash
rosrun map_server map_saver -f map
```

### Real Robot

マップを作る

- ターミナル1
    ```bash
    roslaunch irsl_mobile_arm urg_node.launch
    ```
- ターミナル2
    ```bash
    roslaunch irsl_mobile_arm run_base.launch run_gazebo:=false making_map:=true use_dummy_odom:=true
    ```

### URDF

URDF ファイルは修正の必要ないとは思うが

xacro base.urdf.xacro > simple_move.urdf ## aero_ros_pkg 直下で

libgazebo_force_move の所を修正

## 実行方法 ( Windows )

```bash
git clone https://github.com/IRSL-tut/irsl_docker_irsl_system
```

## OLD information

export ROS_PACKAGE_PATH=$(pwd)/..:$ROS_PACKAGE_PATH

### インストールが必要なもの

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

## OLD information2

このディレクトリを /tmp/simple_move に置く (シンボリックリンクでもよい)
