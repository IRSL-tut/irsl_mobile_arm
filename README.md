# Simple move

Simple move_base sample

## 実行方法

このディレクトリを /tmp/simple_move に置く (シンボリックリンクでもよい)

```bash
cd /tmp/simple_move
roslaunch run_gazebo.launch ## gazeboのlaunch
rosrun rviz rviz -d simple_move.rviz ##
```

- いい感じのところに 2D pose estimate (rviz上)
- 適度なところへ goal を設定 (rviz上)

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
ros-noetic-global-planner

```

### URDF

URDF ファイルは修正の必要ないとは思うが

xacro base.urdf.xacro > simple_move.urdf ## aero_ros_pkg 直下で

libgazebo_force_move の所を修正
