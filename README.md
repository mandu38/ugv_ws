# GAZEBO_UGV

## Build

```GAZEBO bash
cd ~/ugv_ws
source /opt/ros/humble/setup.bash
colcon build --symlink-install
source install/setup.bash
```

---

## 1. Gazebo Launch

```bash
source /opt/ros/humble/setup.bash
source ~/ugv_ws/install/setup.bash
source /usr/share/gazebo/setup.sh

export GAZEBO_MODEL_PATH=$HOME/.gazebo/models:$GAZEBO_MODEL_PATH
export UGV_MODEL=ugv_rover

ros2 launch ugv_gazebo bringup.launch.py
```

---

## 2. Mapping (Gmapping)

```bash
source /opt/ros/humble/setup.bash
source ~/ugv_ws/install/setup.bash

ros2 launch ugv_gazebo gmapping.launch.py
```

---

## 3. Keyboard Control

```bash
source /opt/ros/humble/setup.bash
source ~/ugv_ws/install/setup.bash

ros2 run ugv_tools keyboard_ctrl
```

---

## 4. Navigation (Nav2)

```bash
source /opt/ros/humble/setup.bash
source ~/ugv_ws/install/setup.bash
source /usr/share/gazebo/setup.sh

export GAZEBO_MODEL_PATH=$HOME/.gazebo/models:$GAZEBO_MODEL_PATH
export UGV_MODEL=ugv_rover

ros2 launch ugv_gazebo nav.launch.py use_localization:=amcl
```

---

## 5. Patrol Node

```bash
cd ~/ugv_ws
source /opt/ros/humble/setup.bash
colcon build --symlink-install
source install/setup.bash

source /opt/ros/humble/setup.bash
source ~/ugv_ws/install/setup.bash

ros2 launch my_waver_app patrol.launch.py
```

---

## Execution Order

1. Gazebo Launch  
2. Mapping  
3. Keyboard Control (map generation)  
4. Navigation (AMCL)  
5. Patrol Node  

