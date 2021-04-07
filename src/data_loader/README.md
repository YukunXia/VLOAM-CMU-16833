# Prerequisites

OpenCV 4.5.1
PCL 1.2

# Usage 1

- Before running, change the `pwd_fix` from catkin_ws path

- Terminal 1: `roslaunch data_loader small_dataset_viz.launch`

- Terminal 2: date = 2011_09_26, sequence number = 1 or 113 or others...
```yaml
rostopic pub /load_small_dataset_action_server/goal data_loader/load_small_datasetActionGoal "header:
  seq: 0
  stamp:
    secs: 0
    nsecs: 0
  frame_id: ''
goal_id:
  stamp:
    secs: 0
    nsecs: 0
  id: ''
goal:
  date: '2011_09_26'
  seq: '113'"
```

# Usage 2

- Terminal 1: `roscore`

- Terminal 2: `rosrun tf static_transform_publisher 0 0 0 0 0 0 map velodyne 50`

- Terminal 3: `rosrun data_loader load_small_dataset`, at the root of the catkin_ws

- Terminal 4: `rviz`, add `/kitti_small/point_cloud`

- Terminal 5: date = 2011_09_26, sequence number = 1 or 113 or others...
```yaml
rostopic pub /load_small_dataset_action_server/goal data_loader/load_small_datasetActionGoal "header:
  seq: 0
  stamp:
    secs: 0
    nsecs: 0
  frame_id: ''
goal_id:
  stamp:
    secs: 0
    nsecs: 0
  id: ''
goal:
  date: '2011_09_26'
  seq: '113'"
```