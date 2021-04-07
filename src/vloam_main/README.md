# Prerequisites

OpenCV 4.5.1
PCL 1.2

# Usage 1

- Step 1, download bag files under "/bags"

- Step 2, `catkin_make`, and then `rosrun vloam_main vloam_main` (with roscore started)

- Step 3, check the sequence number and date for the bag file, and in another terminal, run
```
rostopic pub /load_small_dataset_action_server/goal vloam_main/vloam_mainActionGoal "header:
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
  seq: '1'"
```