rostopic pub -1 /traj_start_trigger geometry_msgs/PoseStamped "
header:
  seq: 1
  stamp: 1
  frame_id: "world"
pose:
  position:
    x: 1.0
    y: 1.0
    z: 1.0
  orientation:
    x: 1.0
    y: 1.0
    z: 1.0
    w: 1.0

"
