rosrun baxter_tools camera_control.py -c left_hand_camera
rosrun baxter_tools camera_control.py -o head_camera
rosrun image_transport republish raw in:=/cameras/head_camera/image  out:=/robot/xdisplay
