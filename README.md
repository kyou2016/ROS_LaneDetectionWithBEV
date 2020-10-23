## ROS Lane Detection and Darknet_ros Object Detection

1. Object Detection
- Recieve Video from Vehicle Camera(using usb_cam pkg)
- using Darknet_ros, use networks yoloV3-tiny, for Bus Stop Detection
- is Detected, Publish 'Speed 0' Command Wait 10sec Vehicle is Drive with lane detection

2. Lane Detection
- Recieve Video from usb_cam pkg
- Video Convert Bird Eyes View Image and Binary
- using Sliding Windows and Calculate Vehicle's Steer and Publish Steer command
- Speed is fixed 1000
- subscribe message like 'Bus Stop Detect!' -> Speed is 0 and Wait 10sec
- Over 10sec Drive using lane detection


#1 Command

1. run darknet_ros and usb_cam
		roslaunch darknet_ros darknet_ros2.launch
		
2. run lane detection
		rosrun lane_detection_example hanho_lane_detector.py

3. check binary image is right?
4. binary image is not right -> image shape is not right?
- move 'control penal' top x,y bottom x,y point
- this point is BEV matrix point. this change -> BEV image change

5. binary image is not right -> image binarization is not right?
- in 'hanho_lane_detector.py' image_process_otsu function code change
- i think change cv2.inRange(src, lower, upper)

6. all of image is right
- run vesc_driver_node
- this is vesc_node. if it is not running. vehicle don't Drive Anyway
		roslaunch vesc_driver vesc_driver_node.launch
