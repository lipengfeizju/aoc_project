1. roslaunch flea3 single_node.launch device:=14051393
2. roslaunch ffld_ros object_detector_node.launch
3. rqt_image_view (/object_detector/image)

In Progress: test_ffld_ros.launch creates detection service and client
