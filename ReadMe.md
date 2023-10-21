bag2files.py: 
 - extracts data from a ROS topic and saves it to a destination directory; both topic names and destination directory are defined in a yaml file;
 - this script was originally made to extract point clouds and poses 


Install Dependencies:
   ros_numpy: pip install git+https://github.com/eric-wieser/ros_numpy
   pip install roslibpy   
   pip install --extra-index-url https://rospypi.github.io/simple/ rospy
   pip install --extra-index-url https://rospypi.github.io/simple/ tf2_ros
   pip install --extra-index-url https://rospypi.github.io/simple/ tf

# TO-DOß
    [] before extraction, verify if the ros message type matches the extraction approach;
    [] from the topic2file.yaml file, use the keys as generic file/dir name, add an additional verification approach to select the right extraction approach 