## OpenLane Tools for ROS
### Prerequisites
```angular2html
pip install catkin_pkg && pip install rospkg && pip install gnupg && pip install pycryptodomex
```
### Run
```angular2html
mkdir catkin_ws/src
cd catkin_ws/src
git clone https://github.com/qiaozhijian/openlane_bag.git
cd ..
catkin_make
source devel/setup.bash
cd src/openlane_bag
python openlane_rosbag.py
```