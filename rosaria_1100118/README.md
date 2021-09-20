## bring up

sudo chmod 777 -R /dev/ttyUSB0
sudo chmod 777 -R /dev/ttyUSB1

roslaunch rosaria remap_aria.launch

roslaunch robot_qmzt RSWithArUco.launch

launch-prefix="xterm -e"


 

For information on how to use RosAria, see <http://wiki.ros.org/ROSARIA>,
especially <http://wiki.ros.org/ROSARIA/Tutorials/How to use ROSARIA>.

