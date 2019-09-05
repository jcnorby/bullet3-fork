# bullet3-fork

see bullet3-fork/bullet3/examples/pybullet/examples/minitaur_ros_FD.py and 
bullet3-fork/bullet3/examples/pybullet/examples/minitaur_evaluate_steady_state.py for implementation

Run in command line to execute finite diff:
python minitaur_ros_FD.py

To make compatible with ros, change the main() function of minitaur_ros_FD.py to subscribe to get nominal x and u values then publish the results of A,B = AB_Partial_FD_test( x , u)
