# lars-ros
Repository for our ROS packages
# example of generating a .csv from a .bag
rostopic echo -b $1 -p /torque > pwm.csv
