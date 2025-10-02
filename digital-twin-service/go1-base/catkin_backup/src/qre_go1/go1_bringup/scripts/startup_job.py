#!/usr/bin/env python3

import os
import robot_upstart
import rospkg

job_name = "go1_ros"

os.system("sudo service {} stop".format(job_name))

uninstall_job = robot_upstart.Job(name=job_name, rosdistro=os.environ['ROS_DISTRO'])
uninstall_job.uninstall()

main_job = robot_upstart.Job(name=job_name, 
                             user='root',
                             master_uri=os.environ['ROS_MASTER_URI'],
                             rosdistro=os.environ['ROS_DISTRO'],
                             workspace_setup=os.path.join(rospkg.RosPack().get_path('go1_bringup'), 'config', 'setup.bash'))
main_job.add(package="go1_bringup", filename="launch/bringup.launch")
main_job.install()

os.system("sudo systemctl daemon-reload && sudo systemctl start {}".format(job_name))
