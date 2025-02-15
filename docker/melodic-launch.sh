#!/bin/bash
export __NV_PRIME_RENDER_OFFLOAD=1 __GLX_VENDOR_LIBRARY_NAME=nvidia
set -e

#Build the catkin workspace
cd /root/catkin_ws
catkin config --whitelist $BUILDLIST #only builds these packages
catkin build -v

# setup ros environment
if [[ ! -z "${SETUP}" ]]; then
        #from environment variable; should be a absolute path to the appropriate workspaces's setup.bash
        echo "source env is set to '$SETUP'"
else
        # basic ros environment
	export SETUP="/opt/ros/$ROS_DISTRO/setup.bash"
        echo "source env is set to '/opt/ros/$ROS_DISTRO/setup.bash'"
fi

source $SETUP

cd /
#exec /bin/bash
exec ./app_launcher_melodic.sh
#!/bin/bash
# while true; do
#     echo "Service is running..."
#     sleep 1
# done
#roslaunch --wait $ROSPACKAGE $LAUNCHFILE #launch the file
