
#/bin/bash /home/nighthawk/PX4_Related/Firmware/Tools/sitl_run.sh /home/nighthawk/PX4_Related/Firmware/build/px4_sitl_default/bin/px4 none none none /home/nighthawk/PX4_Related/Firmware/ /home/nighthawk/PX4_Related/Firmware/build/px4_sitl_default/



src_path="/home/nighthawk/PX4_Related/Firmware"
model="iris"
build_path="/home/nighthawk/PX4_Related/Firmware/build/px4_sitl_default/"

# Set the plugin path so Gazebo finds our model and sim
source "/home/nighthawk/PX4_Related/Firmware/Tools/setup_gazebo.bash" "/home/nighthawk/PX4_Related/Firmware" "/home/nighthawk/PX4_Related/Firmware/build/px4_sitl_default/"

gzserver --verbose "/home/nighthawk/PX4_Related/Firmware/Tools/sitl_gazebo/worlds/iris.world" & SIM_PID=`echo $!`
# gzserver needs to be running to avoid a race. Since the launch
# is putting it into the background we need to avoid it by backing off
sleep 3
nice -n 20 gzclient --verbose & GUI_PID=`echo $!`




# pushd "$rootfs" >/dev/null

/bin/bash /home/nighthawk/PX4_Related/Firmware/Tools/sitl_multiple_run.sh #/home/nighthawk/PX4_Related/Firmware/build/px4_sitl_default/bin/px4 none none none /home/nighthawk/PX4_Related/Firmware/ /home/nighthawk/PX4_Related/Firmware/build/px4_sitl_default/


# set +e

# export PX4_SIM_MODEL=iris
# /home/nighthawk/PX4_Related/Firmware/build/px4_sitl_default/bin/px4 -s etc/init.d-posix/rcS -t home/nighthawk/PX4_Related/Firmware/test_data /home/nighthawk/PX4_Related/Firmware/ROMFS/px4fmu_common -i 0 -r 1  # & \
# xterm -hold -e /home/nighthawk/PX4_Related/Firmware/build/px4_sitl_default/bin/px4 -i 1 /home/nighthawk/PX4_Related/Firmware/ROMFS/px4fmu_common -s etc/init.d-posix/rcS -t home/nighthawk/PX4_Related/Firmware/test_data

# popd >/dev/null



# if [[ -z "$DONT_RUN" ]]; then
# 	kill -9 $SIM_PID
# 	if [[ ! -n "$HEADLESS" ]]; then
# 		kill -9 $GUI_PID
# 	fi
# fi
