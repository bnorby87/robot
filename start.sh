cd ~
screen -m -d -S low bash -c 'ros2 launch fiekrobotkocsi_bringup fiekrobotkocsi_low_level.launch.py'
echo "[INFO] low level launch started!"
sleep 5
screen -m -d -S bringup bash -c 'ros2 launch fiekrobotkocsi_bringup fiekrobotkocsi_bringup.launch.py'
echo "[INFO] bringup launch started!"
sleep 5
screen -m -d -S mavproxy bash -c 'mavproxy.py --master=/dev/ttyPX4,115200 --out=udp:10.11.0.23:14550'
echo "[INFO] MAVProxy started!"
sleep 1
screen -m -d -S rtk bash -c 'python3 motion/rtk2.py'
echo "[INFO] RTK started!"
sleep 1
screen -m -d -S vnav bash -c 'python3 motion/vnav.py'
echo "[INFO] Vectornav started!"
