# BirlModuleRobot

BIRL 攀爬机器人控制软件

**1. 安装通信驱动及相关依赖**
```
wget http://www.ixxat.com/docs/librariesprovider8/default-document-library/downloads/other-drivers/socketcan_1-1-92-0_20150508.zip?sfvrsn=10 
unzip socketcan_1-1-92-0_20150508.zip?sfvrsn=10
gedit README &
cd usb-to-can_v2_socketcan
sudo modprobe can-dev
sudo apt-get install module-assistant
sudo module-assistant prepare
make
sudo make install
```

```
sudo touch /etc/can.conf
sudo chmod a+w /etc/can.conf

echo "[default]  
interface = socketcan  
channel = can0" >> /etc/can.conf
```

```
sudo apt-get install can-utils
sudo pip install canopen
```

**2. ROS安装**

  参考网址：


   <http://wiki.ros.org/kinetic/Installation/Ubuntu>

**3. 软件使用**

  *3.1 软件下载与环境配置*
```
mkdir -r ros/modular_robot/src/ && cd ~/ros/modular_robot/src/ 
git clone https://github.com/Jiongyu/BirlModuleRobot.git
sudo echo "source ~/ros/modular_robot/devel/setup.bash">> ~/.bashrc
source ~/.bashrc
```

  *3.2 连接通信启动软件*
```
rosrun canopen_communication can_prepare.sh
roslaunch ui ui_start.launch
```

