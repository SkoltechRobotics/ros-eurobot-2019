# CODE GUIDELINES
---------------------
1) Code should look like a poem and be self-explanatory
2) Use functions
3) First list yaml-params, than publishers, and in the end - Subcribers
4) Timer in the end, it should call function motion_timer_callback, from which desiable methods are chosen
5) No magic constants in code
6) No pre-optimization, and more post-opt
7) Write specs --> write tests --> write code --> release
8) For subscriber callback the last parameter is data from topic, for timer callback - event
9) не надо делать абсолютные пути '/secondary_robot/stm_command', лучше в launch файле сделать remap
10) 


# General info
- Subscribers to the end of init
- no namespaces in init, all in launch


# ROBOT COMMUNICATION
------------------
rostopic pub -1 /move_command std_msgs/String "data: 'abc arc_move 0.3 1.2 0'" 
rostopic pub -1 /secondary_robot/move_command std_msgs/String "data 'abc arc_move 1 1 3.14'"

8 - vels
14 - pos
rostopic pub -1 /secondary_robot/stm_command std_msgs/String "data: 'null 8 0 0 0'" 

## to ride by odometry
rostopic pub -1 /main_robot/stm/command std_msgs/String "data: '1 8 0.2 0 0'" 

rostopic pub -1 /secondary_robot/stm_command std_msgs/String "data: '1 14 0 0 0'" 

rostopic pub -1 /main_robot/score std_msgs/String "data: '24'" 


roslaunch eurobot_tactics tactics_sim_launch.launch


ssh odroid@192.168.88.239

roscd eurobot_nav

на роботе
roslaunch eurobot_loc pf_launch.launch

на компе
roslaunch eurobot_loc remote.launch
roslaunch eurobot_loc visualization.launch

python arc_move_v01.py 

rostopic echo /secondary_robot/stm_command 
rostopic list 
rosnode list

"""
roslaunch eurobot multimaster.launch 
autostart

autostart ??? command
crontab -e
"""

- To turn off odroid from ssh:
sudo shutdown -h now

rosed 

- To check coords and delay
rosrun tf tf_echo map secondary_robot



rostopic pub -1 /secondary_robot/move_command std_msgs/String "data: 'abc move_arc 1.5 0.45 3.14'"
rostopic pub -1 /move_command std_msgs/String "data: 'abc move_arc 0.61 0.45 3.14'"
rostopic pub -1 /move_command std_msgs/String "data: 'abc move_arc 0.5 0.34 1.57'"
rostopic pub -1 /move_command std_msgs/String "data: 'abc move_arc 0.61 1.05 3.14'"

rostopic pub -1 /secondary_robot/move_command std_msgs/String "data: 'abc move_line 0.75 0.5 0'"
rostopic pub -1 /main_robot/move_command std_msgs/String "data: 'abc move_line 0.4 0.4 0'"
rostopic pub -1 /secondary_robot/cmd_tactics std_msgs/String "data: 'abc collect_chaos'"
rosrun eurobot_tactics imitate_cam.py -n 4

rosrun rqt_graph rqt_graph

to puck near red field SMALL
rostopic pub -1 /secondary_robot/move_command std_msgs/String "data: 'abc move_line 0.6 0.45 3.14'"
rostopic pub -1 /secondary_robot/stm_command std_msgs/String "data: '1 0x30'"
rostopic pub -1 /secondary_robot/stm_command std_msgs/String "data: '1 0x11'"

roslaunch eurobot_stm STM_node.launch

rostopic pub -1 /manipulator/command std_msgs/String "data: 'abc release_accelerator'"

WE CHANGED TOPIC
stm 1 0x11

export DISPLAY=:0
roslaunch eurobot_main main_robot_bt.launch 



# WIRELESS PART SETUP
---------------
ifconfig

in /etc/network$ vim interfaces

# interfaces(5) file used by ifup(8) and ifdown(8)
auto lo
iface lo inet loopback
auto wlan0
auto eth0
#allow-hotplug eth0
iface eth0 inet static
        address 192.168.0.15
        netmask 255.255.255.0



ssh keygen
enter
enter enter
copy id
adress



# GIT
---------------
http://rogerdudler.github.io/git-guide/
http://365git.tumblr.com/post/504140728/fast-forward-merge
http://rogerdudler.github.io/git-guide/

git push --set-upstream upstream test_nav_loc

git config --global user.email nikolay.zherdev@gmail.com
git config --global user.name "nickzherdev"

# create new repository
cd eurobot2019_ws/
git init
git status
git checkout branchname  # Change working branch
git checkout -b your-branch # Create the branch on your local machine and switch in this branch
git push -u upstream collect_chaos_pucks - in order for branch to watch origin

git add filename
git commit -m “Add chocolate.jpeg.”
git push origin master (or git push upstream bt_chaos)

git remote add origin (upstream) 
git remote add upstream https://github.com/SkoltechRobotics/ros-eurobot-2019.git
git remote add [name_of_your_remote] [name_of_your_new_branch]
git commit

git remote -v
git push origin [name_of_your_new_branch] 

git push upstream navnew
git push upstream grab_atoms 

git merge branchname (ex: loc)

git push upstream navnew --force
git push --set-upstream upstream test_nav_loc 

В том случае, если ветка master (или branch_name) не является отслеживаемой веткой origin/master (или origin/branch_name), а вы хотите сделать её таковой.

Выполнив команду git push -u origin master вы устанавливаете связь между той веткой, в которой вы находитесь и веткой master на удалённом сервере. Команду требуется выполнить единожды, чтобы потом можно было отправлять/принимать изменения лишь выполняя git push из ветки без указания всяких алиасов для сервера и удалённых веток. Это сделано для удобства.

на роботе
git clone https://github.com/SkoltechRobotics/ros-eurobot-2019.git

git config --global http.sslverify false

in case of problems with sertificates use this command
git config --global http.sslverify false

copy file or folder from other branch
git checkout branchname path/to/file/file.py
git checkout collect_chaos_pucks eurobot_tactics/notebooks


To revert the previous commit (our merge commit), we do:
git revert HEAD

- to show last commits
git reflog

- to remove latest pull
git reset --hard a0d3fe6, // where  a0d3fe6 - is a head id

## Team




# FILESYSTEM
--------------
transfer a file using sftp in UNIX

cd catkin_ws/src/ros-eurobot-2018/

/home/safoex/eurobot2019_ws/src/ros-eurobot-2019/eurobot_nav

~/eurobot2019_ws/src

rm -rf ros-eurobot-2019/ # recursevly delete folder and it's subfolders and files

chmod +x "filename"

sudo pip install --target=/usr/local/lib/python2.7/dist-packages sympy
sudo pip install --target=/opt/ros/kinetic/lib/python2.7/dist-packages sympy

on odroid to show 
ls -l /dev/tty*




# make backup
- to show all connected devices use lsblk in terminal

- completely empty writable emmc through disk util
- to create IMAGE of all partitions
apt-get install pv
sudo dd if=/dev/sdf bs=1M | pv | dd of=/media/safoex/01D251799AF7F600/backup_eurobot_2101/backup.img
https://forum.odroid.com/viewtopic.php?f=52&t=22930

- to restore IMAGE on new emmc / sd_card
sudo dd if=/media/safoex/01D251799AF7F600/backup_eurobot_2101/backup.img of=/dev/sdf bs=1M


Error mounting /dev/sdf1 at /media/safoex/EurobotMain: Command-line `mount -t "ext4" -o "uhelper=udisks2,nodev,nosuid" "/dev/sdf1" "/media/safoex/EurobotMain"' exited with non-zero exit status 32: mount: wrong fs type, bad option, bad superblock on /dev/sdf1,
       missing codepage or helper program, or other error

       In some cases useful info is found in syslog - try
       dmesg | tail or so.alsfasfafkasfsafmsdklgnsdklmsdklmsklvmsdklvmklsa


gedit ~/.bashrc 

sudo vim /etc/hosts

## remove nesessity to enter password 
sudo update-alternatives --config editor
sudo visudo
myuser ALL=(ALL) NOPASSWD:ALL



# ROS WORKSPACE:
--------------

catkin workspace
http://wiki.ros.org/catkin/workspaces

Creating a ROS Package
http://wiki.ros.org/ROS/Tutorials/CreatingPackage
http://wiki.ros.org/ROS/Tutorials/catkin/CreatingPackage
http://wiki.ros.org/ROS/NetworkSetup


# create new node
catkin_create_pkg eurobot_display std_msgs rospy roscpp

# navigate to workspace/src to create a new package eurobot_core and eurobot_nav
cd ~/catkin_ws/src
mkdir ros-eurobot-2019
cd ros-eurobot-2019

mkdir scripts

vim /etc/hosts 
export PYTHONPATH="${PYTHONPATH}:/home/safoex/eurobot2019_ws/src/ros-eurobot-2019/libs"
export PATH="${PATH}:/home/safoex/eurobot2019_ws/src/ros-eurobot-2019/scripts"

export PYTHONPATH="${PYTHONPATH}:/home/odroid/catkin_ws/src/ros-eurobot-2019/libs"
export PATH="${PATH}:/home/odroid/catkin_ws/src/ros-eurobot-2019/scripts"
/home/odroid/catkin_ws/src/ros-eurobot-2019/libs


# Create new workspace
http://wiki.ros.org/catkin/Tutorials/create_a_workspace

mkdir -p ~/eurobot2019_ws/src
cd eurobot2019_ws/
catkin_make
source devel/setup.bash
vim ~/.bashrc

source ~/.bashrc 
rosrun eurobot_tactics TacticsNode.py
gedit ~/.bashrc 

In the header of each file we need to state which env to use:
#!/usr/bin/env python



# Usefull shortcuts
Ctrl Alt 9 / 3 / 6 / 7 for sticking windows to different screen corners



TROOBLESHOOTING
------------------

Localization not working
- check if LIDAR is turned on (starts making noise) and connected to Ethernet port in Odroid
- check in RViz if robot is coreectly localized 






GOALS
------------------
Цель - подобрать роботом шайбу и овтезти её на акселлератор

Задачи

Получить координаты шайбы по камере (Алексей)
Подъехать к ней по дуге (Николай) в правильное место с учётом зазора для манипулятора
Произвести операцию схватывания
Отвезти в акселлератор
Сбросить
Вернуться на место
Вынести все параметры в yaml файл


# Team
Александр Соколовский - XL (54)
Семен Лыхин - M
Батыржан Алиханов - L
Никита Горбадей - L
Хуан Эредиа - L
Эдгар Казиахмедов - L
Антон Егоров - L
Егор Пристанский - XXL
Алексей Кашапов - L
Николай Жердев - L

Sergei Vostrikov - L
Taras Melnik - L
Andrew Chemikhin - M
Evgeny Safronov - M
Vladimir Karandaev - XL
Mikhail Kurenkov - M
Dzmitry Tsetserukou - XXL
Alexandr Kuleshov - XXL

Optional:
Evgeniy - L
poc - L
S - 1

Поло и Худи
S - 1
M - 4
L - 11 (9 + 2)
XL - 2
XXL - 3

Поло + 2 спонсорам


# BT
- Action sets params - SET
- Condition node checks their status if callback activated - GET

- Three main parts: 
-- Dictionary
-- Callbacks
-- Nodes

# Проверить, что робот остановился.
# Информация о том, что робот отсановился, публикуется в топик response
# Проверить, что цель достигнута.

# Alexey
# Проверить, что манипулятор в исходной позиции, Опустить манипулятор, Проверить, что манипулятор опустился
# Включить насос (на сколько?)
# Поднять манипулятор до конца
# Подпереть атом граблей
# Выключить насос


# NO RVIZ IN LAUNCH FILE

#! /bin/bash

source /opt/ros/kinetic/setup.bash
source ~/catkin_ws/devel/setup.bash

roslaunch eurobot main_robot.launch 



To run this on boot you can create a simple systemd service. 
Create mavros.service file in /lib/systemd/system with the following contents:

[Unit]
Description=eurobot 

[Service]
Type=forking
ExecStart=/bin/bash -c "source /opt/ros/kinetic/setup.bash; /usr/bin/python /opt/ros/kinetic/bin/roslaunch mavros apm.launch"
Restart=on-failure

[Install]
WantedBy=multi-user.target
Then run:
sudo systemctl daemon-reload
And enable it on boot:
sudo systemctl enable mavros.service


        <node pkg="eurobot_display" name="display_node" type="gui.py" respawn="true" output="screen"/>

# required packages:
ros-laser-scan-matcher
>>>>>>> Stashed changes
sudo apt-get install ros-kinetic-laser-scan-matcher
roslaunch polar_scan_matcher demo.launch

cp -a .git .git-old1

git hash-object -w Cheatsheet.md
