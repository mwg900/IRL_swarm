군집로봇 위치인식
===============

시뮬레이션
===============
가제보 시뮬레이션 실행
-----------------------
<pre><code> $ roslaunch mobile_robot_sim turtlebot_rplidar.launch </code></pre>

이미지 프로세스 실행
---------------------
<pre><code> $ rosrun robot_mapping image_process.py </code></pre>

폴더 권한 수정
-------------
<pre><code> sudo chmod 775 -R ~/catkin_ws </code></pre>

슬레이브로봇인식
===============
<pre><code> $ roslaunch robot_mapping robot_mapping.launch </code></pre>
