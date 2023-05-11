# HKUST ELEC5660 - Introduction to Aerial Robotics

<div id="course_home_content">

<div id="course_syllabus" style="margin-bottom: 10px;" class="user_content enhanced">
  <p>This course gives a comprehensive introduction to aerial robots. The goal of this course is to expose students to relevant mathematical foundations and algorithms, and train them to develop real-time software modules for aerial robotic systems. Topics to be covered include rigid-body dynamics, system modeling, control, trajectory planning, sensor fusion, and vision-based state estimation. Students will complete a series of projects that can be combined into an aerial robot that is capable of vision-based autonomous indoor navigation.</p>

<p><strong>Instructor:</strong></p>
<ul style="list-style-type: disc;">
<li><span><a class="inline_disabled external" href="http://uav.hkust.edu.hk/" target="_blank" rel="noreferrer noopener"><span>Shaojie Shen</span><span class="external_link_icon" style="margin-inline-start: 5px; display: inline-block; text-indent: initial; " role="presentation"><svg viewBox="0 0 1920 1920" version="1.1" xmlns="http://www.w3.org/2000/svg" style="width:1em; height:1em; vertical-align:middle; fill:currentColor">
    <path d="M1226.66667,267 C1314.88,267 1386.66667,338.786667 1386.66667,427 L1386.66667,427 L1386.66667,853.666667 L1280,853.666667 L1280,693.666667 L106.666667,693.666667 L106.666667,1493.66667 C106.666667,1523 130.56,1547 160,1547 L160,1547 L1226.66667,1547 C1256.10667,1547 1280,1523 1280,1493.66667 L1280,1493.66667 L1280,1280.33333 L1386.66667,1280.33333 L1386.66667,1493.66667 C1386.66667,1581.88 1314.88,1653.66667 1226.66667,1653.66667 L1226.66667,1653.66667 L160,1653.66667 C71.7866667,1653.66667 0,1581.88 0,1493.66667 L0,1493.66667 L0,427 C0,338.786667 71.7866667,267 160,267 L160,267 Z M1584.37333,709.293333 L1904.37333,1029.29333 C1925.17333,1050.09333 1925.17333,1083.90667 1904.37333,1104.70667 L1904.37333,1104.70667 L1584.37333,1424.70667 L1508.96,1349.29333 L1737.86667,1120.38667 L906.613333,1120.38667 L906.613333,1013.72 L1737.86667,1013.72 L1508.96,784.706667 L1584.37333,709.293333 Z M1226.66667,373.666667 L160,373.666667 C130.56,373.666667 106.666667,397.666667 106.666667,427 L106.666667,427 L106.666667,587 L1280,587 L1280,427 C1280,397.666667 1256.10667,373.666667 1226.66667,373.666667 L1226.66667,373.666667 Z" stroke="none" stroke-width="1" fill-rule="evenodd"></path>
</svg>
</a> </span>(eeshaojie@ust.hk)</li>
</ul>
<p><strong>Teaching Assistants:&nbsp;</strong></p>
<ul>
<li><span>Haokun Wang (hwangeh@connect.ust.hk)</span></li>
<li><span>Peize Liu (pliuan@connect.ust.hk)</span></li>
</ul>
<p><strong>Lecture:</strong></p>
<ul>
<li><span>Rm 5560, Lift 27-28 (30)</span></li>
<li><span>Tuesday 13:30 - 16:20</span></li>
</ul>
<p><strong>Lab:</strong></p>
<ul>
<li><span>Rm G03, Lo Ka Chung University Center</span></li>
<li><span>Wednesday 18:00 - 20:50 OR Thursday 13:30 - 16:20</span></li>
</ul>
<p><strong>Reference Book:</strong></p>
<ul style="list-style-type: disc;">
<li>Murray, R. M., Li, Z., &amp; Sastry, S. S.. A mathematical introduction to robotic manipulation., 1994.</li>
</ul>
<p><strong>Midterm exam: </strong></p>
<ul>
<li>The midterm exam will be open book, open notes, close Internet, and close classmates. Honor code will be enforced.</li>
</ul>
<p><strong>Project Overview:&nbsp;</strong></p>
<ul>
<li>Project 1 (Control and planning):
<ul>
<li>Phase1: Quadrotor trajectory tracking control. A simulator implementing the dynamics model of quadrotor is given. You need to implement a controller that outputs force and moment, meanwhile command the quadrotor to track pre-defined trajectories.&nbsp;</li>
<li>Phase2: Optimization-based trajectory generation. Implementation of a trajectory generator to obtain trajectory that connects pre-defined waypoints and meets smoothness requirements. Use the controller in Phase1 to track the trajectory.</li>
<li>Phase3: Path planning + trajectory generation + control. Grid maps containing obstacles, start and end locations are provided. You need to implement an A* path finder to search for shortest path with safety guarantee. Then you should connect your path using previous trajectory generator and track it with your controller.</li>
<li>Phase 4: In this lab assignment, you will learn how to control the drone both manually and autonomously. You need to setup the development environment and fly your drone in autonomous control mode with a motion capture system called OptiTrack. By using OptiTrack, you can get highly accurate position feedback&nbsp;of the drone. You need to verify your controller and trajectory planning algorithms that you developed.</li>
</ul>
</li>
<li>Project 2 (Visual estimator)&nbsp;
<ul>
<li>Phase1: PnP-based localization on marker map. You are provided with images containing AR marker, and a tag detector to calculate the 3D positions for those markers. You need to implement the PnP-based localization method to calculate the camera pose corresponding to each image.</li>
<li>Phase2: Visual odometry in markerless environment. You need to implement a PnP-based estimator to estimate the incremental motion of the camera. The provided images contain no AR marker, so you need to do feature detection and matching, and use them for single keyframe-based pose estimation.</li>
</ul>
</li>
<li>Project 3 (EKF sensor fusion):
<ul>
<li>Phase1: Sensor fusion of IMU and&nbsp;PnP localization on marker using EKF. You need to implement the process model of IMU, the measurement models of PnP pose estimator to integrate them into an EKF-based sensor fusion method.</li>
<li>Phase2: Sensor fusion of keyframe-based visual odometry together with Phase 1 using augmented state EKF.&nbsp;</li>
<li>Phase3: In this lab assignment, you need to integrate the whole system onboard. Using the information from the IMU and camera, your visual estimator and the EKF computes the state of the quadrotor. The drone will use this state for feedback control, and execute trajectories computed by your path planner and your trajectory generator.</li>
</ul>
</li>
</ul>
<p><strong>Lab Overview:</strong></p>
<ul>
<li>A number of lab tutorials are arranged to equip you with sufficient knowledge to operate the experimental drone platform:
<ul>
<li>Lab Tutorial 1: Drone hardware setup and software introduction.</li>
<li>Lab Tutorial 2: Preparation of Project 1 Phase 4, trajectory planning and tracking using motion tracking system.</li>
<li>Lab Tutorial 3: Preparation of Project 3 Phase 3, trajectory planning and tracking using onboard estimator.</li>
</ul>
</li>
</ul>
<p><strong>Grading Scheme:</strong></p>
<ul>
<li>Midterm Exam: 20%</li>
<li>Project 1: 30% (Phase1: 6%, Phase2: 6%, Phase3: 8%, Phase 4: 10%)</li>
<li>Project 2: 20% (Phase1: 8%, Phase2: 12%)</li>
<li>Project 3: 30% (Phase1: 10%, Phase2: 10%, Phase 3: 10%)</li>
</ul>