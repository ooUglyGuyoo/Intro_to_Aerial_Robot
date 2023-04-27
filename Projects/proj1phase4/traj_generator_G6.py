import numpy as np
import rospy
from nav_msgs.msg import Odometry
from geometry_msgs.msg import PoseStamped

from quadrotor_msgs.msg import PositionCommand

class CircleTrajectoryGenerator():
    def __init__(self):
        rospy.init_node("trajectory_generator", anonymous=True)
        # ROS publishers and subscribers
        self._uav_pos_sub = rospy.Subscriber("/pos_vel_mocap/odom_TA", Odometry, self._uav_pos_callback)
        self._trigger_sub = rospy.Subscriber("/traj_start_trigger", PoseStamped, self._trigger_callback)
        self._position_cmd_pub = rospy.Publisher('/position_cmd', PositionCommand, queue_size=1)
        self._current_pose_pub = rospy.Publisher('/current_pose', PositionCommand, queue_size=1)
        
        # Parameters
        self._phase4 = True
        self._is_init = False
        self._is_traj = False
        self._odom_time = 0.0
        self._start_time = 0.0
        self._hover_pos = []
        self._hover_yaw = 0.0

        # Path parameters
        self._path_index = -1
        self._path_s_t = 0.0
        P_filename = "P.txt"
        t_filename = "t.txt"
        self._path_time = np.loadtxt(t_filename, delimiter="\t")
        self._path_poly_coeff = np.loadtxt(P_filename,delimiter="\t")
        print(self._path_time.shape)
        print(self._path_poly_coeff.shape)
        # self._path_time = [0, 0.790569415042095, 1.46827091774948, 2.46631204448566, 4.99498322974010, 8.01557635070268, 11.5316301894276, 15.5447805653572, 20.0559153371813]
        # self._path_poly_coeff = np.array([[0, 0, 0], \
        #                         [-0.541240401651883, -0.272446424437893, 0.982057793302330], \
        #                         [0.743807391782445, 0.356797567719671, -0.457213709210892], \
        #                         [3.70645380288367e-12, 8.39711722942162e-13, -1.03971211541894e-12], \
        #                         [-1.58855103720471e-11, -3.26141765586772e-12, 3.87976367779114e-12], \
        #                         [-0.204968264896233, -0.0421703688548346, 0.0519393122276872], \
        #                         [0.107749393208964, 0.0221684642635020, -0.0273038822817132], \
        #                         [7.12999351789139e-12, 1.47770340774168e-12, -1.81962919952641e-12], \
        #                         [0, 0, 0.500000000000000], \
        #                         [0.434142239590144, 0.250411945100968, 0.309992129867670], \
        #                         [0.362391234621995, 0.278324639039516, -0.360562196430944], \
        #                         [-0.216259471046115, -0.0444934325172579, 0.0548005233614933], \
        #                         [-1.98861422369898, -0.409139411734849, 0.503918277873368], \
        #                         [3.19782468531588, 0.608436176272104, -0.785125294256833], \
        #                         [-2.35912166184493, -0.455620061449977, 0.582651834150112], \
        #                         [0.724081248751273, 0.148973175709972, -0.183483438650309], \
        #                         [0.250000000000000, 0.250000000000000, 0.600000000000000], \
        #                         [-0.00821247323079544, 0.408907946155385, 0.0714359930714949], \
        #                         [-0.894608985265356, -0.0401991025475611, -0.0115192223338384], \
        #                         [-0.260037835157769, -0.0956023850595374, 0.0873408879283920], \
        #                         [0.576557207470299, -0.154026140600479, -0.00721314926709738], \
        #                         [-0.117161498180217, 0.240114680187018, -0.101197427288912], \
        #                         [-0.0759121026239713, -0.140737630784106, 0.0815050081183772], \
        #                         [0.0271769615903298, 0.0315488468370018, -0.0201094780933023], \
        #                         [-0.500000000000000, 0.500000000000000, 0.700000000000000], \
        #                         [-1.12252940628057, 0.00375698620032433, 0.123861393923875], \
        #                         [0.0432286741880859, -0.298430053151183, -0.00431356777969731], \
        #                         [0.309069065878620, -0.0214119100208882, -0.0272622995122337], \
        #                         [0.00435574828643813, 0.0427575655246256, -0.00116160919119024], \
        #                         [-0.0688799100605758, -0.00878624052600082, 0.00725040326304581], \
        #                         [0.0216566697762976, 0.000607012848152623, -0.00227590406940754], \
        #                         [-0.00212341969118646, -5.38892037749728e-06, 0.000223978999106245], \
        #                         [-0.750000000000000, -0.750000000000000, 0.800000000000000], \
        #                         [0.773690023332882, -0.580362689758560, -0.0157560296645487], \
        #                         [0.0893931773498035, 0.119479016303780, 0.00712001088514474], \
        #                         [-0.0865406773971343, 0.0378410890812299, 0.00913288673288860], \
        #                         [-0.01087852060026,	-0.00295521489313884, -0.000516394855408560], \
        #                         [0.0173103154076309, -0.00436695708031137, -0.00138060881658503], \
        #                         [-0.00431246881829772, 0.00126058576827087, 0.000389730378688370], \
        #                         [0.000353386709527461, -0.000106064960721359, -3.26633916079613e-05], \
        #                         [1, -1, 0.900000000000000], \
        #                         [0.323394348419660, 0.371713679611639, 0.0599909439034316], \
        #                         [-0.0385804868948841, 0.111098214252496, -0.00470537296874911], \
        #                         [0.0140363879125480, -0.0105103705797786, -0.00342436280836538], \
        #                         [-0.0111762325548233, 9.09174278335667e-05, 9.67030896292220e-05], \
        #                         [0.00111538777967689, 0.000474290586465340, 0.000394838266487993], \
        #                         [0.000109458759811739, -0.000202276683104314, -9.20662037781099e-05], \
        #                         [-1.78110924578666e-05, 2.10513424865458e-05, 6.48559963705928e-06], \
        #                         [1.25000000000000, 1.25000000000000, 1], \
        #                         [-0.400846030159331, 0.767644886372365, 0.00737317882319627], \
        #                         [-0.184754688820863, -0.0130215521358527, 9.94893078879606e-05], \
        #                         [-0.00537422717009013, -0.0138387028888059, 0.00140287182780701], \
        #                         [0.0196520097743569, -0.0274096292641233, 0.000423244434396366], \
        #                         [-0.00408562362299362, 0.00836984969386545, -0.000245002227783339], \
        #                         [0.000377431792668073, -0.000957690109790828, 3.75487547395897e-05], \
        #                         [-1.42948802530832e-05, 4.00394723571484e-05, -1.99654987127628e-06], \
        #                         [-1.50000000000000, 1.50000000000000, 1.10000000000000], \
        #                         [-0.422120500364295, -1.04737601521576, 0.0437617534637547], \
        #                         [0.164895605295913, -0.269451071503919, 0.00198384317222990], \
        #                         [0.0102050428149070, 0.0196825766105826, -0.000848986262599444], \
        #                         [-0.0192433802269578, 0.0684644321791107, -0.00130455578724219], \
        #                         [0.00430560698679262, -0.0205785339688434, 0.000440783581897320], \
        #                         [-0.000359512870293097, 0.00209050364706154, -4.75527790025529e-05], \
        #                         [8.34200102559854e-06, -6.24767203341479e-05, 1.51765198163418e-06] ])
        # State
        self._cur_pos = []
        self._cur_yaw = 0.0
        # CMD msg info
        self._traj_id = 0
        
        # for circle trajectory
        self._radius = 1.0
        self._period = 10.0
        self._rev    = 1.0
        self._follow_yaw = False

        print("test")

        rospy.spin()

    def _uav_pos_callback(self, msg):
        #print("pos_callback")
        self._odom_time = msg.header.stamp.to_sec()
        self._cur_pos = [msg.pose.pose.position.x, 
                         msg.pose.pose.position.y, 
                         msg.pose.pose.position.z]
        self._cur_yaw = np.math.atan2(2 * (msg.pose.pose.orientation.w * msg.pose.pose.orientation.z +
                                           msg.pose.pose.orientation.x * msg.pose.pose.orientation.y),
                                      1 - 2 * (msg.pose.pose.orientation.y * msg.pose.pose.orientation.y + 
                                               msg.pose.pose.orientation.z * msg.pose.pose.orientation.z))

        if (self._is_init):
            if (self._is_traj):
                if(self._phase4):
                    is_traj_finished = False
                    if self._odom_time - self._start_time > self._path_time[7]:
                        is_traj_finished = True
                    else:
                        for i in range(0,7):
                            if self._odom_time - self._start_time > self._path_time[i] and self._odom_time - self._start_time < self._path_time[i+1]:
                                self._path_index = i
                                self._path_s_t = self._path_time[i] + self._start_time
                                break
                    if (not is_traj_finished):
                        dt = self._odom_time - self._path_s_t
                        f0 = np.array([1.0,dt,dt**2,dt**3,dt**4,dt**5,dt**6,dt**7]).reshape(1,8)
                        f1 = np.array([0.0,1.0,2*dt,3*dt**2,4*dt**3,5*dt**4,6*dt**5,7*dt**6]).reshape(1,8)
                        f2 = np.array([0.0,0.0,2.0,6*dt,12*dt**2,20*dt**3,30*dt**4,42*dt**5]).reshape(1,8)
                        [[des_x,des_y,des_z]] = np.dot(f0,self._path_poly_coeff[0+8*self._path_index:8+8*self._path_index])
                        [[des_vx,des_vy,des_vz]] = np.dot(f1,self._path_poly_coeff[0+8*self._path_index:8+8*self._path_index])
                        [[des_ax,des_ay,des_az]] = np.dot(f2,self._path_poly_coeff[0+8*self._path_index:8+8*self._path_index])
                    else:
                        des_x = -1.75
                        des_y = -1.75
                        des_z = 0.7
                    
                    des_x = des_x + self._hover_pos[0]
                    des_y = des_y + self._hover_pos[1]
                    des_z = des_z + self._hover_pos[2] - 0.5

                    des_yaw = 0.0

                else :
                    dt = self._odom_time - self._start_time
                    is_traj_finished = dt > self._rev * self._period
                    dt = min(dt, self._rev * self._period)

                    des_x = self._hover_pos[0] + self._radius * np.math.sin(2 * np.math.pi / self._period * (dt))
                    des_y = self._hover_pos[1] - self._radius * np.math.cos(2 * np.math.pi / self._period * (dt)) + self._radius
                    des_z = self._hover_pos[2]

                    des_vx = self._radius * 2 * np.math.pi / self._period * np.math.cos(2 * np.math.pi / self._period * (dt))
                    des_vy = self._radius * 2 * np.math.pi / self._period * np.math.sin(2 * np.math.pi / self._period * (dt))
                    des_vz = 0.0

                    des_ax = -4 * np.math.pi * np.math.pi / self._period / self._period * self._radius * np.math.sin(2 * np.math.pi / self._period * (dt))
                    des_ay = 4 * np.math.pi * np.math.pi / self._period / self._period * self._radius * np.math.cos(2 * np.math.pi / self._period * (dt))
                    des_az = 0.0

                    des_yaw = 0.0
                    if (self._follow_yaw):
                        des_yaw = np.math.atan2(des_vy, des_vx)
                        while (des_yaw > np.math.pi):
                            des_yaw -= 2*np.math.pi
                        while (des_yaw < -np.math.pi):
                            des_yaw += 2*np.math.pi

                position_cmd = PositionCommand()
                position_cmd.header.stamp = msg.header.stamp
                position_cmd.header.frame_id = "world"
                if (not is_traj_finished):
                    position_cmd.position.x = des_x
                    position_cmd.position.y = des_y
                    position_cmd.position.z = des_z
                    position_cmd.velocity.x = des_vx
                    position_cmd.velocity.y = des_vy
                    position_cmd.velocity.z = des_vz
                    position_cmd.acceleration.x = des_ax
                    position_cmd.acceleration.y = des_ay
                    position_cmd.acceleration.z = des_az
                    position_cmd.yaw = des_yaw
                    position_cmd.trajectory_flag = position_cmd.TRAJECTORY_STATUS_READY
                    position_cmd.trajectory_id = self._traj_id
                    # publish cmd msg
                    self._position_cmd_pub.publish(position_cmd)
                else:
                    position_cmd.position.x = des_x
                    position_cmd.position.y = des_y
                    position_cmd.position.z = des_z
                    position_cmd.velocity.x = 0.0
                    position_cmd.velocity.y = 0.0
                    position_cmd.velocity.z = 0.0
                    position_cmd.acceleration.x = 0.0
                    position_cmd.acceleration.y = 0.0
                    position_cmd.acceleration.z = 0.0
                    position_cmd.yaw = des_yaw
                    position_cmd.trajectory_flag = position_cmd.TRAJECTORY_STATUS_COMPLETED
                    position_cmd.trajectory_id = self._traj_id
                # publish current state
                position_cmd.position.x      = msg.pose.pose.position.x
                position_cmd.position.y      = msg.pose.pose.position.y
                position_cmd.position.z      = msg.pose.pose.position.z
                position_cmd.velocity.x      = msg.twist.twist.linear.x
                position_cmd.velocity.y      = msg.twist.twist.linear.y
                position_cmd.velocity.z      = msg.twist.twist.linear.z
                position_cmd.acceleration.x  = msg.twist.twist.angular.x
                position_cmd.acceleration.y  = msg.twist.twist.angular.y
                position_cmd.acceleration.z  = msg.twist.twist.angular.z
                position_cmd.yaw             = self._cur_yaw
                self._current_pose_pub.publish(position_cmd)
            else:
                pass
        else:
            self._is_init = True
            print("rcv odom")

    def _trigger_callback(self, msg):
        # print("trigger_callback")
        if (self._is_init):
            rospy.loginfo("[#INFO] Get trajectory trigger INFO.")
            self._traj_id = msg.header.seq + 1
            self._is_traj = True

            # update hover state
            self._hover_pos = self._cur_pos
            self._hover_yaw = self._cur_yaw
            self._start_time = self._odom_time

            print("circle")
            rospy.loginfo("[#INFO] Circle start!")
            rospy.loginfo("[#INFO] Start position: [%f,%f,%f].", self._hover_pos[0], self._hover_pos[1], self._hover_pos[2])
            rospy.loginfo("[#INFO] Follow YAW: %d.", self._follow_yaw)


if __name__ == '__main__':
    traj_generator = CircleTrajectoryGenerator()
    


