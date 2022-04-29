import time
import numpy as np
import rospy
import tf
from gazebo_msgs.msg import ModelStates
from geometry_msgs.msg import Pose
from geometry_msgs.msg import Twist
from controller import Controller 

class RosWrap:
    def __init__(self, N, use_ui=False):
        self.body_length = 0.05

        self.N = N
        self.ctl = Controller(N, vcf=0.7, wcf=5.0, wlim=2.2)
        self.use_ui = use_ui
        self.ui = None
        self.bhav = []
        self.behav_seq = []
        self.bc_seq = []
        self.state_id= 0 
        self.sim_id = 1 # 0 for gazebo, 1 for vrep

        self.x = np.zeros([3, N])
        self.pose_time = [None for i in range(N)]
        self.vel = np.zeros([3, N])

        rospy.init_node('robot_control', anonymous=True) 
        self.state_sub = None
        self.tf_listener = None
        if self.sim_id == 0:
            self.state_sub = rospy.Subscriber('/gazebo/model_states', ModelStates, self.state_cb)
        elif self.sim_id == 1:
            self.tf_listener = tf.TransformListener()

        self.cmd_pub = [rospy.Publisher('r%d/cmd_vel' % i, Twist, queue_size=1) for i in range(N)]

        if use_ui:
            self.ui = UserInterface()
            self.ui.start()
            rospy.loginfo('UI created')

        rospy.loginfo('Robot controller init')

    def state_cb(self, data):
        for i in range(self.N):
            rid = data.name.index('r%d' % i)
            rpose = data.pose[rid]
            self.x[0, i] = rpose.position.x
            self.x[1, i] = rpose.position.y
            quat = (
                rpose.orientation.x,
                rpose.orientation.y,
                rpose.orientation.z,
                rpose.orientation.w
                )
            euler = tf.transformations.euler_from_quaternion(quat)
            #  self.x[2, i] = euler[2]
            self.x[2, i] = - euler[2]

    def listener_update(self):
        x = np.zeros([3, self.N])
        q_rot = tf.transformations.quaternion_from_euler(0, np.pi/2, 0, axes='rxyz')
        for i in range(self.N):
            try:
                robot_name = '/puzbot%d' % i
                t = self.tf_listener.getLatestCommonTime('/world', robot_name)
                (trans, rot) = self.tf_listener.lookupTransform('/world', robot_name, t)
                x[0, i] = trans[0]
                x[1, i] = trans[1]
                euler = tf.transformations.euler_from_quaternion(rot, axes='rxyz')
                q = tf.transformations.quaternion_multiply(q_rot, rot)
                en = tf.transformations.euler_from_quaternion(q, axes='rxyz')
                #  theta  = - euler[1] - np.pi/2
                theta  = en[0]
                x[2, i] = (theta + np.pi) % (2*np.pi) - np.pi

                self.x = x
            except (tf.LookupException, tf.ConnectivityException, tf.ExtrapolationException):
                continue

    def process_ui(self):
        N = self.N
        if self.use_ui and self.ui is None:
            return

        if self.use_ui and self.ui.behav is None:
            return

        self.behav_seq = [self.ctl.go_to_pair_pool for i in range(N-1)]
        #  self.behav_seq = [self.ctl.go_to_pair_pool for i in range(2*int(np.ceil(N/2))+2)]
        self.behav_seq.append(self.ctl.forward_y)
        self.bc_seq = []

    def compute(self):
        N = self.N
        x = self.x

        du = np.zeros([2, N])

        if np.linalg.norm(x) < 1e-8:
            return

        print("x: ", x)

        if self.state_id >= len(self.behav_seq):
            self.state_id = len(self.behav_seq)-1

        self.bhav = self.behav_seq[self.state_id]
        if self.state_id in self.bc_seq:
            du = self.bhav(x, use_bc=True)
        else:
            eth = 1.5e-3
            gth = 1e-1
            if self.state_id > (len(self.behav_seq)-3):
                self.ctl.vcf = 1.0
                self.ctl.wcf = 13.0
                eth=1.8e-3
                gth=2e-1
            du = self.bhav(x, eth=eth, dts=(np.zeros(N)+1e-1), pilot_ids=[], gth=2e-1)
        if np.linalg.norm(du) < 1e-5:
            self.state_id += 1

        print("state: ", self.state_id)
        print("du: ", du)
        for i in range(N):
            tw = Twist()
            tw.linear.x = du[0, i]
            tw.angular.z = du[1, i]
            self.cmd_pub[i].publish(tw)

    def start(self):
        rospy.loginfo('Robot controller started')
        rate = rospy.Rate(10)
        self.process_ui()
        while not rospy.is_shutdown():
            if self.sim_id == 1:
                self.listener_update()
            self.compute()
            rate.sleep()

