#!/usr/bin/env python
import sys, rospy, gazebo_msgs.msg, tf, numpy as np

def list_of_position(p): return (p.x, p.y, p.z)
def list_of_orientation(q): return (q.x, q.y, q.z, q.w)
def T_of_tq(t, q):
    T = tf.transformations.quaternion_matrix(q)
    T[:3,3] = t
    return T
def tq_of_T(T):
    return T[:3, 3], tf.transformations.quaternion_from_matrix(T)

''' Subscribe to gazebo/model_states messages and store the robot pose and twist '''
class TruthListener:
    def __init__(self, model_name="rosmip"):
        self.model_name, self.model_idx = model_name, None
        rospy.Subscriber('/gazebo/model_states', gazebo_msgs.msg.ModelStates, self.callback)
        self.pose, self.twist = None, None
        self.body_to_world_T = None

    def callback(self, msg):
        if self.model_idx is None:
            try:
                self.model_idx = msg.name.index(self.model_name)
            except:
                rospy.logerr('model {} not found in gazebo {}'.format(self.model_name, msg.name))
        if self.model_idx is not None:
            self.pose, self.twist = msg.pose[self.model_idx], msg.twist[self.model_idx]
            self.body_to_world_T = None

    def get_body_to_world_T(self):
        if self.pose is not None and self.body_to_world_T is None:
            _t, _q = list_of_position(self.pose.position), list_of_orientation(self.pose.orientation)
            self.body_to_world_T = T_of_tq(_t, _q)
        return self.body_to_world_T

    
class FakeLocalization:

    def __init__(self):
        self.trl = TruthListener()
        self.br = tf.TransformBroadcaster()
        self.tfl = tf.TransformListener()

    def run(self):
        rate = rospy.Rate(50.)
        while not rospy.is_shutdown():
            self.periodic()
            rate.sleep()
            
    def periodic(self):
        try:
            bl_to_odom_t, bl_to_odom_q = self.tfl.lookupTransform(target_frame='/odom', source_frame='/base_link', time=rospy.Time(0))
            bl_to_odom_T = T_of_tq(bl_to_odom_t, bl_to_odom_q)
            odom_to_bl_T =  np.linalg.inv(bl_to_odom_T)
            bl_to_map_T = self.trl.get_body_to_world_T()
            map_to_odom_T = np.dot(bl_to_map_T, odom_to_bl_T)
            map_to_odom_t, map_to_odom_q = tq_of_T(map_to_odom_T)
            #map_to_odom_t, map_to_odom_q = [0.5, 0.5, 0], tf.transformations.quaternion_from_euler(0, 0, 0.)
            
            # from: http://wiki.ros.org/tf/Tutorials/Writing%20a%20tf%20broadcaster%20%28Python%29
            #   br.sendTransform([...], turtlename, "world")
            #   [...] publishes it as a transform from frame "world" to frame "turtleX
            # here we publish the transform from map to odom
            self.br.sendTransform(map_to_odom_t, map_to_odom_q, rospy.Time.now(), 'odom', 'map')
        except (tf.LookupException, tf.ConnectivityException, tf.ExtrapolationException):
            print "tf lookup failure"
            
def main(args):
  rospy.init_node('fake_localization')
  FakeLocalization().run()

if __name__ == '__main__':
    main(sys.argv)

