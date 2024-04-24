import rospy
import roslib
roslib.load_manifest('amrl_msgs')
from amrl_msgs.msg import AckermannCurvatureDriveMsg

pub = rospy.Publisher('ackermann_curvature_drive', AckermannCurvatureDriveMsg, queue_size=10) # "key" is the publisher name
rospy.init_node('keypress',anonymous=True)

msg = AckermannCurvatureDriveMsg()
velocity = 0;
curvature = 0;

VEL = 1
LEFT_CURV = 1
RIGHT_CURV = -1

if __name__ == "__main__":
    msg.velocity = 1.0
    msg.curvature = 0
    rate = rospy.Rate(20)

    while not rospy.is_shutdown():
        pub.publish(msg)
        rate.sleep()