#!/usr/bin/python
import rospy
from openhand_node.srv import ReadServos
from std_msgs.msg import Float32MultiArray, Int32MultiArray

def service_call_and_publish():
    rospy.init_node('service_caller_and_publisher', anonymous=True)

    # Create a service client
    rospy.wait_for_service('/openhand_node/read_servos')
    try:
        service_client = rospy.ServiceProxy('/openhand_node/read_servos', ReadServos)
    except rospy.ServiceException as e:
        rospy.logerr("Service call failed: %s", e)
        return

    # Create a publisher
    pos_pub = rospy.Publisher('/servo_pos', Float32MultiArray, queue_size=10)
    enc_pub = rospy.Publisher('/servo_encoder', Int32MultiArray, queue_size=10)

    rate = rospy.Rate(30)  # 1 Hz (Change this value for your desired frequency)

    while not rospy.is_shutdown():
        try:
            # Call the service
            response = service_client()
            position = Float32MultiArray(data=response.pos)
            encoder = Int32MultiArray(data=response.enc)
            rospy.loginfo("Service called successfully")

            # Publish the response or any data you want
            pos_pub.publish(position)
            enc_pub.publish(encoder)
        except rospy.ServiceException as e:
            rospy.logerr("Service call failed during loop: %s", e)

        rate.sleep()

if __name__ == '__main__':
    try:
        service_call_and_publish()
    except rospy.ROSInterruptException:
        pass