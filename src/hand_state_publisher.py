#!/usr/bin/python
import rospy
from openhand_node.srv import ReadServos, ReadLoad
from std_msgs.msg import Float32MultiArray, Int32MultiArray

def service_call_and_publish():
    rospy.init_node('service_caller_and_publisher', anonymous=True)

    # Create service clients
    rospy.wait_for_service('/openhand_node/read_servos')
    rospy.wait_for_service('/openhand_node/read_load')
    try:
        service_client1 = rospy.ServiceProxy('/openhand_node/read_servos', ReadServos)
        service_client2 = rospy.ServiceProxy('/openhand_node/read_load', ReadLoad)
    except rospy.ServiceException as e:
        rospy.logerr("Service call failed: %s", e)
        return

    # Create a publisher
    pos_pub = rospy.Publisher('/servo_pos', Float32MultiArray, queue_size=10)
    enc_pub = rospy.Publisher('/servo_encoder', Int32MultiArray, queue_size=10)
    load_pub = rospy.Publisher('/servo_load', Float32MultiArray, queue_size=10)

    rate = rospy.Rate(30)  # 1 Hz (Change this value for your desired frequency)

    while not rospy.is_shutdown():
        try:
            # Call the services
            response1 = service_client1()
            position = Float32MultiArray(data=response1.pos)
            encoder = Int32MultiArray(data=response1.enc)
            response2 = service_client2()
            load = Float32MultiArray(data=response2.load)
            # rospy.loginfo("Service called successfully")

            # Publish the response or any data you want
            pos_pub.publish(position)
            enc_pub.publish(encoder)
            load_pub.publish(load)
        except rospy.ServiceException as e:
            rospy.logerr("Service call failed during loop: %s", e)

        rate.sleep()

if __name__ == '__main__':
    try:
        service_call_and_publish()
    except rospy.ROSInterruptException:
        pass