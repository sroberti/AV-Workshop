#!/usr/bin/env python
import rospy
from std_msgs.msg import Float64
from ackermann_msgs.msg import AckermannDriveStamped


class easyNav():
    def __init__(self):
        self.speed = 0.0
        self.steering = 0.0

        rospy.init_node("VESC_Interface")

        rospy.Subscriber("speed", Float64, self.updateSpeed)
        rospy.Subscriber("steering", Float64, self.updateSteering)

        self.pub = rospy.Publisher("/vesc/ackermann_cmd_mux/input/navigation", AckermannDriveStamped, queue_size=1)
        rospy.spin()


    def updateSpeed(self, data):
        self.speed = data.data
        self.forward()

    def updateSteering(self, data):
        self.steering = data.data
        self.forward()

    def forward(self):
        instructions = AckermannDriveStamped()
        instructions.header.seq = 0
        instructions.header.stamp = rospy.Time.now()
        instructions.header.frame_id = ''
        instructions.drive.speed = self.speed
        instructions.drive.steering_angle = self.steer
        publisher = rospy.Publisher("/vesc/ackermann_cmd_mux/input/navigation", AckermannDriveStamped, queue_size=1)

        publisher.publish(instructions)




if __name__ == '__main__':
    easyNav()

    