import sys
import rclpy
from rclpy.node import Node
from motion_msgs.msg import MotionCtrl

from std_msgs.msg import Int32
from std_msgs.msg import String  # Import String message type for /diablo/MotionCmd

from PyQt5 import QtGui

class MyGuiNode(Node):
    def __init__(self, ui):
        super().__init__("ctrl_gui_node")
        self.ui = ui
        self.stand_mode_on = False
        self.height = 0.4

        # Define our GUI button callback here
        self.ui.btn_forward.pressed.connect(self.forward_button_callback)
        self.ui.btn_forward.released.connect(self.reset_button_callback)

        self.ui.btn_backward.pressed.connect(self.backward_button_callback)
        self.ui.btn_backward.released.connect(self.reset_button_callback)

        self.ui.btn_rotate_right.pressed.connect(self.rotate_right_callback)
        self.ui.btn_rotate_right.released.connect(self.reset_button_callback)

        self.ui.btn_rotate_left.pressed.connect(self.rotate_left_callback)
        self.ui.btn_rotate_left.released.connect(self.reset_button_callback)

        self.ui.btn_tilt_right.pressed.connect(self.tilt_right_callback)
        self.ui.btn_tilt_right.released.connect(self.reset_button_callback)

        self.ui.btn_tilt_left.pressed.connect(self.tilt_left_callback)
        self.ui.btn_tilt_left.released.connect(self.reset_button_callback)

        self.ui.btn_tilt_forward.clicked.connect(self.tilt_forward_callback)
        self.ui.btn_tilt_backward.clicked.connect(self.tilt_backward_callback)
        self.ui.btn_tilt_center.clicked.connect(self.tilt_center_callback)

        self.ui.btn_stand_mode.clicked.connect(self.stand_mode_callback)

        self.ui.btn_up.clicked.connect(self.up_callback)
        self.ui.btn_down.clicked.connect(self.down_callback)


        # Define ROS2 publisher here
        self._motion_cmd_publisher = self.create_publisher(MotionCtrl, '/diablo/MotionCmd', 10)
        self._subscriber    = self.create_subscription(MotionCtrl, '/diablo/MotionCmd', self.subscriber_callback, 1)

        # Sanity check
        self.get_logger().info("Node set up properly.")

    def forward_button_callback(self):
        cmd = MotionCtrl()
        cmd.mode_mark = False
        cmd.value.forward = 0.2

        # Publish the motion command
        self._motion_cmd_publisher.publish(cmd)
        self.get_logger().info("[Publisher] Sent motion command: forward")

    def backward_button_callback(self):
        cmd = MotionCtrl()
        cmd.mode_mark = False
        cmd.value.forward = -0.2

        # Publish the motion command
        self._motion_cmd_publisher.publish(cmd)
        self.get_logger().info("[Publisher] Sent motion command: backward")

    def rotate_right_callback(self):
        cmd = MotionCtrl()
        cmd.mode_mark = False
        cmd.value.left = -0.2

        # Publish the motion command
        self._motion_cmd_publisher.publish(cmd)
        self.get_logger().info("[Publisher] Sent motion command: rotate right")

    def rotate_left_callback(self):
        cmd = MotionCtrl()
        cmd.mode_mark = False
        cmd.value.left = 0.2

        # Publish the motion command
        self._motion_cmd_publisher.publish(cmd)
        self.get_logger().info("[Publisher] Sent motion command: rotate left")

    def tilt_right_callback(self):
        cmd = MotionCtrl()
        cmd.mode_mark = False
        cmd.value.roll = 0.2

        # Publish the motion command
        self._motion_cmd_publisher.publish(cmd)
        self.get_logger().info("[Publisher] Sent motion command: tilt right")

    def tilt_left_callback(self):
        cmd = MotionCtrl()
        cmd.mode_mark = False
        cmd.value.roll = -0.2

        # Publish the motion command
        self._motion_cmd_publisher.publish(cmd)
        self.get_logger().info("[Publisher] Sent motion command: tilt left")

    def tilt_forward_callback(self):
        cmd = MotionCtrl()
        cmd.mode_mark = False
        cmd.value.pitch = 0.5

        # Publish the motion command
        self._motion_cmd_publisher.publish(cmd)
        self.get_logger().info("[Publisher] Sent motion command: tilt forward")

    def tilt_backward_callback(self):
        cmd = MotionCtrl()
        cmd.mode_mark = False
        cmd.value.pitch = -0.5

        # Publish the motion command
        self._motion_cmd_publisher.publish(cmd)
        self.get_logger().info("[Publisher] Sent motion command: tilt backward")

    def tilt_center_callback(self):
        cmd = MotionCtrl()
        cmd.mode_mark = False
        cmd.value.pitch = 0.0
        cmd.value.roll = 0.0

        # Publish the motion command
        self._motion_cmd_publisher.publish(cmd)
        self.get_logger().info("[Publisher] Sent motion command: tilt center")

    def stand_mode_callback(self):
        if self.stand_mode_on == False:
            cmd = MotionCtrl()
            cmd.mode_mark = True
            cmd.mode.stand_mode = True
            cmd.value.up = 0.4
            self.stand_mode_on = True
            # Publish the motion command
            self._motion_cmd_publisher.publish(cmd)
            self.get_logger().info("[Publisher] Sent motion command: Stand mode is ON")
        elif self.stand_mode_on == True:
            cmd = MotionCtrl()
            cmd.mode_mark = True
            cmd.mode.stand_mode = False
            self.stand_mode_on = False
            # Publish the motion command
            self._motion_cmd_publisher.publish(cmd)
            self.get_logger().info("[Publisher] Sent motion command: Stand mode is OFF")

    def up_callback(self):
        if self.stand_mode_on == True:
            if self.height < 1.0:
                self.height = self.height + 0.2
                cmd = MotionCtrl()
                cmd.mode_mark = False
                cmd.value.up = self.height

                # Publish the motion command
                self._motion_cmd_publisher.publish(cmd)
                self.get_logger().info(f"[Publisher] Sent motion command: Move up. Current height is {self.height:.2f}")
            elif self.height == 1.0:
                self.get_logger().info("[Publisher] Maximum height is reached")
        elif self.stand_mode_on == False:
            self.get_logger().info("[Publisher] Turn on the stand mode to control the height")


    def down_callback(self):
        if self.stand_mode_on == True:
            if self.height >= 0.2:
                self.height = self.height - 0.2
                cmd = MotionCtrl()
                cmd.mode_mark = False
                cmd.value.up = self.height

                # Publish the motion command
                self._motion_cmd_publisher.publish(cmd)
                self.get_logger().info(f"[Publisher] Sent motion command: Move down. Current height is {self.height:.2f}")
            elif self.height == 0.0:
                self.get_logger().info("[Publisher] Minimum height is reached")
        elif self.stand_mode_on == False:
            self.get_logger().info("[Publisher] Turn on the stand mode to control the height")


    def reset_button_callback(self):
        cmd = MotionCtrl()
        cmd.mode_mark = False
        cmd.value.forward = 0.0
        cmd.value.left = 0.0
        cmd.value.up = 0.0
        cmd.value.roll = 0.0
        cmd.value.pitch = 0.0
        cmd.value.leg_split = 0.0

        # Publish the motion command
        self._motion_cmd_publisher.publish(cmd)
        self.get_logger().info("[Publisher] Sent motion command: reset")


    def subscriber_callback(self, msg):
        self.get_logger().info(f"[Subscriber] Command sent is {msg.value.forward:.2f}")
