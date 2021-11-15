import pybullet as p
import time
import pybullet_data
import random
import rclpy
from rclpy.node import Node

from sensor_msgs.msg import JointState


TIME_STEP = 0.05


class PyBulletSim(Node):

    def __init__(self):
        super().__init__('PyBulletSim')
        self.subscription = self.create_subscription(
            JointState,
            '/joint_states',
            self.listener_callback,
            10)
        self.timer = self.create_timer(TIME_STEP, self.step)


        self.physicsClient = p.connect(p.GUI)#or p.DIRECT for non-graphical version

        p.setAdditionalSearchPath(pybullet_data.getDataPath()) #optionally
        p.setGravity(0,0,-10)

        self.planeId = p.loadURDF("plane.urdf")
        self.startPos = [0,0,0]
        self.flags = p.URDF_ENABLE_CACHED_GRAPHICS_SHAPES
        self.startOrientation = p.getQuaternionFromEuler([0,0,0])
        self.boxId = p.loadURDF("franka_panda/panda.urdf",self.startPos,self.startOrientation, useFixedBase=True, flags=self.flags)
        self.joints = {}
        for joint_id in range(p.getNumJoints(self.boxId)):
            self.joints[p.getJointInfo(self.boxId, joint_id)[1].decode()] = joint_id


    def step(self):
        p.stepSimulation()


    def listener_callback(self, msg:JointState):   
        for idx, joint_name in enumerate(msg.name):
            joint_id = self.joints[joint_name]
            p.setJointMotorControl2(bodyUniqueId=self.boxId, 
                jointIndex=joint_id, 
                controlMode=p.POSITION_CONTROL,
                targetPosition = msg.position[idx])



def main(args=None):

    rclpy.init(args=args)

    minimal_publisher = PyBulletSim()

    rclpy.spin(minimal_publisher)

    minimal_publisher.destroy_node()

    rclpy.shutdown()
    p.disconnect()


if __name__ == '__main__':
    main()
