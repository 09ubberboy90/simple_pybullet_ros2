import pybullet as p
import time
import pybullet_data
import random
import rclpy
from rclpy.node import Node
from trajectory_follower import TrajectoryFollower

from sensor_msgs.msg import JointState


TIME_STEP = 0.01

class PyBulletSim(Node):

    def __init__(self):
        super().__init__('PyBulletSim')
        self.publisher = self.create_publisher(JointState, "/joint_states", 10)
        self.timer = self.create_timer(TIME_STEP, self.step)
        gui = self.get_parameter('gui').get_parameter_value()
        print(gui)
        if gui:
            self.physicsClient = p.connect(p.GUI_SERVER)#or p.DIRECT for non-graphical version
        else:
            self.physicsClient = p.connect(p.SHARED_MEMORY_SERVER)#or p.DIRECT for non-graphical version
        p.setAdditionalSearchPath(pybullet_data.getDataPath()) #optionally
        p.setGravity(0,0,-9.8)
        self.planeId = p.loadURDF("plane.urdf")
        self.startPos = [0,0,0]
        self.flags = p.URDF_ENABLE_CACHED_GRAPHICS_SHAPES
        self.startOrientation = p.getQuaternionFromEuler([0,0,0])
        self.robot_id = p.loadURDF("franka_panda/panda.urdf",self.startPos,self.startOrientation, useFixedBase=True, flags=self.flags)
        self.joints = {}
        self.hand_joints = {}
        for joint_id in range(p.getNumJoints(self.robot_id)):
            name = p.getJointInfo(self.robot_id, joint_id)[1].decode()
            if "finger" in name:
                self.hand_joints[name] = joint_id
            else:
                self.joints[name] = joint_id

        self.follower = TrajectoryFollower(self.robot_id, self, self.joints, "panda_arm_controller")
        self.hand_follower = TrajectoryFollower(self.robot_id, self, self.hand_joints, "panda_hand_controller")
        self.joints.update(self.hand_joints)
        p.setRealTimeSimulation(1)


    def step(self):
        # p.stepSimulation()
        self.publisher.publish(self.publish())


    def publish(self) -> JointState:

        msg = JointState()
        msg.header.stamp = self.get_clock().now().to_msg()
        for name, idx in self.joints.items():
            if name in ["panda_grasptarget_hand"]:
                continue    
            state = p.getJointState(self.robot_id, idx)
            msg.name.append(name)
            msg.position.append(state[0])
            msg.velocity.append(state[1])
        return msg



def main(args=None):

    rclpy.init(args=args)

    minimal_publisher = PyBulletSim()
    executor = rclpy.executors.MultiThreadedExecutor()

    rclpy.spin(minimal_publisher, executor=executor)

    minimal_publisher.destroy_node()

    rclpy.shutdown()
    p.disconnect()


if __name__ == '__main__':
    main()
