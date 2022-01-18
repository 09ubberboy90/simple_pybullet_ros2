import rclpy
import os
import sys

from ament_index_python.packages import get_package_share_directory
from geometry_msgs.msg import Pose, Point, Quaternion
from gazebo_msgs.srv import GetEntityState, GetModelList
from gazebo_msgs.msg import EntityState
import pyquaternion 
import pybullet as p
import pybullet_data

from rclpy.node import Node

class SpawnerNode(Node):
    def __init__(self, args=None):
        super().__init__("SpawnerNode")
        self.physicsClient = p.connect(p.SHARED_MEMORY)#or p.DIRECT for non-graphical version
        p.setAdditionalSearchPath(pybullet_data.getDataPath())

        self.entity = self.create_service(
            GetEntityState, 'get_entity_state', self.get_entity_state)
        self.model = self.create_service(
            GetModelList, 'get_model_list', self.get_model_list)
        self.objs = {}
        self.spawn_obj("table/table.urdf", position=[0.65, 0, -0.35], rotation=[ 0, 0, 0.7068252, 0.7073883 ]) #rotate 90 degree around z axis

        for x in range(2, 5):
            for y in range(-3, 4):
                self.spawn_obj("cube.urdf", [x/10, y/10, 1], scale=0.05)

    def spawn_obj(self, path, position=[0, 0, 0], rotation = [0,0,0,1], scale = 1):
        name = path.split("/")[-1].split(".")[0]
        if name != "table":
            name += "_" + str(len(self.objs.keys()))
        self.objs[name] = p.loadURDF(path, basePosition = position, baseOrientation = rotation, globalScaling = scale)
        # p.changeDynamics(self.objs[name], -1, 1/scale) #fix mass
    def get_model_list(self, request: GetModelList.Request, response: GetModelList.Response):
        response.model_names = list(self.objs.keys())
        response.success = True
        return response

    def get_entity_state(self, request: GetEntityState.Request, response: GetEntityState.Response):
        obj = self.objs.get(request.name)
        success = True
        if obj is None:
            response.success = False
            return response
        state = EntityState()
        state.name = request.name
        pose = Pose()
        try:    
            pose.position, pose.orientation = self.get_postion_rotation(obj)
        except: # object got deleted
            success = False
        finally:    
            state.pose = pose
            response.state = state
            response.success = success
        return response

    def get_postion_rotation(self, obj):
        out = p.getBasePositionAndOrientation(obj)
        position = Point()
        obj_pose = out[0]
        position.x = obj_pose[0]
        position.y = obj_pose[1]
        position.z = obj_pose[2]
        obj_rot = out[1]
        rotation = Quaternion()
        rotation.x = float(obj_rot[0])
        rotation.y = float(obj_rot[1])
        rotation.z = float(obj_rot[2])
        rotation.w = float(obj_rot[3])
        return position, rotation


def main(args=None):
    rclpy.init(args=args)

    spawner = SpawnerNode(args=args)

    rclpy.spin(spawner)
    rclpy.shutdown()


if __name__ == '__main__':
    main()
