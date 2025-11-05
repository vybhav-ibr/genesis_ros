from .gs_ros_sensor_helper import *
from .gs_ros_robot_control import *
from .gs_ros_sensors import *
from .gs_ros_services import *
from .gs_ros_sim import *
from .gs_ros_utils import make_gs_scene

from rclpy.node import Node
import rclpy
import yaml

class GsRosBridge:
    def __init__(self,ros_node,file_path=None,
                 ros_clock_node=None,ros_control_node=None,ros_service_node=None,
                 add_debug_objects=False):

        self.ros_node=ros_node
        self.all_nodes_to_spin=[self.ros_node]
        if ros_clock_node is not None:
            self.ros_clock_node=ros_clock_node
            self.all_nodes_to_spin.append(self.ros_clock_node)
        else:
            self.ros_clock_node=ros_node

        if ros_service_node is not None:
            self.ros_service_node=ros_service_node
            self.all_nodes_to_spin.append(self.ros_service_node)
        else:
            self.ros_service_node=ros_node
            
        if ros_control_node is not None:
            self.ros_control_node=ros_control_node
            self.all_nodes_to_spin.append(self.ros_control_node)
        else:
            self.ros_control_node=ros_node
        
        if file_path is not None:
            with open(file_path, 'r') as file:
                self.parent_config=yaml.safe_load(file)
            self.scene= make_gs_scene(scene_config=self.parent_config["scene"])
            self.sim=GsRosSim(self.scene,self.parent_config["scene"])
            self.sim.add_world(self.parent_config["world"])
            
            self.robots=[]
            self.objects=[]
            self.all_sensors={}

            if self.parent_config.get("objects") is not None:
                for _, object_config in self.parent_config.get("objects", {}).items():
                    object_name=object_config["name"]
                    gs.logger.info(f"Adding object {object_name} to scene")
                    object=self.sim.spawn_from_config(entity_config=object_config)
                    self.objects.append((object_name,object.idx,object))
                    
            if self.parent_config.get("robots") is not None:
                for robot_name, robot_config in self.parent_config.get("robots", {}).items():
                    namespace=robot_config.get("namespace","")
                    gs.logger.info(f"Adding robot {namespace} to scene")
                    robot=self.sim.spawn_from_config(entity_config=robot_config)
                    self.robots.append((robot_name,robot.idx,robot))
                    setattr(self,f"{robot_name}_robot_control",GsRosRobotControl(self.scene,self.ros_control_node,robot_config,robot))
                    sensor_factory=GsRosSensors(self.scene,self.sim,namespace,robot)
                    setattr(self,f"{robot_name}_sensor_factory",sensor_factory)
                    sensors={}
                    if robot_config.get("sensors") is not None:
                        for sensor_config in robot_config.get("sensors",{}):
                            sensors[sensor_config["name"]]=sensor_factory.add_sensor(sensor_config)
                    self.all_nodes_to_spin.extend(sensor_factory.all_ros_nodes)
                    self.all_sensors[robot_name]=sensors
            
            self.services=GsRosServices(self.scene,self.ros_service_node,self.robots,self.objects,self.all_sensors)
        else:
            self.scene=None
            gs.logger.warn(f"No config file path provided, please provide the path or add the scene, robots, etc as necessary")

        if add_debug_objects:
            # Number of obstacles to create in a ring around the robot
            NUM_CYLINDERS = 8
            NUM_BOXES = 6
            CYLINDER_RING_RADIUS = 3.0
            BOX_RING_RADIUS = 5.0

            for i in range(NUM_CYLINDERS):
                angle = 2 * np.pi * i / NUM_CYLINDERS
                x = CYLINDER_RING_RADIUS * np.cos(angle)
                y = CYLINDER_RING_RADIUS * np.sin(angle)
                self.scene.add_entity(
                    gs.morphs.Cylinder(
                        height=1.5,
                        radius=0.3,
                        pos=(x, y, 0.75),
                        fixed=True,
                    )
                )

            for i in range(NUM_BOXES):
                angle = 2 * np.pi * i / NUM_BOXES + np.pi / 6
                x = BOX_RING_RADIUS * np.cos(angle)
                y = BOX_RING_RADIUS * np.sin(angle)
                self.scene.add_entity(
                    gs.morphs.Box(
                        size=(0.5, 0.5, 2.0 * (i + 1) / NUM_BOXES),
                        pos=(x, y, 1.0),
                        fixed=False,
                    )
                )
            
    def build(self):
        self.sim.build_scene(self.parent_config["scene"])
        self.sim.start_clock(self.ros_clock_node)
        
    def step(self):
        self.scene.step()
        if rclpy.ok():
            for ros_node in self.all_nodes_to_spin:
                rclpy.spin_once(ros_node,timeout_sec=0)
            
            
                    