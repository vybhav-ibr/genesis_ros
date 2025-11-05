from gs_ros import GsRosBridge
import rclpy
from rclpy.node import Node
import genesis as gs

def main(args=None):
    gs.init(logging_level="debug",performance_mode=True)
    rclpy.init(args=args)

    default_ros_node=Node('gs_ros_bridge_node')
    # four ros2 nodes can be provided to the constructor 
    # default_ros_node(default node used for the clock,ros2_control and service unless overiden),ros_clock_node,ros_control_node,ros_service_node
    # if the default_ros_node is used for everything you may experience bottlenecks
    gs_ros_bridge=GsRosBridge(default_ros_node,"src/configs/ackerman_demo.yaml",add_debug_objects=True)
    gs_ros_bridge.build()
    
    while rclpy.ok():
        gs_ros_bridge.step()
    rclpy.shutdown()

if __name__=='__main__':
    main()