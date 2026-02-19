#!/usr/bin/env python3
import rclpy
from rclpy.action import ActionClient
from rclpy.node import Node
from rcl_interfaces.msg import ParameterDescriptor
from nav2_msgs.action import NavigateToPose
from geometry_msgs.msg import PoseStamped
import yaml
import networkx as nx
import math
import time
import numpy as np
from nav2_msgs.action import ComputePathToPose


def load_graph_from_yaml(path):
    with open(path, 'r') as f:
        data = yaml.safe_load(f)
    G = nx.Graph()
    for n in data.get("nodes", []):
        G.add_node(n["id"], x=n.get("x"), y=n.get("y"))
    for e in data.get("edges", []):
        u, v, w = e["u"], e["v"], float(e["cost"])
        G.add_edge(u, v, weight=w)
    return G


def make_eulerian_graph(G):
    M = nx.MultiGraph()
    for u, v, d in G.edges(data=True):
        M.add_edge(u, v, weight=d['weight'])

    odd_nodes = [n for n, deg in G.degree() if deg % 2 == 1]
    if not odd_nodes:
        return M

    K = nx.Graph()
    for i in range(len(odd_nodes)):
        for j in range(i + 1, len(odd_nodes)):
            a, b = odd_nodes[i], odd_nodes[j]
            dist = nx.shortest_path_length(G, a, b, weight='weight')
            K.add_edge(a, b, weight=dist)

    mate = nx.algorithms.matching.min_weight_matching(K, weight='weight')
    for a, b in mate:
        path = nx.shortest_path(G, a, b, weight='weight')
        for i in range(len(path) - 1):
            u, v = path[i], path[i + 1]
            w = G[u][v]['weight']
            M.add_edge(u, v, weight=w)
    return M


def cpp_route_from_eulerian(M, start=1):
    circuit = list(nx.eulerian_circuit(M, source=start, keys=True))
    route = [start]
    total_cost = 0.0
    for u, v, key in circuit:
        w = M[u][v][key]['weight']
        route.append(v)
        total_cost += w
    return route, total_cost


def chinese_postman_from_yaml(yaml_path, start_node=1):
    G = load_graph_from_yaml(yaml_path)
    M = make_eulerian_graph(G)
    assert all(deg % 2 == 0 for _, deg in M.degree()), "Not Eulerian!"
    route, cost = cpp_route_from_eulerian(M, start=start_node)
    return route, cost, G


class CPPNavExecutor(Node):
    def __init__(self):
        super().__init__('cpp_nav_executor_optimized')
        
        # Declare parameters
        self.declare_parameter('robot_namespace', 'robot1',
            ParameterDescriptor(description='Robot namespace for action topics'))
        self.declare_parameter('graph_yaml_path', '',
            ParameterDescriptor(description='Path to the graph YAML file'))
        self.declare_parameter('start_node', 1,
            ParameterDescriptor(description='Starting node ID for CPP route'))
        
        # Get parameters
        self.robot_namespace = self.get_parameter('robot_namespace').get_parameter_value().string_value
        self.yaml_path = self.get_parameter('graph_yaml_path').get_parameter_value().string_value
        self.start_node = self.get_parameter('start_node').get_parameter_value().integer_value
        
        # Build action topic names based on namespace
        navigate_topic = f'/{self.robot_namespace}/navigate_to_pose'
        compute_path_topic = f'/{self.robot_namespace}/compute_path_to_pose'
        
        self.get_logger().info(f'Using namespace: {self.robot_namespace}')
        self.get_logger().info(f'Navigate action: {navigate_topic}')
        self.get_logger().info(f'Compute path action: {compute_path_topic}')
        
        self._action_client = ActionClient(self, NavigateToPose, navigate_topic)
        self.path_client = ActionClient(self, ComputePathToPose, compute_path_topic)

        use_sim_time = self.get_parameter('use_sim_time').get_parameter_value().bool_value
        self.get_logger().info(f'use_sim_time = {use_sim_time}')
    
    def is_goal_reachable(self, x, y, yaw=0.0):
        
        self.path_client.wait_for_server()

        goal = ComputePathToPose.Goal()
        start = PoseStamped()
        start.header.frame_id = 'map'
        start.header.stamp = self.get_clock().now().to_msg()
        start.pose.orientation.w = 1.0
        goal.start = start

        goal.goal = PoseStamped()
        goal.goal.header.frame_id = 'map'
        goal.goal.pose.position.x = x
        goal.goal.pose.position.y = y
        goal.goal.pose.orientation.z = np.sin(yaw/2)
        goal.goal.pose.orientation.w = np.cos(yaw/2)

        future = self.path_client.send_goal_async(goal)
        rclpy.spin_until_future_complete(self, future)
        goal_handle = future.result()

        if not goal_handle.accepted:
            self.get_logger().warn("Path planning request rejected")
            return False
        result_future = goal_handle.get_result_async()
        rclpy.spin_until_future_complete(self, result_future)
        result = result_future.result().result
        

        if len(result.path.poses) > 0:
            return True
        else:
            return False

    def send_goal(self, x, y, yaw=0.0):
        # Check if the goal is valid before sending it to nav2
        if  not self.is_goal_reachable(x, y):
            return False

        # Wait for planner server
        self._action_client.wait_for_server()


        goal_msg = NavigateToPose.Goal()
        goal_msg.pose.header.frame_id = 'map'
        goal_msg.pose.header.stamp = self.get_clock().now().to_msg()
        goal_msg.pose.pose.position.x = x
        goal_msg.pose.pose.position.y = y
        goal_msg.pose.pose.orientation.z = math.sin(yaw / 2.0)
        goal_msg.pose.pose.orientation.w = math.cos(yaw / 2.0)
        


        send_goal_future = self._action_client.send_goal_async(goal_msg)
        rclpy.spin_until_future_complete(self, send_goal_future)
        goal_handle = send_goal_future.result()

        if not goal_handle.accepted:
            self.get_logger().warn('Goal rejected by Nav2!')
            return False
        
        self.get_logger().info(f'Sending goal: x={x:.2f}, y={y:.2f}')
        self.get_logger().info("Goal seems reachable.")

        self.get_logger().info('Goal accepted, waiting for result...')
        result_future = goal_handle.get_result_async()
        rclpy.spin_until_future_complete(self, result_future)
        result = result_future.result().result
        self.get_logger().info('Goal reached successfully!')


        return True

    def execute_cpp_route(self):
        route, cost, G = chinese_postman_from_yaml(self.yaml_path, start_node=self.start_node)
        route.reverse()
        self.get_logger().info(f'CPP route: {route}, total cost: {cost:.2f}')
        for node_id in route:
            node_data = G.nodes[node_id]
            x, y = node_data.get('x'), node_data.get('y')
            if x is None or y is None:
                self.get_logger().warn(f'Node {node_id} missing coordinates!')
                continue

            if not self.is_goal_reachable(x, y):
                self.get_logger().warn("The goal is unreachable passing to the next node")
                continue

            success = self.send_goal(x, y)
            if not success:
                self.get_logger().warn(f'Failed to reach node {node_id}')
            time.sleep(1.0)  # short pause between goals
            

def main(args=None):
    rclpy.init(args=args)
    node = CPPNavExecutor()
    
    if not node.yaml_path:
        node.get_logger().error('graph_yaml_path parameter is required!')
        node.destroy_node()
        rclpy.shutdown()
        return
    
    node.execute_cpp_route()
    node.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()
