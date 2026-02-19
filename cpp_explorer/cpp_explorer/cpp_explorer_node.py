#!/usr/bin/env python3
import rclpy
from rclpy.action import ActionClient
from rclpy.node import Node
from nav2_msgs.action import NavigateToPose
from geometry_msgs.msg import PoseStamped, TransformStamped
import yaml
import networkx as nx
import math
import time
import tf2_ros
import tf_transformations
import numpy as np
from geometry_msgs.msg import PoseWithCovarianceStamped


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
    def __init__(self, yaml_path, start_node=1):
        super().__init__('cpp_nav_executor')
        self._action_client = ActionClient(self, NavigateToPose, 'navigate_to_pose')
        self.yaml_path = yaml_path
        self.start_node = start_node

        use_sim_time = self.get_parameter('use_sim_time').get_parameter_value().bool_value
        self.get_logger().info(f'use_sim_time = {use_sim_time}')


    def send_goal(self, x, y, yaw=0.0):
        # Wait for planner server
        self._action_client.wait_for_server()
        # cov_msg = PoseWithCovarianceStamped()


        goal_msg = NavigateToPose.Goal()
        goal_msg.pose.header.frame_id = 'map'
        goal_msg.pose.header.stamp = self.get_clock().now().to_msg()
        goal_msg.pose.pose.position.x = x
        goal_msg.pose.pose.position.y = y
        goal_msg.pose.pose.orientation.z = math.sin(yaw / 2.0)
        goal_msg.pose.pose.orientation.w = math.cos(yaw / 2.0)
        
        # cov = np.diag([0.01, 0.01, 0.0, 0.0, 0.0, 0.05]).flatten()
        # cov_msg.pose.covariance = cov.tolist()


        self.get_logger().info(f'Sending goal: x={x:.2f}, y={y:.2f}')
        send_goal_future = self._action_client.send_goal_async(goal_msg)
        rclpy.spin_until_future_complete(self, send_goal_future)
        goal_handle = send_goal_future.result()

        if not goal_handle.accepted:
            self.get_logger().warn('Goal rejected by Nav2!')
            return False

        self.get_logger().info('Goal accepted, waiting for result...')
        result_future = goal_handle.get_result_async()
        rclpy.spin_until_future_complete(self, result_future)
        result = result_future.result().result
        self.get_logger().info('Goal reached successfully!')

        # self.cov.publish(cov_msg)
        # self.get_logger().info("Published pose with covariance")
        return True

    def execute_cpp_route(self):
        route, cost, G = chinese_postman_from_yaml(self.yaml_path, start_node=self.start_node)
        route.reverse()
        self.get_logger().info(f'CPP route: {route}, total cost: {cost:.2f}')
        for node_id in route:
            node_data = G.nodes[node_id]
            x, y = node_data['x'], node_data['y']
            if x is None or y is None:
                self.get_logger().warn(f'Node {node_id} missing coordinates!')
                continue
            success = self.send_goal(x, y)
            if not success:
                self.get_logger().warn(f'Failed to reach node {node_id}')
            time.sleep(1.0)  # short pause between goals


def main(args=None):
    rclpy.init(args=args)
    yaml_path = "/home/aymen/ros2_ws/src/cpp_explorer/config/graph.yaml"  # path to your YAML graph
    node = CPPNavExecutor(yaml_path, start_node=1)
    node.execute_cpp_route()
    node.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()
