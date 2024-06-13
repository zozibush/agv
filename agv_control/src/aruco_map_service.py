#!/usr/bin/env python3

import rospy
import yaml
import json
import networkx as nx
import matplotlib.pyplot as plt
from agv_control.srv import JsonPath, JsonPathResponse
import sys

class ArucoMap:
    def __init__(self, file_path):
        self.G = nx.DiGraph()
        self.file_path = file_path

        self.load_map()
        self.visualize_map()

    def load_map(self):
        with open(self.file_path, 'r') as file:
            graph_data = yaml.safe_load(file)

        self.pos = {}
        for node in graph_data['nodes']:
            self.G.add_node(node['id'])
            self.pos[node['id']] = (node['pos'][0], node['pos'][1])
            for edge in node.get('edges', []):
                self.add_edge_with_attributes(node['id'], edge['target'], edge['direction'], edge['lane_color'], edge.get('arrive', None))

    def visualize_map(self):
        plt.figure(figsize=(8, 6))
        edge_labels = nx.get_edge_attributes(self.G, 'direction')
        nx.draw(self.G, self.pos, with_labels=True, node_size=300, node_color="skyblue", font_size=10, font_weight="bold", arrowsize=10)
        nx.draw_networkx_edge_labels(self.G, self.pos, edge_labels=edge_labels, font_color='red', font_size=10)
        plt.title("Directed Graph Visualization with Attributes from YAML")
        plt.show()

    def service_start(self):
        s = rospy.Service('aruco_map_service', JsonPath, self.handle_service_request)
        rospy.loginfo("Service server is ready.")
        rospy.spin()

    def handle_service_request(self, req):
        start_node = req.start_node
        end_node = req.end_node
        path = self.get_shortest_path(start_node, end_node)

        # JSON 응답 생성
        response_json = json.dumps(path)
        return JsonPathResponse(json_path=response_json)

    def add_node(self):
        for i in range(1, 33):
            self.G.add_node(i)

    def add_edge_with_attributes(self, u, v, direction, lane_color, arrive=None):
        self.G.add_edge(u, v, lane_color=lane_color, direction=direction, arrive=arrive)

    def get_shortest_path(self, start_node, end_node):
        path = {}
        shortest_path = nx.dijkstra_path(self.G, start_node, end_node)
        path_edges = list(zip(shortest_path, shortest_path[1:]))
        for u, v in path_edges:
            edge_data = self.G.get_edge_data(u, v)
            path[u] = [v, edge_data['lane_color'], edge_data['direction'], edge_data['arrive']]

        return path

if __name__ == '__main__':
    rospy.init_node('aruco_map_service', anonymous=True)
    if len(sys.argv) > 1:
        file_path = sys.argv[1]
    else:
        file_path = 'aruco_map.yaml'
    arucoMap = ArucoMap(file_path)
    arucoMap.service_start()
