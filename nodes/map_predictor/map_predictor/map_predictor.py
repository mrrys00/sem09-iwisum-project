import os
import json
from datetime import datetime
import numpy as np
from sklearn.cluster import DBSCAN
import matplotlib.pyplot as plt

import rclpy
from rclpy.node import Node
from nav_msgs.msg import OccupancyGrid

ESTIMATED_COLOR = 200

def estimate_rectangles(grid):
    walls = np.column_stack(np.where(grid == 100))

    # Clustering DBSCAN
    clustering = DBSCAN(eps=5, min_samples=5).fit(walls)
    clusters = clustering.labels_

    for cluster_id in set(clusters):
        if cluster_id == -1:  # Omit noise
            continue

        cluster_points = walls[clusters == cluster_id]

        # Check if the cluster is large enough
        if len(cluster_points) < 5:  # Filter small clusters (less than 5 points)
            continue

        # Calculate bounding rectangle
        min_y, min_x = np.min(cluster_points, axis=0)
        max_y, max_x = np.max(cluster_points, axis=0)

        # Draw a rectangle on the map

        # Draw the left edge
        for y in range(min_y, max_y + 1):
            grid[y, min_x] = ESTIMATED_COLOR

        # Draw the right edge
        for y in range(min_y, max_y + 1):
            grid[y, max_x] = ESTIMATED_COLOR

        # Draw the top edge
        for x in range(min_x, max_x + 1):
            grid[min_y, x] = ESTIMATED_COLOR

        # Draw the bottom edge
        for x in range(min_x, max_x + 1):
            grid[max_y, x] = ESTIMATED_COLOR

    return grid

# def visualize_map(grid, estimated_grid):
#     height, width = grid.shape

#     colormap = {
#         -1: [0.5, 0.5, 0.5],  # Gray (unknown terrain)
#         0: [1.0, 1.0, 1.0],  # White (blank space)
#         100: [0.0, 0.0, 0.0],  # Black (walls)
#         ESTIMATED_COLOR: [0.0, 1.0, 0.0]  # Green (estimated walls)
#     }

#     colored_map = np.zeros((height, width, 3))
#     for y in range(height):
#         for x in range(width):
#             color_code = grid[y, x]
#             colored_map[y, x] = colormap[color_code]

#     colored_estimated_map = np.zeros((height, width, 3))
#     for y in range(height):
#         for x in range(width):
#             color_code = estimated_grid[y, x]
#             colored_estimated_map[y, x] = colormap[color_code]

#     plt.figure(figsize=(12, 6))
#     plt.subplot(1, 2, 1)
#     plt.imshow(colored_map, origin='upper')
#     plt.title("Original map")
#     plt.axis('off')

#     plt.subplot(1, 2, 2)
#     plt.imshow(colored_estimated_map, origin='upper')
#     plt.title("Map with estimated obstacles")
#     plt.axis('off')

#     plt.show()

class MapProcessorNode(Node):
    def __init__(self):
        super().__init__('map_processor_node')

        # Subscribe to the /map topic
        self.subscription = self.create_subscription(
            OccupancyGrid,
            '/map',
            self.map_callback,
            10
        )

        self.get_logger().info("Subscribed to /map topic. Processing map data.")

    def map_callback(self, msg):
        try:
            width = msg.info.width
            height = msg.info.height
            data = np.array(msg.data).reshape((height, width))

            self.get_logger().info("Received map data. Processing...")

            # Process the map data
            estimated_grid = estimate_rectangles(data.copy())
            # visualize_map(data, estimated_grid)

            # Overwrite the map (this is a placeholder, replace with actual implementation for your environment)
            self.publish_modified_map(msg, estimated_grid)

        except Exception as e:
            self.get_logger().error(f"Error processing map data: {e}")

    def publish_modified_map(self, original_msg, modified_grid):
        # Update the data in the original message
        modified_msg = OccupancyGrid()
        modified_msg.header = original_msg.header
        modified_msg.info = original_msg.info
        modified_msg.data = modified_grid.flatten().tolist()

        # Publish the modified map
        self.map_publisher.publish(modified_msg)
        # TODO: Publish the modified map to the appropriate topic or service
        # https://docs.ros.org/en/humble/p/nav2_map_server/ - at the bottom
        self.get_logger().info("Modified map ready for publishing (implement publishing logic).")

def main(args=None):
    rclpy.init(args=args)

    try:
        node = MapProcessorNode()
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()
