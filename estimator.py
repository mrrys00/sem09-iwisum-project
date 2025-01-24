import json
import os
import numpy as np
import matplotlib.pyplot as plt
from sklearn.cluster import DBSCAN

ESTIMATED_COLOR = 200
json_maps_path = "example_maps_v2"
results_path = os.path.abspath(os.getcwd()) + "/results/"

figures_nums = []
greatest_circumferencies = []
total_circumferencies = []
accuracies = []

if not os.path.exists(results_path):
    os.makedirs(results_path)

def estimate_rectangles(grid):
    walls = np.column_stack(np.where(grid == 100))

    # Clustering DBSCAN
    clustering = DBSCAN(eps=5, min_samples=5).fit(walls)
    clusters = clustering.labels_

    num_estimated_figures = 0
    greatest_circumference = 0
    total_circumference = 0

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

        # Calculate circumference
        width = max_x - min_x + 1
        height = max_y - min_y + 1
        circumference = 2 * (width + height)

        # Update statistics
        num_estimated_figures += 1
        total_circumference += circumference
        greatest_circumference = max(greatest_circumference, circumference)

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

    # Log results
    print(f"Number of estimated figures: {num_estimated_figures}")
    print(f"Circumference of the greatest figure: {greatest_circumference}")
    print(f"Total circumference of all estimated figures: {total_circumference}")
    accuracy = total_circumference/(num_estimated_figures*greatest_circumference)
    print(f"Accuracy: {accuracy*100}%")
    
    # Save metricies
    figures_nums.append(num_estimated_figures)
    greatest_circumferencies.append(greatest_circumference)
    total_circumferencies.append(total_circumference)
    accuracies.append(round(accuracy, 5))

    return grid

def visualize_map(json_data:dict, filename:str="", show:bool=False):
    width = json_data['info']['width']
    height = json_data['info']['height']
    data = json_data['data']

    grid = np.array(data).reshape((height, width))
    estimated_grid = estimate_rectangles(grid.copy())
            
    colormap = {
        -1: [0.5, 0.5, 0.5],  # Gray (unknown terrain)
        0: [1.0, 1.0, 1.0],  # White (blank space)
        100: [0.0, 0.0, 0.0],  # Black (walls)
        ESTIMATED_COLOR: [0.0, 1.0, 0.0]  # Green (estimated walls)
    }

    colored_map = np.zeros((height, width, 3))
    for y in range(height):
        for x in range(width):
            color_code = grid[y, x]
            colored_map[y, x] = colormap[color_code]

    colored_estimated_map = np.zeros((height, width, 3))
    for y in range(height):
        for x in range(width):
            color_code = estimated_grid[y, x]
            colored_estimated_map[y, x] = colormap[color_code]

    plt.figure(figsize=(12, 6))
    plt.subplot(1, 2, 1)
    plt.imshow(colored_map, origin='upper')
    plt.title("Original map")
    plt.axis('off')
    
    num_estimated_figures = figures_nums[-1]
    accuracy = accuracies[-1]
    metrics = f'accuracy: {accuracy*100}%\nfig numb: {num_estimated_figures}'
    plt.text(
        -50, 0,
        metrics,
        fontsize = 12, 
        bbox = dict(facecolor = 'red', alpha = 0.5))
    
    plt.subplot(1, 2, 2)
    plt.imshow(colored_estimated_map, origin='upper')
    plt.title("Map with estimated obstacles")
    plt.axis('off')

    if filename:
        plt.savefig(results_path + filename)
    if show:
        plt.show()
    
    plt.close()

def generate_plot(filename:str="result_plot", show:bool=False):
    plt.figure(figsize=(12, 12))
    plt.xlabel("epoch")
    plt.axis('off')

    plt.subplot(3,1,1)
    plt.ylabel("accuracy")
    plt.plot([i for i in range(len(accuracies))], accuracies, 'tab:green', label="accuracy")

    plt.subplot(3,1,2)
    plt.ylabel("number of obstacles")
    plt.plot([i for i in range(len(figures_nums))], figures_nums, 'tab:orange', label="number of obstacles")

    plt.subplot(3,1,3)
    plt.ylabel("max obstacle circ.")
    plt.plot([i for i in range(len(greatest_circumferencies))], greatest_circumferencies, 'tab:blue', label="max obstacle circ.")

    plt.savefig(results_path + filename)

    if show:
        plt.show()
    
    plt.close()

def main():
    for filename in sorted(os.listdir(json_maps_path)):
        if filename.endswith(".json"):
            file_path = os.path.join(json_maps_path, filename)
            print(f"Computing a file: {file_path}")

            try:
                with open(file_path, 'r') as file:
                    json_data = json.load(file)

                visualize_map(json_data, filename.split('.')[0], show=False)

            except FileNotFoundError:
                print(f"File: '{file_path}' not found.")

            except json.JSONDecodeError:
                print(f"File: '{file_path}' not correct JSON file.")
                
    generate_plot()

if __name__ == '__main__':
    main()
