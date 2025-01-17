import json
import os

import matplotlib.pyplot as plt
import numpy as np


def visualize_map(json_data):
    width = json_data['info']['width']
    height = json_data['info']['height']
    data = json_data['data']

    grid = np.array(data).reshape((height, width))

    colormap = {
        -1: [0.5, 0.5, 0.5],
        0: [1.0, 1.0, 1.0],
        100: [0.0, 0.0, 0.0]
    }

    rgb_grid = np.zeros((height, width, 3))
    for key, color in colormap.items():
        rgb_grid[grid == key] = color

    plt.figure(figsize=(10, 10))
    plt.imshow(rgb_grid, origin='upper')
    plt.title("Mapa z JSON")
    plt.axis('off')
    plt.show()


if __name__ == "__main__":
    folder_path = "example_maps"

    for filename in os.listdir(folder_path):
        if filename.endswith(".json"):
            file_path = os.path.join(folder_path, filename)
            print(f"Przetwarzanie pliku: {file_path}")

            try:
                with open(file_path, 'r') as file:
                    json_data = json.load(file)

                visualize_map(json_data)
            except FileNotFoundError:
                print(f"Plik o ścieżce '{file_path}' nie został znaleziony.")
            except json.JSONDecodeError:
                print(f"Plik '{file_path}' nie jest poprawnym plikiem JSON.")
