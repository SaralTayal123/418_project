# The only external library required...
import cv2
import numpy as np

# The rest are standard libraries
import argparse
import os

# Global constants
IMG_WIDTH = 4000
IMG_HEIGHT = 4000
DIM_X = 1
DIM_Y = 1
NODE_WIDTH = 10
PATH_NODE_WIDTH = 15
START_GOAL_WIDTH = 20
LINE_WIDTH = 2
PATH_WIDTH = 5

def map_coord(x, y):
    return int(IMG_WIDTH * x / DIM_X), int(IMG_HEIGHT * y / DIM_Y)


if __name__ == "__main__":
    # Parse arguments from cmdline for input, nodes, and path files
    parser = argparse.ArgumentParser()
    parser.add_argument("-i", "--input", help="Input map file", required=True)
    parser.add_argument("-n", "--nodes", help="Output nodes file", required=True)
    parser.add_argument("-p", "--path", help="Output path file", required=True)

    args = parser.parse_args()
    input_file = args.input
    nodes_file = args.nodes
    path_file = args.path

    # Create a blank image
    img = np.ones((IMG_HEIGHT, IMG_WIDTH, 3), np.uint8) * np.array([237, 239, 240][::-1], dtype=np.uint8)

    # Load in map file, read line-by-line
    with open(input_file, "r") as f:
        DIM_X, DIM_Y = [int(x) for x in f.readline().split()]
        next(f) # Skip the next line
        start_x, start_y = [int(x) for x in f.readline().split()]
        goal_x, goal_y = [int(x) for x in f.readline().split()]
        next(f)
        for line in f:
            # Split line into x1, y1, x2, y2
            x1, y1, x2, y2 = line.split()
            # Convert to ints
            x1 = int(x1)
            y1 = int(y1)
            x2 = int(x2)
            y2 = int(y2)

            # Map to image space
            x1, y1 = map_coord(x1, y1)
            x2, y2 = map_coord(x2, y2)

            # Draw rectangle
            cv2.rectangle(img, (x1, y1), (x2, y2), [185, 192, 198][::-1], cv2.FILLED)

    # Load in nodes file, read line-by-line. Put into a list
    with open(nodes_file, "r") as f:
        next(f) # Skip the first line
        nodes = [tuple([int(x) for x in line.split()]) for line in f]

    # Draw the lines between the nodes
    for node in nodes[1:]:
        x, y, parent_index = node
        x, y = map_coord(x, y)
        xp, yp = map_coord(nodes[parent_index][0], nodes[parent_index][1])
        cv2.line(img, (x, y), (xp, yp), [103, 122, 143][::-1], LINE_WIDTH)

    # Draw the node circles
    for node in nodes:
        x, y, _ = node
        x, y = map_coord(x, y)
        cv2.circle(img, (x, y), NODE_WIDTH, [67, 120, 167][::-1], cv2.FILLED)

    # Load in path file
    with open(path_file, "r") as f:
        next(f) # Skip the first line
        next(f) # Skip the second line
        path = [tuple([int(x) for x in line.split()]) for line in f]

    # Draw the lines between the nodes
    for i in range(len(path) - 1):
        x, y, _ = path[i]
        x, y = map_coord(x, y)
        xp, yp = map_coord(path[i+1][0], path[i+1][1])
        cv2.line(img, (x, y), (xp, yp), [132, 141, 195][::-1], PATH_WIDTH)

    # Draw the path nodes
    for node in path:
        x, y, _ = node
        x, y = map_coord(x, y)
        cv2.circle(img, (x, y), PATH_NODE_WIDTH, [227, 151, 175][::-1], cv2.FILLED)

    # Draw the start and goal nodes
    start_x, start_y = map_coord(start_x, start_y)
    goal_x, goal_y = map_coord(goal_x, goal_y)
    cv2.circle(img, (start_x, start_y), START_GOAL_WIDTH, [108, 177, 116][::-1], cv2.FILLED)
    cv2.circle(img, (goal_x, goal_y), START_GOAL_WIDTH, [209, 92, 92][::-1], cv2.FILLED)


    # Save image
    cv2.imwrite(os.path.join("images", os.path.basename(input_file).replace("txt", "png")), img)