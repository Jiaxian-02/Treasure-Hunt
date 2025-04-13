# -*- coding: utf-8 -*-
"""
Created on Wed Jul 10 22:14:19 2024

@author: Group 2
"""

import tkinter as tk  # Import the tkinter library for GUI applications
import heapq  # Import the heapq library for implementing priority queues
import math  # Import the math library for mathematical operations


# Define a Node class to represent each position in the grid
class Node:
    def __init__(self, position, parent=None):
        self.position = position  # The position of the node in the grid (tuple)
        self.parent = parent  # The parent node from which this node is reached
        self.g = 0  # Cost from the start node to this node
        self.h = 0  # Heuristic cost from this node to the goal
        self.f = 0  # Total cost (g + h)


# Define a PriorityQueue class to manage the frontier in the A* algorithm
class PriorityQueue:
    def __init__(self):
        self.elements = []  # Initialize an empty list to store the elements

    def is_empty(self):
        return not self.elements  # Check if the priority queue is empty

    def put(self, item, priority):
        heapq.heappush(self.elements, (priority, item))  # Push an item onto the queue with a given priority

    def get(self):
        return heapq.heappop(self.elements)[1]  # Pop the item with the lowest priority from the queue


# Define a GridWithWeights class to represent the grid world
class GridWithWeights:
    def __init__(self, width, height):
        self.width = width  # The width of the grid
        self.height = height  # The height of the grid
        self.walls = []  # List to store the positions of walls in the grid
        self.weights = {}  # Dictionary to store weights for specific cells

    def in_bounds(self, id):
        (y, x) = id  # Unpack the id into y and x coordinates
        return 0 <= x < self.width and 0 <= y < self.height  # Check if the coordinates are within grid bounds

    def passable(self, id):
        return id not in self.walls  # Check if the cell is not a wall

    def neighbors(self, id):
        (y, x) = id  # Unpack the id into y and x coordinates
        # Determine the neighboring cells based on the current x coordinate (for hexagonal movement)
        if x % 2 == 0:
            results = [(y - 1, x), (y + 1, x), (y, x - 1), (y, x + 1), (y + 1, x - 1), (y + 1, x + 1)]
        else:
            results = [(y - 1, x), (y + 1, x), (y, x - 1), (y, x + 1), (y - 1, x - 1), (y - 1, x + 1)]
        results = filter(self.in_bounds, results)  # Filter out neighbors that are out of bounds
        results = filter(self.passable, results)  # Filter out neighbors that are walls
        return results  # Return the list of valid neighboring cells

    def cost(self, from_node, to_node, traps, rewards):
        base_cost = 1  # Base cost of moving to an adjacent cell
        if to_node in traps:  # Check if the target cell is a trap
            trap_effect = traps[to_node]
            if trap_effect == 'trap1':
                base_cost *= 2  # Double the cost if it's trap1
            elif trap_effect == 'trap2':
                base_cost *= 2  # Double the cost if it's trap2
            elif trap_effect == 'trap3':
                base_cost += 2  # Moves two cells away (add two)
            elif trap_effect == 'trap4':
                base_cost = float('inf')  # Set cost to infinity if it's trap4 (removes all treasures)
        if to_node in rewards:  # Check if the target cell is a reward
            reward_effect = rewards[to_node]
            if reward_effect == 'reward1':
                base_cost /= 2  # Halve the cost if it's reward1
            elif reward_effect == 'reward2':
                base_cost /= 2  # Halve the cost if it's reward2
        return base_cost  # Return the calculated cost


# Define the heuristic function using the Euclidean Distance
def heuristic(current, goal):
    return math.sqrt((current[0] - goal[0])**2 + (current[1] - goal[1])**2)  # Calculate Euclidean distance


# Define the A* search algorithm to find the path to collect all treasures
def a_star_search(graph, start, treasures, traps, rewards):
    all_treasures = set(treasures)  # Convert the list of treasures to a set for easy manipulation
    current_position = start  # Start from the initial position
    path = []  # Initialize an empty list to store the path
    cost_so_far = {start: 0}  # Dictionary to store the cost to reach each node

    while all_treasures:  # Loop until all treasures are collected
        came_from, new_cost_so_far, reachable_treasures = single_target_a_star_search(graph, current_position,
                                                                                      all_treasures, traps, rewards,
                                                                                      cost_so_far)
        if not reachable_treasures:
            print("Some treasures are unreachable.")  # Print a message if some treasures are unreachable
            break

        next_treasure = min(reachable_treasures, key=lambda t: new_cost_so_far.get(t, float('inf')))
        if new_cost_so_far.get(next_treasure, float('inf')) == float('inf'):
            break

        path_segment = reconstruct_path(came_from, current_position, next_treasure)  # Reconstruct the path to the next treasure
        path.extend(path_segment[1:])  # Extend the path with the new segment (excluding the starting point)
        current_position = next_treasure  # Update the current position to the treasure position
        all_treasures.remove(next_treasure)  # Remove the collected treasure from the set
        cost_so_far = new_cost_so_far  # Update the cost_so_far dictionary

    return path, cost_so_far  # Return the complete path and the cost to reach each node


# Define the A* search algorithm to find the path to a single treasure
def single_target_a_star_search(graph, start, goals, traps, rewards, cost_so_far):
    frontier = PriorityQueue()  # Initialize the priority queue for the frontier
    frontier.put(start, 0)  # Add the start node to the frontier with priority 0
    came_from = {}  # Dictionary to store the parent of each node
    came_from[start] = None  # The start node has no parent
    cost_so_far = {start: cost_so_far[start]}  # Initialize cost_so_far with the starting cost
    visited = set()  # Set to keep track of visited nodes

    while not frontier.is_empty():  # Loop until the frontier is empty
        current = frontier.get()  # Get the node with the lowest priority from the frontier
        visited.add(current)  # Add the current node to the visited set

        if current in goals:  # Check if the current node is one of the goals
            return came_from, cost_so_far, goals  # Return the path information and goals

        for next in graph.neighbors(current):  # Loop through the neighbors of the current node
            new_cost = cost_so_far[current] + graph.cost(current, next, traps, rewards)  # Calculate the cost to reach the neighbor
            if next not in cost_so_far or new_cost < cost_so_far[next]:  # Check if the neighbor is unvisited or found a cheaper path
                cost_so_far[next] = new_cost  # Update the cost to reach the neighbor
                priority = new_cost + min(heuristic(next, goal) for goal in goals)  # Calculate the priority using heuristic
                frontier.put(next, priority)  # Add the neighbor to the frontier with the calculated priority
                came_from[next] = current  # Set the parent of the neighbor to the current node

    reachable_treasures = set()  # Initialize an empty set for reachable treasures
    return came_from, cost_so_far, reachable_treasures  # Return the path information and empty reachable treasures


# Reconstruct the path from start to goal using the came_from dictionary
def reconstruct_path(came_from, start, goal):
    current = goal  # Start from the goal node
    path = []  # Initialize an empty list to store the path
    while current != start:  # Loop until the start node is reached
        path.append(current)  # Add the current node to the path
        current = came_from[current]  # Move to the parent node
    path.append(start)  # Add the start node to the path
    path.reverse()  # Reverse the path to get the correct order
    return path  # Return the reconstructed path


# Function to calculate hexagon points for visualization purposes
def hexagon_points(center_x, center_y, size):
    points = []  # Initialize an empty list to store the points
    for i in range(6):  # Loop through the 6 vertices of the hexagon
        angle = math.radians(60 * i)  # Calculate the angle for the current vertex
        x_i = center_x + size * math.cos(angle)  # Calculate the x coordinate of the vertex
        y_i = center_y + size * math.sin(angle)  # Calculate the y coordinate of the vertex
        points.append((x_i, y_i))  # Add the vertex to the points list
    return points  # Return the list of points


# Define the Tkinter application class for visualizing the grid and A* search
class App(tk.Tk):
    trap_symbols = {'trap1': '⊖', 'trap2': '⊕', 'trap3': '⊗', 'trap4': '⊘'}  # Dictionary of symbols for traps
    reward_symbols = {'reward1': '⊞', 'reward2': '⊠'}  # Dictionary of symbols for rewards
    symbol_font = ('Arial', 22, 'bold')  # Font settings for symbols

    def __init__(self, grid, start, traps, rewards, treasures):
        super().__init__()  # Initialize the Tkinter application
        self.title("A* Pathfinding Visualizer")  # Set the window title
        self.canvas = tk.Canvas(self, width=500, height=400, bg='pink')  # Create a canvas for drawing the grid
        self.canvas.pack()  # Pack the canvas into the window
        self.grid = grid  # Store the grid object
        self.start = start  # Store the start position
        self.traps = traps  # Store the traps dictionary
        self.rewards = rewards  # Store the rewards dictionary
        self.treasures = treasures  # Store the treasures list
        self.cell_size = 30  # Set the size of each cell in the grid
        self.draw_grid()  # Draw the initial grid
        self.path = []  # Initialize an empty list to store the path
        self.cost_so_far = 0  # Initialize the total cost
        self.current_step = 0  # Initialize the current step counter
        self.run_search()  # Run the A* search algorithm

    def draw_grid(self):
        for x in range(self.grid.width):  # Loop through each column
            for y in range(self.grid.height):  # Loop through each row
                center_x = x * self.cell_size * 1.5 + 50  # Calculate the x coordinate of the cell center
                center_y = y * self.cell_size * math.sqrt(3) + 50  # Calculate the y coordinate of the cell center
                if x % 2 == 0:  # Adjust the y coordinate for even columns (hexagonal layout)
                    center_y += self.cell_size * math.sqrt(3) / 2
                points = hexagon_points(center_x, center_y, self.cell_size)  # Calculate the hexagon points for the cell
                fill_color = 'white'  # Default fill color for cells
                label = ''  # Default label for cells
                if (y, x) in self.grid.walls:  # Check if the cell is a wall
                    fill_color = 'gray'
                elif (y, x) == self.start:  # Check if the cell is the start position
                    fill_color = 'blue'
                elif (y, x) in self.traps:  # Check if the cell is a trap
                    trap_type = self.traps[(y, x)]
                    fill_color = '#E0BBE4'
                    label = self.trap_symbols[trap_type]
                elif (y, x) in self.rewards:  # Check if the cell is a reward
                    reward_type = self.rewards[(y, x)]
                    fill_color = '#03C03C'
                    label = self.reward_symbols[reward_type]
                elif (y, x) in self.treasures:  # Check if the cell is a treasure
                    fill_color = 'gold'
                self.canvas.create_polygon(points, fill=fill_color, outline='black')  # Draw the hexagon cell
                if label:  # Draw the label if it exists
                    self.canvas.create_text(center_x, center_y, text=label, fill='black', font=self.symbol_font)

    def run_search(self):
        self.path, self.cost_so_far = a_star_search(self.grid, self.start, self.treasures, self.traps, self.rewards)  # Run the A* search algorithm
        self.current_step = 0  # Reset the current step counter
        self.step_through_path()  # Start stepping through the path

    def step_through_path(self):
        if self.current_step < len(self.path):  # Check if there are more steps to take
            (y, x) = self.path[self.current_step]  # Get the current position from the path
            center_x = x * self.cell_size * 1.5 + 50  # Calculate the x coordinate of the cell center
            center_y = y * self.cell_size * math.sqrt(3) + 50  # Calculate the y coordinate of the cell center
            if x % 2 == 0:  # Adjust the y coordinate for even columns (hexagonal layout)
                center_y += self.cell_size * math.sqrt(3) / 2
            points = hexagon_points(center_x, center_y, self.cell_size)  # Calculate the hexagon points for the cell
            self.canvas.create_polygon(points, fill='yellow', outline='black')  # Draw the current cell in yellow

            # Draw a distinct marker for treasures
            if (y, x) in self.treasures:
                self.canvas.create_text(center_x, center_y, text='💰', fill='black', font=('Arial', 22, 'bold'))

            # Draw a distinct marker for rewards
            if (y, x) in self.rewards:
                reward_type = self.rewards[(y, x)]
                label = self.reward_symbols[reward_type]
                self.canvas.create_text(center_x, center_y, text=label, fill='black', font=self.symbol_font)

            self.current_step += 1  # Increment the step counter
            self.after(500, self.step_through_path)  # Call this method again after 500ms
        else:
            print("\nAlgorithm has finished. You may now close the tab to view the path (in coordinates form) and total hops taken.\n")


# Main function to set up the grid, traps, rewards, and start the Tkinter app
def main():
    width, height = 10, 6  # Set the width and height of the grid
    grid = GridWithWeights(width, height)  # Create the grid object

    start = (0, 0)  # Set the initial state

    # Set the positions of walls, treasures, traps, and rewards based on instructions
    grid.walls = [(1, 8), (2, 2), (2, 4), (3, 0), (3, 3), (3, 6), (4, 4), (4, 6), (4, 7)]
    treasures = [(1, 4), (3, 7), (3, 9), (4, 3)]

    traps = {
        (2, 8): 'trap1',
        (1, 1): 'trap2', (4, 2): 'trap2',
        (1, 6): 'trap3', (3, 5): 'trap3',
        (1, 3): 'trap4'
    }

    rewards = {
        (0, 4): 'reward1', (3, 1): 'reward1',
        (2, 7): 'reward2', (5, 5): 'reward2'
    }

    print("---------------------------------------------------------------------------------------")
    print("Treasure Hunt in the Virtual World using A* Search")
    print("---------------------------------------------------------------------------------------")
    print("Objective: Collect all treasures on the treasure hunt map using the shortest possible path.")
    print("NOTE: Please keep the visualization tab open until the algorithm has finished finding the path.")
    app = App(grid, start, traps, rewards, treasures)  # Create the Tkinter app
    app.mainloop()  # Start the Tkinter event loop

    # After the app has run its search and finished, print the path
    print("Path found by A* search:")
    print(app.path)
    # Print the total hops taken
    total_hops = app.current_step
    print("Total hops:", total_hops)


if __name__ == '__main__':
    main()  # Execute the main function