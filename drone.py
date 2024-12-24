import pygame
import random
import math

# Define colors
WHITE = (255, 255, 255)
BLACK = (0, 0, 0)
GOAL_COLOR = (255, 165, 0)  # Orange color for the goal

# Define cell size and margin
CELL_SIZE = 15
MARGIN = 1

# Parameters for flocking behavior
COHESION_RADIUS = 60
ALIGNMENT_RADIUS = 50
SEPARATION_RADIUS = 30
SEPARATION_FORCE = 1

# Mid-level control parameters
SPLIT_THRESHOLD = 100  # Number of iterations before splitting
COMMUNICATION_RANGE = 100  # Communication range for informing other drones

class Drone:
    def __init__(self, id, maze, goal_position, all_drones=None):
        self.id = id
        self.position = (random.randint(0, len(maze) - 1), random.randint(0, len(maze[0]) - 1))  # Start position
        self.maze = maze
        self.visited = set() 
        self.path = [self.position]
        self.goal_position = goal_position
        self.color = (random.randint(0, 255), random.randint(0, 255), random.randint(0, 255))
        self.all_drones = all_drones if all_drones is not None else []
        self.goal_found = False  # Variable to track if goal is found by this drone
        self.iteration_count = 0  # Counter for iterations since the goal was last found
        self.subgroup = 0  # Subgroup identifier

    def move(self):
        x, y = self.position
        neighbors = [drone for drone in self.all_drones if drone != self and self.distance_to(drone) <= COHESION_RADIUS]

        # Cohesion: move towards the average position of neighbors
        if neighbors:
            avg_x = sum(neighbor.position[0] for neighbor in neighbors) / len(neighbors)
            avg_y = sum(neighbor.position[1] for neighbor in neighbors) / len(neighbors)
            target_x = int(avg_x)
            target_y = int(avg_y)
        else:
            target_x, target_y = self.goal_position

        # Alignment: adjust velocity to match neighbors
        alignment_neighbors = [drone for drone in neighbors if self.distance_to(drone) <= ALIGNMENT_RADIUS]
        if alignment_neighbors:
            avg_velocity_x = sum(drone.position[0] - x for drone in alignment_neighbors) / len(alignment_neighbors)
            avg_velocity_y = sum(drone.position[1] - y for drone in alignment_neighbors) / len(alignment_neighbors)
            target_x += int(avg_velocity_x)
            target_y += int(avg_velocity_y)

        # Separation: avoid collisions with neighbors
        separation_force_x, separation_force_y = self.calculate_separation_force(neighbors)
        target_x += separation_force_x
        target_y += separation_force_y

        # Move towards the target position
        dx = target_x - x
        dy = target_y - y
        new_x = int(x + dx)
        new_y = int(y + dy)

        # Ensure the new position is within the maze bounds
        if (0 <= new_x < len(self.maze) and 0 <= new_y < len(self.maze[0]) and self.maze[new_x][new_y] == 0 and (new_x, new_y) not in self.visited):
            self.position = (new_x, new_y)
            self.visited.add(self.position)
            self.path.append(self.position)
            print(f"Drone {self.id} moved to {self.position}")
        else:
            # Backtrack to the previous position
            if len(self.path) > 1:
                self.position = self.path[-2]
                print(f"Drone {self.id} backtracked to {self.position}")
                self.path.pop()

        # Check if goal is found
        if self.position == self.goal_position:
            self.goal_found = True

        # Increment iteration count
        self.iteration_count += 1

    def distance_to(self, other_drone):
        x1, y1 = self.position
        x2, y2 = other_drone.position
        return ((x2 - x1) ** 2 + (y2 - y1) ** 2) ** 0.5

    def calculate_separation_force(self, neighbors):
        separation_force_x = 0
        separation_force_y = 0
        for neighbor in neighbors:
            dx = self.position[0] - neighbor.position[0]
            dy = self.position[1] - neighbor.position[1]
            distance = self.distance_to(neighbor)
            if distance != 0:  # Skip division by zero
                separation_force_x += dx / distance
                separation_force_y += dy / distance
        magnitude = (separation_force_x ** 2 + separation_force_y ** 2) ** 0.5
        if magnitude > 0:
            separation_force_x *= SEPARATION_FORCE / magnitude
            separation_force_y *= SEPARATION_FORCE / magnitude
        return separation_force_x, separation_force_y

def draw_maze(screen, maze):
    for row in range(len(maze)):
        for col in range(len(maze[0])):
            color = WHITE if maze[row][col] == 0 else BLACK
            pygame.draw.rect(screen, color, [(MARGIN + CELL_SIZE) * col + MARGIN,
                                              (MARGIN + CELL_SIZE) * row + MARGIN,
                                              CELL_SIZE, CELL_SIZE])

def draw_drones(screen, drones):
    for drone in drones:
        x, y = drone.position
        pygame.draw.circle(screen, drone.color, [(MARGIN + CELL_SIZE) * y + CELL_SIZE // 2 + MARGIN,
                                                 (MARGIN + CELL_SIZE) * x + CELL_SIZE // 2 + MARGIN], CELL_SIZE // 4)

def main():
    # Sample maze represented as a 2D grid (0: empty, 1: obstacle)
    maze_width = 80
    maze_height = 50
    maze = [[0] * maze_width for _ in range(maze_height)]
    # Place more obstacles
    for _ in range(maze_width * maze_height // 6):
        row = random.randint(0, maze_height - 1)
        col = random.randint(0, maze_width - 1)
        maze[row][col] = 1

    goal_position = (maze_height - 1, maze_width - 1)

    num_drones = 5
    drones = [Drone(i, maze, goal_position) for i in range(1, num_drones + 1)]
    for drone in drones:
        drone.all_drones = drones

    pygame.init()

    # Set up the screen
    screen_size = ((CELL_SIZE + MARGIN) * maze_width + MARGIN, (CELL_SIZE + MARGIN) * maze_height + MARGIN)
    screen = pygame.display.set_mode(screen_size)
    pygame.display.set_caption("Drone Maze Navigation")

    # Set up clock
    clock = pygame.time.Clock()

    running = True
    while running:
        for event in pygame.event.get():
            if event.type == pygame.QUIT:
                running = False

        screen.fill(WHITE)
        draw_maze(screen, maze)
        # Draw goal
        pygame.draw.circle(screen, GOAL_COLOR, [(MARGIN + CELL_SIZE) * goal_position[1] + CELL_SIZE // 2 + MARGIN,
                                             (MARGIN + CELL_SIZE) * goal_position[0] + CELL_SIZE // 2 + MARGIN], CELL_SIZE // 4)
        draw_drones(screen, drones)
        pygame.display.flip()  # Update the display

        # Check if goal is found by all drones
        if all(drone.goal_found for drone in drones):
            print("Goal found by all drones")
            break

        # Simulate drone movement
        for drone in drones:
            drone.move()

            # Mid-level control: Split drones if threshold reached
            if drone.iteration_count >= SPLIT_THRESHOLD:
                # Create new subgroup and reset iteration count
                new_subgroup = max(drone.subgroup, max(drone.all_drones, key=lambda x: x.subgroup).subgroup) + 1
                drone.subgroup = new_subgroup
                drone.iteration_count = 0
                print(f"Drone {drone.id} split into subgroup {drone.subgroup}")

        # Check if any subgroup found the goal and inform other subgroups
        found_subgroups = set()
        for drone in drones:
            if drone.goal_found:
                found_subgroups.add(drone.subgroup)

        # Inform other drones about the goal
        for drone in drones:
            if drone.subgroup not in found_subgroups:
                # Move towards the goal position of the first subgroup
                drone.goal_position = next((drone.goal_position for drone in drones if drone.subgroup in found_subgroups), goal_position)

        clock.tick(10)  # Adjust the speed of simulation

    pygame.quit()

if __name__ == "__main__":
    main()
