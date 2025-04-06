import pygame
import sys
import heapq
import random
WIDTH, HEIGHT = 600, 600
ROWS, COLS = 10, 10
TILE_SIZE = WIDTH // COLS
WHITE = (255, 255, 255)
GREY = (200, 200, 200)
BLACK = (0, 0, 0)
GREEN = (0, 255, 0)
BLUE = (0, 0, 255)
RED = (255, 0, 0)
YELLOW = (255, 255, 0)
CYAN = (0, 255, 255)
pygame.init()
win = pygame.display.set_mode((WIDTH, HEIGHT))
pygame.display.set_caption("Taxi Race: Dijkstra vs A*")
start = (0, 0)
goal_player1 = (0, 9)  # Goal for Player 1
goal_player2 = (0, 9)  # Goal for Player 2
player1 = list(start)  # Dijkstra
player2 = list(start)  # A*
def generate_obstacles(num_obstacles):
    obstacles = set()
    while len(obstacles) < num_obstacles:
        x = random.randint(0, COLS - 1)
        y = random.randint(0, ROWS - 1)
        if (x, y) != start and (x, y) != goal_player1 and (x, y) != goal_player2:
            obstacles.add((x, y))
    return obstacles
def heuristic(a, b):
    return abs(a[0] - b[0]) + abs(a[1] - b[1])
def dijkstra(start, goal, obstacles):
    distances = {start: 0}
    came_from = {}
    heap = [(0, start)]
    visited = set()

    while heap:
        cost, current = heapq.heappop(heap)
        if current in visited:
            continue
        visited.add(current)
        if current == goal:
            break
        x, y = current
        for dx, dy in [(-1, 0), (1, 0), (0, -1), (0, 1)]:
            nx, ny = x + dx, y + dy
            if 0 <= nx < COLS and 0 <= ny < ROWS and (nx, ny) not in obstacles:
                next_node = (nx, ny)
                new_cost = cost + 1
                if next_node not in distances or new_cost < distances[next_node]:
                    distances[next_node] = new_cost
                    heapq.heappush(heap, (new_cost, next_node))
                    came_from[next_node] = current

    return reconstruct_path(came_from, goal)
def a_star(start, goal, obstacles):
    open_set = []
    heapq.heappush(open_set, (0, start))
    came_from = {}
    g_score = {start: 0}

    while open_set:
        _, current = heapq.heappop(open_set)

        if current == goal:
            break

        x, y = current
        for dx, dy in [(-1, 0), (1, 0), (0, -1), (0, 1)]:
            nx, ny = x + dx, y + dy
            neighbor = (nx, ny)
            if 0 <= nx < COLS and 0 <= ny < ROWS and neighbor not in obstacles:
                tentative_g = g_score[current] + 1
                if neighbor not in g_score or tentative_g < g_score[neighbor]:
                    g_score[neighbor] = tentative_g
                    f_score = tentative_g + heuristic(neighbor, goal)
                    heapq.heappush(open_set, (f_score, neighbor))
                    came_from[neighbor] = current

    return reconstruct_path(came_from, goal)
def reconstruct_path(came_from, current):
    path = []
    while current in came_from:
        path.append(current)
        current = came_from[current]
    path.reverse()
    return path
num_obstacles = 10
obstacles = generate_obstacles(num_obstacles)
path_dijkstra = dijkstra(start, goal_player1, obstacles)
path_astar = a_star(start, goal_player2, obstacles)

def draw():
    win.fill(WHITE)
    for x in range(0, WIDTH // 2, TILE_SIZE):
        for y in range(0, HEIGHT, TILE_SIZE):
            pygame.draw.rect(win, GREY, (x, y, TILE_SIZE, TILE_SIZE), 1)
    for x in range(WIDTH // 2, WIDTH, TILE_SIZE):
        for y in range(0, HEIGHT, TILE_SIZE):
            pygame.draw.rect(win, GREY, (x, y, TILE_SIZE, TILE_SIZE), 1)
    gx1, gy1 = goal_player1
    gx2, gy2 = goal_player2
    pygame.draw.rect(win, GREEN, (gx1 * TILE_SIZE, gy1 * TILE_SIZE, TILE_SIZE, TILE_SIZE))  
    pygame.draw.rect(win, GREEN, (gx2 * TILE_SIZE + WIDTH // 2, gy2 * TILE_SIZE, TILE_SIZE, TILE_SIZE))  
    for (ox, oy) in obstacles:
        pygame.draw.rect(win, BLACK, (ox * TILE_SIZE, oy * TILE_SIZE, TILE_SIZE, TILE_SIZE))
        pygame.draw.rect(win, BLACK, (ox * TILE_SIZE + WIDTH // 2, oy * TILE_SIZE, TILE_SIZE, TILE_SIZE)) 
    
    for (x, y) in path_dijkstra:
        pygame.draw.circle(win, RED, (x * TILE_SIZE + TILE_SIZE // 2, y * TILE_SIZE + TILE_SIZE // 2), 5) 

    # A* Path
    for (x, y) in path_astar:
        pygame.draw.circle(win, BLUE, (x * TILE_SIZE + TILE_SIZE // 2 + WIDTH // 2, y * TILE_SIZE + TILE_SIZE // 2), 5) # Centered

    # Players
    pygame.draw.rect(win, YELLOW, (player1[0] * TILE_SIZE, player1[1] * TILE_SIZE, TILE_SIZE, TILE_SIZE))  # Dijkstra
    pygame.draw.rect(win, CYAN, (player2[0] * TILE_SIZE + WIDTH // 2, player2[1] * TILE_SIZE, TILE_SIZE, TILE_SIZE))    # A*

    pygame.display.update()

clock = pygame.time.Clock()
running = True
winner = None

while running:
    clock.tick(10)
    for event in pygame.event.get():
        if event.type == pygame.QUIT:
            running = False

    keys = pygame.key.get_pressed()

    # Player 1 (WASD) - Prevent moving into obstacles
    if keys[pygame.K_w] and player1[1] > 0 and (player1[0], player1[1] - 1) not in obstacles:
        player1[1] -= 1
    if keys[pygame.K_s] and player1[1] < ROWS - 1 and (player1[0], player1[1] + 1) not in obstacles:
        player1[1] += 1
    if keys[pygame.K_a] and player1[0] > 0 and (player1[0] - 1, player1[1]) not in obstacles:
        player1[0] -= 1
    if keys[pygame.K_d] and player1[0] < COLS - 1 and (player1[0] + 1, player1[1]) not in obstacles:
        player1[0] += 1

    # Player 2 (Arrow keys) - Prevent moving into obstacles
    if keys[pygame.K_UP] and player2[1] > 0 and (player2[0], player2[1] - 1) not in obstacles:
        player2[1] -= 1
    if keys[pygame.K_DOWN] and player2[1] < ROWS - 1 and (player2[0], player2[1] + 1) not in obstacles:
        player2[1] += 1
    if keys[pygame.K_LEFT] and player2[0] > 0 and (player2[0] - 1, player2[1]) not in obstacles:
        player2[0] -= 1
    if keys[pygame.K_RIGHT] and player2[0] < COLS - 1 and (player2[0] + 1, player2[1]) not in obstacles:
        player2[0] += 1

    draw()

    # Check for winning conditions
    if tuple(player1) == goal_player1 and winner is None:
        winner = "Player 1 (Dijkstra)"
    if tuple(player2) == goal_player2 and winner is None:
        winner = "Player 2 (A*)"

    if winner:
        print(f"{winner} wins!")
        pygame.time.wait(2000)
        running = False

pygame.quit()
sys.exit()
