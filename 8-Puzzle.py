import random
import time
import heapq  # Using heapq for an efficient priority queue implementation


def is_solvable(puzzle):
    """
    Checks if a given 8-puzzle is solvable.
    A puzzle is solvable if the number of inversions is even.
    """
    # Exclude the blank tile (0) and flatten the puzzle
    puzzle = [num for num in puzzle if num != 0]
    inversions = 0

    # Count inversions: a pair (i, j) forms an inversion if i > j and i appears before j
    for i in range(len(puzzle)):
        for j in range(i + 1, len(puzzle)):
            if puzzle[i] > puzzle[j]:
                inversions += 1

    # A solvable puzzle has an even number of inversions
    return inversions % 2 == 0


def generate_random_solvable_puzzle():
    """
    Generates a random 8-puzzle configuration that is guaranteed to be solvable.
    """
    while True:
        # Generate a random permutation of numbers 0 through 8
        numbers = list(range(9))
        random.shuffle(numbers)

        # Check if the generated puzzle is solvable
        if is_solvable(numbers):
            # Convert the flat list into a 3x3 grid
            puzzle = [numbers[i:i + 3] for i in range(0, 9, 3)]
            return puzzle


def get_goal_state():
    """
    Returns the goal state for the 8-puzzle.
    """
    return [[1, 2, 3],
            [4, 5, 6],
            [7, 8, 0]]


def hamming_distance(state, goal):
    """
    Calculates the Hamming distance between the current state and the goal state.
    Hamming distance is the number of tiles that are out of place (ignoring the blank).
    """
    distance = 0
    for i in range(3):
        for j in range(3):
            if state[i][j] != 0 and state[i][j] != goal[i][j]:
                distance += 1
    return distance


def manhattan_distance(state, goal):
    """
    Calculates the Manhattan distance between the current state and the goal state.
    Manhattan distance is the sum of the vertical and horizontal distances of each tile
    from its correct position.
    """
    distance = 0

    # Precompute goal positions for each tile for quick lookup
    goal_positions = {goal[i][j]: (i, j) for i in range(3) for j in range(3)}

    # Calculate the Manhattan distance for each tile
    for i in range(3):
        for j in range(3):
            if state[i][j] != 0:
                target_x, target_y = goal_positions[state[i][j]]
                distance += abs(i - target_x) + abs(j - target_y)
    return distance


def get_neighbors(state):
    """
    Generates all possible neighbors of the given state by sliding the blank tile
    in one of the four cardinal directions (up, down, left, right).
    """
    neighbors = []

    # Find the position of the blank tile (0)
    blank_x, blank_y = [(i, j) for i in range(3) for j in range(3) if state[i][j] == 0][0]

    # Define possible moves (dx, dy) for the blank tile
    moves = [(-1, 0), (1, 0), (0, -1), (0, 1)]  # Up, Down, Left, Right

    for dx, dy in moves:
        x, y = blank_x + dx, blank_y + dy

        # Check if the move is within bounds
        if 0 <= x < 3 and 0 <= y < 3:
            # Create a new state by swapping the blank tile with the target tile
            new_state = [row[:] for row in state]
            new_state[blank_x][blank_y], new_state[x][y] = new_state[x][y], new_state[blank_x][blank_y]
            neighbors.append(new_state)

    return neighbors


def a_star_search(start, goal, heuristic_func):
    """
    A* search algorithm for solving the 8-puzzle problem.
    Uses a given heuristic function to estimate the cost to the goal.
    """
    # Initialize the open set as a min-heap priority queue
    open_set = []
    heapq.heappush(open_set, (0, start))

    # Dictionaries to store the path and cost information
    came_from = {}
    g_score = {tuple(map(tuple, start)): 0}
    f_score = {tuple(map(tuple, start)): heuristic_func(start, goal)}

    expanded_nodes = 0  # Counter to track the number of expanded nodes

    while open_set:
        # Extract the state with the lowest f_score from the open set
        _, current = heapq.heappop(open_set)
        expanded_nodes += 1

        # Check if the goal state is reached
        if current == goal:
            # Reconstruct the solution path
            path = []
            while current:
                path.append(current)
                current = came_from.get(tuple(map(tuple, current)))
            path.reverse()

            # Print the solution path
            print("\nSolution Path:")
            for step in path:
                for row in step:
                    print(row)
                print("---")
            return expanded_nodes, g_score[tuple(map(tuple, goal))]

        # Explore all valid neighbors of the current state
        neighbors = get_neighbors(current)
        for neighbor in neighbors:
            neighbor_tuple = tuple(map(tuple, neighbor))
            tentative_g_score = g_score[tuple(map(tuple, current))] + 1

            # Update path and costs if a better path is found
            if tentative_g_score < g_score.get(neighbor_tuple, float('inf')):
                came_from[neighbor_tuple] = current
                g_score[neighbor_tuple] = tentative_g_score
                f_score[neighbor_tuple] = tentative_g_score + heuristic_func(neighbor, goal)
                heapq.heappush(open_set, (f_score[neighbor_tuple], neighbor))

    print("No solution found.")
    return expanded_nodes, -1  # Return -1 if no solution is found


def run_experiment():
    """
    Runs an experiment to compare the performance of Hamming and Manhattan heuristics
    on 100 random solvable 8-puzzle configurations.
    """
    goal = get_goal_state()
    hamming_results = []
    manhattan_results = []

    for _ in range(100):
        # Generate a random solvable puzzle
        puzzle = generate_random_solvable_puzzle()

        # Solve using Hamming heuristic and measure performance
        start_time = time.time()
        hamming_nodes, _ = a_star_search(puzzle, goal, hamming_distance)
        hamming_time = time.time() - start_time

        # Solve using Manhattan heuristic and measure performance
        start_time = time.time()
        manhattan_nodes, _ = a_star_search(puzzle, goal, manhattan_distance)
        manhattan_time = time.time() - start_time

        # Store results
        hamming_results.append((hamming_nodes, hamming_time))
        manhattan_results.append((manhattan_nodes, manhattan_time))

    # Compute and print average results
    hamming_avg_nodes = sum(r[0] for r in hamming_results) / 100
    hamming_avg_time = sum(r[1] for r in hamming_results) / 100
    manhattan_avg_nodes = sum(r[0] for r in manhattan_results) / 100
    manhattan_avg_time = sum(r[1] for r in manhattan_results) / 100

    print(f"Hamming - Avg Nodes: {hamming_avg_nodes}, Avg Time: {hamming_avg_time:.4f}s")
    print(f"Manhattan - Avg Nodes: {manhattan_avg_nodes}, Avg Time: {manhattan_avg_time:.4f}s")


# Run the experiment
run_experiment()
