import random

class Puzzle:
    def __init__(self, initial_state=None):
        self.state = initial_state if initial_state else self.generate_random_state()
        self.goal_state = self.generate_goal_state()
        self.size = 3  # 3 x 3

    def generate_random_state(self):
        #random solvable state
        while True:
            state = random.sample(range(9), 9)
            if self.is_solvable(state):
                return state

    def generate_goal_state(self):
        return  [0] + list(range(1, 9))

    def is_solvable(self, state):
        # check how many inversion there are for solvability
        inversions = 0
        for i in range(len(state)):
            for j in range(i + 1, len(state)):
                if state[i] > state[j] != 0:
                    inversions += 1
        return inversions % 2 == 0

    def hamming_heuristic(self, state):
        # Count tiles htat are not on goal state
        misplaced_count = 0
        for i in range(9):
            if state[i] != 0 and state[i] != self.goal_state[i]:
                misplaced_count += 1
            return misplaced_count

    def manhattan_heuristic(self, state):
        distance = 0
        for i in range(9):
            if state[i] != 0:
                 #calc newyork
        return distance

def main():
    puzzle = Puzzle()
    random_state = puzzle.generate_random_state() #save random puzzle state für späteren vergleich aus selben state
    puzzle.state = random_state

    #do stuff

    if __name__ == "__main__":
        main()
        
