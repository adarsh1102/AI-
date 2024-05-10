import heapq
import tkinter as tk

class Node:
    def __init__(self, state, parent=None, cost=0, heuristic=0):
        self.state = state
        self.parent = parent
        self.cost = cost
        self.heuristic = heuristic

    def __lt__(self, other):
        return (self.cost + self.heuristic) < (other.cost + other.heuristic)

def a_star_search(graph, start, goal):
    open_list = []
    closed_set = set()

    start_node = Node(state=start, cost=0, heuristic=calculate_heuristic(start, goal))
    heapq.heappush(open_list, start_node)

    while open_list:
        current_node = heapq.heappop(open_list)

        if current_node.state == goal:
            path = []
            while current_node:
                path.append(current_node.state)
                current_node = current_node.parent
            return path[::-1]

        closed_set.add(current_node.state)

        for neighbor, distance in graph[current_node.state].items():
            if neighbor not in closed_set:
                cost = current_node.cost + distance
                heuristic = calculate_heuristic(neighbor, goal)
                neighbor_node = Node(state=neighbor, parent=current_node, cost=cost, heuristic=heuristic)
                heapq.heappush(open_list, neighbor_node)

    return None

def calculate_heuristic(current, goal):
    return abs(current[0] - goal[0]) + abs(current[1] - goal[1])

def find_nearest_passenger(driver_location, passengers):
    nearest_passenger = None
    min_distance = float('inf')

    for passenger_location in passengers.keys():
        distance = calculate_heuristic(driver_location, passenger_location)
        if distance < min_distance:
            min_distance = distance
            nearest_passenger = passenger_location

    return nearest_passenger

def build_graph(passengers):
    graph = {}
    for location, _ in passengers.items():
        graph[location] = {}
        for destination, _ in passengers.items():
            if location != destination:
                graph[location][destination] = calculate_heuristic(location, destination)
    return graph

def get_user_input(prompt):
    while True:
        try:
            value = tuple(map(int, input(prompt).split()))
            return value
        except ValueError:
            print("Please enter valid coordinates (integers separated by space).")

def display_map(driver_location, passengers):
    max_x = max(driver_location[0], max(p[0] for p in passengers.keys()))
    max_y = max(driver_location[1], max(p[1] for p in passengers.keys()))

    map_grid = [['.' for _ in range(max_x + 1)] for _ in range(max_y + 1)]

    map_grid[driver_location[1]][driver_location[0]] = 'D'

    for passenger_location in passengers.keys():
        map_grid[passenger_location[1]][passenger_location[0]] = 'P'

    print("Map:")
    for row in map_grid[::-1]:
        print(' '.join(row))

def on_submit():
    global driver_location, passengers
    driver_location = (int(entry_driver_x.get()), int(entry_driver_y.get()))

    num_passengers = int(entry_num_passengers.get())
    passengers = {}
    for i in range(1, num_passengers + 1):
        passenger_location = (int(entries_passengers[i-1].get()), int(entries_passengers[i].get()))
        passengers[passenger_location] = f"Passenger {i}"

    display_map(driver_location, passengers)
    nearest_passenger = find_nearest_passenger(driver_location, passengers)
    label_nearest_passenger.config(text=f"Nearest passenger: {nearest_passenger}")

# Create tkinter window
window = tk.Tk()
window.title("A* Search Algorithm")

# Create labels and entry widgets for driver location
label_driver = tk.Label(window, text="Driver Location (x y):")
label_driver.grid(row=0, column=0, sticky="e")
entry_driver_x = tk.Entry(window)
entry_driver_x.grid(row=0, column=1)
entry_driver_y = tk.Entry(window)
entry_driver_y.grid(row=0, column=2)

# Create label and entry widget for number of passengers
label_num_passengers = tk.Label(window, text="Number of Passengers:")
label_num_passengers.grid(row=1, column=0, sticky="e")
entry_num_passengers = tk.Entry(window)
entry_num_passengers.grid(row=1, column=1)

# Create labels and entry widgets for passenger locations
labels_passengers = []
entries_passengers = []
for i in range(1, 6):  # Assuming maximum 5 passengers
    label_passenger = tk.Label(window, text=f"Passenger {i} Location (x y):")
    label_passenger.grid(row=i+1, column=0, sticky="e")
    entry_passenger_x = tk.Entry(window)
    entry_passenger_x.grid(row=i+1, column=1)
    entry_passenger_y = tk.Entry(window)
    entry_passenger_y.grid(row=i+1, column=2)
    labels_passengers.append(label_passenger)
    entries_passengers.append(entry_passenger_x)
    entries_passengers.append(entry_passenger_y)

# Create submit button
submit_button = tk.Button(window, text="Submit", command=on_submit)
submit_button.grid(row=7, columnspan=3)

# Create label to display nearest passenger
label_nearest_passenger = tk.Label(window, text="")
label_nearest_passenger.grid(row=8, columnspan=3)

window.mainloop()
