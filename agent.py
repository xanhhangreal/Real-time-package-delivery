def run_bfs(map, start, goal):
    n_rows = len(map)
    n_cols = len(map[0])
    queue = [goal]
    visited = set([goal])
    d = {goal: 0}

    while queue:
        current = queue.pop(0)
        for dx, dy in [(-1, 0), (1, 0), (0, -1), (0, 1)]:
            next_pos = (current[0] + dx, current[1] + dy)
            if 0 <= next_pos[0] < n_rows and 0 <= next_pos[1] < n_cols:
                if next_pos not in visited and map[next_pos[0]][next_pos[1]] == 0:
                    visited.add(next_pos)
                    d[next_pos] = d[current] + 1
                    queue.append(next_pos)

    if start not in d:
        return 'S', 100000

    actions = ['U', 'D', 'L', 'R']
    current = start
    for t, (dx, dy) in enumerate([(-1, 0), (1, 0), (0, -1), (0, 1)]):
        next_pos = (current[0] + dx, current[1] + dy)
        if next_pos in d and d[next_pos] == d[start] - 1:
            return actions[t], d[next_pos]
    return 'S', d[start]


def bfs_all(map, start):
    n_rows, n_cols = len(map), len(map[0])
    queue = [start]
    visited = set([start])
    d = {start: 0}
    while queue:
        current = queue.pop(0)
        for dx, dy in [(-1, 0), (1, 0), (0, -1), (0, 1)]:
            nxt = (current[0] + dx, current[1] + dy)
            if 0 <= nxt[0] < n_rows and 0 <= nxt[1] < n_cols and map[nxt[0]][nxt[1]] == 0 and nxt not in visited:
                d[nxt] = d[current] + 1
                visited.add(nxt)
                queue.append(nxt)
    return d


class Agents:
    def __init__(self):
        self.agents = []
        self.packages = []
        self.packages_free = []
        self.n_robots = 0
        self.state = None
        self.is_init = False
        self.map_distances = dict()

    def init_agents(self, state):
        self.state = state
        self.n_robots = len(state['robots'])
        self.map = state['map']
        self.robots = [(robot[0] - 1, robot[1] - 1, 0) for robot in state['robots']]
        self.robots_target = ['free'] * self.n_robots
        self.packages += [(p[0], p[1] - 1, p[2] - 1, p[3] - 1, p[4] - 1, p[5]) for p in state['packages']]
        self.packages_free = [True] * len(self.packages)

        # Precompute all BFS distances
        for r in range(len(self.map)):
            for c in range(len(self.map[0])):
                if self.map[r][c] == 0:
                    self.map_distances[(r, c)] = bfs_all(self.map, (r, c))

    def get_distance(self, start, goal):
        return self.map_distances.get(start, {}).get(goal, 100000)

    def update_move_to_target(self, robot_id, target_package_id, phase='start'):
        pkg = self.packages[target_package_id]
        if phase == 'start':
            target_p = (pkg[1], pkg[2])
        else:
            target_p = (pkg[3], pkg[4])

        move, distance = run_bfs(self.map, (self.robots[robot_id][0], self.robots[robot_id][1]), target_p)
        pkg_act = 0
        if distance == 0:
            pkg_act = 1 if phase == 'start' else 2
        return move, str(pkg_act)

    def update_inner_state(self, state):
        for i in range(len(state['robots'])):
            prev = self.robots[i]
            robot = state['robots'][i]
            self.robots[i] = (robot[0] - 1, robot[1] - 1, robot[2])
            if prev[2] != 0 and self.robots[i][2] == 0:
                self.robots_target[i] = 'free'
            elif self.robots[i][2] != 0:
                self.robots_target[i] = self.robots[i][2]

        existing_ids = {p[0] for p in self.packages}
        for p in state['packages']:
            if p[0] not in existing_ids:
                self.packages.append((p[0], p[1] - 1, p[2] - 1, p[3] - 1, p[4] - 1, p[5]))
                self.packages_free.append(True)

    def get_actions(self, state):
        if not self.is_init:
            self.is_init = True
            self.update_inner_state(state)
        else:
            self.update_inner_state(state)

        actions = []
        current_time = state['time_step']

        for i in range(self.n_robots):
            if self.robots_target[i] != 'free':
                target_package_id = self.robots_target[i] - 1
                phase = 'target' if self.robots[i][2] != 0 else 'start'
                move, action = self.update_move_to_target(i, target_package_id, phase)
                actions.append((move, action))
            else:
                best_score = -1
                best_package_index = None

                for j in range(len(self.packages)):
                    if not self.packages_free[j]:
                        continue

                    pkg = self.packages[j]
                    dist_to_pickup = self.get_distance((self.robots[i][0], self.robots[i][1]), (pkg[1], pkg[2]))
                    dist_to_delivery = self.get_distance((pkg[1], pkg[2]), (pkg[3], pkg[4]))

                    if dist_to_pickup == 100000 or dist_to_delivery == 100000:
                        continue

                    total_time_needed = dist_to_pickup + dist_to_delivery
                    time_left = max(pkg[5] - current_time, 0)
                    lateness = total_time_needed - time_left
                    reward = max(0.5, 20 - lateness) - 0.05 * dist_to_pickup
                    if reward < 2: continue  # Bỏ qua gói có điểm thấp không đáng giao  # Bỏ qua gói có điểm thấp không đáng giao  # Cho phép gói trễ vẫn có điểm và ưu tiên gần robot  # Ưu tiên gói gần robot

                    if reward > best_score:
                        best_score = reward
                        best_package_index = j

                if best_package_index is not None:
                    closest_package_id = self.packages[best_package_index][0]
                    self.packages_free[best_package_index] = False
                    self.robots_target[i] = closest_package_id
                    move, action = self.update_move_to_target(i, best_package_index)
                    actions.append((move, str(action)))
                else:
                    actions.append(('S', '0'))

        print("N robots = ", len(self.robots))
        print("Actions = ", actions)
        print(self.robots_target)
        return actions
