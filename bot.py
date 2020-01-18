from typing import Dict, List
from game_message import *
from bot_message import *
import random
import math
import heapq
from collections import namedtuple

import traceback


class Bot:

    STATE_BLITZ = 1
    STATE_DEFENSE = 2
    STATE_RUN_TO_SAFETY = 3
    STATE_ATTACK = 4

    def __init__(self):
        self.state = Bot.STATE_BLITZ

    def get_closest_tile(self, me: Player, game: Game, target_type: TileType):
        game_map = game.map
        closest_blitz = (math.inf, None)
        for y, row in enumerate(game_map):
            for x, tile in enumerate(row):
                if target_type == TileType.get_tile_type(tile):
                    distance = self.manhattan_distance(
                        me.position, Point(x, y))
                    if distance < closest_blitz[0]:
                        path = self.find_path(me, game, Point(x, y))
                        maze = Maze(game, me)
                        neighbors = maze.get_neighbors(Maze.Node(x, y, None))
                        if path != None and len(neighbors) >= 2:
                            closest_blitz = (distance, Point(x, y))
        return closest_blitz[1]

    def get_closest_safe_tile(self, me: Player, game: Game):
        game_map = game.map
        closest_safe_tile = (math.inf, None)
        for y, row in enumerate(game_map):
            for x, tile in enumerate(row):
                if (game.get_tile_type_at(Point(x, y)) == TileType.CONQUERED and game.get_tile_owner_id(Point(x, y)) == me.id) or Point(x, y) == me.tail[0]:
                    distance = self.manhattan_distance(
                        me.position, Point(x, y))
                    if distance < closest_safe_tile[0]:
                        closest_safe_tile = (distance, Point(x, y))
        return closest_safe_tile[1]

    def find_path(self, me: Player, game: Game, target: Point):
        start = Maze.Node(me.position.x, me.position.y, None)
        goal = Maze.Node(target.x, target.y, None)
        maze = Maze(game, me)
        return PathSolver.find_shortest_path(maze, start, goal)

    def manhattan_distance(self, a: Point, b: Point):
        return abs(a.x-b.x) + abs(a.y-b.y)

    def get_legal_moves_for_current_tick(self, game: Game, players_by_id: Dict[int, Player]) -> List[Move]:
        me: Player = players_by_id[game.player_id]

        return [move for move in Move]

    def goto(self, me: Player, direction: Direction) -> Move:
        if direction == Direction.UP:
            if me.direction == Direction.UP:
                return Move.FORWARD
            elif me.direction == Direction.RIGHT:
                return Move.TURN_LEFT
            elif me.direction == Direction.LEFT:
                return Move.TURN_RIGHT
            else:
                raise Exception("backtrack!!")
        elif direction == Direction.DOWN:
            if me.direction == Direction.DOWN:
                return Move.FORWARD
            elif me.direction == Direction.LEFT:
                return Move.TURN_LEFT
            elif me.direction == Direction.RIGHT:
                return Move.TURN_RIGHT
            else:
                raise Exception("backtrack!!")
        elif direction == Direction.LEFT:
            if me.direction == Direction.LEFT:
                return Move.FORWARD
            elif me.direction == Direction.UP:
                return Move.TURN_LEFT
            elif me.direction == Direction.DOWN:
                return Move.TURN_RIGHT
            else:
                raise Exception("backtrack!!")
        elif direction == Direction.RIGHT:
            if me.direction == Direction.RIGHT:
                return Move.FORWARD
            elif me.direction == Direction.UP:
                return Move.TURN_RIGHT
            elif me.direction == Direction.DOWN:
                return Move.TURN_LEFT
            else:
                raise Exception("backtrack!!")
        else:
            raise Exception("uh oh")

    def get_next_move(self, game_message: GameMessage) -> Move:
        players_by_id: Dict[int,
                            Player] = game_message.generate_players_by_id_dict()
        me: Player = players_by_id[game_message.game.player_id]
        legal_moves = self.get_legal_moves_for_current_tick(
            game=game_message.game, players_by_id=players_by_id)

        try:
            closest_blitz = self.get_closest_tile(
                me, game_message.game, TileType.BLITZIUM)

            closest_safe_tile = self.get_closest_safe_tile(
                me, game_message.game)
            if closest_safe_tile == None:
                closest_safe_tile = me.tail[0]

            distance_to_safety = self.manhattan_distance(
                closest_safe_tile, me.position)

            target = closest_safe_tile
            start = Maze.Node(me.position.x, me.position.y, None)
            goal = Maze.Node(target.x, target.y, None)
            maze = Maze(game_message.game, me)
            path_to_safety = PathSolver.find_shortest_path(maze, start, goal)

            for player_id, player in players_by_id.items():
                if player_id == game_message.game.player_id or player.killed:
                    continue

                if me.position.x + 1 == player.position.x and me.position.y == player.position.y and me.direction != Direction.LEFT:
                    return self.goto(me, Direction.RIGHT)
                elif me.position.x - 1 == player.position.x and me.position.y == player.position.y and me.direction != Direction.RIGHT:
                    return self.goto(me, Direction.LEFT)
                elif me.position.x == player.position.x and me.position.y + 1 == player.position.y and me.direction != Direction.UP:
                    return self.goto(me, Direction.DOWN)
                elif me.position.x == player.position.x and me.position.y - 1 == player.position.y and me.direction != Direction.DOWN:
                    return self.goto(me, Direction.UP)

                for point in player.tail:
                    if me.position.x + 1 == point.x and me.position.y == point.y and me.direction != Direction.LEFT:
                        return self.goto(me, Direction.RIGHT)
                    elif me.position.x - 1 == point.x and me.position.y == point.y and me.direction != Direction.RIGHT:
                        return self.goto(me, Direction.LEFT)
                    elif me.position.x == point.x and me.position.y + 1 == point.y and me.direction != Direction.UP:
                        return self.goto(me, Direction.DOWN)
                    elif me.position.x == point.x and me.position.y - 1 == point.y and me.direction != Direction.DOWN:
                        return self.goto(me, Direction.UP)

            if self.state == Bot.STATE_BLITZ and path_to_safety != None and len(me.tail) > 6:
                tail = me.tail[1:]
                tail.append(me.position)

                for player_id, player in players_by_id.items():
                    if player_id == game_message.game.player_id or player.killed:
                        continue

                    smallest_manhattan = None
                    for point in tail:
                        manh = self.manhattan_distance(player.position, point)
                        if manh - 4 < distance_to_safety:
                            if smallest_manhattan == None or manh < smallest_manhattan[2]:
                                smallest_manhattan = (
                                    player.position, point, manh)

                    if smallest_manhattan != None:
                        start = Maze.Node(
                            player.position.x, player.position.y, None)
                        goal = Maze.Node(point.x, point.y, None)
                        maze = Maze(game_message.game, me)
                        path_to_death = PathSolver.find_shortest_path(
                            maze, start, goal, include_tail=True)

                        if len(path_to_safety) > len(path_to_death) or len(path_to_death) < 4:
                            self.state = Bot.STATE_RUN_TO_SAFETY
                            break

            if self.state == Bot.STATE_BLITZ:
                if closest_blitz == None:
                    self.state = Bot.STATE_DEFENSE
                    return self.get_next_move(game_message)

                if closest_blitz != None:
                    path = self.find_path(me, game_message.game, closest_blitz)
                    if path != None:
                        action = self.goto(me, path[0])
                        if action in legal_moves:
                            return action

            elif self.state == Bot.STATE_DEFENSE:
                if len(me.tail) == 1:
                    if closest_blitz != None:
                        self.state = Bot.STATE_BLITZ
                    else:
                        self.state = Bot.STATE_ATTACK
                    return self.get_next_move(game_message)

                target = me.tail[0]
                path = self.find_path(me, game_message.game, target)
                if path != None:
                    action = self.goto(me, path[0])
                    if action in legal_moves:
                        return action

            elif self.state == Bot.STATE_RUN_TO_SAFETY:
                if len(me.tail) == 1:
                    if closest_blitz != None:
                        self.state = Bot.STATE_BLITZ
                    else:
                        self.state = Bot.STATE_ATTACK
                    return self.get_next_move(game_message)

                target = closest_safe_tile
                path = self.find_path(me, game_message.game, target)
                if path != None:
                    action = self.goto(me, path[0])
                    if action in legal_moves:
                        return action

            elif self.state == Bot.STATE_ATTACK:
                for id, player in players_by_id.items():
                    if id != game_message.game.player_id:
                        target = player.tail[0]
                        path = self.find_path(me, game_message.game, target)
                        if path != None:
                            action = self.goto(me, path[0])
                            if action in legal_moves:
                                return action

        except Exception as e:
            traceback.print_exc()

        return random.choice(legal_moves)


class PathSolver:

    def __init__(self, maze, start, goal):
        self.maze = maze
        self.pos = start
        self.goal = goal

    def solve(self):
        return PathSolver.find_shortest_path(self.maze, self.pos, self.goal)

    @staticmethod
    def find_shortest_path(maze, start, goal, include_tail=False):
        visited = {}
        paths = {}
        queue = []
        heapq.heappush(queue, start)
        visited[start.id] = start.cost

        while len(queue) > 0:
            current_pos = heapq.heappop(queue)

            if current_pos == goal:
                return maze.reconstruct_path(current_pos)

            neighbors = maze.get_neighbors(current_pos, include_tail)
            for neighbor in neighbors:
                neighbor.cost = current_pos.cost + \
                    maze.cost(current_pos, neighbor)
                neighbor.estimate = neighbor.cost + \
                    maze.cost_estimate(neighbor, goal)
                if neighbor.id not in visited or visited[neighbor.id] > neighbor.cost:
                    paths[neighbor.id] = current_pos
                    heapq.heappush(queue, neighbor)
                    visited[neighbor.id] = neighbor.cost
        return None


class Maze:
    Point = namedtuple('Point', ['x', 'y'])

    def __init__(self, game, me: Player):
        self.grid = game.map
        self.tail = me.tail[1:]

    def get_neighbors(self, pos, include_tail=False):
        result = []
        moves = [(1, 0), (-1, 0), (0, 1), (0, -1)]
        current_pos = pos.coordinates
        for move in moves:
            new_pos = Maze.Point(
                current_pos.x + move[0], current_pos.y + move[1])
            if new_pos.x >= 0 and new_pos.x < len(self.grid[0]):
                if new_pos.y >= 0 and new_pos.y < len(self.grid):
                    if self.grid[new_pos.y][new_pos.x] not in ["W", "!"]:
                        if include_tail:
                            result.append(Maze.Node(new_pos.x, new_pos.y, pos))
                        elif Point(new_pos.x, new_pos.y) not in self.tail:
                            result.append(Maze.Node(new_pos.x, new_pos.y, pos))
        return result

    def cost_estimate(self, pos, target):
        return 0

    def cost(self, pos, neighbor):
        return 1

    def reconstruct_path(self, node):
        path = []
        parent = node.parent
        while parent != None:
            diff = (node.coordinates.x-parent.coordinates.x,
                    node.coordinates.y-parent.coordinates.y)
            if diff == (0, 1):
                path.append(Direction.DOWN)
            if diff == (0, -1):
                path.append(Direction.UP)
            if diff == (1, 0):
                path.append(Direction.RIGHT)
            if diff == (-1, 0):
                path.append(Direction.LEFT)
            node = parent
            parent = parent.parent
        return path[::-1]

    class Node:
        def __init__(self, x, y, parent):
            self.id = str(x) + "," + str(y)
            self.coordinates = Maze.Point(x, y)
            self.parent = parent
            self.cost = 0
            self.estimate = 0

        def __lt__(self, b):
            return self.estimate < b.estimate

        def __gt__(self, b):
            return self.estimate > b.estimate

        def __eq__(self, b):
            return type(b) == type(self) and self.coordinates.x == b.coordinates.x and self.coordinates.y == b.coordinates.y
