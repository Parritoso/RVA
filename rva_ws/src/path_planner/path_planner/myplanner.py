"""
implement here your path planning method
"""

import math
from nav_msgs.msg import OccupancyGrid

# Coste a partir del cual consideramos obstáculo (no transitable)
LETHAL_COST = 100

class Planner:
    def __init__(self, costmap):
        """ 
        Initialize a map from a ROS costmap
        
        costmap: ROS costmap
        """
        # Copy the map metadata
        self.resolution = costmap.info.resolution
        self.min_x = costmap.info.origin.position.x
        self.min_y = costmap.info.origin.position.y
        self.y_width = costmap.info.height
        self.x_width = costmap.info.width
        self.max_x = self.min_x + self.x_width *self.resolution
        self.max_y = self.min_y + self.y_width *self.resolution
        print("min corner x: %.2f m, y: %.2f m" % (self.min_x, self.min_y))
        print("max corner x: %.2f m, y: %.2f m" % (self.max_x, self.max_y))
        print("Resolution: %.3f m/cell" % self.resolution)
        print("Width: %i cells, height: %i cells" % (self.x_width, self.y_width))
        
        # MODIFICACIÓN 1: Almacenar los costes reales en lugar de un mapa binario
        self.cost_map = [[0 for _ in range(self.y_width)] for _ in range(self.x_width)]
        x = 0
        y = 0
        obstacles = 0
        
        for value in costmap.data:
            # Los mapas de ROS suelen indicar espacio desconocido como -1
            if value < 0:
                cost = LETHAL_COST # Lo tratamos como obstáculo por seguridad
            else:
                cost = value
                
            self.cost_map[x][y] = cost
            
            if cost >= LETHAL_COST:
                obstacles += 1
                
            # Update the iterators
            x += 1
            if x == self.x_width:
                x = 0
                y += 1
        print("Loaded %d lethal obstacles" % obstacles)

           
    class Node:
        def __init__(self, cx, cy, cost, parent):
            self.x_cell = cx  # x index in the obstacle grid
            self.y_cell = cy  # y index in the obstacle grid

            self.cost = cost    # g(n): coste acumulado desde el start
            self.parent = parent # index/key of the previous Node
            self.h = 0.0        # h(n): heurística (estimación al goal)
            self.f = 0.0


    def plan(self, sx, sy, gx, gy):
        """
        input:
            sx: start x position [m]
            sy: start y position [m]
            gx: goal x position [m]
            gx: goal x position [m]

        output:
            rx: x position list of the final path
            ry: y position list of the final path
        """
        
        try:
            # first check if we are already very close
            d = math.sqrt((gx-sx)*(gx-sx) + (gy-sy)*(gy-sy))
            if d <= self.resolution*2.0:
                return None

            # create the start node and the goal node
            start_cell_x, start_cell_y = self.real2cell(sx, sy)  
            start_node = self.Node(start_cell_x, start_cell_y, 0.0, -1)
            goal_cell_x, goal_cell_y = self.real2cell(gx, gy)
            goal_node = self.Node(goal_cell_x, goal_cell_y, 0.0, -1)

            # check if the positions are valid (no obstacle)
            if (not self.node_is_valid(start_node)):
                print("Error: start position not valid!! (Cost >= LETHAL_COST)")
                return None
            
            if (not self.node_is_valid(goal_node)):
                print("Error: goal position not valid!! (Cost >= LETHAL_COST)")
                return None
        except Exception as e:
            print(f"Error in plan: {e}")
            return None
        
        try:
            open_set = {}
            closed_set = {}

            # Calcular heurística del nodo start
            start_node.h = self.calc_heuristic(start_node, goal_node)
            start_node.f = start_node.cost + start_node.h

            open_set[(start_node.x_cell, start_node.y_cell)] = start_node

            # Movimientos posibles: (dx, dy, coste base de distancia)
            motion = [
                (1, 0, 1.0),
                (0, 1, 1.0),
                (-1, 0, 1.0),
                (0, -1, 1.0),
                (1, 1, math.sqrt(2)),
                (1, -1, math.sqrt(2)),
                (-1, 1, math.sqrt(2)),
                (-1, -1, math.sqrt(2)),
            ]

            while open_set:
                # Buscar el nodo con menor f en open_set
                current_key = min(open_set, key=lambda k: open_set[k].f)
                current = open_set[current_key]

                # ¿Hemos llegado al goal?
                if current.x_cell == goal_node.x_cell and current.y_cell == goal_node.y_cell:
                    rx, ry = self.reconstruct_path(current, closed_set)
                    return rx, ry

                # Mover current de open a closed
                del open_set[current_key]
                closed_set[current_key] = current

                # Expandir vecinos
                for dx, dy, move_dist_cost in motion:
                    nx = current.x_cell + dx
                    ny = current.y_cell + dy
                    
                    neighbor_key = (nx, ny)

                    # Crear nodo vecino temporal para validar
                    # El coste lo calculamos un poco más abajo
                    neighbor = self.Node(nx, ny, 0.0, current_key)

                    # Si no es válido (fuera del grid o es LETHAL_COST), saltar
                    if not self.node_is_valid(neighbor):
                        continue

                    # Si ya está en closed, saltar
                    if neighbor_key in closed_set:
                        continue

                    # MODIFICACIÓN 2: Penalización de coste por zona de inflación
                    # El valor en cost_map será de 0 a 252.
                    # Lo dividimos por un factor (ej. 20.0) para balancearlo frente a la distancia de avance.
                    # Ajusta este valor: menor divisor = el robot huirá más de las paredes.
                    inflation_penalty = self.cost_map[nx][ny] / 20.0
                    
                    neighbor.cost = current.cost + move_dist_cost + inflation_penalty
                    neighbor.h = self.calc_heuristic(neighbor, goal_node)
                    neighbor.f = neighbor.cost + neighbor.h

                    # Si ya está en open con un coste g menor, saltar
                    if neighbor_key in open_set:
                        if open_set[neighbor_key].cost <= neighbor.cost:
                            continue

                    # Añadir/actualizar en open_set
                    open_set[neighbor_key] = neighbor

            # Si salimos del while, no hay camino
            print("No path found")
            return None
        except Exception as e:
            print(f"Error in A* algorithm: {e}")
            import traceback
            traceback.print_exc()
            return None

    def real2cell(self, rx, ry):
        cellx = round((rx - self.min_x) / self.resolution)
        celly = round((ry - self.min_y) / self.resolution)
        return cellx, celly
    
    def cell2real(self, cx, cy):
        rx = cx * self.resolution + self.min_x
        ry = cy * self.resolution + self.min_y
        return rx, ry
    
    def calc_heuristic(self, node, goal_node):
        """Distancia euclídea entre un nodo y el goal (en celdas)."""
        dx = node.x_cell - goal_node.x_cell
        dy = node.y_cell - goal_node.y_cell
        return math.sqrt(dx * dx + dy * dy)

    def reconstruct_path(self, goal_node, closed_set):
        """Recorre los parents desde el goal hasta el start y devuelve el path en metros."""
        rx = []
        ry = []

        node = goal_node
        while node.parent != -1:
            wx, wy = self.cell2real(node.x_cell, node.y_cell)
            rx.append(wx)
            ry.append(wy)
            node = closed_set[node.parent]

        wx, wy = self.cell2real(node.x_cell, node.y_cell)
        rx.append(wx)
        ry.append(wy)

        rx.reverse()
        ry.reverse()

        return rx, ry
    
    def node_is_valid(self, node):
        # check that the cell indices are within the grid bounds
        if node.x_cell < 0 or node.x_cell >= self.x_width:
            return False
        if node.y_cell < 0 or node.y_cell >= self.y_width:
            return False

        # check that the node is inside the grid limits
        rx, ry = self.cell2real(node.x_cell, node.y_cell)
        if rx < self.min_x:
            return False
        if ry < self.min_y:
            return False
        if rx >= self.max_x:
            return False
        if ry >= self.max_y:
            return False
        
        # MODIFICACIÓN 3: Comprobar la colisión contra LETHAL_COST en lugar de booleano
        if self.cost_map[int(node.x_cell)][int(node.y_cell)] >= LETHAL_COST:
            return False

        return True