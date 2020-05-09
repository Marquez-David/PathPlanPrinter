import path_planning as pp

def children(point,grid):
    """
        Calculates the children of a given node over a grid.
        Inputs:
            - point: node for which to calculate children.
            - grid: grid over which to calculate children.
        Outputs:
            - list of children for the given node.
    """
    x,y = point.grid_point
    if x > 0 and x < len(grid) - 1:
        if y > 0 and y < len(grid[0]) - 1:
            links = [grid[d[0]][d[1]] for d in\
                     [(x-1, y),(x,y - 1),(x,y + 1),(x+1,y),\
                      (x-1, y-1), (x-1, y+1), (x+1, y-1),\
                      (x+1, y+1)]]
        elif y > 0:
            links = [grid[d[0]][d[1]] for d in\
                     [(x-1, y),(x,y - 1),(x+1,y),\
                      (x-1, y-1), (x+1, y-1)]]
        else:
            links = [grid[d[0]][d[1]] for d in\
                     [(x-1, y),(x,y + 1),(x+1,y),\
                      (x-1, y+1), (x+1, y+1)]]
    elif x > 0:
        if y > 0 and y < len(grid[0]) - 1:
            links = [grid[d[0]][d[1]] for d in\
                     [(x-1, y),(x,y - 1),(x,y + 1),\
                      (x-1, y-1), (x-1, y+1)]]
        elif y > 0:
            links = [grid[d[0]][d[1]] for d in\
                     [(x-1, y),(x,y - 1),(x-1, y-1)]]
        else:
            links = [grid[d[0]][d[1]] for d in\
                     [(x-1, y), (x,y + 1), (x-1, y+1)]]
    else:
        if y > 0 and y < len(grid[0]) - 1:
            links = [grid[d[0]][d[1]] for d in\
                     [(x+1, y),(x,y - 1),(x,y + 1),\
                      (x+1, y-1), (x+1, y+1)]]
        elif y > 0:
            links = [grid[d[0]][d[1]] for d in\
                     [(x+1, y),(x,y - 1),(x+1, y-1)]]
        else:
            links = [grid[d[0]][d[1]] for d in\
                     [(x+1, y), (x,y + 1), (x+1, y+1)]]
    return [link for link in links if link.value != 9]

def thetaStar(start, goal, grid, heur='naive'):
    #The open and closed sets
    openset = set()
    closedset = set()
    #Current point is the starting point
    current = start
    #Add the starting point to the open set
    openset.add(current)
    #While the open set is not empty
    while openset:
        #Find the item in the open set with the lowest G + H score
        current = min(openset, key=lambda o:o.G + o.H)
        pp.expanded_nodes += 1
        #If it is the item we want, retrace the path and return it
        if current == goal:
            path = []
            while current.parent:
                path.append(current)
                current = current.parent
            path.append(current)
            return path[::-1]
        #Remove the item from the open set
        openset.remove(current)
        #Add it to the closed set
        closedset.add(current)
        #Loop through the node's children/siblings
        for node in children(current,grid):
            #If it is already in the closed set, skip it
            if node in closedset:
                continue
            #Otherwise if it is already in the open set
            if node in openset:
                #Check if we beat the G score 
                new_g = current.G + current.move_cost(node)
                if node.G > new_g:
                    #If so, update the node to have a new parent
                    node.G = new_g
                    node.parent = current
            else:
                if (lineOfSight(current, node)):
                    if (current.parent.G + current.parent.move_cost(node) < node.G):
                        node.G = current.parent.G + current.parent.move_cost(node)
                        node.parent = current.parent
                        #if it is already in the open set
                        if (node in openset):
                            openset.remove(node)
                else:
                    if (current.G + current.move_cost(node) < node.G):
                        node.G = current.G + current.move_cost(node)
                        node.parent = current
                        if (node in openset):
                            openset.add(node)


pp.register_search_method('theta*', thetaStar)


def lineOfSight(current, node):
    f = 0
    
    x0 = current.point[0]  # coordenada 'x' del punto inicial
    y0 = current.point[1]  # coordenada 'y' del punto inicial
    x1 = node.point[0]  # coordenada 'x' del punto final
    y1 = node.point[1]  # coordenada 'y' del punto final

    # Se calculan las distancias de los puntos
    distx = x1 - x0
    disty = y1 - y0

    if (disty < 0):
        disty = -disty
        sy = -1
    else:
        sy = 1
    if (distx < 0):
        distx = -distx
        sx = -1
    else:
        sx = 1

    if (distx >= disty):
        while (x0 != x1):
            f = f + disty
            if (f >= distx):
                if (grid(x0 + (sx - 1)/2, y0 + (sy - 1)/2)):
                    return False
                y0 = y0 + sy
                f = f - distx
            if (f != 0 and grid(x0 + ((sx - 1)/2, y0 + (sy - 1)/2))):
                return False
            if (disty == 0 and grid(x0 + (sx - 1)/2, y0) and grid(x0 + (sx - 1)/2, y0 - 1)):
                return False
    else:
        while (y0 != y1):
            f = f + distx
            if (f >= disty):
                if (grid(x0 + (sx - 1)/2, y0 + (sy - 1)/2)):
                    return False
                x0 = x0 + sx
                f = f - disty
            if (f != 0 and grid(x0 + (sx - 1)/2, y0 + (sy - 1)/2)):
                return False
            if (distx == 0 and grid(x0, y0 + (sy - 1)/2) and grid(x0 - 1, y0 + (sy - 1)/2)):
                return False
            y0 = y0 + sy
    return True

def isTransitable(node, grid):
    """
    """
    x,y = point.grid_point




