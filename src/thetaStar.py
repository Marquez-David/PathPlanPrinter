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
            if node in closedset: #If it is already in the closed set, skip it
                continue
            if node in openset: #Otherwise if it is already in the open set
                if lineOfSight(current.parent, node):
                    if node.G > current.G + current.move_cost(node): #Check if we beat the G score -> update the node to have a new parent
                        node.G = current.parent.G + current.parent.move_cost(node)
                        node.parent = current.parent
                else:
                    if current.G + current.move_cost(node) < node.G:
                        node.G = current.G + current.move_cost(node)
                        node.parent = current
            else:
                #If it isn't in the open set, calculate the G and H score for the node
                node.G = current.G + current.move_cost(node)
                node.H = pp.heuristic[heur](node, goal)
                #Set the parent to our current item
                node.parent = current
                #Add it to the set
                openset.add(node)
    #Throw an exception if there is no path
    raise ValueError('No Path Found')
                
pp.register_search_method('theta*', thetaStar)

def lineOfSight(current, node):
    x0, y0 = current.point
    x1, y1 = node.point
    dist_y = y1 - y0
    dist_x = x1- x0
    f = 0
    sx = current.point[0]
    sy = current.point[1]

    if dist_y < 0:
        dist_y = -dist_y
        sy = -1
    else:
        sy = 1

    if dist_x < 0:
        dist_x = -dist_x
        sx = -1
    else:
        sx = 1
    
    if dist_x >= dist_y:
        while x0 != x1:
            f = f + dist_y
            if f >= dist_x:
                if grid(x0 + ((sx - 1)/2), y0 + ((sy - 1)/2)):
                    return False
                y0 = y0 + sy
                f = f - dist_x
            if f != 0 and grid(x0 + ((sx - 1)/2), y0 + ((sy - 1)/2)):
                return False
            if dist_y == 0 and grid(x0 + ((sx - 1)/2), y0) and grid(x0 + ((sy - 1)/2), y0 - 1):
                return False
    else:
        while y0 != y1:
            f = f + dist_x
            if f >= dist_y:
                if grid(x0 + ((sx - 1)/2), y0 + ((sy - 1)/2)):
                    return False
                x0 = x0 + sx
                f = f - dist_y
                if f != 0 and grid(x0 + ((sx - 1)/2), y0 + ((sy - 1)/2)):
                    return False
                if dist_x and grid(x0, y0 + ((sy - 1)/2)) and grid(x0 - 1, y0 + ((sy - 1)/2)):
                    return False
                y0 = y0 + sy
    return True
