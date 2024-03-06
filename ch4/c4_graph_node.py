


class GraphNode:
    def __init__(self, x: int, y: int, neighbours = []):
        self.x = x
        self.y = y
        self.neighbours = neighbours
        self.is_explored = False
        
    def add_neighbours(self, neighbours):
        # for n in neighbours:
        #     if self not in n.neighbours:
        #         n.neighbours.append(self)
        self.neighbours.extend(neighbours)
    
    def __eq__(self, other):
        return self.x == other.x and self.y == other.y
    
    def __str__(self) -> str:
        s =  f"GraphNode({self.x}, {self.y}) ->"
        for g in self.neighbours:
            s += str(g) + "->"
        return s
    
    def __repr__(self):
        return str(self)