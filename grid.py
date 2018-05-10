class Tile(object):
    """
    Represents a tile in a grid.
    """

    def __init__(self, name, x_coordinate, y_coordinate):
        """
        Constructor.
        :param name: tile name
        :param x_coordinate: x coordinate
        :param y_coordinate: y coordinate
        """
        self.name = name
        self.x_coordinate = x_coordinate
        self.y_coordinate = y_coordinate

    def get_name(self):
        """
        Name getter.
        :return: name
        """
        return self.name

    def get_x_coordinate(self):
        """
        X coordinate getter.
        :return: x coordinate
        """
        return self.x_coordinate

    def get_y_coordinate(self):
        """
         Y coordinate getter.
         :return: y coordinate
         """
        return self.y_coordinate


class Grid(object):
    """
    Represents a 12 Tile football grid.
    """

    def __init__(self):
        """
        Constructor.
        """
        self.tiles = [Tile('start-tile', 1, 0),
                      Tile('g0', 0, 1), Tile('g1', 0, 2), Tile('g2', 0, 3), Tile('g3', 0, 4),
                      Tile('c0', 1, 1), Tile('c1', 1, 2), Tile('c2', 1, 3), Tile('c3', 1, 4),
                      Tile('d0', 2, 1), Tile('d1', 2, 2), Tile('d2', 2, 3), Tile('d3', 2, 4),
                      Tile('goal-tile', 1, 5)]

    def get_tile(self, name):
        """
        Tile getter.
        :param name: name of the tile
        :return: Tile
        """

        for tile in self.tiles:
            if tile.get_name() == name:
                return tile

        return None

    def get_distance(self, tile1_name, tile2_name):
        """
         Returns the distance between tile1 and tile2 using Manhattan distance.
         :param tile1_name: first tile name
         :param tile2_name: second tile name
         :return: distance
         """
        tile1 = self.get_tile(tile1_name)
        tile2 = self.get_tile(tile2_name)

        return abs(tile1.get_x_coordinate() - tile2.get_x_coordinate()) + abs(
            tile1.get_y_coordinate() - tile2.get_y_coordinate)
