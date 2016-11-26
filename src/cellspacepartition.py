'''Class to divide a 2D space into a grid of cells each of which
    may contain a number of entities. Once created and initialized
    with entities, fast proximity querys can be made by calling the
    CalculateNeighbors method with a position and proximity radius.

If an entity is capable of moving, and therefore capable of moving
    between cells, the Update method should be called each update-cycle
    to sychronize the entity and the cell space it occupies
'''
from collections.abc import Iterable
from collections import deque

from .invertedaabbox2d import InvertedAABBox2D
from .d2.vector2d import Vector2D


class Cell(Iterable):
    '''Cell(top_left, bottom_right) -> Cell

    Defines a cell containing a list of references to entities

    Keyword arguments:
    top_left -- Vector2D
    bottom_right -- Vector2D
    '''

    __slots__ = (
        '_bounding_box',
        '_members'
    )

    def __init__(self, top_left, bottom_right):
        self._bounding_box = InvertedAABBox2D(top_left, bottom_right)
        self._members = []

    @property
    def members(self):
        return self._members

    @property
    def bounding_box(self):
        return self._bounding_box

    def __iter__(self):
        return iter(self._members)




class CellSpacePartition(Iterable):
    '''CellSpacePartition(width, height, cells_x, cells_y, max_entities)
        -> CellSpacePartition

    Keyword arguments:
    width -- Real; width of 2D space
    height -- Real
    cells_x -- Int; number of cell divisions horizontally
    cells_y -- Int; number of cell divisions vertically
    max_entities -- Int; maximum number of entities to partition
    '''



    __slots__ = (
        '_cells',
        '_space_width',
        '_space_height',
        '_num_cells_x',
        '_num_cells_y',
        '_neighbors',
        '_cell_size_x',
        '_cell_size_y'
    )



    def __init__(self, width, height, cells_x, cells_y, max_entities):
        self._cells = []

        # the width and height of the world space the entities inhabit
        self._space_width = width
        self._space_height = height

        # the number of cells the space is going to be divided up into
        self._num_cells_x = cells_x
        self._num_cells_y = cells_y

        # this is used to store any valid neighbors when an agent searches
        # its neighboring space
        self._neighbors = deque(maxlen=max_entities)
        self._cell_size_x = width // cells_x
        self._cell_size_y = height // cells_y

        for y in range(self._num_cells_y):
            for x in range(self._num_cells_x):
                left = x * self._cell_size_x
                right = left + self._cell_size_x
                top = y * self._cell_size_y
                bot = top + self._cell_size_y

                self._cells.append(
                        Cell(Vector2D(left, top), Vector2D(right, bot))
                        )



    def _position_to_index(self, pos):
        '''CellSpacePartition._position_to_index(self, pos) -> Int

        Given a 2D vector representing a position within the game world, this
            method calculates an index into its appropriate cell.

        Keyword arguments:
        pos -- Vector2D
        '''
        idx = (int(self._num_cells_x * pos.x / self._space_width) +
                int(self._num_cells_y * pos.y / self._space_height) *
                self._num_cells_x)

        # if the entity's position is equal to Vector2D(m_dSpaceWidth,
        # m_dSpaceHeight) then the index will overshoot. We need to check for
        # this and adjust
        if idx > len(self._cells) - 1:
            idx = len(self._cells) - 1

        return idx



    def add_entity(self, entity):
        '''CellSpacePartition.add_entity(self, entity) -> None

        Used to add the entities to the data structure.
        Adds entities to the class by allocating them to the appropriate cell.

        Keyword arguments:
        entity -- BaseGameEntity
        '''
        if entity is None:
            raise ValueError('Attempting to insert None in entity parameter')

        idx = self._position_to_index(entity.position)
        self._cells[idx].members.append(entity)



    def update_entity(self, entity, old_pos):
        '''CellSpacePartition.update_entity(self, entity, old_pos) -> None

        Update an entity's cell by calling this from your entity's Update
            method.
        Checks to see if an entity has moved cells. If so the data structure
            is updated accordingly.

        Keyword arguments:
        entity -- BaseGameEntity
        old_pos -- Vector2D
        '''
        # if the index for the old pos and the new pos are not equal then
        # the entity has moved to another cell.
        old_idx = self._position_to_index(old_pos)
        new_idx = self._position_to_index(entity.position)

        if new_idx == old_idx:
            return

        # the entity has moved into another cell so delete from current cell
        # and add to new one
        self._cells[old_idx].members.remove(entity)
        self._cells[old_idx].members.append(entity)



    def calculate_neighbors(self, target_pos, query_radius):
        '''CellSpacePartition.calculate_neighbors(self, target_pos,
            query_radius) -> None

        This must be called to create the vector of neighbors.This method
            examines each cell within range of the target, If the cells
            contain entities then they are tested to see if they are situated
            within the target's neighborhood region. If they are they are
            added to neighbor list.
        This method stores a target's neighbors in the neighbor vector. After
            you have called this method use the begin, next and end methods to
            iterate through the vector.

        Keyword arguments:
        target_pos -- Vector2D
        query_radius -- Real
        '''
        top_left = target_pos - Vector2D(query_radius, query_radius)
        bottom_right = target_pos + Vector2D(query_radius, query_radius)
        query_box = InvertedAABBox2D(top_left, bottom_right)

        for cell in self._cells:
            if (cell.bounding_box.is_overlapped_with(query_box) and
                    cell.members):
                for entity in cell:
                    if (entity.position.distance_sq(target_pos) >
                            query_radius**2):
                        self._neighbors.append(entity)



    def empty_cells(self):
        '''CellSpacePartition.empty_cells(self) -> None

        Clears the cells of all entities.
        '''
        for cell in self._cells:
            cell.members.clear()



    def render(self):
        raise NotImplementedError



    def __iter__(self):
        '''Returns a iterator to all neighbors.
        '''
        return iter(self._neighbors)
