import numpy as np
import math
import matplotlib.pyplot as plt
import random
import homogeneous_transform as ht
from mpl_toolkits.mplot3d import Axes3D

class Frame(object):
    
    
    def __init__(self, position = np.array([[0, 0, 0]]), orientation = np.array([[0, 0, 0]]), parentFrame = None, childFrame = None):
        """
        Creates an instance of the Frame class.
        :param position: Position in relation to the parent frame
        :param orientation: Orientation in relation to the parent frame
        """
        self.position = position
        self.orientation = orientation
        self.parentFrame = parentFrame
        self.childFrame = childFrame
        self.representation = None
    
    def get_position(self):
        return self.position

    def get_orientation(self):
        return self.orientation
    
    def update_position(self, newPosition):
        """
        Updates the position in relation to the parent frame
        :param newPosition: New position
        """
        self.position = newPosition

    def update_orientation(self, newOrientation):
        """
        Updates the orientation in relation to the parent frame
        """
        self.orientation = newOrientation

    def set_parent_frame(self, parentFrame):
        self.parentFrame = parentFrame

    def set_child_frame(self, childFrame):
        self.childFrame = childFrame
    
    def get_parent_frame(self):
        return self.parentFrame

    def get_child_frame(self):
        return self.childFrame

    def get_rot_matrix(self):
        return ht.euler_to_rot_matrix(self.orientation)

    def get_trans_matrix(self):
        return ht.pos_to_trans_matrix(self.position)

    def get_representation(self):
        return np.matmul(ht.pos_to_trans_matrix(self.get_position()), ht.euler_to_rot_matrix(self.get_orientation()))

    def get_parent_representation(self):
        parent_representation = None
        if self.parentFrame is not None:
            parent_pos = self.parentFrame.get_position()
            parent_orient = self.parentFrame.get_orientation()
            parent_representation = np.matmul(ht.pos_to_trans_matrix(parent_pos), ht.euler_to_rot_matrix(parent_orient))
        return parent_representation
    
    def get_grandparent_representation(self):
        grandparent_representation = None
        if self.parentFrame.get_parent_frame() is not None:
            parent_representation = self.get_parent_representation()
            self.representation = self.get_representation()
            grandparent_representation = np.matmul(parent_representation, self.representation)
        return grandparent_representation


if __name__ == '__main__':
    worldFrame = Frame()
    boatPos = np.array([1, 2, 3])
    boatOrientation = np.array([90, 0, 0])
    boatFrame = Frame(position=boatPos, orientation=boatOrientation, parentFrame=worldFrame)
    tipPos = np.array([2, 4, 6])
    tipOrientation = np.array([13, 73, 52])
    tipFrame = Frame(position=tipPos, orientation=tipOrientation, parentFrame=boatFrame)
    boatFrame.set_child_frame(tipFrame)
    print('Tip orientation: ', tipFrame.get_orientation())
    print('Boat representation: ', boatFrame.get_representation())
    print('Boat representation: ', tipFrame.get_parent_representation())
    print('Boat orientation: ', ht.rot_matrix_to_euler(tipFrame.get_parent_representation()))
    print('World representation: ', tipFrame.get_grandparent_representation())
    print(boatFrame.get_grandparent_representation())
    print('Tip orientation in relation to the world: ', ht.rot_matrix_to_euler(tipFrame.get_grandparent_representation()))
    #boatFrame.update_orientation(np.array([14, -48, 56]))
    print('New boat orientation: ', boatFrame.get_orientation())
    print('New tip orientation in relation to the boat: ', tipFrame.get_orientation())
    print('New tip orientation in relation to the world: ', ht.rot_matrix_to_euler(tipFrame.get_grandparent_representation()))


    for i in range(0,10):
        fig = plt.figure()
        ax = fig.add_subplot(111, projection='3d')
        # Cartesian axes
        ax.quiver(-1, 0, 0, 3, 0, 0, color='#aaaaaa',linestyle='dashed')
        ax.quiver(0, -1, 0, 0, 3, 0, color='#aaaaaa',linestyle='dashed')
        ax.quiver(0, 0, -1, 0, 0, 3, color='#aaaaaa',linestyle='dashed')

        ax.set_xlim([-10, 10])
        ax.set_ylim([-10, 10])
        ax.set_zlim([-10, 10])

        ##labelling the axes
        ax.set_xlabel('X')
        ax.set_ylabel('Y')
        ax.set_zlabel('Z')
        # Vector before rotation
        print('Boat orientation: ', boatFrame.get_orientation())
        boat_pos = boatFrame.get_position()
        boat_repr = boatFrame.get_representation()
        ax.quiver(boat_repr[0, 3], boat_repr[1, 3], boat_repr[2, 3], boat_repr[0, 0], boat_repr[1, 0], boat_repr[2, 0], color='r', length=2)
        ax.quiver(boat_repr[0, 3], boat_repr[1, 3], boat_repr[2, 3], boat_repr[0, 1], boat_repr[1, 1], boat_repr[2, 1], color='g', length=2)
        ax.quiver(boat_repr[0, 3], boat_repr[1, 3], boat_repr[2, 3], boat_repr[0, 2], boat_repr[1, 2], boat_repr[2, 2], color='b', length=2)
        #fig.canvas.draw()
        tip_repr = tipFrame.get_grandparent_representation()
        ax.quiver(tip_repr[0, 3], tip_repr[1, 3], tip_repr[2, 3], tip_repr[0, 0], tip_repr[1, 0], tip_repr[2, 0], color='r', length=1)
        ax.quiver(tip_repr[0, 3], tip_repr[1, 3], tip_repr[2, 3], tip_repr[0, 1], tip_repr[1, 1], tip_repr[2, 1], color='g', length=1)
        ax.quiver(tip_repr[0, 3], tip_repr[1, 3], tip_repr[2, 3], tip_repr[0, 2], tip_repr[1, 2], tip_repr[2, 2], color='b', length=1)
        new_orientation = np.array([random.randint(-90, 90), random.randint(-90, 90), random.randint(-90, 90)])
        new_position = np.array([random.randint(-10, 10), random.randint(-10, 10), random.randint(-10, 10)])

        if i > 5:
            tipFrame.update_position([1, 1, 1])
            tipFrame.update_orientation([90, 90, 90])
        
        boatFrame.update_orientation(new_orientation)
        boatFrame.update_position(new_position)
        plt.show()

    # representation_wb = np.matmul(boatFrame.get_trans_matrix(), boatFrame.get_rot_matrix())
    # representation_bt = np.matmul(tipFrame.get_trans_matrix(), tipFrame.get_rot_matrix())
    # representation_wt = np.matmul(representation_wb, representation_bt)
    # print(representation_wt)
    # print(ht.get_orientation(representation_wt))
    # print(ht.get_pos(representation_wt))