import numpy as np
import math
import matplotlib.pyplot as plt
import random
import homogeneous_transform as ht
import quaternion
from mpl_toolkits.mplot3d import Axes3D


class Frame(object):


    def __init__(self, position = np.array([0, 0, 0]), orientation = np.array([0, 0, 0]), parentFrame = None, childFrame = None):
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

    def get_quat(self):
        return ht.euler_to_quaternion(self.get_orientation())

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
