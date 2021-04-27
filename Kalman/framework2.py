import numpy as np
import math
import matplotlib.pyplot as plt
import random
import homogeneous_transform as ht
import quaternion
from mpl_toolkits.mplot3d import Axes3D



def updateBoatFrame(boom, slew, boatframe, sensorframe):

    bf = boatframe
    sf = sensorframe
    #angles = sf.get_orientation()
    #print(angles)
    # Transforms the sensor pitch and roll values to be aligned with the boat
    rot_quat = ht.euler_to_quaternion([0, -boom, -slew])
    
    new_sensor_orientation = quaternion.multiply(quaternion.multiply(rot_quat, sf.get_quat()), quaternion.conjugate(rot_quat))
    
    
    sensor_angles = np.degrees(ht.quaternion_to_euler(new_sensor_orientation))
   
    #sensor_angles = np.degrees(quaternion.get_axis_angle(new_sensor_orientation))
    #print('Sensor angles: ', sensor_angles)
    # roll_offset and yaw_offset is collected from the gangway encoders
    bf.update_orientation([sensor_angles[0],
                                                sensor_angles[1],
                                                sensor_angles[2]])
    return bf                                    



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
    
    

if __name__ == '__main__':
    world_frame = Frame()
    sensor_frame = Frame(np.array([2, 2, 2]), orientation=np.array([0, 0, 0]))
    boat_frame = Frame(np.array([0, 0, 0]), orientation=np.array([0, 0, 0]))
    #boat_frame.set_parent_frame(sensor_frame)
    #sensor_frame.set_child_frame(boat_frame)
    #boat_frame = updateBoatFrame(0, 0, boatframe=boat_frame, sensorframe=sensor_frame)
    #print('Boat orientation in relation to the world: ', boat_frame.get_orientation())

    for i in range(0,2):
        fig = plt.figure()
        ax = fig.add_subplot(111, projection='3d')
        # Cartesian axes
        ax.quiver(-1, 0, 0, 3, 0, 0, color='#aaaaaa',linestyle='dashed')
        ax.quiver(0, -1, 0, 0, 3, 0, color='#aaaaaa',linestyle='dashed')
        ax.quiver(0, 0, -1, 0, 0, 3, color='#aaaaaa',linestyle='dashed')

        ax.set_xlim([-3, 3])
        ax.set_ylim([-3, 3])
        ax.set_zlim([-3, 3])

        ##labelling the axes
        ax.set_xlabel('X')
        ax.set_ylabel('Y')
        ax.set_zlabel('Z')
        # # Vector before rotation
        #print('Boat orientation: ', ht.rot_matrix_to_euler(boat_frame.get_representation()))
        #print('Sensor orientation: ', ht.rot_matrix_to_euler(sensor_frame.get_representation()))
        
        print('Boat orientation: ', boat_frame.get_orientation())
        print('Sensor orientation: ', sensor_frame.get_orientation())
        boat_pos = boat_frame.get_position()
        boat_repr = boat_frame.get_representation()
        ax.quiver(boat_repr[0, 3], boat_repr[1, 3], boat_repr[2, 3], boat_repr[0, 0], boat_repr[1, 0], boat_repr[2, 0], color='r', length=2)
        ax.quiver(boat_repr[0, 3], boat_repr[1, 3], boat_repr[2, 3], boat_repr[0, 1], boat_repr[1, 1], boat_repr[2, 1], color='g', length=2)
        ax.quiver(boat_repr[0, 3], boat_repr[1, 3], boat_repr[2, 3], boat_repr[0, 2], boat_repr[1, 2], boat_repr[2, 2], color='b', length=2)
        #fig.canvas.draw()
        sensor_repr = sensor_frame.get_representation()
        ax.quiver(sensor_repr[0, 3], sensor_repr[1, 3], sensor_repr[2, 3], sensor_repr[0, 0], sensor_repr[1, 0], sensor_repr[2, 0], color='r', length=1)
        ax.quiver(sensor_repr[0, 3], sensor_repr[1, 3], sensor_repr[2, 3], sensor_repr[0, 1], sensor_repr[1, 1], sensor_repr[2, 1], color='g', length=1)
        ax.quiver(sensor_repr[0, 3], sensor_repr[1, 3], sensor_repr[2, 3], sensor_repr[0, 2], sensor_repr[1, 2], sensor_repr[2, 2], color='b', length=1)
        # ax.quiver(boat_repr[0, 3], boat_repr[1, 3], boat_repr[2, 3], sensor_repr[0, 0], sensor_repr[1, 0], sensor_repr[2, 0], color='r', length=1)
        # ax.quiver(boat_repr[0, 3], boat_repr[1, 3], boat_repr[2, 3], sensor_repr[0, 1], sensor_repr[1, 1], sensor_repr[2, 1], color='g', length=1)
        # ax.quiver(boat_repr[0, 3], boat_repr[1, 3], boat_repr[2, 3], sensor_repr[0, 2], sensor_repr[1, 2], sensor_repr[2, 2], color='b', length=1)
        
        #sensor_frame.update_orientation([random.randint(-15, 15), random.randint(-15, 15), random.randint(-15, 15)])
        
        boom = 8 #random.randint(-15, 15)
        slew = 40 #random.randint(-15, 15)
        
        sensor_frame.update_orientation([-4, 10, 0])
        
        new_boat_frame = updateBoatFrame(boom, slew, boat_frame, sensor_frame)
        boat_frame = new_boat_frame
        #boat_frame = updateBoatFrame(random.randint(-15, 15), random.randint(-45, 45), boat_frame, sensor_frame)

        # if i > 5:
        #     sensor_frame.update_position([1, 1, 1])
        #     sensor_frame.update_orientation([90, 90, 90])
        
        # boat_frame.update_orientation(new_orientation)
        # boat_frame.update_position(new_position)
        plt.show()
