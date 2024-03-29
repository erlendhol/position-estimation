import numpy as np
import matplotlib.pyplot as plt

from threading import Thread
import time
from sys import stdout

class Quaternion:
    """Quaternions for 3D rotations"""
    def __init__(self, x):
        self.x = np.asarray(x, dtype=float)

    @classmethod
    def from_v_theta(cls, v, theta):
        """
        Construct quaternion from unit vector v and rotation angle theta
        """
        theta = np.asarray(theta)
        v = np.asarray(v)

        s = np.sin(0.5 * theta)
        c = np.cos(0.5 * theta)
        vnrm = np.sqrt(np.sum(v * v))

        q = np.concatenate([[c], s * v / vnrm])
        return cls(q)

    def __repr__(self):
        return "Quaternion:\n" + self.x.__repr__()

    def __mul__(self, other):
        # multiplication of two quaternions.
        prod = self.x[:, None] * other.x

        return self.__class__([(prod[0, 0] - prod[1, 1]
                                 - prod[2, 2] - prod[3, 3]),
                                (prod[0, 1] + prod[1, 0]
                                 + prod[2, 3] - prod[3, 2]),
                                (prod[0, 2] - prod[1, 3]
                                 + prod[2, 0] + prod[3, 1]),
                                (prod[0, 3] + prod[1, 2]
                                 - prod[2, 1] + prod[3, 0])])

    def as_v_theta(self):
        """Return the v, theta equivalent of the (normalized) quaternion"""
        # compute theta
        norm = np.sqrt((self.x ** 2).sum(0))
        theta = 2 * np.arccos(self.x[0] / norm)

        # compute the unit vector
        v = np.array(self.x[1:], order='F', copy=True)
        v /= np.sqrt(np.sum(v ** 2, 0))

        return v, theta

    def as_rotation_matrix(self):
        """Return the rotation matrix of the (normalized) quaternion"""
        v, theta = self.as_v_theta()
        c = np.cos(theta)
        s = np.sin(theta)

        return np.array([[v[0] * v[0] * (1. - c) + c,
                          v[0] * v[1] * (1. - c) - v[2] * s,
                          v[0] * v[2] * (1. - c) + v[1] * s],
                         [v[1] * v[0] * (1. - c) + v[2] * s,
                          v[1] * v[1] * (1. - c) + c,
                          v[1] * v[2] * (1. - c) - v[0] * s],
                         [v[2] * v[0] * (1. - c) - v[1] * s,
                          v[2] * v[1] * (1. - c) + v[0] * s,
                          v[2] * v[2] * (1. - c) + c]])

x, y, z = np.eye(3)

class CubeAxes(plt.Axes):
    """An Axes for displaying a 3D cube"""
    # fiducial face is perpendicular to z at z=+1
    one_face = np.array([[1, 1, 1], [1, -1, 1], [-1, -1, 1], [-1, 1, 1], [1, 1, 1]])

    # construct six rotators for the face
    #x, y, z = np.eye(3)
    rots = [Quaternion.from_v_theta(x, theta) for theta in (np.pi / 2, -np.pi / 2)]
    rots += [Quaternion.from_v_theta(y, theta) for theta in (np.pi / 2, -np.pi / 2)]
    rots += [Quaternion.from_v_theta(y, theta) for theta in (np.pi, 0)]

    # colors of the faces
    colors = ['blue', 'green', 'white', 'yellow', 'orange', 'red']

    def __init__(self, fig, rect=[0, 0, 1, 1], *args, **kwargs):
        # We want to set a few of the arguments
        kwargs.update(dict(xlim=(-2.5, 2.5), ylim=(-2.5, 2.5), frameon=False,
                           xticks=[], yticks=[], aspect='equal'))
        super(CubeAxes, self).__init__(fig, rect, *args, **kwargs)
        self.xaxis.set_major_formatter(plt.NullFormatter())
        self.yaxis.set_major_formatter(plt.NullFormatter())

        # define the current rotation
        self.current_rot = Quaternion.from_v_theta((1, 1, 0), np.pi / 6)


    def draw_cube(self):
        """draw a cube rotated by theta around the given vector"""
        # rotate the six faces
        Rs = [(self.current_rot * rot).as_rotation_matrix() for rot in self.rots]
        faces = [np.dot(self.one_face, R.T) for R in Rs]

        # project the faces: we'll use the z coordinate
        # for the z-order
        faces_proj = [face[:, :2] for face in faces]
        zorder = [face[:4, 2].sum() for face in faces]

        # create the polygons if needed.
        # if they're already drawn, then update them
        if not hasattr(self, '_polys'):
            self._polys = [plt.Polygon(faces_proj[i], fc=self.colors[i],
                                       alpha=0.9, zorder=zorder[i])
                           for i in range(6)]
            for i in range(6):
                self.add_patch(self._polys[i])
        else:
            for i in range(6):
                self._polys[i].set_xy(faces_proj[i])
                self._polys[i].set_zorder(zorder[i])

        self.figure.canvas.draw()

class CubeAxesInteractive(CubeAxes):
    """An Axes for displaying an Interactive 3D cube"""
    def __init__(self, *args, **kwargs):
        super(CubeAxesInteractive, self).__init__(*args, **kwargs)

        # define axes for Up/Down motion and Left/Right motion
        self._v_LR = (0, 1, 0)
        self._v_UD = (-1, 0, 0)
        self._active = False
        self._xy = None

        self.v, self.theta = self.current_rot.as_v_theta()

        # connect some GUI events
        self.figure.canvas.mpl_connect('button_press_event',
                                       self._mouse_press)
        self.figure.canvas.mpl_connect('button_release_event',
                                       self._mouse_release)
        self.figure.canvas.mpl_connect('motion_notify_event',
                                       self._mouse_motion)
        self.figure.canvas.mpl_connect('key_press_event',
                                       self._key_press)
        self.figure.canvas.mpl_connect('key_release_event',
                                       self._key_release)

        self.figure.text(0.05, 0.05, ("Click and Drag to Move\n"
                                    "Hold 'r' key to adjust rotation"))

        #self.figure.text(0.05, 0.9, ("Theta:", self.theta))

        self.printThread = Thread(target=self._print_matrix, args=())
        self.printThread.daemon = True
        self.printThread.start()

    #----------------------------------------------------------
    # when the 'r' button is down, change the rotation axis
    # of left-right movement
    def _key_press(self, event):
        """Handler for key press events"""
        if event.key == 'r':
            self._v_LR = (0, 0, -1)

    def _key_release(self, event):
        """Handler for key release event"""
        if event.key == 'r':
            self._v_LR = (0, 1, 0)

    #----------------------------------------------------------
    # while the mouse button is pressed, set state to "active"
    # so that motion event will rotate the plot
    def _mouse_press(self, event):
        """Handler for mouse button press"""
        if event.button == 1:
            self._active = True
            self._xy = (event.x, event.y)

    def _mouse_release(self, event):
        """Handler for mouse button release"""
        if event.button == 1:
            self._active = False
            self._xy = None

    def _mouse_motion(self, event):
        """Handler for mouse motion"""
        if self._active:
            dx = event.x - self._xy[0]
            dy = event.y - self._xy[1]
            self._xy = (event.x, event.y)

            rot1 = Quaternion.from_v_theta(self._v_UD, 0.01 * dy)
            rot2 = Quaternion.from_v_theta(self._v_LR, 0.01 * dx)

            self.current_rot = (rot2 * rot1 * self.current_rot)
            self.draw_cube()

    def _print_matrix(self):
        while(True):
            print("Rotation Matrix: \n", self.current_rot.as_rotation_matrix())
            self.v, self.theta = self.current_rot.as_v_theta()
            print("Rotation vector: ", self.v)
            print("Theta = ", self.theta)
            time.sleep(0.2)


fig = plt.figure(figsize=(4, 4))
ax = CubeAxesInteractive(fig)
fig.add_axes(ax)
ax.draw_cube()
plt.show()
