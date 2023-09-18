import numpy as np
import matplotlib.pyplot as plt
from spatialmath import SO2, SO3, SE2, SE3, UnitQuaternion, Twist3
from spatialmath.base import *
from mpl_toolkits.mplot3d.art3d import Poly3DCollection, Line3DCollection
from IPython.display import clear_output

#NOTA: Não funciona bem no colab, apenas no terminal

def animate_cube_rotation(ax, T):
    ax.clear()
    ax.set_xlim([-1, 1])
    ax.set_ylim([-1, 1])
    ax.set_zlim([-1, 1])
    transformed_vertices = (T * vertices.T).T
    edges = [
        [transformed_vertices[0], transformed_vertices[1], transformed_vertices[5], transformed_vertices[4]],
        [transformed_vertices[1], transformed_vertices[2], transformed_vertices[6], transformed_vertices[5]],
        [transformed_vertices[2], transformed_vertices[3], transformed_vertices[7], transformed_vertices[6]],
        [transformed_vertices[3], transformed_vertices[0], transformed_vertices[4], transformed_vertices[7]],
        [transformed_vertices[0], transformed_vertices[1], transformed_vertices[2], transformed_vertices[3]]
    ]
    ax.add_collection3d(Poly3DCollection(edges, facecolors='cyan', linewidths=1, edgecolors='r', alpha=0.2))
    plt.draw()
    plt.pause(0.01)

fig = plt.figure()
ax = fig.add_subplot(111, projection='3d')

vertices = np.array([
    [-0.5, -0.5, -0.5],
    [ 0.5, -0.5, -0.5],
    [ 0.5,  0.5, -0.5],
    [-0.5,  0.5, -0.5],
    [-0.5, -0.5,  0.5],
    [ 0.5, -0.5,  0.5],
    [ 0.5,  0.5,  0.5],
    [-0.5,  0.5,  0.5]
])

for angle in range(0, 360, 5):
    T = SE3.Rx(np.radians(angle))
    animate_cube_rotation(ax, T)

for angle in range(0, 360, 5):
    T = SE3.Rx(np.radians(angle)) @ SE3.Ry(np.radians(angle)) @ SE3.Rz(np.radians(angle))
    animate_cube_rotation(ax, T)