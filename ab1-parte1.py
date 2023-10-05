#!pip install spatialmath-python
import numpy as np
import matplotlib.pyplot as plt
from spatialmath import SO2, SO3, SE2, SE3, UnitQuaternion, Twist3
from spatialmath.base import *
from mpl_toolkits.mplot3d.art3d import Poly3DCollection, Line3DCollection

def rot_2d(angle=0.5, p=np.array([0.4, 0.4])):
    """
    Rotaciona um ponto em 2D.
    """
    R = rot2(angle) # criando uma matriz de rotação SO(2)
    # trplot2(R) # visualizando como um frame rotacionado
    tranimate2(R) # animando a rotação
    plot_point(p, "ko", text="P")
    plotvol2([-1, 1, -1, 1]) # ajustando limites do plot
    plt.show()

def htransform_2d():
    """
    Matriz de transformação homogênea 2D
    """
    T0 = transl2(0, 0)
    TA = transl2(1, 2) @ trot2(30, "deg")
    TB = transl2(2, 1)
    TAB = TA@TB
    P = np.array([3, 2])

    plotvol2([0, 5]) # new plot with both axes from 0 to 5
    trplot2(T0, frame="O", color="k")
    trplot2(TA, frame="A", color="b")
    trplot2(TB, frame="B", color="r")
    trplot2(TAB, frame="AB", color="g")
    plot_point(P, "ko", text="P")

    plt.show()

def rot_frame():
    """
    Rotacionando um sistema de coordenadas
    """
    # %%
    plotvol2([-5, 4, -1, 5])
    T0 = transl2(0, 0)
    trplot2(T0, frame="0", color="k")
    TX = transl2(2, 3)
    trplot2(TX, frame="X", color="b")
    TR = trot2(2)
    trplot2(TR @ TX, frame="RX", color="g")
    trplot2(TX @ TR, frame="XR", color="g")

    # Rotacionando em torno de um ponto C

    C = np.array([3, 2])
    plot_point(C, "ko", text="C")
    TC = transl2(C) @ TR @ transl2(-C)
    trplot2(TC @ TX, frame="XC", color="r")

    plt.show()

def rot_3d():
    """
    Matriz de rotação 3D.
    """
    R = rotx(np.pi / 2) @ roty(np.pi / 2)
    # trplot(R)
    tranimate(R)
    plt.show()

def htransform_3d():
    """
    Matriz de transformação homogênea 3D.
    """
    T0 = transl(0, 0, 0)
    T = transl(3, 0, 0) @ trotx(np.pi / 2) @ transl(0, 4, 0)
    P = transl(1, 3, 1)

    trplot(T0, frame="O", color="k")
    trplot(T, frame="A", color="b")
    trplot(P, frame="P", color="r", length=0, axislabel=False)

    plt.show()

#########################questão 1#########################
angle = np.pi / 4
#Criação de uma matriz de rotação
rotation2D = np.matrix(rot2(angle))
print("\nMatriz rotação:")
print(rotation2D)

#Utilizar trplot2
trplot2(rotation2D)
#Transformação
vector = np.array([0, 1])
print("\nVetor:")
print(vector)

transVec = rotation2D @ vector
print("\nVetor resultante:")
print(transVec)

#invertento
inverse = np.linalg.inv(rotation2D)
multi = inverse @ rotation2D
print("\nMultiplicação da inversa pela original (os valores estão bem próximos da matriz identidade, usando a inversa da numpy)")
print(multi)

#3D
rotation3D = rotx(angle * 2) @ roty(angle)
print("\nMatriz de rotação 3d:")
print(rotation3D)
trplot(rotation3D)

vector_3d = np.array([1, 0, 1])
print("\nVetor 3d:")
print(vector_3d)

transVec_3d = rotation3D @ vector_3d
print("\nVetor resultante 3d:")
print(transVec_3d)

inverse = np.linalg.inv(rotation3D)
multi_3d = rotation3D @ inverse
print("\nMultiplicação da inversa pela original 3D (os valores estão bem próximos da matriz identidade, usando a inversa da numpy)")
print(multi_3d)

#########################questão 2#########################
def plot(Rx):
    trplot(Rx)
    plt.show()

Ra = rotx(0)
Ra1 = rotx(np.pi / 2)
Ra2 = Ra1 @ roty((np.pi / 2))
print("Inicial Ra:")
plot(Ra)
print("\nRotacionando em 90º em x (Ra)")
plot(Ra1)
print("\nRotacionando em 90º em y (Ra)")
plot(Ra2)

Rb = rotx(0)
Rb1 = roty(np.pi / 2)
Rb2 = Rb1 @ rotx(np.pi / 2)
print("\nInicial Rb:")
plot(Rb)
print("\nRotacionando em 90º em y (Rb)")
plot(Rb1)
print("\nRotacionando em 90º em x (Rb)")
plot(Rb2)

#########################questão 3#########################
fA = SO3(rotz(np.pi / 4))
fB = SO3(rotx(np.pi / 6) @ roty(np.pi / 9))
trplot(fA.R, color="r", frame="A")
trplot(fB.R, color="g", frame="B")
plt.show()

aRb = fA.inv() @ fB
bRa = fB.inv() @ fA
print("Matriz aRb:")
print(aRb.R)
print("\nMatriz bRa:")
print(bRa.R)
trplot(aRb.R, color="y", frame="aRb")
trplot(bRa.R, color="b", frame="bRa")
plt.show()

ang1, eix1 = aRb.angvec()
ang2, eix2 = bRa.angvec()
print("\nEixo-ângulo:")
print(ang1, eix1)
print(ang2, eix2)

print("\nOs valores são bem parecidos, pois ambas matrizes de rotação são apenas uma transposta da outra.\n\
Logo os pontos no eixo são invertidos, porém com o mesmo ângulo.")

#########################questão 4#########################
matrix = (transl(2, 0, 0) @ trotx(np.pi / 4) @ transl(0, 1, 0))
trplot(matrix) #NOTA: tranimate não anima no colab, fica travado (pelo que eu testei).
plt.show()
transform = matrix.T @ np.array([1, 0, 1, 1])

inverse = np.linalg.inv(matrix)
multi = matrix @ inverse
print("Multiplicando matrix pela a sua inversa:")
print(multi)

multi = inverse @ matrix
print("\nMultiplicando a inversa pela a original:")
print(multi)

print("\nOs valores são iguais (Note que a numpy não zera em certos locais da matriz, apenas deixa um valor bem próximo a zero).")

#########################questão 5#########################
matrix = SE3(transl(2, 0, 0) @ trotx(np.pi / 4) @ transl(0, 1, 0))
print("Matriz de transformação:")
print(matrix)

inverse = matrix.inv()
print("Matriz de inversa da transformação:")
print(inverse)

I = matrix @ inverse
print("Resutado da multiplicação:")
print(I)
print("Multiplicando ambas as matrizes a matriz original e a sua inversa,\n\
temos que o resultado usando a SE3 para cirar a matriz, temos que o resultado é a matriz identidade.")

#########################questão 6#########################
from IPython.display import display, clear_output

def animate_cube_rotation(ax, vertices, T):
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
    display(fig)              #Se estiver no colab
    clear_output(wait=True)   #Se estiver no colab
    #plt.draw()               #Se tiver no terminal
    #plt.pause(0.01)          #Se tiver no terminal


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

for angle in range(0, 360, 10):
    T = SE3.Rx(np.radians(angle))
    animate_cube_rotation(ax, vertices, T)

for angle in range(0, 360, 10):
    T = SE3.Rx(np.radians(angle)) @ SE3.Ry(np.radians(angle)) @ SE3.Rz(np.radians(angle))
    animate_cube_rotation(ax, vertices, T)
