"""
Inicializa o workspace do robotics toolbox no iPython.
"""
import math as m
import numpy as np
import matplotlib.pyplot as plt
from roboticstoolbox import ctraj, DHRobot, RevoluteDH, PrismaticDH
from spatialmath import SE3


def cross_product(a: np.ndarray, b: np.ndarray) -> np.ndarray:
    return np.cross(a, b)

def rr_robot(L1=1, L2=1):
    return DHRobot([RevoluteDH(a = L1), RevoluteDH(a = L2)], name = 'RR')

def jacobian(q1, q2, L1=1, L2=1):
    x = L1 * m.cos(q1) + L2 * m.cos(q1 + q2)
    y = L1 * m.sin(q1) + L2 * m.sin(q1 + q2)

    J11 = - L1 * m.sin(q1) - L2 * m.sin(q1 + q2)
    J12 = - L2 * m.sin(q1 + q2)
    J21 =   L1 * m.cos(q1) + L2 * m.cos(q1 + q2)
    J22 =   L2 * m.cos(q1 + q2)

    J = np.array([[J11, J12], [J21, J22]])
    return J

def resolved_rate_control_2r():
    rr = rr_robot()
    q0 = np.array([-np.pi / 3, np.pi / 2])
    TE1 = rr.fkine(q0)
    TE2 = SE3.Trans(-0.5, 0.5, 0) @ TE1
    t = np.arange(0, 2, 0.02)
    Ts = ctraj(TE1, TE2, t)
    q = np.zeros((len(t), 2))

    q1 = np.pi / 4
    q2 = np.pi / 4
    dt = 0.01
    posic_desejada = np.array([0, 2]) 
    L1 = 1
    L2 = 1
    erro_x = []
    erro_y = []
    tempo = []
    for i in range(1000):
        pos_atual = np.array([L1 * np.cos(q1) + L2 * np.cos(q1 + q2), L1 * np.sin(q1) + L2 * np.sin(q1 + q2)])
        erro_x.append(posic_desejada[0] - pos_atual[0])
        erro_y.append(posic_desejada[1] - pos_atual[1])
        tempo.append(dt*i)
        J = jacobian(q1, q2)
        velocities = np.linalg.pinv(J).dot([erro_x[-1], erro_y[-1]])
        q1 += velocities[0] * dt
        q2 += velocities[1] * dt
        erro = m.sqrt(erro_x[-1]**2 + erro_y[-1]**2)
        if erro < 0.01:
            print("Alcançou a posição desejada.")
            break
    rr.teach([q1, q2])
    plt.plot(tempo, erro_x)
    plt.title("Erro de X em função do tempo")
    plt.xlabel("Tempo")
    plt.ylabel("Erro de X")
    plt.show()
    plt.title("Erro de Y em função do tempo")
    plt.xlabel("Tempo")
    plt.ylabel("Erro de Y")
    plt.plot(tempo, erro_y)
    plt.show()
    pos = rr.fkine(q0).t[:2]
    print(np.linalg.norm(pos - Ts[-1].t[:2]))

def rrr_robot(L1=1, L2=1, L3=1):
    return DHRobot([RevoluteDH(a = L1), RevoluteDH(a = L2), RevoluteDH(a = L3)], name = 'RRR')


def T01(q1, L1 = 1):
    T01 = np.array([[m.cos(q1), -m.sen(q1), 0, L1*m.cos(q1)], 
                    [m.sin(q1),  m.cos(q1), 0, L1*m.sin(q1)],
                    [0        ,  0        , 1, 0           ],
                    [0        ,  0        , 0, 1           ],])
    return T01


def T12(q2, L2 = 1):
    T12 = np.array([[m.cos(q2), -m.sen(q2), 0, L2*m.cos(q2)], 
                    [m.sin(q2),  m.cos(q2), 0, L2*m.sin(q2)],
                    [0        ,  0        , 1, 0           ],
                    [0        ,  0        , 0, 1           ],])
    return T12


def T23(q3, L3 = 1):
    T01 = np.array([[m.cos(q3), -m.sen(q3), 0, L3*m.cos(q3)], 
                    [m.sin(q3),  m.cos(q3), 0, L3*m.sin(q3)],
                    [0        ,  0        , 1, 0           ],
                    [0        ,  0        , 0, 1           ],])
    return T23


def geometric_jacobian(q):
    L1 = 1
    L2 = 1
    L3 = 1
    x3 = L1 * np.cos(q[0])
    y3 = L1 * np.sin(q[0])
    x2 = x3 + L2 * np.cos(q[0] + q[1])
    y2 = y3 + L2 * np.sin(q[0] + q[1])
    x1 = x2 + L3 * np.cos(q[0] + q[1] + q[2])
    y1 = y2 + L3 * np.sin(q[0] + q[1] + q[2])
    JP1 = np.array([-y1, x1])
    JP2 = np.array([-y2, x2])
    JP3 = np.array([-y3, x3])
    return np.array([JP1, JP2, JP3, ]).T


def resolved_rate_control_3r():
    rrr = rrr_robot()
    q0 = np.array([-np.pi / 3, np.pi / 2, np.pi / 2])
    TE1 = rrr.fkine(q0)
    TE2 = SE3.Trans(-0.5, 0.5, 0) @ TE1
    t = np.arange(0, 2, 0.02)
    Ts = ctraj(TE1, TE2, t)
    q = np.zeros((len(t), 2))

    q1 = np.pi / 4
    q2 = np.pi / 4
    q3 = 0
    dt = 0.01
    posic_desejada = np.array([2, 1]) 
    L1 = 1
    L2 = 1
    L3 = 1
    erro_x = []
    erro_y = []
    tempo = []
    for i in range(1000):
        pos_atual = np.array([L1 * np.cos(q1) + L2 * np.cos(q1 + q2) + L3 * np.cos(q1 + q2 + q3), L1 * np.sin(q1) + L2 * np.sin(q1 + q2) + L3 * np.sin(q1 + q2 + q3)])
        erro_x.append(posic_desejada[0] - pos_atual[0])
        erro_y.append(posic_desejada[1] - pos_atual[1])
        tempo.append(dt*i)
        J = geometric_jacobian([q1, q2, q3])
        velocities = np.linalg.pinv(J).dot([erro_x[-1], erro_y[-1]])
        q1 += velocities[0] * dt
        q2 += velocities[1] * dt
        q3 += velocities[2] * dt
        erro = m.sqrt(erro_x[-1]**2 + erro_y[-1]**2)
        if erro < 0.01:
            print("Alcançou a posição desejada.")
            break
    rrr.teach([q1, q2, q3])
    plt.plot(tempo, erro_x)
    plt.title("Erro de X em função do tempo")
    plt.xlabel("Tempo")
    plt.ylabel("Erro de X")
    plt.show()
    plt.title("Erro de Y em função do tempo")
    plt.xlabel("Tempo")
    plt.ylabel("Erro de Y")
    plt.plot(tempo, erro_y)
    plt.show()

def geometric_jacobian_scara(q):
    L1 = 1
    L2 = 1
    return np.array([[-L1*m.sin(q[0]) - L2*m.sin(q[0]+q[1]), L1*m.cos(q[0]) + L2*m.cos(q[0] + q[1]), 0],
                     [-L2*m.sin(q[0] + q[1]), L2*m.cos(q[0] + q[1]), 0],
                     [0, 0, 0]])

def questao3():
    L1 = 1
    L2 = 1
    d1 = 0.1
    d3 = 0.3
    d4 = 0.3

    revL1 = RevoluteDH(a = L1)
    revL2 = RevoluteDH(a = L2, alpha = np.pi)
    priDH = PrismaticDH(qlim = [0, 1])
    revD4 =  RevoluteDH(d = d4)
    rob = DHRobot([revL1, revL2, priDH, revD4], name = 'RRPR')

    q1 = 0
    q2 = 0
    q4 = 0
    dt = 0.01
    posic_desejada = np.array([1, 0, 1 - d3 - d4 + d1]) 
    erro_x = []
    erro_z = []
    tempo = []
    for i in range(1000):
        pos_atual = np.array([L1 * np.cos(q1) + L2 * np.cos(q1 + q2), 0, 1 - d3 - d4 + d1])
        erro_x.append(posic_desejada[0] - pos_atual[0])
        erro_z.append(posic_desejada[1] - pos_atual[1])
        tempo.append(dt*i)
        J = geometric_jacobian_scara([q1, q2, q4])
        velocities = np.linalg.pinv(J).dot([erro_x[-1], 0, erro_z[-1]])
        q1 += velocities[0] * dt
        q2 += velocities[1] * dt
        q4 += velocities[2] * dt
        erro = m.sqrt(erro_x[-1]**2 + erro_z[-1]**2)
        if erro < 0.01:
            print("Alcançou a posição desejada.")
            break
    rob.teach([q1, q2, 0, q4])
    plt.plot(tempo, erro_x)
    plt.title("Erro de X em função do tempo")
    plt.xlabel("Tempo")
    plt.ylabel("Erro de X")
    plt.show()
    plt.title("Erro de Z em função do tempo")
    plt.xlabel("Tempo")
    plt.ylabel("Erro de Z")
    plt.plot(tempo, erro_z)
    plt.show()


##################questão 1##################
#resolved_rate_control_2r()

##################questão 2##################
#resolved_rate_control_3r()

##################questão 3##################
questao3()