import math as m
import numpy as np
import matplotlib.pyplot as plt
from spatialmath.base import *
from roboticstoolbox import DHRobot, RevoluteDH, ET, ERobot, models, PrismaticDH

def questao1():
    L1 = 1
    L2 = 1

    ############################Letra A############################
    points_L1_x = []
    points_L1_y = []
    points_L2_x = []
    points_L2_y = []

    samples = 25
    ang1 = np.linspace(0, np.pi, samples)
    ang2 = np.linspace(-np.pi/2, np.pi, samples)

    for i in range(samples):
        x1 = 0
        x2 = 0
        for j in range(11):
            x1 = ((L1 * j) / 10) * np.cos(ang1[i])
            y1 = ((L1 * j) / 10) * np.sin(ang1[i])
            points_L1_x.append(x1)
            points_L1_y.append(y1)
        for k in range(i, samples, 1):
            for j in range(11):
                x2 = x1 + ((L2 * j) / 10) * np.cos(ang2[k])
                y2 = y1 + ((L2 * j) / 10) * np.sin(ang2[k])
                points_L2_x.append(x2)
                points_L2_y.append(y2)

    first = plt.figure().add_subplot(111)
    first.scatter(points_L1_x, points_L1_y, label = "Q1", marker = '.', s = 5, c='r')
    first.scatter(points_L2_x, points_L2_y, label = "Q2", marker = '.', s = 5, c='b')

    plt.title("A: Área de trabalho dos movimentos")
    plt.show()

    ############################Letra B############################
    def ikine_rr(pose):
        T = transl(pose)
        quad = (T[0][3]**2 + T[1][3]**2 - L1**2 - L2**2) / (2 * L1 * L2)
        if abs(quad) > 1:
            print("Sem solução.")
            return
        
        fr = m.sqrt(1 - quad**2)
        atan = m.atan2(fr, quad)
        k1 = (L1 + L2 * quad)
        k2 = (L2 * fr)
        g = m.atan2(k2, k1)
        r = m.sqrt(k1**2 + k2**2)
        k1 = r * m.cos(g)
        k2 = r * m.sin(g)
        O1 = m.atan2(T[1][3], T[0][3]) - m.atan2(k2, k1)
        
        print("Raizes:")
        if 0 <= O1 and O1 <= np.pi and (-np.pi / 2) <= atan and atan <= np.pi:
            print("Raiz 1: ")
            print("O1:", O1, "O2:", atan)
            rob = DHRobot([RevoluteDH(d = 0, a = 1), RevoluteDH(a = 1)], name = 'RR')
            rob.teach([O1, atan])
        else:
            print("Raiz 1: fora do escopo.")

        atan = m.atan2(-fr, quad)
        k1 = (L1 + L2 * quad)
        k2 = (L2 * (-fr))
        g = m.atan2(k2, k1)
        r = m.sqrt(k1**2 + k2**2)
        k1 = r * m.cos(g)
        k2 = r * m.sin(g)
        O1 = m.atan2(T[1][3], T[0][3]) - m.atan2(k2, k1)

        if 0 <= O1 and O1 <= np.pi and (-np.pi / 2) <= atan and atan <= np.pi:
            print("Raiz 2: ")
            print("O1:", O1, "O2:", atan)
            rob = DHRobot([RevoluteDH(d = 0, a = 1), RevoluteDH(a = 1)], name = 'RR')
            rob.teach([O1, atan])
        else:
            print("Raiz 2: fora do escopo.")
    
    ikine_rr([1, 1, 0])

def questao2():
    x = 0
    y = 0.5
    z = -0.5
    L1 = 1
    L2 = 1

    ############################Letra A############################
    points_L1_x = []
    points_L1_y = []
    points_L1_z = []
    points_L2_x = []
    points_L2_y = []
    points_L2_z = []

    samples = 8

    angq1 = np.linspace(0, np.pi, samples)
    angq2 = np.linspace(0, np.pi, samples)
    angq3 = np.linspace(0, np.pi, samples)

    for i in range(samples):
        for j in range(samples):
            x = 0
            y = 0
            z = 0
            for l in range(11):
                x = - ((L1 * l) / 10)*m.sin(angq2[j])*m.cos(angq1[i])
                y =   ((L1 * l) / 10)*m.sin(angq2[j])*m.sin(angq1[i])
                z = - ((L1 * l) / 10)*m.cos(angq2[j])
                points_L1_x.append(x)
                points_L1_y.append(y)
                points_L1_z.append(z)
            aux_x = x
            aux_y = y
            aux_z = z
            for k in range(samples):
                for l in range(11):
                    x = aux_x - ((L2 * l) / 10)*m.sin(angq2[j])*m.cos(angq3[k])
                    y = aux_y + ((L2 * l) / 10)*m.sin(angq2[j])*m.sin(angq3[k])
                    z = aux_z - ((L2 * l) / 10)*m.cos(angq2[j])
                    points_L2_x.append(x)
                    points_L2_y.append(y)
                    points_L2_z.append(z)

    second = plt.figure().add_subplot(111, projection = '3d')

    j1_x = L1 * np.cos(angq1)
    j1_y = L1 * np.sin(angq1)
    second.plot(j1_x, j1_y, 0, label='J1')

    j2_x = L2 * np.cos(angq2)
    j2_y = np.zeros_like(angq2)
    j2_z = L2 * np.sin(angq2)
    second.plot(j2_x, j2_y, j2_z, label='J2')

    j3_x = (L1 + L2) * np.cos(angq3)
    j3_y = (L1 + L2) * np.sin(angq3)
    second.plot(j3_x, j3_y, 0, label='J3')

    second.scatter(points_L1_x, points_L1_y, points_L1_z, marker = '.', s = 5, c='r')
    second.scatter(points_L2_x, points_L2_y, points_L2_z, marker = '.', s = 5, c='b')

    plt.title("A: Área de trabalho dos movimentos")
    plt.show()

    ############################Letra B############################
    def funcao(x, y, z, L1, L2):
        radius = m.sqrt(z**2 + y**2)
        cos3 = (y**2 + z**2 - L1**2 - L2**2) / (2 * L1 * L2)
        cos2 = (y**2 + z**2 + L1**2 - L2**2) / (2 * L1 * radius) if radius != 0 else 0

        if abs(cos3) > 1 or abs(cos2) > 1:
            print("Valor fora dos limites. Valor absoluto do cosseno entre os elos acima de 1")
            return

        sen3 = m.sqrt(1 - cos3**2)
        r31 = m.atan2(sen3, cos3) + np.pi/2
        r32 = m.atan2(-sen3, cos3) + np.pi/2
        sen2 = m.sqrt(1 - cos2**2)
        r21 = m.atan2(z, y) - m.atan2(sen2, cos2)
        r22 = m.atan2(z, y) + m.atan2(sen2, cos2)
        r11 = m.atan2(y, x) - np.pi/2

        if abs(r11) > np.pi/2 or abs(r21) > np.pi/2 or abs(r31) > np.pi/2:
            print("Solução 1:")
            print('θ1 =', r11, 'θ2 =', r21, 'θ3 =', r31)
            e1 = RevoluteDH(d = 0, alpha = np.pi / 2, offset = np.pi / 2)
            e2 = RevoluteDH(a = L1)
            e3 = RevoluteDH(a = L2, offset = -np.pi / 2)
            rob = DHRobot([e1,e2,e3], name = 'RRR')
            rob.teach([r11, r21, r31])
            fkinerob = rob.fkine(q = [r11, r21, r31])
            sol = rob.ikine_LM(fkinerob)
            print('Pose =\n', fkinerob)
            print('Solução 1:\n', sol)
        else:
            print("Valor fora dos limites propostos.")

        if abs(r11 - np.pi/2) > np.pi/2 or abs(r22) > np.pi/2 or abs(r32 - np.pi/2) > np.pi/2:
            print("Solução 2:")
            print('θ1 =', r11, 'θ2 =', r22, 'θ3 =', r32)
            e1 = RevoluteDH(d = 0, alpha = np.pi / 2, offset = np.pi / 2)
            e2 = RevoluteDH(a = L1)
            e3 = RevoluteDH(a = L2, offset = -np.pi / 2)
            rob = DHRobot([e1,e2,e3], name = 'RRR')
            rob.teach([r11, r22, r32]) 
            fkinerob = rob.fkine(q = [r11, r22, r32])
            sol = rob.ikine_LM(fkinerob)
            print('Pose =\n', fkinerob)
            print('Solução 2:\n', sol)
    
    ############################Letra C############################
    funcao(0, 0.1, 0.1, 0.15, 0.15)
        

############Questão 3############
def inkine_scara(x, y, z, L1, L2, D2, D4):
    radius = m.sqrt(x**2 + y**2)
    cos2 = (x**2 + y**2 - L1**2 - L2**2) / (2 * L1 * L2)
    cos1 = (x**2 + y**2 + L1**2 - L2**2) / (2 * L1 * radius) if radius != 0 else 0

    if abs(cos2) > 1 or abs(cos1) > 1:
        print("Valor fora dos limites. Valor absoluto do cosseno entre os elos acima de 1")
        return

    sen2 = m.sqrt(1 - cos2**2)
    r21 = m.atan2(sen2, cos2)
    r22 = m.atan2(-sen2, cos2)
    sen1 = m.sqrt(1 - cos1**2)
    r11 = m.atan2(y, x) - m.atan2(sen1, cos1)
    d3 = D2 - D4 - z

    print("Solução:")
    print('θ1 =', r11, 'θ2 =', r21, 'D3 =', d3)
    revL1 = RevoluteDH(a = L1, d = 0.2)
    revL2 = RevoluteDH(a = L2, alpha = np.pi)
    priDH = PrismaticDH(qlim = [0, 1])
    revD4 =  RevoluteDH(d = D4)
    rob = DHRobot([revL1, revL2, priDH, revD4], name = 'RRPR')
    rob.teach([r11, r21, d3, 0])

questao1()
questao2()
x = 0.8
y = 0.5
z = 0.7
L1 = 1
L2 = 1
D2 = 0.2
D4 = 0.2
inkine_scara(x, y, z, L1, L2, D2, D4)