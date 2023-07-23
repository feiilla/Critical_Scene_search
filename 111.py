from allpairspy import AllPairs
import os
import scenic
from scenic.simulators.newtonian import NewtonianSimulator
import math
w = 5
h = 2

def gjk(juxing0, juxing1):
    def dot(a, b):
            return a[0] * b[0] + a[1] * b[1]
    
    def support(a, b, d):
        def dot(a, b):
            return a[0] * b[0] + a[1] * b[1]

        def subtract(a, b):
            return (a[0] - b[0], a[1] - b[1])

        def farthest_point(points, d):
            max_p = float('-inf')
            max_point = None
            for point in points:
                p = dot(point, d)
                if p > max_p:
                    max_p = p
                    max_point = point
            return max_point

        return subtract(farthest_point(a, d), farthest_point(b, (-d[0], -d[1])))

    def triple_product(a, b, c):
        ac = a[0] * c[0] + a[1] * c[1]
        bc = b[0] * c[0] + b[1] * c[1]
        return (b[0] * ac - a[0] * bc, b[1] * ac - a[1] * bc)

    def contains_origin(simplex):
        a = simplex[-1]
        ao = (-a[0], -a[1])
        if len(simplex) == 3:
            b = simplex[-2]
            c = simplex[-3]
            ab = (b[0] - a[0], b[1] - a[1])
            ac = (c[0] - a[0], c[1] - a[1])
            ab_perp = triple_product(ac, ab, ab)
            ac_perp = triple_product(ab, ac, ac)
            if dot(ab_perp, ao) > 0:
                simplex.remove(c)
                return ab_perp
            elif dot(ac_perp, ao) > 0:
                simplex.remove(b)
                return ac_perp
            else:
                return None
        else:
            b = simplex[-2]
            ab = (b[0] - a[0], b[1] - a[1])
            ab_perp = triple_product(ab, ao, ab)
            return ab_perp

    simplex = [support(juxing0, juxing1, (1, 0))]
    direction = (-simplex[-1][0], -simplex[-1][1])
    while True:
        simplex.append(support(juxing0, juxing1, direction))
        if simplex[-1][0] * direction[0] + simplex[-1][1] * direction[
                1] <= 0:
            return False
        direction = contains_origin(simplex)
        if direction is None:
            return True

def fun_state(data):
    data_state0=[]
    data_state1=[]
    data_state2=[]
    for i in range(len(data)-1):
        if(data[i+1][0].x-data[i][0].x != 0):
            data_state0.append((data[i+1][0].y-data[i][0].y)/(data[i+1][0].x-data[i][0].x))
        else:
            data_state0.append('a')

        if(data[i+1][1].x-data[i][1].x != 0):
            data_state1.append((data[i+1][1].y-data[i][1].y)/(data[i+1][1].x-data[i][1].x))
        else:
            data_state1.append('a')

        if(data[i+1][0].x-data[i][0].x != 0):
            data_state2.append((data[i+1][0].y-data[i][2].y)/(data[i+1][2].x-data[i][2].x))
        else:
            data_state2.append('a')

    return data_state0,data_state1,data_state2

def angel(tana):
    if(tana != 'a'):
        sina = pow((1/(1+pow(tana,2))),1/2)           # sina^2+cosa^2=1 (1) 
        cosa = pow((pow(tana,2)/(1+pow(tana,2))),1/2) # tana=sina/cosa  (2)
    else:
        sina = 1 # tana为无穷大时
        cosa = 0
    return sina,cosa

def getRectVertex(center,sina,cosa):
    juxing=[]
    juxing.append((center[0] + w / 2 * cosa - h / 2 * sina,center[1] - w / 2 * sina - h / 2 * cosa))

    juxing.append((center[0] - w / 2 * cosa - h / 2 * sina,center[1] + w / 2 * sina - h / 2 * cosa))

    juxing.append((center[0] - w / 2 * cosa + h / 2 * sina,center[1] + w / 2 * sina + h / 2 * cosa))

    juxing.append((center[0] + w / 2 * cosa + h / 2 * sina,center[1] - w / 2 * sina + h / 2 * cosa))
    
    return juxing;
if __name__ == '__main__':
    parameters = [
        ["6","7","8","9","10"],
        ["0.25","0.5","0.75","1"],
        ["0.25","0.5","0.75","1"]
    ]
    # pairwise=[]
    pairwise = AllPairs(parameters)
    # for p1 in range(4):
    #     for p2 in range(4):
    #         for p3 in range(4):
    #             pairwise.append((parameters[0][p1],parameters[1][p2],parameters[2][p3]))


    for i, v in enumerate(pairwise):
        os.system("cp -R ~/Scenic/examples/driving/Carla_Challenge/carlaChakkenge5.scenic ~/Desktop/")
        os.system("mv ~/Desktop/carlaChakkenge5.scenic ~/Desktop/carlaChakkenge5testcase"+ str(i) + ".scenic")
    i=0
    v=0
    pairwise = AllPairs(parameters)
    # 测试用例修改
    for i, v in enumerate(pairwise):
        # if float(v[1]) > float(v[2]):
        #     continue
        print("%i:\t%s" %(i, str(v)))
        f1 = open("/Users/zhanpengfei/Desktop/carlaChakkenge5testcase" + str(i) + ".scenic","r")
        
        content = f1.read()
        f1.close()
        content = content.replace("BRAKE_DISTANCE = 6","BRAKE_DISTANCE = "+str(v[0]))
        content = content.replace("EGO_BRAKE_PRESSURE = 0.5","EGO_BRAKE_PRESSURE = "+str(v[1]))
        content = content.replace("BRAKE_PRESSURE1 = 1","BRAKE_PRESSURE1 = "+str(v[2]))
        with open("/Users/zhanpengfei/Desktop/carlaChakkenge5testcase" + str(i) + ".scenic","w") as f2:
            f2.write(content)
    # 测试用例筛选
    pairwise = AllPairs(parameters)
    i = 0
    v = 0
    for i,v in enumerate(pairwise):
        # if float(v[1]) > float(v[2]):
        #     continue
        collision = 0
        scenario = scenic.scenarioFromFile('/Users/zhanpengfei/Desktop/carlaChakkenge5testcase'+ str(i) +'.scenic',
                                               model='scenic.simulators.newtonian.driving_model')
        for k in range(20):
            scene, _ = scenario.generate()
            simulator = NewtonianSimulator()
            simulation = simulator.simulate(scene, maxSteps=60)
            if simulation:
                result = simulation.result     
                station = list(result.trajectory)
                angel0=[]
                angel1=[]
                angel2=[]
                test=0
                diff1,diff2,diff3 = fun_state(station)
                # front_collision = 0
                # behind_collision = 0
                # both_collision = 0
                for m in range(59):
                    angel0.append(list(angel(diff1[m])))
                    angel1.append(list(angel(diff2[m])))
                    angel2.append(list(angel(diff3[m])))
                    juxing0 = getRectVertex(station[m][0],angel0[m][0],angel0[m][1])
                    juxing1 = getRectVertex(station[m][1],angel1[m][0],angel1[m][1])
                    juxing2 = getRectVertex(station[m][2],angel2[m][0],angel2[m][1])
                    if gjk(juxing0,juxing1):
                        collision = collision + 1
                        test = 1
                    if gjk(juxing0,juxing2):
                        collision = collision + 1
                        test = 1
                    if test != 0:
                        break;
        print(f'carlaChakkenge5testcase{i}场景在20次测试中碰撞了{collision}次')



