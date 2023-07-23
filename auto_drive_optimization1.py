from random import random
import os
import scenic
from scenic.simulators.newtonian import NewtonianSimulator
import math
import matplotlib.pyplot as plt
w = 5
h = 2
times = 1;
def func(a,b,c):
    global times
    
    print(f'第{times}次模拟。')
    print(f"刹车距离={a}, 前车刹车力度={b}， 后车刹车力度={c}")
    times = times + 1
    os.system("cp -R ~/Scenic/examples/driving/Carla_Challenge/carlaChakkenge5.scenic ~/Desktop/")
    os.system("mv ~/Desktop/carlaChakkenge5.scenic ~/Desktop/carlaChakkenge5testcaseSA.scenic")
    f1 = open("/Users/zhanpengfei/Desktop/carlaChakkenge5testcaseSA.scenic","r")
    content = f1.read()
    f1.close()
    content = content.replace("BRAKE_DISTANCE = 6","BRAKE_DISTANCE = "+str(a))
    content = content.replace("EGO_BRAKE_PRESSURE = 0.5","EGO_BRAKE_PRESSURE = "+str(b))
    content = content.replace("BRAKE_PRESSURE1 = 1","BRAKE_PRESSURE1 = "+str(c))
    with open("/Users/zhanpengfei/Desktop/carlaChakkenge5testcaseSA.scenic","w") as f2:
        f2.write(content)
    collision = 0
    for k in range(20):
        scenario = scenic.scenarioFromFile('/Users/zhanpengfei/Desktop/carlaChakkenge5testcaseSA.scenic',
                                           model='scenic.simulators.newtonian.driving_model')
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
    
    return collision/20
class SA:
    def __init__(self,func,iter=5, T0=100, Tf=0.01, alpha=0.80):
        self.func = func
        self.iter = iter         #内循环迭代次数,即为L = 5
        self.alpha = alpha       #降温系数，alpha=0.80
        self.T0 = T0             #初始温度T0为100
        self.Tf = Tf             #温度终值Tf为0.01
        self.T = T0              #当前温度
        self.a = int((random()*5+5)*100)/100  
        self.b = int((random()*0.8+0.2)*100)/100 
        self.c = int((random()*0.8+0.2)*100)/100 
        self.most_best=[]
        self.history = {'f': [], 'T': []}
    def generate_new(self, a, b, c):   #扰动产生新解的过程
        while True:
            a_new = int((a + self.T * (random() - random()))*100)/100
            b_new = int((b + self.T * (random() - random()))*100)/100
            c_new = int((c + self.T * (random() - random()))*100)/100
            if (5 <= a_new <= 10) & (0.2 <= b_new <= 1) & (0.2 <= c_new <= 1):  
                break                                  #重复得到新解，直到产生的新解满足约束条件
        return a_new, b_new, c_new
    def Metrospolis(self, f, f_new):   #Metropolis准则
        if f_new <= f:
            return 1
        else:
            p = math.exp((f - f_new) / self.T)
            if random() < p:
                return 1
            else:
                return 0
    def best(self,f_list : list):    #获取最优目标函数值
        f_best = min(f_list)
        idx = f_list.index(f_best)
        return f_best, idx
    def run(self):
        count = 0
        flag = 0
        #外循环迭代，当前温度小于终止温度的阈值
        f = abs(self.func(self.a, self.b, self.c)-0.5)                    #f为初始值
        while self.T > self.Tf:       
            #内循环迭代2次
            for i in range(self.iter): 
                a_new, b_new, c_new = self.generate_new(self.a, self.b, self.c) #产生新解
                f_new = abs(self.func(a_new, b_new, c_new)-0.5)                        #产生新值
                if self.Metrospolis(f, f_new):                         #判断是否接受新值
                    self.a = a_new             #如果接受新值，则把新值的x,y存入x数组和y数组
                    self.b = b_new
                    self.c = c_new
                    f = f_new
            self.most_best.append(f)
            # 迭代L次记录在该温度下最优解
            ft, _ = self.best(self.most_best)
            self.history['f'].append(f)
            self.history['T'].append(self.T)
            #温度按照一定的比例下降（冷却
            if ft <= 0.05:
               print(f"F={ft}, 刹车距离={self.a[_]}, 前车刹车力度={self.b[_]}， 后车刹车力度={self.c[_]}") 
               flag = 1
               break
            else:
                self.T = self.T * self.alpha
                count += 1   
            # 得到最优解
        if flag == 0:
            f_best, idx = self.best(self.most_best)
            print(f"F={f_best}, 刹车距离={self.a[idx]}, 前车刹车力度={self.b[idx]}， 后车刹车力度={self.c[idx]}")

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

sa = SA(func)
sa.run()
plt.plot(sa.history['T'], sa.history['f'])
plt.title('SA')
plt.xlabel('T')
plt.ylabel('f')
plt.gca().invert_xaxis()
plt.show()
# if __name__ == '__main__':
#     proportion = 0
#     while not (0.50 <= proportion<= 0.55):
#         parameters = [
#             ["9"],
#             ["0.5"],
#             ["0.25"]
#         ]
#         # pairwise=[]
#         # pairwise = AllPairs(parameters)
#         # for p1 in range(4):
#         #     for p2 in range(4):
#         #         for p3 in range(4):
#         #             pairwise.append((parameters[0][p1],parameters[1][p2],parameters[2][p3]))


#         for i, v in enumerate(pairwise):
#             os.system("cp -R ~/Scenic/examples/driving/Carla_Challenge/carlaChakkenge5.scenic ~/Desktop/")
#             os.system("mv ~/Desktop/carlaChakkenge5.scenic ~/Desktop/carlaChakkenge5testcase"+ str(i) + ".scenic")
#         i=0
#         v=0
#         pairwise = AllPairs(parameters)
#         # 测试用例修改
#         for i, v in enumerate(pairwise):
#             # if float(v[1]) > float(v[2]):
#             #     continue
#             print("%i:\t%s" %(i, str(v)))
#             f1 = open("/Users/zhanpengfei/Desktop/carlaChakkenge5testcase" + str(i) + ".scenic","r")
            
#             content = f1.read()
#             f1.close()
#             content = content.replace("BRAKE_DISTANCE = 6","BRAKE_DISTANCE = "+str(v[0]))
#             content = content.replace("EGO_BRAKE_PRESSURE = 0.5","EGO_BRAKE_PRESSURE = "+str(v[1]))
#             content = content.replace("BRAKE_PRESSURE1 = 1","BRAKE_PRESSURE1 = "+str(v[2]))
#             with open("/Users/zhanpengfei/Desktop/carlaChakkenge5testcase" + str(i) + ".scenic","w") as f2:
#                 f2.write(content)
#         # 测试用例筛选
#         pairwise = AllPairs(parameters)
#         i = 0
#         v = 0
#         for i,v in enumerate(pairwise):
#             # if float(v[1]) > float(v[2]):
#             #     continue
#             collision = 0
#             for k in range(20):
#                 scenario = scenic.scenarioFromFile('/Users/zhanpengfei/Desktop/carlaChakkenge5testcase' + str(i) + '.scenic',
#                                             model='scenic.simulators.newtonian.driving_model')
#                 scene, _ = scenario.generate()
#                 simulator = NewtonianSimulator()
#                 simulation = simulator.simulate(scene, maxSteps=60)
#                 if simulation:
#                     result = simulation.result     
#                     station = list(result.trajectory)
#                     angel0=[]
#                     angel1=[]
#                     angel2=[]
#                     test=0
#                     diff1,diff2,diff3 = fun_state(station)
#                     # front_collision = 0
#                     # behind_collision = 0
#                     # both_collision = 0
#                     for m in range(59):
#                         angel0.append(list(angel(diff1[m])))
#                         angel1.append(list(angel(diff2[m])))
#                         angel2.append(list(angel(diff3[m])))
#                         juxing0 = getRectVertex(station[m][0],angel0[m][0],angel0[m][1])
#                         juxing1 = getRectVertex(station[m][1],angel1[m][0],angel1[m][1])
#                         juxing2 = getRectVertex(station[m][2],angel2[m][0],angel2[m][1])
#                         if gjk(juxing0,juxing1):
#                             collision = collision + 1
#                             test = 1
#                         if gjk(juxing0,juxing2):
#                             collision = collision + 1
#                             test = 1
#                         if test != 0:
#                             break;
#             print(f'carlaChakkenge5testcase{i}场景在20次测试中碰撞了{collision}次')
