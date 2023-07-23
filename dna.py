from random import random
import os
import scenic
from scenic.simulators.newtonian import NewtonianSimulator
import copy
import xlwt
w = 5
h = 2
p=[]
times = 1
x=[]
y=[]
flags = 0
CESHISHU = 20
CROSSOVER = 0.4
#适应度函数
def F(a:list):
    global times
    global flags
    print(f'第{times}次模拟。')
    print(f"刹车距离={a[0]}, 前车刹车力度={a[1]}， 后车刹车力度={a[2]}")
    times = times + 1
    os.system("cp -R ~/Scenic/examples/driving/Carla_Challenge/carlaChakkenge5.scenic ~/Desktop/")
    os.system("mv ~/Desktop/carlaChakkenge5.scenic ~/Desktop/carlaChakkenge5testcaseSA.scenic")
    f1 = open("/Users/zhanpengfei/Desktop/carlaChakkenge5testcaseSA.scenic","r")
    content = f1.read()
    f1.close()
    content = content.replace("BRAKE_DISTANCE = 6","BRAKE_DISTANCE = "+str(a[0]))
    content = content.replace("EGO_BRAKE_PRESSURE = 0.5","EGO_BRAKE_PRESSURE = "+str(a[1]))
    content = content.replace("BRAKE_PRESSURE1 = 1","BRAKE_PRESSURE1 = "+str(a[1]))
    with open("/Users/zhanpengfei/Desktop/carlaChakkenge5testcaseSA.scenic","w") as f2:
        f2.write(content)
    collision = 0
    # 编译
    scenario = scenic.scenarioFromFile('/Users/zhanpengfei/Desktop/carlaChakkenge5testcaseSA.scenic',
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
    print(f"碰撞次数={collision}")
    p.append([times-1,a[0],a[1],a[2],collision])
    if(collision==CESHISHU/2):
        print(f"找到啦！刹车距离={a[0]}, 前车刹车力度={a[1]}， 后车刹车力度={a[2]},碰撞次数为{collision}") 
        flags = 1
    return abs(collision/20-0.5)

def generateDna(x):
    for i in range(20):
        a = [random()*4+6,random(),random()]
        x.append(a)
        y.append(F(a))

def best(f_list : list):    #获取最优目标函数位置
    f_best = min(f_list)
    idx = f_list.index(f_best)
    return idx
def best2(f_list:list):
    f_new=f_list
    f_best = min(f_new)
    idx1 = f_new.index(f_best)
    f_new[idx1] = 0.5
    f_best = min(f_new)
    idx2 = f_new.index(f_best)
    return idx1,idx2

def worst(f_list : list):    #获取最差目标函数值
    f_best = max(f_list)
    idx = f_list.index(f_best)
    return idx

def new_lives(x:list,y:list):
    global flags
    x_new=copy.deepcopy(x)
    y_new=copy.deepcopy(y)
    for times in range(20):
        if random()<CROSSOVER:
            idxb = best(y)
            idxw = worst(y)
            x_new[idxw] = x_new[idxb]
            y_new[idxw] = y_new[idxb]
        else:
            while 1:
                idx1 = int(random()*20)
                idx2 = int(random()*20)
                if(idx1 != idx2):
                    break;
            while 1:
                flag = 0
                m = copy.deepcopy(x_new[idx1])
                n = copy.deepcopy(x_new[idx2])
                if random()<1/2:
                    m[0],n[0] = n[0],m[0]
                    flag += 1
                if random()<1/2:
                    m[1],n[1] = n[1],m[1]
                    flag += 1
                if random()<1/2:
                    m[2],n[2] = n[2],m[2]
                    flag += 1
                if 0 < flag < 3:
                    x_new[idx1] = m
                    x_new[idx2] = n
                    y_new[idx1] = F(x_new[idx1])
                    if flags == 1:
                        return x_new,y_new
                    y_new[idx2] = F(x_new[idx2])
                    if flags == 1:
                        return x_new,y_new
                    break;
            mutation(x,x_new,y_new)
        if flags == 1:
            return x_new,y_new
    return x_new,y_new

def mutation(x:list,x_new:list,y_new:list,MUTATION = 0.01):
    if(random()<MUTATION):
        k = int(random()*20)
        l = int(random()*3)
        if l == 0:
            x[k][l] = random()*4+6
        else:
            x[k][l] = random()
        x_new[k][l] = x[k][l]
        y_new[k]=F(x[k]) 

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

generateDna(x)
if flags == 0:
    while 1:
        x_new , y_new = new_lives(x,y)
        if flags == 1:
            break
        x = x_new
        y = y_new
workbook = xlwt.Workbook(encoding= 'ascii')
worksheet = workbook.add_sheet("p_sheet")
worksheet.write(0,0, "第几次模拟")
worksheet.write(0,1, "刹车距离")
worksheet.write(0,2, "自车刹车力度")
worksheet.write(0,3, "后车刹车力度")
worksheet.write(0,4, "碰撞次数")
timesss = 1
for x,y in enumerate(p):
    worksheet.write(timesss,0, str(y[0]))
    worksheet.write(timesss,1, str(y[1]))
    worksheet.write(timesss,2, str(y[2]))
    worksheet.write(timesss,3, str(y[3]))
    worksheet.write(timesss,4, str(y[4]))
    timesss+=1
workbook.save("遗传算法测试用例.xls")
