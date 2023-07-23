""" Scenario Description
Cut-in Scenario
"""
param map = localPath('/Users/zhanpengfei/Scenic/tests/formats/opendrive/maps/CARLA/Town03.xodr')  # or other CARLA map that definitely works
param carla_map = 'Town03'
model scenic.domains.driving.model

#CONSTANTS
EGO_SPEED = 70
BRAKE_ACTION = 1.0
EGO_BRAKING_THRESHOLD = 3
RISISTENT_SPEED = -15
OBJECT_SPEED = EGO_SPEED + RISISTENT_SPEED
DISTANCE1 = 10
DISTANCE2 = 20
BRAKE_DISTANCE = 10
#Cutincar BEHAVIOUR:
behavior cutinBehavior(DISTANCE,OBJECT_SPEED,BRAKE_DISTANCE):
    try:
        do FollowLaneBehavior(laneToFollow=ego.lane,target_speed=OBJECT_SPEED)
    interrupt when self.distanceToClosest(object) < BRAKE_DISTANCE:
        while True:
            frontCar = closest (Car in front of self)
            rearCar = closest (Car behind self)
            if distanceTo(frontCar) < 10:
                take setThrottleAction(1)
            elif distanceTo(rearCar) < 10:
                take SetBrakeAction(1)
        

behavior Car2Behavior():
    try:
        do FollowLaneBehavior(target_speed=100)
    interrupt when 15 > simulation().currentTime > 10 :
        take SetBrakeAction(1)

#EGO BEHAVIOR: Follow lane and brake when reaches threshold distance to obstacle
behavior EgoBehavior(speed=10,EGO_BRAKE_PRESSURE=0.5):
    try:
        do FollowLaneBehavior(speed)
    interrupt when 7.5 < self.distanceToClosest(Object) < 10:
        take SetBrakeAction(EGO_BRAKE_PRESSURE)

#GEOMETRY
initLane = Uniform(*network.lanes)
initLaneSec = Uniform(*initLane.sections)

#PLACEMENT
spawnPt = OrientedPoint on initLaneSec.centerline

ego = Car with behavior EgoBehavior(EGO_SPEED)

Car1 = Car offset by (-3 or 3) @ DISTANCE1,
    facing Range(-5, 5) deg relative to ego.heading,
	with behavior cutinBehavior(OBJECT_SPEED=OBJECT_SPEED,BRAKE_DISTANCE=BRAKE_DISTANCE,DISTANCE=DISTANCE1)

Car2 = Car offset by 0 @ DISTANCE2,
    facing Range(-5, 5) deg relative to ego.heading,
    with behavior Car2Behavior()

require (distance to intersection) > 80