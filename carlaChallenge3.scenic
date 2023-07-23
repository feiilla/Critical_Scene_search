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
DISTANCE1 = 5

#Cutincar BEHAVIOUR:
behavior cutinBehavior(DISTANCE,OBJECT_SPEED):
    while (distance from self to ego) < DISTANCE:
        do FollowLaneBehavior(OBJECT_SPEED);
    do FollowLaneBehavior(laneToFollow=ego.lane,target_speed=OBJECT_SPEED)

#EGO BEHAVIOR: Follow lane and brake when reaches threshold distance to obstacle
behavior EgoBehavior(speed=10):
    try:
        do FollowLaneBehavior(speed)
    interrupt when self.distanceToClosest(Object) < 5:
        take SetBrakeAction(1)

#GEOMETRY
initLane = Uniform(*network.lanes)
initLaneSec = Uniform(*initLane.sections)

#PLACEMENT
spawnPt = OrientedPoint on initLaneSec.centerline

ego = Car with behavior EgoBehavior(EGO_SPEED)

Car1 = Car offset by Options({-3,3}) @ DISTANCE1,
    facing Range(-5, 5) deg relative to ego.heading,
	with behavior cutinBehavior(OBJECT_SPEED=OBJECT_SPEED,DISTANCE=DISTANCE1)

require (distance to intersection) > 80,
		