""" Scenario Description
Cut-out Scenario
"""
param map = localPath('/Users/zhanpengfei/Scenic/tests/formats/opendrive/maps/CARLA/Town03.xodr')  # or other CARLA map that definitely works
param carla_map = 'Town03'
model scenic.domains.driving.model
from scenic.core.simulators import *
#CONSTANTS
EGO_SPEED = 70
BRAKE_ACTION = 1.0
RISISTENT_SPEED = 9
OBJECT_SPEED = EGO_SPEED + RISISTENT_SPEED
DISTANCE = 20
BRAKE_PRESSURE = 1
EGO_BRAKE_PRESSURE = 1

#setbrakecar BEHAVIOUR:
behavior cutoutBehavior(OBJECT_SPEED,BRAKE_PRESSURE):
    try:
        do FollowLaneBehavior(OBJECT_SPEED)
    interrupt when 20 > simulation().currentTime > 10 :
        take SetBrakeAction(BRAKE_PRESSURE)

#EGO BEHAVIOR: Follow lane and brake when reaches threshold distance to obstacle
behavior EgoBehavior(EGO_BRAKE_PRESSURE,speed=10):
    try:
        do FollowLaneBehavior(speed)
    interrupt when self.distanceToClosest(Object) < 8:
        take SetBrakeAction(EGO_BRAKE_PRESSURE)

#GEOMETRY
initLane = Uniform(*network.lanes)
initLaneSec = Uniform(*initLane.sections)

#PLACEMENT
spawnPt = OrientedPoint on initLaneSec.centerline

ego = Car with behavior EgoBehavior(speed=EGO_SPEED,EGO_BRAKE_PRESSURE=EGO_BRAKE_PRESSURE)

Car	offset by 0 @ DISTANCE ,
	facing Range(-5, 5) deg relative to ego.heading,
	with behavior cutoutBehavior(OBJECT_SPEED=OBJECT_SPEED,BRAKE_PRESSURE=BRAKE_PRESSURE)

require (distance to intersection) > 80
		