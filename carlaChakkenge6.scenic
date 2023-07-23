param map = localPath('/Users/zhanpengfei/Scenic/tests/formats/opendrive/maps/CARLA/Town03.xodr')  # or other CARLA map that definitely works
param carla_map = 'Town03'
model scenic.domains.driving.model

#CONSTANTS
EGO_SPEED = 70
BRAKE_ACTION = 1.0
EGO_BRAKING_THRESHOLD = 3
RISISTENT_SPEED1 = -15
RISISTENT_SPEED2 = -25
OBJECT_SPEED1 = EGO_SPEED + RISISTENT_SPEED1
OBJECT_SPEED2 = EGO_SPEED + RISISTENT_SPEED2
DISTANCE1 = -20
DISTANCE2 = 10
BRAKE_DISTANCE = 6
BRAKE_PRESSURE1 = 1
BRAKE_PRESSURE2 = 1
EGO_BRAKE_PRESSURE = 0.5
#Cutincar BEHAVIOUR:
behavior cutinBehavior(DISTANCE,OBJECT_SPEED,BRAKE_DISTANCE,BRAKE_PRESSURE1):
    try:
        do FollowLaneBehavior(laneToFollow=ego.lane,target_speed=OBJECT_SPEED)
    interrupt when (distance from self to ego) < DISTANCE:
        do FollowLaneBehavior(OBJECT_SPEED);
    interrupt when (distance from self to ego) < BRAKE_DISTANCE:
        take SetBrakeAction(BRAKE_PRESSURE1)


#brakebehavior:
behavior cutoutBehavior(OBJECT_SPEED,BRAKE_PRESSURE2):
    try:
        do FollowLaneBehavior(OBJECT_SPEED)
    interrupt when 15 > simulation().currentTime > 10 :
        take SetBrakeAction(BRAKE_PRESSURE2)

#EGO BEHAVIOR: Follow lane and brake when reaches threshold distance to obstacle
behavior EgoBehavior(speed=10,EGO_BRAKE_PRESSURE=0.5):
    try:
        do FollowLaneBehavior(speed)
    interrupt when 7.5 < self.distanceToClosest(Object) < 10:
        take SetBrakeAction(EGO_BRAKE_PRESSURE)
    interrupt when self.distanceToClosest(Object) < 7.5:
        do FollowLaneBehavior(speed)

#GEOMETRY
initLane = Uniform(*network.lanes)
initLaneSec = Uniform(*initLane.sections)

#PLACEMENT
spawnPt = OrientedPoint on initLaneSec.centerline

ego = Car with behavior EgoBehavior(EGO_SPEED,EGO_BRAKE_PRESSURE)

Car1 = Car offset by Options({-3,3}) @ DISTANCE1,
    facing Range(-5, 5) deg relative to ego.heading,
	with behavior cutinBehavior(OBJECT_SPEED=OBJECT_SPEED2,DISTANCE=DISTANCE1,BRAKE_DISTANCE=BRAKE_DISTANCE,BRAKE_PRESSURE1=BRAKE_PRESSURE1)

Car2 = Car offset by 0 @ DISTANCE2 ,
	facing Range(-5, 5) deg relative to ego.heading,
	with behavior cutoutBehavior(OBJECT_SPEED=OBJECT_SPEED1,BRAKE_PRESSURE2=BRAKE_PRESSURE2)

require (distance to intersection) > 80