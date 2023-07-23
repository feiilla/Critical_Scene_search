param map = localPath('/Users/zhanpengfei/Scenic/tests/formats/opendrive/maps/CARLA/Town03.xodr')  # or other CARLA map that definitely works
param carla_map = 'Town03'
model scenic.domains.driving.model
ego = Car
Car1 = Car offset by 0 @ 5
