# icuas_waypoint_py

a simple python demo to publish waypoints that can be subscribed by icuas_gazebo.

## usage:
rosrun waypoint_pub waypoint_pub.py 


multi_waypoint_pub.py 可以让icuas_UAV按顺序访问队列里面的点
@todo1: 根据 plant_bed topic 生成 way_point
@todo2: tsp程序排列 way_point ->用时、路径最短，且不碰撞
