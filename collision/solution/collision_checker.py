import itertools
import random
from typing import List

from aido_schemas import Context, FriendlyPose
from dt_protocols import (
    Circle,
    CollisionCheckQuery,
    CollisionCheckResult,
    MapDefinition,
    PlacedPrimitive,
    Rectangle,
    Primitive
)
import numpy as np

__all__ = ["CollisionChecker"]


class CollisionChecker:
    params: MapDefinition

    def init(self, context: Context):
        context.info("init()")

    def on_received_set_params(self, context: Context, data: MapDefinition):
        context.info("initialized")
        self.params = data

    def on_received_query(self, context: Context, data: CollisionCheckQuery):
        collided = check_collision(
            environment=self.params.environment, robot_body=self.params.body, robot_pose=data.pose
        )
        result = CollisionCheckResult(collided)
        context.write("response", result)


def check_collision(environment: List[PlacedPrimitive], robot_body: List[PlacedPrimitive],
        robot_pose: FriendlyPose) -> bool:
    # This is just some code to get you started, but you don't have to follow it exactly

    #Put placed primites in world frame 
    # You can start by rototranslating the robot_body by the robot_pose
    rototranslated_robot: List[PlacedPrimitive] = []
    phi = robot_pose.theta_deg
    for robot_parts in robot_body: 
        robot_frame = np.array([robot_parts.pose.x,robot_parts.pose.y,1])
        roto_translate = np.array([[np.cos(phi),-np.sin(phi), robot_pose.x],
                                   [np.sin(phi), np.cos(phi), robot_pose.y],
                                   [0,0,1]])
        world_frame = np.dot(roto_translate,robot_frame)
        pose_new = FriendlyPose(world_frame[0],world_frame[1],robot_parts.pose.theta_deg + phi)
        rototranslated_robot.append(PlacedPrimitive(pose_new,robot_parts.primitive))
        
        
    # == WRITE ME ==

    # Then, call check_collision_list to see if the robot collides with the environment
    collided = check_collision_list(rototranslated_robot, environment)

    # return a random choice
    return collided


def check_collision_list(rototranslated_robot: List[PlacedPrimitive], environment: List[PlacedPrimitive]) -> bool:
    # This is just some code to get you started, but you don't have to follow it exactly
    for robot, envObject in itertools.product(rototranslated_robot, environment):
        if check_collision_shape(robot, envObject):
            return True

    return False

def distance_between(a,b):                            
    return np.linalg.norm(a-b)
                               
def radii_rectangle(a: Primitive):
    # inscribed and circumscribed circles
    return min(a.ymin+a.ymax,a.xmin + a.xmax)/2, distance_between(np.array([a.xmax,a.ymax]),np.array([a.xmin,a.ymin]))/2

def corners_of_rectangle(a: PlacedPrimitive):
    corners = []
    corners.append(np.array([a.pose.x + a.primitive.xmax,a.pose.y + a.primitive.ymax]))
    corners.append(np.array([a.pose.x + a.primitive.xmax,a.pose.y - a.primitive.ymin]))
    corners.append(np.array([a.pose.x - a.primitive.xmin,a.pose.y + a.primitive.ymax]))
    corners.append(np.array([a.pose.x - a.primitive.xmin,a.pose.y - a.primitive.ymin]))
    
    return corners
                               
def circle_collision(pose1,pose2,radius1,radius2):
    collision_check = False
    if(0 <= distance_between(np.array([pose1.x,pose1.y]),np.array([pose2.x,pose2.y])) <= (radius1 + radius2)):
        collision_check =  True
    return collision_check

def check_collision_shape(a: PlacedPrimitive, b: PlacedPrimitive) -> bool:
    # This is just some code to get you started, but you don't have to follow it exactly
    is_collided = False

    # This is just some code to get you started, but you don't have to follow it exactly
    if isinstance(a.primitive, Circle) and isinstance(b.primitive, Circle):
        is_collided = circle_collision(a.pose,b.pose,a.primitive.radius,b.primitive.radius)
                               
        # == WRITE ME ==
    elif isinstance(a.primitive, Rectangle) and isinstance(b.primitive, Circle):
        r_in,r_cir = radii_rectangle(a.primitive)
        if(circle_collision(a.pose,b.pose,b.primitive.radius,r_in) == True):
            is_collided = True
        elif(circle_collision(a.pose,b.pose,b.primitive.radius,r_cir) == True):
            for corner in corners_of_rectangle(a):
                if(distance_between(corner,np.array([b.pose.x,b.pose.y]))<=b.primitive.radius):
                    is_collided = True
                    break

        # == WRITE ME ==
    elif isinstance(a.primitive, Rectangle) and isinstance(b.primitive, Rectangle):
        r_a_in,r_a_cir = radii_rectangle(a.primitive)
        r_b_in,r_b_cir = radii_rectangle(b.primitive)
        
        if(circle_collision(a.pose,b.pose,r_a_in,r_b_in) == True):
            is_collided = True
        elif(circle_collision(a.pose,b.pose,r_a_cir,r_b_cir == True)):
             for corner in corners_of_rectangle(b):
                    if(a.primitive.xmin <= corner[0] - a.pose.x <= a.primitive.xmax and a.primitive.ymin <= corner[1] - a.pose.y <= a.primitive.ymax ):
                        is_collided = True
                        break  

    return is_collided
