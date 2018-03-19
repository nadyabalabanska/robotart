from __future__ import print_function
import sys
import math
import time
import copy
import rospy
import moveit_commander
import moveit_msgs.msg
import geometry_msgs.msg
import std_msgs.msg
import visualization_msgs.msg
from tf.transformations import quaternion_from_euler, euler_from_quaternion


PRECISE_VELOCITY_SCALING_FACTOR = 0.1
PRECISE_VELOCITY_JOINT_TOLERANCE_DEG = 0.2
PRECISE_ACCELERATION_SCALING_FACTOR = 0.5
FAST_VELOCITY_SCALING_FACTOR = 0.3
FAST_VELOCITY_JOINT_TOLERANCE_DEG = 2
FAST_ACCELERATION_SCALING_FACTOR = 0.5
DOWNWARD_OFFSET = 0.02

JUMP_THRESHOLD = 20


class UnableToComputePathError(Exception):
    pass


class JointPositions:
    HOME = [0, 0, 0, 0, 0, 0]
    CANVAS_HOME = [-0.0975, 0.0704, 1.9869, -math.pi, -1.0791, 3.0448]


class TargetLocations:
    travel_height = 0.27 # Z-coordinate that waypoints in
                         # any path moving to a target
                         # location should have
    canvas_home = (0.35, -0.21, 0.25)
    brush_home = (0.15, -0.4, 0.25)

    
class EefOrientations:
    @staticmethod
    def painting_orientation():
        '''
        Brush pointing downward vertically
        '''
        q = geometry_msgs.msg.Quaternion()
        q.x = 0
        q.y = 0
        q.z = 0
        q.w = -1
        return q


class OrientationConstraints:
    @staticmethod
    def painting(end_effector_link):
        cs = moveit_msgs.msg.Constraints()
        cs.name = "paint"
        c = moveit_msgs.msg.OrientationConstraint()
        c.header = std_msgs.msg.Header()
        c.header.stamp = rospy.Time.now()
        c.link_name = end_effector_link
        c.orientation = EefOrientations.painting_orientation()
        #c.absolute_x_axis_tolerance = 0.5 
        #c.absolute_y_axis_tolerance = 0.5 
        #c.absolute_z_axis_tolerance = 0.5
        c.weight = 1
        return cs

    
class ManipulatorController:
    def __init__(self, args):
        self._marker_index = 0
        self.target_locations = TargetLocations()
        moveit_commander.roscpp_initialize(args)
        rospy.init_node('robotart_manipulator_control')
        self._robot = moveit_commander.RobotCommander()
        self._scene = moveit_commander.PlanningSceneInterface()
        self._group = moveit_commander.MoveGroupCommander('robot')
        self.velocity_mode_precise()

        # for RViz visualization
        self._display_trajectory_publisher = rospy.Publisher(
            '/move_group/display_planned_path',
            moveit_msgs.msg.DisplayTrajectory,
            queue_size=20)
        
        self._marker_pub = rospy.Publisher(
            '/trajectory_markers',
            visualization_msgs.msg.MarkerArray,
            queue_size=None) # need this for markers to reliably show in RViz

    
    def __del__(self):
        moveit_commander.roscpp_shutdown()
        

    def location(self):
        current_pose = self._group.get_current_pose().pose
        return (current_pose.position.x,
                current_pose.position.y,
                current_pose.position.z)

        
    def velocity_mode_precise(self):
        self._velocity_scale = PRECISE_VELOCITY_SCALING_FACTOR
        self._acceleration_scale = PRECISE_ACCELERATION_SCALING_FACTOR
        rospy.set_param("/redhawk_controller/joint_tolerance", PRECISE_VELOCITY_JOINT_TOLERANCE_DEG)
        rospy.set_param("/move_group/cartesian_path_planning/max_velocity_scaling_factor", PRECISE_VELOCITY_SCALING_FACTOR)
        rospy.set_param("/move_group/cartesian_path_planning/max_acceleration_scaling_factor", PRECISE_ACCELERATION_SCALING_FACTOR)
        rospy.set_param("/move_group/cartesian_path_planning/enable_eef_constant_speed", True)

        
    def velocity_mode_fast(self):
        self._velocity_scale = FAST_VELOCITY_SCALING_FACTOR
        self._acceleration_scale = FAST_ACCELERATION_SCALING_FACTOR
        rospy.set_param("/redhawk_controller/joint_tolerance", FAST_VELOCITY_JOINT_TOLERANCE_DEG)
        rospy.set_param("/move_group/cartesian_path_planning/max_velocity_scaling_factor", FAST_VELOCITY_SCALING_FACTOR)
        rospy.set_param("/move_group/cartesian_path_planning/max_acceleration_scaling_factor", FAST_ACCELERATION_SCALING_FACTOR)
        rospy.set_param("/move_group/cartesian_path_planning/enable_eef_constant_speed", False)

        
    def move_to_pose(self, position, orientation, cartesian=False, cartesian_resolution=0.005):
        '''
        position: [x, y, z]
        orientation: [roll, pitch, yaw]
        '''
        target = geometry_msgs.msg.Pose()
        quat = quaternion_from_euler(orientation[0],
                                     orientation[1],
                                     orientation[2])
        target.orientation.x = quat[0]
        target.orientation.y = quat[1]
        target.orientation.z = quat[2]
        target.orientation.w = quat[3]
        target.position.x = position[0]
        target.position.y = position[1]
        target.position.z = position[2]
        self._group.set_pose_target(target)
        self._group.set_max_velocity_scaling_factor(self._velocity_scale)
        self._group.set_max_acceleration_scaling_factor(self._acceleration_scale)
        
        if not cartesian:
            plan = self._group.plan()
            return self._group.execute(plan)
        else:
            current_pose = self._group.get_current_pose().pose
            path = [(0, 0, 0),
                    (target.position.x-current_pose.position.x,
                     target.position.y-current_pose.position.y,
                     target.position.z-current_pose.position.z)]
            wp, plan, fraction = self.plan_cartesian(path, resolution=cartesian_resolution)
            if fraction > 0.99:
                return self.execute_plan(plan, wp)
            else:
                raise UnableToComputePathError()

        
    def move_relative_pose(self, x=None, y=None, z=None,
                           roll=None, pitch=None, yaw=None,
                           cartesian=False,
                           cartesian_resolution=0.005):
        current_pose = self._group.get_current_pose().pose
        
        position = [current_pose.position.x,
                    current_pose.position.y,
                    current_pose.position.z]
        if x is not None:
            position[0] += x
        if y is not None:
            position[1] += y
        if z is not None:
            position[2] += z

        current_quat = (current_pose.orientation.x,
                        current_pose.orientation.y,
                        current_pose.orientation.z,
                        current_pose.orientation.w)
        orient = euler_from_quaternion(current_quat)
    
        if roll is not None:
            orient[0] += roll
        if pitch is not None:
            orient[1] += pitch
        if yaw is not None:
            orient[2] += yaw

        return self.move_to_pose(position, orient, cartesian, cartesian_resolution)


    def move_to_joint_positions(self, positions):
        joint_names = ['joint1', 'joint2', 'joint3', 'joint4', 'joint5', 'joint6']
        self._group.set_joint_value_target({joint_name: position for joint_name, position in zip(joint_names, positions)})
        self._group.set_max_velocity_scaling_factor(self._velocity_scale)
        self._group.set_max_acceleration_scaling_factor(self._acceleration_scale)
        plan = self._group.plan()
        return self._group.execute(plan)
        

    def plan_cartesian(self, path, resolution=0.005, orientation=None):
        template_pose = copy.deepcopy(self._group.get_current_pose().pose)
        if orientation is not None:
            template_pose.orientation = orientation
        wp = [copy.deepcopy(template_pose) for i in range(len(path))]

        for i, pt in enumerate(path):
            wp[i].position.x += pt[0]
            wp[i].position.y += pt[1]
            wp[i].position.z += pt[2]

        plan, fraction = self._group.compute_cartesian_path(wp, resolution, JUMP_THRESHOLD)
        return wp, plan, fraction
        
        
    def plan_cartesian_xy(self, paths, resolution=0.005, downward_offset=DOWNWARD_OFFSET):
        template_pose = copy.deepcopy(self._group.get_current_pose().pose)
        template_pose.orientation = EefOrientations.painting_orientation()
        num_waypoints = sum([len(path)+2 for path in paths])
        wp = [copy.deepcopy(template_pose) for i in range(num_waypoints)]

        point_count = 0
        for path in paths:
            # Add waypoint at beginning of downward path (extend tool)
            wp[point_count].position.x += path[0][0]
            wp[point_count].position.y += path[0][1]
            point_count += 1

            # Add waypoints for individual path
            for pt in path:
                wp[point_count].position.x += pt[0]
                wp[point_count].position.y += pt[1]
                wp[point_count].position.z -= downward_offset
                point_count += 1

            # Add waypoint to retract tool
            wp[point_count].position.x += path[-1][0]
            wp[point_count].position.y += path[-1][1]
            point_count += 1
    
        # Requires a modified version of MoveIt that reads these params
        # (MoveIt from gitlab.com/mitroboteam/moveit)
        plan, fraction = self._group.compute_cartesian_path(wp, resolution, JUMP_THRESHOLD)
        return wp, plan, fraction


    def plan_cartesian_to_location(self, location, resolution=0.01, travel_height=None):
        if travel_height is None:
            travel_height = self.target_locations.travel_height
            
        current_pose = self._group.get_current_pose().pose
        wp = [(0, 0, 0),
              (0, 0, travel_height-current_pose.position.z),
              (location[0]-current_pose.position.x,
               location[1]-current_pose.position.y,
               travel_height-current_pose.position.z),
              (location[0]-current_pose.position.x,
               location[1]-current_pose.position.y,
               location[2]-current_pose.position.z)]
        return self.plan_cartesian(wp, resolution, orientation=EefOrientations.painting_orientation())


    def move_to_location_cartesian(self, location, resolution=0.01, travel_height=None):
        wp, plan, fraction = self.plan_cartesian_to_location(location, resolution, travel_height)
        if fraction > 0.99:
            return self.execute_plan(plan, wp)
        else:
            raise UnableToComputePathError()

                
    def execute_plan(self, plan, wp=None):
        remove_markers = None
        if wp is not None:
            add_markers, remove_markers = self.create_markers(wp)
            self._marker_pub.publish(add_markers)
            
        ret = self._group.execute(plan)

        if remove_markers is not None:
            self._marker_pub.publish(remove_markers)

        return ret

        
    def create_markers(self, wp, i=None):
        if i is None:
            i = self._marker_index
            self._marker_index += 1
        
        markers_add = visualization_msgs.msg.MarkerArray()
        markers_remove = visualization_msgs.msg.MarkerArray()
    
        marker = visualization_msgs.msg.Marker()
        marker.header.frame_id = "base_link"
        marker.ns = "trajectory_waypoints_{}".format(i)
        marker.id = 0;
        marker.type = 4;
        marker.action = 0;
        for i, p in enumerate(wp):
            marker.points += [p.position]
        marker.scale.x = 0.01
        marker.scale.y = 0.01
        marker.scale.z = 0.01
        marker.color.a = 1.0
        marker.color.r = 0.0
        marker.color.g = 1.0
        marker.color.b = 0.0
        markers_add.markers += [marker]
        marker_remove = copy.deepcopy(marker)
        marker_remove.action = 2
        markers_remove.markers += [marker_remove]

        return markers_add, markers_remove
