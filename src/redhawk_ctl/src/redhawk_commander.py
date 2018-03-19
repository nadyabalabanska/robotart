import time
import math
import rospy
import threading
from redhawk_ctl.msg import redhawk_cmd, redhawk_telem


ROS_RATE = 120
#MAX_VEL = 30.0 # deg/s
MAX_VEL = [30.0, 30.0, 30.0, 50.0, 50.0, 50.0] # From 5D Robotics example code
#MAX_ACCEL = 45
MAX_ACCEL = 200.0 # deg/(s^2), also from 5D Robotics example code
POSE_TOLERANCE = 2 # In degrees (+/-); should never be used -- read from rosparam instead
POSE_TOLERANCE_2 = 0.2 # Also in degrees. Used when velocity is specified for check_angle
DURATION_TOLERANCE = 1.1 # Allow a maximum of 10% lateness
MAX_TELEM_AGE = 0.1 # Maximum age of the telemetry data in seconds
VEL_MULT = 2
SMALL_DURATION = 10e-3
VERY_SMALL_DURATION = 10e-5
SMALL_ANGLE = 10e-2
TIMEOUT_OFFSET = 0.5
MIN_COMMAND_DURATION = 0.100


class JointTrajectoryTimeoutException(Exception):
    pass


class RedhawkController:
    def __init__(self, rate=ROS_RATE):
        self.pub = rospy.Publisher('/redhawk_cmd', redhawk_cmd, queue_size=1000)
        self.sub = rospy.Subscriber('/redhawk_telem', redhawk_telem, self._telem_recv)
        self.telem = None
        self.telem_time = None
        self.telem_lock = threading.Lock()
        self.r = rospy.Rate(rate)
        self.stop() # Start applying velocity and acceleration limits
        self.telem_purge(0.25) # Ensure that the controller IP gets set
        self.stop() # Finish applying velocity and acceleration limits


    def _telem_recv(self, telem):
        with self.telem_lock:
            self.telem = telem
            self.telem_time = time.time()


    def _telem(self):
        with self.telem_lock:
            telem = [self.telem.joint_1,
                     self.telem.joint_2,
                     -self.telem.joint_3,
                     self._reduce_angle_deg(self.telem.joint_4),
                     -self.telem.joint_5,
                     self._reduce_angle_deg(self.telem.joint_6)]
            telem_time = self.telem_time
            if telem is None or telem_time is None:
                rospy.logfatal('no telemetry! aborting!')
                return None
            if (time.time()-telem_time) > MAX_TELEM_AGE:
                rospy.logfatal('telemetry outdated! aborting!')
                self.stop()
                return None
            # Telemetry valid
            return telem

        
    def _reduce_angle_deg(self, angle):
        rangle = angle
        while rangle >= 180:
            rangle -= 360
        while rangle < -180:
            rangle += 360
        return rangle


    def _reduce_target_deg(self, target, telem):
        rtarget = target
        while rtarget-telem >= 360:
            rtarget -= 360
        while rtarget-telem <= -360:
            rtarget += 360
        return rtarget
        
    
    def _check_angle(self, goal, current, velocity=None, tolerance=POSE_TOLERANCE):
        rgoal = self._reduce_angle_deg(goal)
        rcurrent = self._reduce_angle_deg(current)

        if velocity is None:
            # Upper and lower bounded check with threshold
            return (rgoal-tolerance) <= rcurrent <= (rgoal+tolerance)
        else:
            # Use velocity to upper or lower bound
            if abs(rgoal-rcurrent) < tolerance:
                return True
            if velocity < 0:
                return rcurrent <= rgoal
            else:
                return rcurrent >= rgoal


    def set_pose(self, angles, wait=True):
        cmd = redhawk_cmd()
        cmd.cmd_type = cmd.CMD_POSITION
        cmd.max_vel = max(MAX_VEL)
        cmd.max_accel = MAX_ACCEL
        cmd.arm_enable = True
        cmd.joint_1 = angles[0]
        cmd.joint_2 = angles[1]
        cmd.joint_3 = angles[2]
        cmd.joint_4 = angles[3]
        cmd.joint_5 = angles[4]
        cmd.joint_6 = angles[5]
        self.pub.publish(cmd)

        if wait:
            while not rospy.is_shutdown():
                self.r.sleep()
                self.pub.publish(cmd)
                if self.check_pose(angles):
                    break


    def set_velocities(self, vel):
        cmd = redhawk_cmd()
        cmd.cmd_type = cmd.CMD_VELOCITY
        cmd.max_vel = max(MAX_VEL)
        cmd.max_accel = MAX_ACCEL
        cmd.arm_enable = True
        cmd.joint_1 = vel[0]
        cmd.joint_2 = vel[1]
        cmd.joint_3 = vel[2]
        cmd.joint_4 = vel[3]
        cmd.joint_5 = vel[4]
        cmd.joint_6 = vel[5]
        self.pub.publish(cmd)


    def check_pose(self, angles):
        with self.telem_lock:
            if self.telem is None:
                rospy.logwarn('cannot check pose, telemetry not available!')
                return False

            return self._check_angle(angles[0], self.telem.joint_1) and \
                   self._check_angle(angles[1], self.telem.joint_2) and \
                   self._check_angle(angles[2], self.telem.joint_3) and \
                   self._check_angle(angles[3], self.telem.joint_4) and \
                   self._check_angle(angles[4], self.telem.joint_5) and \
                   self._check_angle(angles[5], self.telem.joint_6)


    def stop(self):
        cmd = redhawk_cmd()
        cmd.cmd_type = cmd.CMD_VELOCITY
        cmd.max_vel = max(MAX_VEL)
        cmd.max_accel = MAX_ACCEL
        cmd.arm_enable = True
        cmd.joint_1 = 0
        cmd.joint_2 = 0
        cmd.joint_3 = 0
        cmd.joint_4 = 0
        cmd.joint_5 = 0
        cmd.joint_6 = 0
        self.pub.publish(cmd)
        rospy.loginfo("STOP")


    def telem_purge(self, telem_purge_duration):
        telem_purge_start = time.time()
        cmd = redhawk_cmd()
        cmd.max_vel = max(MAX_VEL)
        cmd.max_accel = MAX_ACCEL
        cmd.arm_enable = True
        cmd.cmd_type = cmd.CMD_VELOCITY
        cmd.joint_1 = 0
        cmd.joint_2 = 0
        cmd.joint_3 = 0
        cmd.joint_4 = 0
        cmd.joint_5 = 0
        cmd.joint_6 = 0
        while not rospy.is_shutdown():
            self.pub.publish(cmd)
            self.r.sleep()
            if time.time() > telem_purge_start + telem_purge_duration:
                # Done.
                break

            
    def run(self, joint_trajectory):
        cmd = redhawk_cmd()

        # Force apply velocity and acceleration limits
        cmd.max_vel = max(MAX_VEL)
        cmd.max_accel = MAX_ACCEL
        cmd.arm_enable = False
        self.pub.publish(cmd)

        prev_time = 0

        for i in range(len(joint_trajectory.points)):
            # Wait until timestamp for next command
            end_time = joint_trajectory.points[i].time_from_start.to_sec()
            time.sleep(end_time - prev_time + 0.5)
            prev_time = end_time

            # Perpare and send command
            cmd.cmd_type = cmd.CMD_POSITION
            cmd.arm_enable = True
            cmd.joint_1 = joint_trajectory.points[i].positions[0] / math.pi * 180.0;
            cmd.joint_2 = joint_trajectory.points[i].positions[1] / math.pi * 180.0;
            cmd.joint_3 = joint_trajectory.points[i].positions[2] / math.pi * 180.0;
            cmd.joint_4 = joint_trajectory.points[i].positions[3] / math.pi * 180.0;
            cmd.joint_5 = joint_trajectory.points[i].positions[4] / math.pi * 180.0;
            cmd.joint_6 = joint_trajectory.points[i].positions[5] / math.pi * 180.0;

            rospy.loginfo("[{}s] publishing command".format(prev_time))
            self.pub.publish(cmd)


    def run2(self, joint_trajectory):
        joint_tolerance = rospy.get_param("/redhawk_controller/joint_tolerance", POSE_TOLERANCE_2)
        rospy.loginfo("Using a joint position goal tolerance of {} degrees".format(joint_tolerance))
        
        cmd = redhawk_cmd()

        # Force apply velocity and acceleration limits
        cmd.max_vel = max(MAX_VEL)
        cmd.max_accel = MAX_ACCEL
        cmd.arm_enable = True
        cmd.cmd_type = cmd.CMD_VELOCITY
        cmd.joint_1 = 0
        cmd.joint_2 = 0
        cmd.joint_3 = 0
        cmd.joint_4 = 0
        cmd.joint_5 = 0
        cmd.joint_6 = 0

        # TODO: is it enough to do this in the constructor?
        # Get telemetry going. This is really needed.
        self.telem_purge(0.1)

        start_timestamp = rospy.get_time()
        prev_lateness = 0
        overall_lateness = 0

        try:
            for i in range(len(joint_trajectory.points)):
                # Calculate velocities
                targets = [0, 0, 0, 0, 0, 0]
                velocities = [0, 0, 0, 0, 0, 0]
                target_timestamp = joint_trajectory.points[i].time_from_start.to_sec()
                point_start_timestamp = rospy.get_time() - start_timestamp
                #desired_command_duration = target_timestamp - prev_lateness - point_start_timestamp
                #desired_command_duration = target_timestamp - point_start_timestamp
                #desired_command_duration = target_timestamp - point_start_timestamp + overall_lateness
                if i > 0:
                    desired_command_duration = joint_trajectory.points[i].time_from_start.to_sec() \
                                               - joint_trajectory.points[i-1].time_from_start.to_sec()
                    if desired_command_duration < MIN_COMMAND_DURATION:
                        rospy.logwarn("Desired command duration ({}) is very low. Clipping it to {}".format(desired_command_duration, MIN_COMMAND_DURATION))
                        desired_command_duration = MIN_COMMAND_DURATION
                else:
                    desired_command_duration = 0
                    if joint_trajectory.points[i].time_from_start.to_sec() != 0:
                        rospy.logerror("First point in JointTrajectory should be the current starting position!")
                        raise RuntimeError("Invalid JointTrajectory")
                rospy.loginfo("desired command duration is {:.3f} s".format(desired_command_duration))

                telem = self._telem()
                if telem is None:
                    rospy.logfatal("telemetry invalid, aborting!")
                    self.stop()
                    return False
                
                # Joint 4 and 6 support continuous rotation
                for joint, is_continuous in enumerate([False, False, False, True, False, True]):
                    target_deg = joint_trajectory.points[i].positions[joint] / math.pi * 180.0
                    targets[joint] = self._reduce_target_deg(target_deg, telem[joint])
                    if is_continuous:
                        if abs(targets[joint]-telem[joint]) >= 180:
                            joint_offset_deg = -math.copysign((360 - abs(targets[joint] - telem[joint])),
                                                              targets[joint] - telem[joint])
                        else:
                            joint_offset_deg = (targets[joint] - telem[joint])
                    else:
                        joint_offset_deg = (targets[joint] - telem[joint])
                    #rospy.loginfo("joint offset is {:.2f} deg".format(joint_offset_deg))
                    
                    if desired_command_duration < VERY_SMALL_DURATION:
                        vel_raw = 0
                        rospy.loginfo("command duration is very small ({:.1f} ms)".format(desired_command_duration * 1000))
                        rospy.loginfo("setting velocity to 0")
                    else:
                        vel_raw = joint_offset_deg / desired_command_duration
                        
                    if desired_command_duration < -VERY_SMALL_DURATION:
                        vel_raw = 0
                        rospy.logerr("command duration is negative!!!")
                    #if desired_command_duration < SMALL_DURATION:
                    #    if joint_offset_deg > SMALL_ANGLE:
                    #        vel_raw = 0
                    #        print("[critical warning] command duration too low; calculated velocity is probably wrong. Setting it to 0")
                    #vel_raw *= VEL_MULT
                    vel_clipped = max(min(vel_raw, MAX_VEL[joint]), -MAX_VEL[joint])
                    if vel_clipped != vel_raw:
                        rospy.logerr("velocity clipped")
                        vel_raw = vel_clipped
                        self.stop()
                        raise RuntimeError("Velocity clipped")
                    velocities[joint] = vel_raw

                # Prepare command
                cmd.arm_enable = True
                cmd.cmd_type = cmd.CMD_VELOCITY

                goal_reached = [False, False, False, False, False, False]

                #rospy.loginfo("TARGET: {:.6f} {:.6f} {:.6f} {:.6f} {:.6f} {:.6f}".format(*targets))
                #rospy.loginfo("TELEMETRY: {:.6f} {:.6f} {:.6f} {:.6f} {:.6f} {:.6f}".format(*telem))
                #rospy.loginfo("VELOCITIES: {:.6f} {:.6f} {:.6f} {:.6f} {:.6f} {:.6f}".format(*velocities))

                while not rospy.is_shutdown():
                    telem = self._telem()
                    if telem is None:
                        rospy.logfatal("telemetry invalid, aborting!")
                        self.stop()
                        return False

                    # Check if goal reached
                    for joint in range(6):
                        joint_velocity = velocities[joint]
                        # if joint == 3 or joint == 5:
                        #     joint_velocity = -joint_velocity
                        if self._check_angle(targets[joint], telem[joint], joint_velocity, joint_tolerance):
                            goal_reached[joint] = True

                    actual_duration = rospy.get_time() - start_timestamp - point_start_timestamp
                    
                    # Add a small (~1s) duration to avoid problems when desired_command_duration
                    # is very small (close to arm response latency)
                    if actual_duration > desired_command_duration * DURATION_TOLERANCE + TIMEOUT_OFFSET:
                        rospy.logerr("Arm took too long to reach goal.")
                        raise JointTrajectoryTimeoutException()

                    if all(goal_reached):
                        # Move on to next waypoint
                        prev_lateness = actual_duration - desired_command_duration
                        overall_lateness += max(0, prev_lateness)
                        rospy.loginfo("reached waypoint {} within {:.3f}s of target".format(i, prev_lateness))
                        break
                
                    #rospy.logdebug(('{:.5f} '*6).format(*[target-current for target, current in zip(targets, telem)]))
                    #print(telem)
                    #print(goal_reached)
                    #print(velocities)
                    #print('---')

                    cmd.joint_1 = velocities[0]
                    cmd.joint_2 = velocities[1]
                    cmd.joint_3 = -velocities[2]
                    cmd.joint_4 = velocities[3]
                    cmd.joint_5 = -velocities[4]
                    cmd.joint_6 = velocities[5]

                    self.pub.publish(cmd)
                    self.r.sleep()

            overall_lateness = rospy.get_time() - start_timestamp - joint_trajectory.points[-1].time_from_start.to_sec()
            rospy.loginfo("finished trajectory execution within {:.3f}s of planned time".format(overall_lateness))
            self.stop()
            self.r.sleep()
            return True
        except JointTrajectoryTimeoutException:
            rospy.logerr("stopped execution due to timeout trying to reach target pose.")
            self.stop()
            return False
        except RuntimeError as ex:
            rospy.logerr("Exception during trajectory execution: " + str(ex))
            self.stop()
            return False

        
