#!/usr/bin/env python2
from __future__ import print_function

import sys
import time

import manipulator_tools
import painting_tools


def check_execution_status(executed):
    # Checks the return code of the execute function 
    if not executed:
        raise RuntimeError("Error while going to location. The execution return code was False")


def initial_canvas_home(arm):
    print("======== Moving arm to canvas home position")
    arm.velocity_mode_fast()
    check_execution_status(arm.move_to_joint_positions(manipulator_tools.JointPositions.CANVAS_HOME))
    time.sleep(.1) 
    arm.velocity_mode_precise()
    check_execution_status(arm.move_to_joint_positions(manipulator_tools.JointPositions.CANVAS_HOME))


def interactive_depth_config(arm):
    print("======== Canvas depth configuration")
    print("u = upward, d = downward by 2.5 mm")
    depth = 0
    while True:
        command = raw_input("Type u/d/confirm to set depth: ")
        if command == "u":
            depth = -0.0025
        elif command == "d":
            depth = 0.0025
        elif command == "confirm":
            break
        else:
            print("Invalid input.")
            continue
        arm.move_relative_pose(z=-depth,
                               cartesian=True,
                               cartesian_resolution=0.0025)
    check_execution_status(arm.move_relative_pose(z=manipulator_tools.DOWNWARD_OFFSET,
                           cartesian=True,
                           cartesian_resolution=0.0025))
    time.sleep(.1)
    return arm.location()


def paint_with_pen(arm):
    paths = painting_tools.read_dxf('/root/drawing.dxf')

    plans = []
    
    print("======== Planning cartesian paths")
    arm.velocity_mode_precise()
    wp, plan, fraction = arm.plan_cartesian_xy(paths)
    print("Fraction of desired trajectory completed by plan: {}%".format(fraction*100))
    
    # Execute trajectory
    if fraction > 0.99:
        print("Executing plan")
        arm.execute_plan(plan, wp)
        print("Plan executed.")
    else:
        print("[!] Fraction not high enough. Will not execute plan.")


def paint(arm, filename):
    actions = painting_tools.to_action_sequence(painting_tools.read_painting(filename))

    for action in actions:
        print("* executing action: {}".format(str(action)))
        
        if isinstance(action, painting_tools.PaintingAction.Color):
            arm.velocity_mode_fast()
            check_execution_status(arm.move_to_location_cartesian(arm.target_locations.brush_home, resolution=0.02))
            arm.velocity_mode_precise()
            time.sleep(.1) 
            check_execution_status(arm.move_to_location_cartesian(arm.target_locations.brush_home, resolution=0.005))
            time.sleep(.1)
        elif isinstance(action, painting_tools.PaintingAction.Stroke):
            paths = action.stroke

            # Move to the point over the desired starting location of the stroke
            arm.velocity_mode_fast()
            check_execution_status(arm.move_to_location_cartesian(arm.target_locations.canvas_home, resolution=0.02))
            time.sleep(.1) 
            arm.velocity_mode_precise()
            check_execution_status(arm.move_to_location_cartesian(arm.target_locations.canvas_home, resolution=0.0025))
            time.sleep(.1)

            # Plan trajectory
            arm.velocity_mode_precise()
            wp, plan, fraction = arm.plan_cartesian_xy(paths)
            print("  - fraction of desired trajectory completed by plan: {}%".format(fraction*100))
    
            # Execute trajectory
            if fraction > 0.99:
                print("  - executing plan")
                check_execution_status(arm.execute_plan(plan, wp))
                time.sleep(.1) 
                print("  - plan executed")
            else:
                print("  - [!] Fraction not high enough. Will not execute plan.")
        else:
            print("* [!] unsupported action: {}".format(str(action)))

    print("done!")
    check_execution_status(arm.move_to_location_cartesian(arm.target_locations.canvas_home))
    time.sleep(.1)

        
if __name__ == "__main__":
    arm = manipulator_tools.ManipulatorController(sys.argv)

    # Move to desired starting position
    initial_canvas_home(arm)
    
    # Interactively set depth
    calibrated_canvas_home = interactive_depth_config(arm)
    arm.target_locations.canvas_home = calibrated_canvas_home

    # Demo going to brush home and back
    # print("======== Moving to brush home")
    # arm.move_to_location_cartesian(arm.target_locations.brush_home)
    # print("         ...ok")
    # time.sleep(2)
    # print("======== Moving to canvas home")
    # arm.move_to_location_cartesian(arm.target_locations.canvas_home)
    # print("         ...ok")
    
    # paint_with_pen(arm)
    paint(arm, '/root/drawing.dxf')
