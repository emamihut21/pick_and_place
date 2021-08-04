#!/usr/bin/env python

from __future__ import print_function
from six.moves import input
import sys
import copy
import rospy
import moveit_commander
import moveit_msgs.msg
import geometry_msgs.msg
import random
from math import sin, cos, pi, radians
from std_msgs.msg import String
from moveit_commander.conversions import pose_to_list
import numpy as np    
#import scipy
from tf.transformations import quaternion_from_euler, euler_matrix, quaternion_slerp, quaternion_from_matrix 



def all_close(goal, actual, tolerance):
  
  all_equal = True
  if type(goal) is list:
    for index in range(len(goal)):
      if abs(actual[index] - goal[index]) > tolerance:
        return False

  elif type(goal) is geometry_msgs.msg.Pose:
    return all_close(pose_to_list(goal), pose_to_list(actual), tolerance)

  return True



class MoveGroupPythonPalletizing(object):
  
  def __init__(self):
    super(MoveGroupPythonPalletizing, self).__init__()

    moveit_commander.roscpp_initialize(sys.argv)
    rospy.init_node('palletizing', anonymous=True)

    robot = moveit_commander.RobotCommander()

    scene = moveit_commander.PlanningSceneInterface()

    group_name = "manipulator"
    move_group = moveit_commander.MoveGroupCommander(group_name)

    #planning_frame = move_group.get_planning_frame()
    #print("============ Planning frame: %s" % planning_frame)

    eef_link = move_group.get_end_effector_link()
    print("============ End effector link: %s" % eef_link)

    group_names = robot.get_group_names()
    print("============ Available Planning Groups:", robot.get_group_names())

    print("============ Printing robot state")
    print(robot.get_current_state())
    print("")

    self.box_name = ''
    self.robot = robot
    self.scene = scene
    self.move_group = move_group
    #self.planning_frame = planning_frame
    self.eef_link = eef_link
    self.group_names = group_names



 ####################### GO TO BOX POSE #########################
 
  def go_to_box_pose(self, box_position_x, box_position_y, box_position_z, box_roll, box_pitch, box_yaw):

    move_group = self.move_group                                                  

    TranslationMatrix_on_z = np.matrix([[1, 0, 0, 0  ],       #Translation matrix on z with half of box's length so it's not in collision
                                        [0, 1, 0, 0  ],
                                        [0, 0, 1, 0.05],
                                        [0, 0, 0, 1  ]])
    RotationMatrix_box = euler_matrix(box_roll, box_pitch, box_yaw)       #define rotation matrix from euler angles of the box
    RotationMatrix_box[:3,3] = np.array([box_position_x, box_position_y, box_position_z])     #last column = position of the box
    TransformationMatrix = RotationMatrix_box.dot(TranslationMatrix_on_z)                   #Translation on z with 0.05 on the box frame for the box collision; this will grab the box from below
    
    #grabing from above => rot with 180 on y
    RotationMatrin_on_y = np.matrix([[cos(pi), 0, sin(pi), 0 ],
                                     [ 0      , 1, 0       , 0 ],
                                     [-sin(pi), 0, cos(pi), 0 ],
                                     [0       , 0, 0       , 1 ]])
    TransformationMatrix_box = TransformationMatrix.dot(RotationMatrin_on_y)

    #define pose_box as type pose so it can be planned
    pose_box = geometry_msgs.msg.Pose()
    box_quaternions = quaternion_from_matrix(TransformationMatrix_box)
    pose_box.orientation.x = box_quaternions[0]
    pose_box.orientation.y = box_quaternions[1]
    pose_box.orientation.z = box_quaternions[2]
    pose_box.orientation.w = box_quaternions[3]
    pose_box.position.x = TransformationMatrix_box[0,3]
    pose_box.position.y = TransformationMatrix_box[1,3]
    pose_box.position.z = TransformationMatrix_box[2,3]
    print(pose_box)
    self.pose_box = pose_box
    move_group.set_pose_target(pose_box)
    plan = move_group.go(wait=True)
    
    
   #################################################################



  ############## INTERPOLATION + WAYPOINTS #########################
   
  def go_to_pose_goal(self, goal_position_x, goal_position_y, goal_position_z, goal_roll, goal_pitch, goal_yaw, number_of_steps, transl_x, transl_y, transl_z):

    move_group = self.move_group
   
    current_pose = self.move_group.get_current_pose().pose 
    final_pose = geometry_msgs.msg.Pose() 
    waypoint = geometry_msgs.msg.Pose()
    list_of_waypoints = []
    
    #First we need the initial position of the eef, which is given by moveit_commander and final position(read from keyboard)
    #initial position
    x_start_position = current_pose.position.x
    y_start_position = current_pose.position.y
    z_start_position = current_pose.position.z
    
    #orientation quaternion
    Translation_on_z = np.matrix([[1, 0, 0, 0  ],       #Translation matrix on z 
                                 [0, 1, 0, 0 ],
                                 [0, 0, 1, 0.05],
                                 [0, 0, 0, 1  ]])
    RotationMatrix = euler_matrix(goal_roll, goal_pitch, goal_yaw)       #define transformation matrix from euler angles of the box
    RotationMatrix[:3,3] = np.array([goal_position_x, goal_position_y, goal_position_z])     
    TransformationMatrix = RotationMatrix.dot(Translation_on_z)                   #Translation on z with 0.05 on the box frame for the box collision; this will grab the box from below
    
    #grabing from above => rot with 180 on y
    Rotation_on_y = np.matrix([[cos(pi), 0, sin(pi), 0 ],
                              [ 0      , 1, 0       , 0 ],
                              [-sin(pi), 0, cos(pi), 0 ],
                              [0       , 0, 0       , 1 ]])
    TransformationMatrix_goal = TransformationMatrix.dot(Rotation_on_y)
    x_final_position = TransformationMatrix_goal[0,3] + transl_x
    y_final_position = TransformationMatrix_goal[1,3] + transl_y 
    z_final_position = TransformationMatrix_goal[2,3] + transl_z + 0.2   #so it goes first above the location of the box with 0.2

    quaternion = quaternion_from_matrix(TransformationMatrix_goal)    #takes the quaternions from TF matrix
    final_pose.orientation.x = quaternion[0]
    final_pose.orientation.y = quaternion[1]
    final_pose.orientation.z = quaternion[2]
    final_pose.orientation.w = quaternion[3]
    quat0 = [current_pose.orientation.x, current_pose.orientation.y, current_pose.orientation.z, current_pose.orientation.w]
    quat1 = [final_pose.orientation.x, final_pose.orientation.y, final_pose.orientation.z, final_pose.orientation.w]


    for i in range (number_of_steps+1):       #i for numbering the moves
        step = float(i)/(number_of_steps)    
        x = (1-step) * x_start_position + step * x_final_position      #linear interpolation formula for position
        y = (1-step) * y_start_position + step * y_final_position
        z = (1-step) * z_start_position + step * z_final_position
        waypoint.position.x = x
        waypoint.position.y = y
        waypoint.position.z = z
        quat = quaternion_slerp(quat0, quat1, step)     #slerp interpolation for orientation defined by quaternions
        waypoint.orientation.x = quat[0] 
        waypoint.orientation.y = quat[1]
        waypoint.orientation.z = quat[2]
        waypoint.orientation.w = quat[3]
        list_of_waypoints.append(copy.deepcopy(waypoint))  #add the waypoint at the end of the 'list_of_waypoints' 
    
    waypoint.position.z = waypoint.position.z - 0.2    #last waypoint to place the box decrease with 0.2 on z
    list_of_waypoints.append(copy.deepcopy(waypoint))

    print("waypoints", list_of_waypoints)
    
    (plan, fraction) = move_group.compute_cartesian_path(
                                   list_of_waypoints,   # waypoints to follow
                                   0.01,        # eef_step
                                   0.0,         # jump_threshold 
                                   1)           # avoid collision    

    move_group.execute(plan, wait=True)
    rospy.sleep(0.1)
    move_group.stop()


   ######################################################################



  ######################### GO TO initial POSE ###########################

  def go_to_initial_pose(self, timeout=4):

    move_group = self.move_group
    initial_pose = geometry_msgs.msg.Pose()
    initial_pose.orientation.w = 7.3123010772e-14     #initial orientation from moveit_commander
    initial_pose.orientation.x = -0.707106781188
    initial_pose.orientation.y = -7.31230107717e-14
    initial_pose.orientation.z = -0.707106781185
    initial_pose.position.x = 0.865
    initial_pose.position.y = 0.0
    initial_pose.position.z = 1.15

    move_group.set_pose_target(initial_pose)
    plan = move_group.go(wait=True)
    move_group.stop()
    move_group.clear_pose_targets()

    current_pose = self.move_group.get_current_pose().pose
    return all_close(initial_pose, current_pose, 0.01)

   ######################################################################



  def wait_for_state_update(self, box_is_known=False, box_is_attached=False, timeout=4):
    
    box_name = self.box_name
    scene = self.scene
    start = rospy.get_time()
    seconds = rospy.get_time()

    while (seconds - start < timeout) and not rospy.is_shutdown():
      attached_objects = scene.get_attached_objects([box_name])
      is_attached = len(attached_objects.keys()) > 0
      is_known = box_name in scene.get_known_object_names()
      if (box_is_attached == is_attached) and (box_is_known == is_known):
        return True
      rospy.sleep(0.1)
      seconds = rospy.get_time()
    return False


  
  def add_box(self, box_position_x, box_position_y, box_position_z, box_roll, box_pitch, box_yaw, counter, timeout=4):
    
    box_name = self.box_name
    scene = self.scene
    box_pose = geometry_msgs.msg.PoseStamped()
    box_pose.header.frame_id = "world"
    box_name = ("box"+str(counter))

    box_pose.pose.position.x = box_position_x
    box_pose.pose.position.y = box_position_y
    box_pose.pose.position.z = box_position_z 
    quaternion = quaternion_from_euler(box_roll, box_pitch, box_yaw)   #transf euler in quat
    box_pose.pose.orientation.x = quaternion[0]
    box_pose.pose.orientation.y = quaternion[1]
    box_pose.pose.orientation.z = quaternion[2]
    box_pose.pose.orientation.w = quaternion[3]
    scene.add_box(box_name, box_pose, size=(0.1, 0.2 ,0.1))
    self.box_name=box_name
    return self.wait_for_state_update(box_is_known=True, timeout=timeout)



  def attach_box(self, timeout=4): 

    box_name = self.box_name
    robot = self.robot
    scene = self.scene
    eef_link = self.eef_link
    grasping_group = 'manipulator'

    touch_links = robot.get_link_names(group=grasping_group)
    scene.attach_box(eef_link, box_name, touch_links=touch_links)
    # We wait for the planning scene to update.
    return self.wait_for_state_update(box_is_attached=True, box_is_known=False, timeout=timeout)



  def detach_box(self, timeout=4):

    box_name = self.box_name
    scene = self.scene
    eef_link = self.eef_link
    
    scene.remove_attached_object(eef_link, name=box_name)
    # We wait for the planning scene to update.
    return self.wait_for_state_update(box_is_known=True, box_is_attached=False, timeout=timeout)




def main():
 
  try:

    print("")
    print("Press Ctrl-D to exit at any time")
    print("")
    input("============ Press `Enter` to begin by setting up the moveit_commander ...")
    project = MoveGroupPythonPalletizing()

    box_roll = radians(0)
    box_pitch = radians(0)
    box_position_x = float(input('Insert the x coordinate for box: '))
    box_position_y = float(input('Insert the y coordinate for box: '))
    box_position_z = float(input('Insert the z coordinate for box: '))

    goal_position_x = float(input('Insert the x coordinate for placing the first box: '))
    goal_position_y = float(input('Insert the x coordinate for placing the first box: '))
    goal_position_z = float(input('Insert the x coordinate for placing the first box: '))

    counter_rows = 0
    counter_boxes_pallet = 0
    while counter_rows <= 3:
      counter_boxes_row = 0
      while counter_boxes_row <=2:

        box_yaw = radians(random.randint(0,180)) #so every box that appear in the plan will have a different rotation on z

        input("============ Press `Enter` to add a box to the planning scene ...")
        project.add_box(box_position_x, box_position_y, box_position_z, box_roll, box_pitch, box_yaw, counter_boxes_pallet)
        
        input("============ Press `Enter` to execute a movement to the box pose ...")
        project.go_to_box_pose(box_position_x, box_position_y, box_position_z, box_roll, box_pitch, box_yaw)
        
        input("============ Press `Enter` to attach the box to Fanuc robot ...")
        project.attach_box()

        if counter_rows % 2 == 0:
          if counter_boxes_row == 0:
            input("============ Press `Enter` to execute a movement to the pallet ...")                    
            project.go_to_pose_goal(goal_position_x, goal_position_y, goal_position_z, radians(0), radians(0), radians(0), 10, 0, 0, float(counter_rows)/(10))
          elif counter_boxes_row == 1:
            input("============ Press `Enter` to execute a movement to the pallet ...")                    
            project.go_to_pose_goal(goal_position_x, goal_position_y, goal_position_z , radians(0), radians(0), radians(90), 10, -0.155, -0.05, float(counter_rows)/(10))
          else:
            input("============ Press `Enter` to execute a movement to the pallet ...")                   
            project.go_to_pose_goal(goal_position_x, goal_position_y, goal_position_z , radians(0), radians(0), radians(90), 10, -0.155, 0.055, float(counter_rows)/(10))

        if counter_rows % 2 == 1:
          if counter_boxes_row == 0:
            input("============ Press `Enter` to execute a movement to the pallet ...")                    
            project.go_to_pose_goal(goal_position_x, goal_position_y, goal_position_z, radians(0), radians(0), radians(90), 10, -0.05, -0.05, float(counter_rows)/(10))
          elif counter_boxes_row == 1:
            input("============ Press `Enter` to execute a movement to the pallet ...")                    
            project.go_to_pose_goal(goal_position_x, goal_position_y, goal_position_z , radians(0), radians(0), radians(90), 10, -0.05, 0.055, float(counter_rows)/(10))
          else:
            input("============ Press `Enter` to execute a movement to the pallet ...")                    
            project.go_to_pose_goal(goal_position_x, goal_position_y, goal_position_z , radians(0), radians(0), radians(0), 10, -0.205, 0.0, float(counter_rows)/(10))



        input("============ Press `Enter` to place the box in the pallet ...")
        project.detach_box()
        
        input("============ Press `Enter` to execute a movement to retire pose ...")
        project.go_to_initial_pose()

        counter_boxes_pallet +=1
        counter_boxes_row +=1
      counter_rows +=1
      
      

    print("============ Palletizing SUCCED!")
  except rospy.ROSInterruptException:
    return
  except KeyboardInterrupt:
    return

if __name__ == '__main__':
  main()



