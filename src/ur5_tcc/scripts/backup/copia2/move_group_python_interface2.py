#!/usr/bin/env python3

## INICIO_TUTORIAL imports
##
## To use the Python MoveIt interfaces, we will import the `moveit_commander`_ namespace.
## This namespace provides us with a `MoveGroupCommander`_ class, a `PlanningSceneInterface`_ class,
## and a `RobotCommander`_ class. More on these below. We also import `rospy`_ and some messages that we will use:
##

# Python 2/3 compatibility imports
import sys
import copy
import rospy
import moveit_commander
import moveit_msgs.msg
import geometry_msgs.msg

try:
    from math import pi, tau, dist, fabs, cos
except:  # For Python 2 compatibility
    from math import pi, fabs, cos, sqrt

    tau = 2.0 * pi

    def dist(p, q):
        return sqrt(sum((p_i - q_i) ** 2.0 for p_i, q_i in zip(p, q)))


from std_msgs.msg import String
from moveit_commander.conversions import pose_to_list

## FIM_TUTORIAL


def all_close(goal, actual, tolerance):
    """
    Convenience method for testing if the values in two lists are within a tolerance of each other.
    For Pose and PoseStamped inputs, the angle between the two quaternions is compared (the angle
    between the identical orientations q and -q is calculated correctly).
    @param: goal       A list of floats, a Pose or a PoseStamped
    @param: actual     A list of floats, a Pose or a PoseStamped
    @param: tolerance  A float
    @returns: bool
    """
    if type(goal) is list:
        for index in range(len(goal)):
            if abs(actual[index] - goal[index]) > tolerance:
                return False

    elif type(goal) is geometry_msgs.msg.PoseStamped:
        return all_close(goal.pose, actual.pose, tolerance)

    elif type(goal) is geometry_msgs.msg.Pose:
        x0, y0, z0, qx0, qy0, qz0, qw0 = pose_to_list(actual)
        x1, y1, z1, qx1, qy1, qz1, qw1 = pose_to_list(goal)
        # Euclidean distance
        d = dist((x1, y1, z1), (x0, y0, z0))
        # phi = angle between orientations
        cos_phi_half = fabs(qx0 * qx1 + qy0 * qy1 + qz0 * qz1 + qw0 * qw1)
        return d <= tolerance and cos_phi_half >= cos(tolerance / 2.0)

    return True


class MoveGroupPythonInterface(object):
    """MoveGroupPythonInterface"""

    def __init__(self):
        super(MoveGroupPythonInterface, self).__init__()

        ## INICIO_TUTORIAL setup
        ##
        ## First initialize `moveit_commander`_ and a `rospy`_ node:
        moveit_commander.roscpp_initialize(sys.argv)
        #rospy.init_node("move_group_python_interface", anonymous=True)

        ## Instantiate a `RobotCommander`_ object. Provides information such as the robot's
        ## kinematic model and the robot's current joint states
        robot = moveit_commander.RobotCommander()

        ## Instantiate a `PlanningSceneInterface`_ object.  This provides a remote interface
        ## for getting, setting, and updating the robot's internal understanding of the
        ## surrounding world:
        scene = moveit_commander.PlanningSceneInterface()

        ## Instantiate a `MoveGroupCommander`_ object.  This object is an interface
        ## to a planning group (group of joints).  In this tutorial the group is the primary
        ## arm joints in the Panda robot, so we set the group's name to "panda_arm".
        ## If you are using a different robot, change this value to the name of your robot
        ## arm planning group.
        ## This interface can be used to plan and execute motions:
        group_name = "manipulator"
        move_group = moveit_commander.MoveGroupCommander(group_name)
 
        ## Create a `DisplayTrajectory`_ ROS publisher which is used to display
        ## trajectories in Rviz:
        display_trajectory_publisher = rospy.Publisher(
            "/move_group/display_planned_path",
            moveit_msgs.msg.DisplayTrajectory,
            queue_size=20,
        )

        ## FIM_TUTORIAL

        ## INICIO_TUTORIAL basic_info
        ##
        ## Conseguindo Informações Básicas
        ## ^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^
        
        # We can get the name of the reference frame for this robot:
        planning_frame = move_group.get_planning_frame()
        
        # We can also print the name of the end-effector link for this group:
        eef_link = move_group.get_end_effector_link()
        
        # We can get a list of all the groups in the robot:
        group_names = robot.get_group_names()
        
        ## FIM_TUTORIAL

        # Variáveis diversas
        self.robot = robot
        self.scene = scene
        self.move_group = move_group
        self.display_trajectory_publisher = display_trajectory_publisher
        self.planning_frame = planning_frame
        self.eef_link = eef_link
        self.group_names = group_names
        
        # Motion Planning
        self.planning_time = 0
        self.execution_time = 0
        self.trajectory_message = ""
        self.position_name = ""        

    def motion_planning(self):

        ## INICIO_TUTORIAL motion_planning
        ##
        ## Planning to a Joint Goal
        ## ^^^^^^^^^^^^^^^^^^^^^^^^
        ## The Panda's zero configuration is at a `singularity <https://www.quora.com/Robotics-What-is-meant-by-kinematic-singularity>`_, so the first
        ## thing we want to do is move it to a slightly better configuration.
        ## We use the constant `tau = 2*pi <https://en.wikipedia.org/wiki/Turn_(angle)#Tau_proposals>`_ for convenience:
        # We get the joint values from the group and change some of the values:
        
        # Após realizar o planejamento de movimento do robô, a função ''plan()''
        # retorna uma tupla, que contém informações como: confirmação de sucesso,
        # a trajetória que o robô deve realizar, o tempo de planejamento e um
        # código de erro, caso exista.
        (success_flag,
         trajectory_message,
         planning_time,
         error_code) = self.move_group.plan()
                
        self.planning_time = planning_time
        self.trajectory_message = trajectory_message 

        ## FIM_TUTORIAL

        # For testing:
        # current_joints = self.move_group.get_current_joint_values()
        # return all_close(joints, current_joints, 0.01)

    def execute_plan(self):

        ## INICIO_TUTORIAL execute_plan
        ##
        ## Executing a Plan
        ## ^^^^^^^^^^^^^^^^
        ## Use execute if you would like the robot to follow
        ## the plan that has already been computed:
        
        tempo_inicial = rospy.get_time()
        
        self.move_group.execute(self.trajectory_message, wait=True)
        
        # Chamar a função ''stop()'' garante que não há nenhum movimento residual
        self.move_group.stop()

        tempo_final = rospy.get_time()
       
        tempo_execucao = tempo_final - tempo_inicial

        self.execution_time = tempo_execucao

        ## **Note:** The robot's current joint state must be within some tolerance of the
        ## first waypoint in the `RobotTrajectory`_ or ``execute()`` will fail
        
        ## FIM_TUTORIAL
    
    def config_joints(self):
        joint_goal = self.move_group.get_current_joint_values()
        joint_goal[0] = 0           #shoulder_pan_joint
        joint_goal[1] = -tau / 8    #shoulder_lift_joint
        joint_goal[2] = -tau / 8    #elbow_joint
        joint_goal[3] = -tau / 4    
        joint_goal[4] = 0           
        joint_goal[5] = tau / 6  # 1/6 of a turn

        joint_goal1 = self.move_group.get_current_joint_values()
        joint_goal1[0] = 0          #shoulder_pan_joint
        joint_goal1[1] = -tau / 4   #shoulder_lift_joint
        joint_goal1[2] = -tau / 8   #elbow_joint        
        joint_goal1[3] = -tau / 4
        joint_goal1[4] = 0
        joint_goal1[5] = tau / 6  # 1/6 of a turn

        joint_goal2 = self.move_group.get_current_joint_values()
        joint_goal2[0] = 0          #shoulder_pan_joint
        joint_goal2[1] = -tau / 8   #shoulder_lift_joint
        joint_goal2[2] =  tau / 8   #elbow_joint 
        joint_goal2[3] = -tau / 4
        joint_goal2[4] = 0
        joint_goal2[5] = tau / 6  # 1/6 of a turn

        self.move_group.remember_joint_values("goal1", joint_goal)
        self.move_group.remember_joint_values("goal2", joint_goal1)
        self.move_group.remember_joint_values("goal3", joint_goal2)
       
    def set_planner(self, plan_number):
        if (plan_number == 1):
            self.move_group.set_planner_id("RRT")
        
        if (plan_number == 2):
            self.move_group.set_planner_id("PRM")
        
        if (plan_number == 3):
            self.move_group.set_planner_id("TRRT")
    
        if (plan_number == 4):
            self.move_group.set_planner_id("RRTConnect")
        
        if (plan_number == 5):
            self.move_group.set_planner_id("SPARS")
    
    def set_position(self, position_number):

        position_name = ""
        
        if (position_number == 1):
            position_name = "up"
            self.move_group.set_named_target(position_name)
        if (position_number == 2):
            position_name = "goal1"
            self.move_group.set_named_target(position_name)
        if (position_number == 3):
            position_name = "goal2"
            self.move_group.set_named_target(position_name)
        if (position_number == 4):
            position_name = "goal3"
            self.move_group.set_named_target(position_name)
        if (position_number == 5):
            position_name = "home"
            self.move_group.set_named_target(position_name)

        self.position_name = position_name

    def display_trajectory(self, plan):
        robot = self.robot
        display_trajectory_publisher = self.display_trajectory_publisher

        display_trajectory = moveit_msgs.msg.DisplayTrajectory()
        display_trajectory.trajectory_start = robot.get_current_state()
        display_trajectory.trajectory.append(plan)
        # Publish
        display_trajectory_publisher.publish(display_trajectory)
