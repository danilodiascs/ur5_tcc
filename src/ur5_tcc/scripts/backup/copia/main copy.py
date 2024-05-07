#!/usr/bin/env python3

from move_group_python_interface import *
from aux_functions import *
from bag_record import *
from sensor_msgs.msg import JointState
import rosbag

def retorno(dados:JointState):
    # bag_tempo_final = bag.get_end_time()
    # bag_tempo_atual = rospy.get_time()
    
    # if (bag_tempo_atual > bag_tempo_final):
    #     bag.write('/joint_states', dados)

    if not bag.closed:
        bag.write('/joint_states', dados)
    #bag.write('/joint_states', dados.name)
    #bag.write('/joint_states', dados.position)

def main():
    try:
        global bag
        rospy.init_node('move_group_python_interface', anonymous=False)
        ur5_motion_planning = MoveGroupPythonInterface()
        ur5_motion_planning.config_joints()

        print_information(ur5_motion_planning.move_group, ur5_motion_planning.robot)

        print_menu()
        
        rospy.loginfo(rospy.get_name() + ' start')
        #rospy.set_param('~record_script', 'rosbag record /chatter -O recorded_bag_ur5.bag')
        #rospy.set_param('~record_folder', '/home/dd/Projetos_Ros/ws_ur5tcc/src/universal_robot/ur5_moveit_config/scripts/TCC/')

        #bag_recorder = RosbagRecord()
        #rospy.Subscriber("/chatter", String, retorno)
        rospy.Subscriber("/joint_states", JointState, retorno)

        plan_number = 1
        while (plan_number <= 1) and not rospy.is_shutdown():

            ur5_motion_planning.set_planner(plan_number)
            
            bag_name = f"Algoritmo_{ur5_motion_planning.move_group.get_planner_id()}.bag"
            bag = rosbag.Bag(bag_name, 'w')
            
            position_number = 1
            while (position_number <= 5) and not rospy.is_shutdown():
      
                position_name = ur5_motion_planning.set_position(position_number)

                (planning_time, trajectory_message) = ur5_motion_planning.motion_planning()

                execution_time = ur5_motion_planning.execute_plan(trajectory_message)

                print_results_terminal(ur5_motion_planning.move_group, planning_time, execution_time, position_name)

                #print_results_csv(ur5_motion_planning.move_group, planning_time, execution_time, position_name)

                rospy.sleep(2)

                #input("\nAperte ENTER para continuar...")

                position_number += 1

            bag.close()

            plan_number+=1
        
        #bag.close()

        #bag_recorder.stop_recording_handler()
            
        print("=== Python ur5_motion_planning demo complete!")

    except rospy.ROSInterruptException:
        return
    except KeyboardInterrupt:
        return

if __name__ == "__main__":
    main()
