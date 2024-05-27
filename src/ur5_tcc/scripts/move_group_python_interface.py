#!/usr/bin/env python3

# ========================================================== #
#               move_group_python_interface.py               # 
# ========================================================== #
# Projeto desenvolvido por Danilo Dias Conceição da Silva    #
# como parte do Trabalho de Conclusão de Curso (TCC) do      #
# curso de Engenharia da Computação da Universidade Federal  #
# da Bahia (UFBA)                                            #
# ========================================================== #

## INICIO_DESCRIÇÃO imports
##
## Para usar a interface Python para o MoveIt, é necessário importar o 
## namespace 'moveit_commander'. Esse namespace nos permite utilizar as classes 
## 'MoveGroupCommander', 'PlanningSceneInterface', e a classe 'RobotCommander'.
## Outro import necessário é o namespace 'rospy', que é o cliente Python
## para o ROS. Através dele é possível utilizar diversos comandos do ROS via Python
## inclusive criar nós, publicar e inscrever em tópicos, entre outros.

import sys
import rospy
import moveit_commander
import moveit_msgs.msg
from math import tau    # (2*pi) | 6.283185307179586 rad | 360°

## FIM_DESCRIÇÃO imports


class MoveGroupPythonInterface(object):

    def __init__(self):
        super(MoveGroupPythonInterface, self).__init__()

        ## INICIO_DESCRIÇÃO setup
        ##
        ## Inicialização do 'moveit_commander'
        moveit_commander.roscpp_initialize(sys.argv)
        
        ## Instancia um objeto 'RobotCommander'.
        ## Fornece informações como o modelo cinemático
        ## e os estados atuais dos joints do robô.
        robot = moveit_commander.RobotCommander()

        ## Instancia um objeto 'PlanningSceneInterface'.
        ## Fornece uma interface remota para obter, configurar,
        ## e atualizar o entendimento interno do robô sobre o 
        ## mundo ao seu redor
        scene = moveit_commander.PlanningSceneInterface()

        ## Instancia um objeto 'MoveGroupCommander'.
        ## Esse objeto é uma interface para um grupo de planejamento (grupo de joints).
        ## Neste projeto, o grupo utilizado são os principais joints do braço no robô UR5,
        ## cujo nome está definido como 'manipulator'.
        ## Esta interface é utilizada para planejar e executar movimentos.
        group_name = "manipulator"
        move_group = moveit_commander.MoveGroupCommander(group_name)
 
        ## Cria um publisher do ROS para publicar mensagens do tipo
        ## 'DisplayTrajectory', que é usado para mostrar a trajetória
        ## do robô no Rviz.
        display_trajectory_publisher = rospy.Publisher(
            "/move_group/display_planned_path",
            moveit_msgs.msg.DisplayTrajectory,
            queue_size=20,
        )
        ##
        ## FIM_DESCRIÇÃO setup


        ## INICIO_DESCRIÇÃO basic_info
        ##
        ## Obtendo Informações Básicas
        ## ^^^^^^^^^^^^^^^^^^^^^^^^^^^
        
        ## Obter o nome do frame de referência para o robô:
        planning_frame = move_group.get_planning_frame()
        
        ## Obter o nome do end-effector link do robô:
        eef_link = move_group.get_end_effector_link()
        
        ## Obter uma lista de todos os grupos de links no robot:
        group_names = robot.get_group_names()
        ##
        ## FIM_DESCRIÇÃO basic_info


        # Atributos diversos
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


    ##### CONFIGURAÇÃO DOS JOINTS #####

    def config_joints(self):

        ## INICIO_DESCRIÇÃO config_joints
        ##
        ## Configurando os joints do robô
        ## ^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^
        ## Esta função configura os valores do joints para cada posição do robô
        ## que será utilizada posteriormente no seu planejamento de movimentos.
        ## Os valores iniciais dos joints são obtidos através da função
        ## 'get_current_joint_values()' e alterados para definir a posição desejada.
        ## A constante 'tau = 2*pi' está sendo utilizada como referêcia para
        ## calcular os valores dos joints de cada posição alvo do planejamento.

        ## Configurando os valores dos joints da posição 'goal0'
        joint_goal0 = self.move_group.get_current_joint_values()
        joint_goal0[0] =  tau / 4   # shoulder_pan_joint
        joint_goal0[1] = -tau / 4   # shoulder_lift_joint
        joint_goal0[2] = -tau / 4   # elbow_joint 
        joint_goal0[3] = 0          # wrist_1_joint
        joint_goal0[4] = 0          # wrist_2_joint
        joint_goal0[5] = 0          # wrist_3_joint
        
        ## Configurando os valores dos joints da posição 'goal1'
        joint_goal1 = self.move_group.get_current_joint_values()
        joint_goal1[0] = 0          # shoulder_pan_joint
        joint_goal1[1] = -tau / 8   # shoulder_lift_joint
        joint_goal1[2] = -tau / 8   # elbow_joint
        joint_goal1[3] = 0          # wrist_1_joint    
        joint_goal1[4] = 0          # wrist_2_joint           
        joint_goal1[5] = 0          # wrist_3_joint

        ## Configurando os valores dos joints da posição 'goal2'
        joint_goal2 = self.move_group.get_current_joint_values()
        joint_goal2[0] = 0          # shoulder_pan_joint
        joint_goal2[1] = -tau / 4   # shoulder_lift_joint
        joint_goal2[2] = -tau / 8   # elbow_joint        
        joint_goal2[3] = 0          # wrist_1_joint
        joint_goal2[4] = 0          # wrist_2_joint
        joint_goal2[5] = 0          # wrist_3_joint

        ## Configurando os valores dos joints da posição 'goal3'
        joint_goal3 = self.move_group.get_current_joint_values()
        joint_goal3[0] = 0          # shoulder_pan_joint
        joint_goal3[1] = -tau / 8   # shoulder_lift_joint
        joint_goal3[2] =  tau / 8   # elbow_joint 
        joint_goal3[3] = 0          # wrist_1_joint
        joint_goal3[4] = 0          # wrist_2_joint
        joint_goal3[5] = 0          # wrist_3_joint
        
        ## Após definir os valores dos joints de cada posição, eles serão
        ## armazenados e associados a um nome, através do uso da função
        ## 'remember_joint_values'. A função 'set_position()' utilizará estes
        ## valores posteriormente através dos nomes atribuidos aos grupos de
        ## joints. 
        self.move_group.remember_joint_values("goal0", joint_goal0)
        self.move_group.remember_joint_values("goal1", joint_goal1)
        self.move_group.remember_joint_values("goal2", joint_goal2)
        self.move_group.remember_joint_values("goal3", joint_goal3)
        ##
        ## FIM_DESCRIÇÃO config_joints
         
    def set_position(self, position_number):

        ## INICIO_DESCRIÇÃO set_position
        ##
        ## Definindo as posições alvo
        ## ^^^^^^^^^^^^^^^^^^^^^^^^^^
        ## Esta função define as posições alvo para o planejamento  
        ## de movimentos do robô de acordo com uma ordem estabelecida
        ## no projeto. Para cada planejamento, a ordem é checada e uma
        ## posição é definida através da função 'set_named_target()',
        ## que utiliza os nomes do grupos de joints armazenados após a 
        ## chamada da função 'config_joints()'.
        if (position_number == 1):
            self.position_name = "goal0"
            self.move_group.set_named_target(self.position_name)
        elif (position_number == 2):
            self.position_name = "goal1"
            self.move_group.set_named_target(self.position_name)
        elif (position_number == 3):
            self.position_name = "goal2"
            self.move_group.set_named_target(self.position_name)
        elif (position_number == 4):
            self.position_name = "goal3"
            self.move_group.set_named_target(self.position_name)
        elif (position_number == 5):
            self.position_name = "home"
            self.move_group.set_named_target(self.position_name)
        ## OBS: A posição 'home' está previamente definida nos
        ##      arquivos de configuração do robô UR5.    
        ##
        ## FIM_DESCRIÇÃO set_position


    ##### PLANEJAMENTO E EXECUÇÃO #####

    def set_planner(self, plan_number):
        
        ## INICIO_DESCRIÇÃO set_planner
        ##
        ## Definindo os planejadores
        ## ^^^^^^^^^^^^^^^^^^^^^^^^^
        ## Esta função determina qual será o planejador utilizado
        ## no planejamento de movimentos de acordo com uma ordem
        ## estabelecida no projeto. Para cada mudança de planejador,
        ## a ordem é checada e um planejador é definido através da função
        ## 'set_planner_id()' passando como argumento o nome do planejador.
        if (plan_number == 1):
            self.move_group.set_planner_id("BFMT")
        elif (plan_number == 2):
            self.move_group.set_planner_id("FMT")
        elif (plan_number == 3):
            self.move_group.set_planner_id("PRM")
        elif (plan_number == 4):
            self.move_group.set_planner_id("RRT")
        elif (plan_number == 5):
            self.move_group.set_planner_id("SPARS")
        ##
        ## FIM_DESCRIÇÃO set_planner

    def motion_planning(self):

        ## INICIO_DESCRIÇÃO motion_planning
        ##
        ## Planejando a trajetória do robô
        ## ^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^
        ## Após configurar os joints e definir o planejador que será utilizado, 
        ## o planejamento de movimentos do robô poderá ser realizado. Este
        ## planejamento é feito através da chamada da função 'plan()', que utiliza
        ## os valores dos joints definidos na função 'set_position()' para conhecer 
        ## a trajetória de deslocamento do robô que será planejada.         
        
        ## Após realizar o planejamento de movimentos do robô, a função 'plan()'
        ## retorna uma tupla, que contém informações como: confirmação de sucesso,
        ## a trajetória que o robô deve realizar, o tempo de planejamento e um
        ## código de erro, caso exista.
        (success_flag,
         trajectory_message,
         planning_time,
         error_code) = self.move_group.plan()

        ## Armazena no atributo desta classe o tempo de planejamento
        self.planning_time = planning_time

        ## Armazena no atributo desta classe a trajetória a ser realizada pelo robô
        self.trajectory_message = trajectory_message 
        ##
        ## FIM_DESCRIÇÃO motion_planning

    def execute_plan(self):

        ## INICIO_DESCRIÇÃO execute_plan
        ##
        ## Executando um planejamento de movimentos
        ## ^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^
        ## Esta função utiliza a trajetória do robô, estabelecida após
        ## o planejamento de movimentos, para executar a sua movimentação.
        ## Dentro desta função também é calculado o tempo decorrido
        ## para realizar a execução do robô.
        
        ## Obtém o tempo do inicio da execução de movimentos do robô
        initial_time = rospy.get_time()
        
        ## Realiza a execução do robô utilizando a trajetória armazenada no
        ## atributo 'trajectory_message' desta classe após o planejamento de
        ## movimentos.
        self.move_group.execute(self.trajectory_message, wait=True)
        
        ## Chamar a função 'stop()' garante que não há nenhum movimento residual
        self.move_group.stop()

        ## Obtém o tempo ao final da execução de movimentos do robô
        final_time = rospy.get_time()
       
        ## Calcula o tempo de execução do robô e armazena o valor no
        ## atributo desta classe.
        self.execution_time = final_time - initial_time
        ##
        ## FIM_DESCRIÇÃO execute_plan
    

    ##### VISUALIZAÇÃO DO PLANEJAMENTO NO RVIZ #####

    def display_trajectory(self, plan):
        
        ## Define o tipo de mensagem que será publicada
        display_trajectory = moveit_msgs.msg.DisplayTrajectory()

        ## Obtém o estado atual do robô e define o inicio da trajetória dele.
        display_trajectory.trajectory_start = self.robot.get_current_state()
        
        ## Define o planejamento que será mostrado no Rviz
        display_trajectory.trajectory.append(plan)
                
        ## Publica a mensagem no tópico '/move_group/display_planned_path'
        ## e mostra o planejamento no Rviz.
        self.display_trajectory_publisher.publish(display_trajectory)
