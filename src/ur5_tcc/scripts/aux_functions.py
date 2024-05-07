#!/usr/bin/env python3

# ========================================================== #
#                     aux_functions.py                       # 
# ========================================================== #
# Projeto desenvolvido por Danilo Dias Conceição da Silva    #
# como parte do Trabalho de Conclusão de Curso (TCC) do      #
# curso de Engenharia da Computação da Universidade Federal  #
# da Bahia (UFBA)                                            #
# ========================================================== #

import os, csv, rosbag
from move_group_python_interface import *
from sensor_msgs.msg import JointState

def callbackSubscriber(dados:JointState):
    # Recebe os dados do tópico 'joint_states' e os grava no arquivo .bag
    # caso ele ainda esteja aberto
    if not bag_closed: 
        bag.write('/joint_states', dados)

def plan_and_execute():
    global bag # Variável global utilizada para gravação de arquivos .bag
    global bag_closed # Variável booleana para verificar se o arquivo .bag está fechado
    
    # Inicializa esta variável como verdadeira, impedindo que a gravação do
    # arquivo .bag comece, pois este arquivo ainda não foi inicializado 
    bag_closed = True 

    # Instancia o objeto que fará o planejamento de movimento do robô
    ur5_motion_planning = MoveGroupPythonInterface()
    
    # Configura os ângulos de cada joint do robô e atribui nomes
    # específicos para a posição do robô gerada através da variação
    # angular dos joints
    ur5_motion_planning.config_joints()
    
    # Imprime no terminal algumas informações sobre o robô utilizado,
    # cenário onde o robô está inserido, biblioteca de planejamento
    # de movimento utilizada e seus algoritmos, etc.
    # print_information(ur5_motion_planning)

    # Imprime no terminal o menu da aplicação em execução
    print_menu()
    
    # Define a numeração do primeiro planejador que será utilizado pelo Moveit
    plan_number = 1
    while (plan_number <= 1) and not rospy.is_shutdown():

        # Define o algoritmo de planejamento a ser utilizado,
        # de acordo com uma ordem estabelecida no projeto
        ur5_motion_planning.set_planner(plan_number)
        
        # Criação do bag atual utilizando o nome do Algoritmo de planejamento
        # definido na função 'set_planner()'
        bag_name = f"Algoritmo_{ur5_motion_planning.move_group.get_planner_id()}.bag"
        # bag = rosbag.Bag(bag_name, 'w')
        
        # Informa que o arquivo .bag está aberto e pronto para gravação
        # bag_closed = False 
        
        # Define a numeração da primeira posição alvo para o robô
        position_number = 1 
        while (position_number <= 5) and not rospy.is_shutdown():
            
            # Define a posição alvo para o robô de acordo com uma ordem
            # estabelecida no projeto
            ur5_motion_planning.set_position(position_number)

            # Inicializa o planejamento de movimento do robô
            ur5_motion_planning.motion_planning()

            # Executa a movimentação do robô para a posição desejada
            ur5_motion_planning.execute_plan()
            
            # Imprime no terminal os resultados do Planejamento de movimento 
            # e da Execução do robô 
            print_results_terminal(ur5_motion_planning)

            # Grava num arquivo CSV os dados resultantes do Planejamento de movimento 
            # e da Execução do robô
            # print_results_csv(ur5_motion_planning)

            # Define um intervalo de tempo de 2 segundos entre cada 
            # planejamento e execucão de movimentos do robô
            rospy.sleep(2)

            # input("\nAperte ENTER para continuar...")
            
            # Incrementa o contador do loop que alterna as posições do robô
            position_number += 1
        
        bag_closed = True # Informa que o arquivo .bag será fechado e encerra a gravação
        # bag.close() # Fecha o arquivo .bag
        
        #input("\nAperte ENTER para continuar...")

        # Incrementa o contador do loop que alterna os planejadores
        plan_number+=1
    
    # Exibe no terminal uma mensagem de encerramento da execução da aplicação
    print("\033[1;36m== Execução do UR5 Motion Planning concluída! ==\033[m")

def print_menu():
    print("")
    print("\033[1;36m------------------------------------------------\033[m")
    print("\033[1;36m        MoveIt MoveGroup Python Interface       \033[m")
    print("\033[1;36m------------------------------------------------\033[m")
    print("\033[1;36m           Pressione Ctrl-D para sair           \033[m")
    print("")

def print_information(motion_planning: MoveGroupPythonInterface):
        
    # Armazena informações sobre o robô
    planning_frame = motion_planning.planning_frame
    eef_link = motion_planning.eef_link
    robot_group_names = motion_planning.robot.get_group_names()
    robot_states = motion_planning.robot.get_current_state()

    # Armazena informações sobre os joints do robô
    named_targets = motion_planning.move_group.get_named_targets()
    named_target_values = motion_planning.move_group.get_named_target_values("up")
    remembered_joint_values = motion_planning.move_group.get_remembered_joint_values()

    # Armazena informações sobre os planejadores
    planners = motion_planning.move_group.get_interface_description()
    planner_id = motion_planning.move_group.get_planner_id()
    planning_pipeline_id = motion_planning.move_group.get_planning_pipeline_id()

    # Imprime informações sobre o robô no terminal
    print(f"=== Planning frame: {planning_frame}")
    print(f"=== End effector link: {eef_link}")
    print(f"=== Available Planning Groups: {robot_group_names}")
    print("=== Printing robot state")
    print(f"=== Robot States: {robot_states}")
        
    # Imprime informações sobre os joints do robô no terminal
    print(f"Targets Nomeados: {named_targets}")
    print(f"Valores do Target Nomeado: {named_target_values}")
    print(f"Remember: {remembered_joint_values}")

    # Imprime informações sobre a bibliioteca de planejamento
    # e os planejadores no terminal
    print(f"Nome do Planejador: {planner_id}")
    print(f"Nome da Biblioteca do Planejador: {planning_pipeline_id}")
    print(f"Planejadores:\n{planners}")
  
def print_results_terminal(motion_planning: MoveGroupPythonInterface):
    algoritmo = motion_planning.move_group.get_planner_id()
    planning_time = motion_planning.planning_time
    execution_time = motion_planning.execution_time

    print(f"\033[1;36mTempo de planejamento para o algoritmo {algoritmo}: {planning_time:.3f} segundos\033[m")
    print(f"\033[1;33mTempo de execução para o algoritmo {algoritmo}: {execution_time:.3f} segundos\033[m")  
    
def print_results_file(motion_planning: MoveGroupPythonInterface):
    algoritmo = motion_planning.move_group.get_planner_id()
    planning_time = motion_planning.planning_time
    execution_time = motion_planning.execution_time
    position_name = motion_planning.position_name
    
    caminho_arquivo = "resultados.yaml"

    try:
        if os.path.exists(caminho_arquivo):
            with open(caminho_arquivo, "a") as arquivo:
                print(f"\nAlgoritmo: {algoritmo} \n\tPosição: {position_name} \n\tPlanejamento: {planning_time:.3f} \n\tExecução: {execution_time:.3f}\n", file=arquivo)
        else:
            with open(caminho_arquivo, "w") as arquivo:
                print(f"Algoritmo: {algoritmo} \n\tPosição: {position_name} \n\tPlanejamento: {planning_time:.3f} \n\tExecução: {execution_time:.3f}\n", file=arquivo)
    except FileNotFoundError:
        print("Erro: O arquivo não foi encontrado.")
    except PermissionError:
        print("Erro: Permissão negada para acessar o arquivo.")

def print_results_csv(motion_planning: MoveGroupPythonInterface):
    algoritmo = motion_planning.move_group.get_planner_id()
    planning_time = motion_planning.planning_time
    execution_time = motion_planning.execution_time
    position_name = motion_planning.position_name

    caminho_arquivo = 'resultados.csv'
    campos_head = ['Algoritmo', 'Posicao', 'Tempo_Planejamento', 'Tempo_Execucao']

    try:
        if os.path.exists(caminho_arquivo):
            with open(caminho_arquivo, "a") as arquivo_csv:
                writer = csv.DictWriter(arquivo_csv,fieldnames=campos_head)
                writer.writerow({'Algoritmo': f'{algoritmo}',
                                 'Posicao': f'{position_name}', 
                                 'Tempo_Planejamento': f'{planning_time:.3f}', 
                                 'Tempo_Execucao': f'{execution_time:.3f}'})
        else:
            with open(caminho_arquivo, "w") as arquivo_csv:
                writer = csv.DictWriter(arquivo_csv,fieldnames=campos_head)
                writer.writeheader()
                writer.writerow({'Algoritmo': f'{algoritmo}',
                                 'Posicao': f'{position_name}', 
                                 'Tempo_Planejamento': f'{planning_time:.3f}', 
                                 'Tempo_Execucao': f'{execution_time:.3f}'})
                           
    except FileNotFoundError:
        print("Erro: O arquivo não foi encontrado.")
    except PermissionError:
        print("Erro: Permissão negada para acessar o arquivo.")
