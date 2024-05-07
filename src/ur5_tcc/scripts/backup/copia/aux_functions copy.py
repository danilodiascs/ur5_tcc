#!/usr/bin/env python3

import os, csv
from move_group_python_interface import *

def print_menu():
    print("")
    print("----------------------------------------------------------")
    print("MoveIt MoveGroup Python Interface Tutorial")
    print("----------------------------------------------------------")
    print("Press Ctrl-D to exit at any time")
    print("")

def print_information(move_group, robot):
    # motion_planning = move_group.construct_motion_plan_request()
      planners = move_group.get_interface_description()
    # planner_id = move_group.get_planner_id()
    # planning_pipeline_id = move_group.get_planning_pipeline_id()

    # print("=== Planning frame: %s" % move_group.planning_frame)
    # print("=== End effector link: %s" % move_group.eef_link)
    # print("=== Available Planning Groups:", robot.get_group_names())
    # print("=== Printing robot state")
    # print(robot.get_current_state())
    # print("")

    # print("TESTANDO:\n%s" % motion_planning)
    # print("Nome do Planejador: %s" % planner_id)
    # print("Nome da Biblioteca do Planejador: %s" % planning_pipeline_id)
      print("Planejadores:\n%s" % planners)
  
    # print("Targets Nomeados: %s" % move_group.get_named_targets())
    # print("Valores do Target Nomeado: %s" % move_group.get_named_target_values("up"))
    # print("Remember: %s"% move_group.get_remembered_joint_values())


def print_results_terminal(move_group, planning_time, execution_time, position_name):
    algoritmo = move_group.get_planner_id()

    print("\033[1;36mTempo de planejamento para o algoritmo %s: %.6f segundos\033[m" % (algoritmo, planning_time))
    print("\033[1;33mTempo de execução para o algoritmo %s: %.6f segundos\033[m" % (algoritmo, execution_time))

    # caminho_arquivo = "resultados.yaml"

    # try:
    #     if os.path.exists(caminho_arquivo):
    #         with open(caminho_arquivo, "a") as arquivo:
    #             print(f"\nAlgoritmo: {algoritmo} \n\tPosição: {position_name} \n\tPlanejamento: {planning_time} \n\tExecução: {execution_time}\n", file=arquivo)
    #     else:
    #         with open(caminho_arquivo, "w") as arquivo:
    #             print(f"Algoritmo: {algoritmo} \n\tPosição: {position_name} \n\tPlanejamento: {planning_time} \n\tExecução: {execution_time}\n", file=arquivo)
    # except FileNotFoundError:
    #     print("Erro: O arquivo não foi encontrado.")
    # except PermissionError:
    #     print("Erro: Permissão negada para acessar o arquivo.")    
    
def print_results_file(move_group, planning_time, execution_time, position_name):
    algoritmo = move_group.get_planner_id()
    
    caminho_arquivo = "resultados.yaml"

    try:
        if os.path.exists(caminho_arquivo):
            with open(caminho_arquivo, "a") as arquivo:
                print(f"\nAlgoritmo: {algoritmo} \n\tPosição: {position_name} \n\tPlanejamento: {planning_time} \n\tExecução: {execution_time}\n", file=arquivo)
        else:
            with open(caminho_arquivo, "w") as arquivo:
                print(f"Algoritmo: {algoritmo} \n\tPosição: {position_name} \n\tPlanejamento: {planning_time} \n\tExecução: {execution_time}\n", file=arquivo)
    except FileNotFoundError:
        print("Erro: O arquivo não foi encontrado.")
    except PermissionError:
        print("Erro: Permissão negada para acessar o arquivo.")


def print_results_csv(move_group, planning_time, execution_time, position_name):
    caminho_arquivo = 'resultados.csv'
    campos_head = ['Algoritmo', 'Posicao', 'Tempo_Planejamento', 'Tempo_Execucao']
    algoritmo = move_group.get_planner_id()
    # algoritmo = 'RRT'
    # position_name = 'up'
    # planning_time = 0.001
    # execution_time = 5.000

    try:
        if os.path.exists(caminho_arquivo):
            with open(caminho_arquivo, "a") as arquivo_csv:
                writer = csv.DictWriter(arquivo_csv,fieldnames=campos_head)
                writer.writerow({'Algoritmo': f'{algoritmo}',
                                 'Posicao': f'{position_name}', 
                                 'Tempo_Planejamento': f'{planning_time}', 
                                 'Tempo_Execucao': f'{execution_time}'})
        else:
            with open(caminho_arquivo, "w") as arquivo_csv:
                writer = csv.DictWriter(arquivo_csv,fieldnames=campos_head)
                writer.writeheader()
                writer.writerow({'Algoritmo': f'{algoritmo}',
                                 'Posicao': f'{position_name}', 
                                 'Tempo_Planejamento': f'{planning_time}', 
                                 'Tempo_Execucao': f'{execution_time}'})
                
                #writer.writerow({f'Algoritmo: {algoritmo}', f'Posicao: {position_name}', f'Tempo_Planejamento: {planning_time}', f'Tempo_Execucao: {execution_time}'})
                
    except FileNotFoundError:
        print("Erro: O arquivo não foi encontrado.")
    except PermissionError:
        print("Erro: Permissão negada para acessar o arquivo.")
