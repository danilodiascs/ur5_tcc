#!/usr/bin/env python3

# ========================================================== #
#                          main.py                           # 
# ========================================================== #
# Projeto desenvolvido por Danilo Dias Conceição da Silva    #
# como parte do Trabalho de Conclusão de Curso (TCC) do      #
# curso de Engenharia da Computação da Universidade Federal  #
# da Bahia (UFBA)                                            #
# ========================================================== #

from aux_functions import *

def main():
    try:
        # Inicializa o nó do ROS que será utilizado em toda a aplicação
        rospy.init_node('move_group_python_interface', anonymous=False)

        # Inscreve o nó em execução no tópico 'joint_states'
        # visando capturar os dados de Posição, Velocidade e Aceleração
        # dos joints do robô e gravá-los num arquivo .bag 
        rospy.Subscriber("/joint_states", JointState, callbackSubscriber)

        # Chama a função que realiza todo o processo de Planejamento
        # e Execução de movimentos do robô
        plan_and_execute()

        # rospy.on_shutdown(print_menu)

        # Envia um sinal de encerramento para o nó do ROS que está
        # em execução
        rospy.signal_shutdown("Fim da execução do nó " + rospy.get_name())

    except rospy.ROSInterruptException:
        return
    except KeyboardInterrupt:
        return

if __name__ == "__main__":
    main()
