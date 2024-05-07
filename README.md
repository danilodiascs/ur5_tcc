# ur5_tcc

Repositório do Trabalho de Conclusão de Curso (TCC) de Danilo Dias Conceição da Silva
para o curso de Engenharia da Computação da Universidade Federal da Bahia (UFBA)

==> Informações importantes

-- Sistema Operacional: Ubuntu 20.04
-- Versão do ROS: noetic  
-- Modelo do robô utilizado: UR5 

==> Instruções de uso

-- Executar o UR5 no Gazebo, carregando os planejadores da biblioteca OMPL:

roslaunch ur5_tcc ur5_tcc.launch

-- Executar o programa que realiza o planejamento e a execução de movimentos do UR5:

rosrun ur5_tcc main.py 
