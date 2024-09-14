# Estudo comparativo entre estratégias de planejamento de trajetórias para manipuladores robóticos

[![Manutenção?](https://img.shields.io/badge/Manutenção%3f-Sim-darkgreen.svg)](https://shields.io/)
[![Mantenedor](https://img.shields.io/badge/Mantenedor-Danilo_Dias-darkblue.svg)](https://shields.io/)
<!-- ![Visual Studio Code](https://img.shields.io/badge/Visual%20Studio%20Code-0078d7.svg?style=for-the-badge&logo=visual-studio-code&logoColor=white) -->
![ROS](https://img.shields.io/badge/ros-%230A0FF9.svg?style=for-the-badge&logo=ros&logoColor=white)
![Python](https://img.shields.io/badge/python-3670A0?style=for-the-badge&logo=python&logoColor=ffdd54)
![C++](https://img.shields.io/badge/c++-%2300599C.svg?style=for-the-badge&logo=c%2B%2B&logoColor=white)
![CMake](https://img.shields.io/badge/CMake-%23008FBA.svg?style=for-the-badge&logo=cmake&logoColor=white)


## Descrição

>Repositório do Trabalho de Conclusão de Curso (TCC) de Danilo Dias Conceição da Silva para o curso de Engenharia da Computação da Universidade Federal da Bahia (UFBA)

## Instruções de instalação

### Pré-requisitos

```
Sistema Operacional: Ubuntu 20.04
Versão do ROS: Noetic Ninjemys  
Versão do MoveIt: 1.1.13
Modelo do robô utilizado: UR5 
```

### Etapas

1. No terminal, execute o seguinte comando:

```bash
git clone https://github.com/danilodiascs/ur5_tcc.git
```

2. Após isto, entre na pasta do projeto:
```bash
cd ur5_tcc
```

3. Compile o projeto:
```bash
catkin_make
```

4. Após executar as etapas anteriores, basta seguir as instruções de uso.

## Instruções de uso

### Comandos necessários 

Executar o ***UR5*** no Gazebo, carregando os planejadores da biblioteca **OMPL**:

```bash
roslaunch ur5_tcc ur5_tcc.launch 
```

<!-- **OBS**: -->
>[!NOTE]
>O Gazebo inicia em modo *paused*. É necessário acionar o *play* para carregar os planejadores.


Iniciar o programa que realiza o planejamento e a execução de trajetórias do ***UR5***:

```bash
rosrun ur5_tcc main.py 
```

<!-- ## Ilustrações -->

<!-- ![Logo do ROS](https://d2908q01vomqb2.cloudfront.net/ca3512f4dfa95a03169c5a670a4c91a19b3077b4/2018/11/26/ros-logo-300x168.jpg) -->