#!/bin/bash
#Script de lançamento necessário para spawnar o mundo e o robõ corretamente sem interferência de arquivos default do turtlebot3
clear
echo "--- Configurando ambiente limpo para o Gazebo ---"
source /opt/ros/jazzy/setup.bash
source /home/turtlebot3_ws/install/setup.bash

UFBA_PKG_SHARE=$(ros2 pkg prefix turtlebot3_ufba)/share/turtlebot3_ufba
TB3_GAZEBO_PKG_SHARE=$(ros2 pkg prefix turtlebot3_gazebo)/share/turtlebot3_gazebo

export GZ_SIM_SYSTEM_PLUGIN_PATH=""
export GZ_SIM_RESOURCE_PATH="$UFBA_PKG_SHARE/worlds:$TB3_GAZEBO_PKG_SHARE/models"

echo "--- Ambiente limpo configurado. Iniciando o ROS2 Launch... ---"
ros2 launch turtlebot3_ufba world.launch.py "$@"

