# medir.launch.py — rode em outro terminal enquanto o vision.launch.py da branch está no ar.
#
#   ros2 launch vision_bench medir.launch.py                  # até Ctrl+C
#   ros2 launch vision_bench medir.launch.py duracao:=60      # para sozinho após 60 s
#   ros2 launch vision_bench medir.launch.py label:=vital_gpu # nome no resultado (padrão: branch do finder)

import os

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, EmitEvent, RegisterEventHandler
from launch.event_handlers import OnProcessExit
from launch.events import Shutdown
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node
from launch_ros.parameter_descriptions import ParameterValue


def generate_launch_description():
    arg = LaunchConfiguration
    medidor = Node(
        package='vision_bench',
        executable='medidor',
        name='vision_bench',
        output='screen',
        emulate_tty=True,
        parameters=[{
            'duracao': ParameterValue(arg('duracao'), value_type=float),
            'ignorar_s': ParameterValue(arg('ignorar_s'), value_type=float),
            'label': ParameterValue(arg('label'), value_type=str),
            'processo': ParameterValue(arg('processo'), value_type=str),
            'image_topic': ParameterValue(arg('image_topic'), value_type=str),
            'saida': ParameterValue(arg('saida'), value_type=str),
        }],
    )
    return LaunchDescription([
        DeclareLaunchArgument('duracao', default_value='0.0', description='segundos; 0 = até Ctrl+C'),
        DeclareLaunchArgument('ignorar_s', default_value='5.0',
                              description='segundos iniciais (após a 1ª detecção) fora do resumo: aquecimento'),
        DeclareLaunchArgument('label', default_value='',
                              description='nome no resultado; vazio = branch git de onde o finder foi instalado'),
        DeclareLaunchArgument('processo', default_value='object_finder/finder',
                              description='trecho da linha de comando que identifica o nó de visão'),
        DeclareLaunchArgument('image_topic', default_value='/camera/image_raw'),
        DeclareLaunchArgument('saida', default_value=os.path.expanduser('~/visao-bench/resultados')),
        medidor,
        RegisterEventHandler(OnProcessExit(target_action=medidor, on_exit=[EmitEvent(event=Shutdown())])),
    ])
