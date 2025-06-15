from setuptools import find_packages, setup
import os
from glob import glob

package_name = 'turtlebot3_ufba'

setup(
    name=package_name,
    version='1.0.0',
    packages=find_packages(exclude=['test']),
    data_files=[
        ('share/ament_index/resource_index/packages',
            ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
        # IMPORTANTE: Garante que os arquivos da pasta 'launch' sejam instalados
        (os.path.join('share', package_name, 'launch'), glob(os.path.join('launch', '*launch.[pxy][yma]*'))),
        # IMPORTANTE: Garante que os arquivos da pasta 'worlds' e 'modelos' sejam instalados
        (os.path.join('share', package_name, 'worlds'), glob(os.path.join('worlds', '*.sdf'))),
        (os.path.join('share', package_name, 'models', 'turtlebot3_burger_vermelho'), glob(os.path.join('models', 'turtlebot3_burger_vermelho', '*'))),
        (os.path.join('share', package_name, 'models', 'track1'), glob(os.path.join('models', 'track1', '*'))),
        (os.path.join('share', package_name, 'scripts'), glob(os.path.join('scripts', '*.sh'))),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='Eliezer Rocha',
    maintainer_email='eliezerrocha@gmail.com',
    description='PROJETO ENGG03 UFBA',
    license='Apache 2.0',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            'turtle_controller = turtlebot3_ufba.turtle_node:main',
        ],
    },
)
