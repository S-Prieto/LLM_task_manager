from setuptools import setup, find_packages
import os
from glob import glob

package_name = 'llm_task_manager'

setup(
    name=package_name,
    version='0.0.0',
    packages=find_packages(exclude=['test']),
    data_files=[
        ('share/ament_index/resource_index/packages',
            ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
        (os.path.join('share', package_name), glob('launch/*launch.[pxy][yma]*')),
        # Ensure the .env and thread_info.txt files are included from the correct directory
        (os.path.join('lib', package_name), [
            os.path.join('llm_task_manager', '.env'),
            os.path.join('llm_task_manager', 'thread_info.txt')
        ]),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='your_name',
    maintainer_email='your_email@example.com',
    description='TODO: Package description',
    license='TODO: License declaration',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            'command_parser = llm_task_manager.command_parser:main',
            'openai_bridge = llm_task_manager.openai_bridge:main',
        ],
    },
)

