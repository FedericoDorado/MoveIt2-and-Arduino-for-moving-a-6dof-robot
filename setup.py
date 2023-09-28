from setuptools import setup

package_name = 'centauri_6dof_to_arduino'

setup(
    name=package_name,
    version='0.0.0',
    packages=[package_name],
    py_modules=[
        'centauri_6dof_to_arduino.joint_states_to_arduino_node',
        'centauri_6dof_to_arduino.micro_json_to_arduino_node',
        'centauri_6dof_to_arduino.moveit_joint_states_to_arduino_node'
    ],
    data_files=[
        ('share/ament_index/resource_index/packages',
            ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
    ],
    install_requires=['setuptools', 'pyserial'],
    zip_safe=True,
    maintainer='uao',
    maintainer_email='uao@todo.todo',
    description='TODO: Package description',
    license='TODO: License declaration',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            'joint_states_to_arduino_node = centauri_6dof_to_arduino.joint_states_to_arduino_node:main',
            'micro_json_to_arduino_node = centauri_6dof_to_arduino.micro_json_to_arduino_node:main',
            'moveit_joint_states_to_arduino_node = centauri_6dof_to_arduino.moveit_joint_states_to_arduino_node:main',
        ],
    },
)
