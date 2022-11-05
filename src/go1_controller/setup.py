from setuptools import setup
import os
from glob import glob
import platform

package_name = 'go1_controller'

python_version = platform.python_version_tuple()
python_version = '.'.join([python_version[0], python_version[1]])


def package_files(data_files, directory_list):
    paths_dict = {}
    for directory in directory_list:
        for (path, directories, filenames) in os.walk(directory):
            for filename in filenames:
                file_path = os.path.join(path, filename)
                install_path = os.path.join('share', package_name, path)

                if install_path in paths_dict.keys():
                    paths_dict[install_path].append(file_path)
                else:
                    paths_dict[install_path] = [file_path]

    for key in paths_dict.keys():
        data_files.append((key, paths_dict[key]))
    return data_files

data_files=[
        ('share/ament_index/resource_index/packages',
           ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
        ('lib/python{0}/site-packages/'.format(python_version)+package_name+"/InverseKinematics", glob(package_name+'/InverseKinematics/*.*')),
        ('lib/python{0}/site-packages/'.format(python_version)+package_name+"/RobotController", glob(package_name+'/RobotController/*.*')),
        ('lib/python{0}/site-packages/'.format(python_version)+package_name+"/RoboticsUtilities", glob(package_name+'/RoboticsUtilities/*.*')),
    ]

setup(
    name=package_name,
    version='0.0.0',
    packages=[package_name],
    data_files=package_files(data_files, ['launch/']),
    
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='root',
    maintainer_email='root@todo.todo',
    description='TODO: Package description',
    license='TODO: License declaration',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            'node_controller = go1_controller.go1_controller:main'
        ],
    },
)