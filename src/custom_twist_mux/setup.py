from setuptools import setup
from glob import glob
import platform

python_version = platform.python_version_tuple()
python_version = '.'.join([python_version[0], python_version[1]])

package_name = 'custom_twist_mux'

setup(
    name=package_name,
    version='0.0.0',
    packages=[package_name],
    data_files=[
        ('share/ament_index/resource_index/packages',
            ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
        ("share/config", glob('/config/*.*')),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='yx',
    maintainer_email='yangx21@mails.tsinghua.edu.cn',
    description='TODO: Package description',
    license='TODO: License declaration',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            'node_joystick_relay = custom_twist_mux.custom_twist_mux:main'
        ],
    },
)
