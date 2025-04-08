from setuptools import find_packages, setup

package_name = 'delay_action_py'

setup(
    name=package_name,
    version='0.0.0',
    packages=find_packages(exclude=['test']),
    data_files=[
        ('share/ament_index/resource_index/packages',
            ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='Ahmad Usman',
    maintainer_email='ajusman@student.oauife.edu.ng',
    description='TODO: Package description',
    license='Apache License 2.0',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            'action_server_py = delay_action_py.action_server:main',
            'action_client_py = delay_action_py.action_client:main',
        ],
    },
)
