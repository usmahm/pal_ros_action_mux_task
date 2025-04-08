from setuptools import find_packages, setup

package_name = 'generic_py'

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
    license='TODO: License declaration',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            'gen_subber_py = generic_py.generic_subscriber:main',
            'gen_tester_int_cpp = generic_py.generic_tester_int_pub:main',
            'gen_tester_string_cpp = generic_py.generic_tester_string_pub:main',
        ],
    },
)
