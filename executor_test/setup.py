from setuptools import find_packages, setup

package_name = 'executor_test'

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
    maintainer='xiandi',
    maintainer_email='shan.xiandi@techmagic.co.jp',
    description='TODO: Package description',
    license='TODO: License declaration',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            'service_server = executor_test.call_service_from_action.service_server:main',
            'service_client = executor_test.call_service_from_action.service_client:main',
            'action_server = executor_test.call_service_from_action.action_server:main',
            'action_client = executor_test.call_service_from_action.action_client:main',

            # 'service_server = executor_test.call_action_from_service.service_server:main',
            # 'service_client = executor_test.call_action_from_service.service_client:main',
            # 'action_server = executor_test.call_action_from_service.action_server:main',
            # 'action_client = executor_test.call_action_from_service.action_client:main',

            # 'first_action_server = executor_test.action_service_action.first_action_server:main',
            # 'second_action_server = executor_test.action_service_action.second_action_server:main',
            # 'first_action_client = executor_test.action_service_action.first_action_client:main',
            # 'second_action_client = executor_test.action_service_action.second_action_client:main',
            # 'service_server = executor_test.action_service_action.service_server:main',
            # 'service_client = executor_test.action_service_action.service_client:main',
        ],
    },
)
