from setuptools import find_packages, setup

package_name = 'duckie_vision'

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
    maintainer='shpark',
    maintainer_email='ekgns0411@gmail.com',
    description='TODO: Package description',
    license='TODO: License declaration',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
        # '실행명령어 = 패키지명.파일명:함수명'
            'image_compressor = duckie_vision.image_compressor:main',
            
            'simple_viewer = duckie_vision.simple_viewer:main',
            
            'object_follower = duckie_vision.object_follower:main',
        ],
    },
)
