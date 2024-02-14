# ROS2 Template Workspace

## First steps

Pull the ROS2 Humble container:
```bash
docker pull osrf/ros:humble-desktop
```

Run the container (with GUI):
```bash
xhost +local:root

docker run -it \
    --env="DISPLAY" \
    --env="QT_X11_NO_MITSHM=1" \
    --volume="/tmp/.X11-unix:/tmp/.X11-unix:rw" \
    osrf/ros:humble-desktop \
    bash
```

Source the ```ros_entrypoint.sh``` script:

```bash
./ros_entrypoint.sh
```

## Installing the template workspace

Clone this repository:
```bash
cd $HOME
git clone https://github.com/federicozappone/ros2_template_ws
cd ros2_template_ws
```

## Build the workspace

Build all the packages:
```bash
colcon build --symlink-install
```

Build a list of packages:
```bash
colcon build --symlink-install --packages-select template_package
```

Build a list of packages and their dependencies:
```bash
colcon build --symlink-install --packages-above template_package
```

## Running

Source the workspace:
```bash
source install/setup.bash
```

Run a single node:
```bash
ros2 run template_package publisher
```

Run a launch file:
```bash
ros2 launch template_package pubsub.launch.py
```

## Inspecting nodes

Check the output of a publisher:
```bash
# run this in one terminal window
ros2 run template_package publisher
```

```bash
# run this in a second terminal window
ros2 topic echo /topic
```

Check topic frequency:
```bash
# run this in a second terminal window
ros2 topic hz /topic
```

## Creating new packages

Create an empty python package:
```bash
cd ros2_template_ws/src
ros2 pkg create --build-type ament_python my_package
```

Inspect ```my_package``` structure
```bash
cd $HOME/ros2_template_ws/src/my_package
tree -L 3
```

```
.
├── my_package
│   └── __init__.py
├── package.xml
├── resource
│   └── my_package
├── setup.cfg
├── setup.py
└── test
    ├── test_copyright.py
    ├── test_flake8.py
    └── test_pep257.py

3 directories, 8 files
```

## Creating nodes

Create a new ```ros2_template_ws/src/my_package/my_package/my_node.py```python file:
```bash
touch $HOME/ros2_template_ws/src/my_package/my_package/my_node.py
```

Add the code for a simple publisher:
```my_node.py```
```python
import rclpy
from rclpy.node import Node
from std_msgs.msg import String


class MinimalPublisher(Node):

    def __init__(self):
        super().__init__('minimal_publisher')
        self.publisher_ = self.create_publisher(String, 'topic', 10)
        timer_period = 0.5  # seconds
        self.timer = self.create_timer(timer_period, self.timer_callback)
        self.i = 0

    def timer_callback(self):
        msg = String()
        msg.data = 'Hello World: %d' % self.i
        self.publisher_.publish(msg)
        self.get_logger().info('Publishing: "%s"' % msg.data)
        self.i += 1


def main(args=None):
    rclpy.init(args=args)

    minimal_publisher = MinimalPublisher()

    rclpy.spin(minimal_publisher)

    # Destroy the node explicitly
    # (optional - otherwise it will be done automatically
    # when the garbage collector destroys the node object)
    minimal_publisher.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()
```

Edit the ```setup.py``` file to include the new node:
```python
from setuptools import find_packages, setup

package_name = 'my_package'

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
    maintainer='federico',
    maintainer_email='federico.zappone@gmail.com',
    description='TODO: Package description',
    license='TODO: License declaration',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            "my_node = my_package.my_node:main"
        ],
    },
)
```

Build the workspace:
```bash
cd $HOME/ros2_template_ws
colcon build --symlink-install
```

Source the workspace:
```bash
source install/setup.bash
```

Run the new node:
```bash
ros2 run my_package my_node
```
