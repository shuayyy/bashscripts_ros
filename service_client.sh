#!/bin/bash

# Check if required commands are available
command -v colcon >/dev/null 2>&1 || { echo >&2 "colcon is required but it's not installed. Aborting."; exit 1; }
command -v ros2 >/dev/null 2>&1 || { echo >&2 "ros2 command is required but it's not installed. Aborting."; exit 1; }

# Get user input
read -p "Enter workspace name: " WORKSPACE_NAME
read -p "Enter package name: " PACKAGE_NAME

# Create and navigate to the workspace
mkdir -p $WORKSPACE_NAME/src
cd $WORKSPACE_NAME || { echo "Failed to navigate to workspace directory"; exit 1; }

# Initialize ROS 2 workspace
echo "Building workspace..."
colcon build || { echo "Failed to build workspace"; exit 1; }

# Source the workspace setup file if it exists
if [ -f install/setup.bash ]; then
    source install/setup.bash
else
    echo "Setup file not found. Ensure the workspace is built correctly."
fi

# Navigate to the src directory
cd src || { echo "Failed to navigate to src directory"; exit 1; }

# Create the ROS 2 package inside src
ros2 pkg create --build-type ament_python $PACKAGE_NAME || { echo "Failed to create package"; exit 1; }

# Navigate to the package directory
cd $PACKAGE_NAME || { echo "Failed to navigate to package directory"; exit 1; }

# Create a publisher node script
mkdir -p $PACKAGE_NAME
cat <<EOL > $PACKAGE_NAME/client.py
# client_member_function.py

import sys
import rclpy
from rclpy.node import Node
from example_interfaces.srv import AddTwoInts

class AddTwoIntsClient(Node):

    def __init__(self):
        super().__init__('add_two_ints_client')
        self.cli = self.create_client(AddTwoInts, 'add_two_ints')

        while not self.cli.wait_for_service(timeout_sec=1.0):
            self.get_logger().info('Service not available, waiting...')

        self.req = AddTwoInts.Request()

    def send_request(self):
        self.req.a = int(sys.argv[1])
        self.req.b = int(sys.argv[2])
        self.future = self.cli.call_async(self.req)

def main(args=None):
    rclpy.init(args=args)
    node = AddTwoIntsClient()
    node.send_request()

    while rclpy.ok():
        rclpy.spin_once(node)
        if node.future.done():
            try:
                response = node.future.result()
            except Exception as e:
                node.get_logger().info(f'Service call failed: {e}')
            else:
                node.get_logger().info(f'Result: {node.req.a} + {node.req.b} = {response.sum}')
            break

    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()

EOL

# Create a subscriber node script
cat <<EOL > $PACKAGE_NAME/server.py
# service_member_function.py

import rclpy
from rclpy.node import Node
from example_interfaces.srv import AddTwoInts

class AddTwoIntsService(Node):

    def __init__(self):
        super().__init__('add_two_ints_server')
        self.srv = self.create_service(AddTwoInts, 'add_two_ints', self.add_two_ints_callback)

    def add_two_ints_callback(self, request, response):
        response.sum = request.a + request.b
        self.get_logger().info(f'Request: {request.a} + {request.b} = {response.sum}')
        return response

def main(args=None):
    rclpy.init(args=args)
    node = AddTwoIntsService()
    rclpy.spin(node)

    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()

EOL

# Set up the package to include the scripts
cat <<EOL > setup.py
from setuptools import find_packages, setup

package_name = 'my_servicepkg'

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
    maintainer='gopi',
    maintainer_email='gopi@todo.todo',
    description='TODO: Package description',
    license='TODO: License declaration',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            'client = client.client:main',
            'server= client.service:main',
        ],
    },
)

EOL

# Go back to the workspace root and build the package
cd ../..
colcon build || { echo "Failed to build workspace"; exit 1; }

# Source the workspace setup file again after build
if [ -f install/setup.bash ]; then
    source install/setup.bash
else
    echo "Setup file not found. Ensure the workspace is built correctly."
fi

echo "ROS 2 Client is served with service"

