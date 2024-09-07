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
cat <<EOL > $PACKAGE_NAME/publisher.py
import rclpy
from rclpy.node import Node
from std_msgs.msg import String

class PublisherNode(Node):
    def __init__(self):
        super().__init__('publisher_node')
        self.publisher = self.create_publisher(String, 'topic', 10)
        self.timer = self.create_timer(1.0, self.publish_message)
        self.message_count = 0

    def publish_message(self):
        msg = String()
        msg.data = 'Message number %d' % self.message_count
        self.publisher.publish(msg)
        self.get_logger().info('Publishing: "%s"' % msg.data)
        self.message_count += 1

def main(args=None):
    rclpy.init(args=args)
    node = PublisherNode()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
EOL

# Create a subscriber node script
cat <<EOL > $PACKAGE_NAME/subscriber.py
import rclpy
from rclpy.node import Node
from std_msgs.msg import String

class SubscriberNode(Node):
    def __init__(self):
        super().__init__('subscriber_node')
        self.subscription = self.create_subscription(
            String,
            'topic',
            self.listener_callback,
            10)
        self.subscription

    def listener_callback(self, msg):
        self.get_logger().info('Received: "%s"' % msg.data)

def main(args=None):
    rclpy.init(args=args)
    node = SubscriberNode()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
EOL

# Set up the package to include the scripts
cat <<EOL > setup.py
from setuptools import setup

package_name = '$PACKAGE_NAME'

setup(
    name=package_name,
    version='0.0.0',
    packages=[package_name],
    install_requires=['setuptools'],
    zip_safe=True,
    entry_points={
        'console_scripts': [
            'publisher = $PACKAGE_NAME.publisher:main',
            'subscriber = $PACKAGE_NAME.subscriber:main',
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

echo "ROS 2 package '$PACKAGE_NAME' created with publisher and subscriber nodes."

