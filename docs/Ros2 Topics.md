# 📚 ROS2 Topics

> Nodes in ros2

---

## 1) Resumen

- **Activity name:** _ROS2 Topics_  
- **Team / Author(s):** Leonardo Zamora Hernández  
- **Course / Subject:** Applied Robotics  
- **Date:** _29/01/2026_  
- **Brief Description:** _Building and connecting nodes with ros2; publisher and subscriber._

---

## 2) Publisher

- **General example of a publisher**
```bash
#!/usr/bin/env python3

import rclpy
from rclpy.node import Node

def main(args=None):
    rclpy.init(args=args) # Initialize rclpy
    node = Node('R3D3') #object oriented programming
    node.get_logger().info('R3D3 is ALIVE!')
    rclpy.shutdown()      # Shutdown rclpy

if __name__ == "__main__":
    main()

```
---

## 3) Subscriber

- **General example of a subscriber**


```bash
#!/usr/bin/env python3

import rclpy
from rclpy.node import Node
from example_interfaces.msg import String

class myNode_function(Node):
    def __init__(self):
        super().__init__('C4PO')
        
        # Subscriber
        self.subscriber_ = self.create_subscription(
            String, 
            'Robot_speakingg', 
            self.listener_callback, 
            10)
        
        self.get_logger().info('C4PO Subscriber started')

    def listener_callback(self, msg: String):
        # Prints what it hears
        self.get_logger().info(f'I heard: "{msg.data}"')

def main(args=None):
    rclpy.init(args=args)
    C4PO_node = myNode_function()
    rclpy.spin(C4PO_node)
    rclpy.shutdown()

if __name__ == "__main__":
    main()
```
---
## 4) Communication

- **Publisher**

```bash
#!/usr/bin/env python3

import rclpy
from rclpy.node import Node
from example_interfaces.msg import String


class myNode_function(Node):
    #AT THIS POINT WE CREATED A PUBLISHER

    def __init__(self):
        super().__init__('R2D2')
        # self.get_logger().info('R2D2 is ALIVE!')
        # self.create_timer(1.0,self.print_callback)
        # self.publishers_ = self.create_publisher(String, 'Robot_speaking', 10)
        self.counter = 0
        self.get_logger().info('R2D2 is ALIVE!')
        self.create_timer(1.0,self.print_callback)
        self.publishers_ = self.create_publisher(String, 'Robot_speakingg', 10)
    
    def print_callback(self):
        self.get_logger().info('R2D2 says hello')
        msg = String()
        # msg.data = 'R2D2 says hello! Count: %d' % self.counter
        msg.data = '%d' % self.counter
        self.counter += 1
        self.publishers_.publish(msg)
        

def main(args=None):
    rclpy.init(args=args) # Initialize rclpy
    R2D2_node = myNode_function()
    rclpy.spin(R2D2_node)
    rclpy.shutdown()      # Shutdown rclpy

if __name__ == "__main__":
    main()
```
---
- **Create 2 nodes from scratch. In the first one you’ll have 1 publisher, and in the second one, 1 publisher & 1 subscriber.:**
  - The number_publisher node publishes a number (always the same) on the “/number” topic, with the existing type example_interfaces/msg/Int64._
  - The number_counter node subscribes to the “/number” topic. It keeps a counter variable. Every time a new number is received, it’s added to the counter. The node also has a publisher on the “/number_count” topic. When the counter is updated, the publisher directly publishes the new value on the topic. 



- **Subscriber/publisher**

```bash
#!/usr/bin/env python3

import rclpy
from rclpy.node import Node
from example_interfaces.msg import String


class myNode_function(Node):
    #AT THIS POINT WE CREATED A PUBLISHER
    
    def __init__(self):
        super().__init__('C4PO')
        self.counter = 0
        self.get_logger().info('C4PO is ALIVE!')
        self.create_timer(1.0,self.print_callback)
        self.publishers_ = self.create_publisher(String, 'Robot_speakinggg', 10)
        
        self.subscriber_= self.create_subscription(String, 'Robot_speakingg', self.listener_callback, 10)

    def listener_callback(self, msg: String):
        self.get_logger().info(f'I heard: "{msg.data}"')
        self.counter = int(msg.data)

    def print_callback(self):
        self.get_logger().info('C4PO says hi')
        msg = String()
        msg.data = 'C4PO says hello! Counting: %d' % self.counter
        # self.counter += 1
        self.publishers_.publish(msg)
        
def main(args=None):
    rclpy.init(args=args) # Initialize rclpy
    C4PO_node = myNode_function()
    rclpy.spin(C4PO_node)
    rclpy.shutdown()      # Shutdown rclpy

if __name__ == "__main__":
    main()
```
_Verification with "rqt_graph":_

![3D Printer](recursos/imgs/rqt_graph_ps.jpg)