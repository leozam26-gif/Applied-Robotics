# 📚 ROS2 Services

> Client / Server in ROS2

---

## 1) Resumen

* **Activity name:** *ROS2 Services*
* **Team / Author(s):** Leonardo Zamora Hernández
* **Course / Subject:** Applied Robotics
* **Date:** *16/02/2026*
* **Brief Description:** *Implementation of a Service Server within a node to reset a counter variable using the `example_interfaces/srv/SetBool` interface.*

---

## 2) Publisher Node

* **Number Publisher Code**
This node publishes a constant integer to the `/number` topic.

```python
#!/usr/bin/env python3

import rclpy
from rclpy.node import Node
from example_interfaces.msg import Int64 

class myNode_function(Node):

    def __init__(self):
        super().__init__('number_publisher')

        self.counter = 0
        self.get_logger().info('NP is ALIVE!')
        self.create_timer(1.0,self.print_callback)
        self.publishers_ = self.create_publisher(Int64, 'number', 10)
    
    def print_callback(self):
        self.get_logger().info('NP says hello')
        msg = Int64()
        # msg.data = 'R2D2 says hello! Count: %d' % self.counter
        msg.data = self.counter
        self.counter = 1
        self.publishers_.publish(msg)
        

def main(args=None):
    rclpy.init(args=args) # Initialize rclpy
    number_publisher = myNode_function()
    rclpy.spin(number_publisher)
    rclpy.shutdown()      # Shutdown rclpy

if __name__ == "__main__":
    main()

```

* **Terminal Command:**

```bash
ros2 run my_rbt_pkg number_publisher

```

* **Execution Evidence:**

---

## 3) Subscriber & Service Server

* **Number Counter Code**
This node subscribes to `/number` to increment a counter. It also hosts a service server named `/reset_counter` that resets the count to 0 when called.

```python
#!/usr/bin/env python3

import rclpy
from rclpy.node import Node
from example_interfaces.msg import Int64
from example_interfaces.srv import SetBool

class myNode_function(Node):
    
    def __init__(self):
        super().__init__('number_counter')
        self.counter = 0
        self.get_logger().info('NC is ALIVE!')
        self.create_timer(1.0,self.print_callback)
        self.publishers_ = self.create_publisher(Int64, 'number_count', 10)
        
        self.subscriber_= self.create_subscription(Int64, 'number', self.listener_callback, 10)
        
        # Service Server Definition
        self.server_ = self.create_service(SetBool, # SERVICE TYPE
                                           "reset_counter", # SERVICE NAME
                                           self.reset_counter_callback) # CALLBACK FUNCTION
        self.get_logger().info("Service reset_counter has been started.")

    def reset_counter_callback(self, request, response):
        if request.data == True:
            self.counter = 0
            response.success = True
            response.message = "Counter has been reset to zero."
        else:
            response.success = False
            response.message = "Counter was not reset."
        
        return response

    def listener_callback(self, msg: Int64):
        self.get_logger().info(f'I heard: "{msg.data}"')
        #self.counter = msg.data
        self.counter += msg.data

    def print_callback(self):
        self.get_logger().info('NC says hi')
        msg = Int64()
        msg.data =  self.counter
        self.publishers_.publish(msg)
        
def main(args=None):
    rclpy.init(args=args) # Initialize rclpy
    number_counter = myNode_function()
    rclpy.spin(number_counter)
    rclpy.shutdown()      # Shutdown rclpy

if __name__ == "__main__":
    main()

```

* **Terminal Command:**

```bash
ros2 run my_rbt_pkg number_counter

```

* **Execution Evidence:**

---

## 4) Communication & Service Call

* **Testing the Service**
To test the functionality, we call the `/reset_counter` service from the terminal using the `SetBool` interface. We send `data: true` to trigger the reset logic defined in the server callback.


* **Terminal Command:**

```bash
ros2 service call /reset_counter example_interfaces/srv/SetBool "{data: true}"

```

* **Verification:**
*The service returns success=True and the message "Counter has been reset to zero."*