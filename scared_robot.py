import rclpy
import time
from rclpy.node import Node
from std_msgs.msg import String
from pynput import keyboard

class CarParkRobot(Node):

    def listener_callback(self, msg):
        self.front_sensor_distance = int(msg.data[:-2])  
        self.car_park()

    def listener_callback_left(self, msg):
        self.left_sensor_distance = int(msg.data[:-2])
        self.car_park()

    def listener_callback_right(self, msg):
        self.right_sensor_distance = int(msg.data[:-2]) 
        self.car_park() 

    def car_park(self):
        if self.in_parking_lot:
            self.car_stop()
        elif self.front_sensor_distance > 70:
            if self.right_sensor_distance > 70:
                self.car_right_turn()
                self.car_move_forward_after_turn()
                self.car_stop()
            elif self.left_sensor_distance > 100:
                self.car_left_turn()
                self.car_move_forward_after_turn()
                self.car_stop()
            else:
                self.car_move_forward()
                    
    def car_left_turn(self):
        msg = String()
        msg.data = "TURNL:0800"
        self.publisher.publish(msg)
        print('Moving Left  "%s"' ,msg.data)
        self.in_parking_lot = True
        time.sleep(1)  

    def car_right_turn(self):
        msg = String()
        msg.data = "TURNR:0800"
        self.publisher.publish(msg)
        print('Moving Right  "%s"' ,msg.data)
        self.in_parking_lot = True
        time.sleep(1) 

    def car_move_forward(self):
        msg = String()
        msg.data = "MOVEF:0100"
        print('Moving Forward  "%s"' ,msg.data)
        self.publisher.publish(msg)

    def car_move_forward_after_turn(self):
        msg = String()
        msg.data = "MOVEF:0100"
        print('Moving Forward  "%s"' ,msg.data)
        self.publisher.publish(msg)

    def car_stop(self):
        msg = String()
        msg.data = "STOP"
        print('STOPPED  "%s"' ,msg.data)
        self.publisher.publish(msg)

    def __init__(self):
        super().__init__( CarParkRobot)
        self.publisher = self.create_publisher(String, '/robot/control', 10)
        self.subscription_front = self.create_subscription(String,'/robot/front',self.listener_callback,10)
        self.subscription_left = self.create_subscription(String,'/robot/left',self.listener_callback_left,10)
        self.subscription_right = self.create_subscription(String,'/robot/right',self.listener_callback_right,10)
        self.subscription_front 
        self.subscription_left  
        self.subscription_right  
        self.left_sensor_distance = 0
        self.right_sensor_distance = 0
        self.front_sensor_distance = 0
        self.in_parking_lot = False

def main(args=None):
    rclpy.init(args=args)
    carparkrobot = CarParkRobot ()
    rclpy.spin(carparkrobot)
    carparkrobot.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
