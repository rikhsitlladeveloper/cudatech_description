#!/usr/bin/env python3.9

import rclpy
from rclpy.node import Node
import serial
import time
from geometry_msgs.msg import Twist

class CarController(Node):
    FUNC_PWM_VACUUM_START=0x01
    FUNC_PWM_VACUUM_STOP=0x02
    FUNC_PWM_VACUUM_SET=0x03
    FUNC_PWM_BRUSH_START=0x04
    FUNC_PWM_BRUSH_STOP=0x05
    FUNC_PWM_BRUSH_DIR=0x06
    FUNC_PWN_BRUSH_SET=0x07
    FUNC_SERVO_MOTOR_MODE=0x08
    FUNC_SERVO_MOTOR_SET_ACCELL=0x09
    FUNC_SERVO_MOTOR_SET_DECELL=0x0A
    FUNC_SERVO_MOTOR_VELOCITY_OF_POS_MODE=0x0B
    FUNC_SERVO_MOTOR_RELATIVE_TARGET_POS=0x0C
    FUNC_SERVO_MOTOR_ENABLE_DISABLE=0x0D
    FUNC_IMU_READ_GYRO=0x0E
    FUNC_IMU_READ_ACCEL=0x0F
    FUNC_IMU_READ_MAGENTO=0x10
    FUNC_IMU_READ_ALL_IN_ONE=0x11
    FUNC_SERVO_MOTOR_CHANGE_DIRECTION=0x12
    FUNC_PWM_MAIN_BRUSH_START=0x13
    FUNC_PWM_MAIN_BRUSH_STOP=0x14
    FUNC_PWN_MAIN_BRUSH_SET=0x15
    FUNC_HEAD=0xFF
    FUNC_DEVICE_ID=0xFC


    def __init__(self):
        super().__init__('car_controller')
        self.declare_parameter('serial_port', '/dev/ttyUSB0')
        self.declare_parameter('baudrate', 115200)

        port = self.get_parameter('serial_port').get_parameter_value().string_value
        baudrate = self.get_parameter('baudrate').get_parameter_value().integer_value

        self.serial_port = serial.Serial(port, baudrate)
        self.vacuum = False
        self.side_brush = False
        self.bottom_brush = False

        self.subscription = self.create_subscription(
            Twist,
            '/cmd_vel',
            self.handle_velocity_command,
            10
        )
        self.subscription
        
    def handle_velocity_command(self, msg):
        linear_x = msg.linear.x
        angular_z = msg.angular.z

        if linear_x > 0:
            self.move_forward(int(linear_x * 255))  # Scale factor might need adjustment
        elif linear_x < 0:
            self.move_backward(int(-linear_x * 100))  # Scale factor might need adjustment
        elif angular_z > 0:
            self.move_right()
        elif angular_z < 0:
            self.move_left()
        else:
            self.stop()

    def make_packet(self, function_id, data = None):
        try:
            packet = [255, 252, 4, function_id]

            if data is not None:
                for i in data:
                    hex_part = i.to_bytes(1, byteorder='big')
                    packet.append(hex_part[0])
                    packet[2] += 1

            return bytearray(packet)
        except Exception as e:
            print(e)
            return None

    def send_packet(self, packet):
        if packet is None:
            print("Invalid packet")
            return
        
        print("Sending packet: ", packet)
        self.serial_port.write(packet)
        time.sleep(0.001)

    def send_motor_move(self, motor_id, accel, decell, mode, enable, pos, direction):
        self.send_packet(self.make_packet(self.FUNC_SERVO_MOTOR_CHANGE_DIRECTION, [motor_id, direction]))
        self.send_packet(self.make_packet(self.FUNC_SERVO_MOTOR_SET_ACCELL, [motor_id, accel]))
        self.send_packet(self.make_packet(self.FUNC_SERVO_MOTOR_SET_DECELL, [motor_id, decell]))
        self.send_packet(self.make_packet(self.FUNC_SERVO_MOTOR_MODE, [motor_id, mode]))
        self.send_packet(self.make_packet(self.FUNC_SERVO_MOTOR_ENABLE_DISABLE, [motor_id, enable]))
        self.send_packet(self.make_packet(self.FUNC_SERVO_MOTOR_RELATIVE_TARGET_POS, [motor_id, pos]))

    def move_right(self):
        self.send_motor_move(
            motor_id=1,
            accel=1,
            decell=1,
            mode=1,
            enable=15,
            pos=255,
            direction=2,
        )
        self.send_motor_move(
            motor_id=2,
            accel=1,
            decell=1,
            mode=1,
            enable=15,
            pos=255,
            direction=2,
        )

    def move_left(self):
        print("Moving left")
        self.send_motor_move(
            motor_id=1,
            accel=1,
            decell=1,
            mode=1,
            enable=15,
            pos=255,
            direction=1,
        )
        self.send_motor_move(
            motor_id=2,
            accel=1,
            decell=1,
            mode=1,
            enable=15,
            pos=255,
            direction=1,
        )

    def move_forward(self, distance = 255):
        print("Moving forward")
        #self.send_packet(self.make_packet(self.FUNC_SERVO_MOTOR_ENABLE_DISABLE, [1, 15]))
        self.send_motor_move(
            motor_id=1,
            accel=1,
            decell=1,
            mode=1,
            enable=15,
            pos=distance,
            direction=2,
        )
        self.send_motor_move(
            motor_id=2,
            accel=1,
            decell=1,
            mode=1,
            enable=15,
            pos=distance,
            direction=1,
        )

    def move_backward(self, distance = 100):
        print("Moving backward")
        self.send_motor_move(
            motor_id=1,
            accel=1,
            decell=1,
            mode=1,
            enable=15,
            pos=distance,
            direction=1,
        )
        self.send_motor_move(
            motor_id=2,
            accel=1,
            decell=1,
            mode=1,
            enable=15,
            pos=distance,
            direction=2,
        )

    def stop(self):
        print("Stopping")
        self.send_packet(self.make_packet(self.FUNC_SERVO_MOTOR_ENABLE_DISABLE, [1, 6]))
        self.send_packet(self.make_packet(self.FUNC_SERVO_MOTOR_ENABLE_DISABLE, [2, 6]))

    def set_vacuum(self, enable = False):
        self.vacuum = enable
        self.send_packet(self.make_packet(self.FUNC_PWM_VACUUM_SET, [100]))

        if self.vacuum:
            self.send_packet(self.make_packet(self.FUNC_PWM_VACUUM_START, []))
        else:
            self.send_packet(self.make_packet(self.FUNC_PWM_VACUUM_STOP, []))

    def set_side_brush(self, enable = False):
        self.side_brush = enable
        
        if self.side_brush:
            self.send_packet(self.make_packet(self.FUNC_PWN_BRUSH_SET, [90]))
            self.send_packet(self.make_packet(self.FUNC_PWM_BRUSH_START, []))
        else:
            self.send_packet(self.make_packet(self.FUNC_PWM_BRUSH_STOP, []))

    def set_bottom_brush(self, enable = False):
        self.bottom_brush = enable
        
        if self.bottom_brush:
            self.send_packet(self.make_packet(self.FUNC_PWN_MAIN_BRUSH_SET, [70]))
            self.send_packet(self.make_packet(self.FUNC_PWM_MAIN_BRUSH_START, []))
        else:
            self.send_packet(self.make_packet(self.FUNC_PWM_MAIN_BRUSH_STOP, []))

def main(args=None):
    rclpy.init(args=args)
    car_controller = CarController()
    rclpy.spin(car_controller)
    car_controller.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
