import rclpy
from ackerman_chassis2.srv import ID501
import time

def main():
    rclpy.init()
    node = rclpy.create_node('id501_client')
    client = node.create_client(ID501, 'ID501')
    while not client.wait_for_service(timeout_sec=1.0):
        node.get_logger().info('Waiting for service...')
    req = ID501.Request()
    req.throttle_enable = 1
    req.steer_enable = 1
    req.brake_enable = 1
    req.light_enable = 1
    req.gear = 3
    req.drive_mode = 0
    req.target_moto_rpm = 1000
    req.target_speed_kmh = 0
    req.target_steer_angle = 0
    req.target_brake_pressure = 0
    req.head_light = 0
    req.left_light = 0
    req.right_light = 0

    try:
        while rclpy.ok():
            future = client.call_async(req)
            rclpy.spin_until_future_complete(node, future, timeout_sec=0.05)
            # 可选：打印返回
            # if future.done():
            #     print(future.result().success, future.result().message)
            time.sleep(0.02)  # 20ms
    except KeyboardInterrupt:
        pass
    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
