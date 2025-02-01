import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Imu
from bleak import BleakClient
import asyncio

BLE_DEVICE_ADDRESS = "0A:7E:95:B3:3D:B0"  # Replace with your Arduino's MAC address
IMU_CHARACTERISTIC_UUID = "2A37"

class IMUPublisher(Node):
    def __init__(self):
        super().__init__("imu_publisher")
        self.publisher_ = self.create_publisher(Imu, "/imu/out", 10)
        self.imu_data = None
        self.loop = asyncio.get_event_loop()

    async def connect_ble(self):
        """Establish BLE connection and listen for notifications."""
        self.client = BleakClient(BLE_DEVICE_ADDRESS)
        try:
            self.get_logger().info("Connecting to BLE device...")
            await self.client.connect()
            self.get_logger().info("Connected to BLE device")
            await self.client.start_notify(IMU_CHARACTERISTIC_UUID, self.ble_callback)
            while True:
                await asyncio.sleep(1)  # Keep the BLE connection alive
        except Exception as e:
            self.get_logger().error(f"BLE connection error: {e}")
        finally:
            if self.client.is_connected:
                await self.client.disconnect()
                self.get_logger().info("Disconnected from BLE device")

    def ble_callback(self, sender, data):
        """Handle incoming BLE notifications."""
        self.imu_data = data.decode("utf-8")
        self.publish_imu_data()

    def publish_imu_data(self):
        """Publish IMU data to ROS 2 topic."""
        if self.imu_data:
            imu_msg = Imu()
            try:
                # Split the data and validate the number of parts
                data_parts = self.imu_data.split(",")
                print("imu_data : ")
                print(self.imu_data)
                print("data_parts : ")
                print(data_parts)
                # Convert to floats
                imu_msg.linear_acceleration.x = float(data_parts[0][2:])
                imu_msg.linear_acceleration.y = float(data_parts[1][2:])
                imu_msg.linear_acceleration.z = float(data_parts[2][2:])
                imu_msg.angular_velocity.x = float(data_parts[3][2:])
                imu_msg.angular_velocity.y = float(data_parts[4][2:])
                imu_msg.angular_velocity.z = float(data_parts[5][2:])

                # Publish the message
                self.publisher_.publish(imu_msg)
                self.get_logger().info(f"Published IMU data: {self.imu_data}")
            except ValueError as e:
                self.get_logger().error(f"Error parsing IMU data: {e}")
            except Exception as e:
                self.get_logger().error(f"Unexpected error: {e}")


async def main(args=None):
    rclpy.init(args=args)
    imu_publisher = IMUPublisher()

    # Create a task for the BLE connection
    ble_task = asyncio.create_task(imu_publisher.connect_ble())

    # Run the ROS 2 node in the asyncio loop
    try:
        while rclpy.ok():
            rclpy.spin_once(imu_publisher, timeout_sec=0.1)
            await asyncio.sleep(0.1)
    except KeyboardInterrupt:
        imu_publisher.get_logger().info("Shutting down...")

    # Ensure BLE task is properly stopped
    if not ble_task.done():
        ble_task.cancel()
        await ble_task

    imu_publisher.destroy_node()
    rclpy.shutdown()

if __name__ == "__main__":
    asyncio.run(main())
