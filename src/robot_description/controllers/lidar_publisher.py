import rclpy
from rclpy.node import Node
from sensor_msgs.msg import LaserScan
import serial


class LidarPublisher(Node):
    def __init__(self):
        super().__init__("lidar_publisher")

        # Configurare publisher pentru topicul /scan
        self.publisher_ = self.create_publisher(LaserScan, "/scan", 10)

        # Configurare port serial pentru ESP32
        self.serial_port = serial.Serial("/dev/ttyUSB1", 115200, timeout=1)

        # Parametrii LIDAR
        self.rpm = 166.67  # Viteza motorului (rotații per minut)
        self.points_per_revolution = 360  # Puncte per rotație
        self.scan_time = 60.0 / self.rpm  # Timpul pentru o rotație completă
        self.angle_increment = (
            2 * 3.14159 / self.points_per_revolution
        )  # 1 grad în radiani
        self.time_increment = self.scan_time / self.points_per_revolution

        # Buffer pentru măsurători
        self.ranges = [float("inf")] * self.points_per_revolution

        # Timer pentru publicare sincronizată
        self.timer = self.create_timer(self.scan_time, self.publish_scan)

    def read_and_update_ranges(self):
        """Citește datele de la ESP32 și actualizează buffer-ul ranges."""
        while self.serial_port.in_waiting:
            try:
                # Citește linia serială și parsează unghiul și distanța
                data = self.serial_port.readline().decode().strip()
                angle, distance = map(float, data.split(","))

                # Verifică validitatea unghiului și distanței
                if 0 <= angle < 360 and 0.03 <= distance <= 12.0:
                    index = int(angle)  # Indexul în buffer
                    self.ranges[index] = distance  # Actualizează doar cu valori valide
            except (ValueError, IndexError):
                continue  # Ignoră datele invalide

    def publish_scan(self):
        """Publică mesajul LaserScan pe topicul /scan."""
        msg = LaserScan()

        # Configurare antet
        msg.header.frame_id = "laser_frame"
        msg.header.stamp = self.get_clock().now().to_msg()

        # Configurare parametri unghiulari
        msg.angle_min = 0.0
        msg.angle_max = 2 * 3.14159
        msg.angle_increment = self.angle_increment

        # Configurare parametri temporali
        msg.time_increment = self.time_increment
        msg.scan_time = self.scan_time

        # Configurare interval de măsurare
        msg.range_min = 0.03
        msg.range_max = 12.0

        # Adaugă datele de scanare
        msg.ranges = self.ranges.copy()
        msg.intensities = []  # Fără intensități pentru TFmini Plus

        # Publică mesajul
        self.publisher_.publish(msg)

        # Nu resetăm ranges[], păstrăm datele invalide ca inf


def main(args=None):
    rclpy.init(args=args)
    node = LidarPublisher()

    try:
        while rclpy.ok():
            node.read_and_update_ranges()  # Actualizează buffer-ul cu date de la ESP32
            rclpy.spin_once(node, timeout_sec=0.01)  # Rulează timer-ul pentru publicare
    except KeyboardInterrupt:
        pass

    node.destroy_node()
    rclpy.shutdown()


if __name__ == "__main__":
    main()
