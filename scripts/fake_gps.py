import rclpy
from rclpy.node import Node
from sensor_msgs.msg import NavSatFix
import time
import math

class FakeGPS(Node):
    def __init__(self):
        super().__init__('fake_gps_node')
        
        # Publisher bắn vào topic /fix (y hệt như driver thật)
        self.publisher_ = self.create_publisher(NavSatFix, '/fix', 10)
        self.timer = self.create_timer(0.1, self.publish_fake_gps) # 10Hz
        
        # --- TỌA ĐỘ BÁCH KHOA TP.HCM (CƠ SỞ 1 - LÝ THƯỜNG KIỆT) ---
        # Vị trí: Khoảng sân A1 - A4
        self.lat_start = 10.772109
        self.lon_start = 106.657738
        
        self.start_time = time.time()
        print(f"Đang phát GPS giả tại Bách Khoa TP.HCM (Lat: {self.lat_start}, Lon: {self.lon_start})...")

    def publish_fake_gps(self):
        msg = NavSatFix()
        msg.header.stamp = self.get_clock().now().to_msg()
        msg.header.frame_id = "base_link"
        msg.status.status = 0 # 0 = Fix OK (Đã bắt được vệ tinh)

        # Giả lập xe chạy vòng tròn
        t = time.time() - self.start_time
        
        # Bán kính ~20 mét (0.0002 độ)
        radius = 0.0002 
        speed = 0.5 # Tốc độ quay
        
        # Tạo chuyển động tròn
        msg.latitude = self.lat_start + radius * math.sin(t * speed)
        msg.longitude = self.lon_start + radius * math.cos(t * speed)
        
        # Độ cao giả định (TP.HCM khoảng 10m so với mực nước biển)
        msg.altitude = 10.0

        self.publisher_.publish(msg)

def main(args=None):
    rclpy.init(args=args)
    node = FakeGPS()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()