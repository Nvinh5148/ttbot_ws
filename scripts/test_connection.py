import time
from pymavlink import mavutil

# --- CẤU HÌNH ---
connection_string = 'udpout:localhost:14550' # Sửa IP nếu cần
print(f"Dang ket noi toi: {connection_string}")

mav = mavutil.mavlink_connection(connection_string, source_system=1)
start_time = time.time()

counter = 0

while True:
    current_boot_time = int((time.time() - start_time) * 1000)

    # 1. Gửi Heartbeat (Để giữ kết nối)
    mav.mav.heartbeat_send(
        mavutil.mavlink.MAV_TYPE_GROUND_ROVER,
        mavutil.mavlink.MAV_AUTOPILOT_ARDUPILOTMEGA,
        mavutil.mavlink.MAV_MODE_FLAG_AUTO_ENABLED,
        0, 0
    )
    
    # 2. Gửi Tin Nhắn Văn Bản (STATUSTEXT) - Mỗi 3 giây gửi 1 lần
    if counter % 3 == 0:
        text_message = f"Hello QGC! Count: {counter}"
        print(f"Dang gui tin nhan: {text_message}")
        
        # MAV_SEVERITY_INFO (6): Mức độ thông tin thường
        # MAV_SEVERITY_CRITICAL (2): Mức độ nghiêm trọng (QGC sẽ kêu tít tít)
        mav.mav.statustext_send(
            mavutil.mavlink.MAV_SEVERITY_INFO,
            text_message.encode('utf-8') # Phải encode sang bytes
        )

    # Tăng biến đếm
    counter += 1
    time.sleep(1)