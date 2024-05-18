import rospy
from sensor_msgs.msg import LaserScan
import threading

class LidarSubscriber:
    def __init__(self):
        rospy.init_node('lidar_subscriber', anonymous=True)
        self.scan_data = []
        self.scan_lock = threading.Lock()
        self.angle_range = 360  # 전체 각도 범위
        self.previous_angle = None
        self.data_ready = threading.Event()
        self.first_scan = True

        rospy.Subscriber('/scan', LaserScan, self.callback)

    def callback(self, msg):
        with self.scan_lock:
            for i, distance in enumerate(msg.ranges):
                angle = msg.angle_min + i * msg.angle_increment
                angle_degrees = angle * 180.0 / 3.141592653589793

                # distance가 inf가 아닌 경우만 데이터에 추가
                if not distance == float('inf'):
                    self.scan_data.append((angle, distance))

                # 각도 범위가 360도를 넘어가면 데이터 처리
                if self.previous_angle is not None and self.previous_angle > angle_degrees:
                    if not self.first_scan:  # 첫 스캔에서는 데이터를 처리하지 않음
                        self.data_ready.set()
                    self.first_scan = False

                self.previous_angle = angle_degrees

    def get_scan_data(self):
        with self.scan_lock:
            scan_data_copy = self.scan_data.copy()
            self.scan_data.clear()
            self.previous_angle = None
            return scan_data_copy

    def wait_for_data(self):
        self.data_ready.wait()
        self.data_ready.clear()

    def run(self):
        rospy.spin()

if __name__ == '__main__':
    lidar_subscriber = LidarSubscriber()
    threading.Thread(target=lambda: rospy.spin()).start()
