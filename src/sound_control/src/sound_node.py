import rclpy
from rclpy.node import Node
from std_msgs.msg import String
import subprocess
import os

# ✅ 여기 추가 (중요)
os.environ["AUDIODEV"] = "hw:0,0"

class SoundNode(Node):
    def __init__(self):
        super().__init__('sound_node')

        self.sub = self.create_subscription(
            String,
            '/sound',
            self.callback,
            10
        )

        self.home = os.path.expanduser("~")
        self.device = "plughw:0,0"

    def play(self, filename):
        path = f"{self.home}/sounds/{filename}"

        if not os.path.exists(path):
            self.get_logger().error(f"파일 없음: {path}")
            return

        subprocess.run([
            "aplay",
            "-D", self.device,
            path
        ])

    def callback(self, msg):
        command = msg.data

        if command == "start":
            self.play("start.wav")

        elif command == "stop":
            self.play("stop.wav")

        elif command == "charge":
            # self.play("charge.wav")
            self.play("충전이 완료 되었습니다_12800.wav")


def main(args=None):
    rclpy.init(args=args)
    node = SoundNode()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()