import rclpy
from rclpy.node import Node

from os import system
import socket


class SetupUDPStream(Node):
    def __init__(self):
        super().__init__("setup_udp_stream")
        (self.receive_port, width, height, fps, self.host) = self.declare_parameters(
            namespace="",
            parameters=[
                ("receive_port", 5000),
                ("width", 640),
                ("height", 480),
                ("fps", 30),
                ("host", ""),
            ],
        )
        self.video_mode = (
            "--mode 768:432 --exposure sport --width 768 --height 432 --framerate 60"
        )


def main(args=None):
    rclpy.init(args=args)

    setup_udp_stream = SetupUDPStream()

    port = 10003
    size = 1024
    s = socket.socket(socket.AF_INET, socket.SOCK_STREAM)
    s.connect((setup_udp_stream.host.value, port))
    video_mode_str = (
        str(setup_udp_stream.receive_port.value)
        + ","
        + str(setup_udp_stream.video_mode)
        + "\n"
    )
    s.send(video_mode_str.encode())
    all_data = ""
    while not all_data.endswith("\n"):
        data = s.recv(size)
        all_data += data.decode("utf-8")
    s.close()
    system(
        "hping3 -c 1 -2 -s "
        + str(setup_udp_stream.receive_port.value)
        + " -p "
        + all_data.strip()
        + " "
        + setup_udp_stream.host.value
    )
    print("Received:", all_data)
    rclpy.shutdown()


if __name__ == "__main__":
    main()
