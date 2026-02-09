# Copyright 2022 KUKA Hungaria Kft.
#
# Licensed under the Apache License, Version 2.0 (the "License");
# you may not use this file except in compliance with the License.
# You may obtain a copy of the License at
#
#     http://www.apache.org/licenses/LICENSE-2.0
#
# Unless required by applicable law or agreed to in writing, software
# distributed under the License is distributed on an "AS IS" BASIS,
# WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
# See the License for the specific language governing permissions and
# limitations under the License.from launch import LaunchDescription

import socket
import sys
import xml.etree.ElementTree as ET

import numpy as np
import rclpy
from rclpy.node import Node
from std_msgs.msg import String


def create_rsi_xml_rob(act_joint_pos, timeout_count, ipoc):
    q = act_joint_pos
    root = ET.Element("Rob", {"TYPE": "KUKA"})
    ET.SubElement(
        root, "RIst", {"X": "0.0", "Y": "0.0", "Z": "0.0", "A": "0.0", "B": "0.0", "C": "0.0"}
    )
    ET.SubElement(root, "AIPos", {f"A{i+1}": str(q[i]) for i in range(6)})
    ET.SubElement(root, "Delay", {"D": str(timeout_count)})
    ET.SubElement(root, "EXT", {f"E{i+1}": str(q[i + 6]) for i in range(6)})
    ET.SubElement(root, "IPOC").text = str(ipoc)
    return ET.tostring(root, encoding="utf-8", method="xml").replace(b" />", b"/>")


def parse_rsi_xml_sen(data):
    root = ET.fromstring(data)

    corrections = []
    for tag, letter in [("AK", "A"), ("EK", "E")]:
        attribute = root.find(tag).attrib
        values = [attribute[f"{letter}{i+1}"] for i in range(6)]
        corrections.extend(values)

    IPOC = root.find("IPOC").text
    stop_flag = root.find("Stop").text

    return np.array(corrections).astype(np.float64), int(IPOC), bool(int(stop_flag))


class RSISimulator(Node):
    cycle_time = 0.04
    act_joint_pos = np.array([0, -90, 90, 0, 90, 0] + [0] * 6).astype(np.float64)
    initial_joint_pos = act_joint_pos.copy()
    des_joint_correction_absolute = np.zeros(12)
    timeout_count = 0
    ipoc = 0
    rsi_ip_address_ = "127.0.0.1"
    rsi_port_address_ = 59152
    rsi_act_pub_ = None
    rsi_cmd_pub_ = None
    node_name_ = "rsi_simulator_node"
    socket_ = None

    def __init__(self, node_name):
        super().__init__(node_name)
        self.node_name_ = node_name
        self.timer = self.create_timer(self.cycle_time, self.timer_callback)
        self.declare_parameter("rsi_ip_address", "127.0.0.1")
        self.declare_parameter("rsi_port", 59152)
        self.rsi_ip_address_ = (
            self.get_parameter("rsi_ip_address").get_parameter_value().string_value
        )
        self.rsi_port_address_ = self.get_parameter("rsi_port").get_parameter_value().integer_value
        self.rsi_act_pub_ = self.create_publisher(String, self.node_name_ + "/rsi/state", 1)
        self.rsi_cmd_pub_ = self.create_publisher(String, self.node_name_ + "/rsi/command", 1)
        self.get_logger().info(f"rsi_ip_address: {self.rsi_ip_address_}")
        self.get_logger().info(f"rsi_port: {self.rsi_port_address_}")

        self.socket_ = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
        self.get_logger().info("Successfully created socket")
        self.socket_.settimeout(self.cycle_time)

    def timer_callback(self):
        if self.timeout_count == 100:
            self.get_logger().fatal("Timeout count of 100 exceeded")
            sys.exit()
        try:
            msg = create_rsi_xml_rob(self.act_joint_pos, self.timeout_count, self.ipoc)
            self.rsi_act_pub_.publish(msg)
            self.socket_.sendto(msg, (self.rsi_ip_address_, self.rsi_port_address_))
            recv_msg, _ = self.socket_.recvfrom(1024)
            self.rsi_cmd_pub_.publish(recv_msg)
            self.get_logger().warn(f"msg: {recv_msg}")
            des_joint_correction_absolute, ipoc_recv, stop_flag = parse_rsi_xml_sen(recv_msg)
            if ipoc_recv == self.ipoc:
                self.act_joint_pos = self.initial_joint_pos + des_joint_correction_absolute
            else:
                self.get_logger().warn("Packet is late")
                self.get_logger().warn(f"sent ipoc: {self.ipoc}, received: {ipoc_recv}")
                if self.ipoc != 0:
                    self.timeout_count += 1
            self.ipoc += 1
            if stop_flag:
                self.on_shutdown()
                sys.exit()
        except OSError:
            if self.ipoc != 0:
                self.timeout_count += 1
                self.get_logger().warn("Socket timed out")

    def on_shutdown(self):
        self.socket_.close()
        self.get_logger().info("Socket closed.")


def main():
    rclpy.init()
    node = RSISimulator("rsi_simulator_node")
    node.get_logger().info("Started")

    rclpy.spin(node)
    node.on_shutdown()
    node.get_logger().info("Shutting down")


if __name__ == "__main__":
    main()
