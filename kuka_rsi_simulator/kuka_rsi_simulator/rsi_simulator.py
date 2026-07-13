# Copyright 2026 KUKA Hungaria Kft.
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
# limitations under the License.

import signal
import socket
import sys
import xml.etree.ElementTree as ET
from pathlib import Path

import numpy as np
import rclpy
from rclpy.node import Node
from std_msgs.msg import String


def _default_rsi_xml_config():
    return {
        "motion_state_cartesian_element": "RIst",
        "motion_state_cartesian_attributes": ["X", "Y", "Z", "A", "B", "C"],
        "motion_state_joint_mappings": [
            ("AIPos", "A1", 0),
            ("AIPos", "A2", 1),
            ("AIPos", "A3", 2),
            ("AIPos", "A4", 3),
            ("AIPos", "A5", 4),
            ("AIPos", "A6", 5),
            ("EIPos", "E1", 6),
            ("EIPos", "E2", 7),
            ("EIPos", "E3", 8),
            ("EIPos", "E4", 9),
            ("EIPos", "E5", 10),
            ("EIPos", "E6", 11),
        ],
        "motion_state_delay_element": "Delay",
        "motion_state_delay_attribute": "D",
        "motion_state_ipoc_element": "IPOC",
        "control_signal_joint_element": "AK",
        "control_signal_joint_attributes": ["A1", "A2", "A3", "A4", "A5", "A6"],
        "control_signal_ext_joint_element": "EK",
        "control_signal_ext_joint_attributes": ["E1", "E2", "E3", "E4", "E5", "E6"],
        "control_signal_ipoc_element": "IPOC",
    }


def _load_rsi_xml_config(config_path):
    config = _default_rsi_xml_config()
    if not config_path:
        return config

    try:
        import yaml
    except ImportError as exc:
        raise RuntimeError("PyYAML is required when 'rsi_xml_config_file' is provided.") from exc

    with Path(config_path).open("r", encoding="utf-8") as fh:
        data = yaml.safe_load(fh) or {}

    if not isinstance(data, dict):
        raise ValueError("RSI XML config must be a YAML dictionary.")
    if "rsi_xml_config" not in data:
        raise ValueError("RSI XML config must contain the top-level 'rsi_xml_config' key.")

    rsi_config = data.get("rsi_xml_config") or {}
    if not isinstance(rsi_config, dict):
        raise ValueError("'rsi_xml_config' must be a dictionary.")

    motion_state = rsi_config.get("motion_state") or {}
    control_signal = rsi_config.get("control_signal") or {}

    if not isinstance(motion_state, dict):
        raise ValueError("'motion_state' must be a dictionary when provided.")
    if not isinstance(control_signal, dict):
        raise ValueError("'control_signal' must be a dictionary when provided.")

    cartesian = motion_state.get("cartesian") or {}
    if cartesian:
        config["motion_state_cartesian_element"] = cartesian.get(
            "xml_element", config["motion_state_cartesian_element"]
        )
        config["motion_state_cartesian_attributes"] = cartesian.get(
            "xml_attributes", config["motion_state_cartesian_attributes"]
        )

    joints = motion_state.get("joints")
    if joints is not None:
        if not isinstance(joints, list):
            raise ValueError("'motion_state.joints' must be a list when provided.")
        joint_mappings = []
        for idx, joint in enumerate(joints):
            if not isinstance(joint, dict):
                raise ValueError("Each entry in 'motion_state.joints' must be a dictionary.")
            joint_mappings.append(
                (
                    joint.get("xml_element", "AIPos"),
                    joint.get("xml_attribute", f"A{idx + 1}"),
                    idx,
                )
            )
        if joint_mappings:
            config["motion_state_joint_mappings"] = joint_mappings

    delay = motion_state.get("delay") or {}
    if delay:
        config["motion_state_delay_element"] = delay.get(
            "xml_element", config["motion_state_delay_element"]
        )
        config["motion_state_delay_attribute"] = delay.get(
            "xml_attribute", config["motion_state_delay_attribute"]
        )

    motion_state_ipoc = motion_state.get("ipoc") or {}
    if motion_state_ipoc:
        config["motion_state_ipoc_element"] = motion_state_ipoc.get(
            "xml_element", config["motion_state_ipoc_element"]
        )

    control_joints = control_signal.get("joints") or {}
    if control_joints:
        config["control_signal_joint_element"] = control_joints.get(
            "xml_element", config["control_signal_joint_element"]
        )
        configured_joint_attrs = control_joints.get("xml_attributes")
        if configured_joint_attrs:
            config["control_signal_joint_attributes"] = configured_joint_attrs

    control_ext_joints = control_signal.get("ext_joints")
    if control_ext_joints is not None:
        if not isinstance(control_ext_joints, dict):
            raise ValueError("'control_signal.ext_joints' must be a dictionary when provided.")
        config["control_signal_ext_joint_element"] = control_ext_joints.get(
            "xml_element", config["control_signal_ext_joint_element"]
        )
        configured_ext_attrs = control_ext_joints.get("xml_attributes")
        if configured_ext_attrs is not None:
            config["control_signal_ext_joint_attributes"] = configured_ext_attrs

    control_ipoc = control_signal.get("ipoc") or {}
    if control_ipoc:
        config["control_signal_ipoc_element"] = control_ipoc.get(
            "xml_element", config["control_signal_ipoc_element"]
        )

    return config


def create_rsi_xml_rob(act_joint_pos, timeout_count, ipoc, xml_config):
    q = act_joint_pos
    root = ET.Element("Rob", {"TYPE": "KUKA"})
    ET.SubElement(
        root,
        xml_config["motion_state_cartesian_element"],
        {attr: "0.0" for attr in xml_config["motion_state_cartesian_attributes"]},
    )

    joint_elements = {}
    for xml_element, xml_attribute, joint_idx in xml_config["motion_state_joint_mappings"]:
        if joint_idx < len(q):
            if xml_element not in joint_elements:
                joint_elements[xml_element] = {}
            joint_elements[xml_element][xml_attribute] = str(q[joint_idx])

    for xml_element, attrs in joint_elements.items():
        ET.SubElement(root, xml_element, attrs)

    ET.SubElement(
        root,
        xml_config["motion_state_delay_element"],
        {xml_config["motion_state_delay_attribute"]: str(timeout_count)},
    )
    ET.SubElement(root, xml_config["motion_state_ipoc_element"]).text = str(ipoc)
    return ET.tostring(root, encoding="utf-8", method="xml").replace(b" />", b"/>")


def parse_rsi_xml_sen(data, xml_config):
    root = ET.fromstring(data)

    corrections = []
    for tag, attributes in [
        (
            xml_config["control_signal_joint_element"],
            xml_config["control_signal_joint_attributes"],
        ),
        (
            xml_config["control_signal_ext_joint_element"],
            xml_config["control_signal_ext_joint_attributes"],
        ),
    ]:
        if root.find(tag) is not None:
            element_attributes = root.find(tag).attrib
            values = [
                element_attributes[attr] for attr in attributes if element_attributes.get(attr)
            ]
            corrections.extend(values)

    corrections.extend([0] * (12 - len(corrections)))

    ipoc_node = root.find(xml_config["control_signal_ipoc_element"])
    IPOC = ipoc_node.text if ipoc_node is not None else "0"
    stop_node = root.find("Stop")
    stop_flag = stop_node.text if stop_node is not None else "0"

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
    rsi_xml_config_ = None
    ignore_ipoc_ = False

    def __init__(self, node_name):
        super().__init__(node_name)
        self.node_name_ = node_name
        self.timer = self.create_timer(self.cycle_time, self.timer_callback)
        self.declare_parameter("rsi_ip_address", "127.0.0.1")
        self.declare_parameter("rsi_port", 59152)
        self.declare_parameter("rsi_xml_config_file", "")
        self.declare_parameter("ignore_ipoc", False)
        self.rsi_ip_address_ = (
            self.get_parameter("rsi_ip_address").get_parameter_value().string_value
        )
        self.rsi_port_address_ = self.get_parameter("rsi_port").get_parameter_value().integer_value
        rsi_xml_config_file = (
            self.get_parameter("rsi_xml_config_file").get_parameter_value().string_value
        )
        self.ignore_ipoc_ = self.get_parameter("ignore_ipoc").get_parameter_value().bool_value
        self.rsi_xml_config_ = _load_rsi_xml_config(rsi_xml_config_file)
        self.rsi_act_pub_ = self.create_publisher(String, self.node_name_ + "/rsi/state", 1)
        self.rsi_cmd_pub_ = self.create_publisher(String, self.node_name_ + "/rsi/command", 1)
        self.get_logger().info(f"rsi_ip_address: {self.rsi_ip_address_}")
        self.get_logger().info(f"rsi_port: {self.rsi_port_address_}")
        self.get_logger().info(
            f"rsi_xml_config_file: {rsi_xml_config_file if rsi_xml_config_file else '(defaults)'}"
        )
        self.get_logger().info(f"ignore_ipoc: {self.ignore_ipoc_}")

        self.socket_ = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
        self.get_logger().info("Successfully created socket")
        self.socket_.settimeout(self.cycle_time)

    def timer_callback(self):
        try:
            msg = create_rsi_xml_rob(
                self.act_joint_pos, self.timeout_count, self.ipoc, self.rsi_xml_config_
            )
            self.rsi_act_pub_.publish(String(data=msg.decode("utf-8")))
            self.socket_.sendto(msg, (self.rsi_ip_address_, self.rsi_port_address_))
            recv_msg, _ = self.socket_.recvfrom(1024)
            self.rsi_cmd_pub_.publish(String(data=recv_msg.decode("utf-8")))
            self.get_logger().warn(f"msg: {recv_msg}")
            des_joint_correction_absolute, ipoc_recv, stop_flag = parse_rsi_xml_sen(
                recv_msg, self.rsi_xml_config_
            )
            self.act_joint_pos = self.initial_joint_pos + des_joint_correction_absolute
            if not self.ignore_ipoc_ and ipoc_recv != self.ipoc:
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

    def _sigterm_handler(signum, frame):
        node.get_logger().info("SIGINT received, shutting down")
        rclpy.shutdown()

    signal.signal(signal.SIGINT, _sigterm_handler)

    rclpy.spin(node)
    node.on_shutdown()
    node.get_logger().info("Shutting down")


if __name__ == "__main__":
    main()
