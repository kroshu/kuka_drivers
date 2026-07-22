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
        "motion_state_cartesian_enabled": True,
        "motion_state_cartesian_element": "RIst",
        "motion_state_cartesian_attributes": ["X", "Y", "Z", "A", "B", "C"],
        "motion_state_position_mappings": [
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
        "motion_state_velocity_mappings": [],
        "motion_state_torque_mappings": [],
        "motion_state_delay_element": "Delay",
        "motion_state_delay_attribute": "D",
        "motion_state_ipoc_element": "IPOC",
        "control_signal_position_element": "AK",
        "control_signal_position_attributes": ["A1", "A2", "A3", "A4", "A5", "A6"],
        "control_signal_ext_position_element": "EK",
        "control_signal_ext_position_attributes": ["E1", "E2", "E3", "E4", "E5", "E6"],
        "control_signal_velocity_element": "VK",
        "control_signal_velocity_attributes": [],
        "control_signal_ext_velocity_element": "EVK",
        "control_signal_ext_velocity_attributes": [],
        "control_signal_torque_element": "TK",
        "control_signal_torque_attributes": [],
        "control_signal_ext_torque_element": "ETK",
        "control_signal_ext_torque_attributes": [],
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
        cartesian_enabled = cartesian.get("enabled", True)
        if not isinstance(cartesian_enabled, bool):
            raise ValueError("'motion_state.cartesian.enabled' must be boolean when provided.")
        config["motion_state_cartesian_enabled"] = cartesian_enabled
        config["motion_state_cartesian_element"] = cartesian.get(
            "xml_element", config["motion_state_cartesian_element"]
        )
        config["motion_state_cartesian_attributes"] = cartesian.get(
            "xml_attributes", config["motion_state_cartesian_attributes"]
        )

    def _parse_motion_state_signal_entries(
        entries, signal_name, expected_count, fallback_mappings
    ):
        if entries is None:
            return []
        if not isinstance(entries, list):
            raise ValueError(f"'motion_state.joints.{signal_name}' must be a list.")
        if len(entries) == 0:
            return []
        if len(entries) != expected_count:
            raise ValueError(
                f"'motion_state.joints.{signal_name}' has {len(entries)} entries but "
                f"'motion_state.joints.positions' has {expected_count} entries."
            )

        mappings = []
        for idx, joint in enumerate(entries):
            if not isinstance(joint, dict):
                raise ValueError(
                    f"Each entry in 'motion_state.joints.{signal_name}' must be a dictionary."
                )
            fallback_element, fallback_attribute, _ = fallback_mappings[idx]
            xml_element = joint.get("xml_element", fallback_element)
            xml_attribute = joint.get("xml_attribute", fallback_attribute)
            mappings.append((xml_element, xml_attribute, idx))
        return mappings

    def _apply_optional_control_signal_config(
        control_cfg,
        key,
        element_key,
        attributes_key,
        default_element,
        default_attributes,
    ):
        node = control_cfg.get(key)
        if node is None:
            return
        if not isinstance(node, dict):
            raise ValueError(f"'control_signal.{key}' must be a dictionary when provided.")

        enabled = node.get("enabled", True)
        if not isinstance(enabled, bool):
            raise ValueError(f"'control_signal.{key}.enabled' must be boolean when provided.")

        if not enabled:
            config[attributes_key] = []
            return

        config[element_key] = node.get("xml_element", default_element)
        configured_attributes = node.get("xml_attributes")
        if configured_attributes is not None:
            if not isinstance(configured_attributes, list):
                raise ValueError(
                    f"'control_signal.{key}.xml_attributes' must be a list when provided."
                )
            config[attributes_key] = configured_attributes
        else:
            config[attributes_key] = default_attributes

    joints = motion_state.get("joints")
    external_joint_count = None
    internal_joint_count = None
    if joints is not None:
        if not isinstance(joints, dict):
            raise ValueError("'motion_state.joints' must be a dictionary.")
        positions = joints.get("positions")
        if positions is None:
            raise ValueError(
                "'motion_state.joints.positions' must be a list when "
                "'motion_state.joints' is provided."
            )
        if not isinstance(positions, list):
            raise ValueError("'motion_state.joints.positions' must be a list.")

        joint_mappings = []
        for idx, joint in enumerate(positions):
            if not isinstance(joint, dict):
                raise ValueError(
                    "Each entry in 'motion_state.joints.positions' must be a dictionary."
                )
            xml_element = joint.get("xml_element", "AIPos")
            xml_attribute = joint.get("xml_attribute", f"A{idx + 1}")
            joint_mappings.append((xml_element, xml_attribute, idx))
        if joint_mappings:
            config["motion_state_position_mappings"] = joint_mappings
            external_joint_count = sum(
                1 for _, xml_attribute, _ in joint_mappings if str(xml_attribute).startswith("E")
            )
            internal_joint_count = len(joint_mappings) - external_joint_count

            config["motion_state_velocity_mappings"] = _parse_motion_state_signal_entries(
                joints.get("velocities"), "velocities", len(joint_mappings), joint_mappings
            )
            config["motion_state_torque_mappings"] = _parse_motion_state_signal_entries(
                joints.get("torques"), "torques", len(joint_mappings), joint_mappings
            )

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

    control_joints = control_signal.get("joints")
    if control_joints is not None and not isinstance(control_joints, dict):
        raise ValueError("'control_signal.joints' must be a dictionary when provided.")
    if control_joints:
        config["control_signal_position_element"] = control_joints.get(
            "xml_element", config["control_signal_position_element"]
        )
        configured_joint_attrs = control_joints.get("xml_attributes")
        if configured_joint_attrs:
            config["control_signal_position_attributes"] = configured_joint_attrs
        elif external_joint_count is not None:
            internal_joint_count = (
                len(config["motion_state_position_mappings"]) - external_joint_count
            )
            config["control_signal_position_attributes"] = [
                f"A{i}" for i in range(1, internal_joint_count + 1)
            ]

    control_ext_joints = control_signal.get("ext_joints")
    if control_ext_joints is not None:
        if not isinstance(control_ext_joints, dict):
            raise ValueError("'control_signal.ext_joints' must be a dictionary when provided.")
        config["control_signal_ext_position_element"] = control_ext_joints.get(
            "xml_element", config["control_signal_ext_position_element"]
        )
        configured_ext_attrs = control_ext_joints.get("xml_attributes")
        if configured_ext_attrs is not None:
            config["control_signal_ext_position_attributes"] = configured_ext_attrs
        elif external_joint_count is not None:
            config["control_signal_ext_position_attributes"] = [
                f"E{i}" for i in range(1, external_joint_count + 1)
            ]

    inferred_internal_attributes = []
    if internal_joint_count is not None:
        inferred_internal_attributes = [f"A{i}" for i in range(1, internal_joint_count + 1)]

    inferred_external_attributes = []
    if external_joint_count is not None:
        inferred_external_attributes = [f"E{i}" for i in range(1, external_joint_count + 1)]

    _apply_optional_control_signal_config(
        control_signal,
        "velocities",
        "control_signal_velocity_element",
        "control_signal_velocity_attributes",
        config["control_signal_velocity_element"],
        inferred_internal_attributes,
    )
    _apply_optional_control_signal_config(
        control_signal,
        "ext_velocities",
        "control_signal_ext_velocity_element",
        "control_signal_ext_velocity_attributes",
        config["control_signal_ext_velocity_element"],
        inferred_external_attributes,
    )
    _apply_optional_control_signal_config(
        control_signal,
        "torques",
        "control_signal_torque_element",
        "control_signal_torque_attributes",
        config["control_signal_torque_element"],
        inferred_internal_attributes,
    )
    _apply_optional_control_signal_config(
        control_signal,
        "ext_torques",
        "control_signal_ext_torque_element",
        "control_signal_ext_torque_attributes",
        config["control_signal_ext_torque_element"],
        inferred_external_attributes,
    )

    control_ipoc = control_signal.get("ipoc") or {}
    if control_ipoc:
        config["control_signal_ipoc_element"] = control_ipoc.get(
            "xml_element", config["control_signal_ipoc_element"]
        )

    return config


def _append_joint_signal_elements(root, mappings, values):
    joint_elements = {}
    for xml_element, xml_attribute, joint_idx in mappings:
        if joint_idx < len(values):
            if xml_element not in joint_elements:
                joint_elements[xml_element] = {}
            joint_elements[xml_element][xml_attribute] = str(values[joint_idx])

    for xml_element, attrs in joint_elements.items():
        ET.SubElement(root, xml_element, attrs)


def create_rsi_xml_rob(
    act_joint_pos, act_joint_vel, act_joint_torque, timeout_count, ipoc, xml_config
):
    root = ET.Element("Rob", {"TYPE": "KUKA"})
    if xml_config["motion_state_cartesian_enabled"]:
        ET.SubElement(
            root,
            xml_config["motion_state_cartesian_element"],
            {attr: "0.0" for attr in xml_config["motion_state_cartesian_attributes"]},
        )

    _append_joint_signal_elements(
        root, xml_config["motion_state_position_mappings"], act_joint_pos
    )
    _append_joint_signal_elements(
        root, xml_config["motion_state_velocity_mappings"], act_joint_vel
    )
    _append_joint_signal_elements(
        root, xml_config["motion_state_torque_mappings"], act_joint_torque
    )

    ET.SubElement(
        root,
        xml_config["motion_state_delay_element"],
        {xml_config["motion_state_delay_attribute"]: str(timeout_count)},
    )
    ET.SubElement(root, xml_config["motion_state_ipoc_element"]).text = str(ipoc)
    return ET.tostring(root, encoding="utf-8", method="xml").replace(b" />", b"/>")


def parse_rsi_xml_sen(data, xml_config):
    root = ET.fromstring(data)

    def _extract_values(tag, attributes):
        if not attributes:
            return []
        node = root.find(tag)
        if node is None:
            return ["0"] * len(attributes)
        return [node.attrib.get(attr, "0") for attr in attributes]

    corrections = []
    for tag, attributes in [
        (
            xml_config["control_signal_position_element"],
            xml_config["control_signal_position_attributes"],
        ),
        (
            xml_config["control_signal_ext_position_element"],
            xml_config["control_signal_ext_position_attributes"],
        ),
    ]:
        corrections.extend(_extract_values(tag, attributes))

    corrections = (corrections + [0] * 12)[:12]

    velocities = []
    for tag, attributes in [
        (
            xml_config["control_signal_velocity_element"],
            xml_config["control_signal_velocity_attributes"],
        ),
        (
            xml_config["control_signal_ext_velocity_element"],
            xml_config["control_signal_ext_velocity_attributes"],
        ),
    ]:
        velocities.extend(_extract_values(tag, attributes))
    velocities = (velocities + [0] * 12)[:12]

    torques = []
    for tag, attributes in [
        (
            xml_config["control_signal_torque_element"],
            xml_config["control_signal_torque_attributes"],
        ),
        (
            xml_config["control_signal_ext_torque_element"],
            xml_config["control_signal_ext_torque_attributes"],
        ),
    ]:
        torques.extend(_extract_values(tag, attributes))
    torques = (torques + [0] * 12)[:12]

    ipoc_node = root.find(xml_config["control_signal_ipoc_element"])
    IPOC = ipoc_node.text if ipoc_node is not None else "0"
    stop_node = root.find("Stop")
    stop_flag = stop_node.text if stop_node is not None else "0"

    return (
        np.array(corrections).astype(np.float64),
        np.array(velocities).astype(np.float64),
        np.array(torques).astype(np.float64),
        int(IPOC),
        bool(int(stop_flag)),
    )


class RSISimulator(Node):
    cycle_time = 0.04
    act_joint_pos = np.array([0, -90, 90, 0, 90, 0] + [0] * 6).astype(np.float64)
    act_joint_vel = np.zeros(12).astype(np.float64)
    act_joint_torque = np.zeros(12).astype(np.float64)
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
                self.act_joint_pos,
                self.act_joint_vel,
                self.act_joint_torque,
                self.timeout_count,
                self.ipoc,
                self.rsi_xml_config_,
            )
            self.rsi_act_pub_.publish(String(data=msg.decode("utf-8")))
            self.socket_.sendto(msg, (self.rsi_ip_address_, self.rsi_port_address_))
            recv_msg, _ = self.socket_.recvfrom(1024)
            self.rsi_cmd_pub_.publish(String(data=recv_msg.decode("utf-8")))
            self.get_logger().warn(f"msg: {recv_msg}")
            (
                des_joint_correction_absolute,
                des_joint_velocity,
                des_joint_torque,
                ipoc_recv,
                stop_flag,
            ) = parse_rsi_xml_sen(recv_msg, self.rsi_xml_config_)
            self.act_joint_pos = self.initial_joint_pos + des_joint_correction_absolute
            self.act_joint_vel = des_joint_velocity
            self.act_joint_torque = des_joint_torque
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
