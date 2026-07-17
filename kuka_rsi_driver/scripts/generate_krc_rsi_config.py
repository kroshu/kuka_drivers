#!/usr/bin/env python3
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

r"""
Generate a KRC-side RSI ethernet XML configuration file from an abstract YAML config.

Usage
-----
ros2 run kuka_rsi_driver generate_krc_rsi_config \
    --config rsi_xml_config.yaml \
    --client-ip 192.168.1.100 \
    --client-port 59152 \
    --output rsi_ethernet.xml

The generated file must be placed on the KRC at:
  KSS:      C:/KRC/ROBOTER/Config/User/Common/SensorInterface/
  iiQKA:    /Config/RobotSensorInterface/Ethernet configuration/

Notes
-----
- Motion-state and control-signal XML mappings are generated from the YAML structure under
  `rsi_xml_config`.
- Motion-state groups that use the default KRC element names are emitted using their KRC
  built-in `DEF_` shortcuts (`RIst` -> `DEF_RIst`, `AIPos` -> `DEF_AIPos`,
  `EIPos` -> `DEF_EIPos`). Groups that use custom element names are expanded into individual,
  explicitly-indexed `<ELEMENT>` entries.
- Delay is always generated as KRC built-in `DEF_Delay` (it is not configurable in YAML) and
  is always placed after all configurable SEND fields.
- IPOC is handled by RSI runtime and is not configured in this ethernet XML.

"""

import argparse
import sys
from xml.etree import ElementTree as ET
from xml.dom import minidom

import yaml

# Default element/attribute names (match SDK defaults)
_DEFAULT_CARTESIAN_ELEMENT = "RIst"
_DEFAULT_CARTESIAN_ATTRIBUTES = ["X", "Y", "Z", "A", "B", "C"]
_DEFAULT_POSITIONS_ELEMENT = "AIPos"
_DEFAULT_EXT_JOINT_ELEMENT = "EIPos"
_DEFAULT_JOINT_CMD_ELEMENT = "AK"
_DEFAULT_EXT_JOINT_CMD_ELEMENT = "EK"
_DEFAULT_VELOCITY_CMD_ELEMENT = "VK"
_DEFAULT_EXT_VELOCITY_CMD_ELEMENT = "EVK"
_DEFAULT_TORQUE_CMD_ELEMENT = "TK"
_DEFAULT_EXT_TORQUE_CMD_ELEMENT = "ETK"

# KRC RSI built-in SEND shortcuts. When a motion-state group uses the default
# element name, the KRC provides a built-in "DEF_" shortcut that emits the whole
# standard block internally. In that case we emit the shortcut instead of
# expanding the group into individual, explicitly-indexed elements.
_DEF_SHORTCUTS = {
    _DEFAULT_CARTESIAN_ELEMENT: "DEF_RIst",
    _DEFAULT_POSITIONS_ELEMENT: "DEF_AIPos",
    _DEFAULT_EXT_JOINT_ELEMENT: "DEF_EIPos",
}


def _prettify(elem: ET.Element) -> str:
    """Return a pretty-printed XML string for the Element."""
    raw = ET.tostring(elem, encoding="unicode")
    # minidom adds an XML declaration; remove duplicate declarations if any
    reparsed = minidom.parseString(raw)
    return reparsed.toprettyxml(indent="  ")


def load_config(path: str) -> dict:
    with open(path) as fh:
        data = yaml.safe_load(fh)
    if not isinstance(data, dict) or "rsi_xml_config" not in data:
        raise ValueError(f"Config file '{path}' must contain a top-level 'rsi_xml_config' key.")
    return data.get("rsi_xml_config", {}) or {}


def _warn(message: str) -> None:
    print(f"WARNING: {message}", file=sys.stderr)


def _require_string(node: dict, key: str, ctx: str) -> str:
    value = node.get(key)
    if not isinstance(value, str) or not value:
        raise ValueError(f"{ctx}.{key} must be a non-empty string.")
    return value


def _extract_motion_state_signal_entries(
    joints_cfg: dict,
    signal_name: str,
    expected_count: int,
) -> list:
    entries = joints_cfg.get(signal_name)
    if entries is None:
        return []
    if not isinstance(entries, list):
        raise ValueError(f"motion_state.joints.{signal_name} must be a list.")
    if len(entries) != expected_count:
        raise ValueError(
            f"motion_state.joints.{signal_name} has {len(entries)} entries but "
            f"positions has {expected_count} entries."
        )
    return entries


def _validate_joint_entries(entries: list, ctx: str) -> None:
    for i, entry in enumerate(entries):
        if not isinstance(entry, dict):
            raise ValueError(f"{ctx}[{i}] must be a map.")
        _require_string(entry, "joint_identifier", f"{ctx}[{i}]")
        _require_string(entry, "xml_element", f"{ctx}[{i}]")
        _require_string(entry, "xml_attribute", f"{ctx}[{i}]")


def _add_receive_signal(
    elements_recv: ET.Element,
    start_index: int,
    cfg: dict,
    cfg_key: str,
    default_element: str,
    default_attributes: list,
):
    node = cfg.get(cfg_key)
    if node is None:
        return start_index
    if not isinstance(node, dict):
        raise ValueError(f"control_signal.{cfg_key} must be a map.")

    enabled = node.get("enabled", True)
    if not isinstance(enabled, bool):
        raise ValueError(f"control_signal.{cfg_key}.enabled must be boolean when set.")
    if not enabled:
        return start_index

    xml_element = node.get("xml_element", default_element)
    if not isinstance(xml_element, str) or not xml_element:
        raise ValueError(f"control_signal.{cfg_key}.xml_element must be a non-empty string.")

    xml_attributes = node.get("xml_attributes", default_attributes)
    if not isinstance(xml_attributes, list):
        raise ValueError(
            f"control_signal.{cfg_key}.xml_attributes must be a list "
            f"(or inferable from joint layout)."
        )
    if not xml_attributes:
        _warn(
            f"Skipping control_signal.{cfg_key}: no XML attributes configured or inferable "
            f"from joint layout."
        )
        return start_index

    idx = start_index
    for attr in xml_attributes:
        if not isinstance(attr, str) or not attr:
            raise ValueError(f"control_signal.{cfg_key}.xml_attributes contains invalid value.")
        ET.SubElement(
            elements_recv,
            "ELEMENT",
            TAG=f"{xml_element}.{attr}",
            TYPE="DOUBLE",
            INDX=str(idx),
            HOLDON="1",
        )
        idx += 1
    return idx


def build_krc_xml(
    rsi_cfg: dict,
    client_ip: str,
    client_port: int,
) -> ET.Element:
    """Build the ROOT element for the KRC ethernet XML."""
    root = ET.Element("ROOT")

    # ----- CONFIG -----
    config_el = ET.SubElement(root, "CONFIG")
    ET.SubElement(config_el, "IP_NUMBER").text = client_ip
    ET.SubElement(config_el, "PORT").text = str(client_port)
    ET.SubElement(config_el, "SENTYPE").text = "KROSHU"
    ET.SubElement(config_el, "ONLYSEND").text = "FALSE"

    ms_cfg = rsi_cfg.get("motion_state", {}) or {}
    cs_cfg = rsi_cfg.get("control_signal", {}) or {}

    # ----- Parse and validate motion_state -----
    joints_cfg = ms_cfg.get("joints", {}) or {}
    if not isinstance(joints_cfg, dict):
        raise ValueError("motion_state.joints must be a map.")

    positions_cfg = joints_cfg.get("positions", [])
    if not isinstance(positions_cfg, list) or not positions_cfg:
        raise ValueError("motion_state.joints.positions is required and must be a non-empty list.")
    _validate_joint_entries(positions_cfg, "motion_state.joints.positions")

    velocities_cfg = _extract_motion_state_signal_entries(
        joints_cfg, "velocities", len(positions_cfg)
    )
    torques_cfg = _extract_motion_state_signal_entries(joints_cfg, "torques", len(positions_cfg))
    _validate_joint_entries(velocities_cfg, "motion_state.joints.velocities")
    _validate_joint_entries(torques_cfg, "motion_state.joints.torques")

    ext_positions_send = [
        j for j in positions_cfg if j.get("xml_element", "") == _DEFAULT_EXT_JOINT_ELEMENT
    ]
    n_joints = len(positions_cfg)
    n_external = len(ext_positions_send)
    n_internal = n_joints - n_external

    # ----- SEND (KRC → PC) -----
    send_el = ET.SubElement(root, "SEND")
    elements_send = ET.SubElement(send_el, "ELEMENTS")

    send_index = 1

    def _emit_send_group(elem_name, tags, type_="DOUBLE"):
        """Emit a motion-state SEND group.

        If ``elem_name`` matches a KRC built-in default, emit the corresponding
        ``DEF_`` shortcut (a single internally-handled element). Otherwise, expand
        the group into individual, explicitly-indexed elements.
        """
        nonlocal send_index
        shortcut = _DEF_SHORTCUTS.get(elem_name)
        if shortcut is not None:
            ET.SubElement(elements_send, "ELEMENT", TAG=shortcut, TYPE=type_, INDX="INTERNAL")
            return
        for tag in tags:
            ET.SubElement(elements_send, "ELEMENT", TAG=tag, TYPE=type_, INDX=str(send_index))
            send_index += 1

    # Cartesian motion state
    cartesian_cfg = ms_cfg.get("cartesian", {}) or {}
    cartesian_elem = cartesian_cfg.get("xml_element", _DEFAULT_CARTESIAN_ELEMENT)
    if not isinstance(cartesian_elem, str) or not cartesian_elem:
        raise ValueError("motion_state.cartesian.xml_element must be a non-empty string.")
    cartesian_attrs = cartesian_cfg.get("xml_attributes", _DEFAULT_CARTESIAN_ATTRIBUTES)
    if not isinstance(cartesian_attrs, list) or len(cartesian_attrs) != 6:
        raise ValueError("motion_state.cartesian.xml_attributes must be a list with 6 entries.")
    for attr in cartesian_attrs:
        if not isinstance(attr, str) or not attr:
            raise ValueError("motion_state.cartesian.xml_attributes contains invalid value.")
    _emit_send_group(cartesian_elem, [f"{cartesian_elem}.{attr}" for attr in cartesian_attrs])

    # Joint motion-state mappings. Contiguous runs that share an xml_element are
    # emitted together so default elements collapse into their DEF_ shortcut.
    for entries in (positions_cfg, velocities_cfg, torques_cfg):
        run_elem = None
        run_tags = []
        for entry in entries:
            elem = entry["xml_element"]
            if run_elem is not None and elem != run_elem:
                _emit_send_group(run_elem, run_tags)
                run_tags = []
            run_elem = elem
            run_tags.append(f"{elem}.{entry['xml_attribute']}")
        if run_elem is not None:
            _emit_send_group(run_elem, run_tags)

    # GPIO state SEND entries
    gpio_state_cfg = ms_cfg.get("gpio", {}) or {}
    gpio_state_attrs = gpio_state_cfg.get("xml_attributes", []) or []
    gpio_state_elem = gpio_state_cfg.get("xml_element", "GPIO")
    for attr in gpio_state_attrs:
        if not isinstance(attr, str) or not attr:
            raise ValueError("motion_state.gpio.xml_attributes contains invalid value.")
        tag = f"{gpio_state_elem}.{attr}"
        ET.SubElement(elements_send, "ELEMENT", TAG=tag, TYPE="BOOL", INDX=str(send_index))
        send_index += 1

    # Delay is fixed and always present after configurable send fields.
    ET.SubElement(elements_send, "ELEMENT", TAG="DEF_Delay", TYPE="LONG", INDX="INTERNAL")

    # ----- RECEIVE (PC → KRC) -----
    recv_el = ET.SubElement(root, "RECEIVE")
    elements_recv = ET.SubElement(recv_el, "ELEMENTS")

    # Stop is always first
    ET.SubElement(elements_recv, "ELEMENT", TAG="Stop", TYPE="BOOL", INDX="1", HOLDON="0")

    indx = 2

    # Internal joint command entries
    joints_cmd_cfg = cs_cfg.get("joints", {}) or {}
    if not isinstance(joints_cmd_cfg, dict):
        raise ValueError("control_signal.joints must be a map.")
    joint_cmd_elem = joints_cmd_cfg.get("xml_element", _DEFAULT_JOINT_CMD_ELEMENT)
    joint_cmd_attrs = joints_cmd_cfg.get("xml_attributes", []) or []

    if not joint_cmd_attrs:
        joint_cmd_attrs = [f"A{i}" for i in range(1, n_internal + 1)]

    for attr in joint_cmd_attrs:
        tag = f"{joint_cmd_elem}.{attr}"
        ET.SubElement(elements_recv, "ELEMENT", TAG=tag, TYPE="DOUBLE", INDX=str(indx), HOLDON="1")
        indx += 1

    # External joint command entries
    ext_joints_cmd_cfg = cs_cfg.get("ext_joints", None)
    if ext_joints_cmd_cfg is not None:
        if not isinstance(ext_joints_cmd_cfg, dict):
            raise ValueError("control_signal.ext_joints must be a map.")
        ext_joint_cmd_elem = ext_joints_cmd_cfg.get("xml_element", _DEFAULT_EXT_JOINT_CMD_ELEMENT)
        ext_joint_cmd_attrs = ext_joints_cmd_cfg.get("xml_attributes", []) or []
        if not ext_joint_cmd_attrs:
            ext_joint_cmd_attrs = [f"E{i}" for i in range(1, len(ext_positions_send) + 1)]
        for attr in ext_joint_cmd_attrs:
            tag = f"{ext_joint_cmd_elem}.{attr}"
            ET.SubElement(
                elements_recv, "ELEMENT", TAG=tag, TYPE="DOUBLE", INDX=str(indx), HOLDON="1"
            )
            indx += 1

    # Optional command signals
    indx = _add_receive_signal(
        elements_recv,
        indx,
        cs_cfg,
        "velocities",
        _DEFAULT_VELOCITY_CMD_ELEMENT,
        [f"A{i}" for i in range(1, n_internal + 1)],
    )
    indx = _add_receive_signal(
        elements_recv,
        indx,
        cs_cfg,
        "ext_velocities",
        _DEFAULT_EXT_VELOCITY_CMD_ELEMENT,
        [f"E{i}" for i in range(1, len(ext_positions_send) + 1)],
    )
    indx = _add_receive_signal(
        elements_recv,
        indx,
        cs_cfg,
        "torques",
        _DEFAULT_TORQUE_CMD_ELEMENT,
        [f"A{i}" for i in range(1, n_internal + 1)],
    )
    indx = _add_receive_signal(
        elements_recv,
        indx,
        cs_cfg,
        "ext_torques",
        _DEFAULT_EXT_TORQUE_CMD_ELEMENT,
        [f"E{i}" for i in range(1, len(ext_positions_send) + 1)],
    )

    # GPIO command RECEIVE entries
    gpio_cmd_cfg = cs_cfg.get("gpio", None)
    if gpio_cmd_cfg is not None:
        if not isinstance(gpio_cmd_cfg, dict):
            raise ValueError("control_signal.gpio must be a map.")
        gpio_cmd_elem = gpio_cmd_cfg.get("xml_element", "GPIO")
        # Derive attribute list from motion_state gpio if not explicitly given
        gpio_cmd_attrs = gpio_cmd_cfg.get("xml_attributes", []) or gpio_state_attrs
        for attr in gpio_cmd_attrs:
            if not isinstance(attr, str) or not attr:
                raise ValueError("control_signal.gpio.xml_attributes contains invalid value.")
            tag = f"{gpio_cmd_elem}.{attr}"
            ET.SubElement(
                elements_recv, "ELEMENT", TAG=tag, TYPE="BOOL", INDX=str(indx), HOLDON="1"
            )
            indx += 1

    return root


def main() -> None:
    parser = argparse.ArgumentParser(
        description="Generate a KRC RSI ethernet XML config file from an abstract YAML config.",
        formatter_class=argparse.RawDescriptionHelpFormatter,
        epilog=__doc__,
    )
    parser.add_argument(
        "--config",
        "-c",
        required=True,
        help="Path to the RSI YAML config file (rsi_xml_config_example.yaml).",
    )
    parser.add_argument(
        "--client-ip",
        required=True,
        help="IP address of the ROS PC as seen from the KRC.",
    )
    parser.add_argument(
        "--client-port",
        type=int,
        default=59152,
        help="UDP port the driver listens on (default: 59152).",
    )
    parser.add_argument(
        "--output",
        "-o",
        default="rsi_ethernet.xml",
        help="Output file path (default: rsi_ethernet.xml).",
    )
    args = parser.parse_args()

    rsi_cfg = load_config(args.config)
    root_el = build_krc_xml(rsi_cfg, args.client_ip, args.client_port)

    pretty_xml = _prettify(root_el)
    lines = pretty_xml.splitlines()
    with open(args.output, "w") as fh:
        fh.write("\n".join(lines))
        fh.write("\n")

    print(f"Generated KRC RSI ethernet config: {args.output}")


if __name__ == "__main__":
    main()
