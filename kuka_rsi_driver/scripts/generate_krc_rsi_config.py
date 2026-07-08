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
- The SEND section always uses KRC built-in shortcuts (DEF_RIst, DEF_AIPos, DEF_EIPos,
  DEF_Delay) regardless of the element names specified in the YAML for those fields.  These
  shortcuts generate fixed XML element names on the KRC side.  If you need non-default
  element names for cartesian/joint data, a custom RSIX context file is required instead;
  see the KSS RSI documentation.
- GPIO state (SEND) and GPIO command (RECEIVE) entries are generated from the YAML config.

"""

import argparse
import sys
from xml.etree import ElementTree as ET
from xml.dom import minidom

try:
    import yaml
except ImportError:
    print("ERROR: PyYAML is not installed. Install with: pip install pyyaml", file=sys.stderr)
    sys.exit(1)

# Default legacy element names (match SDK defaults)
_DEFAULT_CARTESIAN_ELEMENT = "RIst"
_DEFAULT_JOINT_ELEMENT = "AIPos"
_DEFAULT_EXT_JOINT_ELEMENT = "EIPos"
_DEFAULT_DELAY_ELEMENT = "Delay"
_DEFAULT_JOINT_CMD_ELEMENT = "AK"
_DEFAULT_EXT_JOINT_CMD_ELEMENT = "EK"


def _prettify(elem: ET.Element) -> str:
    """Return a pretty-printed XML string for the Element."""
    raw = ET.tostring(elem, encoding="unicode")
    reparsed = minidom.parseString(raw)
    return reparsed.toprettyxml(indent="  ")


def _warn_non_default_element(name: str, configured: str, default: str) -> None:
    if configured != default:
        print(
            f'WARNING: {name} is set to "{configured}" (default: "{default}"). '
            f'The KRC DEF_ shortcut always produces element name "{default}". '
            f'To use "{configured}", configure a custom RSIX context on the KRC instead.',
            file=sys.stderr,
        )


def load_config(path: str) -> dict:
    with open(path) as fh:
        data = yaml.safe_load(fh)
    if not isinstance(data, dict) or "rsi_xml_config" not in data:
        raise ValueError(f"Config file '{path}' must contain a top-level 'rsi_xml_config' key.")
    return data.get("rsi_xml_config", {}) or {}


def build_krc_xml(
    rsi_cfg: dict,
    client_ip: str,
    client_port: int,
    sentype: str,
) -> ET.Element:
    """Build the ROOT element for the KRC ethernet XML."""
    root = ET.Element("ROOT")

    # ----- CONFIG -----
    config_el = ET.SubElement(root, "CONFIG")
    ET.SubElement(config_el, "IP_NUMBER").text = client_ip
    ET.SubElement(config_el, "PORT").text = str(client_port)
    ET.SubElement(config_el, "SENTYPE").text = sentype
    ET.SubElement(config_el, "ONLYSEND").text = "FALSE"

    ms_cfg = rsi_cfg.get("motion_state", {}) or {}
    cs_cfg = rsi_cfg.get("control_signal", {}) or {}

    # ----- SEND (KRC → PC) -----
    send_el = ET.SubElement(root, "SEND")
    elements_send = ET.SubElement(send_el, "ELEMENTS")

    # Warn if non-default element names are used for built-in fields
    cartesian_cfg = ms_cfg.get("cartesian", {}) or {}
    cartesian_elem = cartesian_cfg.get("xml_element", _DEFAULT_CARTESIAN_ELEMENT)
    _warn_non_default_element(
        "motion_state.cartesian.xml_element", cartesian_elem, _DEFAULT_CARTESIAN_ELEMENT
    )

    joints_cfg = ms_cfg.get("joints", []) or []
    if joints_cfg:
        # Check if any joint uses a non-default element name
        joint_elements = {j.get("xml_element", _DEFAULT_JOINT_ELEMENT) for j in joints_cfg}
        for jname in joint_elements:
            if jname != _DEFAULT_JOINT_ELEMENT:
                _warn_non_default_element(
                    "motion_state.joints[*].xml_element", jname, _DEFAULT_JOINT_ELEMENT
                )

        # Detect external axes
        ext_joints_send = [
            j for j in joints_cfg if j.get("xml_element", "") == _DEFAULT_EXT_JOINT_ELEMENT
        ]
        has_ext_joints_send = bool(ext_joints_send)
    else:
        has_ext_joints_send = False

    delay_cfg = ms_cfg.get("delay", {}) or {}
    delay_elem = delay_cfg.get("xml_element", _DEFAULT_DELAY_ELEMENT)
    _warn_non_default_element("motion_state.delay.xml_element", delay_elem, _DEFAULT_DELAY_ELEMENT)

    # DEF_ built-in SEND elements
    ET.SubElement(elements_send, "ELEMENT", TAG="DEF_RIst", TYPE="DOUBLE", INDX="INTERNAL")
    ET.SubElement(elements_send, "ELEMENT", TAG="DEF_AIPos", TYPE="DOUBLE", INDX="INTERNAL")
    if has_ext_joints_send:
        ET.SubElement(elements_send, "ELEMENT", TAG="DEF_EIPos", TYPE="DOUBLE", INDX="INTERNAL")
    ET.SubElement(elements_send, "ELEMENT", TAG="DEF_Delay", TYPE="LONG", INDX="INTERNAL")

    # GPIO state SEND entries
    gpio_state_cfg = ms_cfg.get("gpio", {}) or {}
    gpio_state_attrs = gpio_state_cfg.get("xml_attributes", []) or []
    gpio_state_elem = gpio_state_cfg.get("xml_element", "GPIO")
    for idx, attr in enumerate(gpio_state_attrs, start=1):
        tag = f"{gpio_state_elem}.{attr}"
        # Determine type from attribute name heuristic (BOOL by default)
        ET.SubElement(elements_send, "ELEMENT", TAG=tag, TYPE="BOOL", INDX=str(idx))

    # ----- RECEIVE (PC → KRC) -----
    recv_el = ET.SubElement(root, "RECEIVE")
    elements_recv = ET.SubElement(recv_el, "ELEMENTS")

    # Stop is always first
    ET.SubElement(elements_recv, "ELEMENT", TAG="Stop", TYPE="BOOL", INDX="1", HOLDON="0")

    indx = 2

    # Internal joint command entries
    joints_cmd_cfg = cs_cfg.get("joints", {}) or {}
    joint_cmd_elem = joints_cmd_cfg.get("xml_element", _DEFAULT_JOINT_CMD_ELEMENT)
    joint_cmd_attrs = joints_cmd_cfg.get("xml_attributes", []) or []

    if not joint_cmd_attrs:
        # Auto-generate from motion_state joint count or default to 6
        n_joints = len(joints_cfg) if joints_cfg else 6
        n_ext = (
            sum(1 for j in joints_cfg if j.get("xml_element", "") == _DEFAULT_EXT_JOINT_ELEMENT)
            if joints_cfg
            else 0
        )
        n_internal = n_joints - n_ext
        joint_cmd_attrs = [f"A{i}" for i in range(1, n_internal + 1)]

    for attr in joint_cmd_attrs:
        tag = f"{joint_cmd_elem}.{attr}"
        ET.SubElement(elements_recv, "ELEMENT", TAG=tag, TYPE="DOUBLE", INDX=str(indx), HOLDON="1")
        indx += 1

    # External joint command entries
    ext_joints_cmd_cfg = cs_cfg.get("ext_joints", None)
    if ext_joints_cmd_cfg is not None:
        ext_joint_cmd_elem = ext_joints_cmd_cfg.get("xml_element", _DEFAULT_EXT_JOINT_CMD_ELEMENT)
        ext_joint_cmd_attrs = ext_joints_cmd_cfg.get("xml_attributes", []) or []
        if not ext_joint_cmd_attrs:
            # Auto-generate from SEND ext joint count
            ext_joint_cmd_attrs = [f"E{i}" for i in range(1, len(ext_joints_send) + 1)]
        for attr in ext_joint_cmd_attrs:
            tag = f"{ext_joint_cmd_elem}.{attr}"
            ET.SubElement(
                elements_recv, "ELEMENT", TAG=tag, TYPE="DOUBLE", INDX=str(indx), HOLDON="1"
            )
            indx += 1

    # GPIO command RECEIVE entries
    gpio_cmd_cfg = cs_cfg.get("gpio", None)
    if gpio_cmd_cfg is not None:
        gpio_cmd_elem = gpio_cmd_cfg.get("xml_element", "GPIO")
        # Derive attribute list from motion_state gpio if not explicitly given
        gpio_cmd_attrs = gpio_cmd_cfg.get("xml_attributes", []) or gpio_state_attrs
        for attr in gpio_cmd_attrs:
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
        "--sentype",
        default="KROSHU",
        help="RSI SENTYPE value (default: KROSHU).",
    )
    parser.add_argument(
        "--output",
        "-o",
        default="rsi_ethernet.xml",
        help="Output file path (default: rsi_ethernet.xml).",
    )
    args = parser.parse_args()

    rsi_cfg = load_config(args.config)
    root_el = build_krc_xml(rsi_cfg, args.client_ip, args.client_port, args.sentype)

    pretty_xml = _prettify(root_el)
    # minidom adds an XML declaration; remove duplicate declarations if any
    lines = pretty_xml.splitlines()
    with open(args.output, "w") as fh:
        fh.write("\n".join(lines))
        fh.write("\n")

    print(f"Generated KRC RSI ethernet config: {args.output}")


if __name__ == "__main__":
    main()
