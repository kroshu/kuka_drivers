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

from dataclasses import dataclass, field
from datetime import datetime
import re
import signal
import statistics
import time
from typing import Dict, Optional

import rclpy
from rclpy.node import Node
from scapy.error import Scapy_Exception
from scapy.layers.inet import IP, UDP
from scapy.packet import Packet
from scapy.sendrecv import AsyncSniffer


IPOC_PATTERN = re.compile(rb"<IPOC>\s*([^<\s]+)\s*</IPOC>")


def _format_payload_sample(payload: bytes, max_len: int = 220) -> str:
    text = payload.decode("utf-8", errors="replace")
    contains_control_chars = any((ord(ch) < 32 and ch not in "\r\n\t") for ch in text)
    if contains_control_chars:
        return payload.hex(" ")
    compact = " ".join(text.split())
    if len(compact) > max_len:
        return compact[:max_len] + "..."
    return compact


def _format_timestamp_ns(timestamp_ns: Optional[int]) -> str:
    if timestamp_ns is None:
        return "n/a"
    return datetime.fromtimestamp(timestamp_ns / 1e9).isoformat(timespec="milliseconds")


def _extract_ipoc(payload: bytes) -> Optional[str]:
    match = IPOC_PATTERN.search(payload)
    if match is None:
        return None
    return match.group(1).decode("ascii", errors="ignore").strip()


def _parse_ipoc_int(ipoc: str) -> Optional[int]:
    try:
        return int(ipoc)
    except ValueError:
        return None


@dataclass
class _PacketRecord:
    payload: bytes
    source: str
    timestamp_ns: int


@dataclass
class _PortStats:
    label: str
    port: Optional[int]
    packet_count: int = 0
    total_bytes: int = 0
    first_packet_ns: Optional[int] = None
    last_packet_ns: Optional[int] = None
    inter_arrival_ns: list = field(default_factory=list)
    sample_message: Optional[str] = None
    sample_source: Optional[str] = None

    def record(self, payload: bytes, source: str, timestamp_ns: int) -> None:
        self.packet_count += 1
        self.total_bytes += len(payload)
        if self.first_packet_ns is None:
            self.first_packet_ns = timestamp_ns
        if self.last_packet_ns is not None:
            self.inter_arrival_ns.append(timestamp_ns - self.last_packet_ns)
        self.last_packet_ns = timestamp_ns
        if self.sample_message is None:
            self.sample_message = _format_payload_sample(payload)
            self.sample_source = source


class RsiMonitorNode(Node):
    def __init__(self) -> None:
        super().__init__("rsi_monitor_node")
        self.declare_parameter("rsi_port", 59152)

        self._rsi_port = int(self.get_parameter("rsi_port").value)
        self._validate_port(self._rsi_port, "rsi_port")

        self._running = True
        self._sender_port: Optional[int] = None
        self._matched_pairs = 0
        self._receive_to_send_latency_ns: list[int] = []
        self._send_to_receive_latency_ns: list[int] = []
        self._last_sender_timestamp_ns: Optional[int] = None
        self._last_sender_ipoc: Optional[int] = None
        self._pending_receiver: Dict[str, _PacketRecord] = {}
        self._pending_sender: Dict[str, _PacketRecord] = {}
        self._sniffer: Optional[AsyncSniffer] = None

        signal.signal(signal.SIGINT, self._handle_stop_signal)
        signal.signal(signal.SIGTERM, self._handle_stop_signal)

        self._stats = {
            "receiver": _PortStats("receiver", self._rsi_port),
            "sender": _PortStats("sender", None),
        }

        self.get_logger().info(
            f"UDP monitor started for RSI port {self._rsi_port} on all interfaces. "
            "Sender port will be auto-detected from sent RSI packets. "
            "Statistics include only packets with matching IPOC values."
        )

    @staticmethod
    def _validate_port(port: int, port_name: str) -> None:
        if port < 1 or port > 65535:
            raise ValueError(f"{port_name} must be in range [1, 65535], got {port}.")

    def _handle_stop_signal(self, _signum, _frame) -> None:
        self._running = False

    def _classify_packet(self, src_port: int, dst_port: int) -> Optional[str]:
        if dst_port == self._rsi_port:
            return "receiver"

        if self._sender_port is None:
            self._sender_port = src_port
            self._stats["sender"].port = src_port
            self.get_logger().info(f"Auto-detected sender port: {src_port}")
        return "sender"

    def _try_match_ipoc(self, ipoc: str) -> None:
        receiver_record = self._pending_receiver.get(ipoc)
        sender_record = self._pending_sender.get(ipoc)
        if receiver_record is None or sender_record is None:
            return

        self._stats["receiver"].record(
            receiver_record.payload, receiver_record.source, receiver_record.timestamp_ns
        )
        self._stats["sender"].record(
            sender_record.payload, sender_record.source, sender_record.timestamp_ns
        )
        latency_ns = sender_record.timestamp_ns - receiver_record.timestamp_ns
        if latency_ns >= 0:
            self._receive_to_send_latency_ns.append(latency_ns)
        self._matched_pairs += 1

        self._pending_receiver.pop(ipoc, None)
        self._pending_sender.pop(ipoc, None)

    def _handle_packet(self, packet: Packet) -> None:
        if not packet.haslayer(IP) or not packet.haslayer(UDP):
            return

        ip_layer = packet[IP]
        if ip_layer.version != 4:
            return

        udp_layer = packet[UDP]
        payload = bytes(udp_layer.payload)
        ipoc = _extract_ipoc(payload)
        if ipoc is None:
            return
        ipoc_int = _parse_ipoc_int(ipoc)

        direction = self._classify_packet(int(udp_layer.sport), int(udp_layer.dport))
        if direction is None:
            return

        now_ns = time.time_ns()
        if direction == "receiver":
            if self._last_sender_timestamp_ns is not None:
                if (
                    self._last_sender_ipoc is not None
                    and ipoc_int is not None
                    and ipoc_int == self._last_sender_ipoc + 1
                    and now_ns >= self._last_sender_timestamp_ns
                ):
                    self._send_to_receive_latency_ns.append(
                        now_ns - self._last_sender_timestamp_ns
                    )
                self._last_sender_timestamp_ns = None
                self._last_sender_ipoc = None
            if ipoc not in self._pending_receiver:
                self._pending_receiver[ipoc] = _PacketRecord(
                    payload=payload,
                    source=f"{ip_layer.src}:{udp_layer.sport}",
                    timestamp_ns=now_ns,
                )
        else:
            if ipoc in self._pending_receiver and ipoc not in self._pending_sender:
                self._last_sender_timestamp_ns = now_ns
                self._last_sender_ipoc = ipoc_int
                self._pending_sender[ipoc] = _PacketRecord(
                    payload=payload,
                    source=f"{ip_layer.src}:{udp_layer.sport}",
                    timestamp_ns=now_ns,
                )
        self._try_match_ipoc(ipoc)

    def run(self) -> None:
        try:
            self._sniffer = AsyncSniffer(
                iface=["lo", "loopback0", ""],
                filter="udp",
                prn=self._handle_packet,
                store=False,
            )
            self._sniffer.start()
        except (PermissionError, Scapy_Exception) as exc:
            raise RuntimeError(
                "Starting Scapy sniffer failed. UDP monitoring requires capture privileges "
                "(e.g. CAP_NET_RAW/CAP_NET_ADMIN or root)."
            ) from exc

        try:
            while self._running and rclpy.ok():
                time.sleep(0.2)
        finally:
            if self._sniffer is not None and self._sniffer.running:
                self._sniffer.stop()
            self._print_summary()

    def _print_summary(self) -> None:
        self.get_logger().info("UDP monitor stopped. Summary (matching IPOC packets only):")
        self.get_logger().info(f"Matched IPOC pairs: {self._matched_pairs}")
        if self._receive_to_send_latency_ns:
            min_latency_ms = min(self._receive_to_send_latency_ns) / 1e6
            max_latency_ms = max(self._receive_to_send_latency_ns) / 1e6
            avg_latency_ms = statistics.fmean(self._receive_to_send_latency_ns) / 1e6
            stdev_latency_ms = statistics.stdev(self._receive_to_send_latency_ns) / 1e6
            self.get_logger().info(
                "Response latency [ms] (receive->send, matching IPOC): "
                f"min={min_latency_ms:.3f} avg={avg_latency_ms:.3f} max={max_latency_ms:.3f} stdev={stdev_latency_ms:.3f}"
            )
        else:
            self.get_logger().info("Response latency [ms] (receive->send, matching IPOC): n/a")

        if self._send_to_receive_latency_ns:
            min_latency_ms = min(self._send_to_receive_latency_ns) / 1e6
            max_latency_ms = max(self._send_to_receive_latency_ns) / 1e6
            avg_latency_ms = statistics.fmean(self._send_to_receive_latency_ns) / 1e6
            stdev_latency_ms = statistics.stdev(self._send_to_receive_latency_ns) / 1e6
            self.get_logger().info(
                "Response latency [ms] (send->next receive with IPOC+1): "
                f"min={min_latency_ms:.3f} avg={avg_latency_ms:.3f} max={max_latency_ms:.3f} stdev={stdev_latency_ms:.3f}"
            )
        else:
            self.get_logger().info("Response latency [ms] (send->next receive with IPOC+1): n/a")
        self.get_logger().info(f"Unmatched receiver packets: {len(self._pending_receiver)}")
        self.get_logger().info(f"Unmatched sender packets: {len(self._pending_sender)}")

        for stream_name in ("receiver", "sender"):
            stats = self._stats[stream_name]
            port_text = str(stats.port) if stats.port is not None else "n/a"
            self.get_logger().info(
                f"{stats.label} stream (port {port_text}): packets={stats.packet_count}, "
                f"bytes={stats.total_bytes}"
            )

            if stats.packet_count == 0:
                self.get_logger().info(f"{stats.label} stream sample message: n/a")
                continue

            self.get_logger().info(
                f"{stats.label} stream first packet time: "
                f"{_format_timestamp_ns(stats.first_packet_ns)}"
            )
            self.get_logger().info(
                f"{stats.label} stream last packet time: "
                f"{_format_timestamp_ns(stats.last_packet_ns)}"
            )

            if stats.inter_arrival_ns:
                min_dt = min(stats.inter_arrival_ns) / 1e6
                max_dt = max(stats.inter_arrival_ns) / 1e6
                avg_dt = statistics.fmean(stats.inter_arrival_ns) / 1e6
                avg_hz = 1000.0 / avg_dt if avg_dt > 0.0 else 0.0
                self.get_logger().info(
                    f"{stats.label} stream inter-arrival [ms]: min={min_dt:.3f} "
                    f"avg={avg_dt:.3f} max={max_dt:.3f} (avg rate: {avg_hz:.2f} Hz)"
                )
            else:
                self.get_logger().info(
                    f"{stats.label} stream inter-arrival [ms]: n/a (received only one packet)"
                )

            self.get_logger().info(
                f"{stats.label} stream sample message (from {stats.sample_source}): "
                f"{stats.sample_message}"
            )


def main() -> None:
    rclpy.init()
    node = RsiMonitorNode()
    try:
        node.run()
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == "__main__":
    main()
