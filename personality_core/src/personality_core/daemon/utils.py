"""Utilities for the Personality Core daemon."""

import os
from typing import Any, List

from enum import Enum

import serial.tools.list_ports


def find_serial_port(
    vid: str = "0403",
    pid: str = "6014",
) -> list[str]:
    """Find serial ports for Personality Core based on USB VID and PID.

    Args:
        vid: Vendor ID of the device (default: ``"0403"``).
        pid: Product ID of the device (default: ``"6013"``).

    Returns:
        List of matching serial port device paths.

    """
    vid = vid.upper()
    pid = pid.upper()
    # for p in serial.tools.list_ports.comports():
    #     print(p.hwid)
    return [
        p.device
        for p in serial.tools.list_ports.comports()
        if f"USB VID:PID={vid}:{pid}" in p.hwid
    ]


def get_ip_address(ifname: str = "wlan0") -> str | None:
    """Get the IP address of a specific network interface."""
    import platform
    import socket
    import struct

    if platform.system() == "Linux":
        s = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
        try:
            import fcntl

            return socket.inet_ntoa(
                fcntl.ioctl(
                    s.fileno(),
                    0x8915,  # SIOCGIFADDR
                    struct.pack("256s", ifname[:15].encode("utf-8")),
                )[20:24]
            )
        except OSError:
            return None
    elif platform.system() == "Darwin":
        import subprocess

        try:
            result = subprocess.run(
                ["ipconfig", "getifaddr", "en0"],
                capture_output=True,
                text=True,
                check=True,
            )
            return result.stdout.strip() or None
        except (subprocess.CalledProcessError, FileNotFoundError):
            return None
    return None


def convert_enum_to_dict(data: List[Any]) -> dict[str, Any]:
    """Convert a dataclass containing Enums to a dictionary with enum values."""

    def convert_value(obj: Any) -> Any:
        if isinstance(obj, Enum):
            return obj.value
        return obj

    return dict((k, convert_value(v)) for k, v in data)
