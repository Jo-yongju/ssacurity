#!/usr/bin/env python3
"""PC reference tool for ssacurity UART Protocol V1."""

from __future__ import annotations

import argparse
import sys
import time
from dataclasses import dataclass


SOF = b"\xAA\x55"
PROTOCOL_VERSION = 0x01
MSG_DIAG_ECHO_REQUEST = 0xF0
MSG_DIAG_ECHO_RESPONSE = 0xF1
MAX_PAYLOAD = 64
MAX_ECHO_PAYLOAD = 32


def crc16_ccitt_false(data: bytes) -> int:
    crc = 0xFFFF
    for byte in data:
        crc ^= byte << 8
        for _ in range(8):
            if crc & 0x8000:
                crc = ((crc << 1) ^ 0x1021) & 0xFFFF
            else:
                crc = (crc << 1) & 0xFFFF
    return crc


def encode_frame(message_id: int, sequence: int, payload: bytes) -> bytes:
    if len(payload) > MAX_PAYLOAD:
        raise ValueError(f"payload exceeds {MAX_PAYLOAD} bytes")

    body = bytes(
        (
            PROTOCOL_VERSION,
            message_id & 0xFF,
            sequence & 0xFF,
            len(payload),
        )
    ) + payload
    crc = crc16_ccitt_false(body)
    return SOF + body + crc.to_bytes(2, "little")


@dataclass(frozen=True)
class Frame:
    version: int
    message_id: int
    sequence: int
    payload: bytes


def pop_frame(buffer: bytearray) -> Frame | None:
    while True:
        sof_index = buffer.find(SOF)
        if sof_index < 0:
            if buffer[-1:] == SOF[:1]:
                del buffer[:-1]
            else:
                buffer.clear()
            return None

        if sof_index:
            del buffer[:sof_index]

        if len(buffer) < 6:
            return None

        payload_length = buffer[5]
        if payload_length > MAX_PAYLOAD:
            del buffer[0]
            continue

        frame_length = 8 + payload_length
        if len(buffer) < frame_length:
            return None

        raw = bytes(buffer[:frame_length])
        del buffer[:frame_length]

        received_crc = int.from_bytes(raw[-2:], "little")
        calculated_crc = crc16_ccitt_false(raw[2:-2])
        if received_crc != calculated_crc:
            continue

        if raw[2] != PROTOCOL_VERSION:
            continue

        return Frame(
            version=raw[2],
            message_id=raw[3],
            sequence=raw[4],
            payload=raw[6:-2],
        )


def protocol_self_test() -> None:
    assert crc16_ccitt_false(b"123456789") == 0x29B1

    expected = bytes.fromhex("AA 55 01 F0 00 05 53 54 4D 33 32 DC A0")
    encoded = encode_frame(MSG_DIAG_ECHO_REQUEST, 0, b"STM32")
    assert encoded == expected, encoded.hex(" ")

    stream = bytearray(b"\x00\xFF" + encoded)
    parsed = pop_frame(stream)
    assert parsed is not None
    assert parsed.message_id == MSG_DIAG_ECHO_REQUEST
    assert parsed.sequence == 0
    assert parsed.payload == b"STM32"

    print("Protocol self-test: PASS")
    print(f"Golden echo request: {encoded.hex(' ').upper()}")


def require_pyserial():
    try:
        import serial
        from serial.tools import list_ports
    except ImportError as error:
        raise SystemExit(
            "pyserial is required for COM-port tests.\n"
            "Install it with: py -m pip install pyserial"
        ) from error
    return serial, list_ports


def list_serial_ports() -> None:
    _, list_ports = require_pyserial()
    ports = list(list_ports.comports())
    if not ports:
        print("No serial ports found.")
        return
    for port in ports:
        print(f"{port.device}: {port.description}")


def run_echo(port_name: str, text: str, timeout_s: float) -> None:
    serial, _ = require_pyserial()
    payload = text.encode("utf-8")
    if len(payload) > MAX_ECHO_PAYLOAD:
        raise SystemExit(
            f"UTF-8 payload is {len(payload)} bytes; maximum is "
            f"{MAX_ECHO_PAYLOAD} bytes."
        )

    request_sequence = 0
    request = encode_frame(
        MSG_DIAG_ECHO_REQUEST,
        request_sequence,
        payload,
    )

    receive_buffer = bytearray()
    deadline = time.monotonic() + timeout_s

    with serial.Serial(
        port=port_name,
        baudrate=115200,
        bytesize=serial.EIGHTBITS,
        parity=serial.PARITY_NONE,
        stopbits=serial.STOPBITS_ONE,
        timeout=0.05,
    ) as port:
        port.reset_input_buffer()
        port.write(request)
        port.flush()

        while time.monotonic() < deadline:
            chunk = port.read(max(port.in_waiting, 1))
            if chunk:
                receive_buffer.extend(chunk)

            while True:
                frame = pop_frame(receive_buffer)
                if frame is None:
                    break

                if (
                    frame.message_id == MSG_DIAG_ECHO_RESPONSE
                    and frame.sequence == request_sequence
                ):
                    if frame.payload != payload:
                        raise SystemExit(
                            "Echo payload mismatch: "
                            f"sent={payload!r}, received={frame.payload!r}"
                        )
                    print("Protocol echo: PASS")
                    print(f"TX: {request.hex(' ').upper()}")
                    response = encode_frame(
                        frame.message_id,
                        frame.sequence,
                        frame.payload,
                    )
                    print(f"RX: {response.hex(' ').upper()}")
                    return

    raise SystemExit(
        f"No valid DIAG_ECHO_RESPONSE received from {port_name} "
        f"within {timeout_s:.1f}s."
    )


def build_argument_parser() -> argparse.ArgumentParser:
    parser = argparse.ArgumentParser(
        description="Ssacurity STM32 UART Protocol V1 PC test tool"
    )
    subparsers = parser.add_subparsers(dest="command", required=True)

    subparsers.add_parser("self-test", help="verify CRC and golden frame")
    subparsers.add_parser("list", help="list available serial ports")

    echo_parser = subparsers.add_parser(
        "echo",
        help="send DIAG_ECHO_REQUEST and verify the response",
    )
    echo_parser.add_argument("--port", required=True, help="COM port, e.g. COM5")
    echo_parser.add_argument(
        "--text",
        default="STM32",
        help="UTF-8 text, at most 32 encoded bytes",
    )
    echo_parser.add_argument(
        "--timeout",
        type=float,
        default=2.0,
        help="response timeout in seconds",
    )
    return parser


def main() -> int:
    args = build_argument_parser().parse_args()

    if args.command == "self-test":
        protocol_self_test()
    elif args.command == "list":
        list_serial_ports()
    elif args.command == "echo":
        run_echo(args.port, args.text, args.timeout)
    else:
        raise AssertionError(args.command)

    return 0


if __name__ == "__main__":
    sys.exit(main())
