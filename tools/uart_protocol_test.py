#!/usr/bin/env python3
"""PC reference tool for ssacurity STM32 UART diagnostics."""

from __future__ import annotations

import argparse
import csv
import math
import struct
import sys
import time
from dataclasses import dataclass


SOF = b"\xAA\x55"
PROTOCOL_VERSION = 0x02
MOTOR_DIAGNOSTIC_PROTOCOL_VERSION = PROTOCOL_VERSION
SERVO_DIAGNOSTIC_PROTOCOL_VERSION = PROTOCOL_VERSION
MSG_CMD_DRIVE = 0x10
MSG_CMD_STOP = 0x11
MSG_TELEMETRY_DRIVE = 0x80
MSG_FAULT_EVENT = 0x81
MSG_LEGACY_TELEMETRY_ODOMETRY = 0x82
MSG_COMMAND_RESULT = 0x82
MSG_TELEMETRY_IMU = 0x83
MSG_TELEMETRY_RANGE = 0x84
MSG_TELEMETRY_ODOMETRY = 0x85
MSG_DIAG_ECHO_REQUEST = 0xF0
MSG_DIAG_ECHO_RESPONSE = 0xF1
MSG_DIAG_MOTOR_TEST_REQUEST = 0xF2
MSG_DIAG_MOTOR_TEST_RESPONSE = 0xF3
MSG_DIAG_PID_TEST_REQUEST = 0xF4
MSG_DIAG_PID_TEST_RESPONSE = 0xF5
MSG_DIAG_SERVO_REQUEST = 0xF6
MSG_DIAG_SERVO_RESPONSE = 0xF7
MAX_PAYLOAD = 64
MAX_ECHO_PAYLOAD = 31
MIN_MOTOR_TEST_PERCENT = 20.0
MAX_MOTOR_TEST_PERCENT = 95.0
MIN_MOTOR_TEST_DURATION_MS = 100
MAX_MOTOR_TEST_DURATION_MS = 10000
MAX_PID_TEST_SPEED_MM_S = 300
MIN_PID_TEST_DURATION_MS = 500
MAX_PID_TEST_DURATION_MS = 5000
ENCODER_COUNTS_PER_WHEEL_REV = 823.0
WHEEL_CIRCUMFERENCE_MM = 201.06
DRIVE_TELEMETRY_FORMAT = "<IhhhhhihBBI"
DRIVE_TELEMETRY_PAYLOAD_LENGTH = struct.calcsize(DRIVE_TELEMETRY_FORMAT)
FAULT_EVENT_FORMAT = "<IIIBB"
FAULT_EVENT_PAYLOAD_LENGTH = struct.calcsize(FAULT_EVENT_FORMAT)
RANGE_TELEMETRY_FORMAT = "<IHHHHB"
RANGE_TELEMETRY_PAYLOAD_LENGTH = struct.calcsize(RANGE_TELEMETRY_FORMAT)
IMU_TELEMETRY_FORMAT = "<IhhhhiiihhhiBBBB"
IMU_TELEMETRY_PAYLOAD_LENGTH = struct.calcsize(IMU_TELEMETRY_FORMAT)
ODOMETRY_TELEMETRY_FORMAT = "<IiiiihihiHBB"
ODOMETRY_TELEMETRY_PAYLOAD_LENGTH = struct.calcsize(
    ODOMETRY_TELEMETRY_FORMAT
)
MIN_SERVO_DIAGNOSTIC_PULSE_US = 766
MAX_SERVO_DIAGNOSTIC_PULSE_US = 1696
PC_DRIVE_MAX_ABS_SPEED_MM_S = 1565
PC_DRIVE_MIN_STEERING_DEG = -28.69
PC_DRIVE_MAX_STEERING_DEG = 19.55
DRIVE_HEARTBEAT_PERIOD_S = 0.05
DRIVE_TIMEOUT_OBSERVE_S = 0.65

ODOMETRY_STATUS_VALID = 1 << 0
ODOMETRY_STATUS_ENCODER_CALIBRATED = 1 << 1
ODOMETRY_STATUS_GEOMETRY_CALIBRATED = 1 << 2
ODOMETRY_STATUS_STEERING_ESTIMATED = 1 << 3
ODOMETRY_STATUS_IMU_FUSED = 1 << 4
ODOMETRY_STATUS_INPUT_INVALID = 1 << 5
ODOMETRY_STEERING_NONE = 0
ODOMETRY_STEERING_SENSOR = 1
ODOMETRY_STEERING_COMMAND_ESTIMATE = 2

IMU_STATUS_CONNECTED = 1 << 0
IMU_STATUS_GYRO_VALID = 1 << 1
IMU_STATUS_LINEAR_ACCEL_VALID = 1 << 2
IMU_STATUS_QUATERNION_VALID = 1 << 3
IMU_STATUS_STALE = 1 << 4
IMU_STATUS_SPI_ERROR = 1 << 5
IMU_STATUS_PROTOCOL_ERROR = 1 << 6

STATE_NAMES = {
    0: "BOOT",
    1: "SELF_TEST",
    2: "READY",
    3: "DRIVING",
    4: "SAFE_STOP",
    5: "FAULT",
    6: "ESTOP",
}

FAULT_NAMES = {
    0: "COMM_TIMEOUT",
    1: "CRC_ERROR",
    2: "BAD_COMMAND",
    3: "ENCODER_INVALID",
    4: "MOTOR_STALL",
    5: "DIRECTION",
    6: "CONTROL_OVERRUN",
    7: "IMU_LOST",
    8: "RANGE_LOST",
    9: "STEERING_INVALID",
    10: "ESTOP_ACTIVE",
    11: "INTERNAL",
    12: "COMMAND_LIMIT",
    13: "UART_RX_OVERFLOW",
    14: "UART_TX_ERROR",
    15: "SENSOR_STALE",
    16: "OBSTACLE_NEAR",
}


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


def encode_frame(
    message_id: int,
    sequence: int,
    payload: bytes,
    *,
    version: int = PROTOCOL_VERSION,
) -> bytes:
    if len(payload) > MAX_PAYLOAD:
        raise ValueError(f"payload exceeds {MAX_PAYLOAD} bytes")

    body = bytes(
        (
            version,
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


@dataclass(frozen=True)
class DriveTelemetry:
    mcu_time_ms: int
    target_speed_mm_s: int
    measured_speed_mm_s: int
    motor_duty_permille: int
    steering_cmd_cdeg: int
    steering_feedback_cdeg: int
    encoder_count: int
    yaw_cdeg: int
    state: int
    last_drive_seq: int
    active_fault_bits: int


@dataclass(frozen=True)
class FaultEvent:
    occurred_at_ms: int
    active_fault_bits: int
    latched_fault_bits: int
    action: int
    state: int


@dataclass(frozen=True)
class RangeTelemetry:
    mcu_time_ms: int
    front_left_mm: int
    front_right_mm: int
    rear_left_mm: int
    rear_right_mm: int
    valid_mask: int


@dataclass(frozen=True)
class ImuTelemetry:
    mcu_time_ms: int
    quaternion_i_q14: int
    quaternion_j_q14: int
    quaternion_k_q14: int
    quaternion_real_q14: int
    gyro_x_mdeg_s: int
    gyro_y_mdeg_s: int
    gyro_z_mdeg_s: int
    linear_accel_x_mm_s2: int
    linear_accel_y_mm_s2: int
    linear_accel_z_mm_s2: int
    yaw_mdeg: int
    gyro_accuracy: int
    linear_accel_accuracy: int
    quaternion_accuracy: int
    status_flags: int


@dataclass(frozen=True)
class LegacyOdometryTelemetry:
    x_mm: int
    y_mm: int
    yaw_mdeg: int
    yaw_rate_mdeg_s: int
    measured_wheel_steering_cdeg: int
    center_steering_cdeg: int
    status_flags: int
    steering_adc_raw: int
    uptime_ms: int


@dataclass(frozen=True)
class OdometryTelemetry:
    mcu_time_ms: int
    x_mm: int
    y_mm: int
    yaw_mdeg: int
    distance_mm: int
    linear_speed_mm_s: int
    yaw_rate_mdeg_s: int
    steering_cdeg: int
    curvature_micro_per_m: int
    status_flags: int
    steering_source: int
    last_drive_seq: int


def speed_mm_s_to_wheel_rpm(speed_mm_s: float) -> float:
    return speed_mm_s * 60.0 / WHEEL_CIRCUMFERENCE_MM


def encoder_delta_to_wheel_rpm(
    delta_count: int,
    elapsed_ms: int,
) -> float:
    if elapsed_ms <= 0:
        raise ValueError("elapsed_ms must be positive")

    return (
        float(delta_count)
        * 60000.0
        / (ENCODER_COUNTS_PER_WHEEL_REV * float(elapsed_ms))
    )


def encoder_delta_to_counts_per_revolution(
    delta_count: int,
    revolutions: int,
) -> float:
    if revolutions <= 0:
        raise ValueError("revolutions must be positive")

    return abs(float(delta_count)) / float(revolutions)


def pop_frame(
    buffer: bytearray,
    *,
    expected_version: int = PROTOCOL_VERSION,
) -> Frame | None:
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

        if raw[2] != expected_version:
            continue

        return Frame(
            version=raw[2],
            message_id=raw[3],
            sequence=raw[4],
            payload=raw[6:-2],
        )


def decode_drive_telemetry(frame: Frame) -> DriveTelemetry:
    if frame.message_id != MSG_TELEMETRY_DRIVE:
        raise ValueError("frame is not TELEMETRY_DRIVE")
    if len(frame.payload) != DRIVE_TELEMETRY_PAYLOAD_LENGTH:
        raise ValueError(
            f"TELEMETRY_DRIVE payload is {len(frame.payload)} bytes, "
            f"expected {DRIVE_TELEMETRY_PAYLOAD_LENGTH}"
        )

    values = struct.unpack(DRIVE_TELEMETRY_FORMAT, frame.payload)
    return DriveTelemetry(*values)


def decode_fault_event(frame: Frame) -> FaultEvent:
    if frame.message_id != MSG_FAULT_EVENT:
        raise ValueError("frame is not FAULT_EVENT")
    if len(frame.payload) != FAULT_EVENT_PAYLOAD_LENGTH:
        raise ValueError(
            f"FAULT_EVENT payload is {len(frame.payload)} bytes, "
            f"expected {FAULT_EVENT_PAYLOAD_LENGTH}"
        )

    return FaultEvent(*struct.unpack(FAULT_EVENT_FORMAT, frame.payload))


def decode_range_telemetry(frame: Frame) -> RangeTelemetry:
    if frame.message_id != MSG_TELEMETRY_RANGE:
        raise ValueError("frame is not TELEMETRY_RANGE")
    if len(frame.payload) != RANGE_TELEMETRY_PAYLOAD_LENGTH:
        raise ValueError(
            f"TELEMETRY_RANGE payload is {len(frame.payload)} bytes, "
            f"expected {RANGE_TELEMETRY_PAYLOAD_LENGTH}"
        )

    return RangeTelemetry(
        *struct.unpack(RANGE_TELEMETRY_FORMAT, frame.payload)
    )


def decode_imu_telemetry(frame: Frame) -> ImuTelemetry:
    if frame.message_id != MSG_TELEMETRY_IMU:
        raise ValueError("frame is not TELEMETRY_IMU")
    if len(frame.payload) != IMU_TELEMETRY_PAYLOAD_LENGTH:
        raise ValueError(
            f"TELEMETRY_IMU payload is {len(frame.payload)} bytes, "
            f"expected {IMU_TELEMETRY_PAYLOAD_LENGTH}"
        )

    return ImuTelemetry(*struct.unpack(IMU_TELEMETRY_FORMAT, frame.payload))


def decode_legacy_odometry_telemetry(
    frame: Frame,
) -> LegacyOdometryTelemetry:
    if frame.message_id != MSG_LEGACY_TELEMETRY_ODOMETRY:
        raise ValueError("frame is not TELEMETRY_ODOMETRY")
    if len(frame.payload) != 28:
        raise ValueError(
            f"TELEMETRY_ODOMETRY payload is {len(frame.payload)} bytes, "
            "expected 28"
        )

    values = struct.unpack("<iiiihhHHI", frame.payload)
    return LegacyOdometryTelemetry(*values)


def decode_odometry_telemetry(frame: Frame) -> OdometryTelemetry:
    if frame.message_id != MSG_TELEMETRY_ODOMETRY:
        raise ValueError("frame is not V2 TELEMETRY_ODOMETRY")
    if len(frame.payload) != ODOMETRY_TELEMETRY_PAYLOAD_LENGTH:
        raise ValueError(
            f"V2 TELEMETRY_ODOMETRY payload is {len(frame.payload)} bytes, "
            f"expected {ODOMETRY_TELEMETRY_PAYLOAD_LENGTH}"
        )

    return OdometryTelemetry(
        *struct.unpack(ODOMETRY_TELEMETRY_FORMAT, frame.payload)
    )


def protocol_self_test() -> None:
    assert crc16_ccitt_false(b"123456789") == 0x29B1
    assert abs(speed_mm_s_to_wheel_rpm(201.06) - 60.0) < 0.001
    assert (
        abs(
            encoder_delta_to_wheel_rpm(
                int(ENCODER_COUNTS_PER_WHEEL_REV),
                1000,
            )
            - 60.0
        )
        < 0.001
    )
    forward_counts_per_revolution = (
        encoder_delta_to_counts_per_revolution(2473, 3)
    )
    reverse_counts_per_revolution = (
        encoder_delta_to_counts_per_revolution(-2464, 3)
    )
    assert abs(forward_counts_per_revolution - 824.333333) < 0.001
    assert abs(reverse_counts_per_revolution - 821.333333) < 0.001
    assert (
        abs(
            (
                forward_counts_per_revolution
                + reverse_counts_per_revolution
            )
            / 2.0
            - 822.833333
        )
        < 0.001
    )

    expected = bytes.fromhex(
        "AA 55 02 F0 10 04 12 34 AA 55 CD 15"
    )
    encoded = encode_frame(
        MSG_DIAG_ECHO_REQUEST,
        0x10,
        bytes.fromhex("12 34 AA 55"),
    )
    assert encoded == expected, encoded.hex(" ")

    stream = bytearray(b"\x00\xFF" + encoded)
    parsed = pop_frame(stream)
    assert parsed is not None
    assert parsed.message_id == MSG_DIAG_ECHO_REQUEST
    assert parsed.sequence == 0x10
    assert parsed.payload == bytes.fromhex("12 34 AA 55")

    drive_frame = encode_frame(
        MSG_CMD_DRIVE,
        0x2A,
        struct.pack("<hhB", 500, -1250, 1),
    )
    assert drive_frame == bytes.fromhex(
        "AA 55 02 10 2A 05 F4 01 1E FB 01 84 7A"
    )

    motor_payload = struct.pack("<hH", 200, 1000)
    motor_frame = encode_frame(
        MSG_DIAG_MOTOR_TEST_REQUEST,
        0,
        motor_payload,
        version=MOTOR_DIAGNOSTIC_PROTOCOL_VERSION,
    )
    parsed_motor = pop_frame(
        bytearray(motor_frame),
        expected_version=MOTOR_DIAGNOSTIC_PROTOCOL_VERSION,
    )
    assert parsed_motor is not None
    assert parsed_motor.message_id == MSG_DIAG_MOTOR_TEST_REQUEST
    assert struct.unpack("<hH", parsed_motor.payload) == (200, 1000)

    motor_response = encode_frame(
        MSG_DIAG_MOTOR_TEST_RESPONSE,
        9,
        struct.pack("<BBhH", 0, 0, 200, 1000),
        version=MOTOR_DIAGNOSTIC_PROTOCOL_VERSION,
    )
    parsed_motor_response = pop_frame(
        bytearray(motor_response),
        expected_version=MOTOR_DIAGNOSTIC_PROTOCOL_VERSION,
    )
    assert parsed_motor_response is not None
    assert parsed_motor_response.message_id == MSG_DIAG_MOTOR_TEST_RESPONSE
    assert struct.unpack("<BBhH", parsed_motor_response.payload) == (
        0,
        0,
        200,
        1000,
    )

    servo_frame = encode_frame(
        MSG_DIAG_SERVO_REQUEST,
        2,
        struct.pack("<H", 1500),
        version=SERVO_DIAGNOSTIC_PROTOCOL_VERSION,
    )
    parsed_servo = pop_frame(
        bytearray(servo_frame),
        expected_version=SERVO_DIAGNOSTIC_PROTOCOL_VERSION,
    )
    assert parsed_servo is not None
    assert parsed_servo.message_id == MSG_DIAG_SERVO_REQUEST
    assert struct.unpack("<H", parsed_servo.payload) == (1500,)

    telemetry_payload = struct.pack(
        DRIVE_TELEMETRY_FORMAT,
        5000,
        200,
        180,
        250,
        0,
        0,
        -1234,
        0,
        3,
        7,
        0,
    )
    telemetry_frame = Frame(
        version=MOTOR_DIAGNOSTIC_PROTOCOL_VERSION,
        message_id=MSG_TELEMETRY_DRIVE,
        sequence=7,
        payload=telemetry_payload,
    )
    telemetry = decode_drive_telemetry(telemetry_frame)
    assert telemetry.encoder_count == -1234
    assert telemetry.motor_duty_permille == 250
    assert telemetry.mcu_time_ms == 5000

    fault_frame = Frame(
        version=PROTOCOL_VERSION,
        message_id=MSG_FAULT_EVENT,
        sequence=8,
        payload=struct.pack(
            FAULT_EVENT_FORMAT,
            5100,
            1 << 16,
            0,
            2,
            4,
        ),
    )
    fault_event = decode_fault_event(fault_frame)
    assert fault_event.occurred_at_ms == 5100
    assert fault_event.active_fault_bits == (1 << 16)
    assert fault_event.action == 2
    assert fault_event.state == 4

    range_frame = Frame(
        version=PROTOCOL_VERSION,
        message_id=MSG_TELEMETRY_RANGE,
        sequence=9,
        payload=struct.pack(
            RANGE_TELEMETRY_FORMAT,
            5200,
            350,
            0xFFFF,
            0xFFFF,
            0xFFFF,
            0x01,
        ),
    )
    range_telemetry = decode_range_telemetry(range_frame)
    assert range_telemetry.front_left_mm == 350
    assert range_telemetry.valid_mask == 0x01

    imu_frame = Frame(
        version=PROTOCOL_VERSION,
        message_id=MSG_TELEMETRY_IMU,
        sequence=10,
        payload=struct.pack(
            IMU_TELEMETRY_FORMAT,
            5250,
            0,
            0,
            11585,
            11585,
            0,
            0,
            90000,
            100,
            -200,
            50,
            45000,
            3,
            2,
            3,
            (
                IMU_STATUS_CONNECTED
                | IMU_STATUS_GYRO_VALID
                | IMU_STATUS_LINEAR_ACCEL_VALID
                | IMU_STATUS_QUATERNION_VALID
            ),
        ),
    )
    imu_telemetry = decode_imu_telemetry(imu_frame)
    assert imu_telemetry.gyro_z_mdeg_s == 90000
    assert imu_telemetry.yaw_mdeg == 45000
    assert imu_telemetry.status_flags & IMU_STATUS_GYRO_VALID
    roll, pitch, quaternion_yaw, quaternion_norm = (
        imu_quaternion_to_euler_degrees(imu_telemetry)
    )
    assert abs(roll) < 0.01
    assert abs(pitch) < 0.01
    assert abs(quaternion_yaw - 90.0) < 0.02
    assert abs(quaternion_norm - 1.0) < 0.001
    assert normalize_angle_degrees(270.0) == -90.0

    odometry_frame = Frame(
        version=PROTOCOL_VERSION,
        message_id=MSG_TELEMETRY_ODOMETRY,
        sequence=10,
        payload=struct.pack(
            ODOMETRY_TELEMETRY_FORMAT,
            5300,
            100,
            -20,
            1500,
            102,
            80,
            1000,
            1000,
            1300000,
            (
                ODOMETRY_STATUS_VALID
                | ODOMETRY_STATUS_ENCODER_CALIBRATED
                | ODOMETRY_STATUS_GEOMETRY_CALIBRATED
                | ODOMETRY_STATUS_STEERING_ESTIMATED
                | ODOMETRY_STATUS_IMU_FUSED
            ),
            ODOMETRY_STEERING_COMMAND_ESTIMATE,
            11,
        ),
    )
    odometry = decode_odometry_telemetry(odometry_frame)
    assert odometry.x_mm == 100
    assert odometry.y_mm == -20
    assert odometry.steering_cdeg == 1000
    assert odometry.steering_source == (
        ODOMETRY_STEERING_COMMAND_ESTIMATE
    )
    assert odometry.status_flags & ODOMETRY_STATUS_VALID
    assert odometry.status_flags & ODOMETRY_STATUS_IMU_FUSED

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


def wait_for_frame(
    port,
    receive_buffer: bytearray,
    deadline: float,
    message_id: int,
    sequence: int | None = None,
    *,
    expected_version: int = PROTOCOL_VERSION,
) -> Frame:
    while time.monotonic() < deadline:
        chunk = port.read(max(port.in_waiting, 1))
        if chunk:
            receive_buffer.extend(chunk)

        while True:
            frame = pop_frame(
                receive_buffer,
                expected_version=expected_version,
            )
            if frame is None:
                break
            if frame.message_id != message_id:
                continue
            if sequence is not None and frame.sequence != sequence:
                continue
            return frame

    raise TimeoutError(
        f"message 0x{message_id:02X} was not received before timeout"
    )


def validate_motor_test_values(percent: float, duration_ms: int) -> tuple[int, int]:
    duty_permille = round(percent * 10.0)

    if abs(percent) > MAX_MOTOR_TEST_PERCENT:
        raise SystemExit(
            f"Motor test is limited to +/-{MAX_MOTOR_TEST_PERCENT:.0f}%."
        )
    if duty_permille != 0 and abs(percent) < MIN_MOTOR_TEST_PERCENT:
        raise SystemExit(
            "Non-zero motor test is limited to 0% (stop) or "
            f"{MIN_MOTOR_TEST_PERCENT:.0f}~"
            f"{MAX_MOTOR_TEST_PERCENT:.0f}% in either direction."
        )
    if abs((duty_permille / 10.0) - percent) > 0.001:
        raise SystemExit("Percent must use at most one decimal place.")
    if duty_permille != 0 and not (
        MIN_MOTOR_TEST_DURATION_MS
        <= duration_ms
        <= MAX_MOTOR_TEST_DURATION_MS
    ):
        raise SystemExit(
            "Non-zero motor test duration must be between "
            f"{MIN_MOTOR_TEST_DURATION_MS} and "
            f"{MAX_MOTOR_TEST_DURATION_MS} ms."
        )

    if duty_permille == 0:
        duration_ms = 0

    return duty_permille, duration_ms


def send_motor_test_request(
    port,
    receive_buffer: bytearray,
    sequence: int,
    duty_permille: int,
    duration_ms: int,
    timeout_s: float,
) -> Frame:
    request = encode_frame(
        MSG_DIAG_MOTOR_TEST_REQUEST,
        sequence,
        struct.pack("<hH", duty_permille, duration_ms),
        version=MOTOR_DIAGNOSTIC_PROTOCOL_VERSION,
    )
    port.write(request)
    port.flush()

    try:
        response = wait_for_frame(
            port,
            receive_buffer,
            time.monotonic() + timeout_s,
            MSG_DIAG_MOTOR_TEST_RESPONSE,
            expected_version=MOTOR_DIAGNOSTIC_PROTOCOL_VERSION,
        )
    except TimeoutError as error:
        raise SystemExit(
            "No valid DIAG_MOTOR_TEST_RESPONSE received from STM32."
        ) from error

    if len(response.payload) != 6:
        raise SystemExit("Motor-test response has an invalid payload.")

    (
        response_request_sequence,
        status,
        accepted_permille,
        accepted_duration_ms,
    ) = struct.unpack(
        "<BBhH",
        response.payload,
    )
    if response_request_sequence != sequence:
        raise SystemExit(
            "Motor-test response does not match the request sequence."
        )
    if status != 0:
        raise SystemExit(
            "STM32 rejected motor-test: "
            f"{servo_result_name(status)}."
        )
    if (
        accepted_permille != duty_permille
        or accepted_duration_ms != duration_ms
    ):
        raise SystemExit(
            "Motor-test acknowledgement does not match the request."
        )

    return response


def send_pid_test_request(
    port,
    receive_buffer: bytearray,
    sequence: int,
    target_speed_mm_s: int,
    duration_ms: int,
    timeout_s: float,
) -> Frame:
    request = encode_frame(
        MSG_DIAG_PID_TEST_REQUEST,
        sequence,
        struct.pack("<hH", target_speed_mm_s, duration_ms),
    )
    port.write(request)
    port.flush()

    try:
        response = wait_for_frame(
            port,
            receive_buffer,
            time.monotonic() + timeout_s,
            MSG_DIAG_PID_TEST_RESPONSE,
            sequence,
        )
    except TimeoutError as error:
        raise SystemExit(
            "No valid DIAG_PID_TEST_RESPONSE received from STM32."
        ) from error

    if len(response.payload) != 5:
        raise SystemExit("PID-test response has an invalid payload.")

    status = response.payload[0]
    accepted_speed_mm_s, accepted_duration_ms = struct.unpack(
        "<hH", response.payload[1:]
    )
    if status == 1:
        raise SystemExit("STM32 rejected an invalid PID-test payload.")
    if status == 2:
        raise SystemExit(
            "STM32 rejected PID-test values outside its safety limits."
        )
    if status != 0:
        raise SystemExit(
            f"STM32 returned unknown PID-test status {status}."
        )
    if (
        accepted_speed_mm_s != target_speed_mm_s
        or accepted_duration_ms != duration_ms
    ):
        raise SystemExit(
            "PID-test acknowledgement does not match the request."
        )

    return response


def wait_for_drive_telemetry(
    port,
    receive_buffer: bytearray,
    timeout_s: float,
) -> DriveTelemetry:
    try:
        frame = wait_for_frame(
            port,
            receive_buffer,
            time.monotonic() + timeout_s,
            MSG_TELEMETRY_DRIVE,
            expected_version=MOTOR_DIAGNOSTIC_PROTOCOL_VERSION,
        )
    except TimeoutError as error:
        raise SystemExit(
            "No TELEMETRY_DRIVE received. Flash the firmware containing "
            "encoder diagnostic telemetry support."
        ) from error

    try:
        return decode_drive_telemetry(frame)
    except ValueError as error:
        raise SystemExit(str(error)) from error


def print_encoder_snapshot(telemetry: DriveTelemetry) -> None:
    print(f"Encoder count: {telemetry.encoder_count:+d}")
    print(
        "Motor output: "
        f"{telemetry.motor_duty_permille / 10.0:+.1f}%"
    )
    print(f"Measured speed: {telemetry.measured_speed_mm_s:+d} mm/s")
    print(f"Fault bits: 0x{telemetry.active_fault_bits:08X}")
    print(f"STM32 uptime: {telemetry.mcu_time_ms} ms")
    print(
        "PID readiness: this project keeps speed PID blocked until "
        "counts/rev and wheel circumference are explicitly calibrated."
    )


def state_name(state: int) -> str:
    return STATE_NAMES.get(state, f"UNKNOWN({state})")


def fault_names(bits: int) -> str:
    names = [
        name
        for bit, name in FAULT_NAMES.items()
        if bits & (1 << bit)
    ]
    return ", ".join(names) if names else "none"


def read_available_frames(port, receive_buffer: bytearray) -> list[Frame]:
    frames: list[Frame] = []
    chunk = port.read(max(port.in_waiting, 1))
    if chunk:
        receive_buffer.extend(chunk)

    while True:
        frame = pop_frame(receive_buffer)
        if frame is None:
            break
        frames.append(frame)
    return frames


def encode_drive_command(
    sequence: int,
    speed_mm_s: int,
    steering_cdeg: int,
    drive_enable: bool,
) -> bytes:
    return encode_frame(
        MSG_CMD_DRIVE,
        sequence,
        struct.pack(
            "<hhB",
            speed_mm_s,
            steering_cdeg,
            int(drive_enable),
        ),
    )


def validate_drive_values(
    speed_mm_s: int,
    steering_deg: float,
    duration_s: float,
) -> int:
    steering_cdeg = round(steering_deg * 100.0)

    if abs(speed_mm_s) > PC_DRIVE_MAX_ABS_SPEED_MM_S:
        raise SystemExit(
            "PC integration tests are limited to "
            f"+/-{PC_DRIVE_MAX_ABS_SPEED_MM_S} mm/s."
        )
    if not (
        PC_DRIVE_MIN_STEERING_DEG
        <= steering_deg
        <= PC_DRIVE_MAX_STEERING_DEG
    ):
        raise SystemExit(
            "PC integration tests are limited to the calibrated steering "
            f"range {PC_DRIVE_MIN_STEERING_DEG:.2f} to "
            f"+{PC_DRIVE_MAX_STEERING_DEG:.2f} degrees."
        )
    if abs((steering_cdeg / 100.0) - steering_deg) > 0.0001:
        raise SystemExit("Steering angle must use at most 0.01 degree.")
    if not 0.5 <= duration_s <= 5.0:
        raise SystemExit("Drive duration must be from 0.5 to 5.0 seconds.")
    return steering_cdeg


def stream_drive_command(
    port,
    receive_buffer: bytearray,
    next_sequence: int,
    speed_mm_s: int,
    steering_cdeg: int,
    drive_enable: bool,
    duration_s: float,
    label: str,
    odometry_samples: list[OdometryTelemetry] | None = None,
) -> tuple[int, list[DriveTelemetry], set[int]]:
    deadline = time.monotonic() + duration_s
    next_transmit = 0.0
    next_print = 0.0
    samples: list[DriveTelemetry] = []
    sent_sequences: set[int] = set()

    print(
        f"[{label}] speed={speed_mm_s:+d} mm/s, "
        f"steering={steering_cdeg / 100.0:+.2f} deg, "
        f"enable={int(drive_enable)}"
    )

    while time.monotonic() < deadline:
        now = time.monotonic()
        if now >= next_transmit:
            request = encode_drive_command(
                next_sequence,
                speed_mm_s,
                steering_cdeg,
                drive_enable,
            )
            port.write(request)
            sent_sequences.add(next_sequence)
            next_sequence = (next_sequence + 1) & 0xFF
            next_transmit += DRIVE_HEARTBEAT_PERIOD_S
            if next_transmit <= now:
                next_transmit = now + DRIVE_HEARTBEAT_PERIOD_S

        for frame in read_available_frames(port, receive_buffer):
            if frame.message_id == MSG_TELEMETRY_ODOMETRY:
                if odometry_samples is not None:
                    odometry_samples.append(
                        decode_odometry_telemetry(frame)
                    )
                continue
            if frame.message_id != MSG_TELEMETRY_DRIVE:
                continue
            try:
                telemetry = decode_drive_telemetry(frame)
            except ValueError:
                continue
            samples.append(telemetry)

            if now >= next_print:
                print(
                    "  "
                    f"state={state_name(telemetry.state):9s} "
                    f"seq={telemetry.last_drive_seq:3d} "
                    f"target={telemetry.target_speed_mm_s:+4d} "
                    f"measured={telemetry.measured_speed_mm_s:+4d} mm/s "
                    f"duty={telemetry.motor_duty_permille / 10.0:+5.1f}% "
                    f"steer={telemetry.steering_cmd_cdeg / 100.0:+6.2f} deg "
                    f"enc={telemetry.encoder_count:+d} "
                    f"faults={fault_names(telemetry.active_fault_bits)}"
                )
                next_print = now + 0.25

    port.flush()
    return next_sequence, samples, sent_sequences


def verify_drive_step(
    samples: list[DriveTelemetry],
    sent_sequences: set[int],
    speed_mm_s: int,
    steering_cdeg: int,
    label: str,
) -> None:
    accepted = [
        sample
        for sample in samples
        if sample.last_drive_seq in sent_sequences
        and sample.target_speed_mm_s == speed_mm_s
        and sample.steering_cmd_cdeg == steering_cdeg
    ]
    if not accepted:
        latest = samples[-1] if samples else None
        if latest is None:
            raise SystemExit(
                f"{label}: FAIL - no TELEMETRY_DRIVE was received."
            )
        raise SystemExit(
            f"{label}: FAIL - CMD_DRIVE was not reflected in telemetry. "
            f"state={state_name(latest.state)}, "
            f"faults={fault_names(latest.active_fault_bits)}"
        )

    if not any(sample.state == 3 for sample in accepted):
        latest = accepted[-1]
        raise SystemExit(
            f"{label}: FAIL - command was received but STM32 did not enter "
            f"DRIVING. state={state_name(latest.state)}, "
            f"faults={fault_names(latest.active_fault_bits)}. "
            "Keep the HC-SR04 facing at least 30 cm of clear space."
        )

    if speed_mm_s != 0:
        if not any(sample.motor_duty_permille != 0 for sample in accepted):
            latest = accepted[-1]
            raise SystemExit(
                f"{label}: FAIL - target was accepted but motor duty stayed "
                f"zero. faults={fault_names(latest.active_fault_bits)}"
            )

        count_delta = (
            accepted[-1].encoder_count - accepted[0].encoder_count
        )
        if count_delta == 0:
            raise SystemExit(
                f"{label}: FAIL - no encoder count change was detected."
            )
        if (count_delta > 0) != (speed_mm_s > 0):
            raise SystemExit(
                f"{label}: FAIL - encoder direction is reversed "
                f"(delta={count_delta:+d})."
            )
        print(
            f"[{label}] PASS - command accepted, motor active, "
            f"encoder delta={count_delta:+d}"
        )
    else:
        if any(sample.motor_duty_permille != 0 for sample in accepted):
            raise SystemExit(
                f"{label}: FAIL - zero-speed steering command produced "
                "non-zero motor duty."
            )
        print(
            f"[{label}] PASS - steering command accepted with motor stopped"
        )


def verify_ready(
    samples: list[DriveTelemetry],
    sent_sequences: set[int],
) -> None:
    accepted = [
        sample
        for sample in samples
        if sample.last_drive_seq in sent_sequences
        and sample.target_speed_mm_s == 0
        and sample.steering_cmd_cdeg == 0
    ]
    if accepted and any(sample.state == 2 for sample in accepted):
        print("[rearm] PASS - neutral command accepted, state=READY")
        return

    latest = samples[-1] if samples else None
    if latest is None:
        raise SystemExit(
            "Rearm failed: no TELEMETRY_DRIVE received from STM32."
        )
    raise SystemExit(
        "Rearm failed: STM32 did not reach READY. "
        f"state={state_name(latest.state)}, "
        f"faults={fault_names(latest.active_fault_bits)}. "
        "Check firmware version, HC-SR04 clearance, encoder/servo init, "
        "and reset the board after flashing."
    )


def observe_watchdog_stop(
    port,
    receive_buffer: bytearray,
) -> DriveTelemetry:
    print(
        "[watchdog] Deliberately stopping CMD_DRIVE heartbeat for "
        f"{DRIVE_TIMEOUT_OBSERVE_S:.2f} s"
    )
    deadline = time.monotonic() + DRIVE_TIMEOUT_OBSERVE_S
    samples: list[DriveTelemetry] = []
    fault_events: list[FaultEvent] = []

    while time.monotonic() < deadline:
        for frame in read_available_frames(port, receive_buffer):
            if frame.message_id == MSG_TELEMETRY_DRIVE:
                samples.append(decode_drive_telemetry(frame))
            elif frame.message_id == MSG_FAULT_EVENT:
                fault_events.append(decode_fault_event(frame))

    stopped = [
        sample
        for sample in samples
        if sample.state == 4
        and sample.motor_duty_permille == 0
        and (sample.active_fault_bits & (1 << 0))
    ]
    if not stopped:
        latest = samples[-1] if samples else None
        details = "no telemetry" if latest is None else (
            f"state={state_name(latest.state)}, "
            f"duty={latest.motor_duty_permille / 10.0:+.1f}%, "
            f"faults={fault_names(latest.active_fault_bits)}"
        )
        raise SystemExit(
            "Watchdog test: FAIL - 300 ms link-loss stop was not verified "
            f"({details})."
        )

    timeout_events = [
        event
        for event in fault_events
        if event.state == 4
        and event.action == 2
        and (event.active_fault_bits & (1 << 0))
    ]
    if not timeout_events:
        raise SystemExit(
            "Watchdog test: FAIL - TELEMETRY_DRIVE reported the stop, but "
            "the Jetson-facing FAULT_EVENT with COMM_TIMEOUT was not "
            "received."
        )

    result = stopped[-1]
    print(
        "[watchdog] PASS - heartbeat loss produced SAFE_STOP, "
        "COMM_TIMEOUT, 0.0% motor duty, and a FAULT_EVENT"
    )
    return result


def send_stop_command(
    port,
    receive_buffer: bytearray,
    sequence: int,
    timeout_s: float,
) -> int:
    request = encode_frame(MSG_CMD_STOP, sequence, b"\x00")
    port.write(request)
    port.flush()
    deadline = time.monotonic() + timeout_s

    while time.monotonic() < deadline:
        for frame in read_available_frames(port, receive_buffer):
            if (
                frame.message_id == MSG_COMMAND_RESULT
                and len(frame.payload) == 4
            ):
                request_id, request_sequence, result, state = struct.unpack(
                    "<BBBB",
                    frame.payload,
                )
                if (
                    request_id == MSG_CMD_STOP
                    and request_sequence == sequence
                ):
                    if result != 0:
                        raise SystemExit(
                            "CMD_STOP rejected: "
                            f"{servo_result_name(result)}, "
                            f"state={state_name(state)}"
                        )
                    print(
                        "[stop] PASS - CMD_STOP accepted, "
                        f"state={state_name(state)}"
                    )
                    return (sequence + 1) & 0xFF

    raise SystemExit("CMD_STOP response was not received.")


def run_drive_test(
    port_name: str,
    speed_mm_s: int,
    steering_deg: float,
    duration_s: float,
    timeout_s: float,
    wheels_off_ground: bool,
    verify_watchdog: bool,
) -> None:
    if not wheels_off_ground:
        raise SystemExit(
            "Refusing to move the vehicle. Turn it upside down or otherwise "
            "lift all driven wheels, then add --wheels-off-ground."
        )
    steering_cdeg = validate_drive_values(
        speed_mm_s,
        steering_deg,
        duration_s,
    )
    serial, _ = require_pyserial()
    receive_buffer = bytearray()
    next_sequence = 0

    with serial.Serial(
        port=port_name,
        baudrate=115200,
        bytesize=serial.EIGHTBITS,
        parity=serial.PARITY_NONE,
        stopbits=serial.STOPBITS_ONE,
        timeout=0.02,
    ) as port:
        port.reset_input_buffer()
        time.sleep(0.4)
        print(f"Port: {port_name} (115200 8N1, Protocol V2)")
        print("Safety: wheels off ground; HC-SR04 clearance >= 30 cm")

        try:
            (
                next_sequence,
                neutral_samples,
                neutral_sequences,
            ) = stream_drive_command(
                port,
                receive_buffer,
                next_sequence,
                0,
                0,
                False,
                0.8,
                "rearm",
            )
            verify_ready(neutral_samples, neutral_sequences)

            next_sequence, samples, sent_sequences = (
                stream_drive_command(
                    port,
                    receive_buffer,
                    next_sequence,
                    speed_mm_s,
                    steering_cdeg,
                    True,
                    duration_s,
                    "drive",
                )
            )
            verify_drive_step(
                samples,
                sent_sequences,
                speed_mm_s,
                steering_cdeg,
                "drive",
            )

            if verify_watchdog and speed_mm_s != 0:
                observe_watchdog_stop(port, receive_buffer)
                (
                    next_sequence,
                    neutral_samples,
                    neutral_sequences,
                ) = stream_drive_command(
                    port,
                    receive_buffer,
                    next_sequence,
                    0,
                    0,
                    False,
                    0.8,
                    "watchdog-rearm",
                )
                verify_ready(neutral_samples, neutral_sequences)
            else:
                next_sequence, _, _ = stream_drive_command(
                    port,
                    receive_buffer,
                    next_sequence,
                    0,
                    0,
                    False,
                    0.4,
                    "neutral",
                )

            next_sequence = send_stop_command(
                port,
                receive_buffer,
                next_sequence,
                timeout_s,
            )
        except BaseException:
            try:
                request = encode_frame(
                    MSG_CMD_STOP,
                    next_sequence,
                    b"\x00",
                )
                port.write(request)
                port.flush()
            except Exception:
                pass
            raise

    print("PC-as-Jetson drive test: PASS")


def run_drive_scenario(
    port_name: str,
    timeout_s: float,
    wheels_off_ground: bool,
    profile: str,
) -> None:
    if not wheels_off_ground:
        raise SystemExit(
            "Refusing to run the scenario. Lift all driven wheels and add "
            "--wheels-off-ground."
        )

    serial, _ = require_pyserial()
    receive_buffer = bytearray()
    next_sequence = 0
    if profile == "basic":
        steps = (
            ("forward-straight", 100, 0, 1.5),
            ("steering-left", 80, 500, 1.5),
            ("steering-right", 80, -500, 1.5),
            ("reverse-straight", -80, 0, 1.5),
        )
    elif profile == "extended":
        steps = (
            ("forward-slow", 50, 0, 1.0),
            ("forward-full", 100, 0, 1.0),
            ("forward-left-2.5", 80, 250, 1.0),
            ("forward-left-5", 80, 500, 1.0),
            ("forward-left-10", 80, 1000, 1.0),
            ("forward-right-2.5", 80, -250, 1.0),
            ("forward-right-5", 80, -500, 1.0),
            ("forward-right-10", 80, -1000, 1.0),
            ("reverse-straight", -80, 0, 1.0),
            ("reverse-steer-positive", -60, 500, 1.0),
            ("reverse-steer-negative", -60, -500, 1.0),
        )
    else:
        raise AssertionError(profile)

    with serial.Serial(
        port=port_name,
        baudrate=115200,
        bytesize=serial.EIGHTBITS,
        parity=serial.PARITY_NONE,
        stopbits=serial.STOPBITS_ONE,
        timeout=0.02,
    ) as port:
        port.reset_input_buffer()
        time.sleep(0.4)
        print(f"Port: {port_name} (115200 8N1, Protocol V2)")
        print(
            f"Scenario profile: {profile} "
            "(each motion step returns to neutral)"
        )

        try:
            (
                next_sequence,
                neutral_samples,
                neutral_sequences,
            ) = stream_drive_command(
                port,
                receive_buffer,
                next_sequence,
                0,
                0,
                False,
                0.8,
                "rearm",
            )
            verify_ready(neutral_samples, neutral_sequences)

            for label, speed, steering, duration in steps:
                next_sequence, samples, sent_sequences = (
                    stream_drive_command(
                        port,
                        receive_buffer,
                        next_sequence,
                        speed,
                        steering,
                        True,
                        duration,
                        label,
                    )
                )
                verify_drive_step(
                    samples,
                    sent_sequences,
                    speed,
                    steering,
                    label,
                )
                next_sequence, _, _ = stream_drive_command(
                    port,
                    receive_buffer,
                    next_sequence,
                    0,
                    0,
                    False,
                    0.4,
                    f"{label}-neutral",
                )

            next_sequence, samples, sent_sequences = (
                stream_drive_command(
                    port,
                    receive_buffer,
                    next_sequence,
                    80,
                    0,
                    True,
                    0.8,
                    "watchdog-drive",
                )
            )
            verify_drive_step(
                samples,
                sent_sequences,
                80,
                0,
                "watchdog-drive",
            )
            observe_watchdog_stop(port, receive_buffer)

            (
                next_sequence,
                neutral_samples,
                neutral_sequences,
            ) = stream_drive_command(
                port,
                receive_buffer,
                next_sequence,
                0,
                0,
                False,
                0.8,
                "final-rearm",
            )
            verify_ready(neutral_samples, neutral_sequences)
            next_sequence = send_stop_command(
                port,
                receive_buffer,
                next_sequence,
                timeout_s,
            )
        except BaseException:
            try:
                port.write(
                    encode_frame(
                        MSG_CMD_STOP,
                        next_sequence,
                        b"\x00",
                    )
                )
                port.flush()
            except Exception:
                pass
            raise

    print("PC-as-Jetson full scenario: PASS")


def run_steering_command_sequence(
    port_name: str,
    timeout_s: float,
    wheels_off_ground: bool,
    hold_seconds: float,
    steering_sequence_cdeg: tuple[int, ...],
    description: str,
    success_message: str,
) -> None:
    if not wheels_off_ground:
        raise SystemExit(
            "Refusing to move the steering linkage. Lift all driven wheels "
            "and add --wheels-off-ground."
        )
    if not 0.5 <= hold_seconds <= 5.0:
        raise SystemExit(
            "Steering hold duration must be from 0.5 to 5.0 seconds."
        )

    serial, _ = require_pyserial()
    receive_buffer = bytearray()
    next_sequence = 0

    with serial.Serial(
        port=port_name,
        baudrate=115200,
        bytesize=serial.EIGHTBITS,
        parity=serial.PARITY_NONE,
        stopbits=serial.STOPBITS_ONE,
        timeout=0.02,
    ) as port:
        port.reset_input_buffer()
        time.sleep(0.4)
        print(f"Port: {port_name} (115200 8N1, Protocol V2)")
        print(description)
        print("Motor duty must remain at 0.0%.")
        print(
            "Steering is command-estimated from the calibrated seven-point "
            "angle-to-PWM LUT; no physical steering sensor is mounted."
        )

        try:
            (
                next_sequence,
                neutral_samples,
                neutral_sequences,
            ) = stream_drive_command(
                port,
                receive_buffer,
                next_sequence,
                0,
                0,
                False,
                0.8,
                "rearm",
            )
            verify_ready(neutral_samples, neutral_sequences)

            for index, steering_cdeg in enumerate(
                steering_sequence_cdeg
            ):
                label = (
                    f"sweep-{index + 1:02d}-"
                    f"{steering_cdeg / 100.0:+.1f}deg"
                )
                next_sequence, samples, sent_sequences = (
                    stream_drive_command(
                        port,
                        receive_buffer,
                        next_sequence,
                        0,
                        steering_cdeg,
                        True,
                        hold_seconds,
                        label,
                    )
                )
                verify_drive_step(
                    samples,
                    sent_sequences,
                    0,
                    steering_cdeg,
                    label,
                )

            (
                next_sequence,
                neutral_samples,
                neutral_sequences,
            ) = stream_drive_command(
                port,
                receive_buffer,
                next_sequence,
                0,
                0,
                False,
                0.5,
                "final-neutral",
            )
            verify_ready(neutral_samples, neutral_sequences)
            next_sequence = send_stop_command(
                port,
                receive_buffer,
                next_sequence,
                timeout_s,
            )
        except BaseException:
            try:
                port.write(
                    encode_frame(
                        MSG_CMD_STOP,
                        next_sequence,
                        b"\x00",
                    )
                )
                port.flush()
            except Exception:
                pass
            raise

    print(success_message)


def run_steering_sweep(
    port_name: str,
    timeout_s: float,
    wheels_off_ground: bool,
    hold_seconds: float,
) -> None:
    run_steering_command_sequence(
        port_name,
        timeout_s,
        wheels_off_ground,
        hold_seconds,
        (
            0,
            1027,
            1418,
            1955,
            1418,
            1027,
            0,
            -709,
            -1756,
            -2869,
            -1756,
            -709,
            0,
        ),
        (
            "Full LUT sweep: 0 -> +10.27 -> +14.18 -> +19.55 -> 0 "
            "-> -7.09 -> -17.56 -> -28.69 -> 0 deg"
        ),
        "Full LUT steering sweep: PASS",
    )


def run_steering_full_sweep(
    port_name: str,
    timeout_s: float,
    wheels_off_ground: bool,
    hold_seconds: float,
    cycles: int,
) -> None:
    if not 1 <= cycles <= 10:
        raise SystemExit("Full steering cycles must be from 1 to 10.")

    sequence = [0]
    for _ in range(cycles):
        sequence.extend((1955, -2869))
    sequence.append(0)
    run_steering_command_sequence(
        port_name,
        timeout_s,
        wheels_off_ground,
        hold_seconds,
        tuple(sequence),
        (
            f"Endpoint sweep: +19.55 deg (766 us) <-> "
            f"-28.69 deg (1696 us), {cycles} cycle(s)"
        ),
        "Full endpoint steering sweep: PASS",
    )


def format_range_channel(
    value_mm: int,
    valid_mask: int,
    bit: int,
) -> str:
    if (valid_mask & (1 << bit)) == 0:
        return "invalid"
    return f"{value_mm} mm"


def odometry_steering_source_name(source: int) -> str:
    names = {
        ODOMETRY_STEERING_NONE: "NONE",
        ODOMETRY_STEERING_SENSOR: "SENSOR",
        ODOMETRY_STEERING_COMMAND_ESTIMATE: "COMMAND_ESTIMATE",
    }
    return names.get(source, f"UNKNOWN({source})")


def odometry_status_names(flags: int) -> str:
    names = []
    if flags & ODOMETRY_STATUS_VALID:
        names.append("VALID")
    if flags & ODOMETRY_STATUS_ENCODER_CALIBRATED:
        names.append("ENCODER_CALIBRATED")
    if flags & ODOMETRY_STATUS_GEOMETRY_CALIBRATED:
        names.append("GEOMETRY_CALIBRATED")
    if flags & ODOMETRY_STATUS_STEERING_ESTIMATED:
        names.append("STEERING_ESTIMATED")
    if flags & ODOMETRY_STATUS_IMU_FUSED:
        names.append("IMU_FUSED")
    if flags & ODOMETRY_STATUS_INPUT_INVALID:
        names.append("INPUT_INVALID")
    return ", ".join(names) if names else "none"


def imu_status_names(flags: int) -> str:
    names = []
    if flags & IMU_STATUS_CONNECTED:
        names.append("CONNECTED")
    if flags & IMU_STATUS_GYRO_VALID:
        names.append("GYRO_VALID")
    if flags & IMU_STATUS_LINEAR_ACCEL_VALID:
        names.append("LINEAR_ACCEL_VALID")
    if flags & IMU_STATUS_QUATERNION_VALID:
        names.append("QUATERNION_VALID")
    if flags & IMU_STATUS_STALE:
        names.append("STALE")
    if flags & IMU_STATUS_SPI_ERROR:
        names.append("SPI_ERROR")
    if flags & IMU_STATUS_PROTOCOL_ERROR:
        names.append("PROTOCOL_ERROR")
    return ", ".join(names) if names else "none"


def normalize_angle_degrees(angle_degrees: float) -> float:
    return ((angle_degrees + 180.0) % 360.0) - 180.0


def imu_quaternion_to_euler_degrees(
    imu: ImuTelemetry,
) -> tuple[float, float, float, float]:
    quaternion_i = imu.quaternion_i_q14 / 16384.0
    quaternion_j = imu.quaternion_j_q14 / 16384.0
    quaternion_k = imu.quaternion_k_q14 / 16384.0
    quaternion_real = imu.quaternion_real_q14 / 16384.0
    quaternion_norm = math.sqrt(
        quaternion_i * quaternion_i
        + quaternion_j * quaternion_j
        + quaternion_k * quaternion_k
        + quaternion_real * quaternion_real
    )
    if quaternion_norm <= 1.0e-9:
        nan = float("nan")
        return nan, nan, nan, quaternion_norm

    quaternion_i /= quaternion_norm
    quaternion_j /= quaternion_norm
    quaternion_k /= quaternion_norm
    quaternion_real /= quaternion_norm

    roll_radians = math.atan2(
        2.0
        * (
            quaternion_real * quaternion_i
            + quaternion_j * quaternion_k
        ),
        1.0
        - 2.0
        * (
            quaternion_i * quaternion_i
            + quaternion_j * quaternion_j
        ),
    )
    pitch_sine = 2.0 * (
        quaternion_real * quaternion_j
        - quaternion_k * quaternion_i
    )
    pitch_radians = math.asin(max(-1.0, min(1.0, pitch_sine)))
    yaw_radians = math.atan2(
        2.0
        * (
            quaternion_real * quaternion_k
            + quaternion_i * quaternion_j
        ),
        1.0
        - 2.0
        * (
            quaternion_j * quaternion_j
            + quaternion_k * quaternion_k
        ),
    )

    return (
        math.degrees(roll_radians),
        math.degrees(pitch_radians),
        math.degrees(yaw_radians),
        quaternion_norm,
    )


def imu_motion_direction(gyro_z_deg_s: float) -> str:
    if gyro_z_deg_s >= 1.0:
        return "LEFT/CCW"
    if gyro_z_deg_s <= -1.0:
        return "RIGHT/CW"
    return "STILL"


def write_imu_csv(
    path: str,
    samples: list[tuple[float, ImuTelemetry]],
    reference_yaw_degrees: float,
) -> None:
    field_names = (
        "host_elapsed_s",
        "mcu_time_ms",
        "roll_deg",
        "pitch_deg",
        "yaw_deg",
        "quaternion_yaw_deg",
        "yaw_delta_deg",
        "quaternion_i",
        "quaternion_j",
        "quaternion_k",
        "quaternion_real",
        "quaternion_norm",
        "gyro_x_deg_s",
        "gyro_y_deg_s",
        "gyro_z_deg_s",
        "linear_accel_x_m_s2",
        "linear_accel_y_m_s2",
        "linear_accel_z_m_s2",
        "linear_accel_norm_m_s2",
        "gyro_accuracy",
        "linear_accel_accuracy",
        "quaternion_accuracy",
        "status_flags_hex",
        "status",
    )

    with open(path, "w", newline="", encoding="utf-8") as csv_file:
        writer = csv.DictWriter(csv_file, fieldnames=field_names)
        writer.writeheader()
        for elapsed, imu in samples:
            roll, pitch, quaternion_yaw, quaternion_norm = (
                imu_quaternion_to_euler_degrees(imu)
            )
            yaw = imu.yaw_mdeg / 1000.0
            accel_x = imu.linear_accel_x_mm_s2 / 1000.0
            accel_y = imu.linear_accel_y_mm_s2 / 1000.0
            accel_z = imu.linear_accel_z_mm_s2 / 1000.0
            writer.writerow(
                {
                    "host_elapsed_s": f"{elapsed:.6f}",
                    "mcu_time_ms": imu.mcu_time_ms,
                    "roll_deg": f"{roll:.6f}",
                    "pitch_deg": f"{pitch:.6f}",
                    "yaw_deg": f"{yaw:.6f}",
                    "quaternion_yaw_deg": f"{quaternion_yaw:.6f}",
                    "yaw_delta_deg": f"{normalize_angle_degrees(yaw - reference_yaw_degrees):.6f}",
                    "quaternion_i": f"{imu.quaternion_i_q14 / 16384.0:.8f}",
                    "quaternion_j": f"{imu.quaternion_j_q14 / 16384.0:.8f}",
                    "quaternion_k": f"{imu.quaternion_k_q14 / 16384.0:.8f}",
                    "quaternion_real": f"{imu.quaternion_real_q14 / 16384.0:.8f}",
                    "quaternion_norm": f"{quaternion_norm:.8f}",
                    "gyro_x_deg_s": f"{imu.gyro_x_mdeg_s / 1000.0:.6f}",
                    "gyro_y_deg_s": f"{imu.gyro_y_mdeg_s / 1000.0:.6f}",
                    "gyro_z_deg_s": f"{imu.gyro_z_mdeg_s / 1000.0:.6f}",
                    "linear_accel_x_m_s2": f"{accel_x:.6f}",
                    "linear_accel_y_m_s2": f"{accel_y:.6f}",
                    "linear_accel_z_m_s2": f"{accel_z:.6f}",
                    "linear_accel_norm_m_s2": (
                        f"{math.sqrt(accel_x * accel_x + accel_y * accel_y + accel_z * accel_z):.6f}"
                    ),
                    "gyro_accuracy": imu.gyro_accuracy,
                    "linear_accel_accuracy": imu.linear_accel_accuracy,
                    "quaternion_accuracy": imu.quaternion_accuracy,
                    "status_flags_hex": f"0x{imu.status_flags:02X}",
                    "status": imu_status_names(imu.status_flags),
                }
            )


def run_imu_monitor(
    port_name: str,
    seconds: float,
    settle_seconds: float,
    rate_hz: float,
    csv_path: str | None,
) -> None:
    if not 5.0 <= seconds <= 300.0:
        raise SystemExit("IMU monitor duration must be from 5 to 300 seconds.")
    if not 0.0 <= settle_seconds < seconds:
        raise SystemExit(
            "Settle duration must be zero or more and shorter than the test."
        )
    if not 0.5 <= rate_hz <= 20.0:
        raise SystemExit("IMU print rate must be from 0.5 to 20 Hz.")

    serial, _ = require_pyserial()
    receive_buffer = bytearray()
    samples: list[tuple[float, ImuTelemetry]] = []
    reference_yaw_degrees: float | None = None
    next_print = 0.0
    print_period = 1.0 / rate_hz

    with serial.Serial(
        port=port_name,
        baudrate=115200,
        bytesize=serial.EIGHTBITS,
        parity=serial.PARITY_NONE,
        stopbits=serial.STOPBITS_ONE,
        timeout=0.02,
    ) as port:
        port.reset_input_buffer()
        time.sleep(0.2)
        started_at = time.monotonic()
        deadline = started_at + seconds
        print(f"Port: {port_name} (115200 8N1, Protocol V2)")
        print(
            "IMU-only monitor: no motor, servo, stop, reset, or drive "
            "command will be transmitted."
        )
        print(
            f"Keep the vehicle still for {settle_seconds:.1f} s, then rotate "
            "it left and right while keeping the sensor level."
        )
        print(
            "Configured sign: positive yaw/gyro_z = LEFT/CCW, negative = "
            "RIGHT/CW. Roll/pitch/quaternion_yaw are calculated from the "
            "transmitted quaternion."
        )

        while time.monotonic() < deadline:
            now = time.monotonic()
            elapsed = now - started_at
            for frame in read_available_frames(port, receive_buffer):
                if frame.message_id != MSG_TELEMETRY_IMU:
                    continue

                try:
                    imu = decode_imu_telemetry(frame)
                except ValueError as error:
                    raise SystemExit(str(error)) from error

                samples.append((elapsed, imu))
                quaternion_healthy = (
                    imu.status_flags & IMU_STATUS_QUATERNION_VALID
                ) != 0 and (imu.status_flags & IMU_STATUS_STALE) == 0
                if (
                    reference_yaw_degrees is None
                    and elapsed >= settle_seconds
                    and quaternion_healthy
                ):
                    reference_yaw_degrees = imu.yaw_mdeg / 1000.0
                    print(
                        "[REFERENCE] "
                        f"yaw={reference_yaw_degrees:+.3f} deg at "
                        f"t={elapsed:.2f} s; delta_yaw is now zeroed."
                    )

                if now < next_print:
                    continue

                roll, pitch, quaternion_yaw, quaternion_norm = (
                    imu_quaternion_to_euler_degrees(imu)
                )
                yaw = imu.yaw_mdeg / 1000.0
                gyro_x = imu.gyro_x_mdeg_s / 1000.0
                gyro_y = imu.gyro_y_mdeg_s / 1000.0
                gyro_z = imu.gyro_z_mdeg_s / 1000.0
                accel_x = imu.linear_accel_x_mm_s2 / 1000.0
                accel_y = imu.linear_accel_y_mm_s2 / 1000.0
                accel_z = imu.linear_accel_z_mm_s2 / 1000.0
                accel_norm = math.sqrt(
                    accel_x * accel_x
                    + accel_y * accel_y
                    + accel_z * accel_z
                )
                if reference_yaw_degrees is None:
                    yaw_delta_text = "waiting"
                else:
                    yaw_delta_text = (
                        f"{normalize_angle_degrees(yaw - reference_yaw_degrees):+.3f}deg"
                    )

                print(
                    f"[IMU] t={elapsed:6.2f}s mcu={imu.mcu_time_ms:10d}ms "
                    f"motion={imu_motion_direction(gyro_z):9s}"
                )
                print(
                    "      angle "
                    f"roll={roll:+8.3f}deg pitch={pitch:+8.3f}deg "
                    f"yaw={yaw:+8.3f}deg delta_yaw={yaw_delta_text} "
                    f"quat_yaw={quaternion_yaw:+8.3f}deg"
                )
                print(
                    "      gyro "
                    f"x={gyro_x:+9.3f} y={gyro_y:+9.3f} "
                    f"z={gyro_z:+9.3f} deg/s"
                )
                print(
                    "      lin_accel "
                    f"x={accel_x:+8.3f} y={accel_y:+8.3f} "
                    f"z={accel_z:+8.3f} norm={accel_norm:8.3f} m/s^2"
                )
                print(
                    "      quaternion "
                    f"i={imu.quaternion_i_q14 / 16384.0:+.6f} "
                    f"j={imu.quaternion_j_q14 / 16384.0:+.6f} "
                    f"k={imu.quaternion_k_q14 / 16384.0:+.6f} "
                    f"real={imu.quaternion_real_q14 / 16384.0:+.6f} "
                    f"norm={quaternion_norm:.6f}"
                )
                print(
                    "      accuracy "
                    f"gyro=g{imu.gyro_accuracy} "
                    f"linear_accel=a{imu.linear_accel_accuracy} "
                    f"quaternion=q{imu.quaternion_accuracy} "
                    f"status={imu_status_names(imu.status_flags)}"
                )
                next_print = now + print_period

    if not samples:
        raise SystemExit(
            "IMU monitor: FAIL - no TELEMETRY_IMU was received. Confirm "
            "the selected COM port, PC communication firmware, and BNO085 "
            "SPI initialization."
        )
    if reference_yaw_degrees is None:
        latest = samples[-1][1]
        raise SystemExit(
            "IMU monitor: FAIL - no fresh valid quaternion was received "
            f"after settling. Latest status: {imu_status_names(latest.status_flags)}"
        )

    checked = [
        (elapsed, sample)
        for elapsed, sample in samples
        if elapsed >= settle_seconds
    ]
    yaw_deltas = [
        normalize_angle_degrees(
            sample.yaw_mdeg / 1000.0 - reference_yaw_degrees
        )
        for _, sample in checked
    ]
    peak_gyro_x = max(abs(sample.gyro_x_mdeg_s) for _, sample in checked) / 1000.0
    peak_gyro_y = max(abs(sample.gyro_y_mdeg_s) for _, sample in checked) / 1000.0
    peak_gyro_z = max(abs(sample.gyro_z_mdeg_s) for _, sample in checked) / 1000.0
    latest = samples[-1][1]

    print("IMU monitor: PASS - TELEMETRY_IMU received and decoded.")
    print(
        f"Samples={len(samples)}, reference_yaw={reference_yaw_degrees:+.3f}deg, "
        f"delta_yaw_range=[{min(yaw_deltas):+.3f}, "
        f"{max(yaw_deltas):+.3f}]deg"
    )
    print(
        "Peak absolute gyro: "
        f"x={peak_gyro_x:.3f}, y={peak_gyro_y:.3f}, "
        f"z={peak_gyro_z:.3f} deg/s"
    )
    print(
        "Expected hand-turn result: about +90 deg at the left endpoint and "
        "about -90 deg at the right endpoint relative to REFERENCE."
    )
    print(f"Latest status: {imu_status_names(latest.status_flags)}")

    if csv_path is not None:
        write_imu_csv(csv_path, samples, reference_yaw_degrees)
        print(f"CSV saved: {csv_path}")


def run_imu_calibration_test(
    port_name: str,
    seconds: float,
    settle_seconds: float,
) -> None:
    if not 5.0 <= seconds <= 60.0:
        raise SystemExit(
            "IMU calibration test duration must be from 5 to 60 seconds."
        )
    if not 0.0 <= settle_seconds < seconds:
        raise SystemExit(
            "Settle duration must be zero or more and shorter than the test."
        )

    serial, _ = require_pyserial()
    receive_buffer = bytearray()
    samples: list[tuple[float, ImuTelemetry]] = []
    next_print = 0.0

    with serial.Serial(
        port=port_name,
        baudrate=115200,
        bytesize=serial.EIGHTBITS,
        parity=serial.PARITY_NONE,
        stopbits=serial.STOPBITS_ONE,
        timeout=0.02,
    ) as port:
        port.reset_input_buffer()
        time.sleep(0.2)
        started_at = time.monotonic()
        deadline = started_at + seconds
        print(f"Port: {port_name} (115200 8N1, Protocol V2)")
        print(
            "IMU-only listen test: no motor, servo, or drive command will "
            "be transmitted."
        )
        print(
            "Place the BNO085 on a rigid stationary surface and do not "
            f"touch it for {seconds:.1f} seconds."
        )

        while time.monotonic() < deadline:
            now = time.monotonic()
            elapsed = now - started_at
            for frame in read_available_frames(port, receive_buffer):
                if frame.message_id != MSG_TELEMETRY_IMU:
                    continue

                try:
                    imu = decode_imu_telemetry(frame)
                except ValueError as error:
                    raise SystemExit(str(error)) from error

                samples.append((elapsed, imu))
                if now >= next_print:
                    phase = "settling" if elapsed < settle_seconds else "checking"
                    print(
                        "[IMU-CAL] "
                        f"phase={phase:8s} "
                        f"time={imu.mcu_time_ms} "
                        f"accuracy=g{imu.gyro_accuracy} "
                        f"gyro_z={imu.gyro_z_mdeg_s / 1000.0:+.3f}deg/s "
                        f"yaw={imu.yaw_mdeg / 1000.0:+.3f}deg "
                        f"status={imu_status_names(imu.status_flags)}"
                    )
                    next_print = now + 0.5

    if not samples:
        raise SystemExit(
            "IMU calibration test: FAIL - no TELEMETRY_IMU was received. "
            "Confirm that the new firmware was flashed and BNO085 SPI "
            "initialization completed."
        )

    checked = [sample for elapsed, sample in samples if elapsed >= settle_seconds]
    if len(checked) < 5:
        raise SystemExit(
            "IMU calibration test: FAIL - fewer than five IMU samples were "
            "received after the settling interval."
        )

    required_status = IMU_STATUS_CONNECTED | IMU_STATUS_GYRO_VALID
    error_status = (
        IMU_STATUS_STALE | IMU_STATUS_SPI_ERROR | IMU_STATUS_PROTOCOL_ERROR
    )
    latest = checked[-1]
    if (
        (latest.status_flags & required_status) != required_status
        or (latest.status_flags & error_status) != 0
    ):
        raise SystemExit(
            "IMU calibration test: FAIL - the latest IMU status is not "
            f"healthy: {imu_status_names(latest.status_flags)}"
        )

    consecutive_qualified = 0
    maximum_consecutive = 0
    for sample in checked:
        healthy = (
            (sample.status_flags & required_status) == required_status
            and (sample.status_flags & error_status) == 0
        )
        if healthy and sample.gyro_accuracy >= 1:
            consecutive_qualified += 1
            maximum_consecutive = max(
                maximum_consecutive,
                consecutive_qualified,
            )
        else:
            consecutive_qualified = 0

    if latest.gyro_accuracy < 1 or maximum_consecutive < 5:
        observed = sorted({sample.gyro_accuracy for sample in checked})
        raise SystemExit(
            "IMU calibration test: FAIL - gyro accuracy did not remain at "
            "g1 or higher for five consecutive samples. "
            f"Observed levels={observed}, latest=g{latest.gyro_accuracy}. "
            "This is a calibration-grade failure only; the current emergency "
            "commissioning firmware still permits fresh, valid g0 gyro data "
            "for odometry fusion. Verify TELEMETRY_ODOMETRY IMU_FUSED while "
            "the measured vehicle speed is at least 0.02 m/s."
        )

    yaw_delta_mdeg = normalize_yaw_delta_mdeg(
        latest.yaw_mdeg - checked[0].yaw_mdeg
    )
    max_abs_gyro_deg_s = max(
        abs(sample.gyro_z_mdeg_s) for sample in checked
    ) / 1000.0

    print(
        "IMU calibration test: PASS - gyro accuracy reached and retained "
        f"g{latest.gyro_accuracy}."
    )
    print(
        f"Checked samples={len(checked)}, "
        f"maximum consecutive g1+={maximum_consecutive}, "
        f"yaw change={yaw_delta_mdeg / 1000.0:+.3f} deg, "
        f"max |gyro_z|={max_abs_gyro_deg_s:.3f} deg/s"
    )
    if max_abs_gyro_deg_s > 5.0:
        print(
            "Warning: calibration passed, but motion/vibration was detected. "
            "Repeat with the module and wires fully supported."
        )


def run_telemetry_monitor(
    port_name: str,
    seconds: float,
) -> None:
    if not 0.5 <= seconds <= 60.0:
        raise SystemExit(
            "Telemetry monitor duration must be from 0.5 to 60 seconds."
        )

    serial, _ = require_pyserial()
    receive_buffer = bytearray()
    drive_count = 0
    fault_count = 0
    range_count = 0
    imu_count = 0
    odometry_count = 0
    last_drive: DriveTelemetry | None = None
    last_range: RangeTelemetry | None = None
    last_imu: ImuTelemetry | None = None
    last_odometry: OdometryTelemetry | None = None
    next_drive_print = 0.0
    next_range_print = 0.0
    next_imu_print = 0.0
    next_odometry_print = 0.0

    with serial.Serial(
        port=port_name,
        baudrate=115200,
        bytesize=serial.EIGHTBITS,
        parity=serial.PARITY_NONE,
        stopbits=serial.STOPBITS_ONE,
        timeout=0.02,
    ) as port:
        port.reset_input_buffer()
        time.sleep(0.2)
        deadline = time.monotonic() + seconds
        print(
            f"Monitoring Jetson-facing telemetry on {port_name} for "
            f"{seconds:.1f} s"
        )

        while time.monotonic() < deadline:
            now = time.monotonic()
            for frame in read_available_frames(port, receive_buffer):
                try:
                    if frame.message_id == MSG_TELEMETRY_DRIVE:
                        last_drive = decode_drive_telemetry(frame)
                        drive_count += 1
                        if now >= next_drive_print:
                            print(
                                "[DRIVE] "
                                f"time={last_drive.mcu_time_ms} "
                                f"target={last_drive.target_speed_mm_s:+d} "
                                f"measured={last_drive.measured_speed_mm_s:+d} "
                                f"duty="
                                f"{last_drive.motor_duty_permille / 10.0:+.1f}% "
                                f"steer_cmd="
                                f"{last_drive.steering_cmd_cdeg / 100.0:+.2f}deg "
                                f"steer_feedback="
                                f"{last_drive.steering_feedback_cdeg / 100.0:+.2f}deg "
                                f"encoder={last_drive.encoder_count:+d} "
                                f"yaw={last_drive.yaw_cdeg / 100.0:+.2f}deg "
                                f"state={state_name(last_drive.state)} "
                                f"last_seq={last_drive.last_drive_seq} "
                                f"faults="
                                f"{fault_names(last_drive.active_fault_bits)}"
                            )
                            next_drive_print = now + 0.5
                    elif frame.message_id == MSG_FAULT_EVENT:
                        event = decode_fault_event(frame)
                        fault_count += 1
                        print(
                            "[FAULT] "
                            f"time={event.occurred_at_ms} "
                            f"state={state_name(event.state)} "
                            f"active={fault_names(event.active_fault_bits)} "
                            f"latched="
                            f"{fault_names(event.latched_fault_bits)} "
                            f"action={event.action}"
                        )
                    elif frame.message_id == MSG_TELEMETRY_RANGE:
                        last_range = decode_range_telemetry(frame)
                        range_count += 1
                        if now >= next_range_print:
                            print(
                                "[RANGE] "
                                f"time={last_range.mcu_time_ms} "
                                f"front_left="
                                f"{format_range_channel(last_range.front_left_mm, last_range.valid_mask, 0)} "
                                f"front_right="
                                f"{format_range_channel(last_range.front_right_mm, last_range.valid_mask, 1)} "
                                f"rear_left="
                                f"{format_range_channel(last_range.rear_left_mm, last_range.valid_mask, 2)} "
                                f"rear_right="
                                f"{format_range_channel(last_range.rear_right_mm, last_range.valid_mask, 3)} "
                                f"valid_mask=0x{last_range.valid_mask:02X}"
                            )
                            next_range_print = now + 0.5
                    elif frame.message_id == MSG_TELEMETRY_IMU:
                        last_imu = decode_imu_telemetry(frame)
                        imu_count += 1
                        if now >= next_imu_print:
                            print(
                                "[IMU] "
                                f"time={last_imu.mcu_time_ms} "
                                f"yaw={last_imu.yaw_mdeg / 1000.0:+.3f}deg "
                                f"gyro_z="
                                f"{last_imu.gyro_z_mdeg_s / 1000.0:+.3f}deg/s "
                                f"lin_accel=("
                                f"{last_imu.linear_accel_x_mm_s2}, "
                                f"{last_imu.linear_accel_y_mm_s2}, "
                                f"{last_imu.linear_accel_z_mm_s2})mm/s^2 "
                                f"accuracy=(g{last_imu.gyro_accuracy}, "
                                f"a{last_imu.linear_accel_accuracy}, "
                                f"q{last_imu.quaternion_accuracy}) "
                                f"status={imu_status_names(last_imu.status_flags)}"
                            )
                            next_imu_print = now + 0.5
                    elif frame.message_id == MSG_TELEMETRY_ODOMETRY:
                        last_odometry = decode_odometry_telemetry(frame)
                        odometry_count += 1
                        if now >= next_odometry_print:
                            print(
                                "[ODOM] "
                                f"time={last_odometry.mcu_time_ms} "
                                f"pose=({last_odometry.x_mm}, "
                                f"{last_odometry.y_mm})mm "
                                f"yaw="
                                f"{last_odometry.yaw_mdeg / 1000.0:+.3f}deg "
                                f"distance={last_odometry.distance_mm:+d}mm "
                                f"speed="
                                f"{last_odometry.linear_speed_mm_s:+d}mm/s "
                                f"yaw_rate="
                                f"{last_odometry.yaw_rate_mdeg_s / 1000.0:+.3f}deg/s "
                                f"steer="
                                f"{last_odometry.steering_cdeg / 100.0:+.2f}deg "
                                f"curvature="
                                f"{last_odometry.curvature_micro_per_m / 1000000.0:+.4f}/m "
                                f"source="
                                f"{odometry_steering_source_name(last_odometry.steering_source)} "
                                f"status="
                                f"{odometry_status_names(last_odometry.status_flags)}"
                            )
                            next_odometry_print = now + 0.5
                except ValueError as error:
                    raise SystemExit(str(error)) from error

    if drive_count == 0:
        raise SystemExit(
            "Telemetry monitor: FAIL - no TELEMETRY_DRIVE was received."
        )
    if range_count == 0:
        raise SystemExit(
            "Telemetry monitor: FAIL - no TELEMETRY_RANGE was received."
        )
    if odometry_count == 0:
        raise SystemExit(
            "Telemetry monitor: FAIL - no V2 TELEMETRY_ODOMETRY was "
            "received. Flash the firmware containing message 0x85."
        )
    if imu_count == 0:
        raise SystemExit(
            "Telemetry monitor: FAIL - no TELEMETRY_IMU was received. "
            "Flash the SPI5+BNO085 firmware."
        )

    print("Telemetry monitor: PASS")
    print(
        "Frames received: "
        f"DRIVE={drive_count}, FAULT={fault_count}, "
        f"RANGE={range_count}, IMU={imu_count}, "
        f"ODOMETRY={odometry_count}"
    )
    print(
        "Jetson transport fields decoded: target/measured speed, motor duty, "
        "steering command/feedback field, encoder count, yaw field, state, "
        "last accepted sequence, and fault bits."
    )
    print(
        "Steering feedback status: no sensor is mounted. Odometry therefore "
        "uses the seven-point command-angle LUT and reports "
        "COMMAND_ESTIMATE/STEERING_ESTIMATED."
    )
    if last_imu is not None and (
        last_imu.status_flags
        & (IMU_STATUS_CONNECTED | IMU_STATUS_GYRO_VALID)
    ) != (IMU_STATUS_CONNECTED | IMU_STATUS_GYRO_VALID):
        print(
            "IMU status: telemetry transport works, but fusion is degraded: "
            f"{imu_status_names(last_imu.status_flags)}"
        )
    if last_range is not None and last_range.valid_mask == 0:
        print(
            "Range status: frame transport works, but all channels are "
            "currently invalid; the installed front HC-SR04 is not yet "
            "mapped into TELEMETRY_RANGE."
        )
    if last_odometry is not None:
        required_flags = (
            ODOMETRY_STATUS_VALID
            | ODOMETRY_STATUS_ENCODER_CALIBRATED
            | ODOMETRY_STATUS_GEOMETRY_CALIBRATED
            | ODOMETRY_STATUS_STEERING_ESTIMATED
        )
        if (
            last_odometry.status_flags & required_flags
        ) != required_flags:
            raise SystemExit(
                "Telemetry monitor: FAIL - odometry frame arrived but its "
                "required valid/calibration/estimated flags are incomplete: "
                f"{odometry_status_names(last_odometry.status_flags)}"
            )
        print(
            "Odometry status: PASS - V2 pose telemetry is valid and clearly "
            "marked as command-steering-estimated."
        )


def normalize_yaw_delta_mdeg(delta_mdeg: int) -> int:
    while delta_mdeg > 180000:
        delta_mdeg -= 360000
    while delta_mdeg < -180000:
        delta_mdeg += 360000
    return delta_mdeg


def verify_odometry_step(
    samples: list[OdometryTelemetry],
    sent_sequences: set[int],
    steering_cdeg: int,
    expected_distance_sign: int,
    expected_yaw_sign: int,
    label: str,
) -> None:
    required_flags = (
        ODOMETRY_STATUS_VALID
        | ODOMETRY_STATUS_ENCODER_CALIBRATED
        | ODOMETRY_STATUS_GEOMETRY_CALIBRATED
        | ODOMETRY_STATUS_STEERING_ESTIMATED
    )
    accepted = [
        sample
        for sample in samples
        if sample.last_drive_seq in sent_sequences
        and sample.steering_cdeg == steering_cdeg
        and (sample.status_flags & required_flags) == required_flags
        and sample.steering_source
        == ODOMETRY_STEERING_COMMAND_ESTIMATE
    ]
    if len(accepted) < 2:
        raise SystemExit(
            f"{label}: FAIL - fewer than two valid command-estimated "
            "odometry samples matched the drive command."
        )

    distance_delta = (
        accepted[-1].distance_mm - accepted[0].distance_mm
    )
    yaw_delta = normalize_yaw_delta_mdeg(
        accepted[-1].yaw_mdeg - accepted[0].yaw_mdeg
    )

    if distance_delta == 0 or (
        (distance_delta > 0) != (expected_distance_sign > 0)
    ):
        raise SystemExit(
            f"{label}: FAIL - odometry distance direction is wrong "
            f"(delta={distance_delta:+d} mm)."
        )

    if expected_yaw_sign == 0:
        if abs(yaw_delta) > 100:
            raise SystemExit(
                f"{label}: FAIL - straight command changed yaw by "
                f"{yaw_delta / 1000.0:+.3f} deg."
            )
    elif abs(yaw_delta) <= 100 or (
        (yaw_delta > 0) != (expected_yaw_sign > 0)
    ):
        raise SystemExit(
            f"{label}: FAIL - odometry yaw direction is wrong or too small "
            f"(delta={yaw_delta / 1000.0:+.3f} deg)."
        )

    print(
        f"[{label}] ODOM PASS - distance={distance_delta:+d} mm, "
        f"yaw={yaw_delta / 1000.0:+.3f} deg, "
        f"source=COMMAND_ESTIMATE"
    )


def run_odometry_test(
    port_name: str,
    timeout_s: float,
    wheels_off_ground: bool,
) -> None:
    if not wheels_off_ground:
        raise SystemExit(
            "Refusing to run the odometry motion test. Lift all driven "
            "wheels and add --wheels-off-ground."
        )

    serial, _ = require_pyserial()
    receive_buffer = bytearray()
    next_sequence = 0
    steps = (
        ("odom-forward-straight", 80, 0, 1, 0),
        ("odom-forward-left", 80, 1000, 1, 1),
        ("odom-forward-right", 80, -1000, 1, -1),
        ("odom-reverse-straight", -80, 0, -1, 0),
    )

    with serial.Serial(
        port=port_name,
        baudrate=115200,
        bytesize=serial.EIGHTBITS,
        parity=serial.PARITY_NONE,
        stopbits=serial.STOPBITS_ONE,
        timeout=0.02,
    ) as port:
        port.reset_input_buffer()
        time.sleep(0.4)
        print(f"Port: {port_name} (115200 8N1, Protocol V2)")
        print(
            "Odometry scenario: forward straight -> forward left -> "
            "forward right -> reverse straight"
        )
        print(
            "This wheels-off-ground test verifies calculation signs and UART "
            "transport, not real floor-position accuracy."
        )

        try:
            (
                next_sequence,
                neutral_samples,
                neutral_sequences,
            ) = stream_drive_command(
                port,
                receive_buffer,
                next_sequence,
                0,
                0,
                False,
                0.8,
                "rearm",
            )
            verify_ready(neutral_samples, neutral_sequences)

            for (
                label,
                speed_mm_s,
                steering_cdeg,
                distance_sign,
                yaw_sign,
            ) in steps:
                odometry_samples: list[OdometryTelemetry] = []
                next_sequence, drive_samples, sent_sequences = (
                    stream_drive_command(
                        port,
                        receive_buffer,
                        next_sequence,
                        speed_mm_s,
                        steering_cdeg,
                        True,
                        1.5,
                        label,
                        odometry_samples,
                    )
                )
                verify_drive_step(
                    drive_samples,
                    sent_sequences,
                    speed_mm_s,
                    steering_cdeg,
                    label,
                )
                verify_odometry_step(
                    odometry_samples,
                    sent_sequences,
                    steering_cdeg,
                    distance_sign,
                    yaw_sign,
                    label,
                )
                next_sequence, _, _ = stream_drive_command(
                    port,
                    receive_buffer,
                    next_sequence,
                    0,
                    0,
                    False,
                    0.4,
                    f"{label}-neutral",
                )

            next_sequence = send_stop_command(
                port,
                receive_buffer,
                next_sequence,
                timeout_s,
            )
        except BaseException:
            try:
                port.write(
                    encode_frame(
                        MSG_CMD_STOP,
                        next_sequence,
                        b"\x00",
                    )
                )
                port.flush()
            except Exception:
                pass
            raise

    print("Command-estimated odometry scenario: PASS")


def run_steering_monitor(
    port_name: str,
    seconds: float,
    timeout_s: float,
) -> None:
    raise SystemExit(
        "steering-monitor is unavailable in the active Protocol V2 firmware: "
        "no steering sensor is mounted and 0x82 is COMMAND_RESULT. Use "
        "servo-calibrate now; steering telemetry will be added after the "
        "sensor hardware is finalized."
    )

    if not 0.5 <= seconds <= 60.0:
        raise SystemExit("Monitor duration must be from 0.5 to 60 seconds.")

    serial, _ = require_pyserial()
    receive_buffer = bytearray()

    with serial.Serial(
        port=port_name,
        baudrate=115200,
        bytesize=serial.EIGHTBITS,
        parity=serial.PARITY_NONE,
        stopbits=serial.STOPBITS_ONE,
        timeout=0.05,
    ) as port:
        port.reset_input_buffer()
        send_motor_test_request(
            port,
            receive_buffer,
            sequence=0,
            duty_permille=0,
            duration_ms=0,
            timeout_s=timeout_s,
        )

        deadline = time.monotonic() + seconds
        next_print = 0.0
        while time.monotonic() < deadline:
            received = port.read(max(port.in_waiting, 1))
            if received:
                receive_buffer.extend(received)

            while True:
                frame = pop_frame(receive_buffer)
                if frame is None:
                    break
                if frame.message_id != MSG_LEGACY_TELEMETRY_ODOMETRY:
                    continue

                telemetry = decode_legacy_odometry_telemetry(frame)
                now = time.monotonic()
                if now < next_print:
                    continue
                next_print = now + 0.1
                print(
                    f"raw={telemetry.steering_adc_raw:4d} "
                    f"wheel={telemetry.measured_wheel_steering_cdeg / 100.0:+7.2f}deg "
                    f"center={telemetry.center_steering_cdeg / 100.0:+7.2f}deg "
                    f"status=0x{telemetry.status_flags:04X} "
                    f"pose=({telemetry.x_mm}, {telemetry.y_mm}, "
                    f"{telemetry.yaw_mdeg / 1000.0:+.2f}deg)"
                )


def decode_servo_diagnostic_response(
    frame: Frame,
) -> tuple[int, int, int, int, int]:
    if frame.message_id != MSG_DIAG_SERVO_RESPONSE:
        raise ValueError("frame is not DIAG_SERVO_RESPONSE")
    if len(frame.payload) != 7:
        raise ValueError(
            f"DIAG_SERVO_RESPONSE payload is {len(frame.payload)} bytes, "
            "expected 7"
        )
    return struct.unpack("<BBHHB", frame.payload)


def servo_result_name(result: int) -> str:
    names = {
        0: "accepted",
        1: "invalid state",
        2: "out of range",
        3: "fault active",
        4: "unsupported",
        5: "invalid value",
        6: "duplicate sequence",
        7: "not armed",
    }
    return names.get(result, f"unknown result {result}")


def raise_for_servo_command_result(frame: Frame) -> None:
    if (frame.message_id != MSG_COMMAND_RESULT) or (len(frame.payload) != 4):
        return

    request_message_id, _, result, state = struct.unpack(
        "<BBBB",
        frame.payload,
    )
    if request_message_id != MSG_DIAG_SERVO_REQUEST:
        return
    raise SystemExit(
        "STM32 rejected servo calibration: "
        f"{servo_result_name(result)} (system state={state})."
    )


def send_servo_diagnostic_request(
    port: object,
    sequence: int,
    pulse_us: int,
) -> None:
    frame = encode_frame(
        MSG_DIAG_SERVO_REQUEST,
        sequence,
        struct.pack("<H", pulse_us),
        version=SERVO_DIAGNOSTIC_PROTOCOL_VERSION,
    )
    port.write(frame)


def run_servo_calibrate(
    port_name: str,
    pulse_us: int,
    seconds: float,
    timeout_s: float,
) -> None:
    if not (
        MIN_SERVO_DIAGNOSTIC_PULSE_US
        <= pulse_us
        <= MAX_SERVO_DIAGNOSTIC_PULSE_US
    ):
        raise SystemExit(
            "Initial calibration pulse must be from "
            f"{MIN_SERVO_DIAGNOSTIC_PULSE_US} to "
            f"{MAX_SERVO_DIAGNOSTIC_PULSE_US} us."
        )
    if not 0.5 <= seconds <= 10.0:
        raise SystemExit("Calibration duration must be from 0.5 to 10 seconds.")

    serial, _ = require_pyserial()
    receive_buffer = bytearray()
    sequence = 0
    response_count = 0
    last_printed: tuple[int, int, int] | None = None

    with serial.Serial(
        port=port_name,
        baudrate=115200,
        bytesize=serial.EIGHTBITS,
        parity=serial.PARITY_NONE,
        stopbits=serial.STOPBITS_ONE,
        timeout=0.05,
    ) as port:
        port.reset_input_buffer()
        time.sleep(0.4)
        deadline = time.monotonic() + seconds
        next_send = 0.0

        print(
            f"Holding {pulse_us} us for {seconds:.1f} s. "
            "The firmware will stop PWM within 300 ms if commands stop."
        )
        try:
            while time.monotonic() < deadline:
                now = time.monotonic()
                if now >= next_send:
                    send_servo_diagnostic_request(
                        port,
                        sequence,
                        pulse_us,
                    )
                    sequence = (sequence + 1) & 0xFF
                    next_send = now + 0.1

                received = port.read(max(port.in_waiting, 1))
                if received:
                    receive_buffer.extend(received)

                while True:
                    frame = pop_frame(
                        receive_buffer,
                        expected_version=SERVO_DIAGNOSTIC_PROTOCOL_VERSION,
                    )
                    if frame is None:
                        break
                    raise_for_servo_command_result(frame)
                    if frame.message_id != MSG_DIAG_SERVO_RESPONSE:
                        continue

                    _, result, applied_pulse, adc_raw, flags = (
                        decode_servo_diagnostic_response(frame)
                    )
                    if result != 0:
                        raise SystemExit(
                            "STM32 rejected servo calibration: "
                            f"{servo_result_name(result)}."
                        )
                    response_count += 1
                    printable = (applied_pulse, adc_raw, flags)
                    if printable != last_printed:
                        print(
                            f"pulse={applied_pulse:4d} us "
                            f"raw={adc_raw:4d} "
                            f"pwm={'on' if flags & 0x01 else 'starting'} "
                            f"adc={'valid' if flags & 0x02 else 'invalid'}"
                        )
                        last_printed = printable
        finally:
            send_servo_diagnostic_request(port, sequence, 0)
            port.flush()

        if response_count == 0:
            raise SystemExit(
                f"No valid V2 servo response received from {port_name} "
                f"within {timeout_s:.1f}s. Flash the updated firmware first."
            )

    print("Servo PWM off.")


def run_servo_off(port_name: str, timeout_s: float) -> None:
    serial, _ = require_pyserial()
    receive_buffer = bytearray()

    with serial.Serial(
        port=port_name,
        baudrate=115200,
        bytesize=serial.EIGHTBITS,
        parity=serial.PARITY_NONE,
        stopbits=serial.STOPBITS_ONE,
        timeout=0.05,
    ) as port:
        port.reset_input_buffer()
        time.sleep(0.4)
        send_servo_diagnostic_request(port, 0, 0)
        deadline = time.monotonic() + timeout_s

        while time.monotonic() < deadline:
            received = port.read(max(port.in_waiting, 1))
            if received:
                receive_buffer.extend(received)

            while True:
                frame = pop_frame(
                    receive_buffer,
                    expected_version=SERVO_DIAGNOSTIC_PROTOCOL_VERSION,
                )
                if frame is None:
                    break
                raise_for_servo_command_result(frame)
                if frame.message_id == MSG_DIAG_SERVO_RESPONSE:
                    _, result, _, adc_raw, _ = (
                        decode_servo_diagnostic_response(frame)
                    )
                    if result != 0:
                        raise SystemExit(
                            "STM32 rejected servo-off: "
                            f"{servo_result_name(result)}."
                        )
                    print(f"Servo PWM off. Current steering raw={adc_raw}.")
                    return

    raise SystemExit(
        f"No valid V2 servo-off response received from {port_name}."
    )


def run_encoder_read(port_name: str, timeout_s: float) -> None:
    serial, _ = require_pyserial()
    receive_buffer = bytearray()

    with serial.Serial(
        port=port_name,
        baudrate=115200,
        bytesize=serial.EIGHTBITS,
        parity=serial.PARITY_NONE,
        stopbits=serial.STOPBITS_ONE,
        timeout=0.05,
    ) as port:
        port.reset_input_buffer()
        send_motor_test_request(
            port,
            receive_buffer,
            sequence=0,
            duty_permille=0,
            duration_ms=0,
            timeout_s=timeout_s,
        )
        telemetry = wait_for_drive_telemetry(
            port,
            receive_buffer,
            timeout_s,
        )

    print("Encoder snapshot: PASS")
    print_encoder_snapshot(telemetry)


def run_encoder_monitor(
    port_name: str,
    seconds: float,
    revolutions: int,
    timeout_s: float,
) -> None:
    if not 0.5 <= seconds <= 60.0:
        raise SystemExit("Monitor duration must be from 0.5 to 60 seconds.")
    if not 1 <= revolutions <= 20:
        raise SystemExit("Revolutions must be from 1 to 20.")

    serial, _ = require_pyserial()
    receive_buffer = bytearray()

    with serial.Serial(
        port=port_name,
        baudrate=115200,
        bytesize=serial.EIGHTBITS,
        parity=serial.PARITY_NONE,
        stopbits=serial.STOPBITS_ONE,
        timeout=0.05,
    ) as port:
        port.reset_input_buffer()
        send_motor_test_request(
            port,
            receive_buffer,
            sequence=0,
            duty_permille=0,
            duration_ms=0,
            timeout_s=timeout_s,
        )
        baseline = wait_for_drive_telemetry(
            port,
            receive_buffer,
            timeout_s,
        )
        latest = baseline
        last_printed_count = baseline.encoder_count
        deadline = time.monotonic() + seconds

        print("Encoder monitor started; motor output is disabled.")
        print(
            "Rotate the wheel by hand in one direction, exactly "
            f"{revolutions} full wheel revolution(s)."
        )
        print(f"Start count: {baseline.encoder_count:+d}")

        while time.monotonic() < deadline:
            remaining = max(deadline - time.monotonic(), 0.05)
            try:
                latest = wait_for_drive_telemetry(
                    port,
                    receive_buffer,
                    min(remaining, 0.25),
                )
            except SystemExit:
                continue

            if latest.encoder_count != last_printed_count:
                delta = latest.encoder_count - baseline.encoder_count
                print(
                    f"count={latest.encoder_count:+d}, "
                    f"delta={delta:+d}"
                )
                last_printed_count = latest.encoder_count

    total_delta = latest.encoder_count - baseline.encoder_count
    print("Encoder monitor: COMPLETE")
    print(f"End count: {latest.encoder_count:+d}")
    print(f"Total delta: {total_delta:+d} counts")
    if total_delta > 0:
        print("Detected raw direction: POSITIVE")
    elif total_delta < 0:
        print("Detected raw direction: NEGATIVE")
    else:
        print("Detected raw direction: NONE (no encoder pulse detected)")
    if total_delta == 0:
        print("Calibration result: INVALID - no encoder pulse detected.")
    else:
        measured_counts_per_revolution = (
            encoder_delta_to_counts_per_revolution(
                total_delta,
                revolutions,
            )
        )
        difference_percent = (
            (
                measured_counts_per_revolution
                - ENCODER_COUNTS_PER_WHEEL_REV
            )
            / ENCODER_COUNTS_PER_WHEEL_REV
            * 100.0
        )
        print(
            "Measured calibration: "
            f"{measured_counts_per_revolution:.2f} counts/rev "
            f"from {revolutions} revolution(s)"
        )
        print(
            "Configured calibration: "
            f"{ENCODER_COUNTS_PER_WHEEL_REV:.0f} counts/rev "
            f"({difference_percent:+.2f}% difference)"
        )


def run_encoder_motor_test(
    port_name: str,
    percent: float,
    duration_ms: int,
    timeout_s: float,
) -> None:
    duty_permille, duration_ms = validate_motor_test_values(
        percent,
        duration_ms,
    )
    if duty_permille == 0:
        raise SystemExit("Encoder motor test requires a non-zero percent.")

    serial, _ = require_pyserial()
    receive_buffer = bytearray()
    active_samples: list[DriveTelemetry] = []

    with serial.Serial(
        port=port_name,
        baudrate=115200,
        bytesize=serial.EIGHTBITS,
        parity=serial.PARITY_NONE,
        stopbits=serial.STOPBITS_ONE,
        timeout=0.05,
    ) as port:
        port.reset_input_buffer()
        time.sleep(0.4)

        # Stop first, enable telemetry, and capture a stable baseline.
        send_motor_test_request(
            port,
            receive_buffer,
            sequence=0,
            duty_permille=0,
            duration_ms=0,
            timeout_s=timeout_s,
        )
        baseline = wait_for_drive_telemetry(
            port,
            receive_buffer,
            timeout_s,
        )

        send_motor_test_request(
            port,
            receive_buffer,
            sequence=1,
            duty_permille=duty_permille,
            duration_ms=duration_ms,
            timeout_s=timeout_s,
        )

        latest = baseline
        deadline = time.monotonic() + (duration_ms / 1000.0) + 0.35
        try:
            while time.monotonic() < deadline:
                remaining = max(deadline - time.monotonic(), 0.05)
                try:
                    latest = wait_for_drive_telemetry(
                        port,
                        receive_buffer,
                        min(remaining, 0.25),
                    )
                    if latest.motor_duty_permille == duty_permille:
                        active_samples.append(latest)
                except SystemExit:
                    continue
        finally:
            # Explicit stop is a second safety layer; firmware also auto-stops.
            send_motor_test_request(
                port,
                receive_buffer,
                sequence=2,
                duty_permille=0,
                duration_ms=0,
                timeout_s=timeout_s,
            )
            latest = wait_for_drive_telemetry(
                port,
                receive_buffer,
                timeout_s,
            )

    count_delta = latest.encoder_count - baseline.encoder_count
    print("Encoder motor test: COMPLETE")
    print(f"PWM request: {percent:+.1f}% for {duration_ms} ms")
    print(f"Start count: {baseline.encoder_count:+d}")
    print(f"End count: {latest.encoder_count:+d}")
    print(f"Count delta: {count_delta:+d}")

    if active_samples:
        steady_samples = active_samples[len(active_samples) // 2 :]
        average_speed_mm_s = sum(
            sample.measured_speed_mm_s for sample in steady_samples
        ) / len(steady_samples)
        average_wheel_rpm = speed_mm_s_to_wheel_rpm(
            average_speed_mm_s
        )
        peak_speed_mm_s = max(
            active_samples,
            key=lambda sample: abs(sample.measured_speed_mm_s),
        ).measured_speed_mm_s
        print(
            "Average measured speed (second half): "
            f"{average_speed_mm_s:+.1f} mm/s"
        )
        print(f"Average wheel speed: {average_wheel_rpm:+.2f} RPM")
        print(f"Peak measured speed: {peak_speed_mm_s:+d} mm/s")
    else:
        print(
            "Speed samples: WARNING - no telemetry sample reported the "
            "requested PWM while it was active."
        )

    if count_delta == 0:
        print("Direction result: FAIL - no encoder pulse was detected.")
    elif (count_delta > 0) == (percent > 0):
        print("Direction result: MATCH - PWM sign and encoder sign agree.")
    else:
        print(
            "Direction result: REVERSED - encoder direction sign must be "
            "inverted in firmware."
        )

    if latest.motor_duty_permille == 0:
        print("Automatic stop: VERIFIED (reported duty is 0.0%)")
    else:
        print(
            "Automatic stop: WARNING "
            f"(reported duty is {latest.motor_duty_permille / 10.0:+.1f}%)"
        )


def run_pid_test(
    port_name: str,
    target_speed_mm_s: int,
    duration_ms: int,
    timeout_s: float,
) -> None:
    raise SystemExit(
        "pid-test (legacy message 0xF4) is not implemented by the active "
        "Protocol V2 firmware. Use drive-test, which exercises the real "
        "CMD_DRIVE speed-PID path and its 20 Hz heartbeat."
    )

    if target_speed_mm_s == 0:
        raise SystemExit("PID test requires a non-zero target speed.")
    if abs(target_speed_mm_s) > MAX_PID_TEST_SPEED_MM_S:
        raise SystemExit(
            "PID test target is limited to "
            f"+/-{MAX_PID_TEST_SPEED_MM_S} mm/s."
        )
    if not MIN_PID_TEST_DURATION_MS <= duration_ms <= MAX_PID_TEST_DURATION_MS:
        raise SystemExit(
            "PID test duration must be between "
            f"{MIN_PID_TEST_DURATION_MS} and "
            f"{MAX_PID_TEST_DURATION_MS} ms."
        )

    serial, _ = require_pyserial()
    receive_buffer = bytearray()
    samples: list[DriveTelemetry] = []

    with serial.Serial(
        port=port_name,
        baudrate=115200,
        bytesize=serial.EIGHTBITS,
        parity=serial.PARITY_NONE,
        stopbits=serial.STOPBITS_ONE,
        timeout=0.05,
    ) as port:
        port.reset_input_buffer()

        # Establish a stopped baseline and turn on standard drive telemetry.
        send_motor_test_request(
            port,
            receive_buffer,
            sequence=0,
            duty_permille=0,
            duration_ms=0,
            timeout_s=timeout_s,
        )
        baseline = wait_for_drive_telemetry(
            port,
            receive_buffer,
            timeout_s,
        )

        send_pid_test_request(
            port,
            receive_buffer,
            sequence=1,
            target_speed_mm_s=target_speed_mm_s,
            duration_ms=duration_ms,
            timeout_s=timeout_s,
        )

        deadline = time.monotonic() + (duration_ms / 1000.0) + 0.35
        try:
            while time.monotonic() < deadline:
                remaining = max(deadline - time.monotonic(), 0.05)
                try:
                    telemetry = wait_for_drive_telemetry(
                        port,
                        receive_buffer,
                        min(remaining, 0.25),
                    )
                except SystemExit:
                    continue
                samples.append(telemetry)
        finally:
            # Explicit stop backs up the firmware's time-limited auto-stop.
            send_motor_test_request(
                port,
                receive_buffer,
                sequence=2,
                duty_permille=0,
                duration_ms=0,
                timeout_s=timeout_s,
            )
            stopped = wait_for_drive_telemetry(
                port,
                receive_buffer,
                timeout_s,
            )

    active_samples = [
        sample
        for sample in samples
        if sample.target_speed_mm_s == target_speed_mm_s
    ]
    rpm_samples: list[tuple[DriveTelemetry, float]] = []
    previous_sample = baseline

    for sample in active_samples:
        elapsed_ms = sample.mcu_time_ms - previous_sample.mcu_time_ms
        if elapsed_ms > 0:
            rpm_samples.append(
                (
                    sample,
                    encoder_delta_to_wheel_rpm(
                        sample.encoder_count - previous_sample.encoder_count,
                        elapsed_ms,
                    ),
                )
            )
        previous_sample = sample

    target_rpm = speed_mm_s_to_wheel_rpm(target_speed_mm_s)

    print("PID speed test: COMPLETE")
    print(
        f"Configured calibration: "
        f"{ENCODER_COUNTS_PER_WHEEL_REV:.0f} counts/rev, "
        f"{WHEEL_CIRCUMFERENCE_MM:.2f} mm circumference"
    )
    print(
        f"Target: {target_speed_mm_s:+d} mm/s "
        f"({target_rpm:+.2f} RPM) for {duration_ms} ms"
    )
    print(
        " time(ms)  target  measured  targetRPM  measuredRPM"
        "   PWM    encoder"
    )

    for sample, measured_rpm in rpm_samples:
        print(
            f" {sample.mcu_time_ms:8d} "
            f"{sample.target_speed_mm_s:+7d} "
            f"{sample.measured_speed_mm_s:+9d} "
            f"{target_rpm:+9.2f} "
            f"{measured_rpm:+11.2f} "
            f"{sample.motor_duty_permille / 10.0:+6.1f}% "
            f"{sample.encoder_count:+9d}"
        )

    if rpm_samples:
        steady_start = len(rpm_samples) // 2
        steady_samples = rpm_samples[steady_start:]
        average_speed = sum(
            sample.measured_speed_mm_s for sample, _ in steady_samples
        ) / len(steady_samples)
        average_error = target_speed_mm_s - average_speed
        average_rpm = sum(
            measured_rpm for _, measured_rpm in steady_samples
        ) / len(steady_samples)
        average_rpm_error = target_rpm - average_rpm
        peak_pwm = max(
            abs(sample.motor_duty_permille)
            for sample, _ in rpm_samples
        ) / 10.0
        count_delta = (
            rpm_samples[-1][0].encoder_count - baseline.encoder_count
        )

        print(f"Average measured speed (second half): {average_speed:+.1f} mm/s")
        print(f"Average tracking error: {average_error:+.1f} mm/s")
        print(f"Average wheel RPM (second half): {average_rpm:+.2f} RPM")
        print(f"Average RPM error: {average_rpm_error:+.2f} RPM")
        print(f"Peak absolute PWM: {peak_pwm:.1f}%")
        print(f"Encoder delta during test: {count_delta:+d}")

        if count_delta == 0:
            print("Encoder response: FAIL - no count change detected.")
        elif (count_delta > 0) == (target_speed_mm_s > 0):
            print("Encoder response: MATCH")
        else:
            print("Encoder response: REVERSED")
    else:
        print(
            "PID telemetry: FAIL - no active sample reported the requested "
            "target."
        )

    if stopped.motor_duty_permille == 0:
        print("Automatic stop: VERIFIED (reported duty is 0.0%)")
    else:
        print(
            "Automatic stop: WARNING "
            f"(reported duty is {stopped.motor_duty_permille / 10.0:+.1f}%)"
        )


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
        time.sleep(0.4)
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
                ):
                    expected_payload = (
                        bytes((request_sequence,)) + payload
                    )
                    if frame.payload != expected_payload:
                        raise SystemExit(
                            "Echo payload mismatch: "
                            f"expected={expected_payload!r}, "
                            f"received={frame.payload!r}"
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


def run_motor_test(
    port_name: str,
    percent: float,
    duration_ms: int,
    timeout_s: float,
) -> None:
    serial, _ = require_pyserial()
    duty_permille, duration_ms = validate_motor_test_values(
        percent,
        duration_ms,
    )

    request_sequence = 0
    request = encode_frame(
        MSG_DIAG_MOTOR_TEST_REQUEST,
        request_sequence,
        struct.pack("<hH", duty_permille, duration_ms),
        version=MOTOR_DIAGNOSTIC_PROTOCOL_VERSION,
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
        time.sleep(0.4)
        port.write(request)
        port.flush()

        while time.monotonic() < deadline:
            chunk = port.read(max(port.in_waiting, 1))
            if chunk:
                receive_buffer.extend(chunk)

            while True:
                frame = pop_frame(
                    receive_buffer,
                    expected_version=MOTOR_DIAGNOSTIC_PROTOCOL_VERSION,
                )
                if frame is None:
                    break

                if frame.message_id == MSG_DIAG_MOTOR_TEST_RESPONSE:
                    if len(frame.payload) != 6:
                        raise SystemExit(
                            "Motor-test response has an invalid payload."
                        )

                    (
                        response_request_sequence,
                        status,
                        accepted_permille,
                        accepted_duration_ms,
                    ) = struct.unpack(
                        "<BBhH",
                        frame.payload,
                    )
                    if response_request_sequence != request_sequence:
                        continue
                    if status != 0:
                        raise SystemExit(
                            "STM32 rejected motor-test: "
                            f"{servo_result_name(status)}."
                        )
                    if (
                        accepted_permille != duty_permille
                        or accepted_duration_ms != duration_ms
                    ):
                        raise SystemExit(
                            "Motor-test acknowledgement does not match the "
                            "request."
                        )

                    print("Motor test command: ACCEPTED")
                    print(
                        "PWM request: "
                        f"{accepted_permille / 10.0:+.1f}%"
                    )
                    if accepted_permille == 0:
                        print("Motor output: DISABLED")
                    else:
                        print(
                            "Automatic stop: "
                            f"{accepted_duration_ms} ms"
                        )
                    print(f"TX: {request.hex(' ').upper()}")
                    response = encode_frame(
                        frame.message_id,
                        frame.sequence,
                        frame.payload,
                        version=MOTOR_DIAGNOSTIC_PROTOCOL_VERSION,
                    )
                    print(f"RX: {response.hex(' ').upper()}")
                    return

    raise SystemExit(
        f"No valid DIAG_MOTOR_TEST_RESPONSE received from {port_name} "
        f"within {timeout_s:.1f}s."
    )


def build_argument_parser() -> argparse.ArgumentParser:
    parser = argparse.ArgumentParser(
        description="Ssacurity STM32 UART Protocol V2 PC/Jetson test tool"
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
        help="UTF-8 text, at most 31 encoded bytes",
    )
    echo_parser.add_argument(
        "--timeout",
        type=float,
        default=2.0,
        help="response timeout in seconds",
    )

    motor_parser = subparsers.add_parser(
        "motor-test",
        help="run a time-limited open-loop motor output test",
    )
    motor_parser.add_argument("--port", required=True, help="COM port, e.g. COM5")
    motor_parser.add_argument(
        "--percent",
        required=True,
        type=float,
        help="PWM percent: 0 (stop) or +/-20.0 to +/-60.0",
    )
    motor_parser.add_argument(
        "--duration",
        required=True,
        type=int,
        help="automatic stop time in ms, from 100 to 10000",
    )
    motor_parser.add_argument(
        "--timeout",
        type=float,
        default=2.0,
        help="response timeout in seconds",
    )

    encoder_read_parser = subparsers.add_parser(
        "encoder-read",
        help="read the current raw encoder count and control telemetry",
    )
    encoder_read_parser.add_argument(
        "--port",
        required=True,
        help="COM port, e.g. COM5",
    )
    encoder_read_parser.add_argument(
        "--timeout",
        type=float,
        default=2.0,
        help="response timeout in seconds",
    )

    encoder_monitor_parser = subparsers.add_parser(
        "encoder-monitor",
        help="monitor raw counts while the wheel is rotated by hand",
    )
    encoder_monitor_parser.add_argument(
        "--port",
        required=True,
        help="COM port, e.g. COM5",
    )
    encoder_monitor_parser.add_argument(
        "--seconds",
        type=float,
        default=10.0,
        help="monitor duration from 0.5 to 60 seconds",
    )
    encoder_monitor_parser.add_argument(
        "--revolutions",
        type=int,
        default=1,
        help="number of full wheel revolutions, from 1 to 20",
    )
    encoder_monitor_parser.add_argument(
        "--timeout",
        type=float,
        default=2.0,
        help="response timeout in seconds",
    )

    encoder_test_parser = subparsers.add_parser(
        "encoder-test",
        help="run a motor pulse and verify encoder count direction",
    )
    encoder_test_parser.add_argument(
        "--port",
        required=True,
        help="COM port, e.g. COM5",
    )
    encoder_test_parser.add_argument(
        "--percent",
        required=True,
        type=float,
        help="PWM percent from -95.0 to -20.0 or +20.0 to +95.0",
    )
    encoder_test_parser.add_argument(
        "--duration",
        required=True,
        type=int,
        help="automatic stop time in ms, from 100 to 10000",
    )
    encoder_test_parser.add_argument(
        "--timeout",
        type=float,
        default=2.0,
        help="response timeout in seconds",
    )

    steering_monitor_parser = subparsers.add_parser(
        "steering-monitor",
        help="legacy command; unavailable until steering telemetry is added",
    )
    steering_monitor_parser.add_argument(
        "--port",
        required=True,
        help="COM port, e.g. COM5",
    )
    steering_monitor_parser.add_argument(
        "--seconds",
        type=float,
        default=10.0,
        help="monitor duration from 0.5 to 60 seconds",
    )
    steering_monitor_parser.add_argument(
        "--timeout",
        type=float,
        default=2.0,
        help="response timeout in seconds",
    )

    servo_calibrate_parser = subparsers.add_parser(
        "servo-calibrate",
        help="apply a guarded raw steering-servo pulse for calibration",
    )
    servo_calibrate_parser.add_argument(
        "--port",
        required=True,
        help="COM port, e.g. COM5",
    )
    servo_calibrate_parser.add_argument(
        "--pulse-us",
        required=True,
        type=int,
        help="guarded pulse from 766 to 1696 us",
    )
    servo_calibrate_parser.add_argument(
        "--seconds",
        type=float,
        default=3.0,
        help="hold duration from 0.5 to 10 seconds",
    )
    servo_calibrate_parser.add_argument(
        "--timeout",
        type=float,
        default=2.0,
        help="response timeout in seconds",
    )

    servo_off_parser = subparsers.add_parser(
        "servo-off",
        help="stop the steering-servo PWM diagnostic output",
    )
    servo_off_parser.add_argument(
        "--port",
        required=True,
        help="COM port, e.g. COM5",
    )
    servo_off_parser.add_argument(
        "--timeout",
        type=float,
        default=2.0,
        help="response timeout in seconds",
    )

    drive_test_parser = subparsers.add_parser(
        "drive-test",
        help="exercise the real CMD_DRIVE speed/steering/telemetry path",
    )
    drive_test_parser.add_argument(
        "--port",
        required=True,
        help="COM port, e.g. COM11",
    )
    drive_test_parser.add_argument(
        "--speed-mm-s",
        required=True,
        type=int,
        help="target speed from -1565 to +1565 mm/s",
    )
    drive_test_parser.add_argument(
        "--steering-deg",
        required=True,
        type=float,
        help=(
            "center-equivalent steering from -28.69 (right) "
            "to +19.55 deg (left)"
        ),
    )
    drive_test_parser.add_argument(
        "--seconds",
        type=float,
        default=1.5,
        help="command duration from 0.5 to 5.0 seconds",
    )
    drive_test_parser.add_argument(
        "--verify-watchdog",
        action="store_true",
        help="stop heartbeat and verify the 300 ms link-loss stop",
    )
    drive_test_parser.add_argument(
        "--wheels-off-ground",
        action="store_true",
        help="confirm all driven wheels are physically off the ground",
    )
    drive_test_parser.add_argument(
        "--timeout",
        type=float,
        default=2.0,
        help="response timeout in seconds",
    )

    drive_scenario_parser = subparsers.add_parser(
        "drive-scenario",
        help="run basic or extended driving/watchdog integration steps",
    )
    drive_scenario_parser.add_argument(
        "--port",
        required=True,
        help="COM port, e.g. COM11",
    )
    drive_scenario_parser.add_argument(
        "--wheels-off-ground",
        action="store_true",
        help="confirm all driven wheels are physically off the ground",
    )
    drive_scenario_parser.add_argument(
        "--profile",
        choices=("basic", "extended"),
        default="basic",
        help="basic quick test or extended speed/steering matrix",
    )
    drive_scenario_parser.add_argument(
        "--timeout",
        type=float,
        default=2.0,
        help="response timeout in seconds",
    )

    steering_sweep_parser = subparsers.add_parser(
        "steering-sweep",
        help="sweep every measured point across the full seven-point LUT",
    )
    steering_sweep_parser.add_argument(
        "--port",
        required=True,
        help="COM port, e.g. COM11",
    )
    steering_sweep_parser.add_argument(
        "--hold-seconds",
        type=float,
        default=0.7,
        help="hold each angle from 0.5 to 5.0 seconds",
    )
    steering_sweep_parser.add_argument(
        "--wheels-off-ground",
        action="store_true",
        help="confirm all driven wheels are physically off the ground",
    )
    steering_sweep_parser.add_argument(
        "--timeout",
        type=float,
        default=2.0,
        help="response timeout in seconds",
    )

    steering_full_sweep_parser = subparsers.add_parser(
        "steering-full-sweep",
        help="alternate the calibrated +19.55/-28.69 degree endpoints",
    )
    steering_full_sweep_parser.add_argument(
        "--port",
        required=True,
        help="COM port, e.g. COM11",
    )
    steering_full_sweep_parser.add_argument(
        "--cycles",
        type=int,
        default=3,
        help="number of endpoint-to-endpoint cycles, from 1 to 10",
    )
    steering_full_sweep_parser.add_argument(
        "--hold-seconds",
        type=float,
        default=1.2,
        help="hold each endpoint from 0.5 to 5.0 seconds",
    )
    steering_full_sweep_parser.add_argument(
        "--wheels-off-ground",
        action="store_true",
        help="confirm all driven wheels are physically off the ground",
    )
    steering_full_sweep_parser.add_argument(
        "--timeout",
        type=float,
        default=2.0,
        help="response timeout in seconds",
    )

    telemetry_monitor_parser = subparsers.add_parser(
        "telemetry-monitor",
        help="decode the values currently sent from STM32 to Jetson",
    )
    telemetry_monitor_parser.add_argument(
        "--port",
        required=True,
        help="COM port, e.g. COM11",
    )
    telemetry_monitor_parser.add_argument(
        "--seconds",
        type=float,
        default=5.0,
        help="monitor duration from 0.5 to 60 seconds",
    )

    imu_monitor_parser = subparsers.add_parser(
        "imu-monitor",
        help="listen only and print every transmitted BNO085 value and angle",
    )
    imu_monitor_parser.add_argument(
        "--port",
        required=True,
        help="COM port, e.g. COM11",
    )
    imu_monitor_parser.add_argument(
        "--seconds",
        type=float,
        default=30.0,
        help="observation duration from 5 to 300 seconds",
    )
    imu_monitor_parser.add_argument(
        "--settle-seconds",
        type=float,
        default=3.0,
        help="initial stationary interval used to zero delta_yaw",
    )
    imu_monitor_parser.add_argument(
        "--rate-hz",
        type=float,
        default=2.0,
        help="terminal print rate from 0.5 to 20 Hz",
    )
    imu_monitor_parser.add_argument(
        "--csv",
        help="optional path for all received 20 Hz samples, e.g. imu_test.csv",
    )

    imu_calibration_parser = subparsers.add_parser(
        "imu-calibration-test",
        help="listen only and verify that BNO085 gyro accuracy reaches g1+",
    )
    imu_calibration_parser.add_argument(
        "--port",
        required=True,
        help="COM port, e.g. COM11",
    )
    imu_calibration_parser.add_argument(
        "--seconds",
        type=float,
        default=15.0,
        help="stationary observation duration from 5 to 60 seconds",
    )
    imu_calibration_parser.add_argument(
        "--settle-seconds",
        type=float,
        default=3.0,
        help="initial samples to exclude from the pass/fail decision",
    )

    odometry_test_parser = subparsers.add_parser(
        "odometry-test",
        help="verify command-estimated V2 odometry signs and transport",
    )
    odometry_test_parser.add_argument(
        "--port",
        required=True,
        help="COM port, e.g. COM11",
    )
    odometry_test_parser.add_argument(
        "--wheels-off-ground",
        action="store_true",
        help="confirm all driven wheels are physically off the ground",
    )
    odometry_test_parser.add_argument(
        "--timeout",
        type=float,
        default=2.0,
        help="response timeout in seconds",
    )

    pid_test_parser = subparsers.add_parser(
        "pid-test",
        help="legacy command; use drive-test with the active V2 firmware",
    )
    pid_test_parser.add_argument(
        "--port",
        required=True,
        help="COM port, e.g. COM5",
    )
    pid_test_parser.add_argument(
        "--target-mm-s",
        required=True,
        type=int,
        help="target wheel linear speed from -300 to +300 mm/s",
    )
    pid_test_parser.add_argument(
        "--duration",
        required=True,
        type=int,
        help="automatic stop time in ms, from 500 to 5000",
    )
    pid_test_parser.add_argument(
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
    elif args.command == "motor-test":
        run_motor_test(
            args.port,
            args.percent,
            args.duration,
            args.timeout,
        )
    elif args.command == "encoder-read":
        run_encoder_read(args.port, args.timeout)
    elif args.command == "encoder-monitor":
        run_encoder_monitor(
            args.port,
            args.seconds,
            args.revolutions,
            args.timeout,
        )
    elif args.command == "encoder-test":
        run_encoder_motor_test(
            args.port,
            args.percent,
            args.duration,
            args.timeout,
        )
    elif args.command == "steering-monitor":
        run_steering_monitor(
            args.port,
            args.seconds,
            args.timeout,
        )
    elif args.command == "servo-calibrate":
        run_servo_calibrate(
            args.port,
            args.pulse_us,
            args.seconds,
            args.timeout,
        )
    elif args.command == "servo-off":
        run_servo_off(args.port, args.timeout)
    elif args.command == "drive-test":
        run_drive_test(
            args.port,
            args.speed_mm_s,
            args.steering_deg,
            args.seconds,
            args.timeout,
            args.wheels_off_ground,
            args.verify_watchdog,
        )
    elif args.command == "drive-scenario":
        run_drive_scenario(
            args.port,
            args.timeout,
            args.wheels_off_ground,
            args.profile,
        )
    elif args.command == "steering-sweep":
        run_steering_sweep(
            args.port,
            args.timeout,
            args.wheels_off_ground,
            args.hold_seconds,
        )
    elif args.command == "steering-full-sweep":
        run_steering_full_sweep(
            args.port,
            args.timeout,
            args.wheels_off_ground,
            args.hold_seconds,
            args.cycles,
        )
    elif args.command == "telemetry-monitor":
        run_telemetry_monitor(
            args.port,
            args.seconds,
        )
    elif args.command == "imu-monitor":
        run_imu_monitor(
            args.port,
            args.seconds,
            args.settle_seconds,
            args.rate_hz,
            args.csv,
        )
    elif args.command == "imu-calibration-test":
        run_imu_calibration_test(
            args.port,
            args.seconds,
            args.settle_seconds,
        )
    elif args.command == "odometry-test":
        run_odometry_test(
            args.port,
            args.timeout,
            args.wheels_off_ground,
        )
    elif args.command == "pid-test":
        run_pid_test(
            args.port,
            args.target_mm_s,
            args.duration,
            args.timeout,
        )
    else:
        raise AssertionError(args.command)

    return 0


if __name__ == "__main__":
    sys.exit(main())
