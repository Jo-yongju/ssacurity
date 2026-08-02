#!/usr/bin/env python3
"""
STM32 속도 PID 20초 테스트 및 곡선 기록 프로그램

동작
- PID 시작을 누르면 먼저 중립 명령을 보내 STM32 재주행 잠금을 해제한다.
- 목표 속도 CMD_DRIVE를 50 ms마다 20초 동안 반복 전송한다.
- 20초가 지나면 drive_enable=0 중립 명령을 보내 자동 정지한다.
- 정지 뒤 1초 동안 감속 데이터를 추가로 기록한다.
- 목표 속도, 측정 속도, PWM을 실시간 그래프로 표시하고 CSV로 저장한다.

설치
    py -m pip install pyserial matplotlib

실행
    py motor_pid_20s_tester.py
"""

from __future__ import annotations

import csv
import queue
import struct
import threading
import time
import tkinter as tk
from dataclasses import dataclass
from tkinter import filedialog, messagebox, ttk
from typing import Optional

import serial
from serial.tools import list_ports

import matplotlib

matplotlib.use("TkAgg")
from matplotlib.backends.backend_tkagg import FigureCanvasTkAgg
from matplotlib.figure import Figure


# -----------------------------------------------------------------------------
# UART protocol
# -----------------------------------------------------------------------------
SOF1 = 0xAA
SOF2 = 0x55
VERSION = 0x01
MAX_PAYLOAD = 64

MSG_CMD_DRIVE = 0x01
MSG_TELEMETRY_DRIVE = 0x80

BAUD = 115200
COMMAND_PERIOD_MS = 50       # CMD_DRIVE 20 Hz
GUI_PERIOD_MS = 100
TEST_DURATION_S = 20.0       # 실제 목표 속도 명령 유지 시간
STOP_TAIL_S = 1.0            # 정지 후 감속 곡선 기록 시간
REARM_DELAY_MS = 150         # 중립 명령 처리 후 PID 명령 시작 대기

# STM32 CommService_SetDriveLimits(500, 3000)에 맞춘 속도 제한
MAX_ABS_SPEED_MM_S = 500

# 현재 STM32 CMD_DRIVE payload는 정확히 5바이트다.
# int16 target_speed_mm_s
# int16 target_steering_cdeg
# uint8 drive_enable
CMD_DRIVE_FMT = "<hhB"
CMD_DRIVE_LEN = struct.calcsize(CMD_DRIVE_FMT)

# CommDriveTelemetry payload, 21바이트
# int16 target_speed_mm_s
# int16 measured_speed_mm_s
# int32 encoder_count
# int16 motor_duty_permille
# int16 steering_cdeg
# uint8 state
# uint32 active_fault_bits
# uint32 uptime_ms
TELEMETRY_FMT = "<hhihhBII"
TELEMETRY_LEN = struct.calcsize(TELEMETRY_FMT)

STATE_NAMES = {
    0: "BOOT",
    1: "SELF_TEST",
    2: "READY",
    3: "DRIVING",
    4: "SAFE_STOP",
    5: "FAULT",
}


def crc16_ccitt_false(data: bytes) -> int:
    """CRC-16/CCITT-FALSE: poly=0x1021, init=0xFFFF."""
    crc = 0xFFFF
    for byte in data:
        crc ^= byte << 8
        for _ in range(8):
            if crc & 0x8000:
                crc = ((crc << 1) ^ 0x1021) & 0xFFFF
            else:
                crc = (crc << 1) & 0xFFFF
    return crc


def build_frame(message_id: int, sequence: int, payload: bytes) -> bytes:
    """AA 55 | VERSION MSG_ID SEQ LEN PAYLOAD | CRC_L CRC_H."""
    if len(payload) > MAX_PAYLOAD:
        raise ValueError("payload가 너무 깁니다.")

    crc_input = bytes(
        (VERSION, message_id & 0xFF, sequence & 0xFF, len(payload))
    ) + payload
    crc = crc16_ccitt_false(crc_input)
    return bytes((SOF1, SOF2)) + crc_input + struct.pack("<H", crc)


class FrameParser:
    def __init__(self) -> None:
        self.buffer = bytearray()
        self.valid_frames = 0
        self.crc_errors = 0
        self.discarded_bytes = 0

    def feed(self, data: bytes) -> list[tuple[int, int, bytes]]:
        self.buffer.extend(data)
        frames: list[tuple[int, int, bytes]] = []

        while True:
            sof_index = self.buffer.find(bytes((SOF1, SOF2)))

            if sof_index < 0:
                if self.buffer and self.buffer[-1] == SOF1:
                    self.discarded_bytes += max(0, len(self.buffer) - 1)
                    self.buffer[:] = self.buffer[-1:]
                else:
                    self.discarded_bytes += len(self.buffer)
                    self.buffer.clear()
                break

            if sof_index > 0:
                self.discarded_bytes += sof_index
                del self.buffer[:sof_index]

            # SOF 2 + version/msg/seq/len 4 + CRC 2
            if len(self.buffer) < 8:
                break

            version, msg_id, seq, payload_len = self.buffer[2:6]

            if payload_len > MAX_PAYLOAD:
                del self.buffer[0]
                continue

            frame_len = 8 + payload_len
            if len(self.buffer) < frame_len:
                break

            raw = bytes(self.buffer[:frame_len])
            del self.buffer[:frame_len]

            received_crc = struct.unpack_from("<H", raw, 6 + payload_len)[0]
            calculated_crc = crc16_ccitt_false(raw[2:6 + payload_len])

            if received_crc != calculated_crc:
                self.crc_errors += 1
                continue

            if version != VERSION:
                continue

            self.valid_frames += 1
            frames.append((msg_id, seq, raw[6:6 + payload_len]))

        return frames


@dataclass
class Sample:
    time_s: float
    target_speed_mm_s: int
    measured_speed_mm_s: int
    measured_speed_m_s: float
    pwm_percent: float
    encoder_count: int
    steering_cdeg: int
    state: int
    fault_bits: int
    uptime_ms: int


class SerialReader(threading.Thread):
    def __init__(
        self,
        serial_port: serial.Serial,
        output_queue: queue.Queue,
        stop_event: threading.Event,
    ) -> None:
        super().__init__(daemon=True)
        self.serial_port = serial_port
        self.output_queue = output_queue
        self.stop_event = stop_event
        self.parser = FrameParser()

    def run(self) -> None:
        try:
            while not self.stop_event.is_set():
                waiting = self.serial_port.in_waiting
                data = self.serial_port.read(waiting if waiting > 0 else 1)

                if not data:
                    continue

                for frame in self.parser.feed(data):
                    self.output_queue.put(("frame", frame))

                self.output_queue.put(
                    (
                        "stats",
                        (
                            self.parser.valid_frames,
                            self.parser.crc_errors,
                            self.parser.discarded_bytes,
                        ),
                    )
                )
        except Exception as exc:  # serial disconnect, OS error, etc.
            self.output_queue.put(("error", str(exc)))


class PidTestApp:
    def __init__(self, root: tk.Tk) -> None:
        self.root = root
        self.root.title("STM32 속도 PID 20초 테스트")
        self.root.geometry("1220x800")

        self.serial_port: Optional[serial.Serial] = None
        self.reader: Optional[SerialReader] = None
        self.stop_event = threading.Event()
        self.rx_queue: queue.Queue = queue.Queue()

        self.recording = False
        self.samples: list[Sample] = []
        self.record_start_uptime: Optional[int] = None
        self.last_rx_monotonic = 0.0

        self.tx_sequence = 0
        self.test_running = False
        self.test_target_speed_mm_s = 300
        self.test_end_monotonic = 0.0
        self.command_after_id: Optional[str] = None
        self.start_after_id: Optional[str] = None
        self.finish_after_id: Optional[str] = None

        self.port_var = tk.StringVar()
        self.target_speed_var = tk.StringVar(value="300")
        self.connection_var = tk.StringVar(value="연결 안 됨")
        self.test_status_var = tk.StringVar(value="대기")
        self.recording_var = tk.StringVar(value="기록 정지")
        self.status_var = tk.StringVar(value="텔레메트리 대기")
        self.sample_count_var = tk.StringVar(value="0 samples")
        self.remaining_var = tk.StringVar(value="20.0 s")

        self.live_target_var = tk.StringVar(value="0.000 m/s")
        self.live_speed_var = tk.StringVar(value="0.000 m/s")
        self.live_pwm_var = tk.StringVar(value="0.0 %")
        self.live_encoder_var = tk.StringVar(value="0")
        self.live_state_var = tk.StringVar(value="-")
        self.live_fault_var = tk.StringVar(value="0x00000000")

        self._build_ui()
        self.refresh_ports()

        self.root.after(GUI_PERIOD_MS, self.process_rx_queue)
        self.root.after(GUI_PERIOD_MS, self.update_plot)
        self.root.protocol("WM_DELETE_WINDOW", self.close)

    # ------------------------------------------------------------------ UI
    def _build_ui(self) -> None:
        main = ttk.Frame(self.root, padding=10)
        main.pack(fill=tk.BOTH, expand=True)

        connection_frame = ttk.LabelFrame(main, text="STM32 연결", padding=8)
        connection_frame.pack(fill=tk.X)

        ttk.Label(connection_frame, text="COM 포트").pack(side=tk.LEFT)

        self.port_box = ttk.Combobox(
            connection_frame,
            textvariable=self.port_var,
            width=38,
            state="readonly",
        )
        self.port_box.pack(side=tk.LEFT, padx=6)

        ttk.Button(
            connection_frame,
            text="새로고침",
            command=self.refresh_ports,
        ).pack(side=tk.LEFT, padx=3)

        self.connect_button = ttk.Button(
            connection_frame,
            text="연결",
            command=self.toggle_connection,
        )
        self.connect_button.pack(side=tk.LEFT, padx=3)

        ttk.Label(
            connection_frame,
            textvariable=self.connection_var,
        ).pack(side=tk.LEFT, padx=12)

        ttk.Label(
            connection_frame,
            textvariable=self.status_var,
        ).pack(side=tk.RIGHT)

        control_frame = ttk.LabelFrame(main, text="속도 PID 20초 시험", padding=8)
        control_frame.pack(fill=tk.X, pady=(8, 0))

        ttk.Label(control_frame, text="목표 속도").pack(side=tk.LEFT)
        ttk.Entry(
            control_frame,
            textvariable=self.target_speed_var,
            width=8,
            justify=tk.RIGHT,
        ).pack(side=tk.LEFT, padx=(5, 2))
        ttk.Label(control_frame, text="mm/s").pack(side=tk.LEFT, padx=(0, 12))

        self.start_button = ttk.Button(
            control_frame,
            text="PID 시작 (20초)",
            command=self.start_pid_test,
        )
        self.start_button.pack(side=tk.LEFT, padx=3)

        self.stop_button = ttk.Button(
            control_frame,
            text="즉시 정지",
            command=self.stop_pid_test,
            state=tk.DISABLED,
        )
        self.stop_button.pack(side=tk.LEFT, padx=3)

        ttk.Button(
            control_frame,
            text="기록 초기화",
            command=self.clear_recording,
        ).pack(side=tk.LEFT, padx=(14, 3))

        ttk.Button(
            control_frame,
            text="CSV 저장",
            command=self.save_csv,
        ).pack(side=tk.LEFT, padx=3)

        ttk.Label(control_frame, text="남은 시간:").pack(side=tk.LEFT, padx=(20, 3))
        ttk.Label(
            control_frame,
            textvariable=self.remaining_var,
            font=("TkDefaultFont", 10, "bold"),
        ).pack(side=tk.LEFT)

        ttk.Label(
            control_frame,
            textvariable=self.test_status_var,
        ).pack(side=tk.RIGHT, padx=10)

        ttk.Label(
            control_frame,
            textvariable=self.sample_count_var,
        ).pack(side=tk.RIGHT)

        live_frame = ttk.LabelFrame(main, text="실시간 값", padding=8)
        live_frame.pack(fill=tk.X, pady=(8, 0))

        live_items = [
            ("목표 속도", self.live_target_var),
            ("측정 속도", self.live_speed_var),
            ("PWM", self.live_pwm_var),
            ("엔코더", self.live_encoder_var),
            ("상태", self.live_state_var),
            ("Fault", self.live_fault_var),
        ]

        for index, (title, variable) in enumerate(live_items):
            card = ttk.Frame(live_frame, padding=6)
            card.grid(row=0, column=index, sticky="nsew")

            ttk.Label(card, text=title).pack()
            ttk.Label(
                card,
                textvariable=variable,
                font=("TkDefaultFont", 11, "bold"),
            ).pack()

            live_frame.columnconfigure(index, weight=1)

        plot_frame = ttk.LabelFrame(
            main,
            text="PID 응답 곡선: 목표 속도 / 측정 속도 / PWM",
            padding=4,
        )
        plot_frame.pack(fill=tk.BOTH, expand=True, pady=(8, 0))

        self.figure = Figure(figsize=(10, 5.7), dpi=100)
        self.speed_axis = self.figure.add_subplot(211)
        self.pwm_axis = self.figure.add_subplot(212)

        self.canvas = FigureCanvasTkAgg(self.figure, master=plot_frame)
        self.canvas.get_tk_widget().pack(fill=tk.BOTH, expand=True)

    # ------------------------------------------------------------- connection
    def refresh_ports(self) -> None:
        ports = list(list_ports.comports())
        values = [f"{port.device} — {port.description}" for port in ports]
        self.port_box["values"] = values

        if values and not self.port_var.get():
            self.port_box.current(0)

    def selected_port_name(self) -> str:
        return self.port_var.get().split(" — ", 1)[0].strip()

    def toggle_connection(self) -> None:
        if self.serial_port and self.serial_port.is_open:
            self.disconnect()
        else:
            self.connect()

    def connect(self) -> None:
        port_name = self.selected_port_name()

        if not port_name:
            messagebox.showwarning("연결", "COM 포트를 선택하세요.")
            return

        try:
            self.serial_port = serial.Serial(
                port=port_name,
                baudrate=BAUD,
                bytesize=serial.EIGHTBITS,
                parity=serial.PARITY_NONE,
                stopbits=serial.STOPBITS_ONE,
                timeout=0.05,
                write_timeout=0.2,
            )
            self.serial_port.reset_input_buffer()
            self.serial_port.reset_output_buffer()
        except Exception as exc:
            self.serial_port = None
            messagebox.showerror("연결 실패", str(exc))
            return

        self.stop_event = threading.Event()
        self.reader = SerialReader(
            self.serial_port,
            self.rx_queue,
            self.stop_event,
        )
        self.reader.start()

        self.tx_sequence = 0
        self.connection_var.set(f"{port_name} 연결됨 · 115200 8N1")
        self.connect_button.configure(text="연결 해제")
        self.test_status_var.set("연결됨 · PID 시험 가능")

    def disconnect(self) -> None:
        self._cancel_scheduled_callbacks()

        if self.serial_port and self.serial_port.is_open:
            try:
                self._send_drive_command(0, 0, False)
            except Exception:
                pass

        self.test_running = False
        self.recording = False
        self.recording_var.set("기록 정지")
        self.start_button.configure(state=tk.NORMAL)
        self.stop_button.configure(state=tk.DISABLED)

        self.stop_event.set()

        if self.reader and self.reader.is_alive():
            self.reader.join(timeout=0.3)

        if self.serial_port:
            try:
                self.serial_port.close()
            except Exception:
                pass

        self.serial_port = None
        self.reader = None
        self.connection_var.set("연결 안 됨")
        self.connect_button.configure(text="연결")
        self.test_status_var.set("대기")

    # --------------------------------------------------------------- TX logic
    def _next_sequence(self) -> int:
        sequence = self.tx_sequence
        self.tx_sequence = (self.tx_sequence + 1) & 0xFF
        return sequence

    def _send_drive_command(
        self,
        target_speed_mm_s: int,
        target_steering_cdeg: int,
        drive_enable: bool,
    ) -> None:
        if not self.serial_port or not self.serial_port.is_open:
            raise RuntimeError("STM32가 연결되어 있지 않습니다.")

        payload = struct.pack(
            CMD_DRIVE_FMT,
            int(target_speed_mm_s),
            int(target_steering_cdeg),
            1 if drive_enable else 0,
        )
        if len(payload) != CMD_DRIVE_LEN:
            raise RuntimeError("CMD_DRIVE payload 길이 오류")

        frame = build_frame(MSG_CMD_DRIVE, self._next_sequence(), payload)
        written = self.serial_port.write(frame)
        if written != len(frame):
            raise serial.SerialTimeoutException("CMD_DRIVE 프레임 일부만 전송됨")

    def start_pid_test(self) -> None:
        if not self.serial_port or not self.serial_port.is_open:
            messagebox.showwarning("PID 시험", "STM32에 먼저 연결하세요.")
            return

        if self.test_running:
            return

        try:
            target_speed = int(self.target_speed_var.get().strip())
        except ValueError:
            messagebox.showwarning("목표 속도", "목표 속도를 정수 mm/s로 입력하세요.")
            return

        if target_speed == 0:
            messagebox.showwarning("목표 속도", "0이 아닌 목표 속도를 입력하세요.")
            return

        if abs(target_speed) > MAX_ABS_SPEED_MM_S:
            messagebox.showwarning(
                "목표 속도",
                f"현재 STM32 제한에 맞춰 -{MAX_ABS_SPEED_MM_S}~"
                f"{MAX_ABS_SPEED_MM_S} mm/s 범위로 입력하세요.",
            )
            return

        self._cancel_scheduled_callbacks()
        self.samples.clear()
        self.record_start_uptime = None
        self.recording = True
        self.recording_var.set("● 기록 중")
        self.sample_count_var.set("0 samples")

        self.test_running = True
        self.test_target_speed_mm_s = target_speed
        self.remaining_var.set(f"{TEST_DURATION_S:.1f} s")
        self.test_status_var.set("중립 명령 전송 · 재주행 준비")
        self.start_button.configure(state=tk.DISABLED)
        self.stop_button.configure(state=tk.NORMAL)

        try:
            # STM32 CommService는 부팅/Timeout 직후 rearm_required=true이다.
            # 먼저 0, 0, drive_enable=0 중립 명령을 보내야 다음 PID 명령이 허용된다.
            self._send_drive_command(0, 0, False)
        except Exception as exc:
            self._abort_test_with_error(str(exc))
            return

        self.start_after_id = self.root.after(
            REARM_DELAY_MS,
            self._begin_active_test,
        )

    def _begin_active_test(self) -> None:
        self.start_after_id = None
        if not self.test_running:
            return

        self.test_end_monotonic = time.monotonic() + TEST_DURATION_S
        self.test_status_var.set(
            f"PID 주행 중 · 목표 {self.test_target_speed_mm_s} mm/s"
        )
        self._command_tick()

    def _command_tick(self) -> None:
        self.command_after_id = None
        if not self.test_running:
            return

        remaining = self.test_end_monotonic - time.monotonic()
        if remaining <= 0.0:
            self.stop_pid_test(auto=True)
            return

        try:
            self._send_drive_command(
                self.test_target_speed_mm_s,
                0,
                True,
            )
        except Exception as exc:
            self._abort_test_with_error(str(exc))
            return

        self.remaining_var.set(f"{remaining:.1f} s")
        self.command_after_id = self.root.after(
            COMMAND_PERIOD_MS,
            self._command_tick,
        )

    def stop_pid_test(self, auto: bool = False) -> None:
        if not self.test_running:
            return

        self.test_running = False
        self._cancel_command_callbacks_only()

        try:
            if self.serial_port and self.serial_port.is_open:
                # CMD_STOP 대신 중립 CMD_DRIVE를 사용한다.
                # 다음 시험에서 별도의 fault reset 없이 다시 시작할 수 있다.
                self._send_drive_command(0, 0, False)
        except Exception as exc:
            self.status_var.set(f"정지 명령 전송 실패: {exc}")

        self.remaining_var.set("0.0 s")
        self.stop_button.configure(state=tk.DISABLED)
        self.test_status_var.set(
            "20초 완료 · 정지 곡선 기록 중" if auto
            else "수동 정지 · 정지 곡선 기록 중"
        )

        # 정지 직후 데이터를 1초 더 받아 감속과 PWM 0을 그래프에 남긴다.
        self.finish_after_id = self.root.after(
            int(STOP_TAIL_S * 1000),
            self._finish_test_recording,
        )

    def _finish_test_recording(self) -> None:
        self.finish_after_id = None
        self.recording = False
        self.recording_var.set("기록 정지")
        self.start_button.configure(state=tk.NORMAL)
        self.test_status_var.set("시험 완료 · CSV 저장 가능")

    def _abort_test_with_error(self, error_text: str) -> None:
        self.test_running = False
        self.recording = False
        self._cancel_scheduled_callbacks()
        self.recording_var.set("기록 정지")
        self.start_button.configure(state=tk.NORMAL)
        self.stop_button.configure(state=tk.DISABLED)
        self.test_status_var.set("시험 중단")
        messagebox.showerror("PID 시험 오류", error_text)

    def _cancel_command_callbacks_only(self) -> None:
        for attribute in ("command_after_id", "start_after_id"):
            callback_id = getattr(self, attribute)
            if callback_id is not None:
                try:
                    self.root.after_cancel(callback_id)
                except tk.TclError:
                    pass
                setattr(self, attribute, None)

    def _cancel_scheduled_callbacks(self) -> None:
        self._cancel_command_callbacks_only()
        if self.finish_after_id is not None:
            try:
                self.root.after_cancel(self.finish_after_id)
            except tk.TclError:
                pass
            self.finish_after_id = None

    # --------------------------------------------------------------- RX logic
    def process_rx_queue(self) -> None:
        try:
            while True:
                kind, value = self.rx_queue.get_nowait()

                if kind == "frame":
                    self.handle_frame(*value)

                elif kind == "stats":
                    valid, crc_errors, discarded = value
                    self.status_var.set(
                        f"frames={valid}  crc={crc_errors}  discarded={discarded}"
                    )

                elif kind == "error":
                    messagebox.showerror("시리얼 오류", value)
                    self.disconnect()
                    break

        except queue.Empty:
            pass

        if (
            self.serial_port
            and self.last_rx_monotonic
            and time.monotonic() - self.last_rx_monotonic > 0.5
        ):
            self.status_var.set("텔레메트리 500ms 이상 없음")

        self.root.after(GUI_PERIOD_MS, self.process_rx_queue)

    def handle_frame(self, msg_id: int, seq: int, payload: bytes) -> None:
        del seq

        if msg_id != MSG_TELEMETRY_DRIVE:
            return

        if len(payload) != TELEMETRY_LEN:
            self.status_var.set(
                f"텔레메트리 길이 오류: {len(payload)} / {TELEMETRY_LEN}"
            )
            return

        (
            target_speed_mm_s,
            measured_speed_mm_s,
            encoder_count,
            motor_duty_permille,
            steering_cdeg,
            state,
            fault_bits,
            uptime_ms,
        ) = struct.unpack(TELEMETRY_FMT, payload)

        self.last_rx_monotonic = time.monotonic()

        target_speed_m_s = target_speed_mm_s / 1000.0
        measured_speed_m_s = measured_speed_mm_s / 1000.0
        pwm_percent = motor_duty_permille / 10.0

        self.live_target_var.set(f"{target_speed_m_s:.3f} m/s")
        self.live_speed_var.set(f"{measured_speed_m_s:.3f} m/s")
        self.live_pwm_var.set(f"{pwm_percent:.1f} %")
        self.live_encoder_var.set(str(encoder_count))
        self.live_state_var.set(STATE_NAMES.get(state, str(state)))
        self.live_fault_var.set(f"0x{fault_bits:08X}")

        if not self.recording:
            return

        if self.record_start_uptime is None:
            self.record_start_uptime = uptime_ms

        elapsed_ms = (uptime_ms - self.record_start_uptime) & 0xFFFFFFFF
        elapsed_s = elapsed_ms / 1000.0

        self.samples.append(
            Sample(
                time_s=elapsed_s,
                target_speed_mm_s=target_speed_mm_s,
                measured_speed_mm_s=measured_speed_mm_s,
                measured_speed_m_s=measured_speed_m_s,
                pwm_percent=pwm_percent,
                encoder_count=encoder_count,
                steering_cdeg=steering_cdeg,
                state=state,
                fault_bits=fault_bits,
                uptime_ms=uptime_ms,
            )
        )

        self.sample_count_var.set(f"{len(self.samples)} samples")

    # --------------------------------------------------------------- plotting
    def update_plot(self) -> None:
        self.speed_axis.clear()
        self.pwm_axis.clear()

        if self.samples:
            times = [sample.time_s for sample in self.samples]
            target_speeds = [
                sample.target_speed_mm_s / 1000.0 for sample in self.samples
            ]
            measured_speeds = [sample.measured_speed_m_s for sample in self.samples]
            pwm_values = [sample.pwm_percent for sample in self.samples]

            self.speed_axis.plot(times, target_speeds, label="Target speed")
            self.speed_axis.plot(times, measured_speeds, label="Measured speed")
            self.pwm_axis.plot(times, pwm_values, label="PID output PWM")

            self.speed_axis.legend(loc="upper right")
            self.pwm_axis.legend(loc="upper right")

        self.speed_axis.axhline(0.0, linewidth=0.8)
        self.speed_axis.set_ylabel("Speed (m/s)")
        self.speed_axis.grid(True)

        self.pwm_axis.axhline(0.0, linewidth=0.8)
        self.pwm_axis.set_xlabel("Time (s)")
        self.pwm_axis.set_ylabel("PWM (%)")
        self.pwm_axis.grid(True)

        self.figure.tight_layout()
        self.canvas.draw_idle()

        self.root.after(GUI_PERIOD_MS, self.update_plot)

    # --------------------------------------------------------------- records
    def clear_recording(self) -> None:
        if self.test_running:
            messagebox.showwarning("기록 초기화", "시험을 먼저 정지하세요.")
            return

        self.recording = False
        self.samples.clear()
        self.record_start_uptime = None
        self.recording_var.set("기록 정지")
        self.sample_count_var.set("0 samples")
        self.test_status_var.set("기록 초기화 완료")

    def save_csv(self) -> None:
        if not self.samples:
            messagebox.showwarning("CSV", "저장할 기록이 없습니다.")
            return

        default_name = time.strftime("pid_20s_%Y%m%d_%H%M%S.csv")

        path = filedialog.asksaveasfilename(
            defaultextension=".csv",
            initialfile=default_name,
            filetypes=[("CSV 파일", "*.csv")],
        )

        if not path:
            return

        try:
            with open(path, "w", newline="", encoding="utf-8-sig") as csv_file:
                writer = csv.writer(csv_file)

                writer.writerow(
                    [
                        "time_s",
                        "target_speed_mm_s",
                        "target_speed_m_s",
                        "measured_speed_mm_s",
                        "measured_speed_m_s",
                        "pwm_percent",
                        "encoder_count",
                        "steering_cdeg",
                        "state",
                        "fault_bits",
                        "uptime_ms",
                    ]
                )

                for sample in self.samples:
                    writer.writerow(
                        [
                            f"{sample.time_s:.3f}",
                            sample.target_speed_mm_s,
                            f"{sample.target_speed_mm_s / 1000.0:.4f}",
                            sample.measured_speed_mm_s,
                            f"{sample.measured_speed_m_s:.4f}",
                            f"{sample.pwm_percent:.2f}",
                            sample.encoder_count,
                            sample.steering_cdeg,
                            sample.state,
                            f"0x{sample.fault_bits:08X}",
                            sample.uptime_ms,
                        ]
                    )

        except OSError as exc:
            messagebox.showerror("CSV 저장 실패", str(exc))
            return

        messagebox.showinfo("CSV 저장", f"저장 완료:\n{path}")

    def close(self) -> None:
        self.disconnect()
        self.root.destroy()


def main() -> None:
    root = tk.Tk()
    PidTestApp(root)
    root.mainloop()


if __name__ == "__main__":
    main()
