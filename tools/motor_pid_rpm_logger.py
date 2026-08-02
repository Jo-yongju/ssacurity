#!/usr/bin/env python3
"""
STM32 PID RPM 곡선 기록 프로그램

역할
- STM32가 자체적으로 수행하는 PID 스텝 테스트를 수신만 함
- PC에서 CMD_DRIVE를 전송하지 않음
- 목표 RPM, 측정 RPM, PWM, 엔코더를 실시간 그래프로 표시
- 측정 결과를 CSV로 저장

설치
    py -m pip install pyserial matplotlib

실행
    py motor_pid_rpm_logger.py
"""

from __future__ import annotations

import csv
import math
import queue
import struct
import threading
import time
import tkinter as tk
from dataclasses import dataclass
from pathlib import Path
from tkinter import filedialog, messagebox, ttk
from typing import Optional

import serial
from serial.tools import list_ports

import matplotlib
matplotlib.use("TkAgg")
from matplotlib.backends.backend_tkagg import FigureCanvasTkAgg
from matplotlib.figure import Figure


SOF1 = 0xAA
SOF2 = 0x55
VERSION = 0x01
MAX_PAYLOAD = 64

MSG_TELEMETRY_DRIVE = 0x80

BAUD = 115200
GUI_PERIOD_MS = 100
WHEEL_DIAMETER_M = 0.064


def speed_mm_s_to_wheel_rpm(speed_mm_s: int) -> float:
    """차량 선속도(mm/s)를 지름 64 mm 바퀴의 RPM으로 변환."""
    speed_m_s = speed_mm_s / 1000.0
    circumference_m = math.pi * WHEEL_DIAMETER_M
    return (speed_m_s / circumference_m) * 60.0

# CommDriveTelemetry:
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
    crc = 0xFFFF
    for byte in data:
        crc ^= byte << 8
        for _ in range(8):
            if crc & 0x8000:
                crc = ((crc << 1) ^ 0x1021) & 0xFFFF
            else:
                crc = (crc << 1) & 0xFFFF
    return crc


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

            # SOF2 + version + msg_id + seq + len + CRC2
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
    target_rpm: float
    measured_rpm: float
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
        except Exception as exc:
            self.output_queue.put(("error", str(exc)))


class PidRpmLoggerApp:
    def __init__(self, root: tk.Tk) -> None:
        self.root = root
        self.root.title("STM32 PID RPM 곡선 기록기")
        self.root.geometry("1180x760")

        self.serial_port: Optional[serial.Serial] = None
        self.reader: Optional[SerialReader] = None
        self.stop_event = threading.Event()
        self.rx_queue: queue.Queue = queue.Queue()

        self.recording = False
        self.samples: list[Sample] = []
        self.record_start_uptime: Optional[int] = None
        self.last_rx_monotonic = 0.0

        self.port_var = tk.StringVar()
        self.connection_var = tk.StringVar(value="연결 안 됨")
        self.recording_var = tk.StringVar(value="기록 정지")
        self.status_var = tk.StringVar(value="텔레메트리 대기")
        self.sample_count_var = tk.StringVar(value="0 samples")

        self.live_target_rpm_var = tk.StringVar(value="0.0 RPM")
        self.live_measured_rpm_var = tk.StringVar(value="0.0 RPM")
        self.live_pwm_var = tk.StringVar(value="0.0 %")
        self.live_encoder_var = tk.StringVar(value="0")
        self.live_state_var = tk.StringVar(value="-")
        self.live_fault_var = tk.StringVar(value="0x00000000")

        self._build_ui()
        self.refresh_ports()

        self.root.after(GUI_PERIOD_MS, self.process_rx_queue)
        self.root.after(GUI_PERIOD_MS, self.update_plot)
        self.root.protocol("WM_DELETE_WINDOW", self.close)

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

        control_frame = ttk.LabelFrame(main, text="PID RPM 측정", padding=8)
        control_frame.pack(fill=tk.X, pady=(8, 0))

        ttk.Button(
            control_frame,
            text="기록 시작",
            command=self.start_recording,
        ).pack(side=tk.LEFT, padx=3)

        ttk.Button(
            control_frame,
            text="기록 정지",
            command=self.stop_recording,
        ).pack(side=tk.LEFT, padx=3)

        ttk.Button(
            control_frame,
            text="기록 초기화",
            command=self.clear_recording,
        ).pack(side=tk.LEFT, padx=3)

        ttk.Button(
            control_frame,
            text="CSV 저장",
            command=self.save_csv,
        ).pack(side=tk.LEFT, padx=(15, 3))

        ttk.Label(
            control_frame,
            text="STM32 명령 전송 없음 · 수신 전용",
        ).pack(side=tk.LEFT, padx=18)

        ttk.Label(
            control_frame,
            textvariable=self.recording_var,
        ).pack(side=tk.RIGHT, padx=10)

        ttk.Label(
            control_frame,
            textvariable=self.sample_count_var,
        ).pack(side=tk.RIGHT)

        live_frame = ttk.LabelFrame(main, text="실시간 값", padding=8)
        live_frame.pack(fill=tk.X, pady=(8, 0))

        live_items = [
            ("목표 RPM", self.live_target_rpm_var),
            ("측정 RPM", self.live_measured_rpm_var),
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
            text="PID 응답: 목표 RPM / 측정 RPM / PWM",
            padding=4,
        )
        plot_frame.pack(fill=tk.BOTH, expand=True, pady=(8, 0))

        self.figure = Figure(figsize=(10, 5.5), dpi=100)
        self.speed_axis = self.figure.add_subplot(211)
        self.pwm_axis = self.figure.add_subplot(212)

        self.canvas = FigureCanvasTkAgg(self.figure, master=plot_frame)
        self.canvas.get_tk_widget().pack(fill=tk.BOTH, expand=True)

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
            )
            self.serial_port.reset_input_buffer()
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

        self.connection_var.set(f"{port_name} 연결됨 · 115200 8N1")
        self.connect_button.configure(text="연결 해제")

    def disconnect(self) -> None:
        self.recording = False
        self.recording_var.set("기록 정지")

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

    def start_recording(self) -> None:
        if not self.serial_port or not self.serial_port.is_open:
            messagebox.showwarning("기록", "STM32에 먼저 연결하세요.")
            return

        self.samples.clear()
        self.record_start_uptime = None
        self.recording = True
        self.recording_var.set("● 기록 중")
        self.sample_count_var.set("0 samples")

    def stop_recording(self) -> None:
        self.recording = False
        self.recording_var.set("기록 정지")

    def clear_recording(self) -> None:
        self.recording = False
        self.samples.clear()
        self.record_start_uptime = None
        self.recording_var.set("기록 정지")
        self.sample_count_var.set("0 samples")

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

        measured_speed_m_s = measured_speed_mm_s / 1000.0
        target_rpm = speed_mm_s_to_wheel_rpm(target_speed_mm_s)
        measured_rpm = speed_mm_s_to_wheel_rpm(measured_speed_mm_s)
        pwm_percent = motor_duty_permille / 10.0

        self.live_target_rpm_var.set(f"{target_rpm:.1f} RPM")
        self.live_measured_rpm_var.set(f"{measured_rpm:.1f} RPM")
        self.live_pwm_var.set(f"{pwm_percent:.1f} %")
        self.live_encoder_var.set(str(encoder_count))
        self.live_state_var.set(STATE_NAMES.get(state, str(state)))
        self.live_fault_var.set(f"0x{fault_bits:08X}")

        if not self.recording:
            return

        if self.record_start_uptime is None:
            self.record_start_uptime = uptime_ms
        elif uptime_ms < self.record_start_uptime:
            # 기록 시작 후 STM32를 Reset한 경우 uptime이 0 근처로 돌아간다.
            # 이전 데이터를 버리고 Reset 시점부터 새 곡선을 기록한다.
            self.samples.clear()
            self.record_start_uptime = uptime_ms

        elapsed_ms = uptime_ms - self.record_start_uptime
        elapsed_s = elapsed_ms / 1000.0

        self.samples.append(
            Sample(
                time_s=elapsed_s,
                target_speed_mm_s=target_speed_mm_s,
                measured_speed_mm_s=measured_speed_mm_s,
                measured_speed_m_s=measured_speed_m_s,
                target_rpm=target_rpm,
                measured_rpm=measured_rpm,
                pwm_percent=pwm_percent,
                encoder_count=encoder_count,
                steering_cdeg=steering_cdeg,
                state=state,
                fault_bits=fault_bits,
                uptime_ms=uptime_ms,
            )
        )

        self.sample_count_var.set(f"{len(self.samples)} samples")

    def update_plot(self) -> None:
        self.speed_axis.clear()
        self.pwm_axis.clear()

        if self.samples:
            times = [sample.time_s for sample in self.samples]
            target_rpms = [sample.target_rpm for sample in self.samples]
            measured_rpms = [sample.measured_rpm for sample in self.samples]
            pwm_values = [sample.pwm_percent for sample in self.samples]

            self.speed_axis.plot(times, target_rpms, label="Target RPM")
            self.speed_axis.plot(times, measured_rpms, label="Measured RPM")
            self.pwm_axis.plot(times, pwm_values, label="PWM")

        self.speed_axis.set_ylabel("Wheel RPM")
        self.speed_axis.grid(True)
        self.speed_axis.legend(loc="upper right")

        self.pwm_axis.axhline(0.0, linewidth=0.8)
        self.pwm_axis.set_xlabel("Time (s)")
        self.pwm_axis.set_ylabel("PWM (%)")
        self.pwm_axis.grid(True)
        self.pwm_axis.legend(loc="upper right")

        self.figure.tight_layout()
        self.canvas.draw_idle()

        self.root.after(GUI_PERIOD_MS, self.update_plot)

    def save_csv(self) -> None:
        if not self.samples:
            messagebox.showwarning("CSV", "저장할 기록이 없습니다.")
            return

        default_name = time.strftime("pid_rpm_%Y%m%d_%H%M%S.csv")

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
                        "measured_speed_mm_s",
                        "measured_speed_m_s",
                        "target_wheel_rpm",
                        "measured_wheel_rpm",
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
                            sample.measured_speed_mm_s,
                            f"{sample.measured_speed_m_s:.4f}",
                            f"{sample.target_rpm:.3f}",
                            f"{sample.measured_rpm:.3f}",
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
    PidRpmLoggerApp(root)
    root.mainloop()


if __name__ == "__main__":
    main()