import json
import threading
from queue import Queue
from typing import Dict, Any

import serial


class RadioTaskReceiver(threading.Thread):
    """Background thread listening to a radio module via serial port.

    The radio should send newline-terminated JSON strings.
    """

    def __init__(self, queue: Queue, device: str = "/dev/ttyAMA0", baud: int = 57600):
        super().__init__(daemon=True)
        self.queue = queue
        self.device = device
        self.baud = baud
        try:
            self._ser = serial.Serial(self.device, self.baud, timeout=1)
        except serial.SerialException as e:
            raise RuntimeError(f"Unable to open serial port {self.device}: {e}")

    def run(self):
        print(f"RadioTaskReceiver listening on {self.device} @ {self.baud} baud")
        buffer = ""
        while True:
            try:
                data = self._ser.read(self._ser.in_waiting or 1).decode("utf-8", errors="ignore")
                if not data:
                    continue
                buffer += data
                while "\n" in buffer:
                    line, buffer = buffer.split("\n", 1)
                    line = line.strip()
                    if not line:
                        continue
                    try:
                        task: Dict[str, Any] = json.loads(line)
                        self.queue.put(task)
                        print(f"Radio task: {task}")
                    except json.JSONDecodeError as e:
                        print(f"Malformed JSON over radio: {e} — line: {line}")
            except Exception as exc:
                print(f"RadioTaskReceiver error: {exc}") 