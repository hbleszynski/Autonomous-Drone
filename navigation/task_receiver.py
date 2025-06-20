import json
import socket
import threading
from queue import Queue
from typing import Dict, Any


class TaskReceiver(threading.Thread):
    """Background thread that listens for JSON task commands over UDP."""

    def __init__(self, queue: Queue, host: str = "0.0.0.0", port: int = 5005):
        super().__init__(daemon=True)
        self.queue = queue
        self.host = host
        self.port = port
        self._sock = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
        self._sock.bind((self.host, self.port))

    def run(self):
        print(f"TaskReceiver listening on UDP {self.host}:{self.port}")
        while True:
            try:
                data, addr = self._sock.recvfrom(4096)
                msg = data.decode("utf-8").strip()
                task: Dict[str, Any] = json.loads(msg)
                self.queue.put(task)
                print(f"Received task from {addr}: {task}")
            except (json.JSONDecodeError, UnicodeDecodeError) as e:
                print(f"Invalid task message: {e}")
            except Exception as exc:
                print(f"TaskReceiver error: {exc}") 