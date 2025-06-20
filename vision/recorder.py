import cv2
import time
from pathlib import Path
from typing import Optional

from vision.detector import ObjectDetector, CLASSES


class VideoRecorder:
    """Captures camera feed, runs object detection, saves annotated video."""

    def __init__(self, output_path: str = "recordings", fps: int = 20, display: bool = False):
        self.output_path = Path(output_path)
        self.output_path.mkdir(exist_ok=True)
        ts = time.strftime("%Y%m%d-%H%M%S")
        self.video_file = self.output_path / f"flight_{ts}.avi"
        self._fps = fps
        self._display = display

        # camera init
        self._cap = cv2.VideoCapture(0)
        if not self._cap.isOpened():
            raise RuntimeError("Cannot open camera device 0.")
        width = int(self._cap.get(cv2.CAP_PROP_FRAME_WIDTH))
        height = int(self._cap.get(cv2.CAP_PROP_FRAME_HEIGHT))

        fourcc = cv2.VideoWriter_fourcc(*"XVID")
        self._writer = cv2.VideoWriter(str(self.video_file), fourcc, fps, (width, height))

        self._detector = ObjectDetector()

    def run(self):
        print(f"[VideoRecorder] Recording to {self.video_file} (Ctrl-C to stop)…")
        try:
            while True:
                ret, frame = self._cap.read()
                if not ret:
                    print("Failed to grab frame")
                    break

                detections = self._detector.detect(frame)
                for label, conf, (x1, y1, x2, y2) in detections:
                    color = (0, 255, 0)
                    cv2.rectangle(frame, (x1, y1), (x2, y2), color, 2)
                    text = f"{label}: {conf*100:.1f}%"
                    cv2.putText(frame, text, (x1, y1 - 10), cv2.FONT_HERSHEY_SIMPLEX, 0.5, color, 1)

                self._writer.write(frame)
                if self._display:
                    cv2.imshow("FlightCam", frame)
                    if cv2.waitKey(1) & 0xFF == ord('q'):
                        break
        except KeyboardInterrupt:
            print("Stopping recording…")
        finally:
            self._cap.release()
            self._writer.release()
            if self._display:
                cv2.destroyAllWindows() 