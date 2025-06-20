"""Lightweight object detector using MobileNet SSD (Caffe) via OpenCV DNN.

Automatically downloads the model the first time you run on the Pi.
"""
from __future__ import annotations

import os
import urllib.request
from pathlib import Path
from typing import List, Tuple

import cv2
import numpy as np


MODEL_URL = "https://github.com/opencv/opencv_extra/raw/master/testdata/dnn/"
PROTO_FILE = "MobileNetSSD_deploy.prototxt"
WEIGHTS_FILE = "MobileNetSSD_deploy.caffemodel"

CLASSES = [
    "background",
    "aeroplane",
    "bicycle",
    "bird",
    "boat",
    "bottle",
    "bus",
    "car",
    "cat",
    "chair",
    "cow",
    "diningtable",
    "dog",
    "horse",
    "motorbike",
    "person",
    "pottedplant",
    "sheep",
    "sofa",
    "train",
    "tvmonitor",
]


class ObjectDetector:
    def __init__(self, conf_threshold: float = 0.4):
        self.conf_threshold = conf_threshold
        self._net = self._load_network()

    # ------------------------------------------------------------------
    # Public API
    # ------------------------------------------------------------------
    def detect(self, frame: np.ndarray) -> List[Tuple[str, float, Tuple[int, int, int, int]]]:
        """Return list of detections: (label, confidence, bbox)."""
        (h, w) = frame.shape[:2]
        blob = cv2.dnn.blobFromImage(frame, 0.007843, (300, 300), 127.5)
        self._net.setInput(blob)
        detections = self._net.forward()

        results = []
        for i in range(detections.shape[2]):
            confidence = float(detections[0, 0, i, 2])
            if confidence < self.conf_threshold:
                continue
            idx = int(detections[0, 0, i, 1])
            if idx >= len(CLASSES):
                continue
            label = CLASSES[idx]
            box = detections[0, 0, i, 3:7] * np.array([w, h, w, h])
            (startX, startY, endX, endY) = box.astype("int")
            results.append((label, confidence, (startX, startY, endX, endY)))
        return results

    # ------------------------------------------------------------------
    # Internal helpers
    # ------------------------------------------------------------------
    def _load_network(self):
        model_dir = Path(__file__).with_suffix("").parent / "models"
        model_dir.mkdir(exist_ok=True)
        proto_path = model_dir / PROTO_FILE
        weights_path = model_dir / WEIGHTS_FILE

        if not proto_path.exists():
            self._download_file(MODEL_URL + PROTO_FILE, proto_path)
        if not weights_path.exists():
            self._download_file(MODEL_URL + WEIGHTS_FILE, weights_path)

        net = cv2.dnn.readNetFromCaffe(str(proto_path), str(weights_path))
        return net

    @staticmethod
    def _download_file(url: str, dest: Path):
        print(f"Downloading {url} → {dest} …")
        urllib.request.urlretrieve(url, dest) 