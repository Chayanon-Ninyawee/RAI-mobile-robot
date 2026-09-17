#!/usr/bin/env python3

import time

import cv2


class Camera:
    def __init__(
        self,
        index=0,
        width=640,
        height=480,
        warmup_seconds=1.0,
    ):
        self.index = index
        self.width = width
        self.height = height
        self.warmup_seconds = warmup_seconds

    def capture(self):
        cap = cv2.VideoCapture(self.index, cv2.CAP_V4L2)

        if not cap.isOpened():
            raise RuntimeError(f"Could not open camera index {self.index}")

        cap.set(cv2.CAP_PROP_FOURCC, cv2.VideoWriter_fourcc(*"MJPG"))
        cap.set(cv2.CAP_PROP_FRAME_WIDTH, self.width)
        cap.set(cv2.CAP_PROP_FRAME_HEIGHT, self.height)

        deadline = time.monotonic() + self.warmup_seconds

        while time.monotonic() < deadline:
            ret, frame = cap.read()

            if not ret:
                cap.release()
                raise RuntimeError("Failed to read camera")

        # Capture the actual frame used by the detector.
        ret, frame = cap.read()

        cap.release()

        if not ret or frame is None or frame.size == 0:
            raise RuntimeError("Failed to capture image")

        return frame
