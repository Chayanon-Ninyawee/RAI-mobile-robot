#!/usr/bin/env python3

import cv2


class Camera:
    def __init__(self, index=0, width=640, height=480, warmup_frames=3):
        self.index = index
        self.width = width
        self.height = height
        self.warmup_frames = warmup_frames

    def capture(self):
        cap = cv2.VideoCapture(self.index, cv2.CAP_V4L2)

        if not cap.isOpened():
            raise RuntimeError(f"Could not open camera index {self.index}")

        cap.set(cv2.CAP_PROP_FOURCC, cv2.VideoWriter_fourcc(*"MJPG"))
        cap.set(cv2.CAP_PROP_FRAME_WIDTH, self.width)
        cap.set(cv2.CAP_PROP_FRAME_HEIGHT, self.height)

        frame = None

        # Let auto exposure / white balance settle.
        for _ in range(self.warmup_frames):
            ret, frame = cap.read()

        cap.release()

        if not ret or frame is None or frame.size == 0:
            raise RuntimeError("Failed to capture image")

        return frame
