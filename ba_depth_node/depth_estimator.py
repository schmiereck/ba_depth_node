"""Depth Anything V2 inference wrapper (no ROS2 dependency).

This module provides a pure-Python class that loads a Depth Anything V2
model via the HuggingFace ``transformers`` pipeline and exposes a single
``estimate()`` method that converts a BGR image to a float32 depth map.

The class is designed to be importable by other packages (e.g.
``ba_perception_pipeline``) without pulling in ``rclpy``.
"""

import time
from typing import Callable

import cv2
import numpy as np
from PIL import Image as PILImage
from transformers import pipeline


class DepthEstimator:
    """Depth Anything V2 inference engine.

    Parameters
    ----------
    model_id : str
        HuggingFace model identifier, e.g.
        ``"depth-anything/Depth-Anything-V2-Small-hf"``.
    device : str
        PyTorch device string (``"cpu"``, ``"cuda"``, …).
    log_fn : callable, optional
        Logging function (e.g. ``node.get_logger().info``).  Falls back
        to :func:`print` when *None*.
    """

    def __init__(
        self,
        model_id: str = 'depth-anything/Depth-Anything-V2-Small-hf',
        device: str = 'cpu',
        log_fn: Callable[[str], None] | None = None,
    ) -> None:
        self._log = log_fn or print

        self._log(f'Loading model {model_id} on {device} ...')
        t0 = time.perf_counter()
        self._pipe = pipeline(
            task='depth-estimation', model=model_id, device=device)
        self._log(f'Model loaded in {time.perf_counter() - t0:.2f} s')

    def estimate(self, bgr: np.ndarray) -> np.ndarray:
        """Run depth estimation on a BGR image.

        Parameters
        ----------
        bgr : np.ndarray
            Input image in OpenCV BGR format (H×W×3, uint8).

        Returns
        -------
        np.ndarray
            Float32 depth map normalised to [0.0, 1.0], shape (H, W).
        """
        rgb = cv2.cvtColor(bgr, cv2.COLOR_BGR2RGB)
        pil_img = PILImage.fromarray(rgb)
        result = self._pipe(pil_img)
        depth_pil = result['depth']
        return np.ascontiguousarray(
            np.array(depth_pil, dtype=np.float32) / 255.0)
