# SPDX-License-Identifier: Apache-2.0
"""Pluggable person detector.

Default implementation is torchvision Keypoint R-CNN (ResNet-50 FPN): it returns
a person box **and** 17 COCO keypoints, which the depth/monocular lifting uses to
pick body points and the foot pixel — richer than a bare box, as step 3 wants.

Licensing (recorded per the brief):
  * torchvision (code + the ``KeypointRCNN_ResNet50_FPN_Weights.COCO_LEGACY``
    pretrained weights): BSD-3-Clause. https://github.com/pytorch/vision
  * torch: BSD-3-Clause.
  No AGPL / ultralytics anywhere.

torch is imported lazily inside the torch implementation so this module (and the
detector base class / registry) imports fine on a machine with no torch and no
GPU — the offline geometry/tracker/safety tests never touch it.
"""
from __future__ import annotations

import abc
from typing import List

import numpy as np

from .messages import Detection2D


class PersonDetector(abc.ABC):
    """Return the people found in one HxWx3 uint8 RGB image."""

    name: str = "base"
    license: str = ""

    @abc.abstractmethod
    def detect(self, rgb: np.ndarray) -> List[Detection2D]:
        ...

    def detect_batch(self, rgbs: List[np.ndarray]) -> List[List[Detection2D]]:
        """Detect on several images at once, returning one result list per image.

        The shared-detector path (``nodes/shared_detector_node.py``) batches the
        frames that need detection this tick into a single call so one model
        instance serves every camera in one forward pass. The default is a plain
        loop (correct for any detector); a GPU detector overrides it with a true
        batched forward pass. The order of the returned lists matches ``rgbs``.
        """
        return [self.detect(rgb) for rgb in rgbs]

    def warmup(self):
        """Optional: run a dummy inference so the first real frame isn't slow."""


class KeypointRCNNDetector(PersonDetector):
    name = "keypoint_rcnn"
    license = ("torchvision BSD-3-Clause (code); weights "
               "KeypointRCNN_ResNet50_FPN COCO, BSD-3-Clause")

    def __init__(self, device: str = "cuda", score_thresh: float = 0.6,
                 kp_conf: float = 0.3, max_size: int = 640, fp16: bool = False):
        import torch
        import torchvision
        from torchvision.models.detection import (
            KeypointRCNN_ResNet50_FPN_Weights, keypointrcnn_resnet50_fpn)

        self._torch = torch
        if device == "cuda" and not torch.cuda.is_available():
            device = "cpu"
        self.device = device
        self.score_thresh = score_thresh
        self.kp_conf = kp_conf
        # FP16 (half-precision) inference via autocast on CUDA only; a no-op on
        # CPU. Opt-in: the default fp32 path is the shipped baseline. Verify it
        # does not change detections materially (tools/bench_frame_processing.py
        # --mode gpu reports the fp32-vs-fp16 box/score delta).
        self.fp16 = bool(fp16) and device == "cuda"
        # min_size/max_size cap the internal resize -> bounds GPU memory + latency.
        weights = KeypointRCNN_ResNet50_FPN_Weights.COCO_LEGACY
        self.model = keypointrcnn_resnet50_fpn(
            weights=weights, min_size=int(max_size * 0.75), max_size=max_size)
        self.model.eval().to(device)
        self._tv = torchvision

    def warmup(self):
        dummy = np.zeros((480, 640, 3), dtype=np.uint8)
        self.detect(dummy)

    def _to_tensor(self, rgb: np.ndarray):
        torch = self._torch
        t = torch.from_numpy(np.ascontiguousarray(rgb)).permute(2, 0, 1)
        return t.float().div_(255.0).to(self.device)

    def _infer(self, tensors):
        """Run the model on a list of CHW float tensors -> list of raw outputs."""
        torch = self._torch
        with torch.no_grad():
            if self.fp16:
                with torch.autocast("cuda", dtype=torch.float16):
                    return self.model(tensors)
            return self.model(tensors)

    def _parse(self, out) -> List[Detection2D]:
        boxes = out["boxes"].detach().cpu().numpy()
        scores = out["scores"].detach().cpu().numpy()
        kpts = out["keypoints"].detach().cpu().numpy()          # (N, 17, 3) x,y,vis
        kpts_scores = out["keypoints_scores"].detach().cpu().numpy()  # (N, 17)
        dets: List[Detection2D] = []
        for i in range(len(boxes)):
            if scores[i] < self.score_thresh:
                continue
            k = kpts[i].copy().astype(np.float64)
            # Replace the model's visibility flag with a normalised confidence
            # from the per-keypoint score (sigmoid), so lifting can gate on it.
            conf = 1.0 / (1.0 + np.exp(-kpts_scores[i].astype(np.float64)))
            k[:, 2] = conf
            dets.append(Detection2D(
                bbox=tuple(float(v) for v in boxes[i]),
                score=float(scores[i]),
                keypoints=k))
        return dets

    def detect(self, rgb: np.ndarray) -> List[Detection2D]:
        out = self._infer([self._to_tensor(rgb)])[0]
        return self._parse(out)

    def detect_batch(self, rgbs: List[np.ndarray]) -> List[List[Detection2D]]:
        """One forward pass over all images. Keypoint R-CNN accepts a list of
        images and returns one output dict per image, so a true batch is a single
        model call — this is what lets one instance serve every camera at once."""
        if not rgbs:
            return []
        outs = self._infer([self._to_tensor(r) for r in rgbs])
        return [self._parse(o) for o in outs]

    def gpu_mem_mb(self) -> float:
        torch = self._torch
        if self.device != "cuda":
            return 0.0
        return torch.cuda.max_memory_allocated() / (1024 ** 2)


_REGISTRY = {"keypoint_rcnn": KeypointRCNNDetector}


def build_detector(name: str, **kw) -> PersonDetector:
    if name not in _REGISTRY:
        raise ValueError(
            f"unknown detector {name!r}; known: {sorted(_REGISTRY)}")
    return _REGISTRY[name](**kw)
