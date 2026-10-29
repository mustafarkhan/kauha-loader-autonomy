"""Learned bucket-filling policy: NumPy-only inference for the vehicle.

The policy is a productivity function, not a safety function. It only proposes
speed/boom/bucket requests during the DIG phase; every request still passes
through the drive-by-wire arbiter and the independent safety channel.
"""
from __future__ import annotations

from dataclasses import dataclass
from pathlib import Path

import numpy as np


@dataclass
class BucketPolicy:
    weights: list[tuple[np.ndarray, np.ndarray]]
    obs_mean: np.ndarray
    obs_var: np.ndarray
    clip_obs: float
    eps: float
    residual_scale: float = 0.0
    base_params: tuple | None = None

    @classmethod
    def load(cls, path: str | Path) -> "BucketPolicy":
        z = np.load(path)
        n = int(z["n_layers"])
        weights = [(z[f"w{i}"].astype(np.float64), z[f"b{i}"].astype(np.float64)) for i in range(n)]
        res = float(z["residual_scale"]) if "residual_scale" in z else 0.0
        base = tuple(float(v) for v in z["base_params"]) if "base_params" in z else None
        return cls(weights, z["obs_mean"].astype(np.float64), z["obs_var"].astype(np.float64),
                   float(z["clip_obs"]), float(z["eps"]), res, base)

    def act(self, obs: np.ndarray) -> np.ndarray:
        x = np.clip((np.asarray(obs, dtype=np.float64) - self.obs_mean) / np.sqrt(self.obs_var + self.eps),
                    -self.clip_obs, self.clip_obs)
        for w, b in self.weights[:-1]:
            x = np.tanh(w @ x + b)
        w, b = self.weights[-1]
        a = np.clip(w @ x + b, -1.0, 1.0)
        if self.residual_scale > 0:
            # deployed controller = rule-based operator + learned correction
            from kauha.train.bucket_env import ScriptedOperator
            base = ScriptedOperator(self.base_params or ScriptedOperator.DEFAULT)(np.asarray(obs))
            a = np.clip(base + self.residual_scale * a, -1.0, 1.0)
        return a.astype(np.float32)
