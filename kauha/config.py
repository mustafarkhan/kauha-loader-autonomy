"""Machine and site configuration loading."""
from __future__ import annotations

from dataclasses import dataclass, field
from pathlib import Path

import yaml

REPO_ROOT = Path(__file__).resolve().parents[1]
DEFAULT_MACHINE_FILE = REPO_ROOT / "hardware" / "config" / "loader_medium.yaml"


@dataclass(frozen=True)
class Material:
    name: str
    density_t_m3: float
    k_resist: float
    friction_deg: float


@dataclass(frozen=True)
class MachineConfig:
    operating_mass_kg: float
    lf: float
    lr: float
    track_width: float
    body_length: float
    body_width: float
    max_articulation: float
    articulation_rate: float
    max_speed: float
    max_accel: float
    brake_decel: float
    actuator_latency: float
    traction_coefficient: float
    bucket_volume: float
    bucket_width: float
    boom_pivot_forward: float
    boom_pivot_height: float
    boom_length: float
    bucket_length: float
    boom_min: float
    boom_max: float
    boom_rate: float
    bucket_min: float
    bucket_max: float
    bucket_rate: float
    breakout_force: float
    auto_max_speed: float
    warning_speed: float
    teleop_max_speed: float
    materials: dict = field(default_factory=dict)


def load_machine(path: str | Path = DEFAULT_MACHINE_FILE) -> MachineConfig:
    data = yaml.safe_load(Path(path).read_text())
    m, b, s = data["machine"], data["bucket"], data["site_limits"]
    materials = {
        name: Material(name, v["density_t_m3"], v["k_resist"], v["friction_deg"])
        for name, v in data["materials"].items()
    }
    return MachineConfig(
        operating_mass_kg=m["operating_mass_kg"],
        lf=m["front_axle_to_joint_m"],
        lr=m["rear_axle_to_joint_m"],
        track_width=m["track_width_m"],
        body_length=m["body_length_m"],
        body_width=m["body_width_m"],
        max_articulation=m["max_articulation_rad"],
        articulation_rate=m["articulation_rate_rad_s"],
        max_speed=m["max_speed_m_s"],
        max_accel=m["max_accel_m_s2"],
        brake_decel=m["service_brake_decel_m_s2"],
        actuator_latency=m["actuator_latency_s"],
        traction_coefficient=m["traction_coefficient"],
        bucket_volume=b["rated_volume_m3"],
        bucket_width=b["width_m"],
        boom_pivot_forward=b["boom_pivot_forward_m"],
        boom_pivot_height=b["boom_pivot_height_m"],
        boom_length=b["boom_length_m"],
        bucket_length=b["bucket_length_m"],
        boom_min=b["boom_min_rad"],
        boom_max=b["boom_max_rad"],
        boom_rate=b["boom_rate_rad_s"],
        bucket_min=b["bucket_min_rad"],
        bucket_max=b["bucket_max_rad"],
        bucket_rate=b["bucket_rate_rad_s"],
        breakout_force=b["breakout_force_n"],
        auto_max_speed=s["auto_max_speed_m_s"],
        warning_speed=s["warning_field_speed_m_s"],
        teleop_max_speed=s["teleop_max_speed_m_s"],
        materials=materials,
    )
