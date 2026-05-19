from __future__ import annotations

from dataclasses import dataclass
from typing import Literal, Optional


@dataclass(frozen=True)
class KnobSkirt:
    diameter: float
    height: float
    flare: float = 0.0
    chamfer: float = 0.0


@dataclass(frozen=True)
class KnobGrip:
    style: Literal[
        "none",
        "fluted",
        "scalloped",
        "knurled",
        "ribbed",
        "diamond_knurl",
    ] = "none"
    count: Optional[int] = None
    depth: float = 0.0
    width: Optional[float] = None
    helix_angle_deg: float = 0.0


@dataclass(frozen=True)
class KnobIndicator:
    style: Literal["none", "line", "notch", "wedge", "dot"] = "none"
    length: Optional[float] = None
    width: Optional[float] = None
    depth: float = 0.0
    angle_deg: float = 0.0
    mode: Literal["engraved", "raised"] = "engraved"


@dataclass(frozen=True)
class KnobTopFeature:
    style: Literal["none", "flush_disk", "recessed_disk", "top_insert"] = "none"
    diameter: Optional[float] = None
    depth: float = 0.0
    height: float = 0.0


@dataclass(frozen=True)
class KnobBore:
    style: Literal["none", "round", "d_shaft", "double_d", "splined", "hex"] = "round"
    diameter: Optional[float] = None
    flat_depth: Optional[float] = None
    spline_count: Optional[int] = None
    spline_depth: float = 0.0
    through: bool = True


@dataclass(frozen=True)
class KnobRelief:
    style: Literal["side_window", "top_recess", "coin_slot"]
    angle_deg: float = 0.0
    width: Optional[float] = None
    height: Optional[float] = None
    depth: float = 0.0


@dataclass(frozen=True)
class BezelFace:
    style: Literal["flat", "rounded", "chamfered", "radiused_step"] = "flat"
    front_lip: float = 0.0
    chamfer: float = 0.0
    fillet: float = 0.0


@dataclass(frozen=True)
class BezelRecess:
    depth: float
    inset: float
    floor_radius: float = 0.0
    wall_draft_deg: float = 0.0


@dataclass(frozen=True)
class BezelVisor:
    top_extension: float = 0.0
    side_extension: float = 0.0
    thickness: float = 0.0
    draft_deg: float = 0.0


@dataclass(frozen=True)
class BezelFlange:
    width: float = 0.0
    thickness: float = 0.0
    offset: float = 0.0


@dataclass(frozen=True)
class BezelMounts:
    style: Literal["none", "bosses", "tabs", "rear_flange"] = "none"
    hole_count: int = 0
    hole_diameter: Optional[float] = None
    boss_diameter: Optional[float] = None
    setback: float = 0.0


@dataclass(frozen=True)
class BezelCutout:
    edge: Literal["top", "bottom", "left", "right"]
    width: float
    depth: float
    offset: float = 0.0


@dataclass(frozen=True)
class BezelEdgeFeature:
    style: Literal["bead", "step", "notch"] = "bead"
    edge: Literal["top", "bottom", "left", "right"] = "top"
    size: float = 0.0
    offset: float = 0.0
    extent: float = 0.0


@dataclass(frozen=True)
class BoltPattern:
    count: int
    circle_diameter: float
    hole_diameter: float
    countersink: float = 0.0


@dataclass(frozen=True)
class WheelRim:
    outer_radius: Optional[float] = None
    inner_radius: Optional[float] = None
    flange_height: float = 0.0
    flange_thickness: float = 0.0
    bead_seat_depth: float = 0.0


@dataclass(frozen=True)
class WheelHub:
    radius: float
    width: float
    cap_style: Literal["flat", "domed", "protruding", "recessed"] = "flat"
    bolt_pattern: Optional[BoltPattern] = None


@dataclass(frozen=True)
class WheelFace:
    dish_depth: float = 0.0
    front_inset: float = 0.0
    rear_inset: float = 0.0
    window_depth: float = 0.0


@dataclass(frozen=True)
class WheelSpokes:
    style: Literal["none", "disc", "solid_slots", "straight", "split_y", "mesh"] = "none"
    count: Optional[int] = None
    thickness: float = 0.0
    window_radius: float = 0.0
    twist_deg: float = 0.0


@dataclass(frozen=True)
class WheelBore:
    style: Literal["round", "keyed", "hex"] = "round"
    diameter: float = 0.0
    key_width: Optional[float] = None


@dataclass(frozen=True)
class WheelFlange:
    radius: float
    thickness: float
    offset: float = 0.0
    side: Literal["front", "rear", "both"] = "both"
    section: Literal["flat", "round"] = "flat"


@dataclass(frozen=True)
class TireCarcass:
    crown_radius: Optional[float] = None
    belt_width_ratio: float = 0.34
    sidewall_bulge: float = 0.08


@dataclass(frozen=True)
class TireTread:
    style: Literal["none", "circumferential", "block", "rib", "chevron", "lug"] = "none"
    depth: float = 0.0
    pitch: Optional[float] = None
    count: Optional[int] = None
    angle_deg: float = 0.0
    land_ratio: float = 0.5


@dataclass(frozen=True)
class TireGroove:
    center_offset: float = 0.0
    width: float = 0.0
    depth: float = 0.0


@dataclass(frozen=True)
class TireSidewall:
    style: Literal["flat", "bulged", "square", "rounded"] = "rounded"
    bulge: float = 0.08
    inset_ratio: float = 0.18


@dataclass(frozen=True)
class TireShoulder:
    style: Literal["soft", "square"] = "soft"
    radius: float = 0.0
    width: float = 0.0


@dataclass(frozen=True)
class HingeHolePattern:
    style: Literal["none", "round", "countersunk", "slotted"] = "none"
    count: int = 0
    diameter: Optional[float] = None
    slot_size: Optional[tuple[float, float]] = None
    edge_margin: float = 0.0
    pitch: Optional[float] = None


@dataclass(frozen=True)
class HingePinStyle:
    head_style: Literal["plain", "button", "flat", "peened"] = "plain"
    head_height: float = 0.0
    head_diameter: Optional[float] = None
    exposed_end: float = 0.0


@dataclass(frozen=True)
class VentGrilleSlats:
    profile: Literal["flat", "airfoil", "boxed"] = "flat"
    direction: Literal["down", "up"] = "down"
    inset: float = 0.0
    divider_count: int = 0
    divider_width: float = 0.004


@dataclass(frozen=True)
class VentGrilleFrame:
    style: Literal["flush", "beveled", "radiused"] = "flush"
    depth: float = 0.0


@dataclass(frozen=True)
class VentGrilleMounts:
    style: Literal["none", "holes"] = "none"
    inset: float = 0.008
    hole_diameter: Optional[float] = None


@dataclass(frozen=True)
class VentGrilleSleeve:
    style: Literal["none", "short", "full"] = "full"
    depth: Optional[float] = None
    wall: Optional[float] = None


@dataclass(frozen=True)
class FanRotorBlade:
    shape: Literal["straight", "scimitar", "broad", "narrow"] = "straight"
    tip_pitch_deg: Optional[float] = None
    camber: float = 0.0
    tip_clearance: float = 0.0


@dataclass(frozen=True)
class FanRotorHub:
    style: Literal["flat", "domed", "capped", "spinner"] = "domed"
    rear_collar_height: Optional[float] = None
    rear_collar_radius: Optional[float] = None
    bore_diameter: Optional[float] = None


@dataclass(frozen=True)
class FanRotorShroud:
    thickness: float
    depth: Optional[float] = None
    clearance: float = 0.0
    lip_depth: float = 0.0
