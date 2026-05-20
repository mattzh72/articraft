from __future__ import annotations

import math

import cadquery as cq

from sdk import (
    ArticulatedObject,
    ArticulationType,
    Box,
    Cylinder,
    Material,
    Mimic,
    MotionLimits,
    MotionProperties,
    Origin,
    SpurGear,
    TestContext,
    TestReport,
    TorusGeometry,
    mesh_from_cadquery,
    mesh_from_geometry,
)


MODULE = 0.006
GEAR_Z = 0.140
GEAR_WIDTH = 0.046
CENTER_CLEARANCE = 0.005
CENTER_PHASE = math.pi / 40.0
CENTER_TOOTH_PITCH = 2.0 * math.pi / 40.0
UPPER_MESH_THETA = CENTER_PHASE + 6.0 * CENTER_TOOTH_PITCH
LOWER_MESH_THETA = CENTER_PHASE - 7.0 * CENTER_TOOTH_PITCH
UPPER_IDLER_PHASE = (UPPER_MESH_THETA + math.pi - (2.0 * math.pi / 18.0) / 2.0) % (
    2.0 * math.pi / 18.0
)
LOWER_IDLER_PHASE = (LOWER_MESH_THETA + math.pi - (2.0 * math.pi / 22.0) / 2.0) % (
    2.0 * math.pi / 22.0
)
UPPER_OUTPUT_THETA = UPPER_IDLER_PHASE - (2.0 * math.pi / 18.0) / 2.0
UPPER_OUTPUT_PHASE = (UPPER_OUTPUT_THETA + math.pi) % (2.0 * math.pi / 30.0)
LOWER_OUTPUT_THETA = LOWER_IDLER_PHASE - 2.0 * math.pi / 22.0
LOWER_OUTPUT_PHASE = (
    LOWER_OUTPUT_THETA + math.pi - (2.0 * math.pi / 26.0) / 2.0
) % (2.0 * math.pi / 26.0)


GEAR_SPECS = {
    "drive_pinion": {
        "teeth": 20,
        "center": (-0.185, 0.000),
        "phase": 0.00,
        "material": "case_hardened_steel",
        "hub_d": 0.060,
        "bore_d": 0.026,
        "spokes": 5,
    },
    "center_gear": {
        "teeth": 40,
        "center": (0.000, 0.000),
        "phase": CENTER_PHASE,
        "material": "blued_steel",
        "hub_d": 0.085,
        "bore_d": 0.032,
        "spokes": 8,
    },
    "upper_idler": {
        "teeth": 18,
        "center": (
            math.cos(UPPER_MESH_THETA) * (0.120 + 0.054 + CENTER_CLEARANCE),
            math.sin(UPPER_MESH_THETA) * (0.120 + 0.054 + CENTER_CLEARANCE),
        ),
        "phase": UPPER_IDLER_PHASE,
        "material": "warm_gear_bronze",
        "hub_d": 0.054,
        "bore_d": 0.024,
        "spokes": 4,
    },
    "upper_output": {
        "teeth": 30,
        "center": (
            math.cos(UPPER_MESH_THETA) * (0.120 + 0.054 + CENTER_CLEARANCE)
            + math.cos(UPPER_OUTPUT_THETA) * (0.054 + 0.090 + CENTER_CLEARANCE),
            math.sin(UPPER_MESH_THETA) * (0.120 + 0.054 + CENTER_CLEARANCE)
            + math.sin(UPPER_OUTPUT_THETA) * (0.054 + 0.090 + CENTER_CLEARANCE),
        ),
        "phase": UPPER_OUTPUT_PHASE,
        "material": "case_hardened_steel",
        "hub_d": 0.070,
        "bore_d": 0.030,
        "spokes": 6,
    },
    "lower_idler": {
        "teeth": 22,
        "center": (
            math.cos(LOWER_MESH_THETA) * (0.120 + 0.066 + CENTER_CLEARANCE),
            math.sin(LOWER_MESH_THETA) * (0.120 + 0.066 + CENTER_CLEARANCE),
        ),
        "phase": LOWER_IDLER_PHASE,
        "material": "warm_gear_bronze",
        "hub_d": 0.058,
        "bore_d": 0.024,
        "spokes": 5,
    },
    "lower_output": {
        "teeth": 26,
        "center": (
            math.cos(LOWER_MESH_THETA) * (0.120 + 0.066 + CENTER_CLEARANCE)
            + math.cos(LOWER_OUTPUT_THETA) * (0.066 + 0.078 + CENTER_CLEARANCE),
            math.sin(LOWER_MESH_THETA) * (0.120 + 0.066 + CENTER_CLEARANCE)
            + math.sin(LOWER_OUTPUT_THETA) * (0.066 + 0.078 + CENTER_CLEARANCE),
        ),
        "phase": LOWER_OUTPUT_PHASE,
        "material": "blued_steel",
        "hub_d": 0.064,
        "bore_d": 0.028,
        "spokes": 6,
    },
}


GEAR_MESH_PAIRS = (
    ("drive_pinion", "center_gear"),
    ("center_gear", "upper_idler"),
    ("upper_idler", "upper_output"),
    ("center_gear", "lower_idler"),
    ("lower_idler", "lower_output"),
)


def _pitch_radius(teeth: int) -> float:
    return MODULE * teeth * 0.5


def _make_materials(model: ArticulatedObject) -> dict[str, Material]:
    return {
        "cast_iron": model.material("cast_iron", rgba=(0.08, 0.085, 0.09, 1.0)),
        "dark_machined": model.material("dark_machined", rgba=(0.18, 0.19, 0.19, 1.0)),
        "bearing_black": model.material("bearing_black", rgba=(0.02, 0.022, 0.025, 1.0)),
        "bolt_heads": model.material("bolt_heads", rgba=(0.58, 0.59, 0.56, 1.0)),
        "case_hardened_steel": model.material(
            "case_hardened_steel", rgba=(0.62, 0.64, 0.62, 1.0)
        ),
        "blued_steel": model.material("blued_steel", rgba=(0.12, 0.20, 0.32, 1.0)),
        "warm_gear_bronze": model.material(
            "warm_gear_bronze", rgba=(0.78, 0.51, 0.22, 1.0)
        ),
        "safety_yellow": model.material("safety_yellow", rgba=(0.95, 0.70, 0.08, 1.0)),
    }


def _add_bearing_support(base, name: str, x: float, y: float, materials: dict[str, Material]) -> None:
    # A raised annular bearing lip with four columns.  The center stays open so
    # the rotating shaft has visible clearance instead of passing through a
    # solid proxy.
    base.visual(
        mesh_from_geometry(TorusGeometry(0.032, 0.006, radial_segments=24, tubular_segments=12), f"{name}_bearing_ring"),
        origin=Origin(xyz=(x, y, 0.094)),
        material=materials["bearing_black"],
        name=f"{name}_bearing_ring",
    )
    for idx, (dx, dy) in enumerate(((0.032, 0.0), (-0.032, 0.0), (0.0, 0.032), (0.0, -0.032))):
        base.visual(
            Cylinder(radius=0.0065, length=0.058),
            origin=Origin(xyz=(x + dx, y + dy, 0.067)),
            material=materials["dark_machined"],
            name=f"{name}_bearing_post_{idx}",
        )
    base.visual(
        Cylinder(radius=0.045, length=0.010),
        origin=Origin(xyz=(x, y, 0.045)),
        material=materials["dark_machined"],
        name=f"{name}_pedestal_flange",
    )
    for idx, (dx, dy) in enumerate(((0.030, 0.030), (-0.030, 0.030), (0.030, -0.030), (-0.030, -0.030))):
        base.visual(
            Cylinder(radius=0.0045, length=0.006),
            origin=Origin(xyz=(x + dx, y + dy, 0.052)),
            material=materials["bolt_heads"],
            name=f"{name}_flange_bolt_{idx}",
        )


def _add_guard_frame(base, materials: dict[str, Material]) -> None:
    # Perimeter base, side rails, and high protective crossbars give the open
    # gear train an industrial test-stand character while staying outside the
    # rotating swept envelopes.
    base.visual(
        Box((0.84, 0.66, 0.040)),
        origin=Origin(xyz=(0.02, 0.0, 0.020)),
        material=materials["cast_iron"],
        name="ribbed_base_plate",
    )
    base.visual(
        Box((0.82, 0.035, 0.036)),
        origin=Origin(xyz=(0.02, 0.335, 0.058)),
        material=materials["dark_machined"],
        name="rear_longitudinal_rail",
    )
    base.visual(
        Box((0.82, 0.035, 0.036)),
        origin=Origin(xyz=(0.02, -0.335, 0.058)),
        material=materials["dark_machined"],
        name="front_longitudinal_rail",
    )
    base.visual(
        Box((0.035, 0.62, 0.036)),
        origin=Origin(xyz=(-0.420, 0.0, 0.058)),
        material=materials["dark_machined"],
        name="left_end_rail",
    )
    base.visual(
        Box((0.035, 0.62, 0.036)),
        origin=Origin(xyz=(0.460, 0.0, 0.058)),
        material=materials["dark_machined"],
        name="right_end_rail",
    )
    for idx, x in enumerate((-0.39, 0.43)):
        for jdx, y in enumerate((-0.29, 0.29)):
            post_name = f"guard_post_{idx}_{jdx}"
            base.visual(
                Box((0.028, 0.028, 0.165)),
                origin=Origin(xyz=(x, y, 0.122)),
                material=materials["dark_machined"],
                name=post_name,
            )
    base.visual(
        Box((0.86, 0.022, 0.026)),
        origin=Origin(xyz=(0.02, 0.29, 0.2175)),
        material=materials["safety_yellow"],
        name="rear_guard_bar",
    )
    base.visual(
        Box((0.86, 0.022, 0.026)),
        origin=Origin(xyz=(0.02, -0.29, 0.2175)),
        material=materials["safety_yellow"],
        name="front_guard_bar",
    )
    for idx, x in enumerate((-0.30, -0.15, 0.00, 0.15, 0.30)):
        base.visual(
            Box((0.024, 0.54, 0.010)),
            origin=Origin(xyz=(x, 0.0, 0.045)),
            material=materials["dark_machined"],
            name=f"base_stiffening_rib_{idx}",
        )


def _add_gear_part(
    model: ArticulatedObject,
    base,
    gear_name: str,
    spec: dict[str, object],
    materials: dict[str, Material],
) -> None:
    teeth = int(spec["teeth"])
    gear = SpurGear(
        module=MODULE,
        teeth_number=teeth,
        width=GEAR_WIDTH,
        pressure_angle=20.0,
        clearance=0.0008,
        backlash=0.0009,
    )
    spoke_inner = max(float(spec["bore_d"]) * 0.75, _pitch_radius(teeth) * 0.28)
    spoke_outer = _pitch_radius(teeth) * 0.78
    gear_shape = gear.build(
        bore_d=float(spec["bore_d"]),
        hub_d=float(spec["hub_d"]),
        hub_length=GEAR_WIDTH + 0.020,
        n_spokes=int(spec["spokes"]),
        spoke_width=0.012,
        spoke_fillet=None,
        spokes_id=spoke_inner,
        spokes_od=spoke_outer,
        recess_d=_pitch_radius(teeth) * 1.55,
        recess=0.004,
        bottom_recess=0.004,
        bottom_recess_d=_pitch_radius(teeth) * 1.55,
        chamfer=None,
    )

    part = model.part(gear_name)
    part.visual(
        mesh_from_cadquery(gear_shape, f"{gear_name}_toothed_body", tolerance=0.0008, angular_tolerance=0.12),
        origin=Origin(rpy=(0.0, 0.0, float(spec["phase"]))),
        material=materials[str(spec["material"])],
        name="toothed_body",
    )
    shaft_radius = float(spec["bore_d"]) * 0.39
    part.visual(
        Cylinder(radius=shaft_radius, length=0.158),
        origin=Origin(xyz=(0.0, 0.0, 0.000)),
        material=materials["case_hardened_steel"],
        name="keyed_shaft",
    )
    part.visual(
        Cylinder(radius=0.0262, length=0.020),
        origin=Origin(xyz=(0.0, 0.0, -0.046)),
        material=materials["dark_machined"],
        name="bearing_journal",
    )
    part.visual(
        Box((shaft_radius * 0.55, shaft_radius * 0.18, 0.070)),
        origin=Origin(xyz=(shaft_radius * 0.78, 0.0, 0.000)),
        material=materials["bolt_heads"],
        name="shaft_key",
    )
    part.visual(
        Cylinder(radius=float(spec["hub_d"]) * 0.53, length=0.010),
        origin=Origin(xyz=(0.0, 0.0, GEAR_WIDTH * 0.5 + 0.008)),
        material=materials["dark_machined"],
        name="front_lock_collar",
    )

    x, y = spec["center"]
    mimic = None
    if gear_name != "drive_pinion":
        driver_teeth = int(GEAR_SPECS["drive_pinion"]["teeth"])
        if gear_name == "center_gear":
            multiplier = -driver_teeth / teeth
        elif gear_name == "upper_idler":
            multiplier = driver_teeth / teeth
        elif gear_name == "upper_output":
            multiplier = -driver_teeth / teeth
        elif gear_name == "lower_idler":
            multiplier = driver_teeth / teeth
        else:
            multiplier = -driver_teeth / teeth
        mimic = Mimic("base_to_drive_pinion", multiplier=multiplier, offset=0.0)

    model.articulation(
        f"base_to_{gear_name}",
        ArticulationType.CONTINUOUS,
        parent=base,
        child=part,
        origin=Origin(xyz=(float(x), float(y), GEAR_Z)),
        axis=(0.0, 0.0, 1.0),
        motion_limits=MotionLimits(effort=35.0, velocity=9.0),
        motion_properties=MotionProperties(damping=0.015, friction=0.01),
        mimic=mimic,
    )


def build_object_model() -> ArticulatedObject:
    model = ArticulatedObject(name="industrial_intermeshing_gear_train")
    materials = _make_materials(model)

    base = model.part("base")
    _add_guard_frame(base, materials)
    for gear_name, spec in GEAR_SPECS.items():
        x, y = spec["center"]
        _add_bearing_support(base, gear_name, float(x), float(y), materials)

    for gear_name, spec in GEAR_SPECS.items():
        _add_gear_part(model, base, gear_name, spec, materials)

    return model


def run_tests() -> TestReport:
    ctx = TestContext(object_model)

    for gear_name in GEAR_SPECS:
        ctx.allow_overlap(
            "base",
            gear_name,
            elem_a=f"{gear_name}_bearing_ring",
            elem_b="bearing_journal",
            reason=(
                "The rotating bearing journal is intentionally seated into the "
                "annular bearing lip so the gear reads as physically supported."
            ),
        )
        ctx.expect_within(
            gear_name,
            "base",
            axes="xy",
            inner_elem="bearing_journal",
            outer_elem=f"{gear_name}_bearing_ring",
            margin=0.002,
            name=f"{gear_name} journal is centered in bearing ring",
        )
        ctx.expect_overlap(
            gear_name,
            "base",
            axes="z",
            elem_a="bearing_journal",
            elem_b=f"{gear_name}_bearing_ring",
            min_overlap=0.010,
            name=f"{gear_name} journal remains seated in bearing ring",
        )

    ctx.check(
        "six articulated gears",
        all(object_model.get_part(name) is not None for name in GEAR_SPECS),
        details="Every named gear in the train should be an articulated part.",
    )

    for gear_name, spec in GEAR_SPECS.items():
        joint = object_model.get_articulation(f"base_to_{gear_name}")
        ctx.check(
            f"{gear_name} uses vertical continuous bearing",
            joint.articulation_type == ArticulationType.CONTINUOUS and tuple(joint.axis) == (0.0, 0.0, 1.0),
            details=f"{joint.name}: type={joint.articulation_type}, axis={joint.axis}",
        )

    for first, second in GEAR_MESH_PAIRS:
        first_spec = GEAR_SPECS[first]
        second_spec = GEAR_SPECS[second]
        expected = (
            _pitch_radius(int(first_spec["teeth"]))
            + _pitch_radius(int(second_spec["teeth"]))
            + CENTER_CLEARANCE
        )
        ctx.expect_origin_distance(
            first,
            second,
            axes="xy",
            min_dist=expected - 0.004,
            max_dist=expected + 0.004,
            name=f"{first} meshes with {second} at pitch spacing",
        )

    drive = object_model.get_articulation("base_to_drive_pinion")
    rest_position = ctx.part_world_position("upper_output")
    with ctx.pose({drive: 1.0}):
        driven_position = ctx.part_world_position("upper_output")
    ctx.check(
        "gear train spins without translating shafts",
        rest_position is not None
        and driven_position is not None
        and all(abs(rest_position[i] - driven_position[i]) < 1e-6 for i in range(3)),
        details=f"rest={rest_position}, driven={driven_position}",
    )

    return ctx.report()


object_model = build_object_model()
