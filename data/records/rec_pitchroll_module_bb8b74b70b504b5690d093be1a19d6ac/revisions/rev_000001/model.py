from __future__ import annotations

import math

import cadquery as cq

from sdk import (
    ArticulatedObject,
    ArticulationType,
    Box,
    Cylinder,
    Material,
    MotionLimits,
    Origin,
    TestContext,
    TestReport,
    mesh_from_cadquery,
)


PITCH_Z = 0.160
ROLL_X = 0.232
ROLL_BEARING_INNER_RADIUS = 0.031
ROLL_CARTRIDGE_RADIUS = 0.024
PITCH_JOURNAL_RADIUS = 0.0174
ROLL_INNER_RACE_RADIUS = 0.0314


def _box(size: tuple[float, float, float], center: tuple[float, float, float]) -> cq.Workplane:
    return cq.Workplane("XY").box(*size).translate(center)


def _y_axis_cylinder(radius: float, length: float, center: tuple[float, float, float]) -> cq.Workplane:
    return (
        cq.Workplane("XZ")
        .center(center[0], center[2])
        .circle(radius)
        .extrude(length, both=True)
        .translate((0.0, center[1], 0.0))
    )


def _annular_ring(
    *,
    outer_radius: float,
    inner_radius: float,
    length: float,
    axis: str,
    center: tuple[float, float, float],
) -> cq.Workplane:
    """Make a true hollow bearing ring centered on X or Y."""

    if axis == "x":
        ring = (
            cq.Workplane("YZ")
            .center(center[1], center[2])
            .circle(outer_radius)
            .circle(inner_radius)
            .extrude(length, both=True)
            .translate((center[0], 0.0, 0.0))
        )
    elif axis == "y":
        ring = (
            cq.Workplane("XZ")
            .center(center[0], center[2])
            .circle(outer_radius)
            .circle(inner_radius)
            .extrude(length, both=True)
            .translate((0.0, center[1], 0.0))
        )
    else:
        raise ValueError(f"unsupported ring axis {axis!r}")
    return ring


def _base_frame_shape() -> cq.Workplane:
    base = _box((0.430, 0.270, 0.026), (0.075, 0.0, 0.013))

    rear_bridge = _box((0.070, 0.205, 0.086), (-0.070, 0.0, 0.069))
    left_cheek = _box((0.090, 0.030, 0.182), (0.000, 0.094, 0.117))
    right_cheek = _box((0.090, 0.030, 0.182), (0.000, -0.094, 0.117))
    front_rail_0 = _box((0.270, 0.018, 0.030), (0.130, 0.116, 0.041))
    front_rail_1 = _box((0.270, 0.018, 0.030), (0.130, -0.116, 0.041))
    gusset_0 = _box((0.035, 0.028, 0.095), (-0.025, 0.070, 0.081))
    gusset_1 = _box((0.035, 0.028, 0.095), (-0.025, -0.070, 0.081))
    pitch_stop_0 = _box((0.048, 0.012, 0.010), (0.068, 0.116, 0.034))
    pitch_stop_1 = _box((0.048, 0.012, 0.010), (0.068, -0.116, 0.034))

    frame = (
        base.union(rear_bridge)
        .union(left_cheek)
        .union(right_cheek)
        .union(front_rail_0)
        .union(front_rail_1)
        .union(gusset_0)
        .union(gusset_1)
        .union(pitch_stop_0)
        .union(pitch_stop_1)
    )

    # True clearance bores through the fixed pitch side trunnions.
    pitch_clearance = _y_axis_cylinder(0.020, 0.340, (0.0, 0.0, PITCH_Z))
    return frame.cut(pitch_clearance)


def _roll_bearing_shape(x: float) -> cq.Workplane:
    ring = _annular_ring(
        outer_radius=0.048,
        inner_radius=ROLL_BEARING_INNER_RADIUS,
        length=0.017,
        axis="x",
        center=(x, 0.0, 0.0),
    )
    flange = _annular_ring(
        outer_radius=0.055,
        inner_radius=ROLL_BEARING_INNER_RADIUS,
        length=0.006,
        axis="x",
        center=(x - 0.011, 0.0, 0.0),
    )
    return ring.union(flange)


def _rounded_bolt(radius: float = 0.0038, height: float = 0.0040) -> Cylinder:
    return Cylinder(radius=radius, length=height)


def build_object_model() -> ArticulatedObject:
    model = ArticulatedObject(name="fork_spindle_sensor_bracket")

    dark_anodized = model.material("dark_anodized", rgba=(0.06, 0.065, 0.070, 1.0))
    satin_black = model.material("satin_black", rgba=(0.010, 0.012, 0.014, 1.0))
    bearing_steel = model.material("bearing_steel", rgba=(0.55, 0.57, 0.58, 1.0))
    cap_screw = model.material("black_oxide_screw", rgba=(0.005, 0.005, 0.004, 1.0))
    sensor_shell = model.material("sensor_shell", rgba=(0.11, 0.12, 0.13, 1.0))
    glass = model.material("smoked_glass", rgba=(0.05, 0.16, 0.23, 0.72))

    base = model.part("base_frame")
    base.visual(
        mesh_from_cadquery(_base_frame_shape(), "base_frame_shell", tolerance=0.001),
        material=dark_anodized,
        name="base_shell",
    )

    for name, y in (("pitch_cap_0", 0.122), ("pitch_cap_1", -0.122)):
        base.visual(
            mesh_from_cadquery(
                _annular_ring(
                    outer_radius=0.034,
                    inner_radius=0.017,
                    length=0.024,
                    axis="y",
                    center=(0.0, y, PITCH_Z),
                ),
                name,
                tolerance=0.0007,
            ),
            material=bearing_steel,
            name=name,
        )

    for y, suffix, face_sign in ((0.136, "0", 1.0), (-0.136, "1", -1.0)):
        for i, (dx, dz) in enumerate(
            ((0.024, 0.024), (-0.024, 0.024), (0.024, -0.024), (-0.024, -0.024))
        ):
            base.visual(
                _rounded_bolt(),
                origin=Origin(
                    xyz=(dx, y, PITCH_Z + dz),
                    rpy=(-face_sign * math.pi / 2.0, 0.0, 0.0),
                ),
                material=cap_screw,
                name=f"pitch_bolt_{suffix}_{i}",
            )

    pitch = model.part("pitch_yoke")
    # The pitch frame is centered on the trunnion axis.  The fork extends forward
    # along +X and carries the roll bearing pair at its nose.
    pitch.visual(
        Cylinder(radius=PITCH_JOURNAL_RADIUS, length=0.252),
        origin=Origin(rpy=(-math.pi / 2.0, 0.0, 0.0)),
        material=bearing_steel,
        name="pitch_spindle",
    )
    pitch.visual(
        Cylinder(radius=0.024, length=0.036),
        origin=Origin(xyz=(0.0, 0.060, 0.0), rpy=(-math.pi / 2.0, 0.0, 0.0)),
        material=dark_anodized,
        name="trunnion_hub_0",
    )
    pitch.visual(
        Cylinder(radius=0.024, length=0.036),
        origin=Origin(xyz=(0.0, -0.060, 0.0), rpy=(-math.pi / 2.0, 0.0, 0.0)),
        material=dark_anodized,
        name="trunnion_hub_1",
    )
    pitch.visual(
        Box((0.066, 0.096, 0.034)),
        origin=Origin(xyz=(0.027, 0.0, 0.0)),
        material=dark_anodized,
        name="rear_saddle",
    )
    pitch.visual(
        Box((0.210, 0.017, 0.024)),
        origin=Origin(xyz=(0.126, 0.041, 0.0)),
        material=dark_anodized,
        name="fork_arm_0",
    )
    pitch.visual(
        Box((0.210, 0.017, 0.024)),
        origin=Origin(xyz=(0.126, -0.041, 0.0)),
        material=dark_anodized,
        name="fork_arm_1",
    )
    pitch.visual(
        mesh_from_cadquery(
            _annular_ring(
                outer_radius=0.038,
                inner_radius=0.034,
                length=0.078,
                axis="x",
                center=(ROLL_X, 0.0, 0.0),
            ),
            "roll_bearing_sleeve",
            tolerance=0.0007,
        ),
        material=dark_anodized,
        name="roll_bearing_sleeve",
    )
    pitch.visual(
        mesh_from_cadquery(_roll_bearing_shape(0.206), "rear_roll_bearing", tolerance=0.0007),
        material=bearing_steel,
        name="rear_roll_bearing",
    )
    pitch.visual(
        mesh_from_cadquery(_roll_bearing_shape(0.258), "front_roll_bearing", tolerance=0.0007),
        material=bearing_steel,
        name="front_roll_bearing",
    )

    for bearing_x, label in ((0.206, "rear"), (0.258, "front")):
        for i, (dy, dz) in enumerate(((0.041, 0.024), (-0.041, 0.024), (0.041, -0.024), (-0.041, -0.024))):
            pitch.visual(
                _rounded_bolt(radius=0.0033, height=0.0060),
                origin=Origin(
                    xyz=(bearing_x - 0.017, dy, dz),
                    rpy=(0.0, math.pi / 2.0, 0.0),
                ),
                material=cap_screw,
                name=f"{label}_roll_bolt_{i}",
            )

    roll = model.part("roll_cartridge")
    roll.visual(
        Cylinder(radius=0.015, length=0.150),
        origin=Origin(xyz=(-0.020, 0.0, 0.0), rpy=(0.0, math.pi / 2.0, 0.0)),
        material=bearing_steel,
        name="roll_spindle",
    )
    roll.visual(
        Cylinder(radius=ROLL_INNER_RACE_RADIUS, length=0.011),
        origin=Origin(xyz=(0.206 - ROLL_X, 0.0, 0.0), rpy=(0.0, math.pi / 2.0, 0.0)),
        material=bearing_steel,
        name="rear_inner_race",
    )
    roll.visual(
        Cylinder(radius=ROLL_INNER_RACE_RADIUS, length=0.011),
        origin=Origin(xyz=(0.258 - ROLL_X, 0.0, 0.0), rpy=(0.0, math.pi / 2.0, 0.0)),
        material=bearing_steel,
        name="front_inner_race",
    )
    roll.visual(
        Cylinder(radius=ROLL_CARTRIDGE_RADIUS, length=0.112),
        origin=Origin(xyz=(0.028, 0.0, 0.0), rpy=(0.0, math.pi / 2.0, 0.0)),
        material=sensor_shell,
        name="sensor_body",
    )
    roll.visual(
        Cylinder(radius=0.0265, length=0.008),
        origin=Origin(xyz=(0.088, 0.0, 0.0), rpy=(0.0, math.pi / 2.0, 0.0)),
        material=satin_black,
        name="front_bezel",
    )
    roll.visual(
        Cylinder(radius=0.019, length=0.006),
        origin=Origin(xyz=(0.091, 0.0, 0.0), rpy=(0.0, math.pi / 2.0, 0.0)),
        material=glass,
        name="sensor_lens",
    )
    roll.visual(
        Box((0.004, 0.006, 0.030)),
        origin=Origin(xyz=(0.058, 0.0, 0.024)),
        material=cap_screw,
        name="roll_index_mark",
    )

    model.articulation(
        "base_to_yoke",
        ArticulationType.REVOLUTE,
        parent=base,
        child=pitch,
        origin=Origin(xyz=(0.0, 0.0, PITCH_Z)),
        axis=(0.0, 1.0, 0.0),
        motion_limits=MotionLimits(effort=18.0, velocity=1.5, lower=-0.24, upper=0.24),
    )
    model.articulation(
        "yoke_to_cartridge",
        ArticulationType.REVOLUTE,
        parent=pitch,
        child=roll,
        origin=Origin(xyz=(ROLL_X, 0.0, 0.0)),
        axis=(1.0, 0.0, 0.0),
        motion_limits=MotionLimits(effort=8.0, velocity=2.0, lower=-1.57, upper=1.57),
    )
    return model


def run_tests() -> TestReport:
    ctx = TestContext(object_model)
    base = object_model.get_part("base_frame")
    pitch = object_model.get_part("pitch_yoke")
    roll = object_model.get_part("roll_cartridge")
    pitch_joint = object_model.get_articulation("base_to_yoke")
    roll_joint = object_model.get_articulation("yoke_to_cartridge")

    ctx.allow_overlap(
        "base_frame",
        "pitch_yoke",
        elem_a="pitch_cap_0",
        elem_b="pitch_spindle",
        reason="The pitch journal is intentionally seated in the fixed bearing cap bore.",
    )
    ctx.allow_overlap(
        "base_frame",
        "pitch_yoke",
        elem_a="pitch_cap_1",
        elem_b="pitch_spindle",
        reason="The pitch journal is intentionally seated in the fixed bearing cap bore.",
    )
    ctx.allow_overlap(
        "pitch_yoke",
        "roll_cartridge",
        elem_a="rear_roll_bearing",
        elem_b="rear_inner_race",
        reason="The roll cartridge inner race is captured in the rear coaxial bearing.",
    )
    ctx.allow_overlap(
        "pitch_yoke",
        "roll_cartridge",
        elem_a="front_roll_bearing",
        elem_b="front_inner_race",
        reason="The roll cartridge inner race is captured in the front coaxial bearing.",
    )

    ctx.expect_within(
        "pitch_yoke",
        "base_frame",
        axes="y",
        inner_elem="pitch_spindle",
        outer_elem="base_shell",
        margin=0.002,
        name="pitch spindle is captured between the side trunnions",
    )
    ctx.expect_overlap(
        "pitch_yoke",
        "base_frame",
        axes="y",
        elem_a="pitch_spindle",
        elem_b="pitch_cap_0",
        min_overlap=0.010,
        name="pitch journal remains inserted in first trunnion cap",
    )
    ctx.expect_overlap(
        "pitch_yoke",
        "base_frame",
        axes="y",
        elem_a="pitch_spindle",
        elem_b="pitch_cap_1",
        min_overlap=0.010,
        name="pitch journal remains inserted in second trunnion cap",
    )
    ctx.expect_within(
        "roll_cartridge",
        "pitch_yoke",
        axes="yz",
        inner_elem="sensor_body",
        outer_elem="front_roll_bearing",
        margin=0.005,
        name="roll cartridge stays inside coaxial bearing envelope",
    )
    ctx.expect_overlap(
        "roll_cartridge",
        "pitch_yoke",
        axes="x",
        elem_a="rear_inner_race",
        elem_b="rear_roll_bearing",
        min_overlap=0.008,
        name="rear roll race stays seated in bearing",
    )
    ctx.expect_overlap(
        "roll_cartridge",
        "pitch_yoke",
        axes="x",
        elem_a="front_inner_race",
        elem_b="front_roll_bearing",
        min_overlap=0.008,
        name="front roll race stays seated in bearing",
    )
    ctx.check(
        "roll cartridge has radial bearing clearance",
        ROLL_CARTRIDGE_RADIUS + 0.004 <= ROLL_BEARING_INNER_RADIUS,
        details=f"body radius={ROLL_CARTRIDGE_RADIUS}, bearing bore={ROLL_BEARING_INNER_RADIUS}",
    )

    rest_aabb = ctx.part_world_aabb(roll)
    with ctx.pose({pitch_joint: pitch_joint.motion_limits.upper, roll_joint: 1.20}):
        pitched_up_aabb = ctx.part_world_aabb(roll)
        nose_aabb = ctx.part_element_world_aabb(pitch, elem="front_roll_bearing")
        ctx.check(
            "nose clears base plate at positive pitch stop",
            nose_aabb is not None and nose_aabb[0][2] > 0.030,
            details=f"front_roll_bearing_aabb={nose_aabb}",
        )
    with ctx.pose({pitch_joint: pitch_joint.motion_limits.lower, roll_joint: -1.20}):
        nose_aabb = ctx.part_element_world_aabb(pitch, elem="front_roll_bearing")
        ctx.check(
            "nose clears base plate at negative pitch stop",
            nose_aabb is not None and nose_aabb[0][2] > 0.030,
            details=f"front_roll_bearing_aabb={nose_aabb}",
        )
    ctx.check(
        "pitch motion changes cartridge elevation",
        rest_aabb is not None
        and pitched_up_aabb is not None
        and abs(pitched_up_aabb[0][2] - rest_aabb[0][2]) > 0.020,
        details=f"rest={rest_aabb}, pitched={pitched_up_aabb}",
    )

    return ctx.report()


object_model = build_object_model()
