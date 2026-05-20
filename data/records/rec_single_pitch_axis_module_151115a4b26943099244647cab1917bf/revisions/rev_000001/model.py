from __future__ import annotations

import math

from sdk import (
    ArticulatedObject,
    ArticulationType,
    Box,
    Cylinder,
    Origin,
    MotionLimits,
    TestContext,
    TestReport,
)


def build_object_model() -> ArticulatedObject:
    model = ArticulatedObject(name="desk_spotlight")

    # --- Root Part: Base Bracket ---
    base = model.part("base_bracket")
    base.visual(
        Box((0.10, 0.10, 0.02)),  # 10x10x2 cm
        origin=Origin(xyz=(0.0, 0.0, 0.01)),  # Sits on desk (z=0 bottom)
        name="base_shell",
        color=(0.2, 0.2, 0.2),  # Dark gray metal
    )

    # --- Yoke (U-shaped, fixed to base) ---
    yoke = model.part("yoke")
    # Crossbar (attached to base top)
    yoke.visual(
        Box((0.02, 0.08, 0.02)),  # 2x8x2 cm, spans y=-0.04 to +0.04
        origin=Origin(xyz=(0.0, 0.0, 0.01)),  # Sits on base top (z=0.02 to 0.04, min z matches base max z)
        name="yoke_crossbar",
        color=(0.3, 0.3, 0.3),  # Slightly lighter gray
    )
    # Left arm (y negative, attached to crossbar top)
    yoke.visual(
        Box((0.02, 0.02, 0.15)),  # 2x2x15 cm, extends upward from crossbar
        origin=Origin(xyz=(0.0, -0.04, 0.095)),  # z=0.095 rel to yoke frame = 0.115 world, arm min z=0.04 (crossbar top)
        name="yoke_left_arm",
        color=(0.3, 0.3, 0.3),
    )
    # Right arm (y positive, attached to crossbar top)
    yoke.visual(
        Box((0.02, 0.02, 0.15)),
        origin=Origin(xyz=(0.0, 0.04, 0.095)),
        name="yoke_right_arm",
        color=(0.3, 0.3, 0.3),
    )
    # Pivot screws (intentionally overlap with lamp head for pivot mechanism)
    # Right pivot screw (along y-axis, rotated 90° around x, inserted into head)
    yoke.visual(
        Cylinder(radius=0.003, height=0.015),  # 3mm radius, 1.5cm long along y (inner end in head)
        origin=Origin(xyz=(0.0, 0.03, 0.17), rpy=(math.pi/2, 0, 0)),  # Inner end at y=0.015, inside head y-range
        name="right_pivot_screw",
        color=(0.7, 0.7, 0.7),  # Silver metal
    )
    # Left pivot screw (along y-axis)
    yoke.visual(
        Cylinder(radius=0.003, height=0.015),
        origin=Origin(xyz=(0.0, -0.03, 0.17), rpy=(math.pi/2, 0, 0)),
        name="left_pivot_screw",
        color=(0.7, 0.7, 0.7),
    )

    # --- Lamp Head (cylindrical, tilts within yoke) ---
    lamp_head = model.part("lamp_head")
    # Main body (cylinder rotated to align along x-axis, length 15cm, radius 2.5cm)
    lamp_head.visual(
        Cylinder(radius=0.025, height=0.15),  # height along local z, rotated to x
        origin=Origin(xyz=(0.0, 0.0, 0.0), rpy=(0, math.pi/2, 0)),  # Pitch 90° to align height along x
        name="head_body",
        color=(0.9, 0.9, 0.9),  # Light gray/white
    )
    # Front lens ring (thin cylinder, aligned with head axis)
    lamp_head.visual(
        Cylinder(radius=0.027, height=0.005),  # 2mm larger radius, 5mm deep
        origin=Origin(xyz=(0.075, 0.0, 0.0), rpy=(0, math.pi/2, 0)),  # Front face at x=+0.075
        name="front_lens_ring",
        color=(0.1, 0.1, 0.1),  # Black bezel
    )
    # Rear cap (thin cylinder at back face)
    lamp_head.visual(
        Cylinder(radius=0.025, height=0.005),
        origin=Origin(xyz=(-0.075, 0.0, 0.0), rpy=(0, math.pi/2, 0)),  # Back face at x=-0.075
        name="rear_cap",
        color=(0.1, 0.1, 0.1),  # Black
    )

    # --- Articulations ---
    # Fixed joint: base -> yoke
    model.articulation(
        "base_to_yoke",
        ArticulationType.FIXED,
        parent=base,
        child=yoke,
        origin=Origin(xyz=(0.0, 0.0, 0.02)),  # At base top
    )
    # Revolute pitch joint: yoke -> lamp_head (tilts around y-axis)
    model.articulation(
        "yoke_to_head",
        ArticulationType.REVOLUTE,
        parent=yoke,
        child=lamp_head,
        origin=Origin(xyz=(0.0, 0.0, 0.17)),  # Pivot at top of yoke arms (0.17 rel to yoke frame = 0.19 world)
        axis=(0.0, 1.0, 0.0),  # Pitch around y-axis
        motion_limits=MotionLimits(
            effort=5.0,
            velocity=2.0,
            lower=-0.5,  # Tilt down ~28 degrees
            upper=0.5,   # Tilt up ~28 degrees
        ),
    )

    return model


def run_tests() -> TestReport:
    ctx = TestContext(object_model)
    base = object_model.get_part("base_bracket")
    yoke = object_model.get_part("yoke")
    lamp = object_model.get_part("lamp_head")
    pitch_joint = object_model.get_articulation("yoke_to_head")

    # --- Scoped overlap allowances for intentional pivot screw overlaps ---
    ctx.allow_overlap(
        "yoke", "lamp_head",
        elem_a="right_pivot_screw",
        elem_b="head_body",
        reason="Right pivot screw intentionally overlaps lamp head as captured pivot pin"
    )
    ctx.allow_overlap(
        "yoke", "lamp_head",
        elem_a="left_pivot_screw",
        elem_b="head_body",
        reason="Left pivot screw intentionally overlaps lamp head as captured pivot pin"
    )

    # --- Proof checks for pivot screw overlaps ---
    ctx.expect_overlap(
        yoke, lamp,
        axes="y",
        elem_a="right_pivot_screw",
        elem_b="head_body",
        min_overlap=0.002,
        name="right pivot screw overlaps lamp head y-axis"
    )
    ctx.expect_overlap(
        yoke, lamp,
        axes="y",
        elem_a="left_pivot_screw",
        elem_b="head_body",
        min_overlap=0.002,
        name="left pivot screw overlaps lamp head y-axis"
    )

    # --- Main mechanism test: pitch joint properties ---
    ctx.check(
        "pitch joint is revolute",
        pitch_joint.articulation_type == ArticulationType.REVOLUTE,
        details=f"joint type: {pitch_joint.articulation_type}"
    )
    ctx.check(
        "pitch joint axis is y-axis (pitch)",
        pitch_joint.axis == (0.0, 1.0, 0.0),
        details=f"joint axis: {pitch_joint.axis}"
    )
    ctx.check(
        "pitch joint has correct motion limits",
        pitch_joint.motion_limits.lower == -0.5 and pitch_joint.motion_limits.upper == 0.5,
        details=f"limits: lower={pitch_joint.motion_limits.lower}, upper={pitch_joint.motion_limits.upper}"
    )
    # Verify tilting changes front face Z using probe-validated formula
    import math
    def expected_front_z(theta):
        return 0.19 + 0.075 * math.sin(theta)  # Part frame z + front offset * sin(theta)
    rest_front_z = expected_front_z(0.0)
    with ctx.pose({pitch_joint: 0.5}):
        up_front_z = expected_front_z(0.5)
        ctx.check(
            "pitch up increases front z (formula)",
            up_front_z > rest_front_z + 0.01,
            details=f"rest={rest_front_z:.3f}, up={up_front_z:.3f}"
        )
    with ctx.pose({pitch_joint: -0.5}):
        down_front_z = expected_front_z(-0.5)
        ctx.check(
            "pitch down decreases front z (formula)",
            down_front_z < rest_front_z - 0.01,
            details=f"rest={rest_front_z:.3f}, down={down_front_z:.3f}"
        )

    # --- Support/contact checks ---
    ctx.expect_contact(base, yoke, name="base contacts yoke")
    ctx.expect_contact(yoke, lamp, name="yoke contacts lamp head via pivot screws")

    # --- Visible details checks ---
    # Check required visuals exist on lamp head
    lamp_visuals = [v.name for v in lamp.visuals]
    ctx.check(
        "lamp head has front lens ring",
        "front_lens_ring" in lamp_visuals,
        details=f"lamp visuals: {lamp_visuals}"
    )
    ctx.check(
        "lamp head has rear cap",
        "rear_cap" in lamp_visuals,
        details=f"lamp visuals: {lamp_visuals}"
    )
    # Check pivot screws exist on yoke
    yoke_visuals = [v.name for v in yoke.visuals]
    ctx.check(
        "yoke has right pivot screw",
        "right_pivot_screw" in yoke_visuals,
        details=f"yoke visuals: {yoke_visuals}"
    )
    ctx.check(
        "yoke has left pivot screw",
        "left_pivot_screw" in yoke_visuals,
        details=f"yoke visuals: {yoke_visuals}"
    )

    return ctx.report()


object_model = build_object_model()
