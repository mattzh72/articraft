from __future__ import annotations

import math

from sdk import (
    ArticulatedObject,
    ArticulationType,
    Box,
    Cylinder,
    MotionLimits,
    Origin,
    TestContext,
    TestReport,
)


UPPER_DX = 0.44
UPPER_DZ = 0.22
UPPER_LEN = math.hypot(UPPER_DX, UPPER_DZ)
UPPER_PITCH = -math.atan2(UPPER_DZ, UPPER_DX)

FOREARM_DX = 0.35
FOREARM_DZ = -0.18
FOREARM_LEN = math.hypot(FOREARM_DX, FOREARM_DZ)
FOREARM_PITCH = -math.atan2(FOREARM_DZ, FOREARM_DX)


def _along_link(distance: float, pitch: float, y: float = 0.0) -> tuple[float, float, float]:
    """Center a feature a given distance along a pitched local +X link."""
    return (distance * math.cos(pitch), y, -distance * math.sin(pitch))


def build_object_model() -> ArticulatedObject:
    model = ArticulatedObject(name="compact_service_elbow_arm")

    bracket_paint = model.material("bracket_paint", rgba=(0.10, 0.12, 0.14, 1.0))
    arm_paint = model.material("arm_paint", rgba=(0.96, 0.48, 0.10, 1.0))
    forearm_paint = model.material("forearm_paint", rgba=(0.88, 0.90, 0.88, 1.0))
    dark_metal = model.material("dark_metal", rgba=(0.03, 0.035, 0.04, 1.0))
    face_metal = model.material("face_metal", rgba=(0.62, 0.64, 0.62, 1.0))

    root = model.part("root_bracket")
    root.visual(
        Box((0.42, 0.30, 0.045)),
        origin=Origin(xyz=(0.0, 0.0, 0.0225)),
        material=bracket_paint,
        name="base_plate",
    )
    root.visual(
        Box((0.11, 0.22, 0.43)),
        origin=Origin(xyz=(-0.13, 0.0, 0.237)),
        material=bracket_paint,
        name="rear_column",
    )
    root.visual(
        Box((0.12, 0.22, 0.12)),
        origin=Origin(xyz=(-0.15, 0.0, 0.45)),
        material=bracket_paint,
        name="shoulder_back",
    )
    for side, y in (("near", -0.097), ("far", 0.097)):
        root.visual(
            Box((0.16, 0.035, 0.25)),
            origin=Origin(xyz=(0.0, y, 0.45)),
            material=bracket_paint,
            name=f"{side}_shoulder_cheek",
        )
        root.visual(
            Box((0.09, 0.035, 0.18)),
            origin=Origin(xyz=(-0.095, y, 0.45)),
            material=bracket_paint,
            name=f"{side}_cheek_neck",
        )
        root.visual(
            Cylinder(radius=0.085, length=0.040),
            origin=Origin(xyz=(0.0, 0.125 if y > 0 else -0.125, 0.45), rpy=(-math.pi / 2, 0.0, 0.0)),
            material=dark_metal,
            name=f"{side}_shoulder_bearing",
        )
    root.visual(
        Cylinder(radius=0.018, length=0.27),
        origin=Origin(xyz=(0.0, 0.0, 0.45), rpy=(-math.pi / 2, 0.0, 0.0)),
        material=dark_metal,
        name="shoulder_pin",
    )
    for i, (x, y) in enumerate(((-0.15, -0.10), (-0.15, 0.10), (0.15, -0.10), (0.15, 0.10))):
        root.visual(
            Cylinder(radius=0.017, length=0.012),
            origin=Origin(xyz=(x, y, 0.047), rpy=(0.0, 0.0, 0.0)),
            material=dark_metal,
            name=f"anchor_bolt_{i}",
        )

    upper = model.part("upper_arm")
    upper.visual(
        Cylinder(radius=0.074, length=0.130),
        origin=Origin(xyz=(0.0, 0.0, 0.0), rpy=(-math.pi / 2, 0.0, 0.0)),
        material=arm_paint,
        name="shoulder_hub",
    )
    upper.visual(
        Box((0.340, 0.150, 0.155)),
        origin=Origin(xyz=_along_link(0.235, UPPER_PITCH), rpy=(0.0, UPPER_PITCH, 0.0)),
        material=arm_paint,
        name="upper_spine",
    )
    for side, y in (("near", -0.072), ("far", 0.072)):
        upper.visual(
            Box((0.145, 0.025, 0.118)),
            origin=Origin(xyz=_along_link(0.440, UPPER_PITCH, y), rpy=(0.0, UPPER_PITCH, 0.0)),
            material=arm_paint,
            name=f"{side}_elbow_fork",
        )
        upper.visual(
            Cylinder(radius=0.073, length=0.032),
            origin=Origin(
                xyz=(UPPER_DX, 0.084 if y > 0 else -0.084, UPPER_DZ),
                rpy=(-math.pi / 2, 0.0, 0.0),
            ),
            material=dark_metal,
            name=f"{side}_elbow_cap",
        )
    upper.visual(
        Cylinder(radius=0.017, length=0.205),
        origin=Origin(xyz=(UPPER_DX, 0.0, UPPER_DZ), rpy=(-math.pi / 2, 0.0, 0.0)),
        material=dark_metal,
        name="elbow_pin",
    )

    forearm = model.part("forearm")
    forearm.visual(
        Cylinder(radius=0.058, length=0.090),
        origin=Origin(xyz=(0.0, 0.0, 0.0), rpy=(-math.pi / 2, 0.0, 0.0)),
        material=forearm_paint,
        name="elbow_hub",
    )
    forearm.visual(
        Box((0.340, 0.082, 0.086)),
        origin=Origin(xyz=_along_link(0.225, FOREARM_PITCH), rpy=(0.0, FOREARM_PITCH, 0.0)),
        material=forearm_paint,
        name="forearm_spine",
    )
    forearm.visual(
        Box((0.120, 0.092, 0.026)),
        origin=Origin(xyz=_along_link(0.150, FOREARM_PITCH), rpy=(0.0, FOREARM_PITCH, 0.0)),
        material=dark_metal,
        name="service_label",
    )

    terminal = model.part("terminal_face")
    terminal.visual(
        Box((0.080, 0.052, 0.052)),
        origin=Origin(xyz=(0.041, 0.0, 0.0)),
        material=face_metal,
        name="face_neck",
    )
    terminal.visual(
        Box((0.035, 0.155, 0.120)),
        origin=Origin(xyz=(0.098, 0.0, 0.0)),
        material=face_metal,
        name="face_plate",
    )
    for i, (y, z) in enumerate(((-0.050, -0.038), (-0.050, 0.038), (0.050, -0.038), (0.050, 0.038))):
        terminal.visual(
            Cylinder(radius=0.010, length=0.010),
            origin=Origin(xyz=(0.119, y, z), rpy=(0.0, math.pi / 2, 0.0)),
            material=dark_metal,
            name=f"face_bolt_{i}",
        )

    model.articulation(
        "shoulder",
        ArticulationType.REVOLUTE,
        parent=root,
        child=upper,
        origin=Origin(xyz=(0.0, 0.0, 0.45)),
        axis=(0.0, -1.0, 0.0),
        motion_limits=MotionLimits(effort=85.0, velocity=1.2, lower=-0.55, upper=0.75),
    )
    model.articulation(
        "elbow",
        ArticulationType.REVOLUTE,
        parent=upper,
        child=forearm,
        origin=Origin(xyz=(UPPER_DX, 0.0, UPPER_DZ)),
        axis=(0.0, -1.0, 0.0),
        motion_limits=MotionLimits(effort=55.0, velocity=1.5, lower=-1.05, upper=1.10),
    )
    model.articulation(
        "face_mount",
        ArticulationType.FIXED,
        parent=forearm,
        child=terminal,
        origin=Origin(xyz=(FOREARM_DX, 0.0, FOREARM_DZ), rpy=(0.0, FOREARM_PITCH, 0.0)),
    )

    return model


def run_tests() -> TestReport:
    ctx = TestContext(object_model)
    root = object_model.get_part("root_bracket")
    upper = object_model.get_part("upper_arm")
    forearm = object_model.get_part("forearm")
    terminal = object_model.get_part("terminal_face")
    shoulder = object_model.get_articulation("shoulder")
    elbow = object_model.get_articulation("elbow")

    ctx.allow_overlap(
        root,
        upper,
        elem_a="shoulder_pin",
        elem_b="shoulder_hub",
        reason="The shoulder pin is intentionally captured through the upper-arm hub.",
    )
    ctx.expect_within(
        root,
        upper,
        axes="xz",
        inner_elem="shoulder_pin",
        outer_elem="shoulder_hub",
        margin=0.002,
        name="shoulder pin sits inside hub bore",
    )
    ctx.expect_overlap(
        root,
        upper,
        axes="y",
        elem_a="shoulder_pin",
        elem_b="shoulder_hub",
        min_overlap=0.120,
        name="shoulder pin spans the hub",
    )

    ctx.allow_overlap(
        upper,
        forearm,
        elem_a="elbow_pin",
        elem_b="elbow_hub",
        reason="The elbow pin is intentionally captured through the forearm hub.",
    )
    ctx.expect_within(
        upper,
        forearm,
        axes="xz",
        inner_elem="elbow_pin",
        outer_elem="elbow_hub",
        margin=0.002,
        name="elbow pin sits inside hub bore",
    )
    ctx.expect_overlap(
        upper,
        forearm,
        axes="y",
        elem_a="elbow_pin",
        elem_b="elbow_hub",
        min_overlap=0.085,
        name="elbow pin spans the hub",
    )

    ctx.allow_overlap(
        forearm,
        terminal,
        elem_a="forearm_spine",
        elem_b="face_neck",
        reason="The terminal neck is modeled as a short seated plug inside the forearm end.",
    )
    ctx.expect_within(
        terminal,
        forearm,
        axes="y",
        inner_elem="face_neck",
        outer_elem="forearm_spine",
        margin=0.002,
        name="terminal neck is laterally captured",
    )
    ctx.expect_overlap(
        terminal,
        forearm,
        axes="x",
        elem_a="face_neck",
        elem_b="forearm_spine",
        min_overlap=0.020,
        name="terminal neck has retained insertion",
    )

    root_aabb = ctx.part_world_aabb(root)
    ctx.check(
        "root bracket is grounded",
        root_aabb is not None and abs(root_aabb[0][2]) < 0.001,
        details=f"root_aabb={root_aabb}",
    )
    ctx.check(
        "two-joint revolute chain",
        shoulder.articulation_type == ArticulationType.REVOLUTE
        and elbow.articulation_type == ArticulationType.REVOLUTE,
        details=f"shoulder={shoulder.articulation_type}, elbow={elbow.articulation_type}",
    )
    ctx.expect_contact(
        terminal,
        forearm,
        elem_a="face_neck",
        elem_b="forearm_spine",
        contact_tol=0.003,
        name="terminal face seats on forearm end",
    )

    closed_tip = ctx.part_world_position(terminal)
    with ctx.pose({shoulder: 0.45, elbow: -0.45}):
        moved_tip = ctx.part_world_position(terminal)
    ctx.check(
        "shoulder and elbow move the terminal face",
        closed_tip is not None
        and moved_tip is not None
        and math.dist(closed_tip, moved_tip) > 0.08,
        details=f"closed={closed_tip}, moved={moved_tip}",
    )

    return ctx.report()


object_model = build_object_model()
