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


def _capsule_profile(length: float, radius: float, *, segments_per_end: int = 24):
    # 2-D capsule in local X/Z coordinates, later extruded through thickness.
    pts = []
    for i in range(segments_per_end + 1):
        a = -math.pi / 2.0 + math.pi * i / segments_per_end
        pts.append((length + radius * math.cos(a), radius * math.sin(a)))
    for i in range(segments_per_end + 1):
        a = math.pi / 2.0 + math.pi * i / segments_per_end
        pts.append((radius * math.cos(a), radius * math.sin(a)))
    return pts


def _dogbone_plate_mesh(
    length: float,
    outer_radius: float,
    hole_radius: float | tuple[float, float],
    thickness: float,
    name: str,
):
    root_hole, tip_hole = (
        hole_radius if isinstance(hole_radius, tuple) else (hole_radius, hole_radius)
    )
    profile = _capsule_profile(length, outer_radius)
    plate = (
        cq.Workplane("XY")
        .moveTo(*profile[0])
        .polyline(profile[1:])
        .close()
        .extrude(thickness)
        .faces(">Z")
        .workplane()
        .pushPoints([(0.0, 0.0)])
        .circle(root_hole)
        .cutThruAll()
        .faces(">Z")
        .workplane()
        .pushPoints([(length, 0.0)])
        .circle(tip_hole)
        .cutThruAll()
        .translate((0.0, 0.0, -thickness / 2.0))
        .rotate((0.0, 0.0, 0.0), (1.0, 0.0, 0.0), 90.0)
    )
    return mesh_from_cadquery(plate, name, tolerance=0.0005, angular_tolerance=0.05)


def build_object_model() -> ArticulatedObject:
    model = ArticulatedObject(name="fork_backed_telescoping_arm")

    dark = Material("dark_powder_coat", rgba=(0.05, 0.055, 0.06, 1.0))
    blue = Material("blue_anodized_link", rgba=(0.08, 0.20, 0.55, 1.0))
    steel = Material("brushed_steel", rgba=(0.72, 0.74, 0.73, 1.0))
    orange = Material("orange_output_nose", rgba=(0.95, 0.42, 0.08, 1.0))

    support = model.part("rear_support")
    # The support part frame is the rear pivot axis.  A broad foot and back
    # plate carry an open fork, so the first link is visibly captured between
    # two cheeks instead of floating off the stand.
    support.visual(
        Box((0.26, 0.18, 0.035)),
        origin=Origin(xyz=(-0.055, 0.0, -0.190)),
        material=dark,
        name="base_foot",
    )
    support.visual(
        Box((0.050, 0.18, 0.240)),
        origin=Origin(xyz=(-0.095, 0.0, -0.070)),
        material=dark,
        name="rear_web",
    )
    for side, y in (("upper", 0.0475), ("lower", -0.0475)):
        support.visual(
            Box((0.170, 0.025, 0.130)),
            origin=Origin(xyz=(0.015, y, 0.0)),
            material=dark,
            name=f"fork_cheek_{side}",
        )
    support.visual(
        Cylinder(radius=0.0205, length=0.155),
        origin=Origin(rpy=(math.pi / 2.0, 0.0, 0.0)),
        material=steel,
        name="rear_pin",
    )
    for side, y in (("upper", 0.082), ("lower", -0.082)):
        support.visual(
            Cylinder(radius=0.024, length=0.009),
            origin=Origin(xyz=(0.0, y, 0.0), rpy=(math.pi / 2.0, 0.0, 0.0)),
            material=steel,
            name=f"pin_cap_{side}",
        )

    link_0_len = 0.42
    arm_link_0 = model.part("arm_link_0")
    arm_link_0.visual(
        _dogbone_plate_mesh(link_0_len, 0.052, (0.020, 0.017), 0.030, "arm_link_0_plate"),
        material=blue,
        name="main_plate",
    )
    arm_link_0.visual(
        Cylinder(radius=0.0175, length=0.110),
        origin=Origin(xyz=(link_0_len, 0.0, 0.0), rpy=(math.pi / 2.0, 0.0, 0.0)),
        material=steel,
        name="elbow_pin",
    )

    link_1_len = 0.36
    arm_link_1 = model.part("arm_link_1")
    cheek_y = 0.034
    for side, visual_name, y in (
        ("upper", "side_plate_upper", cheek_y),
        ("lower", "side_plate_lower", -cheek_y),
    ):
        arm_link_1.visual(
            _dogbone_plate_mesh(link_1_len, 0.045, 0.017, 0.016, f"arm_link_1_plate_{side}"),
            origin=Origin(xyz=(0.0, y, 0.0)),
            material=blue,
            name=visual_name,
        )
    # Rectangular telescoping sleeve at the nose of the second link.  Four
    # separate walls leave a true clear cavity for the sliding output member.
    arm_link_1.visual(
        Box((0.260, 0.082, 0.012)),
        origin=Origin(xyz=(0.300, 0.0, 0.031)),
        material=dark,
        name="sleeve_top",
    )
    arm_link_1.visual(
        Box((0.260, 0.082, 0.012)),
        origin=Origin(xyz=(0.300, 0.0, -0.031)),
        material=dark,
        name="sleeve_bottom",
    )
    for side, y in (("upper", 0.0335), ("lower", -0.0335)):
        arm_link_1.visual(
            Box((0.260, 0.015, 0.074)),
            origin=Origin(xyz=(0.300, y, 0.0)),
            material=dark,
            name=f"sleeve_side_{side}",
        )
    arm_link_1.visual(
        Box((0.024, 0.082, 0.074)),
        origin=Origin(xyz=(0.180, 0.0, 0.0)),
        material=dark,
        name="sleeve_rear_collar",
    )

    nose_extension = model.part("nose_extension")
    nose_extension.visual(
        Box((0.250, 0.052, 0.024)),
        origin=Origin(xyz=(-0.075, 0.0, 0.0)),
        material=steel,
        name="slide_bar",
    )
    nose_extension.visual(
        Box((0.052, 0.050, 0.036)),
        origin=Origin(xyz=(0.071, 0.0, 0.0)),
        material=orange,
        name="output_block",
    )
    nose_extension.visual(
        Cylinder(radius=0.025, length=0.044),
        origin=Origin(xyz=(0.105, 0.0, 0.0), rpy=(math.pi / 2.0, 0.0, 0.0)),
        material=orange,
        name="output_boss",
    )

    model.articulation(
        "rear_pivot",
        ArticulationType.REVOLUTE,
        parent=support,
        child=arm_link_0,
        origin=Origin(),
        axis=(0.0, -1.0, 0.0),
        motion_limits=MotionLimits(effort=90.0, velocity=1.8, lower=-0.75, upper=1.15),
    )
    model.articulation(
        "elbow_pivot",
        ArticulationType.REVOLUTE,
        parent=arm_link_0,
        child=arm_link_1,
        origin=Origin(xyz=(link_0_len, 0.0, 0.0)),
        axis=(0.0, -1.0, 0.0),
        motion_limits=MotionLimits(effort=65.0, velocity=2.0, lower=-1.35, upper=1.35),
    )
    model.articulation(
        "nose_slide",
        ArticulationType.PRISMATIC,
        parent=arm_link_1,
        child=nose_extension,
        origin=Origin(xyz=(0.420, 0.0, 0.0)),
        axis=(1.0, 0.0, 0.0),
        motion_limits=MotionLimits(effort=45.0, velocity=0.35, lower=0.0, upper=0.160),
    )

    return model


def run_tests() -> TestReport:
    ctx = TestContext(object_model)
    support = object_model.get_part("rear_support")
    arm_link_0 = object_model.get_part("arm_link_0")
    arm_link_1 = object_model.get_part("arm_link_1")
    nose_extension = object_model.get_part("nose_extension")

    rear_pivot = object_model.get_articulation("rear_pivot")
    elbow_pivot = object_model.get_articulation("elbow_pivot")
    nose_slide = object_model.get_articulation("nose_slide")

    ctx.allow_overlap(
        support,
        arm_link_0,
        elem_a="rear_pin",
        elem_b="main_plate",
        reason="The rear pivot pin is intentionally captured through the first-link pivot eye.",
    )
    ctx.allow_overlap(
        arm_link_0,
        arm_link_1,
        elem_a="elbow_pin",
        elem_b="side_plate_upper",
        reason="The elbow pin is intentionally captured through the forked second-link cheek.",
    )
    ctx.allow_overlap(
        arm_link_0,
        arm_link_1,
        elem_a="elbow_pin",
        elem_b="side_plate_lower",
        reason="The elbow pin is intentionally captured through the forked second-link cheek.",
    )

    ctx.expect_overlap(
        support,
        arm_link_0,
        axes="xz",
        elem_a="rear_pin",
        elem_b="main_plate",
        min_overlap=0.025,
        name="rear pin crosses first link eye",
    )
    ctx.expect_overlap(
        arm_link_0,
        arm_link_1,
        axes="y",
        elem_a="elbow_pin",
        elem_b="side_plate_upper",
        min_overlap=0.010,
        name="elbow pin engages upper fork cheek",
    )
    ctx.expect_overlap(
        arm_link_0,
        arm_link_1,
        axes="y",
        elem_a="elbow_pin",
        elem_b="side_plate_lower",
        min_overlap=0.010,
        name="elbow pin engages lower fork cheek",
    )
    ctx.expect_within(
        nose_extension,
        arm_link_1,
        axes="yz",
        inner_elem="slide_bar",
        outer_elem="sleeve_top",
        margin=0.040,
        name="slide bar stays centered within sleeve envelope",
    )
    ctx.expect_overlap(
        nose_extension,
        arm_link_1,
        axes="x",
        elem_a="slide_bar",
        elem_b="sleeve_top",
        min_overlap=0.180,
        name="collapsed nose remains deeply inserted",
    )

    rest_elbow_pos = ctx.part_world_position(arm_link_1)
    with ctx.pose({rear_pivot: 0.55}):
        raised_elbow_pos = ctx.part_world_position(arm_link_1)
    ctx.check(
        "rear pivot raises the linked arm",
        rest_elbow_pos is not None
        and raised_elbow_pos is not None
        and raised_elbow_pos[2] > rest_elbow_pos[2] + 0.18,
        details=f"rest={rest_elbow_pos}, raised={raised_elbow_pos}",
    )

    rest_nose_pos = ctx.part_world_position(nose_extension)
    with ctx.pose({elbow_pivot: 0.55}):
        folded_nose_pos = ctx.part_world_position(nose_extension)
    ctx.check(
        "elbow pivot swings the nose upward",
        rest_nose_pos is not None
        and folded_nose_pos is not None
        and folded_nose_pos[2] > rest_nose_pos[2] + 0.18,
        details=f"rest={rest_nose_pos}, folded={folded_nose_pos}",
    )

    rest_slide_pos = ctx.part_world_position(nose_extension)
    with ctx.pose({nose_slide: 0.160}):
        extended_slide_pos = ctx.part_world_position(nose_extension)
        ctx.expect_overlap(
            nose_extension,
            arm_link_1,
            axes="x",
            elem_a="slide_bar",
            elem_b="sleeve_top",
            min_overlap=0.035,
            name="extended nose retains sleeve insertion",
        )
        ctx.expect_within(
            nose_extension,
            arm_link_1,
            axes="yz",
            inner_elem="slide_bar",
            outer_elem="sleeve_top",
            margin=0.040,
            name="extended nose stays aligned in sleeve",
        )
    ctx.check(
        "prismatic nose extends forward",
        rest_slide_pos is not None
        and extended_slide_pos is not None
        and extended_slide_pos[0] > rest_slide_pos[0] + 0.150,
        details=f"rest={rest_slide_pos}, extended={extended_slide_pos}",
    )

    return ctx.report()


object_model = build_object_model()
