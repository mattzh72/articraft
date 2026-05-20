from __future__ import annotations

import math

from sdk import (
    ArticulatedObject,
    ArticulationType,
    Box,
    Cylinder,
    ExtrudeGeometry,
    MotionLimits,
    Origin,
    TestContext,
    TestReport,
    mesh_from_geometry,
)


PIN_RPY = (-math.pi / 2.0, 0.0, 0.0)


def tapered_web_mesh(
    *,
    name: str,
    start_x: float,
    end_x: float,
    start_height: float,
    end_height: float,
    thickness: float,
):
    """Flat tapered link web, authored in X/Z profile and extruded through Y."""
    profile = [
        (start_x, -start_height / 2.0),
        (end_x, -end_height / 2.0),
        (end_x, end_height / 2.0),
        (start_x, start_height / 2.0),
    ]
    geom = ExtrudeGeometry(profile, thickness, center=True)
    geom.rotate_x(math.pi / 2.0)
    return mesh_from_geometry(geom, name)


def build_object_model() -> ArticulatedObject:
    model = ArticulatedObject(name="bridge_mounted_folding_arm_chain")

    painted_steel = model.material("painted_steel", rgba=(0.18, 0.20, 0.22, 1.0))
    dark_link = model.material("dark_anodized_link", rgba=(0.08, 0.095, 0.105, 1.0))
    light_edge = model.material("machined_chamfer", rgba=(0.42, 0.44, 0.45, 1.0))
    pin_steel = model.material("brushed_pin_steel", rgba=(0.70, 0.68, 0.60, 1.0))
    rubber = model.material("matte_rubber_pad", rgba=(0.02, 0.02, 0.018, 1.0))

    bridge = model.part("bridge")
    bridge.visual(
        Box((0.34, 0.26, 0.055)),
        origin=Origin(xyz=(-0.15, 0.0, 0.985)),
        material=painted_steel,
        name="bridge_beam",
    )
    bridge.visual(
        Box((0.050, 0.18, 0.245)),
        origin=Origin(xyz=(-0.080, 0.0, 0.845)),
        material=painted_steel,
        name="hanger_web",
    )
    bridge.visual(
        Box((0.032, 0.145, 0.118)),
        origin=Origin(xyz=(-0.072, 0.0, 0.720)),
        material=painted_steel,
        name="clevis_back",
    )
    for cheek_name, cap_name, y in (
        ("root_cheek_side_0", "root_pin_cap_side_0", 0.062),
        ("root_cheek_side_1", "root_pin_cap_side_1", -0.062),
    ):
        bridge.visual(
            Box((0.142, 0.018, 0.124)),
            origin=Origin(xyz=(0.010, y, 0.720)),
            material=painted_steel,
            name=cheek_name,
        )
        bridge.visual(
            Cylinder(radius=0.030, length=0.014),
            origin=Origin(xyz=(0.0, 0.076 if y > 0 else -0.076, 0.720), rpy=PIN_RPY),
            material=pin_steel,
            name=cap_name,
        )
    bridge.visual(
        Cylinder(radius=0.018, length=0.150),
        origin=Origin(xyz=(0.0, 0.0, 0.720), rpy=PIN_RPY),
        material=pin_steel,
        name="root_pin_shaft",
    )

    link_0 = model.part("link_0")
    L0 = 0.420
    link_0.visual(
        Cylinder(radius=0.047, length=0.052),
        origin=Origin(xyz=(0.0, 0.0, 0.0), rpy=PIN_RPY),
        material=dark_link,
        name="proximal_boss",
    )
    link_0.visual(
        tapered_web_mesh(
            name="link_0_tapered_web",
            start_x=0.018,
            end_x=L0 - 0.072,
            start_height=0.078,
            end_height=0.060,
            thickness=0.048,
        ),
        material=dark_link,
        name="tapered_web",
    )
    for fork_name, cap_name, y in (
        ("distal_fork_side_0", "distal_pin_cap_side_0", 0.030),
        ("distal_fork_side_1", "distal_pin_cap_side_1", -0.030),
    ):
        link_0.visual(
            Box((0.128, 0.013, 0.070)),
            origin=Origin(xyz=(L0 - 0.045, y, 0.0)),
            material=dark_link,
            name=fork_name,
        )
        link_0.visual(
            Cylinder(radius=0.031, length=0.012),
            origin=Origin(xyz=(L0, 0.039 if y > 0 else -0.039, 0.0), rpy=PIN_RPY),
            material=pin_steel,
            name=cap_name,
        )
    link_0.visual(
        Cylinder(radius=0.015, length=0.080),
        origin=Origin(xyz=(L0, 0.0, 0.0), rpy=PIN_RPY),
        material=pin_steel,
        name="distal_pin_shaft",
    )
    link_0.visual(
        Box((0.050, 0.052, 0.046)),
        origin=Origin(xyz=(L0 - 0.105, 0.0, 0.0)),
        material=light_edge,
        name="fork_shoulder",
    )

    link_1 = model.part("link_1")
    L1 = 0.335
    link_1.visual(
        Cylinder(radius=0.037, length=0.038),
        origin=Origin(xyz=(0.0, 0.0, 0.0), rpy=PIN_RPY),
        material=dark_link,
        name="proximal_boss",
    )
    link_1.visual(
        tapered_web_mesh(
            name="link_1_tapered_web",
            start_x=0.016,
            end_x=L1 - 0.060,
            start_height=0.061,
            end_height=0.046,
            thickness=0.036,
        ),
        material=dark_link,
        name="tapered_web",
    )
    for fork_name, cap_name, y in (
        ("distal_fork_side_0", "distal_pin_cap_side_0", 0.023),
        ("distal_fork_side_1", "distal_pin_cap_side_1", -0.023),
    ):
        link_1.visual(
            Box((0.106, 0.011, 0.054)),
            origin=Origin(xyz=(L1 - 0.038, y, 0.0)),
            material=dark_link,
            name=fork_name,
        )
        link_1.visual(
            Cylinder(radius=0.024, length=0.010),
            origin=Origin(xyz=(L1, 0.030 if y > 0 else -0.030, 0.0), rpy=PIN_RPY),
            material=pin_steel,
            name=cap_name,
        )
    link_1.visual(
        Cylinder(radius=0.012, length=0.064),
        origin=Origin(xyz=(L1, 0.0, 0.0), rpy=PIN_RPY),
        material=pin_steel,
        name="distal_pin_shaft",
    )
    link_1.visual(
        Box((0.042, 0.039, 0.036)),
        origin=Origin(xyz=(L1 - 0.087, 0.0, 0.0)),
        material=light_edge,
        name="fork_shoulder",
    )

    link_2 = model.part("link_2")
    L2 = 0.235
    link_2.visual(
        Cylinder(radius=0.028, length=0.028),
        origin=Origin(xyz=(0.0, 0.0, 0.0), rpy=PIN_RPY),
        material=dark_link,
        name="proximal_boss",
    )
    link_2.visual(
        tapered_web_mesh(
            name="link_2_tapered_web",
            start_x=0.012,
            end_x=L2 + 0.024,
            start_height=0.045,
            end_height=0.026,
            thickness=0.026,
        ),
        material=dark_link,
        name="tapered_web",
    )
    link_2.visual(
        Box((0.024, 0.070, 0.052)),
        origin=Origin(xyz=(L2 + 0.043, 0.0, 0.0)),
        material=rubber,
        name="end_pad",
    )
    link_2.visual(
        Box((0.030, 0.030, 0.030)),
        origin=Origin(xyz=(L2 + 0.020, 0.0, 0.0)),
        material=light_edge,
        name="pad_mount",
    )

    model.articulation(
        "root_pivot",
        ArticulationType.REVOLUTE,
        parent=bridge,
        child=link_0,
        origin=Origin(xyz=(0.0, 0.0, 0.720)),
        axis=(0.0, -1.0, 0.0),
        motion_limits=MotionLimits(effort=30.0, velocity=2.5, lower=-1.15, upper=1.35),
    )
    model.articulation(
        "mid_pivot",
        ArticulationType.REVOLUTE,
        parent=link_0,
        child=link_1,
        origin=Origin(xyz=(L0, 0.0, 0.0)),
        axis=(0.0, -1.0, 0.0),
        motion_limits=MotionLimits(effort=20.0, velocity=2.8, lower=-1.45, upper=1.55),
    )
    model.articulation(
        "tip_pivot",
        ArticulationType.REVOLUTE,
        parent=link_1,
        child=link_2,
        origin=Origin(xyz=(L1, 0.0, 0.0)),
        axis=(0.0, -1.0, 0.0),
        motion_limits=MotionLimits(effort=12.0, velocity=3.0, lower=-1.60, upper=1.65),
    )

    return model


def run_tests() -> TestReport:
    ctx = TestContext(object_model)
    bridge = object_model.get_part("bridge")
    link_0 = object_model.get_part("link_0")
    link_1 = object_model.get_part("link_1")
    link_2 = object_model.get_part("link_2")
    root = object_model.get_articulation("root_pivot")
    mid = object_model.get_articulation("mid_pivot")
    tip = object_model.get_articulation("tip_pivot")

    ctx.check(
        "three revolute joints in series",
        len(object_model.articulations) == 3
        and all(j.articulation_type == ArticulationType.REVOLUTE for j in object_model.articulations),
        details=f"joints={[j.name for j in object_model.articulations]}",
    )
    ctx.allow_overlap(
        bridge,
        link_0,
        elem_a="root_pin_shaft",
        elem_b="proximal_boss",
        reason="The root pin is intentionally captured through the first link boss.",
    )
    ctx.allow_overlap(
        link_0,
        link_1,
        elem_a="distal_pin_shaft",
        elem_b="proximal_boss",
        reason="The middle pivot pin is intentionally captured through the second link boss.",
    )
    ctx.allow_overlap(
        link_1,
        link_2,
        elem_a="distal_pin_shaft",
        elem_b="proximal_boss",
        reason="The tip pivot pin is intentionally captured through the tapered end link boss.",
    )
    ctx.expect_within(
        link_0,
        bridge,
        axes="y",
        inner_elem="proximal_boss",
        outer_elem="clevis_back",
        margin=0.0,
        name="root boss sits between bridge clevis cheeks",
    )
    ctx.expect_overlap(
        link_0,
        bridge,
        axes="yz",
        elem_a="proximal_boss",
        elem_b="root_pin_shaft",
        min_overlap=0.030,
        name="root pin passes through first link boss",
    )
    ctx.expect_gap(
        link_0,
        link_1,
        axis="y",
        positive_elem="distal_fork_side_0",
        negative_elem="proximal_boss",
        min_gap=0.003,
        name="middle tongue clears upper fork cheek",
    )
    ctx.expect_overlap(
        link_1,
        link_0,
        axes="yz",
        elem_a="proximal_boss",
        elem_b="distal_pin_shaft",
        min_overlap=0.024,
        name="middle pin passes through second link boss",
    )
    ctx.expect_gap(
        link_1,
        link_2,
        axis="y",
        positive_elem="distal_fork_side_0",
        negative_elem="proximal_boss",
        min_gap=0.002,
        name="tip tongue clears smaller fork cheek",
    )
    ctx.expect_overlap(
        link_2,
        link_1,
        axes="yz",
        elem_a="proximal_boss",
        elem_b="distal_pin_shaft",
        min_overlap=0.020,
        name="tip pin passes through end link boss",
    )

    rest_tip = ctx.part_world_position(link_2)
    with ctx.pose({root: 0.45, mid: -0.75, tip: 0.85}):
        folded_tip = ctx.part_world_position(link_2)

    ctx.check(
        "folded pose changes distal pad height",
        rest_tip is not None and folded_tip is not None and abs(folded_tip[2] - rest_tip[2]) > 0.04,
        details=f"rest={rest_tip}, folded={folded_tip}",
    )

    return ctx.report()


object_model = build_object_model()
