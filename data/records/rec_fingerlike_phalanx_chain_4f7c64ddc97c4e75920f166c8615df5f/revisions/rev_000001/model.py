from __future__ import annotations

import math

from sdk import (
    ArticulatedObject,
    ArticulationType,
    Box,
    Cylinder,
    Material,
    MeshGeometry,
    MotionLimits,
    Origin,
    TestContext,
    TestReport,
    mesh_from_geometry,
)


def _plate_prism(profile_xz: list[tuple[float, float]], thickness_y: float) -> MeshGeometry:
    """Extrude a chamfered link-side profile in XZ through a real plate thickness."""

    geom = MeshGeometry()
    half_y = thickness_y / 2.0
    for y in (-half_y, half_y):
        for x, z in profile_xz:
            geom.add_vertex(x, y, z)

    n = len(profile_xz)
    for i in range(1, n - 1):
        geom.add_face(0, i, i + 1)
        geom.add_face(n, n + i + 1, n + i)

    for i in range(n):
        j = (i + 1) % n
        geom.add_face(i, j, n + j)
        geom.add_face(i, n + j, n + i)

    return geom


def _link_plate_profile(length: float, root_half_z: float, tip_half_z: float) -> list[tuple[float, float]]:
    """One chamfered tapered side-plate silhouette from proximal to distal hinge."""

    return [
        (0.024, -root_half_z * 0.60),
        (0.050, -root_half_z),
        (length - 0.045, -tip_half_z),
        (length + 0.018, -tip_half_z * 0.58),
        (length + 0.018, tip_half_z * 0.58),
        (length - 0.045, tip_half_z),
        (0.058, root_half_z),
        (0.030, root_half_z * 0.60),
    ]


def _root_cheek_profile() -> list[tuple[float, float]]:
    return [
        (-0.068, -0.043),
        (-0.050, -0.052),
        (0.014, -0.052),
        (0.032, -0.035),
        (0.032, 0.035),
        (0.014, 0.052),
        (-0.050, 0.052),
        (-0.068, 0.043),
    ]


def build_object_model() -> ArticulatedObject:
    model = ArticulatedObject(name="industrial_finger_module")

    gunmetal = model.material("dark_hardcoat", rgba=(0.11, 0.12, 0.13, 1.0))
    steel = model.material("brushed_steel", rgba=(0.58, 0.60, 0.58, 1.0))
    edge_steel = model.material("machined_edges", rgba=(0.78, 0.78, 0.72, 1.0))
    black = model.material("black_rubber", rgba=(0.015, 0.014, 0.012, 1.0))
    stop_red = model.material("red_stop_marks", rgba=(0.75, 0.08, 0.04, 1.0))

    middle_len = 0.220
    distal_len = 0.135

    cyl_y = Origin(rpy=(math.pi / 2.0, 0.0, 0.0))

    root = model.part("root_knuckle")
    root.visual(
        Box((0.040, 0.180, 0.135)),
        origin=Origin(xyz=(-0.138, 0.0, 0.0)),
        material=gunmetal,
        name="mount_flange",
    )
    root.visual(
        Box((0.078, 0.140, 0.108)),
        origin=Origin(xyz=(-0.088, 0.0, 0.0)),
        material=gunmetal,
        name="rear_bridge",
    )
    root_cheek_mesh = _plate_prism(_root_cheek_profile(), 0.026)
    root.visual(
        mesh_from_geometry(root_cheek_mesh, "root_cheek_pos_mesh"),
        origin=Origin(xyz=(0.0, 0.061, 0.0)),
        material=gunmetal,
        name="root_cheek_pos",
    )
    root.visual(
        Cylinder(radius=0.038, length=0.026),
        origin=Origin(xyz=(0.0, 0.061, 0.0), rpy=cyl_y.rpy),
        material=steel,
        name="root_boss_pos",
    )
    root.visual(
        mesh_from_geometry(root_cheek_mesh, "root_cheek_neg_mesh"),
        origin=Origin(xyz=(0.0, -0.061, 0.0)),
        material=gunmetal,
        name="root_cheek_neg",
    )
    root.visual(
        Cylinder(radius=0.038, length=0.026),
        origin=Origin(xyz=(0.0, -0.061, 0.0), rpy=cyl_y.rpy),
        material=steel,
        name="root_boss_neg",
    )
    root.visual(
        Cylinder(radius=0.014, length=0.168),
        origin=Origin(rpy=cyl_y.rpy),
        material=edge_steel,
        name="root_axle",
    )
    for side, y in (("pos", 0.082), ("neg", -0.082)):
        root.visual(
            Cylinder(radius=0.018, length=0.010),
            origin=Origin(xyz=(0.0, y, 0.0), rpy=cyl_y.rpy),
            material=edge_steel,
            name=f"root_pin_cap_{side}",
        )
    root.visual(
        Box((0.018, 0.125, 0.008)),
        origin=Origin(xyz=(-0.036, 0.0, 0.056)),
        material=edge_steel,
        name="top_clamp_land",
    )
    root.visual(
        Box((0.018, 0.125, 0.008)),
        origin=Origin(xyz=(-0.036, 0.0, -0.056)),
        material=edge_steel,
        name="bottom_clamp_land",
    )
    root.visual(
        Box((0.010, 0.030, 0.028)),
        origin=Origin(xyz=(0.026, 0.036, 0.044)),
        material=stop_red,
        name="root_stop_tab",
    )

    middle = model.part("middle_link")
    middle_plate = _plate_prism(_link_plate_profile(middle_len, 0.033, 0.026), 0.012)
    middle.visual(
        mesh_from_geometry(middle_plate, "middle_plate_pos_mesh"),
        origin=Origin(xyz=(0.0, 0.036, 0.0)),
        material=gunmetal,
        name="middle_plate_pos",
    )
    middle.visual(
        Cylinder(radius=0.027, length=0.012),
        origin=Origin(xyz=(middle_len, 0.039, 0.0), rpy=cyl_y.rpy),
        material=steel,
        name="middle_boss_pos",
    )
    middle.visual(
        mesh_from_geometry(middle_plate, "middle_plate_neg_mesh"),
        origin=Origin(xyz=(0.0, -0.036, 0.0)),
        material=gunmetal,
        name="middle_plate_neg",
    )
    middle.visual(
        Cylinder(radius=0.027, length=0.012),
        origin=Origin(xyz=(middle_len, -0.039, 0.0), rpy=cyl_y.rpy),
        material=steel,
        name="middle_boss_neg",
    )
    middle.visual(
        Cylinder(radius=0.032, length=0.082),
        origin=Origin(rpy=cyl_y.rpy),
        material=steel,
        name="proximal_barrel",
    )
    middle.visual(
        Cylinder(radius=0.011, length=0.090),
        origin=Origin(xyz=(middle_len, 0.0, 0.0), rpy=cyl_y.rpy),
        material=edge_steel,
        name="middle_axle",
    )
    middle.visual(
        Box((0.046, 0.080, 0.009)),
        origin=Origin(xyz=(0.105, 0.0, 0.030)),
        material=edge_steel,
        name="top_spacer",
    )
    middle.visual(
        Box((0.046, 0.080, 0.009)),
        origin=Origin(xyz=(0.105, 0.0, -0.030)),
        material=edge_steel,
        name="bottom_spacer",
    )
    middle.visual(
        Box((0.058, 0.006, 0.016)),
        origin=Origin(xyz=(0.124, 0.043, 0.0)),
        material=edge_steel,
        name="side_milled_land_pos",
    )
    middle.visual(
        Box((0.058, 0.006, 0.016)),
        origin=Origin(xyz=(0.124, -0.043, 0.0)),
        material=edge_steel,
        name="side_milled_land_neg",
    )

    distal = model.part("distal_link")
    distal_plate = _plate_prism(_link_plate_profile(distal_len, 0.026, 0.021), 0.009)
    distal.visual(
        mesh_from_geometry(distal_plate, "distal_plate_pos_mesh"),
        origin=Origin(xyz=(0.0, 0.024, 0.0)),
        material=gunmetal,
        name="distal_plate_pos",
    )
    distal.visual(
        Cylinder(radius=0.022, length=0.011),
        origin=Origin(xyz=(distal_len, 0.024, 0.0), rpy=cyl_y.rpy),
        material=steel,
        name="distal_boss_pos",
    )
    distal.visual(
        mesh_from_geometry(distal_plate, "distal_plate_neg_mesh"),
        origin=Origin(xyz=(0.0, -0.024, 0.0)),
        material=gunmetal,
        name="distal_plate_neg",
    )
    distal.visual(
        Cylinder(radius=0.022, length=0.011),
        origin=Origin(xyz=(distal_len, -0.024, 0.0), rpy=cyl_y.rpy),
        material=steel,
        name="distal_boss_neg",
    )
    distal.visual(
        Cylinder(radius=0.025, length=0.052),
        origin=Origin(rpy=cyl_y.rpy),
        material=steel,
        name="middle_barrel",
    )
    distal.visual(
        Box((0.042, 0.006, 0.026)),
        origin=Origin(xyz=(0.033, 0.017, 0.0)),
        material=gunmetal,
        name="proximal_web_pos",
    )
    distal.visual(
        Box((0.042, 0.006, 0.026)),
        origin=Origin(xyz=(0.033, -0.017, 0.0)),
        material=gunmetal,
        name="proximal_web_neg",
    )
    distal.visual(
        Cylinder(radius=0.009, length=0.065),
        origin=Origin(xyz=(distal_len, 0.0, 0.0), rpy=cyl_y.rpy),
        material=edge_steel,
        name="distal_axle",
    )
    distal.visual(
        Box((0.032, 0.056, 0.007)),
        origin=Origin(xyz=(0.070, 0.0, 0.024)),
        material=edge_steel,
        name="distal_top_spacer",
    )
    distal.visual(
        Box((0.032, 0.056, 0.007)),
        origin=Origin(xyz=(0.070, 0.0, -0.024)),
        material=edge_steel,
        name="distal_bottom_spacer",
    )

    pad = model.part("pad_tip")
    pad.visual(
        Cylinder(radius=0.018, length=0.034),
        origin=Origin(rpy=cyl_y.rpy),
        material=steel,
        name="distal_barrel",
    )
    pad.visual(
        Box((0.042, 0.030, 0.030)),
        origin=Origin(xyz=(0.033, 0.0, 0.0)),
        material=gunmetal,
        name="pad_neck",
    )
    pad.visual(
        Box((0.050, 0.108, 0.058)),
        origin=Origin(xyz=(0.076, 0.0, 0.0)),
        material=gunmetal,
        name="pad_carrier",
    )
    pad.visual(
        Box((0.018, 0.122, 0.068)),
        origin=Origin(xyz=(0.108, 0.0, 0.0)),
        material=black,
        name="rubber_insert",
    )
    pad.visual(
        Box((0.004, 0.110, 0.010)),
        origin=Origin(xyz=(0.118, 0.0, 0.020)),
        material=edge_steel,
        name="pad_wear_line_top",
    )
    pad.visual(
        Box((0.004, 0.110, 0.010)),
        origin=Origin(xyz=(0.118, 0.0, -0.020)),
        material=edge_steel,
        name="pad_wear_line_bottom",
    )

    model.articulation(
        "root_joint",
        ArticulationType.REVOLUTE,
        parent=root,
        child=middle,
        origin=Origin(),
        axis=(0.0, 1.0, 0.0),
        motion_limits=MotionLimits(effort=180.0, velocity=2.0, lower=0.0, upper=1.05),
    )
    model.articulation(
        "middle_joint",
        ArticulationType.REVOLUTE,
        parent=middle,
        child=distal,
        origin=Origin(xyz=(middle_len, 0.0, 0.0)),
        axis=(0.0, 1.0, 0.0),
        motion_limits=MotionLimits(effort=120.0, velocity=2.4, lower=0.0, upper=1.15),
    )
    model.articulation(
        "distal_joint",
        ArticulationType.REVOLUTE,
        parent=distal,
        child=pad,
        origin=Origin(xyz=(distal_len, 0.0, 0.0)),
        axis=(0.0, 1.0, 0.0),
        motion_limits=MotionLimits(effort=80.0, velocity=2.8, lower=0.0, upper=0.85),
    )

    return model


def run_tests() -> TestReport:
    ctx = TestContext(object_model)
    root = object_model.get_part("root_knuckle")
    middle = object_model.get_part("middle_link")
    distal = object_model.get_part("distal_link")
    pad = object_model.get_part("pad_tip")

    root_joint = object_model.get_articulation("root_joint")
    middle_joint = object_model.get_articulation("middle_joint")
    distal_joint = object_model.get_articulation("distal_joint")

    joints = (root_joint, middle_joint, distal_joint)
    ctx.allow_overlap(
        root,
        middle,
        elem_a="root_axle",
        elem_b="proximal_barrel",
        reason="The root hinge axle is intentionally captured inside the proximal barrel bushing.",
    )
    ctx.allow_overlap(
        middle,
        distal,
        elem_a="middle_axle",
        elem_b="middle_barrel",
        reason="The middle hinge axle is intentionally captured inside the distal bushing barrel.",
    )
    ctx.allow_overlap(
        distal,
        pad,
        elem_a="distal_axle",
        elem_b="distal_barrel",
        reason="The distal hinge axle is intentionally captured inside the pad-tip bushing barrel.",
    )
    ctx.check(
        "three serial revolute joints",
        all(j.articulation_type == ArticulationType.REVOLUTE for j in joints),
        details=str([j.articulation_type for j in joints]),
    )
    ctx.check(
        "coplanar bend axes",
        all(tuple(round(v, 6) for v in j.axis) == (0.0, 1.0, 0.0) for j in joints),
        details=str([j.axis for j in joints]),
    )
    ctx.expect_within(
        root,
        middle,
        axes="xz",
        inner_elem="root_axle",
        outer_elem="proximal_barrel",
        margin=0.001,
        name="root axle is centered in proximal barrel",
    )
    ctx.expect_overlap(
        root,
        middle,
        axes="y",
        elem_a="root_axle",
        elem_b="proximal_barrel",
        min_overlap=0.070,
        name="root axle spans the proximal barrel",
    )
    ctx.expect_within(
        middle,
        distal,
        axes="xz",
        inner_elem="middle_axle",
        outer_elem="middle_barrel",
        margin=0.001,
        name="middle axle is centered in distal barrel",
    )
    ctx.expect_overlap(
        middle,
        distal,
        axes="y",
        elem_a="middle_axle",
        elem_b="middle_barrel",
        min_overlap=0.045,
        name="middle axle spans the distal barrel",
    )
    ctx.expect_within(
        distal,
        pad,
        axes="xz",
        inner_elem="distal_axle",
        outer_elem="distal_barrel",
        margin=0.001,
        name="distal axle is centered in pad barrel",
    )
    ctx.expect_overlap(
        distal,
        pad,
        axes="y",
        elem_a="distal_axle",
        elem_b="distal_barrel",
        min_overlap=0.030,
        name="distal axle spans the pad barrel",
    )

    ctx.expect_gap(
        root,
        middle,
        axis="y",
        positive_elem="root_cheek_pos",
        negative_elem="proximal_barrel",
        min_gap=0.003,
        name="middle barrel clears positive root cheek",
    )
    ctx.expect_gap(
        middle,
        root,
        axis="y",
        positive_elem="proximal_barrel",
        negative_elem="root_cheek_neg",
        min_gap=0.003,
        name="middle barrel clears negative root cheek",
    )
    ctx.expect_gap(
        middle,
        distal,
        axis="y",
        positive_elem="middle_plate_pos",
        negative_elem="middle_barrel",
        min_gap=0.002,
        name="distal barrel clears positive middle plate",
    )
    ctx.expect_gap(
        distal,
        middle,
        axis="y",
        positive_elem="middle_barrel",
        negative_elem="middle_plate_neg",
        min_gap=0.002,
        name="distal barrel clears negative middle plate",
    )
    ctx.expect_gap(
        distal,
        pad,
        axis="y",
        positive_elem="distal_plate_pos",
        negative_elem="distal_barrel",
        min_gap=0.001,
        name="pad barrel clears positive distal plate",
    )
    ctx.expect_gap(
        pad,
        distal,
        axis="y",
        positive_elem="distal_barrel",
        negative_elem="distal_plate_neg",
        min_gap=0.001,
        name="pad barrel clears negative distal plate",
    )

    rest_tip = ctx.part_world_position(pad)
    with ctx.pose({root_joint: 0.85, middle_joint: 0.85, distal_joint: 0.55}):
        flexed_tip = ctx.part_world_position(pad)
        ctx.expect_gap(
            root,
            middle,
            axis="y",
            positive_elem="root_cheek_pos",
            negative_elem="proximal_barrel",
            min_gap=0.003,
            name="root hinge keeps side clearance while flexed",
        )
        ctx.expect_gap(
            middle,
            distal,
            axis="y",
            positive_elem="middle_plate_pos",
            negative_elem="middle_barrel",
            min_gap=0.002,
            name="middle hinge keeps side clearance while flexed",
        )

    with ctx.pose({root_joint: 1.05, middle_joint: 1.15, distal_joint: 0.85}):
        ctx.expect_gap(
            middle,
            distal,
            axis="y",
            positive_elem="middle_boss_pos",
            negative_elem="distal_plate_pos",
            min_gap=0.003,
            name="positive middle boss clears distal plate at full curl",
        )
        ctx.expect_gap(
            distal,
            middle,
            axis="y",
            positive_elem="distal_plate_neg",
            negative_elem="middle_boss_neg",
            min_gap=0.003,
            name="negative middle boss clears distal plate at full curl",
        )
        ctx.expect_gap(
            distal,
            pad,
            axis="y",
            positive_elem="distal_boss_pos",
            negative_elem="distal_barrel",
            min_gap=0.001,
            name="positive distal boss clears pad barrel at full curl",
        )
        ctx.expect_gap(
            pad,
            distal,
            axis="y",
            positive_elem="distal_barrel",
            negative_elem="distal_boss_neg",
            min_gap=0.001,
            name="negative distal boss clears pad barrel at full curl",
        )

    ctx.check(
        "finger curls downward",
        rest_tip is not None and flexed_tip is not None and flexed_tip[2] < rest_tip[2] - 0.06,
        details=f"rest={rest_tip}, flexed={flexed_tip}",
    )

    return ctx.report()


object_model = build_object_model()
