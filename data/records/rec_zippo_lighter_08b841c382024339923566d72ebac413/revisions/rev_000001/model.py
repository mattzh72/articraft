from __future__ import annotations

import math

import cadquery as cq

from sdk import (
    ArticulatedObject,
    ArticulationType,
    Box,
    Cylinder,
    KnobGeometry,
    KnobGrip,
    Material,
    Mimic,
    MotionLimits,
    Origin,
    Sphere,
    TestContext,
    TestReport,
    mesh_from_cadquery,
    mesh_from_geometry,
)


CASE_W = 0.038
CASE_D = 0.013
LOWER_H = 0.038
LID_H = 0.019
WALL = 0.00125
HINGE_X = -CASE_W / 2.0 - 0.00155
HINGE_GAP = -CASE_W / 2.0 - HINGE_X


def _lower_case_shell() -> cq.Workplane:
    """Rounded, open-topped metal cup for the lower lighter case."""
    outer = (
        cq.Workplane("XY")
        .box(CASE_W, CASE_D, LOWER_H, centered=(True, True, False))
        .edges("|Z")
        .fillet(0.0027)
    )
    inner_cut = (
        cq.Workplane("XY")
        .box(CASE_W - 2 * WALL, CASE_D - 2 * WALL, LOWER_H + 0.004, centered=(True, True, False))
        .translate((0.0, 0.0, WALL))
    )
    return outer.cut(inner_cut)


def _lid_shell() -> cq.Workplane:
    """Rounded cap modeled in the lid's hinge-line local frame."""
    center_x = HINGE_GAP + CASE_W / 2.0
    outer = (
        cq.Workplane("XY")
        .box(CASE_W, CASE_D, LID_H, centered=(True, True, False))
        .translate((center_x, 0.0, 0.0))
        .edges("|Z")
        .fillet(0.0027)
    )
    cavity = (
        cq.Workplane("XY")
        .box(CASE_W - 2 * WALL, CASE_D - 2 * WALL, LID_H, centered=(True, True, False))
        .translate((center_x, 0.0, -0.0020))
    )
    return outer.cut(cavity)


def build_object_model() -> ArticulatedObject:
    model = ArticulatedObject(name="classic_flip_top_lighter")

    polished_chrome = Material("polished_chrome", rgba=(0.78, 0.76, 0.70, 1.0))
    brushed_steel = Material("brushed_steel", rgba=(0.62, 0.63, 0.60, 1.0))
    dark_hole = Material("dark_chimney_holes", rgba=(0.015, 0.014, 0.012, 1.0))
    wick_fiber = Material("braided_wick", rgba=(0.86, 0.74, 0.55, 1.0))
    charred_tip = Material("charred_wick_tip", rgba=(0.04, 0.035, 0.030, 1.0))
    brass = Material("warm_brass", rgba=(0.83, 0.60, 0.28, 1.0))

    lower_case = model.part("lower_case")
    lower_case.visual(
        mesh_from_cadquery(_lower_case_shell(), "lower_case_shell", tolerance=0.00035),
        material=polished_chrome,
        name="case_shell",
    )
    # The root-side hinge barrels are split along the short depth axis so the
    # lid's middle knuckle can sit between them without collision.
    for name, y in (("hinge_barrel_0", -0.0041), ("hinge_barrel_1", 0.0041)):
        lower_case.visual(
            Cylinder(radius=0.00155, length=0.0036),
            origin=Origin(xyz=(HINGE_X, y, LOWER_H), rpy=(math.pi / 2, 0.0, 0.0)),
            material=polished_chrome,
            name=name,
        )
    lower_case.visual(
        Box((0.0024, 0.0105, 0.0020)),
        origin=Origin(xyz=(HINGE_X + 0.0016, 0.0, LOWER_H - 0.0004)),
        material=polished_chrome,
        name="lower_hinge_leaf",
    )

    chimney = model.part("chimney")
    insert_h = LOWER_H - WALL
    chimney.visual(
        Box((0.0300, 0.0087, insert_h)),
        origin=Origin(xyz=(0.0010, 0.0, WALL + insert_h / 2.0)),
        material=brushed_steel,
        name="insert_body",
    )
    chimney.visual(
        Box((0.0260, 0.0091, 0.0014)),
        origin=Origin(xyz=(0.0010, 0.0, LOWER_H + 0.0007)),
        material=brushed_steel,
        name="chimney_deck",
    )
    # Two perforated chimney cheeks stand above the fitted insert.  Black inset
    # disks make the punched wind holes read clearly at pocket-lighter scale.
    for plate_name, y in (("front_chimney_wall", 0.0042), ("rear_chimney_wall", -0.0042)):
        chimney.visual(
            Box((0.0210, 0.00075, 0.0152)),
            origin=Origin(xyz=(0.0020, y, LOWER_H + 0.0085)),
            material=brushed_steel,
            name=plate_name,
        )
    hole_index = 0
    for y, roll in ((0.00465, math.pi / 2), (-0.00465, math.pi / 2)):
        for z in (LOWER_H + 0.0060, LOWER_H + 0.0100, LOWER_H + 0.0140):
            for x in (-0.0046, 0.0018, 0.0082):
                chimney.visual(
                    Cylinder(radius=0.00095, length=0.00028),
                    origin=Origin(xyz=(x, y, z), rpy=(roll, 0.0, 0.0)),
                    material=dark_hole,
                    name=f"wind_hole_{hole_index}",
                )
                hole_index += 1

    # Brackets straddle the separate striker wheel and visibly carry its short axle.
    for name, y in (("wheel_bracket_front", 0.003125), ("wheel_bracket_rear", -0.003125)):
        chimney.visual(
            Box((0.0038, 0.00290, 0.0088)),
            origin=Origin(xyz=(-0.0080, y, LOWER_H + 0.0060)),
            material=brushed_steel,
            name=name,
        )
    chimney.visual(
        Cylinder(radius=0.0010, length=0.0080),
        origin=Origin(xyz=(-0.0080, 0.0, LOWER_H + 0.0019)),
        material=brass,
        name="flint_tube",
    )
    chimney.visual(
        Cylinder(radius=0.00120, length=0.0090),
        origin=Origin(xyz=(0.0052, 0.0, LOWER_H + 0.0058)),
        material=wick_fiber,
        name="wick",
    )
    chimney.visual(
        Sphere(radius=0.00135),
        origin=Origin(xyz=(0.0052, 0.0, LOWER_H + 0.0107)),
        material=charred_tip,
        name="wick_tip",
    )

    lid = model.part("lid")
    lid.visual(
        mesh_from_cadquery(_lid_shell(), "lid_shell", tolerance=0.00035),
        material=polished_chrome,
        name="lid_shell",
    )
    lid.visual(
        Cylinder(radius=0.00150, length=0.0038),
        origin=Origin(xyz=(0.0, 0.0, 0.0), rpy=(math.pi / 2, 0.0, 0.0)),
        material=polished_chrome,
        name="lid_hinge_barrel",
    )
    lid.visual(
        Box((0.0024, 0.0042, 0.0019)),
        origin=Origin(xyz=(0.0017, 0.0, 0.0002)),
        material=polished_chrome,
        name="lid_hinge_leaf",
    )
    # Small fork inside the lid supports the separate cam lever pivot.
    for name, y in (("cam_pivot_ear_0", -0.00215), ("cam_pivot_ear_1", 0.00215)):
        lid.visual(
            Box((0.0024, 0.00055, 0.0118)),
            origin=Origin(xyz=(0.0062, y, 0.0120)),
            material=polished_chrome,
            name=name,
        )

    striker_wheel = model.part("striker_wheel")
    wheel_mesh = mesh_from_geometry(
        KnobGeometry(
            0.0062,
            0.0030,
            body_style="cylindrical",
            grip=KnobGrip(style="ribbed", count=28, depth=0.00035),
        ),
        "serrated_striker_wheel",
    )
    striker_wheel.visual(
        wheel_mesh,
        origin=Origin(rpy=(math.pi / 2, 0.0, 0.0)),
        material=brushed_steel,
        name="striker_disc",
    )

    cam_lever = model.part("cam_lever")
    cam_lever.visual(
        Cylinder(radius=0.00080, length=0.0031),
        origin=Origin(rpy=(math.pi / 2, 0.0, 0.0)),
        material=brass,
        name="cam_pivot",
    )
    cam_lever.visual(
        Box((0.0090, 0.0011, 0.0017)),
        origin=Origin(xyz=(0.0048, 0.0, -0.0010)),
        material=brass,
        name="lever_arm",
    )
    cam_lever.visual(
        Sphere(radius=0.00115),
        origin=Origin(xyz=(0.0093, 0.0, -0.0010)),
        material=brass,
        name="rounded_cam_nose",
    )

    model.articulation(
        "case_to_chimney",
        ArticulationType.FIXED,
        parent=lower_case,
        child=chimney,
        origin=Origin(),
    )
    model.articulation(
        "case_to_lid",
        ArticulationType.REVOLUTE,
        parent=lower_case,
        child=lid,
        origin=Origin(xyz=(HINGE_X, 0.0, LOWER_H)),
        axis=(0.0, -1.0, 0.0),
        motion_limits=MotionLimits(effort=2.0, velocity=4.0, lower=0.0, upper=2.05),
    )
    model.articulation(
        "chimney_to_striker_wheel",
        ArticulationType.CONTINUOUS,
        parent=chimney,
        child=striker_wheel,
        origin=Origin(xyz=(-0.0080, 0.0, LOWER_H + 0.0090)),
        axis=(0.0, 1.0, 0.0),
        motion_limits=MotionLimits(effort=0.05, velocity=25.0),
    )
    model.articulation(
        "lid_to_cam_lever",
        ArticulationType.REVOLUTE,
        parent=lid,
        child=cam_lever,
        origin=Origin(xyz=(0.0062, 0.0, 0.0070)),
        axis=(0.0, 1.0, 0.0),
        motion_limits=MotionLimits(effort=0.10, velocity=6.0, lower=-0.40, upper=0.20),
        mimic=Mimic("case_to_lid", multiplier=-0.22, offset=0.0),
    )

    return model


def run_tests() -> TestReport:
    ctx = TestContext(object_model)
    lower_case = object_model.get_part("lower_case")
    chimney = object_model.get_part("chimney")
    lid = object_model.get_part("lid")
    wheel = object_model.get_part("striker_wheel")
    cam = object_model.get_part("cam_lever")
    lid_joint = object_model.get_articulation("case_to_lid")
    wheel_joint = object_model.get_articulation("chimney_to_striker_wheel")

    ctx.allow_overlap(
        chimney,
        lower_case,
        elem_a="insert_body",
        elem_b="case_shell",
        reason=(
            "The insert is intentionally seated inside the lower case cavity; "
            "the visible lower case is a hollow mesh shell, so exact proxy overlap "
            "represents containment rather than metal passing through metal."
        ),
    )

    ctx.expect_gap(
        lid,
        lower_case,
        axis="z",
        positive_elem="lid_shell",
        negative_elem="case_shell",
        max_gap=0.0006,
        max_penetration=0.00005,
        name="closed lid shell sits on lower case rim",
    )
    ctx.expect_within(
        chimney,
        lower_case,
        axes="xy",
        inner_elem="insert_body",
        outer_elem="case_shell",
        margin=0.0001,
        name="insert body fits inside lower case shell",
    )
    ctx.expect_gap(
        chimney,
        wheel,
        axis="y",
        positive_elem="wheel_bracket_front",
        negative_elem="striker_disc",
        min_gap=0.0001,
        max_gap=0.0010,
        name="striker wheel clears front fork bracket",
    )
    ctx.check(
        "striker wheel uses continuous rotation",
        wheel_joint.articulation_type == ArticulationType.CONTINUOUS,
        details=f"type={wheel_joint.articulation_type}",
    )

    closed_lid_aabb = ctx.part_element_world_aabb(lid, elem="lid_shell")
    closed_cam_aabb = ctx.part_element_world_aabb(cam, elem="lever_arm")
    with ctx.pose({lid_joint: 1.55}):
        open_lid_aabb = ctx.part_element_world_aabb(lid, elem="lid_shell")
        open_cam_aabb = ctx.part_element_world_aabb(cam, elem="lever_arm")
    ctx.check(
        "lid rotates upward from side hinge",
        closed_lid_aabb is not None
        and open_lid_aabb is not None
        and open_lid_aabb[1][2] > closed_lid_aabb[1][2] + 0.020,
        details=f"closed={closed_lid_aabb}, open={open_lid_aabb}",
    )
    ctx.check(
        "cam lever moves as lid opens",
        closed_cam_aabb is not None
        and open_cam_aabb is not None
        and abs(open_cam_aabb[0][0] - closed_cam_aabb[0][0]) > 0.002,
        details=f"closed={closed_cam_aabb}, open={open_cam_aabb}",
    )

    return ctx.report()


object_model = build_object_model()
