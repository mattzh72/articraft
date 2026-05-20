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
    MotionLimits,
    Origin,
    TestContext,
    TestReport,
    mesh_from_cadquery,
    mesh_from_geometry,
)


CASE_W = 0.022
CASE_D = 0.010
LOWER_H = 0.048
LID_H = 0.026
WALL = 0.00115
CORNER_R = 0.0021
HINGE_R = 0.00155
HINGE_X = -CASE_W / 2.0 - HINGE_R
HINGE_Z = LOWER_H


def _open_top_shell(width: float, depth: float, height: float, wall: float, radius: float):
    """Thin rounded lighter lower shell: closed bottom, open top."""
    body = (
        cq.Workplane("XY")
        .box(width, depth, height, centered=(True, True, False))
        .edges("|Z")
        .fillet(radius)
    )
    return body.faces(">Z").shell(-wall)


def _open_bottom_lid(width: float, depth: float, height: float, wall: float, radius: float):
    """Thin rounded cap shell: closed top, open bottom, local hinge axis at x=0."""
    body = (
        cq.Workplane("XY")
        .box(width, depth, height, centered=(True, True, False))
        .edges("|Z")
        .fillet(radius)
        .translate((width / 2.0 + HINGE_R, 0.0, 0.0))
    )
    return body.faces("<Z").shell(-wall)


def build_object_model() -> ArticulatedObject:
    model = ArticulatedObject(name="slim_side_hinged_lighter")

    brass = model.material("brushed_warm_brass", rgba=(0.88, 0.66, 0.30, 1.0))
    steel = model.material("brushed_chrome", rgba=(0.72, 0.74, 0.72, 1.0))
    dark = model.material("blackened_shadow", rgba=(0.015, 0.014, 0.012, 1.0))
    flint = model.material("dark_knurled_steel", rgba=(0.10, 0.10, 0.095, 1.0))
    wick_mat = model.material("charred_wick", rgba=(0.025, 0.020, 0.014, 1.0))

    lower_shell = model.part("lower_shell")
    lower_shell.visual(
        mesh_from_cadquery(
            _open_top_shell(CASE_W, CASE_D, LOWER_H, WALL, CORNER_R),
            "lower_shell",
            tolerance=0.00035,
            angular_tolerance=0.08,
        ),
        material=brass,
        name="lower_shell_body",
    )
    lower_shell.visual(
        Box((0.00075, 0.0058, 0.020)),
        origin=Origin(xyz=(-CASE_W / 2.0 - 0.00035, 0.0, LOWER_H - 0.0105)),
        material=brass,
        name="lower_hinge_leaf",
    )
    lower_shell.visual(
        Cylinder(radius=HINGE_R, length=0.019),
        origin=Origin(xyz=(HINGE_X, 0.0, LOWER_H - 0.0100)),
        material=brass,
        name="lower_hinge_barrel",
    )
    # Small inward spring leaves make the removable insert visibly seated rather
    # than floating inside the case; they are hidden just below the rim.
    lower_shell.visual(
        Box((0.0120, 0.0010, 0.0090)),
        origin=Origin(xyz=(0.0, 0.00340, LOWER_H - 0.0045)),
        material=steel,
        name="front_insert_spring",
    )
    lower_shell.visual(
        Box((0.0120, 0.0010, 0.0090)),
        origin=Origin(xyz=(0.0, -0.00340, LOWER_H - 0.0045)),
        material=steel,
        name="rear_insert_spring",
    )

    insert = model.part("insert")
    insert.visual(
        Box((0.0172, 0.0062, 0.044)),
        origin=Origin(xyz=(0.0, 0.0, -0.022)),
        material=steel,
        name="insert_reservoir",
    )
    insert.visual(
        Box((0.0185, 0.00752, 0.0020)),
        origin=Origin(xyz=(0.0, 0.0, 0.0010)),
        material=steel,
        name="insert_lip",
    )

    plate_y = 0.00310
    insert.visual(
        Box((0.0133, 0.00055, 0.0170)),
        origin=Origin(xyz=(-0.0008, plate_y, 0.0105)),
        material=steel,
        name="front_chimney_plate",
    )
    insert.visual(
        Box((0.0133, 0.00055, 0.0170)),
        origin=Origin(xyz=(-0.0008, -plate_y, 0.0105)),
        material=steel,
        name="rear_chimney_plate",
    )
    for side, y in (("front", plate_y), ("rear", -plate_y)):
        # Dark recessed rectangles read as the air holes in the wind screen.
        for i, x in enumerate((-0.0047, -0.0013, 0.0021)):
            insert.visual(
                Box((0.00155, 0.00008, 0.0024)),
                origin=Origin(xyz=(x, y + math.copysign(0.000315, y), 0.0115)),
                material=dark,
                name=f"{side}_chimney_hole_{i}",
            )
        for i, x in enumerate((-0.0030, 0.0004)):
            insert.visual(
                Box((0.00155, 0.00008, 0.0021)),
                origin=Origin(xyz=(x, y + math.copysign(0.000315, y), 0.0160)),
                material=dark,
                name=f"{side}_upper_hole_{i}",
            )
    insert.visual(
        Cylinder(radius=0.0010, length=0.0070),
        origin=Origin(xyz=(0.0038, 0.0, 0.0055)),
        material=wick_mat,
        name="wick",
    )
    insert.visual(
        Box((0.0042, 0.0047, 0.0011)),
        origin=Origin(xyz=(0.0038, 0.0, 0.00255)),
        material=dark,
        name="wick_well",
    )

    lid = model.part("lid")
    lid.visual(
        mesh_from_cadquery(
            _open_bottom_lid(CASE_W, CASE_D, LID_H, WALL, CORNER_R),
            "lid_shell",
            tolerance=0.00035,
            angular_tolerance=0.08,
        ),
        material=brass,
        name="lid_shell",
    )
    lid.visual(
        Box((0.00075, 0.0058, 0.0185)),
        origin=Origin(xyz=(HINGE_R - 0.00032, 0.0, 0.0124)),
        material=brass,
        name="lid_hinge_leaf",
    )
    lid.visual(
        Cylinder(radius=HINGE_R, length=0.0185),
        origin=Origin(xyz=(0.0, 0.0, 0.0124)),
        material=brass,
        name="lid_hinge_barrel",
    )

    striker_wheel = model.part("striker_wheel")
    wheel_mesh = mesh_from_geometry(
        KnobGeometry(
            0.0062,
            0.0046,
            body_style="cylindrical",
            edge_radius=0.00025,
            grip=KnobGrip(style="fluted", count=28, depth=0.00045),
        ),
        "striker_wheel",
    )
    striker_wheel.visual(
        wheel_mesh,
        origin=Origin(rpy=(-math.pi / 2.0, 0.0, 0.0)),
        material=flint,
        name="knurled_wheel",
    )
    striker_wheel.visual(
        Cylinder(radius=0.00055, length=0.0070),
        origin=Origin(rpy=(-math.pi / 2.0, 0.0, 0.0)),
        material=steel,
        name="axle_pin",
    )

    model.articulation(
        "insert_slide",
        ArticulationType.PRISMATIC,
        parent=lower_shell,
        child=insert,
        origin=Origin(xyz=(0.0, 0.0, LOWER_H)),
        axis=(0.0, 0.0, 1.0),
        motion_limits=MotionLimits(effort=12.0, velocity=0.18, lower=0.0, upper=0.030),
    )
    model.articulation(
        "lid_hinge",
        ArticulationType.REVOLUTE,
        parent=lower_shell,
        child=lid,
        origin=Origin(xyz=(HINGE_X, 0.0, HINGE_Z)),
        axis=(0.0, 0.0, 1.0),
        motion_limits=MotionLimits(effort=3.0, velocity=5.0, lower=0.0, upper=1.95),
    )
    model.articulation(
        "wheel_axle",
        ArticulationType.CONTINUOUS,
        parent=insert,
        child=striker_wheel,
        origin=Origin(xyz=(-0.0045, 0.0, 0.0160)),
        axis=(0.0, 1.0, 0.0),
        motion_limits=MotionLimits(effort=0.15, velocity=40.0),
    )

    return model


def run_tests() -> TestReport:
    ctx = TestContext(object_model)
    lower_shell = object_model.get_part("lower_shell")
    insert = object_model.get_part("insert")
    lid = object_model.get_part("lid")
    striker_wheel = object_model.get_part("striker_wheel")
    insert_slide = object_model.get_articulation("insert_slide")
    lid_hinge = object_model.get_articulation("lid_hinge")
    wheel_axle = object_model.get_articulation("wheel_axle")

    ctx.allow_overlap(
        striker_wheel,
        insert,
        elem_a="axle_pin",
        elem_b="front_chimney_plate",
        reason="The small striker axle is intentionally captured through the front chimney bearing plate.",
    )
    ctx.allow_overlap(
        striker_wheel,
        insert,
        elem_a="axle_pin",
        elem_b="rear_chimney_plate",
        reason="The small striker axle is intentionally captured through the rear chimney bearing plate.",
    )
    ctx.allow_overlap(
        lower_shell,
        insert,
        elem_a="front_insert_spring",
        elem_b="insert_reservoir",
        reason="A small internal spring leaf intentionally presses the lift-out insert as a friction fit.",
    )
    ctx.allow_overlap(
        lower_shell,
        insert,
        elem_a="rear_insert_spring",
        elem_b="insert_reservoir",
        reason="A small internal spring leaf intentionally presses the lift-out insert as a friction fit.",
    )

    ctx.check(
        "slim pocket-lighter proportions",
        CASE_D < CASE_W < LOWER_H and CASE_W / CASE_D > 1.8 and (LOWER_H + LID_H) < 0.080,
        details=f"width={CASE_W}, depth={CASE_D}, height={LOWER_H + LID_H}",
    )
    ctx.check(
        "requested articulation types",
        insert_slide.articulation_type == ArticulationType.PRISMATIC
        and lid_hinge.articulation_type == ArticulationType.REVOLUTE
        and wheel_axle.articulation_type == ArticulationType.CONTINUOUS,
        details=(
            f"insert={insert_slide.articulation_type}, "
            f"lid={lid_hinge.articulation_type}, wheel={wheel_axle.articulation_type}"
        ),
    )
    ctx.check(
        "motion axes match lighter mechanisms",
        insert_slide.axis == (0.0, 0.0, 1.0)
        and lid_hinge.axis == (0.0, 0.0, 1.0)
        and wheel_axle.axis == (0.0, 1.0, 0.0),
        details=f"axes: slide={insert_slide.axis}, hinge={lid_hinge.axis}, wheel={wheel_axle.axis}",
    )

    ctx.expect_gap(
        lid,
        lower_shell,
        axis="z",
        max_gap=0.0008,
        max_penetration=0.0,
        positive_elem="lid_shell",
        negative_elem="lower_shell_body",
        name="closed lid seats on lower shell seam",
    )
    ctx.expect_within(
        insert,
        lower_shell,
        axes="xy",
        inner_elem="insert_reservoir",
        outer_elem="lower_shell_body",
        margin=0.0,
        name="insert reservoir fits inside lower case footprint",
    )
    ctx.expect_overlap(
        striker_wheel,
        insert,
        axes="xyz",
        elem_a="axle_pin",
        elem_b="front_chimney_plate",
        min_overlap=0.00025,
        name="front bearing plate captures striker axle",
    )
    ctx.expect_overlap(
        striker_wheel,
        insert,
        axes="xyz",
        elem_a="axle_pin",
        elem_b="rear_chimney_plate",
        min_overlap=0.00025,
        name="rear bearing plate captures striker axle",
    )
    ctx.expect_overlap(
        lower_shell,
        insert,
        axes="xyz",
        elem_a="front_insert_spring",
        elem_b="insert_reservoir",
        min_overlap=0.00008,
        name="front spring leaf grips insert reservoir",
    )
    ctx.expect_overlap(
        lower_shell,
        insert,
        axes="xyz",
        elem_a="rear_insert_spring",
        elem_b="insert_reservoir",
        min_overlap=0.00008,
        name="rear spring leaf grips insert reservoir",
    )

    closed_lid_aabb = ctx.part_world_aabb(lid)
    with ctx.pose({lid_hinge: 1.95}):
        opened_lid_aabb = ctx.part_world_aabb(lid)
    if closed_lid_aabb is not None and opened_lid_aabb is not None:
        closed_y = (closed_lid_aabb[0][1] + closed_lid_aabb[1][1]) / 2.0
        opened_y = (opened_lid_aabb[0][1] + opened_lid_aabb[1][1]) / 2.0
    else:
        closed_y = opened_y = None
    ctx.check(
        "side hinge swings lid away from case",
        closed_y is not None and opened_y is not None and opened_y > closed_y + 0.008,
        details=f"closed_y={closed_y}, opened_y={opened_y}",
    )

    rest_insert_pos = ctx.part_world_position(insert)
    with ctx.pose({insert_slide: 0.030}):
        ctx.expect_overlap(
            insert,
            lower_shell,
            axes="z",
            elem_a="insert_reservoir",
            elem_b="lower_shell_body",
            min_overlap=0.012,
            name="lifted insert remains retained in lower shell",
        )
        lifted_insert_pos = ctx.part_world_position(insert)
    ctx.check(
        "insert slides upward along case height",
        rest_insert_pos is not None
        and lifted_insert_pos is not None
        and lifted_insert_pos[2] > rest_insert_pos[2] + 0.025,
        details=f"rest={rest_insert_pos}, lifted={lifted_insert_pos}",
    )

    return ctx.report()


object_model = build_object_model()
