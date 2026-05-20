from __future__ import annotations

import math

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
import cadquery as cq


FRONT_X = 1.45


def _hollow_canister_block():
    """Two joined square launch tubes, open through the front and rear."""

    body_length = 1.25
    body_width = 0.86
    body_height = 0.46
    tube_width = 0.33
    tube_height = 0.31
    tube_offset_y = 0.215
    body_center_x = 0.5 * (0.20 + FRONT_X)

    block = (
        cq.Workplane("XY")
        .box(body_length, body_width, body_height)
        .translate((body_center_x, 0.0, 0.0))
    )
    for y in (-tube_offset_y, tube_offset_y):
        cutter = (
            cq.Workplane("XY")
            .box(body_length + 0.16, tube_width, tube_height)
            .translate((body_center_x, y, 0.0))
        )
        block = block.cut(cutter)
    return block


def _add_blast_cover_visuals(part, *, side_sign: float, armor, dark):
    """Build one hinged armored door in a frame whose origin is the hinge line."""

    panel_y = -side_sign * 0.180
    part.visual(
        Box((0.045, 0.335, 0.370)),
        origin=Origin(xyz=(0.017, panel_y, 0.0)),
        material=armor,
        name="cover_plate",
    )
    part.visual(
        Cylinder(radius=0.018, length=0.430),
        origin=Origin(xyz=(0.0, 0.0, 0.0)),
        material=dark,
        name="hinge_barrel",
    )
    part.visual(
        Box((0.018, 0.290, 0.028)),
        origin=Origin(xyz=(0.043, panel_y, 0.125)),
        material=dark,
        name="upper_rib",
    )
    part.visual(
        Box((0.018, 0.290, 0.028)),
        origin=Origin(xyz=(0.043, panel_y, -0.125)),
        material=dark,
        name="lower_rib",
    )
    part.visual(
        Box((0.018, 0.030, 0.320)),
        origin=Origin(xyz=(0.043, panel_y - side_sign * 0.140, 0.0)),
        material=dark,
        name="latch_rib",
    )


def build_object_model() -> ArticulatedObject:
    model = ArticulatedObject(name="naval_box_missile_launcher")

    naval_grey = model.material("naval_grey", rgba=(0.37, 0.42, 0.42, 1.0))
    dark_grey = model.material("dark_grey", rgba=(0.06, 0.07, 0.07, 1.0))
    olive = model.material("canister_olive", rgba=(0.26, 0.32, 0.25, 1.0))
    armor = model.material("blast_cover_armor", rgba=(0.46, 0.50, 0.48, 1.0))
    warning = model.material("safety_yellow", rgba=(0.85, 0.68, 0.12, 1.0))

    base = model.part("base")
    base.visual(
        Box((1.25, 1.10, 0.080)),
        origin=Origin(xyz=(0.0, 0.0, 0.040)),
        material=naval_grey,
        name="deck_plate",
    )
    base.visual(
        Cylinder(radius=0.390, length=0.080),
        origin=Origin(xyz=(0.0, 0.0, 0.120)),
        material=dark_grey,
        name="fixed_bearing_ring",
    )
    base.visual(
        Box((0.95, 0.055, 0.035)),
        origin=Origin(xyz=(0.0, -0.490, 0.096)),
        material=warning,
        name="front_safety_stripe",
    )

    pedestal = model.part("pedestal")
    pedestal.visual(
        Cylinder(radius=0.340, length=0.120),
        origin=Origin(xyz=(0.0, 0.0, 0.060)),
        material=naval_grey,
        name="rotating_turntable",
    )
    pedestal.visual(
        Cylinder(radius=0.165, length=0.720),
        origin=Origin(xyz=(0.0, 0.0, 0.480)),
        material=naval_grey,
        name="pedestal_column",
    )
    pedestal.visual(
        Box((0.230, 1.210, 0.140)),
        origin=Origin(xyz=(0.0, 0.0, 0.430)),
        material=naval_grey,
        name="yoke_crosshead",
    )
    for i, y in enumerate((-0.600, 0.600)):
        pedestal.visual(
            Box((0.150, 0.100, 0.560)),
            origin=Origin(xyz=(0.0, y, 0.720)),
            material=naval_grey,
            name=f"yoke_arm_{i}",
        )
        pedestal.visual(
            Cylinder(radius=0.170, length=0.095),
            origin=Origin(xyz=(0.0, y, 1.000), rpy=(math.pi / 2.0, 0.0, 0.0)),
            material=dark_grey,
            name=f"elevation_bearing_{i}",
        )

    canister_frame = model.part("canister_frame")
    canister_frame.visual(
        mesh_from_cadquery(_hollow_canister_block(), "dual_hollow_canisters"),
        origin=Origin(),
        material=olive,
        name="dual_canister_body",
    )
    for i, y in enumerate((-0.430, 0.430)):
        canister_frame.visual(
            Box((0.310, 0.080, 0.210)),
            origin=Origin(xyz=(0.105, y, 0.0)),
            material=olive,
            name=f"trunnion_cheek_{i}",
        )
        canister_frame.visual(
            Cylinder(radius=0.120, length=0.170),
            origin=Origin(xyz=(0.0, y + (0.035 if y > 0 else -0.035), 0.0), rpy=(math.pi / 2.0, 0.0, 0.0)),
            material=dark_grey,
            name=f"trunnion_journal_{i}",
        )
    for hinge_name, pad_name, y in (
        ("frame_hinge_strip_0", "hinge_mount_pad_0", -0.395),
        ("frame_hinge_strip_1", "hinge_mount_pad_1", 0.395),
    ):
        canister_frame.visual(
            Cylinder(radius=0.012, length=0.430),
            origin=Origin(xyz=(FRONT_X - 0.009, y, 0.0)),
            material=dark_grey,
            name=hinge_name,
        )
        canister_frame.visual(
            Box((0.040, 0.070, 0.390)),
            origin=Origin(xyz=(FRONT_X - 0.025, y, 0.0)),
            material=olive,
            name=pad_name,
        )

    blast_cover_0 = model.part("blast_cover_0")
    _add_blast_cover_visuals(blast_cover_0, side_sign=-1.0, armor=armor, dark=dark_grey)
    blast_cover_1 = model.part("blast_cover_1")
    _add_blast_cover_visuals(blast_cover_1, side_sign=1.0, armor=armor, dark=dark_grey)

    model.articulation(
        "pedestal_yaw",
        ArticulationType.REVOLUTE,
        parent=base,
        child=pedestal,
        origin=Origin(xyz=(0.0, 0.0, 0.160)),
        axis=(0.0, 0.0, 1.0),
        motion_limits=MotionLimits(effort=12000.0, velocity=0.45, lower=-math.pi, upper=math.pi),
    )
    model.articulation(
        "head_elevation",
        ArticulationType.REVOLUTE,
        parent=pedestal,
        child=canister_frame,
        origin=Origin(xyz=(0.0, 0.0, 1.000)),
        axis=(0.0, -1.0, 0.0),
        motion_limits=MotionLimits(effort=9000.0, velocity=0.30, lower=-0.10, upper=1.05),
    )
    model.articulation(
        "cover_hinge_0",
        ArticulationType.REVOLUTE,
        parent=canister_frame,
        child=blast_cover_0,
        origin=Origin(xyz=(FRONT_X + 0.021, -0.395, 0.0)),
        axis=(0.0, 0.0, -1.0),
        motion_limits=MotionLimits(effort=180.0, velocity=1.2, lower=0.0, upper=1.75),
    )
    model.articulation(
        "cover_hinge_1",
        ArticulationType.REVOLUTE,
        parent=canister_frame,
        child=blast_cover_1,
        origin=Origin(xyz=(FRONT_X + 0.021, 0.395, 0.0)),
        axis=(0.0, 0.0, 1.0),
        motion_limits=MotionLimits(effort=180.0, velocity=1.2, lower=0.0, upper=1.75),
    )

    return model


def run_tests() -> TestReport:
    ctx = TestContext(object_model)

    canister_frame = object_model.get_part("canister_frame")
    cover_0 = object_model.get_part("blast_cover_0")
    cover_1 = object_model.get_part("blast_cover_1")
    yaw = object_model.get_articulation("pedestal_yaw")
    elevation = object_model.get_articulation("head_elevation")
    hinge_0 = object_model.get_articulation("cover_hinge_0")
    hinge_1 = object_model.get_articulation("cover_hinge_1")

    ctx.check(
        "covers are mounted to the canister frame",
        hinge_0.parent == "canister_frame"
        and hinge_1.parent == "canister_frame"
        and hinge_0.child == "blast_cover_0"
        and hinge_1.child == "blast_cover_1",
        details=f"cover parents: {hinge_0.parent}, {hinge_1.parent}",
    )
    ctx.check(
        "primary launcher joints are revolute",
        yaw.articulation_type == ArticulationType.REVOLUTE
        and elevation.articulation_type == ArticulationType.REVOLUTE
        and hinge_0.articulation_type == ArticulationType.REVOLUTE
        and hinge_1.articulation_type == ArticulationType.REVOLUTE,
        details="pedestal, elevation, and both blast covers must rotate",
    )
    ctx.check(
        "pedestal yaw is vertical",
        yaw.axis == (0.0, 0.0, 1.0),
        details=f"axis={yaw.axis}",
    )
    ctx.check(
        "canister frame elevates about a horizontal trunnion",
        elevation.axis == (0.0, -1.0, 0.0),
        details=f"axis={elevation.axis}",
    )

    with ctx.pose({hinge_0: 0.0, hinge_1: 0.0, elevation: 0.0, yaw: 0.0}):
        ctx.expect_gap(
            cover_0,
            canister_frame,
            axis="x",
            max_gap=0.025,
            max_penetration=0.0,
            positive_elem="cover_plate",
            negative_elem="dual_canister_body",
            name="closed cover 0 sits just in front of the canisters",
        )
        ctx.expect_gap(
            cover_1,
            canister_frame,
            axis="x",
            max_gap=0.025,
            max_penetration=0.0,
            positive_elem="cover_plate",
            negative_elem="dual_canister_body",
            name="closed cover 1 sits just in front of the canisters",
        )
        ctx.expect_overlap(
            cover_0,
            canister_frame,
            axes="yz",
            min_overlap=0.30,
            elem_a="cover_plate",
            elem_b="dual_canister_body",
            name="cover 0 spans its canister mouth",
        )
        ctx.expect_overlap(
            cover_1,
            canister_frame,
            axes="yz",
            min_overlap=0.30,
            elem_a="cover_plate",
            elem_b="dual_canister_body",
            name="cover 1 spans its canister mouth",
        )
        ctx.expect_contact(
            cover_0,
            canister_frame,
            contact_tol=0.003,
            elem_a="hinge_barrel",
            elem_b="frame_hinge_strip_0",
            name="cover 0 hinge barrel seats on the frame hinge strip",
        )
        ctx.expect_contact(
            cover_1,
            canister_frame,
            contact_tol=0.003,
            elem_a="hinge_barrel",
            elem_b="frame_hinge_strip_1",
            name="cover 1 hinge barrel seats on the frame hinge strip",
        )

        rest_head_aabb = ctx.part_world_aabb(canister_frame)
        rest_cover_0_aabb = ctx.part_world_aabb(cover_0)
        rest_cover_1_aabb = ctx.part_world_aabb(cover_1)

    with ctx.pose({elevation: 0.85}):
        raised_head_aabb = ctx.part_world_aabb(canister_frame)
    ctx.check(
        "positive elevation raises the launcher head",
        rest_head_aabb is not None
        and raised_head_aabb is not None
        and raised_head_aabb[1][2] > rest_head_aabb[1][2] + 0.45,
        details=f"rest={rest_head_aabb}, raised={raised_head_aabb}",
    )

    with ctx.pose({hinge_0: 1.2, hinge_1: 1.2}):
        opened_cover_0_aabb = ctx.part_world_aabb(cover_0)
        opened_cover_1_aabb = ctx.part_world_aabb(cover_1)
    ctx.check(
        "positive cover rotation swings both blast covers forward",
        rest_cover_0_aabb is not None
        and rest_cover_1_aabb is not None
        and opened_cover_0_aabb is not None
        and opened_cover_1_aabb is not None
        and opened_cover_0_aabb[1][0] > rest_cover_0_aabb[1][0] + 0.15
        and opened_cover_1_aabb[1][0] > rest_cover_1_aabb[1][0] + 0.15,
        details=f"rest0={rest_cover_0_aabb}, open0={opened_cover_0_aabb}, rest1={rest_cover_1_aabb}, open1={opened_cover_1_aabb}",
    )

    return ctx.report()


object_model = build_object_model()
