from __future__ import annotations

import math

import cadquery as cq

from sdk import (
    ArticulatedObject,
    ArticulationType,
    Box,
    Cylinder,
    Inertial,
    MotionLimits,
    Origin,
    TestContext,
    TestReport,
    mesh_from_cadquery,
)


BODY_DEPTH = 0.255
BODY_WIDTH = 0.285
BASE_HEIGHT = 0.072

LID_DEPTH = 0.240
LID_WIDTH = 0.278
LID_BODY_HEIGHT = 0.044
LID_CROWN_HEIGHT = 0.014
LID_BOTTOM_Z = -0.009
LID_REAR_OFFSET = 0.008

HINGE_X = -0.1245
HINGE_Z = 0.081

LATCH_X = (BODY_DEPTH * 0.5) + 0.001
LATCH_Z = 0.054

CLIP_X = 0.108
CLIP_Y = (BODY_WIDTH * 0.5) + 0.009
CLIP_Z = 0.047


def _base_shell_mesh() -> object:
    outer = (
        cq.Workplane("XY")
        .box(BODY_DEPTH, BODY_WIDTH, BASE_HEIGHT, centered=(True, True, False))
        .edges("|Z")
        .fillet(0.022)
        .edges(">Z")
        .fillet(0.010)
    )
    cavity = (
        cq.Workplane("XY")
        .workplane(offset=0.012)
        .box(BODY_DEPTH - 0.040, BODY_WIDTH - 0.042, BASE_HEIGHT, centered=(True, True, False))
    )
    shell = outer.cut(cavity)
    front_bevel = (
        cq.Workplane("XY")
        .workplane(offset=0.010)
        .center(0.060, 0.0)
        .box(0.120, BODY_WIDTH - 0.055, 0.008, centered=(True, True, False))
        .edges(">Z")
        .fillet(0.004)
    )
    shell = shell.union(front_bevel)
    return mesh_from_cadquery(shell, "press_base_shell")


def _lid_shell_mesh() -> object:
    center_x = LID_REAR_OFFSET + (LID_DEPTH * 0.5)
    outer = (
        cq.Workplane("XY")
        .workplane(offset=LID_BOTTOM_Z)
        .center(center_x, 0.0)
        .box(LID_DEPTH, LID_WIDTH, LID_BODY_HEIGHT, centered=(True, True, False))
        .edges("|Z")
        .fillet(0.020)
        .edges(">Z")
        .fillet(0.014)
    )
    crown = (
        cq.Workplane("XY")
        .workplane(offset=LID_BOTTOM_Z + LID_BODY_HEIGHT - 0.006)
        .center(center_x - 0.004, 0.0)
        .box(LID_DEPTH - 0.036, LID_WIDTH - 0.048, LID_CROWN_HEIGHT, centered=(True, True, False))
        .edges("|Z")
        .fillet(0.010)
        .edges(">Z")
        .fillet(0.010)
    )
    pocket = (
        cq.Workplane("XY")
        .workplane(offset=LID_BOTTOM_Z)
        .center(center_x, 0.0)
        .box(LID_DEPTH - 0.032, LID_WIDTH - 0.030, 0.008, centered=(True, True, False))
    )
    shell = outer.union(crown).cut(pocket)
    return mesh_from_cadquery(shell, "press_lid_shell")


def build_object_model() -> ArticulatedObject:
    model = ArticulatedObject(name="clamshell_press")

    cast_black = model.material("cast_black", rgba=(0.12, 0.12, 0.13, 1.0))
    cast_satin = model.material("cast_satin", rgba=(0.16, 0.16, 0.17, 1.0))
    plate_dark = model.material("plate_dark", rgba=(0.25, 0.27, 0.29, 1.0))
    steel = model.material("steel", rgba=(0.67, 0.69, 0.71, 1.0))
    rubber = model.material("rubber", rgba=(0.09, 0.09, 0.10, 1.0))

    base = model.part("base")
    base.visual(_base_shell_mesh(), material=cast_black, name="base_shell")
    base.visual(
        Box((0.205, 0.235, 0.008)),
        origin=Origin(xyz=(0.0, 0.0, 0.016)),
        material=plate_dark,
        name="base_plate",
    )
    base.visual(
        Cylinder(radius=0.008, length=0.030),
        origin=Origin(xyz=(HINGE_X, -0.082, HINGE_Z), rpy=(math.pi / 2.0, 0.0, 0.0)),
        material=cast_satin,
        name="hinge_ear_0",
    )
    base.visual(
        Cylinder(radius=0.008, length=0.030),
        origin=Origin(xyz=(HINGE_X, 0.082, HINGE_Z), rpy=(math.pi / 2.0, 0.0, 0.0)),
        material=cast_satin,
        name="hinge_ear_1",
    )
    base.visual(
        Box((0.012, 0.030, 0.016)),
        origin=Origin(xyz=(HINGE_X + 0.002, -0.082, HINGE_Z - 0.010)),
        material=cast_satin,
        name="hinge_web_0",
    )
    base.visual(
        Box((0.012, 0.030, 0.016)),
        origin=Origin(xyz=(HINGE_X + 0.002, 0.082, HINGE_Z - 0.010)),
        material=cast_satin,
        name="hinge_web_1",
    )
    base.visual(
        Box((0.010, 0.010, 0.014)),
        origin=Origin(xyz=(LATCH_X - 0.005, -0.011, LATCH_Z)),
        material=cast_satin,
        name="latch_mount_0",
    )
    base.visual(
        Box((0.010, 0.010, 0.014)),
        origin=Origin(xyz=(LATCH_X - 0.005, 0.011, LATCH_Z)),
        material=cast_satin,
        name="latch_mount_1",
    )
    base.visual(
        Box((0.016, 0.010, 0.018)),
        origin=Origin(xyz=(CLIP_X, (BODY_WIDTH * 0.5) + 0.004, CLIP_Z)),
        material=cast_satin,
        name="clip_mount",
    )
    base.visual(
        Box((0.030, 0.018, 0.006)),
        origin=Origin(xyz=(-0.080, -0.100, 0.003)),
        material=rubber,
        name="foot_0",
    )
    base.visual(
        Box((0.030, 0.018, 0.006)),
        origin=Origin(xyz=(-0.080, 0.100, 0.003)),
        material=rubber,
        name="foot_1",
    )
    base.visual(
        Box((0.030, 0.018, 0.006)),
        origin=Origin(xyz=(0.080, -0.100, 0.003)),
        material=rubber,
        name="foot_2",
    )
    base.visual(
        Box((0.030, 0.018, 0.006)),
        origin=Origin(xyz=(0.080, 0.100, 0.003)),
        material=rubber,
        name="foot_3",
    )
    base.inertial = Inertial.from_geometry(
        Box((BODY_DEPTH, BODY_WIDTH, BASE_HEIGHT)),
        mass=2.9,
        origin=Origin(xyz=(0.0, 0.0, BASE_HEIGHT * 0.5)),
    )

    lid = model.part("lid")
    lid.visual(_lid_shell_mesh(), material=cast_black, name="lid_shell")
    lid.visual(
        Box((0.206, 0.232, 0.006)),
        origin=Origin(
            xyz=(
                LID_REAR_OFFSET + (LID_DEPTH * 0.5),
                0.0,
                -0.0005,
            )
        ),
        material=plate_dark,
        name="lid_plate",
    )
    lid.visual(
        Box((0.014, 0.060, 0.008)),
        origin=Origin(
            xyz=(
                LID_REAR_OFFSET + LID_DEPTH - 0.015,
                0.0,
                -0.004,
            )
        ),
        material=cast_satin,
        name="latch_catch",
    )
    lid.visual(
        Cylinder(radius=0.008, length=0.110),
        origin=Origin(xyz=(0.0, 0.0, 0.0), rpy=(math.pi / 2.0, 0.0, 0.0)),
        material=cast_satin,
        name="hinge_barrel",
    )
    lid.inertial = Inertial.from_geometry(
        Box((0.248, LID_WIDTH, 0.054)),
        mass=1.8,
        origin=Origin(xyz=(0.128, 0.0, 0.017)),
    )

    latch = model.part("latch_strap")
    latch.visual(
        Cylinder(radius=0.0035, length=0.012),
        origin=Origin(rpy=(math.pi / 2.0, 0.0, 0.0)),
        material=steel,
        name="strap_barrel",
    )
    latch.visual(
        Box((0.005, 0.012, 0.024)),
        origin=Origin(xyz=(0.000, 0.000, 0.012)),
        material=steel,
        name="strap_arm",
    )
    latch.visual(
        Box((0.012, 0.012, 0.005)),
        origin=Origin(xyz=(0.005, 0.000, 0.024)),
        material=steel,
        name="strap_bridge",
    )
    latch.visual(
        Box((0.005, 0.012, 0.015)),
        origin=Origin(xyz=(0.009, 0.000, 0.016)),
        material=steel,
        name="strap_hook",
    )
    latch.inertial = Inertial.from_geometry(
        Box((0.020, 0.012, 0.030)),
        mass=0.08,
        origin=Origin(xyz=(0.006, 0.0, 0.015)),
    )

    clip = model.part("side_clip")
    clip.visual(
        Box((0.012, 0.006, 0.020)),
        origin=Origin(xyz=(0.004, 0.003, 0.010)),
        material=steel,
        name="clip_body",
    )
    clip.visual(
        Box((0.018, 0.006, 0.005)),
        origin=Origin(xyz=(0.013, 0.003, 0.020)),
        material=steel,
        name="clip_tab",
    )
    clip.inertial = Inertial.from_geometry(
        Box((0.020, 0.006, 0.024)),
        mass=0.03,
        origin=Origin(xyz=(0.008, 0.003, 0.012)),
    )

    model.articulation(
        "base_to_lid",
        ArticulationType.REVOLUTE,
        parent=base,
        child=lid,
        origin=Origin(xyz=(HINGE_X, 0.0, HINGE_Z)),
        axis=(0.0, -1.0, 0.0),
        motion_limits=MotionLimits(effort=18.0, velocity=2.0, lower=0.0, upper=1.25),
    )
    model.articulation(
        "base_to_latch",
        ArticulationType.REVOLUTE,
        parent=base,
        child=latch,
        origin=Origin(xyz=(LATCH_X, 0.0, LATCH_Z)),
        axis=(0.0, 1.0, 0.0),
        motion_limits=MotionLimits(effort=2.0, velocity=3.0, lower=0.0, upper=1.10),
    )
    model.articulation(
        "base_to_clip",
        ArticulationType.REVOLUTE,
        parent=base,
        child=clip,
        origin=Origin(xyz=(CLIP_X, CLIP_Y, CLIP_Z)),
        axis=(0.0, 1.0, 0.0),
        motion_limits=MotionLimits(effort=1.0, velocity=3.0, lower=0.0, upper=0.95),
    )

    return model


def run_tests() -> TestReport:
    ctx = TestContext(object_model)

    base = object_model.get_part("base")
    lid = object_model.get_part("lid")
    latch = object_model.get_part("latch_strap")
    clip = object_model.get_part("side_clip")

    lid_hinge = object_model.get_articulation("base_to_lid")
    latch_hinge = object_model.get_articulation("base_to_latch")
    clip_hinge = object_model.get_articulation("base_to_clip")

    ctx.expect_gap(
        lid,
        base,
        axis="z",
        positive_elem="lid_shell",
        negative_elem="base_shell",
        max_gap=0.004,
        max_penetration=0.0,
        name="lid sits above the base with a tight seam",
    )
    ctx.expect_overlap(
        lid,
        base,
        axes="xy",
        elem_a="lid_shell",
        elem_b="base_shell",
        min_overlap=0.180,
        name="lid footprint covers the lower frame",
    )
    ctx.expect_gap(
        clip,
        base,
        axis="y",
        positive_elem="clip_body",
        negative_elem="clip_mount",
        max_gap=0.006,
        max_penetration=0.0,
        name="side clip stays mounted on the housing side",
    )

    closed_lid_aabb = ctx.part_element_world_aabb(lid, elem="lid_shell")
    closed_latch_aabb = ctx.part_world_aabb(latch)
    closed_clip_aabb = ctx.part_world_aabb(clip)

    with ctx.pose({latch_hinge: 1.05, lid_hinge: 1.10, clip_hinge: 0.80}):
        open_lid_aabb = ctx.part_element_world_aabb(lid, elem="lid_shell")
        open_latch_aabb = ctx.part_world_aabb(latch)
        open_clip_aabb = ctx.part_world_aabb(clip)

    ctx.check(
        "lid opens upward on the rear hinge",
        closed_lid_aabb is not None
        and open_lid_aabb is not None
        and open_lid_aabb[1][2] > closed_lid_aabb[1][2] + 0.055
        and open_lid_aabb[0][0] < closed_lid_aabb[0][0] - 0.020,
        details=f"closed={closed_lid_aabb}, open={open_lid_aabb}",
    )
    ctx.check(
        "latch strap swings clear when released",
        closed_latch_aabb is not None
        and open_latch_aabb is not None
        and open_latch_aabb[1][0] > closed_latch_aabb[1][0] + 0.010
        and open_latch_aabb[1][2] < closed_latch_aabb[1][2] - 0.010,
        details=f"closed={closed_latch_aabb}, open={open_latch_aabb}",
    )
    ctx.check(
        "side clip rotates away from the side seam",
        closed_clip_aabb is not None
        and open_clip_aabb is not None
        and open_clip_aabb[1][0] > closed_clip_aabb[1][0] + 0.008
        and open_clip_aabb[1][2] < closed_clip_aabb[1][2] - 0.006,
        details=f"closed={closed_clip_aabb}, open={open_clip_aabb}",
    )

    return ctx.report()


object_model = build_object_model()
