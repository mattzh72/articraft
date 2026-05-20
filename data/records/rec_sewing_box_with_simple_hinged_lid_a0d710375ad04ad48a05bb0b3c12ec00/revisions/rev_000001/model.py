from __future__ import annotations

import math

import cadquery as cq

from sdk import (
    ArticulatedObject,
    ArticulationType,
    Box,
    Cylinder,
    MotionLimits,
    Origin,
    TestContext,
    TestReport,
    mesh_from_cadquery,
)


BOX_W = 0.32
BOX_D = 0.20
BOX_H = 0.11
WALL_T = 0.012

LID_OVERHANG = 0.012
LID_W = BOX_W + 2.0 * LID_OVERHANG
LID_D = BOX_D + 2.0 * LID_OVERHANG
LID_T = 0.018

BAND_T = 0.018
BAND_H = 0.040
BAND_W = LID_W

HINGE_R = 0.0055
HINGE_Y = BOX_D / 2.0 + BAND_T + HINGE_R + 0.0005
HINGE_Z = BOX_H + LID_T / 2.0 + 0.001


def _open_shell_mesh():
    """Single-piece open rectangular shell: base plus four continuous walls."""
    outer = cq.Workplane("XY").rect(BOX_W, BOX_D).extrude(BOX_H)
    inner = (
        cq.Workplane("XY")
        .rect(BOX_W - 2.0 * WALL_T, BOX_D - 2.0 * WALL_T)
        .extrude(BOX_H + WALL_T)
        .translate((0.0, 0.0, WALL_T))
    )
    return outer.cut(inner)


def build_object_model() -> ArticulatedObject:
    model = ArticulatedObject(name="small_sewing_box")

    warm_wood = model.material("warm_wood", rgba=(0.66, 0.40, 0.20, 1.0))
    darker_wood = model.material("darker_wood", rgba=(0.48, 0.27, 0.13, 1.0))
    hinge_steel = model.material("brushed_steel", rgba=(0.72, 0.70, 0.66, 1.0))

    shell = model.part("main_shell")
    shell.visual(
        mesh_from_cadquery(_open_shell_mesh(), "open_box_shell", tolerance=0.0007),
        material=warm_wood,
        name="shell_body",
    )

    band_center = (0.0, BOX_D / 2.0 + BAND_T / 2.0, BOX_H - BAND_H / 2.0 - 0.006)
    band = model.part("hinge_band")
    band.visual(
        Box((BAND_W, BAND_T, BAND_H)),
        origin=Origin(),
        material=darker_wood,
        name="reinforcement_band",
    )
    # A simple fixed hinge leaf and barrel make the rear hinge axis legible while
    # the lid itself remains a plain single panel.
    band.visual(
        Box((BAND_W * 0.94, 0.002, 0.026)),
        origin=Origin(xyz=(0.0, BAND_T / 2.0 + 0.001, BAND_H / 2.0 + 0.006)),
        material=hinge_steel,
        name="hinge_leaf",
    )
    band.visual(
        Cylinder(radius=HINGE_R, length=BAND_W * 0.96),
        origin=Origin(
            xyz=(0.0, HINGE_Y - band_center[1], HINGE_Z - band_center[2]),
            rpy=(0.0, math.pi / 2.0, 0.0),
        ),
        material=hinge_steel,
        name="hinge_barrel",
    )
    model.articulation(
        "shell_to_band",
        ArticulationType.FIXED,
        parent=shell,
        child=band,
        origin=Origin(xyz=band_center),
    )

    lid = model.part("lid")
    lid.visual(
        Box((LID_W, LID_D, LID_T)),
        # The child frame is the rear hinge axis.  The plain panel sits just in
        # front of the barrel, spans toward the front of the box, and rests just
        # above the shell rim in the closed pose.
        origin=Origin(xyz=(0.0, -(LID_D / 2.0 + HINGE_R + 0.0005), 0.0)),
        material=warm_wood,
        name="lid_panel",
    )
    model.articulation(
        "band_to_lid",
        ArticulationType.REVOLUTE,
        parent=band,
        child=lid,
        origin=Origin(xyz=(0.0, HINGE_Y - band_center[1], HINGE_Z - band_center[2])),
        axis=(-1.0, 0.0, 0.0),
        motion_limits=MotionLimits(effort=3.0, velocity=2.0, lower=0.0, upper=1.75),
    )

    return model


def run_tests() -> TestReport:
    ctx = TestContext(object_model)

    shell = object_model.get_part("main_shell")
    band = object_model.get_part("hinge_band")
    lid = object_model.get_part("lid")
    hinge = object_model.get_articulation("band_to_lid")

    ctx.expect_gap(
        band,
        shell,
        axis="y",
        max_gap=0.001,
        max_penetration=0.0,
        positive_elem="reinforcement_band",
        negative_elem="shell_body",
        name="reinforcement band is seated on rear shell face",
    )
    ctx.expect_gap(
        lid,
        shell,
        axis="z",
        min_gap=0.0,
        max_gap=0.004,
        max_penetration=0.0,
        positive_elem="lid_panel",
        negative_elem="shell_body",
        name="closed lid rests just above the shell rim",
    )
    ctx.expect_overlap(
        lid,
        shell,
        axes="xy",
        min_overlap=0.18,
        elem_a="lid_panel",
        elem_b="shell_body",
        name="plain lid covers the rectangular box opening",
    )

    closed_aabb = ctx.part_world_aabb(lid)
    with ctx.pose({hinge: 1.45}):
        open_aabb = ctx.part_world_aabb(lid)
    ctx.check(
        "lid opens upward about rear horizontal hinge",
        closed_aabb is not None
        and open_aabb is not None
        and open_aabb[1][2] > closed_aabb[1][2] + 0.12,
        details=f"closed_aabb={closed_aabb}, open_aabb={open_aabb}",
    )

    return ctx.report()


object_model = build_object_model()
