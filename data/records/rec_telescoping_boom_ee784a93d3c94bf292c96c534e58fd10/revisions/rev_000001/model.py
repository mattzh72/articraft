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


def _rectangular_tube(
    *,
    x_min: float,
    x_max: float,
    outer_y: float,
    outer_z: float,
    wall: float,
    collar_len: float = 0.0,
    collar_extra: float = 0.0,
    fillet: float = 0.0,
):
    """Hollow rectangular box-section with an optional reinforced front collar."""
    length = x_max - x_min
    center_x = (x_min + x_max) * 0.5
    inner_y = outer_y - 2.0 * wall
    inner_z = outer_z - 2.0 * wall

    outer = cq.Workplane("XY").box(length, outer_y, outer_z).translate((center_x, 0.0, 0.0))
    void = cq.Workplane("XY").box(length + 0.010, inner_y, inner_z).translate((center_x, 0.0, 0.0))
    tube = outer.cut(void)

    if collar_len > 0.0 and collar_extra > 0.0:
        collar_center = x_max - collar_len * 0.5
        collar_outer = (
            cq.Workplane("XY")
            .box(collar_len, outer_y + 2.0 * collar_extra, outer_z + 2.0 * collar_extra)
            .translate((collar_center, 0.0, 0.0))
        )
        collar_void = (
            cq.Workplane("XY")
            .box(collar_len + 0.010, inner_y, inner_z)
            .translate((collar_center, 0.0, 0.0))
        )
        tube = tube.union(collar_outer.cut(collar_void))

    if fillet > 0.0:
        tube = tube.edges("|X").fillet(fillet)
    return tube


def _head_bracket():
    """Compact clevis-style inspection/lifting head bracket."""
    back_plate = cq.Workplane("XY").box(0.035, 0.165, 0.125).translate((0.0175, 0.0, 0.0))
    lug_a = cq.Workplane("XY").box(0.160, 0.020, 0.105).translate((0.100, -0.070, 0.0))
    lug_b = cq.Workplane("XY").box(0.160, 0.020, 0.105).translate((0.100, 0.070, 0.0))
    nose_web = cq.Workplane("XY").box(0.030, 0.160, 0.035).translate((0.170, 0.0, -0.035))
    body = back_plate.union(lug_a).union(lug_b).union(nose_web)
    return body.edges("|X").fillet(0.004)


def build_object_model() -> ArticulatedObject:
    model = ArticulatedObject(name="rectangular_telescoping_boom")

    dark_steel = model.material("dark_powder_coat", rgba=(0.06, 0.07, 0.075, 1.0))
    black = model.material("black_hardware", rgba=(0.01, 0.01, 0.012, 1.0))
    aluminum = model.material("brushed_aluminum", rgba=(0.72, 0.75, 0.76, 1.0))
    yellow = model.material("safety_yellow", rgba=(1.0, 0.68, 0.06, 1.0))
    orange = model.material("inspection_orange", rgba=(0.95, 0.30, 0.05, 1.0))
    red = model.material("red_tip_marking", rgba=(0.85, 0.05, 0.03, 1.0))

    root = model.part("root_mount")
    root.visual(
        mesh_from_cadquery(
            _rectangular_tube(
                x_min=-1.15,
                x_max=0.0,
                outer_y=0.260,
                outer_z=0.190,
                wall=0.020,
                collar_len=0.095,
                collar_extra=0.014,
                fillet=0.006,
            ),
            "outer_sleeve",
            tolerance=0.0008,
        ),
        material=dark_steel,
        name="outer_sleeve",
    )
    root.visual(
        Box((0.080, 0.460, 0.360)),
        origin=Origin(xyz=(-1.190, 0.0, 0.0)),
        material=dark_steel,
        name="mount_plate",
    )
    root.visual(
        Box((0.620, 0.315, 0.035)),
        origin=Origin(xyz=(-0.835, 0.0, -0.1125)),
        material=dark_steel,
        name="bottom_saddle",
    )
    for y in (-0.157, 0.157):
        root.visual(
            Box((0.560, 0.020, 0.135)),
            origin=Origin(xyz=(-0.870, y, -0.028)),
            material=dark_steel,
            name=f"side_gusset_{0 if y < 0 else 1}",
        )
    bolt_positions = [(-0.160, -0.115), (0.160, -0.115), (-0.160, 0.115), (0.160, 0.115)]
    for i, (y, z) in enumerate(bolt_positions):
        root.visual(
            Cylinder(radius=0.025, length=0.026),
            origin=Origin(xyz=(-1.236, y, z), rpy=(0.0, math.pi / 2.0, 0.0)),
            material=black,
            name=f"bolt_head_{i}",
        )

    stage_1 = model.part("stage_1")
    stage_1.visual(
        mesh_from_cadquery(
            _rectangular_tube(
                x_min=-0.900,
                x_max=0.450,
                outer_y=0.205,
                outer_z=0.135,
                wall=0.015,
                collar_len=0.080,
                collar_extra=0.010,
                fillet=0.004,
            ),
            "stage_1_shell",
            tolerance=0.0008,
        ),
        material=aluminum,
        name="stage_shell",
    )
    stage_1.visual(
        Box((0.160, 0.212, 0.012)),
        origin=Origin(xyz=(0.300, 0.0, 0.0735)),
        material=yellow,
        name="top_warning_band",
    )
    stage_1.visual(
        Box((0.160, 0.055, 0.0075)),
        origin=Origin(xyz=(-0.650, 0.0, 0.07125)),
        material=black,
        name="upper_wear_pad",
    )
    stage_1.visual(
        Box((0.160, 0.055, 0.0075)),
        origin=Origin(xyz=(-0.650, 0.0, -0.07125)),
        material=black,
        name="lower_wear_pad",
    )

    stage_2 = model.part("stage_2")
    stage_2.visual(
        mesh_from_cadquery(
            _rectangular_tube(
                x_min=-0.700,
                x_max=0.350,
                outer_y=0.158,
                outer_z=0.092,
                wall=0.012,
                collar_len=0.070,
                collar_extra=0.008,
                fillet=0.003,
            ),
            "stage_2_shell",
            tolerance=0.0008,
        ),
        material=yellow,
        name="stage_shell",
    )
    stage_2.visual(
        Box((0.135, 0.164, 0.010)),
        origin=Origin(xyz=(0.245, 0.0, 0.052)),
        material=orange,
        name="top_warning_band",
    )
    stage_2.visual(
        Box((0.130, 0.048, 0.0065)),
        origin=Origin(xyz=(-0.550, 0.0, 0.04925)),
        material=black,
        name="upper_wear_pad",
    )
    stage_2.visual(
        Box((0.130, 0.048, 0.0065)),
        origin=Origin(xyz=(-0.550, 0.0, -0.04925)),
        material=black,
        name="lower_wear_pad",
    )

    stage_3 = model.part("stage_3")
    stage_3.visual(
        mesh_from_cadquery(
            _rectangular_tube(
                x_min=-0.550,
                x_max=0.270,
                outer_y=0.116,
                outer_z=0.058,
                wall=0.009,
                collar_len=0.060,
                collar_extra=0.006,
                fillet=0.002,
            ),
            "stage_3_shell",
            tolerance=0.0008,
        ),
        material=orange,
        name="stage_shell",
    )
    stage_3.visual(
        Box((0.100, 0.122, 0.008)),
        origin=Origin(xyz=(0.180, 0.0, 0.033)),
        material=red,
        name="top_tip_band",
    )
    stage_3.visual(
        Box((0.110, 0.040, 0.005)),
        origin=Origin(xyz=(-0.420, 0.0, 0.0315)),
        material=black,
        name="upper_wear_pad",
    )
    stage_3.visual(
        Box((0.110, 0.040, 0.005)),
        origin=Origin(xyz=(-0.420, 0.0, -0.0315)),
        material=black,
        name="lower_wear_pad",
    )

    head = model.part("head_bracket")
    head.visual(
        mesh_from_cadquery(_head_bracket(), "head_bracket", tolerance=0.0008),
        material=dark_steel,
        name="bracket_body",
    )
    head.visual(
        Cylinder(radius=0.014, length=0.175),
        origin=Origin(xyz=(0.105, 0.0, 0.006), rpy=(math.pi / 2.0, 0.0, 0.0)),
        material=black,
        name="cross_pin",
    )

    model.articulation(
        "root_to_stage_1",
        ArticulationType.PRISMATIC,
        parent=root,
        child=stage_1,
        origin=Origin(xyz=(0.0, 0.0, 0.0)),
        axis=(1.0, 0.0, 0.0),
        motion_limits=MotionLimits(effort=400.0, velocity=0.20, lower=0.0, upper=0.55),
    )
    model.articulation(
        "stage_1_to_stage_2",
        ArticulationType.PRISMATIC,
        parent=stage_1,
        child=stage_2,
        origin=Origin(xyz=(0.450, 0.0, 0.0)),
        axis=(1.0, 0.0, 0.0),
        motion_limits=MotionLimits(effort=280.0, velocity=0.18, lower=0.0, upper=0.45),
    )
    model.articulation(
        "stage_2_to_stage_3",
        ArticulationType.PRISMATIC,
        parent=stage_2,
        child=stage_3,
        origin=Origin(xyz=(0.350, 0.0, 0.0)),
        axis=(1.0, 0.0, 0.0),
        motion_limits=MotionLimits(effort=180.0, velocity=0.16, lower=0.0, upper=0.30),
    )
    model.articulation(
        "stage_3_to_head",
        ArticulationType.FIXED,
        parent=stage_3,
        child=head,
        origin=Origin(xyz=(0.270, 0.0, 0.0)),
    )

    return model


def run_tests() -> TestReport:
    ctx = TestContext(object_model)
    root = object_model.get_part("root_mount")
    stage_1 = object_model.get_part("stage_1")
    stage_2 = object_model.get_part("stage_2")
    stage_3 = object_model.get_part("stage_3")
    head = object_model.get_part("head_bracket")

    j1 = object_model.get_articulation("root_to_stage_1")
    j2 = object_model.get_articulation("stage_1_to_stage_2")
    j3 = object_model.get_articulation("stage_2_to_stage_3")

    ctx.allow_overlap(
        root,
        stage_1,
        elem_a="outer_sleeve",
        elem_b="upper_wear_pad",
        reason=(
            "The local low-friction wear pad is intentionally seated against the "
            "inside of the root sleeve proxy so the prismatic box section is guided."
        ),
    )
    ctx.expect_contact(
        root,
        stage_1,
        elem_a="outer_sleeve",
        elem_b="upper_wear_pad",
        contact_tol=0.001,
        name="stage 1 upper wear pad bears on root sleeve",
    )
    ctx.allow_overlap(
        root,
        stage_1,
        elem_a="outer_sleeve",
        elem_b="lower_wear_pad",
        reason=(
            "The opposing local wear pad is intentionally seated against the "
            "inside of the root sleeve proxy to remove vertical play."
        ),
    )
    ctx.expect_contact(
        root,
        stage_1,
        elem_a="outer_sleeve",
        elem_b="lower_wear_pad",
        contact_tol=0.001,
        name="stage 1 lower wear pad bears on root sleeve",
    )
    ctx.allow_overlap(
        stage_1,
        stage_2,
        elem_a="stage_shell",
        elem_b="upper_wear_pad",
        reason=(
            "The local low-friction wear pad is intentionally seated against the "
            "inside of the stage 1 sleeve proxy to carry the sliding stage."
        ),
    )
    ctx.expect_contact(
        stage_1,
        stage_2,
        elem_a="stage_shell",
        elem_b="upper_wear_pad",
        contact_tol=0.001,
        name="stage 2 upper wear pad bears on stage 1 sleeve",
    )
    ctx.allow_overlap(
        stage_1,
        stage_2,
        elem_a="stage_shell",
        elem_b="lower_wear_pad",
        reason=(
            "The opposing local wear pad is intentionally seated against the "
            "inside of the stage 1 sleeve proxy to remove vertical play."
        ),
    )
    ctx.expect_contact(
        stage_1,
        stage_2,
        elem_a="stage_shell",
        elem_b="lower_wear_pad",
        contact_tol=0.001,
        name="stage 2 lower wear pad bears on stage 1 sleeve",
    )
    ctx.allow_overlap(
        stage_2,
        stage_3,
        elem_a="stage_shell",
        elem_b="upper_wear_pad",
        reason=(
            "The local low-friction wear pad is intentionally seated against the "
            "inside of the stage 2 sleeve proxy to carry the terminal stage."
        ),
    )
    ctx.expect_contact(
        stage_2,
        stage_3,
        elem_a="stage_shell",
        elem_b="upper_wear_pad",
        contact_tol=0.001,
        name="stage 3 upper wear pad bears on stage 2 sleeve",
    )
    ctx.allow_overlap(
        stage_2,
        stage_3,
        elem_a="stage_shell",
        elem_b="lower_wear_pad",
        reason=(
            "The opposing local wear pad is intentionally seated against the "
            "inside of the stage 2 sleeve proxy to remove vertical play."
        ),
    )
    ctx.expect_contact(
        stage_2,
        stage_3,
        elem_a="stage_shell",
        elem_b="lower_wear_pad",
        contact_tol=0.001,
        name="stage 3 lower wear pad bears on stage 2 sleeve",
    )

    ctx.expect_within(
        stage_1,
        root,
        axes="yz",
        inner_elem="stage_shell",
        outer_elem="outer_sleeve",
        margin=0.0,
        name="stage 1 box section fits inside root sleeve cross-section",
    )
    ctx.expect_overlap(
        stage_1,
        root,
        axes="x",
        elem_a="stage_shell",
        elem_b="outer_sleeve",
        min_overlap=0.80,
        name="stage 1 is deeply inserted when retracted",
    )
    ctx.expect_within(
        stage_2,
        stage_1,
        axes="yz",
        inner_elem="stage_shell",
        outer_elem="stage_shell",
        margin=0.0,
        name="stage 2 box section nests inside stage 1",
    )
    ctx.expect_overlap(
        stage_2,
        stage_1,
        axes="x",
        elem_a="stage_shell",
        elem_b="stage_shell",
        min_overlap=0.60,
        name="stage 2 is retained when retracted",
    )
    ctx.expect_within(
        stage_3,
        stage_2,
        axes="yz",
        inner_elem="stage_shell",
        outer_elem="stage_shell",
        margin=0.0,
        name="stage 3 box section nests inside stage 2",
    )
    ctx.expect_overlap(
        stage_3,
        stage_2,
        axes="x",
        elem_a="stage_shell",
        elem_b="stage_shell",
        min_overlap=0.45,
        name="stage 3 is retained when retracted",
    )
    ctx.expect_contact(
        head,
        stage_3,
        contact_tol=0.003,
        elem_a="bracket_body",
        elem_b="stage_shell",
        name="head bracket seats against terminal box section",
    )

    rest_tip = ctx.part_world_position(head)
    with ctx.pose({j1: 0.55, j2: 0.45, j3: 0.30}):
        ctx.expect_overlap(
            stage_1,
            root,
            axes="x",
            elem_a="stage_shell",
            elem_b="outer_sleeve",
            min_overlap=0.30,
            name="stage 1 keeps retained insertion at full travel",
        )
        ctx.expect_overlap(
            stage_2,
            stage_1,
            axes="x",
            elem_a="stage_shell",
            elem_b="stage_shell",
            min_overlap=0.22,
            name="stage 2 keeps retained insertion at full travel",
        )
        ctx.expect_overlap(
            stage_3,
            stage_2,
            axes="x",
            elem_a="stage_shell",
            elem_b="stage_shell",
            min_overlap=0.20,
            name="stage 3 keeps retained insertion at full travel",
        )
        extended_tip = ctx.part_world_position(head)

    ctx.check(
        "tip bracket extends along boom axis",
        rest_tip is not None and extended_tip is not None and extended_tip[0] > rest_tip[0] + 1.20,
        details=f"rest_tip={rest_tip}, extended_tip={extended_tip}",
    )

    return ctx.report()


object_model = build_object_model()
