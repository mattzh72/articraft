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


def _add_square_tube_visuals(
    part,
    prefix: str,
    *,
    outer_width: float,
    inner_width: float,
    height: float,
    center_z: float,
    material,
):
    """Add four overlapping wall visuals for a clear, collision-safe square tube."""

    wall = (outer_width - inner_width) * 0.5
    offset = inner_width * 0.5 + wall * 0.5
    part.visual(
        Box((wall, outer_width, height)),
        origin=Origin(xyz=(offset, 0.0, center_z)),
        material=material,
        name=f"{prefix}_east_wall",
    )
    part.visual(
        Box((wall, outer_width, height)),
        origin=Origin(xyz=(-offset, 0.0, center_z)),
        material=material,
        name=f"{prefix}_west_wall",
    )
    part.visual(
        Box((outer_width, wall, height)),
        origin=Origin(xyz=(0.0, offset, center_z)),
        material=material,
        name=f"{prefix}_north_wall",
    )
    part.visual(
        Box((outer_width, wall, height)),
        origin=Origin(xyz=(0.0, -offset, center_z)),
        material=material,
        name=f"{prefix}_south_wall",
    )


def _add_glide_pads(
    part,
    prefix: str,
    *,
    child_width: float,
    parent_inner_width: float,
    center_z: float,
    pad_height: float,
    material,
):
    """Four bearing pads that close the telescoping clearance without overlap."""

    gap = (parent_inner_width - child_width) * 0.5
    pad_span = min(child_width * 0.45, 0.040)
    side_center = child_width * 0.5 + gap * 0.5
    part.visual(
        Box((gap, pad_span, pad_height)),
        origin=Origin(xyz=(side_center, 0.0, center_z)),
        material=material,
        name=f"{prefix}_east_pad",
    )
    part.visual(
        Box((gap, pad_span, pad_height)),
        origin=Origin(xyz=(-side_center, 0.0, center_z)),
        material=material,
        name=f"{prefix}_west_pad",
    )
    part.visual(
        Box((pad_span, gap, pad_height)),
        origin=Origin(xyz=(0.0, side_center, center_z)),
        material=material,
        name=f"{prefix}_north_pad",
    )
    part.visual(
        Box((pad_span, gap, pad_height)),
        origin=Origin(xyz=(0.0, -side_center, center_z)),
        material=material,
        name=f"{prefix}_south_pad",
    )


def build_object_model() -> ArticulatedObject:
    model = ArticulatedObject(name="nested_lift_mast_pan_face")

    painted = model.material("painted_black", rgba=(0.05, 0.055, 0.06, 1.0))
    dark = model.material("dark_stage", rgba=(0.11, 0.12, 0.13, 1.0))
    mid = model.material("telescoping_grey", rgba=(0.42, 0.44, 0.46, 1.0))
    light = model.material("brushed_aluminum", rgba=(0.72, 0.74, 0.73, 1.0))
    plate = model.material("faceplate_satin", rgba=(0.18, 0.20, 0.22, 1.0))
    bolt = model.material("bolt_heads", rgba=(0.02, 0.02, 0.022, 1.0))
    bearing = model.material("black_glide_pad", rgba=(0.015, 0.015, 0.014, 1.0))

    lower = model.part("lower_mast")
    lower.visual(
        Box((0.38, 0.38, 0.045)),
        origin=Origin(xyz=(0.0, 0.0, 0.0225)),
        material=painted,
        name="floor_plate",
    )
    lower.visual(
        Box((0.23, 0.23, 0.030)),
        origin=Origin(xyz=(0.0, 0.0, 0.060)),
        material=painted,
        name="raised_plinth",
    )
    _add_square_tube_visuals(
        lower,
        "lower_sleeve",
        outer_width=0.140,
        inner_width=0.112,
        height=0.760,
        center_z=0.445,
        material=dark,
    )
    _add_square_tube_visuals(
        lower,
        "guide_collar",
        outer_width=0.180,
        inner_width=0.114,
        height=0.046,
        center_z=0.835,
        material=painted,
    )
    for index, (x, y, yaw) in enumerate(
        (
            (0.098, 0.0, 0.0),
            (-0.098, 0.0, 0.0),
            (0.0, 0.098, math.pi / 2.0),
            (0.0, -0.098, math.pi / 2.0),
        )
    ):
        lower.visual(
            Box((0.030, 0.140, 0.140)),
            origin=Origin(xyz=(x, y, 0.130), rpy=(0.0, 0.0, yaw)),
            material=painted,
            name=f"base_web_{index}",
        )

    stage_1 = model.part("stage_1")
    _add_square_tube_visuals(
        stage_1,
        "stage_shell",
        outer_width=0.094,
        inner_width=0.072,
        height=0.850,
        center_z=-0.125,
        material=mid,
    )
    _add_glide_pads(
        stage_1,
        "lower_guide",
        child_width=0.094,
        parent_inner_width=0.112,
        center_z=-0.380,
        pad_height=0.080,
        material=bearing,
    )
    _add_square_tube_visuals(
        stage_1,
        "top_bushing",
        outer_width=0.112,
        inner_width=0.074,
        height=0.036,
        center_z=0.305,
        material=dark,
    )

    stage_2 = model.part("stage_2")
    _add_square_tube_visuals(
        stage_2,
        "stage_shell",
        outer_width=0.060,
        inner_width=0.044,
        height=0.700,
        center_z=-0.110,
        material=light,
    )
    _add_glide_pads(
        stage_2,
        "middle_guide",
        child_width=0.060,
        parent_inner_width=0.072,
        center_z=-0.310,
        pad_height=0.070,
        material=bearing,
    )
    _add_square_tube_visuals(
        stage_2,
        "top_bushing",
        outer_width=0.073,
        inner_width=0.046,
        height=0.032,
        center_z=0.256,
        material=mid,
    )

    stage_3 = model.part("stage_3")
    stage_3.visual(
        Box((0.036, 0.036, 0.500)),
        origin=Origin(xyz=(0.0, 0.0, -0.070)),
        material=light,
        name="inner_bar",
    )
    _add_glide_pads(
        stage_3,
        "inner_guide",
        child_width=0.036,
        parent_inner_width=0.044,
        center_z=-0.260,
        pad_height=0.060,
        material=bearing,
    )
    stage_3.visual(
        Cylinder(radius=0.032, length=0.030),
        origin=Origin(xyz=(0.0, 0.0, 0.190)),
        material=mid,
        name="pan_bearing",
    )

    faceplate = model.part("faceplate")
    faceplate.visual(
        Cylinder(radius=0.045, length=0.026),
        origin=Origin(xyz=(0.0, 0.0, 0.013)),
        material=plate,
        name="rotary_hub",
    )
    faceplate.visual(
        Box((0.048, 0.070, 0.045)),
        origin=Origin(xyz=(0.0, 0.034, 0.045)),
        material=plate,
        name="neck_block",
    )
    faceplate.visual(
        Box((0.150, 0.014, 0.110)),
        origin=Origin(xyz=(0.0, 0.073, 0.083)),
        material=plate,
        name="pan_face",
    )
    for index, (x, z) in enumerate(
        ((-0.050, 0.060), (0.050, 0.060), (-0.050, 0.110), (0.050, 0.110))
    ):
        faceplate.visual(
            Cylinder(radius=0.006, length=0.006),
            origin=Origin(xyz=(x, 0.064, z), rpy=(math.pi / 2.0, 0.0, 0.0)),
            material=bolt,
            name=f"face_bolt_{index}",
        )

    model.articulation(
        "lower_to_stage_1",
        ArticulationType.PRISMATIC,
        parent=lower,
        child=stage_1,
        origin=Origin(xyz=(0.0, 0.0, 0.800)),
        axis=(0.0, 0.0, 1.0),
        motion_limits=MotionLimits(effort=180.0, velocity=0.25, lower=0.0, upper=0.280),
    )
    model.articulation(
        "stage_1_to_stage_2",
        ArticulationType.PRISMATIC,
        parent=stage_1,
        child=stage_2,
        origin=Origin(xyz=(0.0, 0.0, 0.280)),
        axis=(0.0, 0.0, 1.0),
        motion_limits=MotionLimits(effort=140.0, velocity=0.22, lower=0.0, upper=0.230),
    )
    model.articulation(
        "stage_2_to_stage_3",
        ArticulationType.PRISMATIC,
        parent=stage_2,
        child=stage_3,
        origin=Origin(xyz=(0.0, 0.0, 0.220)),
        axis=(0.0, 0.0, 1.0),
        motion_limits=MotionLimits(effort=90.0, velocity=0.18, lower=0.0, upper=0.180),
    )
    model.articulation(
        "stage_3_to_faceplate",
        ArticulationType.REVOLUTE,
        parent=stage_3,
        child=faceplate,
        origin=Origin(xyz=(0.0, 0.0, 0.205)),
        axis=(0.0, 0.0, 1.0),
        motion_limits=MotionLimits(effort=18.0, velocity=1.5, lower=-math.pi, upper=math.pi),
    )
    return model


def run_tests() -> TestReport:
    ctx = TestContext(object_model)

    lower = object_model.get_part("lower_mast")
    stage_1 = object_model.get_part("stage_1")
    stage_2 = object_model.get_part("stage_2")
    stage_3 = object_model.get_part("stage_3")
    faceplate = object_model.get_part("faceplate")
    j1 = object_model.get_articulation("lower_to_stage_1")
    j2 = object_model.get_articulation("stage_1_to_stage_2")
    j3 = object_model.get_articulation("stage_2_to_stage_3")
    pan = object_model.get_articulation("stage_3_to_faceplate")

    ctx.check(
        "serial vertical lift joints",
        j1.axis == (0.0, 0.0, 1.0)
        and j2.axis == (0.0, 0.0, 1.0)
        and j3.axis == (0.0, 0.0, 1.0),
        details=f"axes: {j1.axis}, {j2.axis}, {j3.axis}",
    )
    ctx.check(
        "faceplate pans on mast axis",
        pan.axis == (0.0, 0.0, 1.0),
        details=f"axis: {pan.axis}",
    )

    ctx.expect_origin_distance(
        stage_1,
        lower,
        axes="xy",
        max_dist=0.001,
        name="first stage centered on mast axis",
    )
    ctx.expect_overlap(
        stage_1,
        lower,
        axes="z",
        min_overlap=0.40,
        name="first stage retained in lower sleeve",
    )
    ctx.expect_origin_distance(
        stage_2,
        stage_1,
        axes="xy",
        max_dist=0.001,
        name="second stage centered on mast axis",
    )
    ctx.expect_overlap(
        stage_2,
        stage_1,
        axes="z",
        min_overlap=0.30,
        name="second stage retained in first stage",
    )
    ctx.expect_origin_distance(
        stage_3,
        stage_2,
        axes="xy",
        max_dist=0.001,
        name="third stage centered on mast axis",
    )
    ctx.expect_overlap(
        stage_3,
        stage_2,
        axes="z",
        min_overlap=0.18,
        name="third stage retained in second stage",
    )

    rest_top = ctx.part_element_world_aabb(stage_3, elem="pan_bearing")
    rest_z = rest_top[1][2] if rest_top is not None else None
    with ctx.pose({j1: 0.280, j2: 0.230, j3: 0.180}):
        ctx.expect_overlap(
            stage_1,
            lower,
            axes="z",
            min_overlap=0.13,
            name="first stage retained when extended",
        )
        ctx.expect_overlap(
            stage_2,
            stage_1,
            axes="z",
            min_overlap=0.10,
            name="second stage retained when extended",
        )
        ctx.expect_overlap(
            stage_3,
            stage_2,
            axes="z",
            min_overlap=0.07,
            name="third stage retained when extended",
        )
        extended_top = ctx.part_element_world_aabb(stage_3, elem="pan_bearing")
        extended_z = extended_top[1][2] if extended_top is not None else None

    ctx.check(
        "mast extends upward",
        rest_z is not None and extended_z is not None and extended_z > rest_z + 0.60,
        details=f"rest_z={rest_z}, extended_z={extended_z}",
    )

    rest_face = ctx.part_element_world_aabb(faceplate, elem="pan_face")
    rest_center = (
        ((rest_face[0][0] + rest_face[1][0]) * 0.5, (rest_face[0][1] + rest_face[1][1]) * 0.5)
        if rest_face is not None
        else None
    )
    with ctx.pose({pan: math.pi / 2.0}):
        turned_face = ctx.part_element_world_aabb(faceplate, elem="pan_face")
        turned_center = (
            (
                (turned_face[0][0] + turned_face[1][0]) * 0.5,
                (turned_face[0][1] + turned_face[1][1]) * 0.5,
            )
            if turned_face is not None
            else None
        )
    ctx.check(
        "faceplate visibly rotates about vertical axis",
        rest_center is not None
        and turned_center is not None
        and turned_center[0] < rest_center[0] - 0.05
        and abs(turned_center[1]) < 0.02,
        details=f"rest_center={rest_center}, turned_center={turned_center}",
    )

    return ctx.report()


object_model = build_object_model()
