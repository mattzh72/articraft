from __future__ import annotations

import math

from sdk import (
    ArticulatedObject,
    ArticulationType,
    Box,
    Cylinder,
    Mimic,
    MotionLimits,
    Origin,
    TestContext,
    TestReport,
)


def build_object_model() -> ArticulatedObject:
    model = ArticulatedObject(name="three_column_bench_standing_desk")

    warm_oak = model.material("warm_oak", rgba=(0.78, 0.55, 0.32, 1.0))
    dark_edge = model.material("dark_edge_band", rgba=(0.34, 0.23, 0.14, 1.0))
    black = model.material("satin_black_powdercoat", rgba=(0.015, 0.017, 0.018, 1.0))
    graphite = model.material("graphite_metal", rgba=(0.12, 0.13, 0.14, 1.0))
    rubber = model.material("matte_black_rubber", rgba=(0.004, 0.004, 0.004, 1.0))
    paddle_mat = model.material("soft_touch_paddles", rgba=(0.045, 0.050, 0.055, 1.0))
    white = model.material("white_legends", rgba=(0.92, 0.94, 0.95, 1.0))

    column_xs = (-0.90, 0.0, 0.90)
    sleeve_top_z = 0.72
    desktop_mount_z = 0.152

    base = model.part("base_frame")

    # Three foot assemblies tied together by low stretchers so the bench base
    # reads as one shared workstation undercarriage rather than three stools.
    for index, x in enumerate(column_xs):
        base.visual(
            Box((0.11, 0.68, 0.036)),
            origin=Origin(xyz=(x, 0.0, 0.018)),
            material=black,
            name=f"foot_{index}",
        )
        base.visual(
            Box((0.125, 0.050, 0.012)),
            origin=Origin(xyz=(x, -0.315, 0.042)),
            material=rubber,
            name=f"front_glide_{index}",
        )
        base.visual(
            Box((0.125, 0.050, 0.012)),
            origin=Origin(xyz=(x, 0.315, 0.042)),
            material=rubber,
            name=f"rear_glide_{index}",
        )

        # Rectangular outer lifting column modeled as four tube walls with a
        # real center clearance for the sliding inner stage.
        wall_h = sleeve_top_z - 0.034
        center_z = 0.034 + wall_h / 2.0
        base.visual(
            Box((0.104, 0.010, wall_h)),
            origin=Origin(xyz=(x, -0.047, center_z)),
            material=black,
            name=f"front_outer_wall_{index}",
        )
        base.visual(
            Box((0.104, 0.010, wall_h)),
            origin=Origin(xyz=(x, 0.047, center_z)),
            material=black,
            name=f"rear_outer_wall_{index}",
        )
        base.visual(
            Box((0.010, 0.104, wall_h)),
            origin=Origin(xyz=(x - 0.047, 0.0, center_z)),
            material=black,
            name=f"side_outer_wall_{index}_0",
        )
        base.visual(
            Box((0.010, 0.104, wall_h)),
            origin=Origin(xyz=(x + 0.047, 0.0, center_z)),
            material=black,
            name=f"side_outer_wall_{index}_1",
        )
        # Four separate collar lips leave the center open, like a real sleeve,
        # so the sliding inner tube is captured without a false solid overlap.
        base.visual(
            Box((0.120, 0.012, 0.020)),
            origin=Origin(xyz=(x, -0.054, sleeve_top_z + 0.010)),
            material=graphite,
            name=f"front_collar_lip_{index}",
        )
        base.visual(
            Box((0.120, 0.012, 0.020)),
            origin=Origin(xyz=(x, 0.054, sleeve_top_z + 0.010)),
            material=graphite,
            name=f"rear_collar_lip_{index}",
        )
        base.visual(
            Box((0.012, 0.120, 0.020)),
            origin=Origin(xyz=(x - 0.054, 0.0, sleeve_top_z + 0.010)),
            material=graphite,
            name=f"side_collar_lip_{index}_0",
        )
        base.visual(
            Box((0.012, 0.120, 0.020)),
            origin=Origin(xyz=(x + 0.054, 0.0, sleeve_top_z + 0.010)),
            material=graphite,
            name=f"side_collar_lip_{index}_1",
        )

    for y, name in ((-0.315, "front_floor_stretcher"), (0.315, "rear_floor_stretcher")):
        base.visual(
            Box((1.93, 0.044, 0.032)),
            origin=Origin(xyz=(0.0, y, 0.032)),
            material=black,
            name=name,
        )

    inner_stages = []
    for index in range(3):
        inner = model.part(f"inner_stage_{index}")
        inner.visual(
            Box((0.056, 0.056, 0.740)),
            origin=Origin(xyz=(0.0, 0.0, -0.250)),
            material=graphite,
            name="inner_tube",
        )
        inner.visual(
            Box((0.090, 0.090, 0.032)),
            origin=Origin(xyz=(0.0, 0.0, 0.136)),
            material=graphite,
            name="top_cap",
        )
        # Narrow bright witness line at the sleeve lip makes the telescoping
        # joint readable without touching the outer tube walls.
        inner.visual(
            Box((0.061, 0.061, 0.010)),
            origin=Origin(xyz=(0.0, 0.0, 0.006)),
            material=model.material("brushed_stage_line" if index == 0 else f"brushed_stage_line_{index}", rgba=(0.40, 0.42, 0.43, 1.0)),
            name="height_witness_band",
        )
        for z, level_name in ((-0.45, "lower"), (-0.05, "upper")):
            inner.visual(
                Box((0.035, 0.0145, 0.050)),
                origin=Origin(xyz=(0.0, -0.03475, z)),
                material=rubber,
                name=f"{level_name}_front_guide",
            )
            inner.visual(
                Box((0.035, 0.0145, 0.050)),
                origin=Origin(xyz=(0.0, 0.03475, z)),
                material=rubber,
                name=f"{level_name}_rear_guide",
            )
            inner.visual(
                Box((0.0145, 0.035, 0.050)),
                origin=Origin(xyz=(-0.03475, 0.0, z)),
                material=rubber,
                name=f"{level_name}_side_guide_0",
            )
            inner.visual(
                Box((0.0145, 0.035, 0.050)),
                origin=Origin(xyz=(0.03475, 0.0, z)),
                material=rubber,
                name=f"{level_name}_side_guide_1",
            )
        inner_stages.append(inner)

    center_lift = model.articulation(
        "center_lift",
        ArticulationType.PRISMATIC,
        parent=base,
        child=inner_stages[1],
        origin=Origin(xyz=(column_xs[1], 0.0, sleeve_top_z)),
        axis=(0.0, 0.0, 1.0),
        motion_limits=MotionLimits(effort=900.0, velocity=0.06, lower=0.0, upper=0.34),
    )
    for index in (0, 2):
        model.articulation(
            f"lift_{index}",
            ArticulationType.PRISMATIC,
            parent=base,
            child=inner_stages[index],
            origin=Origin(xyz=(column_xs[index], 0.0, sleeve_top_z)),
            axis=(0.0, 0.0, 1.0),
            motion_limits=MotionLimits(effort=900.0, velocity=0.06, lower=0.0, upper=0.34),
            mimic=Mimic(joint=center_lift.name, multiplier=1.0, offset=0.0),
        )

    desktop = model.part("desktop")
    desktop.visual(
        Box((2.46, 0.82, 0.055)),
        origin=Origin(xyz=(0.0, 0.0, 0.0725)),
        material=warm_oak,
        name="long_worktop",
    )
    # Separate edge bands give the long shared top a furniture-like perimeter.
    desktop.visual(
        Box((2.48, 0.020, 0.060)),
        origin=Origin(xyz=(0.0, -0.420, 0.073)),
        material=dark_edge,
        name="front_edge_band",
    )
    desktop.visual(
        Box((2.48, 0.020, 0.060)),
        origin=Origin(xyz=(0.0, 0.420, 0.073)),
        material=dark_edge,
        name="rear_edge_band",
    )
    desktop.visual(
        Box((0.020, 0.84, 0.060)),
        origin=Origin(xyz=(-1.240, 0.0, 0.073)),
        material=dark_edge,
        name="end_edge_band_0",
    )
    desktop.visual(
        Box((0.020, 0.84, 0.060)),
        origin=Origin(xyz=(1.240, 0.0, 0.073)),
        material=dark_edge,
        name="end_edge_band_1",
    )

    for y, name in ((-0.315, "front_under_rail"), (0.315, "rear_under_rail")):
        desktop.visual(
            Box((2.12, 0.052, 0.045)),
            origin=Origin(xyz=(0.0, y, 0.0225)),
            material=black,
            name=name,
        )
    for index, x in enumerate(column_xs):
        desktop.visual(
            Box((0.060, 0.630, 0.045)),
            origin=Origin(xyz=(x, 0.0, 0.0225)),
            material=black,
            name=f"cross_beam_{index}",
        )
        desktop.visual(
            Box((0.125, 0.125, 0.024)),
            origin=Origin(xyz=(x, 0.0, 0.012)),
            material=black,
            name=f"support_plate_{index}",
        )

    # A compact front paddle controller mounted beneath the center front edge.
    desktop.visual(
        Box((0.260, 0.066, 0.038)),
        origin=Origin(xyz=(0.0, -0.433, 0.019)),
        material=black,
        name="controller_body",
    )
    desktop.visual(
        Box((0.070, 0.150, 0.018)),
        origin=Origin(xyz=(0.0, -0.370, 0.024)),
        material=black,
        name="controller_neck",
    )
    for x in (-0.060, 0.060):
        for side in (-1, 1):
            desktop.visual(
                Box((0.008, 0.020, 0.025)),
                origin=Origin(xyz=(x + side * 0.032, -0.468, 0.027)),
                material=black,
                name=f"hinge_ear_{x:+.2f}_{side:+d}",
            )

    model.articulation(
        "center_stage_mount",
        ArticulationType.FIXED,
        parent=inner_stages[1],
        child=desktop,
        origin=Origin(xyz=(0.0, 0.0, desktop_mount_z)),
    )

    # The two front paddles are independent narrow tabs with visible air between
    # them, not a single rocker strip.
    for name, x, icon_z, stem_z in (
        ("up_paddle", -0.060, -0.022, -0.034),
        ("down_paddle", 0.060, -0.044, -0.032),
    ):
        paddle = model.part(name)
        paddle.visual(
            Cylinder(radius=0.007, length=0.052),
            origin=Origin(rpy=(0.0, math.pi / 2.0, 0.0)),
            material=graphite,
            name="pivot_barrel",
        )
        paddle.visual(
            Box((0.040, 0.012, 0.012)),
            origin=Origin(xyz=(0.0, -0.007, -0.008)),
            material=paddle_mat,
            name="barrel_web",
        )
        paddle.visual(
            Box((0.052, 0.012, 0.056)),
            origin=Origin(xyz=(0.0, -0.010, -0.034)),
            material=paddle_mat,
            name="separate_paddle_tab",
        )
        paddle.visual(
            Box((0.024, 0.0025, 0.006)),
            origin=Origin(xyz=(0.0, -0.017, icon_z)),
            material=white,
            name="paddle_legend",
        )
        paddle.visual(
            Box((0.006, 0.0025, 0.020)),
            origin=Origin(xyz=(0.0, -0.017, stem_z)),
            material=white,
            name="paddle_stem",
        )
        model.articulation(
            f"{name}_pivot",
            ArticulationType.REVOLUTE,
            parent=desktop,
            child=paddle,
            origin=Origin(xyz=(x, -0.471, 0.029)),
            axis=(1.0, 0.0, 0.0),
            motion_limits=MotionLimits(effort=0.3, velocity=4.0, lower=0.0, upper=0.32),
        )

    return model


def run_tests() -> TestReport:
    ctx = TestContext(object_model)
    base = object_model.get_part("base_frame")
    desktop = object_model.get_part("desktop")
    center = object_model.get_part("inner_stage_1")
    left = object_model.get_part("inner_stage_0")
    right = object_model.get_part("inner_stage_2")
    center_lift = object_model.get_articulation("center_lift")
    up_pivot = object_model.get_articulation("up_paddle_pivot")
    down_pivot = object_model.get_articulation("down_paddle_pivot")

    # The inner tubes are real telescoping members: centered within the hollow
    # outer columns in plan and retained in the sleeves at both low and high
    # positions.
    for stage, index in ((left, 0), (center, 1), (right, 2)):
        ctx.expect_gap(
            stage,
            base,
            axis="y",
            min_gap=0.006,
            positive_elem="inner_tube",
            negative_elem=f"front_outer_wall_{index}",
            name=f"{stage.name} clears front sleeve wall",
        )
        ctx.expect_gap(
            base,
            stage,
            axis="y",
            min_gap=0.006,
            positive_elem=f"rear_outer_wall_{index}",
            negative_elem="inner_tube",
            name=f"{stage.name} clears rear sleeve wall",
        )
        ctx.expect_gap(
            stage,
            base,
            axis="x",
            min_gap=0.006,
            positive_elem="inner_tube",
            negative_elem=f"side_outer_wall_{index}_0",
            name=f"{stage.name} clears side sleeve wall 0",
        )
        ctx.expect_gap(
            base,
            stage,
            axis="x",
            min_gap=0.006,
            positive_elem=f"side_outer_wall_{index}_1",
            negative_elem="inner_tube",
            name=f"{stage.name} clears side sleeve wall 1",
        )
        ctx.expect_overlap(
            stage,
            base,
            axes="z",
            elem_a="inner_tube",
            elem_b=f"front_outer_wall_{index}",
            min_overlap=0.120,
            name=f"{stage.name} retained at sleeve lip",
        )

    for stage, plate in (
        (left, "support_plate_0"),
        (center, "support_plate_1"),
        (right, "support_plate_2"),
    ):
        ctx.expect_contact(
            stage,
            desktop,
            elem_a="top_cap",
            elem_b=plate,
            contact_tol=0.001,
            name=f"{stage.name} touches rigid desktop frame",
        )

    rest_z = ctx.part_world_position(desktop)[2]
    with ctx.pose({center_lift: 0.34}):
        raised_z = ctx.part_world_position(desktop)[2]
        for stage, plate in (
            (left, "support_plate_0"),
            (center, "support_plate_1"),
            (right, "support_plate_2"),
        ):
            ctx.expect_contact(
                stage,
                desktop,
                elem_a="top_cap",
                elem_b=plate,
                contact_tol=0.001,
                name=f"{stage.name} remains under frame at full height",
            )
    ctx.check(
        "desktop rises with synchronized lifts",
        rest_z is not None and raised_z is not None and raised_z > rest_z + 0.30,
        details=f"rest_z={rest_z}, raised_z={raised_z}",
    )

    # The controller has two independently hinged paddles with a real gap
    # between them, and each local pivot moves without becoming a single rocker.
    ctx.expect_gap(
        "down_paddle",
        "up_paddle",
        axis="x",
        min_gap=0.010,
        name="up and down paddles are visibly split",
    )
    with ctx.pose({up_pivot: 0.28, down_pivot: 0.0}):
        ctx.expect_gap(
            "down_paddle",
            "up_paddle",
            axis="x",
            min_gap=0.010,
            name="paddle split remains during up press",
        )
    with ctx.pose({up_pivot: 0.0, down_pivot: 0.28}):
        ctx.expect_gap(
            "down_paddle",
            "up_paddle",
            axis="x",
            min_gap=0.010,
            name="paddle split remains during down press",
        )

    return ctx.report()


object_model = build_object_model()
