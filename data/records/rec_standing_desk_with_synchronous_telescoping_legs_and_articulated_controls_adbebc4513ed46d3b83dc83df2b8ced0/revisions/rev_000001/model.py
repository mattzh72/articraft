from __future__ import annotations

from sdk import (
    ArticulatedObject,
    ArticulationType,
    Box,
    Cylinder,
    Material,
    Mimic,
    MotionLimits,
    Origin,
    TestContext,
    TestReport,
)


def build_object_model() -> ArticulatedObject:
    model = ArticulatedObject(name="bench_standing_desk")

    wood = model.material("warm_bamboo_top", rgba=(0.72, 0.52, 0.30, 1.0))
    black = model.material("satin_black_steel", rgba=(0.015, 0.017, 0.018, 1.0))
    dark = model.material("dark_plastic", rgba=(0.025, 0.028, 0.032, 1.0))
    grey = model.material("brushed_aluminum", rgba=(0.55, 0.58, 0.58, 1.0))
    rubber = model.material("matte_rubber", rgba=(0.02, 0.02, 0.018, 1.0))

    # The base is a connected floor assembly carrying three hollow outer columns.
    # X is the long shared-workstation span, Y is depth (negative Y is front),
    # and Z is height from the floor.
    column_x = (-0.85, 0.0, 0.85)
    column_top_z = 0.55
    column_bottom_z = 0.045
    column_height = column_top_z - column_bottom_z
    column_center_z = (column_top_z + column_bottom_z) / 2.0
    outer_w = 0.110
    wall = 0.018
    opening = outer_w - 2.0 * wall

    base = model.part("base")
    # Long floor rails make the three lifting pods a single bench-style base.
    for y, name in ((-0.34, "front_floor_rail"), (0.34, "rear_floor_rail")):
        base.visual(
            Box((2.04, 0.055, 0.040)),
            origin=Origin(xyz=(0.0, y, 0.020)),
            material=black,
            name=name,
        )
    for i, x in enumerate(column_x):
        base.visual(
            Box((0.24, 0.78, 0.045)),
            origin=Origin(xyz=(x, 0.0, 0.0225)),
            material=black,
            name=f"foot_{i}",
        )
        for y, name in ((-0.36, f"front_glide_{i}"), (0.36, f"rear_glide_{i}")):
            base.visual(
                Cylinder(radius=0.035, length=0.010),
                origin=Origin(xyz=(x, y, 0.005)),
                material=rubber,
                name=name,
            )

        # Square tube walls.  The center is open so the inner stage can move
        # through the sleeve without needing an artificial overlap allowance.
        base.visual(
            Box((outer_w, wall, column_height)),
            origin=Origin(xyz=(x, -outer_w / 2.0 + wall / 2.0, column_center_z)),
            material=black,
            name=f"column_{i}_front_wall",
        )
        base.visual(
            Box((outer_w, wall, column_height)),
            origin=Origin(xyz=(x, outer_w / 2.0 - wall / 2.0, column_center_z)),
            material=black,
            name=f"column_{i}_rear_wall",
        )
        base.visual(
            Box((wall, opening, column_height)),
            origin=Origin(xyz=(x - outer_w / 2.0 + wall / 2.0, 0.0, column_center_z)),
            material=black,
            name=f"column_{i}_side_wall_0",
        )
        base.visual(
            Box((wall, opening, column_height)),
            origin=Origin(xyz=(x + outer_w / 2.0 - wall / 2.0, 0.0, column_center_z)),
            material=black,
            name=f"column_{i}_side_wall_1",
        )
        # Slightly lighter wear collars at the top make the telescoping sleeves
        # read clearly while keeping the aperture open.
        base.visual(
            Box((outer_w, wall, 0.026)),
            origin=Origin(xyz=(x, -outer_w / 2.0 + wall / 2.0, column_top_z - 0.013)),
            material=grey,
            name=f"top_collar_{i}_front",
        )
        base.visual(
            Box((outer_w, wall, 0.026)),
            origin=Origin(xyz=(x, outer_w / 2.0 - wall / 2.0, column_top_z - 0.013)),
            material=grey,
            name=f"top_collar_{i}_rear",
        )
        base.visual(
            Box((wall, opening, 0.026)),
            origin=Origin(xyz=(x - outer_w / 2.0 + wall / 2.0, 0.0, column_top_z - 0.013)),
            material=grey,
            name=f"top_collar_{i}_side_0",
        )
        base.visual(
            Box((wall, opening, 0.026)),
            origin=Origin(xyz=(x + outer_w / 2.0 - wall / 2.0, 0.0, column_top_z - 0.013)),
            material=grey,
            name=f"top_collar_{i}_side_1",
        )

    # Three inner stages slide vertically in synchrony.  Their local frames sit
    # at the sleeve mouth; the geometry extends downward for retained insertion
    # and upward to touch the desktop frame mounting plates.
    inner_stages = []
    for i, x in enumerate(column_x):
        stage = model.part(f"inner_stage_{i}")
        stage.visual(
            Box((opening, opening, 0.588)),
            origin=Origin(xyz=(0.0, 0.0, -0.204)),
            material=grey,
            name="inner_column",
        )
        stage.visual(
            Box((0.092, 0.092, 0.018)),
            origin=Origin(xyz=(0.0, 0.0, 0.081)),
            material=grey,
            name="top_cap",
        )
        inner_stages.append(stage)

        mimic = None if i == 1 else Mimic(joint="lift_stage_1", multiplier=1.0, offset=0.0)
        model.articulation(
            f"lift_stage_{i}",
            ArticulationType.PRISMATIC,
            parent=base,
            child=stage,
            origin=Origin(xyz=(x, 0.0, column_top_z)),
            axis=(0.0, 0.0, 1.0),
            motion_limits=MotionLimits(effort=950.0, velocity=0.055, lower=0.0, upper=0.40),
            mimic=mimic,
        )

    desktop = model.part("desktop_frame")
    # A long rigid top for a shared bench workstation.
    desktop.visual(
        Box((2.45, 0.92, 0.055)),
        origin=Origin(xyz=(0.0, 0.0, 0.0795)),
        material=wood,
        name="desktop_top",
    )
    # Steel frame directly beneath the top; the crossmembers tie all three
    # lift columns into the same rigid desktop assembly.
    for y, name in ((-0.335, "front_frame_rail"), (0.335, "rear_frame_rail")):
        desktop.visual(
            Box((2.22, 0.050, 0.040)),
            origin=Origin(xyz=(0.0, y, 0.032)),
            material=black,
            name=name,
        )
    for i, x in enumerate(column_x):
        desktop.visual(
            Box((0.060, 0.72, 0.040)),
            origin=Origin(xyz=(x, 0.0, 0.032)),
            material=black,
            name=f"crossmember_{i}",
        )
        desktop.visual(
            Box((0.180, 0.140, 0.012)),
            origin=Origin(xyz=(x, 0.0, 0.006)),
            material=black,
            name=f"mount_plate_{i}",
        )

    # Short side guides for the sliding keyboard tray, visibly hung from the
    # underside of the front frame/top.
    desktop.visual(
        Box((0.052, 0.420, 0.030)),
        origin=Origin(xyz=(-0.43, -0.28, -0.039)),
        material=black,
        name="tray_guide_0",
    )
    desktop.visual(
        Box((0.060, 0.045, 0.076)),
        origin=Origin(xyz=(-0.43, -0.43, 0.014)),
        material=black,
        name="guide_bracket_0",
    )
    desktop.visual(
        Box((0.052, 0.420, 0.030)),
        origin=Origin(xyz=(0.43, -0.28, -0.039)),
        material=black,
        name="tray_guide_1",
    )
    desktop.visual(
        Box((0.060, 0.045, 0.076)),
        origin=Origin(xyz=(0.43, -0.43, 0.014)),
        material=black,
        name="guide_bracket_1",
    )

    # Fixed housing for the under-edge paddle controller.
    desktop.visual(
        Box((0.165, 0.040, 0.070)),
        origin=Origin(xyz=(0.0, -0.455, 0.017)),
        material=dark,
        name="controller_bracket",
    )
    desktop.visual(
        Box((0.200, 0.060, 0.030)),
        origin=Origin(xyz=(0.0, -0.500, -0.025)),
        material=dark,
        name="controller_housing",
    )

    model.articulation(
        "center_stage_to_desktop",
        ArticulationType.FIXED,
        parent=inner_stages[1],
        child=desktop,
        origin=Origin(xyz=(0.0, 0.0, 0.090)),
    )

    tray = model.part("keyboard_tray")
    tray.visual(
        Box((0.825, 0.300, 0.022)),
        origin=Origin(xyz=(0.0, -0.270, -0.090)),
        material=dark,
        name="tray_shelf",
    )
    tray.visual(
        Box((0.825, 0.025, 0.040)),
        origin=Origin(xyz=(0.0, -0.4275, -0.072)),
        material=dark,
        name="front_lip",
    )
    tray.visual(
        Box((0.035, 0.300, 0.025)),
        origin=Origin(xyz=(-0.430, -0.270, -0.0665)),
        material=grey,
        name="runner_0",
    )
    tray.visual(
        Box((0.035, 0.300, 0.025)),
        origin=Origin(xyz=(0.430, -0.270, -0.0665)),
        material=grey,
        name="runner_1",
    )
    model.articulation(
        "tray_slide",
        ArticulationType.PRISMATIC,
        parent=desktop,
        child=tray,
        origin=Origin(),
        axis=(0.0, -1.0, 0.0),
        motion_limits=MotionLimits(effort=80.0, velocity=0.25, lower=0.0, upper=0.26),
    )

    paddle = model.part("paddle")
    paddle.visual(
        Box((0.155, 0.014, 0.065)),
        origin=Origin(xyz=(0.0, 0.0, -0.0325)),
        material=Material("soft_blue_paddle", rgba=(0.05, 0.13, 0.18, 1.0)),
        name="paddle_plate",
    )
    model.articulation(
        "paddle_hinge",
        ArticulationType.REVOLUTE,
        parent=desktop,
        child=paddle,
        origin=Origin(xyz=(0.0, -0.525, -0.040)),
        axis=(1.0, 0.0, 0.0),
        motion_limits=MotionLimits(effort=2.0, velocity=1.4, lower=-0.35, upper=0.35),
    )

    return model


def run_tests() -> TestReport:
    ctx = TestContext(object_model)
    base = object_model.get_part("base")
    desktop = object_model.get_part("desktop_frame")
    tray = object_model.get_part("keyboard_tray")
    lift = object_model.get_articulation("lift_stage_1")
    tray_slide = object_model.get_articulation("tray_slide")
    paddle_hinge = object_model.get_articulation("paddle_hinge")

    ctx.check(
        "three synchronized lift joints",
        all(
            object_model.get_articulation(f"lift_stage_{i}").mimic is not None
            for i in (0, 2)
        )
        and object_model.get_articulation("lift_stage_1").mimic is None,
        details="side lift stages should mimic the center lift stage",
    )

    for i in range(3):
        stage = object_model.get_part(f"inner_stage_{i}")
        ctx.expect_overlap(
            stage,
            base,
            axes="z",
            min_overlap=0.09,
            elem_a="inner_column",
            elem_b=f"column_{i}_front_wall",
            name=f"stage {i} remains inserted at rest",
        )
        ctx.expect_contact(
            stage,
            desktop,
            elem_a="top_cap",
            elem_b=f"mount_plate_{i}",
            name=f"stage {i} supports desktop at rest",
        )

    rest_desktop = ctx.part_world_position(desktop)
    with ctx.pose({lift: 0.40}):
        raised_desktop = ctx.part_world_position(desktop)
        for i in range(3):
            stage = object_model.get_part(f"inner_stage_{i}")
            ctx.expect_overlap(
                stage,
                base,
                axes="z",
                min_overlap=0.09,
                elem_a="inner_column",
                elem_b=f"column_{i}_front_wall",
                name=f"stage {i} retains insertion when raised",
            )
            ctx.expect_contact(
                stage,
                desktop,
                elem_a="top_cap",
                elem_b=f"mount_plate_{i}",
                name=f"stage {i} still supports raised desktop",
            )
    ctx.check(
        "desktop rises with lift",
        rest_desktop is not None
        and raised_desktop is not None
        and raised_desktop[2] > rest_desktop[2] + 0.38,
        details=f"rest={rest_desktop}, raised={raised_desktop}",
    )

    ctx.expect_contact(
        tray,
        desktop,
        elem_a="runner_0",
        elem_b="tray_guide_0",
        name="tray left runner rides guide",
    )
    ctx.expect_contact(
        tray,
        desktop,
        elem_a="runner_1",
        elem_b="tray_guide_1",
        name="tray right runner rides guide",
    )
    closed_tray = ctx.part_world_position(tray)
    with ctx.pose({tray_slide: 0.26}):
        open_tray = ctx.part_world_position(tray)
        ctx.expect_overlap(
            tray,
            desktop,
            axes="y",
            min_overlap=0.10,
            elem_a="runner_0",
            elem_b="tray_guide_0",
            name="extended tray remains captured by guide",
        )
    ctx.check(
        "tray slides forward",
        closed_tray is not None
        and open_tray is not None
        and open_tray[1] < closed_tray[1] - 0.24,
        details=f"closed={closed_tray}, open={open_tray}",
    )

    ctx.check(
        "paddle controller has small hinge travel",
        paddle_hinge.motion_limits is not None
        and paddle_hinge.motion_limits.lower <= -0.30
        and paddle_hinge.motion_limits.upper >= 0.30,
        details=f"limits={paddle_hinge.motion_limits}",
    )

    return ctx.report()


object_model = build_object_model()
