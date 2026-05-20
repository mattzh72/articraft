from __future__ import annotations

import math

from sdk import (
    ArticulatedObject,
    ArticulationType,
    Box,
    Cylinder,
    ExtrudeGeometry,
    Mimic,
    MotionLimits,
    Origin,
    TestContext,
    TestReport,
    mesh_from_geometry,
    rounded_rect_profile,
)


def build_object_model() -> ArticulatedObject:
    model = ArticulatedObject(name="drafting_standing_desk")

    matte_black = model.material("matte_black", rgba=(0.02, 0.022, 0.024, 1.0))
    dark_metal = model.material("dark_metal", rgba=(0.12, 0.13, 0.14, 1.0))
    satin_steel = model.material("satin_steel", rgba=(0.55, 0.57, 0.58, 1.0))
    birch = model.material("birch_laminate", rgba=(0.78, 0.62, 0.42, 1.0))
    edge_black = model.material("black_edge_band", rgba=(0.025, 0.025, 0.025, 1.0))
    rubber = model.material("rubber_black", rgba=(0.006, 0.006, 0.007, 1.0))
    white = model.material("white_marking", rgba=(0.92, 0.92, 0.88, 1.0))

    base = model.part("base_frame")

    # Two long studio-desk feet with cross ties form the fixed lifting frame.
    for i, x in enumerate((-0.43, 0.43)):
        base.visual(
            Box((0.13, 0.92, 0.050)),
            origin=Origin(xyz=(x, -0.04, 0.025)),
            material=matte_black,
            name=f"floor_foot_{i}",
        )
        base.visual(
            Box((0.17, 0.14, 0.020)),
            origin=Origin(xyz=(x, 0.12, 0.060)),
            material=matte_black,
            name=f"column_foot_plate_{i}",
        )

        # Hollow rectangular outer column, built from four walls so the sliding
        # leg stage reads as entering a real sleeve instead of a solid block.
        base.visual(
            Box((0.120, 0.014, 0.82)),
            origin=Origin(xyz=(x, 0.074, 0.47)),
            material=dark_metal,
            name="outer_column_0_front_wall" if i == 0 else "outer_column_1_front_wall",
        )
        for side, dx, dy, size in (
            ("rear_wall", 0.0, 0.046, (0.120, 0.014, 0.82)),
            ("side_wall_0", -0.053, 0.0, (0.014, 0.092, 0.82)),
            ("side_wall_1", 0.053, 0.0, (0.014, 0.092, 0.82)),
        ):
            base.visual(
                Box(size),
                origin=Origin(xyz=(x + dx, 0.12 + dy, 0.47)),
                material=dark_metal,
                name=f"outer_column_{i}_{side}",
            )

        for lip_side, dx, dy, size in (
            ("front", 0.0, -0.050, (0.130, 0.014, 0.026)),
            ("rear", 0.0, 0.050, (0.130, 0.014, 0.026)),
            ("side_0", -0.058, 0.0, (0.014, 0.100, 0.026)),
            ("side_1", 0.058, 0.0, (0.014, 0.100, 0.026)),
        ):
            base.visual(
                Box(size),
                origin=Origin(xyz=(x + dx, 0.12 + dy, 0.888)),
                material=dark_metal,
                name=f"sleeve_lip_{i}_{lip_side}",
            )

    base.visual(
        Box((0.98, 0.055, 0.052)),
        origin=Origin(xyz=(0.0, 0.34, 0.050)),
        material=matte_black,
        name="rear_floor_tie",
    )
    base.visual(
        Box((0.92, 0.045, 0.040)),
        origin=Origin(xyz=(0.0, -0.40, 0.050)),
        material=matte_black,
        name="front_floor_tie",
    )
    base.visual(
        Box((1.02, 0.080, 0.042)),
        origin=Origin(xyz=(0.0, 0.185, 0.835)),
        material=matte_black,
        name="upper_rear_tie",
    )

    leg_0 = model.part("leg_stage_0")
    leg_1 = model.part("leg_stage_1")
    for i, leg in enumerate((leg_0, leg_1)):
        leg.visual(
            Box((0.056, 0.046, 0.78)),
            # The lower portion remains hidden inside the outer column at all
            # allowed heights, giving the telescoping stage retained insertion.
            origin=Origin(xyz=(0.0, 0.0, 0.100)),
            material=satin_steel,
            name="inner_post",
        )
        for pad_name, dx, dy, size in (
            ("front_glide", 0.0, -0.031, (0.050, 0.018, 0.075)),
            ("rear_glide", 0.0, 0.031, (0.050, 0.018, 0.075)),
            ("side_glide_0", -0.0375, 0.0, (0.019, 0.040, 0.075)),
            ("side_glide_1", 0.0375, 0.0, (0.019, 0.040, 0.075)),
        ):
            leg.visual(
                Box(size),
                origin=Origin(xyz=(dx, dy, -0.250)),
                material=rubber,
                name=pad_name,
            )
        leg.visual(
            Box((0.104, 0.080, 0.030)),
            origin=Origin(xyz=(0.0, 0.0, 0.505)),
            material=satin_steel,
            name="top_cap",
        )
        model.articulation(
            f"base_to_leg_{i}",
            ArticulationType.PRISMATIC,
            parent=base,
            child=leg,
            origin=Origin(xyz=(-0.43 if i == 0 else 0.43, 0.12, 0.86)),
            axis=(0.0, 0.0, 1.0),
            motion_limits=MotionLimits(effort=320.0, velocity=0.08, lower=0.0, upper=0.24),
            mimic=None if i == 0 else Mimic("base_to_leg_0", multiplier=1.0),
        )

    hinge_beam = model.part("hinge_beam")
    hinge_beam.visual(
        Cylinder(radius=0.022, length=1.10),
        origin=Origin(rpy=(0.0, math.pi / 2.0, 0.0)),
        material=satin_steel,
        name="hinge_pin",
    )
    hinge_beam.visual(
        Box((1.16, 0.040, 0.035)),
        origin=Origin(xyz=(0.0, -0.018, -0.050)),
        material=matte_black,
        name="rear_hinge_rail",
    )
    for i, x in enumerate((-0.54, 0.54)):
        hinge_beam.visual(
            Box((0.050, 0.040, 0.050)),
            origin=Origin(xyz=(x, -0.018, -0.025)),
            material=matte_black,
            name=f"end_bearing_{i}",
        )
    for i, x in enumerate((-0.43, 0.43)):
        hinge_beam.visual(
            Box((0.034, 0.245, 0.035)),
            origin=Origin(xyz=(x + (-0.060 if x < 0.0 else 0.060), -0.110, -0.048)),
            material=matte_black,
            name=f"saddle_arm_{i}",
        )
        hinge_beam.visual(
            Box((0.125, 0.030, 0.042)),
            origin=Origin(xyz=(x, -0.233, -0.015)),
            material=matte_black,
            name=f"leg_saddle_{i}",
        )
    model.articulation(
        "leg_to_hinge_beam",
        ArticulationType.FIXED,
        parent=leg_0,
        child=hinge_beam,
        origin=Origin(xyz=(0.43, 0.18, 0.515)),
    )

    work = model.part("work_surface")
    top_profile = rounded_rect_profile(1.25, 0.76, 0.040, corner_segments=10)
    work.visual(
        mesh_from_geometry(
            ExtrudeGeometry(top_profile, 0.040, center=True),
            "rounded_work_surface",
        ),
        origin=Origin(xyz=(0.0, -0.380, 0.055)),
        material=birch,
        name="tilting_board",
    )
    work.visual(
        Box((1.18, 0.040, 0.070)),
        origin=Origin(xyz=(0.0, -0.755, 0.020)),
        material=edge_black,
        name="front_lip",
    )
    work.visual(
        Box((1.18, 0.024, 0.022)),
        origin=Origin(xyz=(0.0, -0.735, 0.082)),
        material=birch,
        name="pencil_stop",
    )
    for i, x in enumerate((-0.36, 0.0, 0.36)):
        work.visual(
            Box((0.175, 0.070, 0.016)),
            origin=Origin(xyz=(x, -0.033, 0.029)),
            material=satin_steel,
            name=f"hinge_leaf_{i}",
        )
        work.visual(
            Cylinder(radius=0.028, length=0.155),
            origin=Origin(xyz=(x, 0.0, 0.0), rpy=(0.0, math.pi / 2.0, 0.0)),
            material=satin_steel,
            name=f"hinge_sleeve_{i}",
        )
    model.articulation(
        "hinge_beam_to_surface",
        ArticulationType.REVOLUTE,
        parent=hinge_beam,
        child=work,
        origin=Origin(),
        axis=(-1.0, 0.0, 0.0),
        motion_limits=MotionLimits(effort=45.0, velocity=0.45, lower=0.0, upper=0.65),
    )

    control = model.part("control_strip")
    control.visual(
        Box((1.05, 0.030, 0.022)),
        origin=Origin(),
        material=matte_black,
        name="front_control_bar",
    )
    control.visual(
        Box((0.280, 0.060, 0.048)),
        origin=Origin(xyz=(0.385, -0.006, -0.026)),
        material=matte_black,
        name="controller_body",
    )
    control.visual(
        Box((0.012, 0.046, 0.032)),
        origin=Origin(xyz=(0.385, -0.012, -0.050)),
        material=rubber,
        name="paddle_split_gap",
    )
    model.articulation(
        "surface_to_control",
        ArticulationType.FIXED,
        parent=work,
        child=control,
        origin=Origin(xyz=(0.0, -0.760, -0.0255)),
    )

    up_paddle = model.part("up_paddle")
    down_paddle = model.part("down_paddle")
    for part, arrow_name, points in (
        (up_paddle, "up_arrow", [(-0.010, -0.006), (0.010, -0.006), (0.0, 0.010)]),
        (down_paddle, "down_arrow", [(-0.010, 0.006), (0.010, 0.006), (0.0, -0.010)]),
    ):
        part.visual(
            Cylinder(radius=0.006, length=0.074),
            origin=Origin(rpy=(0.0, math.pi / 2.0, 0.0)),
            material=rubber,
            name="pivot_barrel",
        )
        part.visual(
            Box((0.074, 0.042, 0.010)),
            origin=Origin(xyz=(0.0, -0.021, -0.01075)),
            material=rubber,
            name="paddle_plate",
        )
        part.visual(
            mesh_from_geometry(ExtrudeGeometry(points, 0.0012, center=True), arrow_name),
            origin=Origin(xyz=(0.0, -0.021, -0.0160)),
            material=white,
            name=arrow_name,
        )

    model.articulation(
        "control_to_up_paddle",
        ArticulationType.REVOLUTE,
        parent=control,
        child=up_paddle,
        origin=Origin(xyz=(0.335, -0.015, -0.055)),
        axis=(1.0, 0.0, 0.0),
        motion_limits=MotionLimits(effort=1.5, velocity=3.0, lower=-0.12, upper=0.22),
    )
    model.articulation(
        "control_to_down_paddle",
        ArticulationType.REVOLUTE,
        parent=control,
        child=down_paddle,
        origin=Origin(xyz=(0.435, -0.015, -0.055)),
        axis=(1.0, 0.0, 0.0),
        motion_limits=MotionLimits(effort=1.5, velocity=3.0, lower=-0.12, upper=0.22),
    )

    return model


def run_tests() -> TestReport:
    ctx = TestContext(object_model)

    hinge_beam = object_model.get_part("hinge_beam")
    work = object_model.get_part("work_surface")
    leg_0 = object_model.get_part("leg_stage_0")
    leg_1 = object_model.get_part("leg_stage_1")
    up_paddle = object_model.get_part("up_paddle")
    down_paddle = object_model.get_part("down_paddle")

    lift = object_model.get_articulation("base_to_leg_0")
    tilt = object_model.get_articulation("hinge_beam_to_surface")
    up_joint = object_model.get_articulation("control_to_up_paddle")
    down_joint = object_model.get_articulation("control_to_down_paddle")

    for sleeve in ("hinge_sleeve_0", "hinge_sleeve_1", "hinge_sleeve_2"):
        ctx.allow_overlap(
            work,
            hinge_beam,
            elem_a=sleeve,
            elem_b="hinge_pin",
            reason="The visible tabletop hinge sleeve is intentionally modeled around the captured hinge pin.",
        )
        ctx.expect_overlap(
            work,
            hinge_beam,
            axes="x",
            elem_a=sleeve,
            elem_b="hinge_pin",
            min_overlap=0.12,
            name=f"{sleeve} captures the hinge pin along the beam",
        )

    ctx.expect_overlap(
        leg_0,
        "base_frame",
        axes="z",
        elem_a="inner_post",
        elem_b="outer_column_0_front_wall",
        min_overlap=0.08,
        name="leg stage 0 remains inserted in its outer column",
    )
    ctx.expect_overlap(
        leg_1,
        "base_frame",
        axes="z",
        elem_a="inner_post",
        elem_b="outer_column_1_front_wall",
        min_overlap=0.08,
        name="leg stage 1 remains inserted in its outer column",
    )

    rest_leg = ctx.part_world_position(leg_0)
    with ctx.pose({lift: 0.24}):
        raised_leg = ctx.part_world_position(leg_0)
        ctx.expect_overlap(
            leg_0,
            "base_frame",
            axes="z",
            elem_a="inner_post",
            elem_b="outer_column_0_front_wall",
            min_overlap=0.06,
            name="raised leg stage 0 keeps retained insertion",
        )
        ctx.expect_overlap(
            leg_1,
            "base_frame",
            axes="z",
            elem_a="inner_post",
            elem_b="outer_column_1_front_wall",
            min_overlap=0.06,
            name="raised leg stage 1 keeps retained insertion",
        )
    ctx.check(
        "lift joint raises the telescoping stages",
        rest_leg is not None and raised_leg is not None and raised_leg[2] > rest_leg[2] + 0.20,
        details=f"rest={rest_leg}, raised={raised_leg}",
    )

    rest_front = ctx.part_element_world_aabb(work, elem="front_lip")
    with ctx.pose({tilt: 0.65}):
        tilted_front = ctx.part_element_world_aabb(work, elem="front_lip")
    rest_front_z = None if rest_front is None else (rest_front[0][2] + rest_front[1][2]) * 0.5
    tilted_front_z = None if tilted_front is None else (tilted_front[0][2] + tilted_front[1][2]) * 0.5
    ctx.check(
        "tilt hinge raises the front lip",
        rest_front_z is not None and tilted_front_z is not None and tilted_front_z > rest_front_z + 0.35,
        details=f"rest_z={rest_front_z}, tilted_z={tilted_front_z}",
    )

    ctx.expect_origin_distance(
        up_paddle,
        down_paddle,
        axes="x",
        min_dist=0.095,
        max_dist=0.105,
        name="up and down paddles are visibly split side by side",
    )

    rest_up = ctx.part_element_world_aabb(up_paddle, elem="paddle_plate")
    rest_down = ctx.part_element_world_aabb(down_paddle, elem="paddle_plate")
    with ctx.pose({up_joint: 0.22, down_joint: -0.12}):
        moved_up = ctx.part_element_world_aabb(up_paddle, elem="paddle_plate")
        moved_down = ctx.part_element_world_aabb(down_paddle, elem="paddle_plate")
    rest_up_z = None if rest_up is None else (rest_up[0][2] + rest_up[1][2]) * 0.5
    moved_up_z = None if moved_up is None else (moved_up[0][2] + moved_up[1][2]) * 0.5
    rest_down_z = None if rest_down is None else (rest_down[0][2] + rest_down[1][2]) * 0.5
    moved_down_z = None if moved_down is None else (moved_down[0][2] + moved_down[1][2]) * 0.5
    ctx.check(
        "each paddle has an independent short pivot",
        rest_up_z is not None
        and moved_up_z is not None
        and rest_down_z is not None
        and moved_down_z is not None
        and abs(moved_up_z - rest_up_z) > 0.003
        and abs(moved_down_z - rest_down_z) > 0.001,
        details=f"up z {rest_up_z}->{moved_up_z}, down z {rest_down_z}->{moved_down_z}",
    )

    return ctx.report()


object_model = build_object_model()
