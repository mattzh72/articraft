from __future__ import annotations

from math import pi

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
    TireGeometry,
    mesh_from_cadquery,
    mesh_from_geometry,
)


def build_object_model() -> ArticulatedObject:
    model = ArticulatedObject(name="pneumatic_overbed_table")

    metal = model.material("powder_coated_metal", rgba=(0.72, 0.74, 0.75, 1.0))
    dark_metal = model.material("dark_hardware", rgba=(0.08, 0.09, 0.10, 1.0))
    column_plastic = model.material("warm_grey_column_cover", rgba=(0.82, 0.83, 0.80, 1.0))
    chrome = model.material("brushed_chrome", rgba=(0.78, 0.80, 0.78, 1.0))
    laminate = model.material("clinical_laminate", rgba=(0.94, 0.91, 0.83, 1.0))
    edge = model.material("soft_grey_edge_bumper", rgba=(0.55, 0.58, 0.58, 1.0))
    black = model.material("black_rubber", rgba=(0.01, 0.01, 0.012, 1.0))
    paddle_blue = model.material("blue_release_paddle", rgba=(0.05, 0.21, 0.46, 1.0))

    base = model.part("base")

    # Rolling C-base: two long low rails joined at the column end, open toward +X.
    base.visual(
        Box((1.02, 0.055, 0.045)),
        origin=Origin(xyz=(0.05, 0.285, 0.125)),
        material=metal,
        name="rail_0",
    )
    base.visual(
        Box((1.02, 0.055, 0.045)),
        origin=Origin(xyz=(0.05, -0.285, 0.125)),
        material=metal,
        name="rail_1",
    )
    base.visual(
        Box((0.080, 0.64, 0.045)),
        origin=Origin(xyz=(-0.46, 0.0, 0.125)),
        material=metal,
        name="rear_crossbar",
    )
    base.visual(
        Box((0.18, 0.16, 0.030)),
        origin=Origin(xyz=(-0.43, 0.0, 0.160)),
        material=metal,
        name="column_foot",
    )

    # A hollow rectangular gas-spring/column cover leaves the sliding mast visible
    # as a separate nested member instead of collapsing the support into one post.
    outer_sleeve_shape = (
        cq.Workplane("XY")
        .rect(0.112, 0.086)
        .rect(0.074, 0.048)
        .extrude(0.58)
        .translate((0.0, 0.0, -0.29))
    )
    base.visual(
        mesh_from_cadquery(outer_sleeve_shape, "outer_sleeve"),
        origin=Origin(xyz=(-0.43, 0.0, 0.405)),
        material=column_plastic,
        name="outer_sleeve",
    )
    top_collar_shape = (
        cq.Workplane("XY")
        .rect(0.132, 0.106)
        .rect(0.074, 0.048)
        .extrude(0.026)
        .translate((0.0, 0.0, -0.013))
    )
    base.visual(
        mesh_from_cadquery(top_collar_shape, "sleeve_top_collar"),
        origin=Origin(xyz=(-0.43, 0.0, 0.705)),
        material=edge,
        name="sleeve_top_collar",
    )
    base.visual(
        Box((0.006, 0.030, 0.050)),
        origin=Origin(xyz=(-0.395, 0.0, 0.705)),
        material=edge,
        name="guide_pad_0",
    )
    base.visual(
        Box((0.006, 0.030, 0.050)),
        origin=Origin(xyz=(-0.465, 0.0, 0.705)),
        material=edge,
        name="guide_pad_1",
    )

    tire_mesh = mesh_from_geometry(
        TireGeometry(0.046, 0.032, inner_radius=0.031),
        "caster_tire",
    )

    caster_positions = [
        (-0.43, 0.285, 0.052),
        (-0.43, -0.285, 0.052),
        (0.54, 0.285, 0.052),
        (0.54, -0.285, 0.052),
    ]
    for i, (x, y, z) in enumerate(caster_positions):
        # Fork cheeks and axle are static hardware on the C-base; the wheel link
        # itself spins continuously about the axle.
        base.visual(
            Box((0.006, 0.060, 0.096)),
            origin=Origin(xyz=(x - 0.026, y, z + 0.029)),
            material=dark_metal,
            name=f"fork_cheek_{i}_0",
        )
        base.visual(
            Box((0.006, 0.060, 0.096)),
            origin=Origin(xyz=(x + 0.026, y, z + 0.029)),
            material=dark_metal,
            name=f"fork_cheek_{i}_1",
        )
        base.visual(
            Box((0.070, 0.060, 0.014)),
            origin=Origin(xyz=(x, y, z + 0.058)),
            material=dark_metal,
            name=f"fork_bridge_{i}",
        )
        base.visual(
            Cylinder(radius=0.006, length=0.068),
            origin=Origin(xyz=(x, y, z), rpy=(0.0, pi / 2.0, 0.0)),
            material=dark_metal,
            name=f"axle_{i}",
        )

        caster = model.part(f"caster_{i}")
        caster.visual(tire_mesh, material=black, name="tire")
        caster.visual(
            Cylinder(radius=0.030, length=0.024),
            origin=Origin(rpy=(0.0, pi / 2.0, 0.0)),
            material=metal,
            name="rim",
        )
        # A small face mark makes the continuous spin visually legible.
        caster.visual(
            Box((0.006, 0.010, 0.010)),
            origin=Origin(xyz=(0.014, 0.0, 0.020)),
            material=paddle_blue,
            name="spin_mark",
        )
        model.articulation(
            f"base_to_caster_{i}",
            ArticulationType.CONTINUOUS,
            parent=base,
            child=caster,
            origin=Origin(xyz=(x, y, z)),
            axis=(1.0, 0.0, 0.0),
            motion_limits=MotionLimits(effort=8.0, velocity=12.0),
        )

    lift_column = model.part("lift_column")
    lift_column.visual(
        Box((0.064, 0.040, 0.650)),
        origin=Origin(xyz=(0.0, 0.0, -0.155)),
        material=chrome,
        name="inner_mast",
    )
    lift_column.visual(
        Cylinder(radius=0.010, length=0.56),
        origin=Origin(xyz=(0.020, 0.0, -0.135)),
        material=dark_metal,
        name="gas_spring_rod",
    )
    lift_column.visual(
        Box((0.084, 0.39, 0.045)),
        origin=Origin(xyz=(-0.042, 0.0, 0.165)),
        material=dark_metal,
        name="tilt_head",
    )
    lift_column.visual(
        Cylinder(radius=0.018, length=0.080),
        origin=Origin(xyz=(0.0, 0.163, 0.205), rpy=(pi / 2.0, 0.0, 0.0)),
        material=dark_metal,
        name="head_knuckle_0",
    )
    lift_column.visual(
        Cylinder(radius=0.018, length=0.080),
        origin=Origin(xyz=(0.0, -0.163, 0.205), rpy=(pi / 2.0, 0.0, 0.0)),
        material=dark_metal,
        name="head_knuckle_1",
    )
    model.articulation(
        "base_to_lift_column",
        ArticulationType.PRISMATIC,
        parent=base,
        child=lift_column,
        origin=Origin(xyz=(-0.43, 0.0, 0.705)),
        axis=(0.0, 0.0, 1.0),
        motion_limits=MotionLimits(effort=120.0, velocity=0.18, lower=0.0, upper=0.24),
    )

    main_tray = model.part("main_tray")
    main_tray.visual(
        Box((0.780, 0.460, 0.032)),
        origin=Origin(xyz=(0.390, 0.0, 0.030)),
        material=laminate,
        name="tray_panel",
    )
    main_tray.visual(
        Box((0.805, 0.020, 0.047)),
        origin=Origin(xyz=(0.402, 0.235, 0.057)),
        material=edge,
        name="front_lip",
    )
    main_tray.visual(
        Box((0.805, 0.020, 0.047)),
        origin=Origin(xyz=(0.402, -0.235, 0.057)),
        material=edge,
        name="rear_lip",
    )
    main_tray.visual(
        Box((0.024, 0.488, 0.047)),
        origin=Origin(xyz=(0.000, 0.0, 0.057)),
        material=edge,
        name="column_edge_lip",
    )
    main_tray.visual(
        Box((0.024, 0.488, 0.047)),
        origin=Origin(xyz=(0.780, 0.0, 0.057)),
        material=edge,
        name="wing_edge_lip",
    )
    main_tray.visual(
        Cylinder(radius=0.017, length=0.250),
        origin=Origin(xyz=(0.0, 0.0, 0.0), rpy=(pi / 2.0, 0.0, 0.0)),
        material=dark_metal,
        name="tray_knuckle",
    )
    # Short bracket at the main tray edge supporting the auxiliary reading wing.
    main_tray.visual(
        Box((0.150, 0.018, 0.014)),
        origin=Origin(xyz=(0.725, 0.150, 0.008)),
        material=dark_metal,
        name="wing_support_0",
    )
    main_tray.visual(
        Box((0.150, 0.018, 0.014)),
        origin=Origin(xyz=(0.725, -0.150, 0.008)),
        material=dark_metal,
        name="wing_support_1",
    )
    main_tray.visual(
        Cylinder(radius=0.012, length=0.090),
        origin=Origin(xyz=(0.805, 0.145, 0.020), rpy=(pi / 2.0, 0.0, 0.0)),
        material=dark_metal,
        name="wing_hinge_knuckle_0",
    )
    main_tray.visual(
        Cylinder(radius=0.012, length=0.090),
        origin=Origin(xyz=(0.805, -0.145, 0.020), rpy=(pi / 2.0, 0.0, 0.0)),
        material=dark_metal,
        name="wing_hinge_knuckle_1",
    )
    # Bracket cheeks for the release paddle mounted under the specified right side.
    main_tray.visual(
        Box((0.030, 0.008, 0.050)),
        origin=Origin(xyz=(0.615, 0.043, -0.010)),
        material=dark_metal,
        name="paddle_bracket_0",
    )
    main_tray.visual(
        Box((0.030, 0.008, 0.050)),
        origin=Origin(xyz=(0.615, -0.043, -0.010)),
        material=dark_metal,
        name="paddle_bracket_1",
    )
    model.articulation(
        "lift_column_to_tray",
        ArticulationType.REVOLUTE,
        parent=lift_column,
        child=main_tray,
        origin=Origin(xyz=(0.0, 0.0, 0.205)),
        axis=(0.0, -1.0, 0.0),
        motion_limits=MotionLimits(effort=45.0, velocity=0.8, lower=-0.20, upper=0.85),
    )

    reading_wing = model.part("reading_wing")
    reading_wing.visual(
        Box((0.285, 0.360, 0.026)),
        origin=Origin(xyz=(0.153, 0.0, 0.027)),
        material=laminate,
        name="wing_panel",
    )
    reading_wing.visual(
        Box((0.300, 0.017, 0.037)),
        origin=Origin(xyz=(0.158, 0.188, 0.049)),
        material=edge,
        name="wing_front_lip",
    )
    reading_wing.visual(
        Box((0.300, 0.017, 0.037)),
        origin=Origin(xyz=(0.158, -0.188, 0.049)),
        material=edge,
        name="wing_rear_lip",
    )
    reading_wing.visual(
        Box((0.017, 0.375, 0.037)),
        origin=Origin(xyz=(0.305, 0.0, 0.049)),
        material=edge,
        name="wing_outer_lip",
    )
    reading_wing.visual(
        Cylinder(radius=0.011, length=0.200),
        origin=Origin(xyz=(0.0, 0.0, 0.020), rpy=(pi / 2.0, 0.0, 0.0)),
        material=dark_metal,
        name="wing_knuckle",
    )
    model.articulation(
        "tray_to_reading_wing",
        ArticulationType.REVOLUTE,
        parent=main_tray,
        child=reading_wing,
        origin=Origin(xyz=(0.805, 0.0, 0.020)),
        axis=(0.0, -1.0, 0.0),
        motion_limits=MotionLimits(effort=12.0, velocity=1.2, lower=-0.15, upper=1.25),
    )

    release_paddle = model.part("release_paddle")
    release_paddle.visual(
        Cylinder(radius=0.010, length=0.078),
        origin=Origin(rpy=(pi / 2.0, 0.0, 0.0)),
        material=dark_metal,
        name="pivot_barrel",
    )
    release_paddle.visual(
        Box((0.075, 0.030, 0.028)),
        origin=Origin(xyz=(0.045, 0.0, -0.015)),
        material=paddle_blue,
        name="paddle_neck",
    )
    release_paddle.visual(
        Box((0.145, 0.070, 0.012)),
        origin=Origin(xyz=(0.120, 0.0, -0.035)),
        material=paddle_blue,
        name="paddle_plate",
    )
    model.articulation(
        "tray_to_release_paddle",
        ArticulationType.REVOLUTE,
        parent=main_tray,
        child=release_paddle,
        origin=Origin(xyz=(0.615, 0.0, -0.018)),
        axis=(0.0, 1.0, 0.0),
        motion_limits=MotionLimits(effort=3.0, velocity=4.0, lower=0.0, upper=0.55),
    )

    return model


def run_tests() -> TestReport:
    ctx = TestContext(object_model)

    base = object_model.get_part("base")
    lift_column = object_model.get_part("lift_column")
    main_tray = object_model.get_part("main_tray")
    reading_wing = object_model.get_part("reading_wing")
    release_paddle = object_model.get_part("release_paddle")
    lift = object_model.get_articulation("base_to_lift_column")
    tray_hinge = object_model.get_articulation("lift_column_to_tray")
    wing_hinge = object_model.get_articulation("tray_to_reading_wing")
    paddle_hinge = object_model.get_articulation("tray_to_release_paddle")

    ctx.check("lift column is prismatic", lift.articulation_type == ArticulationType.PRISMATIC)
    ctx.check("tray has tilt hinge", tray_hinge.articulation_type == ArticulationType.REVOLUTE)
    ctx.check("reading wing has hinge", wing_hinge.articulation_type == ArticulationType.REVOLUTE)
    ctx.check("release paddle has pivot", paddle_hinge.articulation_type == ArticulationType.REVOLUTE)
    for i in range(4):
        caster = object_model.get_part(f"caster_{i}")
        joint = object_model.get_articulation(f"base_to_caster_{i}")
        ctx.check(f"caster {i} spins continuously", joint.articulation_type == ArticulationType.CONTINUOUS)
        ctx.allow_overlap(
            base,
            caster,
            elem_a=f"axle_{i}",
            elem_b="rim",
            reason="The simplified caster hub is intentionally captured around the fixed axle.",
        )
        ctx.expect_within(
            base,
            caster,
            axes="yz",
            inner_elem=f"axle_{i}",
            outer_elem="rim",
            margin=0.0,
            name=f"caster {i} axle sits inside the hub diameter",
        )
        ctx.expect_overlap(
            caster,
            base,
            axes="x",
            elem_a="rim",
            elem_b=f"axle_{i}",
            min_overlap=0.020,
            name=f"caster {i} axle spans the hub width",
        )

    ctx.expect_within(
        lift_column,
        base,
        axes="xy",
        inner_elem="inner_mast",
        outer_elem="outer_sleeve",
        margin=0.002,
        name="inner mast stays centered in hollow sleeve",
    )
    ctx.expect_overlap(
        lift_column,
        base,
        axes="z",
        elem_a="inner_mast",
        elem_b="outer_sleeve",
        min_overlap=0.30,
        name="collapsed lift remains deeply nested",
    )
    rest_lift_pos = ctx.part_world_position(lift_column)
    with ctx.pose({lift: 0.24}):
        ctx.expect_within(
            lift_column,
            base,
            axes="xy",
            inner_elem="inner_mast",
            outer_elem="outer_sleeve",
            margin=0.002,
            name="extended mast remains centered in sleeve",
        )
        ctx.expect_overlap(
            lift_column,
            base,
            axes="z",
            elem_a="inner_mast",
            elem_b="outer_sleeve",
            min_overlap=0.18,
            name="extended lift retains sleeve insertion",
        )
        extended_lift_pos = ctx.part_world_position(lift_column)
    ctx.check(
        "pneumatic column raises the tray stack",
        rest_lift_pos is not None
        and extended_lift_pos is not None
        and extended_lift_pos[2] > rest_lift_pos[2] + 0.20,
        details=f"rest={rest_lift_pos}, extended={extended_lift_pos}",
    )

    rest_tray = ctx.part_element_world_aabb(main_tray, elem="tray_panel")
    with ctx.pose({tray_hinge: 0.60}):
        tilted_tray = ctx.part_element_world_aabb(main_tray, elem="tray_panel")
    ctx.check(
        "tray tilt hinge lifts the free edge",
        rest_tray is not None
        and tilted_tray is not None
        and tilted_tray[1][2] > rest_tray[1][2] + 0.12,
        details=f"rest={rest_tray}, tilted={tilted_tray}",
    )

    rest_wing = ctx.part_element_world_aabb(reading_wing, elem="wing_panel")
    with ctx.pose({wing_hinge: 0.80}):
        raised_wing = ctx.part_element_world_aabb(reading_wing, elem="wing_panel")
    ctx.check(
        "reading wing folds upward on its own short hinge",
        rest_wing is not None
        and raised_wing is not None
        and raised_wing[1][2] > rest_wing[1][2] + 0.12,
        details=f"rest={rest_wing}, raised={raised_wing}",
    )

    rest_paddle = ctx.part_element_world_aabb(release_paddle, elem="paddle_plate")
    with ctx.pose({paddle_hinge: 0.50}):
        pressed_paddle = ctx.part_element_world_aabb(release_paddle, elem="paddle_plate")
    ctx.check(
        "release paddle rotates downward on its pivot",
        rest_paddle is not None
        and pressed_paddle is not None
        and pressed_paddle[0][2] < rest_paddle[0][2] - 0.025,
        details=f"rest={rest_paddle}, pressed={pressed_paddle}",
    )

    return ctx.report()


object_model = build_object_model()
