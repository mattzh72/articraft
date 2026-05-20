from __future__ import annotations

import math

from sdk import (
    ArticulatedObject,
    ArticulationType,
    Box,
    Cylinder,
    KnobBore,
    KnobGeometry,
    KnobGrip,
    KnobRelief,
    Material,
    MotionLimits,
    Origin,
    TestContext,
    TestReport,
    mesh_from_geometry,
)


def build_object_model() -> ArticulatedObject:
    model = ArticulatedObject(name="professional_drafting_table")

    dark_steel = model.material("powder_coated_steel", color=(0.08, 0.09, 0.10, 1.0))
    brushed_aluminum = model.material("brushed_aluminum", color=(0.64, 0.66, 0.64, 1.0))
    board_surface = model.material("matte_drafting_surface", color=(0.88, 0.92, 0.87, 1.0))
    edge_trim = model.material("anodized_edge_trim", color=(0.18, 0.20, 0.21, 1.0))
    black_plastic = model.material("black_lobed_plastic", color=(0.015, 0.015, 0.016, 1.0))
    satin_metal = model.material("satin_threaded_steel", color=(0.55, 0.56, 0.54, 1.0))

    # Root: welded steel pedestal with broad floor footprint and a vertical mast.
    base = model.part("base")
    base.visual(
        Box((1.50, 0.08, 0.08)),
        origin=Origin(xyz=(0.0, -0.36, 0.04)),
        material=dark_steel,
        name="rear_cross_tube",
    )
    base.visual(
        Box((1.20, 0.07, 0.07)),
        origin=Origin(xyz=(0.0, 0.44, 0.035)),
        material=dark_steel,
        name="front_cross_tube",
    )
    for x, name in ((-0.56, "floor_rail_0"), (0.56, "floor_rail_1")):
        base.visual(
            Box((0.09, 0.96, 0.07)),
            origin=Origin(xyz=(x, 0.04, 0.035)),
            material=dark_steel,
            name=name,
        )
    base.visual(
        Box((0.11, 0.09, 1.36)),
        origin=Origin(xyz=(0.0, -0.30, 0.72)),
        material=dark_steel,
        name="vertical_support",
    )
    base.visual(
        Box((0.18, 0.11, 0.10)),
        origin=Origin(xyz=(0.0, -0.30, 0.10)),
        material=dark_steel,
        name="column_foot_weldment",
    )
    base.visual(
        Box((0.055, 0.055, 0.70)),
        origin=Origin(xyz=(0.0, -0.08, 0.38), rpy=(0.62, 0.0, 0.0)),
        material=dark_steel,
        name="front_diagonal_brace",
    )
    base.visual(
        Box((0.055, 0.055, 0.62)),
        origin=Origin(xyz=(0.0, -0.50, 0.34), rpy=(-0.58, 0.0, 0.0)),
        material=dark_steel,
        name="rear_diagonal_brace",
    )

    # Height carriage: a hollow four-sided sleeve around the mast, with a welded
    # tilt head clevis projecting forward from it.
    carriage = model.part("carriage")
    carriage.visual(
        Box((0.018, 0.134, 0.30)),
        origin=Origin(xyz=(-0.067, 0.0, 0.0)),
        material=brushed_aluminum,
        name="sleeve_side_0",
    )
    carriage.visual(
        Box((0.018, 0.134, 0.30)),
        origin=Origin(xyz=(0.067, 0.0, 0.0)),
        material=brushed_aluminum,
        name="sleeve_side_1",
    )
    carriage.visual(
        Box((0.152, 0.018, 0.30)),
        origin=Origin(xyz=(0.0, 0.058, 0.0)),
        material=brushed_aluminum,
        name="sleeve_front",
    )
    carriage.visual(
        Box((0.152, 0.018, 0.30)),
        origin=Origin(xyz=(0.0, -0.058, 0.0)),
        material=brushed_aluminum,
        name="sleeve_rear",
    )
    carriage.visual(
        Box((0.005, 0.050, 0.25)),
        origin=Origin(xyz=(-0.0575, 0.0, 0.0)),
        material=satin_metal,
        name="guide_pad_0",
    )
    carriage.visual(
        Box((0.005, 0.050, 0.25)),
        origin=Origin(xyz=(0.0575, 0.0, 0.0)),
        material=satin_metal,
        name="guide_pad_1",
    )
    carriage.visual(
        Box((0.20, 0.24, 0.065)),
        origin=Origin(xyz=(0.0, 0.185, -0.005)),
        material=brushed_aluminum,
        name="head_neck",
    )
    carriage.visual(
        Box((0.44, 0.055, 0.055)),
        origin=Origin(xyz=(0.0, 0.235, 0.005)),
        material=brushed_aluminum,
        name="lower_yoke_bridge",
    )
    carriage.visual(
        Box((0.035, 0.12, 0.16)),
        origin=Origin(xyz=(-0.205, 0.235, 0.070)),
        material=brushed_aluminum,
        name="tilt_cheek_0",
    )
    carriage.visual(
        Box((0.035, 0.12, 0.16)),
        origin=Origin(xyz=(0.205, 0.235, 0.070)),
        material=brushed_aluminum,
        name="tilt_cheek_1",
    )

    # Main drawing board.  The part frame is the tilt hinge line; visual geometry
    # is built on a modest drafting angle so the rest pose already reads as a
    # working table, and the revolute joint adds further tilt.
    main_board = model.part("main_board")
    rest_tilt = math.radians(12.0)

    def board_xyz(x: float, y: float, z: float = 0.0) -> tuple[float, float, float]:
        return (
            x,
            y * math.cos(rest_tilt) - z * math.sin(rest_tilt),
            y * math.sin(rest_tilt) + z * math.cos(rest_tilt),
        )

    def board_origin(x: float, y: float, z: float = 0.0) -> Origin:
        return Origin(xyz=board_xyz(x, y, z), rpy=(rest_tilt, 0.0, 0.0))

    main_board.visual(
        Box((1.20, 0.76, 0.026)),
        origin=board_origin(0.0, 0.515, 0.0),
        material=board_surface,
        name="drawing_panel",
    )
    main_board.visual(
        Box((1.30, 0.060, 0.045)),
        origin=board_origin(0.0, 0.095, 0.0),
        material=edge_trim,
        name="rear_frame",
    )
    main_board.visual(
        Box((1.30, 0.045, 0.045)),
        origin=board_origin(0.0, 0.915, 0.0),
        material=edge_trim,
        name="front_frame",
    )
    for x, name in ((-0.63, "side_frame_0"), (0.63, "side_frame_1")):
        main_board.visual(
            Box((0.045, 0.86, 0.045)),
            origin=board_origin(x, 0.505, 0.0),
            material=edge_trim,
            name=name,
        )
    main_board.visual(
        Box((1.05, 0.024, 0.008)),
        origin=board_origin(-0.06, 0.735, 0.017),
        material=brushed_aluminum,
        name="parallel_rule",
    )
    main_board.visual(
        Cylinder(radius=0.025, length=0.34),
        origin=Origin(xyz=board_xyz(0.0, 0.0, 0.0), rpy=(0.0, math.pi / 2.0, 0.0)),
        material=satin_metal,
        name="tilt_hinge_barrel",
    )
    main_board.visual(
        Box((0.34, 0.080, 0.030)),
        origin=board_origin(0.0, 0.052, -0.010),
        material=edge_trim,
        name="hinge_mount_plate",
    )
    main_board.visual(
        Box((0.034, 0.64, 0.018)),
        origin=board_origin(0.645, 0.505, -0.020),
        material=brushed_aluminum,
        name="wing_hinge_leaf",
    )
    main_board.visual(
        Cylinder(radius=0.008, length=0.64),
        origin=Origin(
            xyz=board_xyz(0.654, 0.505, -0.006),
            rpy=(rest_tilt - math.pi / 2.0, 0.0, 0.0),
        ),
        material=satin_metal,
        name="wing_hinge_pin",
    )

    # Smaller hinged reference wing, carried at the outer edge of the main frame.
    reference_wing = model.part("reference_wing")
    reference_wing.visual(
        Box((0.32, 0.58, 0.022)),
        origin=Origin(xyz=(0.215, 0.0, 0.0)),
        material=board_surface,
        name="reference_panel",
    )
    reference_wing.visual(
        Box((0.38, 0.040, 0.040)),
        origin=Origin(xyz=(0.215, -0.31, 0.0)),
        material=edge_trim,
        name="wing_rear_frame",
    )
    reference_wing.visual(
        Box((0.38, 0.040, 0.040)),
        origin=Origin(xyz=(0.215, 0.31, 0.0)),
        material=edge_trim,
        name="wing_front_frame",
    )
    reference_wing.visual(
        Box((0.040, 0.62, 0.040)),
        origin=Origin(xyz=(0.055, 0.0, 0.0)),
        material=edge_trim,
        name="wing_inner_frame",
    )
    reference_wing.visual(
        Box((0.040, 0.62, 0.040)),
        origin=Origin(xyz=(0.395, 0.0, 0.0)),
        material=edge_trim,
        name="wing_outer_frame",
    )
    reference_wing.visual(
        Box((0.040, 0.60, 0.012)),
        origin=Origin(xyz=(0.017, 0.0, -0.020)),
        material=brushed_aluminum,
        name="wing_hinge_leaf",
    )

    # Separate continuous clamp knob on a short visible threaded shaft.
    clamp_knob = model.part("clamp_knob")
    knob_mesh = mesh_from_geometry(
        KnobGeometry(
            0.105,
            0.042,
            body_style="lobed",
            base_diameter=0.070,
            top_diameter=0.090,
            crown_radius=0.002,
            grip=KnobGrip(style="ribbed", count=8, depth=0.002),
            bore=KnobBore(style="round", diameter=0.016),
            body_reliefs=(KnobRelief(style="top_recess", width=0.040, depth=0.003),),
        ),
        "lobed_clamp_knob",
    )
    clamp_knob.visual(
        Cylinder(radius=0.010, length=0.024),
        origin=Origin(xyz=(0.012, 0.0, 0.0), rpy=(0.0, math.pi / 2.0, 0.0)),
        material=satin_metal,
        name="threaded_stub",
    )
    clamp_knob.visual(
        knob_mesh,
        origin=Origin(xyz=(0.045, 0.0, 0.0), rpy=(0.0, math.pi / 2.0, 0.0)),
        material=black_plastic,
        name="lobed_knob",
    )

    model.articulation(
        "height_slide",
        ArticulationType.PRISMATIC,
        parent=base,
        child=carriage,
        origin=Origin(xyz=(0.0, -0.30, 0.82)),
        axis=(0.0, 0.0, 1.0),
        motion_limits=MotionLimits(effort=180.0, velocity=0.18, lower=0.0, upper=0.35),
    )
    model.articulation(
        "tilt_hinge",
        ArticulationType.REVOLUTE,
        parent=carriage,
        child=main_board,
        origin=Origin(xyz=(0.0, 0.235, 0.075)),
        axis=(1.0, 0.0, 0.0),
        motion_limits=MotionLimits(effort=80.0, velocity=0.9, lower=-0.25, upper=0.95),
    )
    model.articulation(
        "reference_hinge",
        ArticulationType.REVOLUTE,
        parent=main_board,
        child=reference_wing,
        origin=Origin(xyz=board_xyz(0.665, 0.505, -0.006), rpy=(rest_tilt, 0.0, 0.0)),
        axis=(0.0, -1.0, 0.0),
        motion_limits=MotionLimits(effort=8.0, velocity=1.4, lower=0.0, upper=1.25),
    )
    model.articulation(
        "knob_spin",
        ArticulationType.CONTINUOUS,
        parent=carriage,
        child=clamp_knob,
        origin=Origin(xyz=(0.2225, 0.235, 0.075)),
        axis=(1.0, 0.0, 0.0),
        motion_limits=MotionLimits(effort=4.0, velocity=7.0),
    )

    return model


def run_tests() -> TestReport:
    ctx = TestContext(object_model)
    base = object_model.get_part("base")
    carriage = object_model.get_part("carriage")
    main_board = object_model.get_part("main_board")
    reference_wing = object_model.get_part("reference_wing")
    clamp_knob = object_model.get_part("clamp_knob")
    height_slide = object_model.get_articulation("height_slide")
    tilt_hinge = object_model.get_articulation("tilt_hinge")
    reference_hinge = object_model.get_articulation("reference_hinge")
    knob_spin = object_model.get_articulation("knob_spin")

    ctx.expect_overlap(
        carriage,
        base,
        axes="z",
        min_overlap=0.25,
        elem_a="sleeve_front",
        elem_b="vertical_support",
        name="carriage sleeve remains engaged on the support",
    )
    ctx.expect_gap(
        clamp_knob,
        carriage,
        axis="x",
        min_gap=0.0,
        max_gap=0.010,
        positive_elem="threaded_stub",
        negative_elem="tilt_cheek_1",
        name="knob threaded stub seats against but is separate from the head cheek",
    )
    ctx.expect_gap(
        reference_wing,
        main_board,
        axis="x",
        min_gap=0.0,
        max_gap=0.020,
        positive_elem="wing_hinge_leaf",
        negative_elem="wing_hinge_pin",
        name="reference wing hinge is carried at the main board edge",
    )

    lower_carriage = ctx.part_world_position(carriage)
    with ctx.pose({height_slide: 0.35}):
        raised_carriage = ctx.part_world_position(carriage)
        ctx.expect_overlap(
            carriage,
            base,
            axes="z",
            min_overlap=0.24,
            elem_a="sleeve_front",
            elem_b="vertical_support",
            name="raised carriage still overlaps the vertical support length",
        )
    ctx.check(
        "height slide raises the carriage",
        lower_carriage is not None
        and raised_carriage is not None
        and raised_carriage[2] > lower_carriage[2] + 0.30,
        details=f"lower={lower_carriage}, raised={raised_carriage}",
    )

    rest_board_aabb = ctx.part_world_aabb(main_board)
    with ctx.pose({tilt_hinge: 0.65}):
        tilted_board_aabb = ctx.part_world_aabb(main_board)
    ctx.check(
        "tilt hinge lifts the front of the drawing board",
        rest_board_aabb is not None
        and tilted_board_aabb is not None
        and (tilted_board_aabb[1][2] + tilted_board_aabb[0][2])
        > (rest_board_aabb[1][2] + rest_board_aabb[0][2]) + 0.12,
        details=f"rest={rest_board_aabb}, tilted={tilted_board_aabb}",
    )

    rest_wing_aabb = ctx.part_world_aabb(reference_wing)
    with ctx.pose({reference_hinge: 0.9}):
        lifted_wing_aabb = ctx.part_world_aabb(reference_wing)
    ctx.check(
        "reference hinge folds the small wing upward",
        rest_wing_aabb is not None
        and lifted_wing_aabb is not None
        and (lifted_wing_aabb[1][2] + lifted_wing_aabb[0][2])
        > (rest_wing_aabb[1][2] + rest_wing_aabb[0][2]) + 0.09,
        details=f"rest={rest_wing_aabb}, lifted={lifted_wing_aabb}",
    )

    ctx.check(
        "clamp knob uses a continuous threaded-axis joint",
        knob_spin.articulation_type == ArticulationType.CONTINUOUS,
        details=f"type={knob_spin.articulation_type}",
    )

    return ctx.report()


object_model = build_object_model()
