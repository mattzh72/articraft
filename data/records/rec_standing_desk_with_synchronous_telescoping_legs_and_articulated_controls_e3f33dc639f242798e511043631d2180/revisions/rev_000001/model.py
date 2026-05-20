from __future__ import annotations

import math

from sdk import (
    ArticulatedObject,
    ArticulationType,
    Box,
    Cylinder,
    ExtrudeGeometry,
    Mesh,
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

    steel = model.material("satin_black_steel", rgba=(0.04, 0.045, 0.05, 1.0))
    rail = model.material("brushed_aluminum", rgba=(0.62, 0.64, 0.62, 1.0))
    wood = model.material("pale_birch_laminate", rgba=(0.86, 0.72, 0.50, 1.0))
    edge = model.material("dark_edge_band", rgba=(0.12, 0.11, 0.095, 1.0))
    matte = model.material("matte_black_plastic", rgba=(0.015, 0.016, 0.018, 1.0))
    tray_mat = model.material("charcoal_tray", rgba=(0.10, 0.105, 0.11, 1.0))
    rubber = model.material("soft_rubber", rgba=(0.006, 0.006, 0.006, 1.0))
    blue = model.material("blue_status_led", rgba=(0.1, 0.38, 0.95, 1.0))

    # Coordinate convention: +X is the user's/front edge, +Z is vertical.
    base = model.part("base_frame")

    # Two broad floor feet and a rear cross member give the standing drafting
    # desk a studio-scale footprint while keeping the lift columns tied together.
    for y in (-0.43, 0.43):
        base.visual(
            Box((0.92, 0.075, 0.070)),
            origin=Origin(xyz=(0.02, y, 0.035)),
            material=steel,
            name=f"floor_foot_{0 if y < 0 else 1}",
        )
        base.visual(
            Box((0.13, 0.090, 0.020)),
            origin=Origin(xyz=(-0.36, y, 0.010)),
            material=rubber,
            name=f"rear_leveler_{0 if y < 0 else 1}",
        )
        base.visual(
            Box((0.13, 0.090, 0.020)),
            origin=Origin(xyz=(0.40, y, 0.010)),
            material=rubber,
            name=f"front_leveler_{0 if y < 0 else 1}",
        )

    base.visual(
        Box((0.075, 0.94, 0.080)),
        origin=Origin(xyz=(-0.34, 0.0, 0.225)),
        material=steel,
        name="lower_crossbar",
    )
    base.visual(
        Box((0.084, 0.92, 0.045)),
        origin=Origin(xyz=(-0.370, 0.0, 0.62)),
        material=steel,
        name="rear_brace",
    )

    # Hollow square outer columns: four wall strips around a clear central sleeve
    # so the smaller moving stages can genuinely pass inside instead of colliding
    # with a solid placeholder column.
    column_x = -0.28
    column_bottom = 0.22
    column_height = 0.72
    wall = 0.012
    outer = 0.096
    for i, y in enumerate((-0.43, 0.43)):
        zc = column_bottom + column_height / 2.0
        base.visual(
            Box((outer, wall, column_height)),
            origin=Origin(xyz=(column_x, y - outer / 2.0 + wall / 2.0, zc)),
            material=steel,
            name=f"outer_column_{i}_side_a",
        )
        base.visual(
            Box((outer, wall, column_height)),
            origin=Origin(xyz=(column_x, y + outer / 2.0 - wall / 2.0, zc)),
            material=steel,
            name=f"outer_column_{i}_side_b",
        )
        base.visual(
            Box((wall, outer, column_height)),
            origin=Origin(xyz=(column_x - outer / 2.0 + wall / 2.0, y, zc)),
            material=steel,
            name=f"outer_column_{i}_rear_wall",
        )
        base.visual(
            Box((wall, outer, column_height)),
            origin=Origin(xyz=(column_x + outer / 2.0 - wall / 2.0, y, zc)),
            material=steel,
            name=f"outer_column_{i}_front_wall",
        )
        base.visual(
            Box((0.13, 0.13, 0.035)),
            origin=Origin(xyz=(column_x, y, column_bottom - 0.0175)),
            material=steel,
            name=f"column_foot_plate_{i}",
        )
        base.visual(
            Box((0.090, 0.090, column_bottom - 0.105)),
            origin=Origin(xyz=(column_x, y, 0.5 * (0.070 + column_bottom - 0.035))),
            material=steel,
            name=f"column_riser_{i}",
        )
        # A thin collar at the sleeve mouth reads as the bearing surface for the
        # telescoping stage without blocking the central opening.
        base.visual(
            Box((0.130, 0.020, 0.030)),
            origin=Origin(xyz=(column_x, y - 0.055, column_bottom + column_height + 0.015)),
            material=rail,
            name=f"top_collar_{i}_side_a",
        )
        base.visual(
            Box((0.130, 0.020, 0.030)),
            origin=Origin(xyz=(column_x, y + 0.055, column_bottom + column_height + 0.015)),
            material=rail,
            name=f"top_collar_{i}_side_b",
        )
        base.visual(
            Box((0.020, 0.130, 0.030)),
            origin=Origin(xyz=(column_x - 0.055, y, column_bottom + column_height + 0.015)),
            material=rail,
            name=f"top_collar_{i}_rear",
        )
        base.visual(
            Box((0.020, 0.130, 0.030)),
            origin=Origin(xyz=(column_x + 0.055, y, column_bottom + column_height + 0.015)),
            material=rail,
            name=f"top_collar_{i}_front",
        )

    # Two independently authored, mimic-coupled telescoping stages.
    stage_parts = []
    for i, y in enumerate((-0.43, 0.43)):
        stage = model.part(f"leg_stage_{i}")
        stage.visual(
            Box((0.052, 0.052, 0.500)),
            # Child frame is at the top lip of the outer sleeve.  The moving tube
            # extends downward for retained insertion and upward to carry the head.
            origin=Origin(xyz=(0.0, 0.0, -0.130)),
            material=rail,
            name="inner_tube",
        )
        stage.visual(
            Box((0.046, 0.010, 0.080)),
            origin=Origin(xyz=(0.0, -0.031, -0.200)),
            material=rubber,
            name="glide_pad_a",
        )
        stage.visual(
            Box((0.046, 0.010, 0.080)),
            origin=Origin(xyz=(0.0, 0.031, -0.200)),
            material=rubber,
            name="glide_pad_b",
        )
        stage.visual(
            Box((0.090, 0.090, 0.028)),
            origin=Origin(xyz=(0.0, 0.0, 0.134)),
            material=rail,
            name="top_cap",
        )
        stage_parts.append(stage)

    lift_0 = model.articulation(
        "base_to_leg_stage_0",
        ArticulationType.PRISMATIC,
        parent=base,
        child=stage_parts[0],
        origin=Origin(xyz=(column_x, -0.43, column_bottom + column_height)),
        axis=(0.0, 0.0, 1.0),
        motion_limits=MotionLimits(effort=220.0, velocity=0.18, lower=0.0, upper=0.240),
    )
    model.articulation(
        "base_to_leg_stage_1",
        ArticulationType.PRISMATIC,
        parent=base,
        child=stage_parts[1],
        origin=Origin(xyz=(column_x, 0.43, column_bottom + column_height)),
        axis=(0.0, 0.0, 1.0),
        motion_limits=MotionLimits(effort=220.0, velocity=0.18, lower=0.0, upper=0.240),
        mimic=Mimic(joint=lift_0.name, multiplier=1.0, offset=0.0),
    )

    hinge_beam = model.part("hinge_beam")
    hinge_beam.visual(
        Box((0.130, 0.970, 0.060)),
        origin=Origin(xyz=(0.000, 0.430, 0.094)),
        material=steel,
        name="upper_crosshead",
    )
    hinge_beam.visual(
        Cylinder(radius=0.026, length=1.050),
        origin=Origin(xyz=(-0.060, 0.430, 0.165), rpy=(math.pi / 2.0, 0.0, 0.0)),
        material=rail,
        name="round_hinge_beam",
    )
    hinge_beam.visual(
        Box((0.075, 0.880, 0.022)),
        origin=Origin(xyz=(-0.105, 0.430, 0.154)),
        material=steel,
        name="beam_mount_plate",
    )
    hinge_beam.visual(
        Box((0.040, 0.040, 0.045)),
        origin=Origin(xyz=(-0.060, -0.030, 0.123)),
        material=steel,
        name="hinge_bearing",
    )
    hinge_beam.visual(
        Box((0.060, 0.145, 0.070)),
        origin=Origin(xyz=(0.000, 0.000, 0.049)),
        material=steel,
        name="stage_socket_0",
    )
    hinge_beam.visual(
        Box((0.060, 0.145, 0.070)),
        origin=Origin(xyz=(0.000, 0.860, 0.049)),
        material=steel,
        name="stage_socket_1",
    )
    model.articulation(
        "stage_to_hinge_beam",
        ArticulationType.FIXED,
        parent=stage_parts[0],
        child=hinge_beam,
        origin=Origin(xyz=(0.0, 0.0, 0.134)),
    )

    top = model.part("work_surface")
    top_mesh = mesh_from_geometry(
        ExtrudeGeometry(
            rounded_rect_profile(0.860, 1.420, 0.045, corner_segments=10),
            0.044,
            cap=True,
            center=True,
        ),
        "rounded_work_surface",
    )
    top.visual(
        top_mesh,
        # The panel extends forward from the hinge line; its underside is just
        # above the hinge axis so positive rotation lifts the front lip.
        origin=Origin(xyz=(0.430, 0.0, 0.055)),
        material=wood,
        name="laminated_top",
    )
    top.visual(
        Box((0.042, 1.410, 0.075)),
        origin=Origin(xyz=(0.850, 0.0, 0.018)),
        material=edge,
        name="front_lip",
    )
    top.visual(
        Box((0.040, 1.360, 0.040)),
        origin=Origin(xyz=(0.060, 0.0, 0.035)),
        material=edge,
        name="rear_edge_band",
    )
    top.visual(
        Box((0.048, 1.060, 0.034)),
        origin=Origin(xyz=(0.810, 0.0, -0.035)),
        material=matte,
        name="control_strip",
    )
    top.visual(
        Box((0.010, 0.120, 0.012)),
        origin=Origin(xyz=(0.840, 0.240, -0.024)),
        material=blue,
        name="status_window",
    )
    top.visual(
        Box((0.520, 0.036, 0.044)),
        origin=Origin(xyz=(0.585, -0.432, -0.110)),
        material=rail,
        name="tray_guide_0",
    )
    top.visual(
        Box((0.060, 0.036, 0.130)),
        origin=Origin(xyz=(0.585, -0.432, -0.032)),
        material=rail,
        name="guide_bracket_0",
    )
    top.visual(
        Box((0.520, 0.036, 0.044)),
        origin=Origin(xyz=(0.585, 0.432, -0.110)),
        material=rail,
        name="tray_guide_1",
    )
    top.visual(
        Box((0.060, 0.036, 0.130)),
        origin=Origin(xyz=(0.585, 0.432, -0.032)),
        material=rail,
        name="guide_bracket_1",
    )
    top.visual(
        Box((0.100, 0.820, 0.010)),
        origin=Origin(xyz=(0.050, 0.0, -0.031)),
        material=rail,
        name="top_hinge_leaf",
    )
    top.visual(
        Box((0.025, 0.820, 0.064)),
        origin=Origin(xyz=(0.075, 0.0, 0.003)),
        material=rail,
        name="hinge_web",
    )

    model.articulation(
        "hinge_beam_to_surface",
        ArticulationType.REVOLUTE,
        parent=hinge_beam,
        child=top,
        origin=Origin(xyz=(-0.060, 0.430, 0.165)),
        axis=(0.0, -1.0, 0.0),
        motion_limits=MotionLimits(effort=70.0, velocity=0.45, lower=0.0, upper=0.87),
    )

    tray = model.part("keyboard_tray")
    tray_mesh = mesh_from_geometry(
        ExtrudeGeometry(
            rounded_rect_profile(0.500, 0.780, 0.035, corner_segments=8),
            0.030,
            cap=True,
            center=True,
        ),
        "shallow_keyboard_tray",
    )
    tray.visual(
        tray_mesh,
        origin=Origin(xyz=(0.250, 0.0, -0.018)),
        material=tray_mat,
        name="tray_pan",
    )
    tray.visual(
        Box((0.018, 0.760, 0.035)),
        origin=Origin(xyz=(0.500, 0.0, -0.006)),
        material=edge,
        name="tray_front_lip",
    )
    tray.visual(
        Box((0.460, 0.032, 0.026)),
        origin=Origin(xyz=(0.240, -0.405, 0.004)),
        material=rail,
        name="tray_runner_0",
    )
    tray.visual(
        Box((0.460, 0.032, 0.026)),
        origin=Origin(xyz=(0.240, 0.405, 0.004)),
        material=rail,
        name="tray_runner_1",
    )
    model.articulation(
        "surface_to_keyboard_tray",
        ArticulationType.PRISMATIC,
        parent=top,
        child=tray,
        origin=Origin(xyz=(0.340, 0.0, -0.149)),
        axis=(1.0, 0.0, 0.0),
        motion_limits=MotionLimits(effort=35.0, velocity=0.35, lower=0.0, upper=0.300),
    )

    # Two small tactile height buttons on the control strip.  They press inward
    # into the strip but remain proud at the default pose.
    for i, y in enumerate((-0.070, 0.070)):
        button = model.part(f"button_{i}")
        button.visual(
            Box((0.010, 0.045, 0.023)),
            origin=Origin(xyz=(0.005, 0.0, 0.0)),
            material=rubber,
            name="button_cap",
        )
        model.articulation(
            f"surface_to_button_{i}",
            ArticulationType.PRISMATIC,
            parent=top,
            child=button,
            origin=Origin(xyz=(0.834, y, -0.035)),
            axis=(-1.0, 0.0, 0.0),
            motion_limits=MotionLimits(effort=2.0, velocity=0.03, lower=0.0, upper=0.006),
        )

    return model


def run_tests() -> TestReport:
    ctx = TestContext(object_model)
    # `compile_model` automatically runs baseline sanity/QC:
    # - `check_model_valid()`
    # - exactly one root part
    # - `check_mesh_assets_ready()`
    # - disconnected floating-part-group detection
    # - disconnected within-part geometry-island detection
    # - current-pose real 3D overlap detection
    # Use `run_tests()` only for prompt-specific exact checks, targeted poses,
    # and explicit allowances such as `ctx.allow_overlap(...)`.
    # If overlap QC reports an intersection, classify it first: intentional
    # embeddings or nested fits should get a scoped allowance; unintended
    # collisions should be fixed in geometry, support, mount, or pose.

    base = object_model.get_part("base_frame")
    leg_0 = object_model.get_part("leg_stage_0")
    leg_1 = object_model.get_part("leg_stage_1")
    hinge = object_model.get_part("hinge_beam")
    top = object_model.get_part("work_surface")
    tray = object_model.get_part("keyboard_tray")

    lift = object_model.get_articulation("base_to_leg_stage_0")
    tilt = object_model.get_articulation("hinge_beam_to_surface")
    tray_slide = object_model.get_articulation("surface_to_keyboard_tray")

    # The moving tubes are designed to live inside hollow outer sleeves.  These
    # exact checks prove retained insertion and centering instead of hiding a
    # solid-column overlap with a broad allowance.
    ctx.expect_within(
        leg_0,
        base,
        axes="xy",
        inner_elem="inner_tube",
        margin=0.080,
        name="leg stage 0 centered in its column footprint",
    )
    ctx.expect_within(
        leg_1,
        base,
        axes="xy",
        inner_elem="inner_tube",
        margin=0.080,
        name="leg stage 1 centered in its column footprint",
    )
    ctx.expect_overlap(
        leg_0,
        base,
        axes="z",
        elem_a="inner_tube",
        min_overlap=0.220,
        name="collapsed stage 0 remains inserted",
    )

    rest_top = ctx.part_world_position(top)
    with ctx.pose({lift: 0.240}):
        raised_top = ctx.part_world_position(top)
        ctx.expect_overlap(
            leg_0,
            base,
            axes="z",
            elem_a="inner_tube",
            min_overlap=0.090,
            name="raised stage 0 retains insertion",
        )
    ctx.check(
        "lift raises hinge and work surface",
        rest_top is not None and raised_top is not None and raised_top[2] > rest_top[2] + 0.20,
        details=f"rest={rest_top}, raised={raised_top}",
    )

    with ctx.pose({tilt: 0.70}):
        tilted_aabb = ctx.part_world_aabb(top)
    flat_aabb = ctx.part_world_aabb(top)
    ctx.check(
        "surface tilt lifts front edge",
        flat_aabb is not None
        and tilted_aabb is not None
        and tilted_aabb[1][2] > flat_aabb[1][2] + 0.18,
        details=f"flat={flat_aabb}, tilted={tilted_aabb}",
    )

    ctx.expect_overlap(
        tray,
        top,
        axes="x",
        elem_a="tray_pan",
        elem_b="tray_guide_0",
        min_overlap=0.45,
        name="stowed tray sits under guide travel",
    )
    tray_rest = ctx.part_world_position(tray)
    with ctx.pose({tray_slide: 0.300}):
        tray_out = ctx.part_world_position(tray)
        ctx.expect_overlap(
            tray,
            top,
            axes="x",
            elem_a="tray_pan",
            elem_b="tray_guide_0",
            min_overlap=0.18,
            name="extended tray remains captured by guides",
        )
    ctx.check(
        "keyboard tray slides out toward front",
        tray_rest is not None and tray_out is not None and tray_out[0] > tray_rest[0] + 0.25,
        details=f"rest={tray_rest}, out={tray_out}",
    )

    ctx.expect_contact(
        hinge,
        top,
        elem_a="round_hinge_beam",
        elem_b="top_hinge_leaf",
        contact_tol=0.030,
        name="rear hinge beam is visibly tied to top leaf",
    )

    return ctx.report()


object_model = build_object_model()
