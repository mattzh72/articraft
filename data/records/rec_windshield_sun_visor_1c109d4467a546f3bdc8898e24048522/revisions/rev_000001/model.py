from __future__ import annotations

from math import pi

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

BODY_WIDTH = 0.340
BODY_DEPTH = 0.160
BODY_THICKNESS = 0.020

HINGE_X = 0.055
HINGE_Z = -0.010

CAVITY_START = 0.090
CAVITY_RAIL = 0.018
CAVITY_SKIN = 0.004
CAVITY_CLEARANCE = 0.0012
EXTENDER_LENGTH = 0.244
EXTENDER_TRAVEL = 0.110

CLIP_INSERTION = 0.016
CLIP_Y = 0.040
CLIP_DEPTH = 0.022
CLIP_SPAN_Y = 0.036
CLIP_FINGER_THICKNESS = 0.004
CLIP_OUTER_WALL = 0.004
CLIP_CLEARANCE = 0.001


def build_object_model() -> ArticulatedObject:
    model = ArticulatedObject(name="driver_side_sun_visor")
    cavity_depth = BODY_DEPTH - (2.0 * CAVITY_RAIL)
    cavity_height = BODY_THICKNESS - (2.0 * CAVITY_SKIN)
    extender_depth = cavity_depth - (2.0 * CAVITY_CLEARANCE)
    extender_thickness = cavity_height - (2.0 * CAVITY_CLEARANCE)

    model.material("visor_fabric", rgba=(0.73, 0.72, 0.69, 1.0))
    model.material("visor_trim", rgba=(0.62, 0.61, 0.58, 1.0))
    model.material("extender_panel", rgba=(0.56, 0.56, 0.54, 1.0))
    model.material("clip_plastic", rgba=(0.78, 0.77, 0.74, 1.0))
    model.material("rod_metal", rgba=(0.35, 0.37, 0.39, 1.0))

    roof_header = model.part("roof_header")
    roof_header.visual(
        Box((0.420, 0.060, 0.006)),
        origin=Origin(xyz=(0.190, 0.020, 0.022)),
        material="clip_plastic",
        name="headliner_strip",
    )

    roof_bracket = model.part("roof_bracket")
    roof_bracket.visual(
        Cylinder(radius=0.011, length=0.012),
        origin=Origin(xyz=(0.0, 0.0, 0.002)),
        material="visor_trim",
        name="socket",
    )
    roof_bracket.visual(
        Box((0.024, 0.030, 0.020)),
        origin=Origin(xyz=(0.0, 0.0, 0.005)),
        material="visor_trim",
        name="pedestal",
    )
    roof_bracket.visual(
        Box((0.075, 0.045, 0.006)),
        origin=Origin(xyz=(0.0, 0.0, 0.016)),
        material="clip_plastic",
        name="mount_plate",
    )

    pivot_rod = model.part("pivot_rod")
    pivot_rod.visual(
        Cylinder(radius=0.0045, length=0.008),
        origin=Origin(xyz=(0.0, 0.0, -0.009)),
        material="rod_metal",
        name="swivel_pin",
    )
    pivot_rod.visual(
        Box((0.016, 0.014, 0.012)),
        origin=Origin(xyz=(0.008, 0.0, -0.010)),
        material="rod_metal",
        name="elbow",
    )
    pivot_rod.visual(
        Cylinder(radius=0.0045, length=0.046),
        origin=Origin(xyz=(0.023, 0.0, -0.010), rpy=(0.0, pi / 2.0, 0.0)),
        material="rod_metal",
        name="hinge_rod",
    )
    pivot_rod.visual(
        Box((0.008, 0.018, 0.012)),
        origin=Origin(xyz=(0.049, 0.0, -0.010)),
        material="rod_metal",
        name="hinge_block",
    )

    visor_body = model.part("visor_body")
    visor_body.visual(
        Box((BODY_WIDTH, BODY_DEPTH, CAVITY_SKIN)),
        origin=Origin(xyz=(BODY_WIDTH / 2.0, BODY_DEPTH / 2.0, -(CAVITY_SKIN / 2.0))),
        material="visor_fabric",
        name="top_skin",
    )
    visor_body.visual(
        Box((BODY_WIDTH, BODY_DEPTH, CAVITY_SKIN)),
        origin=Origin(
            xyz=(BODY_WIDTH / 2.0, BODY_DEPTH / 2.0, -(BODY_THICKNESS - (CAVITY_SKIN / 2.0)))
        ),
        material="visor_fabric",
        name="bottom_skin",
    )
    visor_body.visual(
        Box((BODY_WIDTH, CAVITY_RAIL, cavity_height)),
        origin=Origin(
            xyz=(BODY_WIDTH / 2.0, CAVITY_RAIL / 2.0, -(CAVITY_SKIN + (cavity_height / 2.0)))
        ),
        material="visor_fabric",
        name="front_rail",
    )
    visor_body.visual(
        Box((BODY_WIDTH, CAVITY_RAIL, cavity_height)),
        origin=Origin(
            xyz=(
                BODY_WIDTH / 2.0,
                BODY_DEPTH - (CAVITY_RAIL / 2.0),
                -(CAVITY_SKIN + (cavity_height / 2.0)),
            )
        ),
        material="visor_fabric",
        name="rear_rail",
    )
    visor_body.visual(
        Box((CAVITY_START, cavity_depth, cavity_height)),
        origin=Origin(
            xyz=(CAVITY_START / 2.0, BODY_DEPTH / 2.0, -(CAVITY_SKIN + (cavity_height / 2.0)))
        ),
        material="visor_fabric",
        name="panel_shell",
    )
    visor_body.visual(
        Box((0.006, 0.030, 0.012)),
        origin=Origin(xyz=(0.001, 0.018, -0.010)),
        material="visor_trim",
        name="hinge_cap",
    )

    extender = model.part("extender")
    extender.visual(
        Box((EXTENDER_LENGTH, extender_depth, extender_thickness)),
        origin=Origin(xyz=(EXTENDER_LENGTH / 2.0, extender_depth / 2.0, -(extender_thickness / 2.0))),
        material="extender_panel",
        name="extender_panel",
    )

    side_clip = model.part("side_clip")
    side_clip.visual(
        Box((CLIP_DEPTH, CLIP_SPAN_Y, CLIP_FINGER_THICKNESS)),
        origin=Origin(xyz=(CLIP_DEPTH / 2.0, 0.0, 0.011 + (CLIP_FINGER_THICKNESS / 2.0))),
        material="clip_plastic",
        name="upper_finger",
    )
    side_clip.visual(
        Box((CLIP_DEPTH, CLIP_SPAN_Y, CLIP_FINGER_THICKNESS)),
        origin=Origin(xyz=(CLIP_DEPTH / 2.0, 0.0, -(0.011 + (CLIP_FINGER_THICKNESS / 2.0)))),
        material="clip_plastic",
        name="lower_finger",
    )
    side_clip.visual(
        Box((CLIP_OUTER_WALL, CLIP_SPAN_Y, BODY_THICKNESS + (2.0 * CLIP_CLEARANCE) + (2.0 * CLIP_FINGER_THICKNESS))),
        origin=Origin(xyz=(CLIP_DEPTH - (CLIP_OUTER_WALL / 2.0), 0.0, 0.0)),
        material="clip_plastic",
        name="outer_wall",
    )
    side_clip.visual(
        Box((0.012, 0.020, 0.028)),
        origin=Origin(xyz=(0.017, 0.0, 0.025)),
        material="clip_plastic",
        name="mount_stem",
    )

    model.articulation(
        "bracket_mount",
        ArticulationType.FIXED,
        parent=roof_header,
        child=roof_bracket,
        origin=Origin(),
    )
    model.articulation(
        "clip_mount",
        ArticulationType.FIXED,
        parent=roof_header,
        child=side_clip,
        origin=Origin(
            xyz=(
                HINGE_X + BODY_WIDTH - CLIP_INSERTION,
                CLIP_Y,
                HINGE_Z - (BODY_THICKNESS / 2.0),
            )
        ),
    )
    model.articulation(
        "side_swivel",
        ArticulationType.REVOLUTE,
        parent=roof_bracket,
        child=pivot_rod,
        origin=Origin(),
        axis=(0.0, 0.0, 1.0),
        motion_limits=MotionLimits(lower=0.0, upper=1.45, effort=6.0, velocity=2.5),
    )
    model.articulation(
        "main_hinge",
        ArticulationType.REVOLUTE,
        parent=pivot_rod,
        child=visor_body,
        origin=Origin(xyz=(HINGE_X, 0.0, HINGE_Z)),
        axis=(-1.0, 0.0, 0.0),
        motion_limits=MotionLimits(lower=0.0, upper=1.35, effort=6.0, velocity=2.5),
    )
    model.articulation(
        "extender_slide",
        ArticulationType.PRISMATIC,
        parent=visor_body,
        child=extender,
        origin=Origin(
            xyz=(
                CAVITY_START + CAVITY_CLEARANCE,
                CAVITY_RAIL + CAVITY_CLEARANCE,
                -CAVITY_SKIN,
            )
        ),
        axis=(1.0, 0.0, 0.0),
        motion_limits=MotionLimits(lower=0.0, upper=EXTENDER_TRAVEL, effort=12.0, velocity=0.18),
    )

    return model


def run_tests() -> TestReport:
    ctx = TestContext(object_model)
    roof_bracket = object_model.get_part("roof_bracket")
    visor_body = object_model.get_part("visor_body")
    extender = object_model.get_part("extender")
    side_clip = object_model.get_part("side_clip")

    side_swivel = object_model.get_articulation("side_swivel")
    main_hinge = object_model.get_articulation("main_hinge")
    extender_slide = object_model.get_articulation("extender_slide")

    hinge_limits = main_hinge.motion_limits
    swivel_limits = side_swivel.motion_limits
    slide_limits = extender_slide.motion_limits

    ctx.expect_gap(
        roof_bracket,
        visor_body,
        axis="z",
        positive_elem="mount_plate",
        min_gap=0.018,
        max_gap=0.035,
        name="roof bracket stays visually separate from visor panel",
    )
    ctx.expect_overlap(
        visor_body,
        side_clip,
        axes="xy",
        min_overlap=0.015,
        name="clip sits over the visor free end",
    )
    ctx.expect_gap(
        side_clip,
        visor_body,
        axis="z",
        positive_elem="upper_finger",
        min_gap=0.0005,
        max_gap=0.0015,
        name="clip upper finger clears the visor top surface",
    )
    ctx.expect_gap(
        visor_body,
        side_clip,
        axis="z",
        negative_elem="lower_finger",
        min_gap=0.0005,
        max_gap=0.0015,
        name="clip lower finger clears the visor bottom surface",
    )
    ctx.expect_within(
        extender,
        visor_body,
        axes="yz",
        margin=0.002,
        name="retracted extender stays nested inside the visor sleeve",
    )
    ctx.expect_overlap(
        extender,
        visor_body,
        axes="x",
        min_overlap=0.240,
        name="retracted extender remains deeply inserted",
    )

    if slide_limits is not None and slide_limits.upper is not None:
        rest_ext_pos = ctx.part_world_position(extender)
        with ctx.pose({extender_slide: slide_limits.upper}):
            ctx.expect_within(
                extender,
                visor_body,
                axes="yz",
                margin=0.002,
                name="extended extender stays centered in the visor sleeve",
            )
            ctx.expect_overlap(
                extender,
                visor_body,
                axes="x",
                min_overlap=0.135,
                name="extended extender retains insertion",
            )
            extended_ext_pos = ctx.part_world_position(extender)
        ctx.check(
            "extender slides outward",
            rest_ext_pos is not None
            and extended_ext_pos is not None
            and extended_ext_pos[0] > rest_ext_pos[0] + 0.08,
            details=f"rest={rest_ext_pos}, extended={extended_ext_pos}",
        )

    if hinge_limits is not None and hinge_limits.upper is not None:
        rest_visor_aabb = ctx.part_world_aabb(visor_body)
        with ctx.pose({main_hinge: hinge_limits.upper}):
            dropped_visor_aabb = ctx.part_world_aabb(visor_body)
        ctx.check(
            "visor rotates downward from the roof",
            rest_visor_aabb is not None
            and dropped_visor_aabb is not None
            and dropped_visor_aabb[0][2] < rest_visor_aabb[0][2] - 0.06,
            details=f"rest={rest_visor_aabb}, dropped={dropped_visor_aabb}",
        )

    if swivel_limits is not None and swivel_limits.upper is not None:
        rest_visor_pos = ctx.part_world_position(visor_body)
        with ctx.pose({side_swivel: swivel_limits.upper}):
            side_visor_pos = ctx.part_world_position(visor_body)
        ctx.check(
            "visor pivots toward the side window",
            rest_visor_pos is not None
            and side_visor_pos is not None
            and side_visor_pos[1] > rest_visor_pos[1] + 0.04,
            details=f"rest={rest_visor_pos}, side={side_visor_pos}",
        )

    return ctx.report()


object_model = build_object_model()
