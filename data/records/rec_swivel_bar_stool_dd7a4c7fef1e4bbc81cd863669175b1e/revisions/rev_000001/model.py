from __future__ import annotations

import math

from sdk import (
    ArticulatedObject,
    ArticulationType,
    Box,
    Cylinder,
    ExtrudeGeometry,
    LatheGeometry,
    Material,
    MotionLimits,
    MotionProperties,
    Origin,
    TestContext,
    TestReport,
    TorusGeometry,
    mesh_from_geometry,
    rounded_rect_profile,
)


def build_object_model() -> ArticulatedObject:
    model = ArticulatedObject(name="compact_swivel_bar_stool")

    brushed_steel = model.material("brushed_steel", rgba=(0.72, 0.73, 0.70, 1.0))
    dark_metal = model.material("dark_metal", rgba=(0.055, 0.060, 0.065, 1.0))
    black_rubber = model.material("black_rubber", rgba=(0.015, 0.014, 0.013, 1.0))
    graphite = model.material("graphite", rgba=(0.12, 0.13, 0.14, 1.0))
    warm_cushion = model.material("warm_cushion", rgba=(0.58, 0.39, 0.22, 1.0))

    # One compact pedestal root: a weighted floor saucer, non-slip ring,
    # hollow outer gas-lift sleeve, and fixed lower race for the height stage.
    pedestal = model.part("pedestal")
    pedestal.visual(
        Cylinder(radius=0.185, length=0.024),
        origin=Origin(xyz=(0.0, 0.0, 0.012)),
        material=dark_metal,
        name="weighted_base",
    )
    pedestal.visual(
        mesh_from_geometry(TorusGeometry(radius=0.148, tube=0.0075, radial_segments=18, tubular_segments=56), "rubber_floor_ring"),
        origin=Origin(xyz=(0.0, 0.0, 0.007)),
        material=black_rubber,
        name="rubber_floor_ring",
    )
    pedestal.visual(
        mesh_from_geometry(
            LatheGeometry([(0.027, 0.022), (0.055, 0.022), (0.055, 0.052), (0.027, 0.052)], segments=64, closed=True),
            "base_collar_ring",
        ),
        material=graphite,
        name="base_collar",
    )
    sleeve_profile = [
        (0.030, 0.020),
        (0.039, 0.020),
        (0.039, 0.356),
        (0.030, 0.356),
    ]
    pedestal.visual(
        mesh_from_geometry(LatheGeometry(sleeve_profile, segments=64, closed=True), "outer_sleeve"),
        material=brushed_steel,
        name="outer_sleeve",
    )
    pedestal.visual(
        mesh_from_geometry(
            LatheGeometry([(0.0215, 0.052), (0.030, 0.052), (0.030, 0.070), (0.0215, 0.070)], segments=64, closed=True),
            "lower_bushing",
        ),
        material=black_rubber,
        name="lower_bushing",
    )
    pedestal.visual(
        mesh_from_geometry(
            LatheGeometry([(0.0215, 0.338), (0.030, 0.338), (0.030, 0.356), (0.0215, 0.356)], segments=64, closed=True),
            "top_bushing",
        ),
        material=black_rubber,
        name="top_bushing",
    )
    pedestal.visual(
        mesh_from_geometry(
            LatheGeometry([(0.028, 0.347), (0.050, 0.347), (0.050, 0.365), (0.028, 0.365)], segments=64, closed=True),
            "height_lock_collar",
        ),
        material=graphite,
        name="height_lock_collar",
    )

    # Sliding gas-lift mast.  Its hidden lower length remains captured in the
    # hollow sleeve at the full height limit.
    mast = model.part("mast")
    mast.visual(
        Cylinder(radius=0.022, length=0.410),
        origin=Origin(xyz=(0.0, 0.0, -0.105)),
        material=brushed_steel,
        name="gas_lift",
    )
    mast.visual(
        Cylinder(radius=0.043, length=0.020),
        origin=Origin(xyz=(0.0, 0.0, 0.095)),
        material=graphite,
        name="upper_collar",
    )
    mast.visual(
        Cylinder(radius=0.068, length=0.017),
        origin=Origin(xyz=(0.0, 0.0, 0.1135)),
        material=dark_metal,
        name="lower_race",
    )

    lift = model.articulation(
        "pedestal_to_mast",
        ArticulationType.PRISMATIC,
        parent=pedestal,
        child=mast,
        origin=Origin(xyz=(0.0, 0.0, 0.350)),
        axis=(0.0, 0.0, 1.0),
        motion_limits=MotionLimits(effort=320.0, velocity=0.16, lower=0.0, upper=0.110),
        motion_properties=MotionProperties(damping=6.0, friction=1.0),
    )
    lift.meta["description"] = "Short gas-lift travel keeps the apartment stool compact while preserving height adjustment."

    # Rotating seat stage.  The part frame sits on the center of the supported
    # bearing stack; all seat geometry rotates about this exact vertical axis.
    seat_stage = model.part("seat_stage")
    seat_stage.visual(
        Cylinder(radius=0.066, length=0.012),
        origin=Origin(xyz=(0.0, 0.0, 0.006)),
        material=dark_metal,
        name="upper_race",
    )
    seat_stage.visual(
        Cylinder(radius=0.043, length=0.018),
        origin=Origin(xyz=(0.0, 0.0, 0.021)),
        material=brushed_steel,
        name="center_spacer",
    )
    seat_stage.visual(
        Cylinder(radius=0.126, length=0.012),
        origin=Origin(xyz=(0.0, 0.0, 0.033)),
        material=graphite,
        name="under_plate",
    )
    for angle, name in (
        (0.0, "rib_0"),
        (math.pi / 2.0, "rib_1"),
        (math.pi, "rib_2"),
        (3.0 * math.pi / 2.0, "rib_3"),
    ):
        seat_stage.visual(
            Box((0.115, 0.018, 0.014)),
            origin=Origin(
                xyz=(0.055 * math.cos(angle), 0.055 * math.sin(angle), 0.039),
                rpy=(0.0, 0.0, angle),
            ),
            material=graphite,
            name=name,
        )
    cushion_profile = [
        (0.000, 0.000),
        (0.145, 0.000),
        (0.176, 0.012),
        (0.180, 0.035),
        (0.160, 0.053),
        (0.055, 0.061),
        (0.000, 0.056),
    ]
    seat_stage.visual(
        mesh_from_geometry(LatheGeometry(cushion_profile, segments=72, closed=True), "rounded_seat_cushion"),
        origin=Origin(xyz=(0.0, 0.0, 0.039)),
        material=warm_cushion,
        name="seat_cushion",
    )
    # Rear yoke ears hold the fold-flat backrest hinge clear of the cushion.
    for x, name in ((-0.135, "rear_arm_0"), (0.135, "rear_arm_1")):
        seat_stage.visual(
            Box((0.024, 0.108, 0.016)),
            origin=Origin(xyz=(x, -0.140, 0.055)),
            material=graphite,
            name=name,
        )
    for x, name in ((-0.149, "rear_lug_0"), (0.149, "rear_lug_1")):
        seat_stage.visual(
            Box((0.024, 0.024, 0.052)),
            origin=Origin(xyz=(x * 0.933, -0.190, 0.080)),
            material=graphite,
            name=name,
        )

    swivel = model.articulation(
        "mast_to_seat_stage",
        ArticulationType.CONTINUOUS,
        parent=mast,
        child=seat_stage,
        origin=Origin(xyz=(0.0, 0.0, 0.122)),
        axis=(0.0, 0.0, 1.0),
        motion_limits=MotionLimits(effort=25.0, velocity=2.4),
        motion_properties=MotionProperties(damping=0.7, friction=0.25),
    )
    swivel.meta["description"] = "Centered swivel bearing; the round seat, yoke, and folded back rotate as one stage."

    # Low back folds forward over the seat to reduce storage envelope.
    backrest = model.part("backrest")
    backrest.visual(
        Cylinder(radius=0.010, length=0.255),
        origin=Origin(rpy=(0.0, math.pi / 2.0, 0.0)),
        material=dark_metal,
        name="hinge_barrel",
    )
    for x, name in ((-0.105, "back_post_0"), (0.105, "back_post_1")):
        backrest.visual(
            Box((0.018, 0.018, 0.092)),
            origin=Origin(xyz=(x, -0.006, 0.049)),
            material=graphite,
            name=name,
        )
    back_pad_profile = rounded_rect_profile(0.305, 0.120, 0.026, corner_segments=8)
    backrest.visual(
        mesh_from_geometry(ExtrudeGeometry.centered(back_pad_profile, 0.028), "rounded_back_pad"),
        origin=Origin(xyz=(0.0, -0.026, 0.145), rpy=(math.pi / 2.0, 0.0, 0.0)),
        material=warm_cushion,
        name="back_pad",
    )
    back_hinge = model.articulation(
        "seat_stage_to_backrest",
        ArticulationType.REVOLUTE,
        parent=seat_stage,
        child=backrest,
        origin=Origin(xyz=(0.0, -0.190, 0.093)),
        axis=(1.0, 0.0, 0.0),
        motion_limits=MotionLimits(effort=35.0, velocity=1.2, lower=-1.50, upper=0.0),
        motion_properties=MotionProperties(damping=1.4, friction=0.8),
    )
    back_hinge.meta["description"] = "Negative travel folds the backrest forward over the seat with clearance."

    return model


def run_tests() -> TestReport:
    ctx = TestContext(object_model)
    pedestal = object_model.get_part("pedestal")
    mast = object_model.get_part("mast")
    seat_stage = object_model.get_part("seat_stage")
    backrest = object_model.get_part("backrest")
    lift = object_model.get_articulation("pedestal_to_mast")
    swivel = object_model.get_articulation("mast_to_seat_stage")
    back_hinge = object_model.get_articulation("seat_stage_to_backrest")

    ctx.allow_overlap(
        pedestal,
        mast,
        elem_a="lower_bushing",
        elem_b="gas_lift",
        reason="The low-friction guide bushing is modeled with a tiny interference fit around the sliding gas-lift tube.",
    )
    ctx.allow_overlap(
        pedestal,
        mast,
        elem_a="top_bushing",
        elem_b="gas_lift",
        reason="The upper guide bushing intentionally kisses the gas-lift tube so the telescoping stage is visibly supported.",
    )

    ctx.expect_overlap(
        mast,
        pedestal,
        axes="z",
        elem_a="gas_lift",
        elem_b="lower_bushing",
        min_overlap=0.015,
        name="lower bushing captures gas lift",
    )
    ctx.expect_overlap(
        mast,
        pedestal,
        axes="z",
        elem_a="gas_lift",
        elem_b="top_bushing",
        min_overlap=0.015,
        name="top bushing supports gas lift",
    )
    ctx.expect_within(
        mast,
        pedestal,
        axes="xy",
        inner_elem="gas_lift",
        outer_elem="outer_sleeve",
        margin=0.004,
        name="mast centered in hollow sleeve",
    )
    ctx.expect_overlap(
        mast,
        pedestal,
        axes="z",
        elem_a="gas_lift",
        elem_b="outer_sleeve",
        min_overlap=0.260,
        name="collapsed mast remains captured",
    )
    ctx.expect_within(
        seat_stage,
        mast,
        axes="xy",
        inner_elem="upper_race",
        outer_elem="lower_race",
        margin=0.003,
        name="swivel race centered on support",
    )
    ctx.expect_gap(
        seat_stage,
        mast,
        axis="z",
        positive_elem="upper_race",
        negative_elem="lower_race",
        max_gap=0.001,
        max_penetration=0.0,
        name="swivel races seat without penetration",
    )

    rest_seat = ctx.part_world_position(seat_stage)
    with ctx.pose({lift: 0.110}):
        ctx.expect_within(
            mast,
            pedestal,
            axes="xy",
            inner_elem="gas_lift",
            outer_elem="outer_sleeve",
            margin=0.004,
            name="extended mast stays centered",
        )
        ctx.expect_overlap(
            mast,
            pedestal,
            axes="z",
            elem_a="gas_lift",
            elem_b="outer_sleeve",
            min_overlap=0.150,
            name="extended mast preserves retained insertion",
        )
        raised_seat = ctx.part_world_position(seat_stage)
    ctx.check(
        "height stage raises seat",
        rest_seat is not None and raised_seat is not None and raised_seat[2] > rest_seat[2] + 0.095,
        details=f"rest={rest_seat}, raised={raised_seat}",
    )

    with ctx.pose({swivel: math.pi / 2.0}):
        ctx.expect_within(
            seat_stage,
            mast,
            axes="xy",
            inner_elem="upper_race",
            outer_elem="lower_race",
            margin=0.003,
            name="swivel remains centered at quarter turn",
        )

    with ctx.pose({back_hinge: -1.50}):
        ctx.expect_gap(
            backrest,
            seat_stage,
            axis="z",
            positive_elem="back_pad",
            negative_elem="seat_cushion",
            min_gap=0.010,
            name="folded back clears cushion",
        )
        ctx.expect_overlap(
            backrest,
            seat_stage,
            axes="xy",
            elem_a="back_pad",
            elem_b="seat_cushion",
            min_overlap=0.060,
            name="folded back nests over seat footprint",
        )

    return ctx.report()


object_model = build_object_model()
