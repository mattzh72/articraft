from __future__ import annotations

import math

from sdk import (
    ArticulatedObject,
    ArticulationType,
    Box,
    Cylinder,
    LatheGeometry,
    MotionLimits,
    MotionProperties,
    Origin,
    TestContext,
    TestReport,
    TorusGeometry,
    mesh_from_geometry,
)


def build_object_model() -> ArticulatedObject:
    model = ArticulatedObject(name="premium_swivel_bar_stool")

    satin_steel = model.material("satin_steel", rgba=(0.72, 0.70, 0.66, 1.0))
    warm_shadow = model.material("warm_shadow", rgba=(0.045, 0.043, 0.040, 1.0))
    matte_leather = model.material("matte_charcoal_leather", rgba=(0.06, 0.058, 0.055, 1.0))
    seam_thread = model.material("subtle_graphite_stitching", rgba=(0.18, 0.17, 0.16, 1.0))
    rubber = model.material("matte_black_rubber", rgba=(0.015, 0.014, 0.013, 1.0))

    pedestal = model.part("pedestal")

    base_profile = [
        (0.000, 0.000),
        (0.245, 0.000),
        (0.276, 0.008),
        (0.286, 0.021),
        (0.272, 0.038),
        (0.224, 0.052),
        (0.070, 0.056),
        (0.000, 0.056),
    ]
    pedestal.visual(
        mesh_from_geometry(LatheGeometry(base_profile, segments=96), "low_domed_pedestal_base"),
        material=satin_steel,
        name="low_domed_base",
    )
    pedestal.visual(
        mesh_from_geometry(TorusGeometry(radius=0.258, tube=0.007, radial_segments=18, tubular_segments=96), "recessed_rubber_foot"),
        origin=Origin(xyz=(0.0, 0.0, 0.010)),
        material=rubber,
        name="rubber_foot_ring",
    )
    pedestal.visual(
        Cylinder(radius=0.074, length=0.032),
        origin=Origin(xyz=(0.0, 0.0, 0.063)),
        material=satin_steel,
        name="base_collar",
    )
    pedestal.visual(
        mesh_from_geometry(TorusGeometry(radius=0.071, tube=0.0045, radial_segments=14, tubular_segments=72), "base_shadow_seam"),
        origin=Origin(xyz=(0.0, 0.0, 0.079)),
        material=warm_shadow,
        name="base_shadow_seam",
    )
    pedestal.visual(
        Cylinder(radius=0.037, length=0.590),
        origin=Origin(xyz=(0.0, 0.0, 0.355)),
        material=satin_steel,
        name="pedestal_column",
    )

    pedestal.visual(
        Cylinder(radius=0.055, length=0.034),
        origin=Origin(xyz=(0.0, 0.0, 0.300)),
        material=satin_steel,
        name="footrest_hub",
    )
    pedestal.visual(
        mesh_from_geometry(TorusGeometry(radius=0.182, tube=0.012, radial_segments=18, tubular_segments=112), "round_footrest_ring"),
        origin=Origin(xyz=(0.0, 0.0, 0.300)),
        material=satin_steel,
        name="footrest_ring",
    )
    for i, angle in enumerate((0.0, math.pi / 2.0, math.pi, 3.0 * math.pi / 2.0)):
        pedestal.visual(
            Box((0.132, 0.020, 0.014)),
            origin=Origin(xyz=(0.116 * math.cos(angle), 0.116 * math.sin(angle), 0.300), rpy=(0.0, 0.0, angle)),
            material=satin_steel,
            name=f"footrest_spoke_{i}",
        )

    pedestal.visual(
        Cylinder(radius=0.058, length=0.046),
        origin=Origin(xyz=(0.0, 0.0, 0.638)),
        material=satin_steel,
        name="upper_column_collar",
    )
    pedestal.visual(
        Cylinder(radius=0.108, length=0.020),
        origin=Origin(xyz=(0.0, 0.0, 0.669)),
        material=satin_steel,
        name="lower_race",
    )
    pedestal.visual(
        Cylinder(radius=0.111, length=0.004),
        origin=Origin(xyz=(0.0, 0.0, 0.679)),
        material=warm_shadow,
        name="bearing_shadow",
    )

    seat = model.part("seat")
    seat.visual(
        Cylinder(radius=0.104, length=0.018),
        origin=Origin(xyz=(0.0, 0.0, 0.010)),
        material=satin_steel,
        name="upper_race",
    )
    seat.visual(
        Cylinder(radius=0.150, length=0.024),
        origin=Origin(xyz=(0.0, 0.0, 0.030)),
        material=satin_steel,
        name="circular_support",
    )
    pan_profile = [
        (0.000, 0.035),
        (0.118, 0.035),
        (0.158, 0.044),
        (0.171, 0.055),
        (0.158, 0.067),
        (0.080, 0.072),
        (0.000, 0.071),
    ]
    seat.visual(
        mesh_from_geometry(LatheGeometry(pan_profile, segments=96), "brushed_underseat_pan"),
        material=satin_steel,
        name="underseat_pan",
    )
    cushion_profile = [
        (0.000, 0.060),
        (0.132, 0.060),
        (0.174, 0.066),
        (0.193, 0.084),
        (0.190, 0.108),
        (0.166, 0.130),
        (0.090, 0.141),
        (0.000, 0.144),
    ]
    seat.visual(
        mesh_from_geometry(LatheGeometry(cushion_profile, segments=128), "soft_rounded_leather_seat"),
        material=matte_leather,
        name="leather_cushion",
    )
    seat.visual(
        mesh_from_geometry(TorusGeometry(radius=0.190, tube=0.006, radial_segments=16, tubular_segments=128), "raised_outer_piping"),
        origin=Origin(xyz=(0.0, 0.0, 0.096)),
        material=seam_thread,
        name="outer_piping",
    )
    seat.visual(
        mesh_from_geometry(TorusGeometry(radius=0.118, tube=0.003, radial_segments=10, tubular_segments=96), "recessed_top_stitch"),
        origin=Origin(xyz=(0.0, 0.0, 0.137)),
        material=seam_thread,
        name="top_stitch_ring",
    )
    for i, angle in enumerate((0.0, math.pi / 3.0, 2.0 * math.pi / 3.0, math.pi, 4.0 * math.pi / 3.0, 5.0 * math.pi / 3.0)):
        seat.visual(
            Cylinder(radius=0.009, length=0.008),
            origin=Origin(xyz=(0.082 * math.cos(angle), 0.082 * math.sin(angle), 0.139)),
            material=seam_thread,
            name=f"tuft_button_{i}",
        )

    seat.visual(
        Box((0.050, 0.042, 0.008)),
        origin=Origin(xyz=(-0.152, 0.0, 0.042)),
        material=satin_steel,
        name="lever_mount_bridge",
    )
    seat.visual(
        Box((0.038, 0.004, 0.036)),
        origin=Origin(xyz=(-0.172, 0.016, 0.024)),
        material=satin_steel,
        name="bracket_cheek_pos",
    )
    seat.visual(
        Box((0.038, 0.004, 0.036)),
        origin=Origin(xyz=(-0.172, -0.016, 0.024)),
        material=satin_steel,
        name="bracket_cheek_neg",
    )
    seat.visual(
        Cylinder(radius=0.009, length=0.007),
        origin=Origin(xyz=(-0.172, 0.0205, 0.024), rpy=(-math.pi / 2.0, 0.0, 0.0)),
        material=warm_shadow,
        name="outer_pivot_boss_pos",
    )
    seat.visual(
        Cylinder(radius=0.009, length=0.007),
        origin=Origin(xyz=(-0.172, -0.0205, 0.024), rpy=(-math.pi / 2.0, 0.0, 0.0)),
        material=warm_shadow,
        name="outer_pivot_boss_neg",
    )

    lever = model.part("lever")
    lever.visual(
        Cylinder(radius=0.007, length=0.028),
        origin=Origin(rpy=(-math.pi / 2.0, 0.0, 0.0)),
        material=satin_steel,
        name="pivot_pin",
    )
    lever.visual(
        Box((0.128, 0.011, 0.008)),
        origin=Origin(xyz=(-0.066, 0.0, -0.008)),
        material=satin_steel,
        name="lever_arm",
    )
    lever.visual(
        Box((0.045, 0.023, 0.014)),
        origin=Origin(xyz=(-0.142, 0.0, -0.013)),
        material=warm_shadow,
        name="paddle",
    )

    model.articulation(
        "seat_swivel",
        ArticulationType.CONTINUOUS,
        parent=pedestal,
        child=seat,
        origin=Origin(xyz=(0.0, 0.0, 0.680)),
        axis=(0.0, 0.0, 1.0),
        motion_limits=MotionLimits(effort=35.0, velocity=4.0),
        motion_properties=MotionProperties(damping=0.12, friction=0.06),
    )
    model.articulation(
        "lever_pivot",
        ArticulationType.REVOLUTE,
        parent=seat,
        child=lever,
        origin=Origin(xyz=(-0.172, 0.0, 0.024)),
        axis=(0.0, 1.0, 0.0),
        motion_limits=MotionLimits(effort=8.0, velocity=1.2, lower=-0.45, upper=0.10),
    )

    return model


def run_tests() -> TestReport:
    ctx = TestContext(object_model)
    pedestal = object_model.get_part("pedestal")
    seat = object_model.get_part("seat")
    lever = object_model.get_part("lever")
    swivel = object_model.get_articulation("seat_swivel")
    lever_pivot = object_model.get_articulation("lever_pivot")

    ctx.expect_gap(
        seat,
        pedestal,
        axis="z",
        positive_elem="upper_race",
        negative_elem="bearing_shadow",
        min_gap=0.0,
        max_gap=0.001,
        name="controlled shadow gap at swivel bearing",
    )
    ctx.expect_overlap(
        seat,
        pedestal,
        axes="xy",
        elem_a="upper_race",
        elem_b="lower_race",
        min_overlap=0.16,
        name="concentric visible swivel races overlap in plan",
    )
    ctx.expect_gap(
        seat,
        lever,
        axis="y",
        positive_elem="bracket_cheek_pos",
        negative_elem="pivot_pin",
        min_gap=0.0,
        max_gap=0.001,
        name="lever pivot pin is clearanced inside yoke",
    )
    ctx.expect_overlap(
        lever,
        seat,
        axes="xz",
        elem_a="pivot_pin",
        elem_b="bracket_cheek_pos",
        min_overlap=0.012,
        name="lever pin aligns with clevis cheek",
    )

    lever_rest = ctx.part_world_position(lever)
    handle_rest = ctx.part_element_world_aabb(lever, elem="paddle")
    with ctx.pose({lever_pivot: -0.35}):
        handle_lowered = ctx.part_element_world_aabb(lever, elem="paddle")
    ctx.check(
        "height lever pulls downward",
        handle_rest is not None and handle_lowered is not None and handle_lowered[0][2] < handle_rest[0][2] - 0.015,
        details=f"rest={handle_rest}, lowered={handle_lowered}",
    )

    with ctx.pose({swivel: math.pi / 2.0}):
        lever_rotated = ctx.part_world_position(lever)
    ctx.check(
        "seat swivel carries side lever around column",
        lever_rest is not None
        and lever_rotated is not None
        and abs(lever_rotated[0] - lever_rest[0]) > 0.12
        and abs(lever_rotated[1] - lever_rest[1]) > 0.12,
        details=f"rest={lever_rest}, rotated={lever_rotated}",
    )

    return ctx.report()


object_model = build_object_model()
