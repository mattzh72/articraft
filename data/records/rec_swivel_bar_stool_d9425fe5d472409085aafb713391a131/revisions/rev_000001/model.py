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
    Sphere,
    TestContext,
    TestReport,
    TorusGeometry,
    mesh_from_geometry,
)


def _annular_mesh(
    inner_radius: float,
    outer_radius: float,
    height: float,
    name: str,
    *,
    segments: int = 96,
):
    """Flat annular solid, centered on local Z."""

    h = height * 0.5
    profile = [
        (inner_radius, -h),
        (outer_radius, -h),
        (outer_radius, h),
        (inner_radius, h),
    ]
    return mesh_from_geometry(LatheGeometry(profile, segments=segments), name)


def build_object_model() -> ArticulatedObject:
    model = ArticulatedObject(name="precision_swivel_bar_stool")

    brushed = model.material("brushed_steel", rgba=(0.58, 0.60, 0.60, 1.0))
    dark = model.material("matte_graphite", rgba=(0.055, 0.058, 0.062, 1.0))
    black = model.material("engraved_black", rgba=(0.0, 0.0, 0.0, 1.0))
    red = model.material("datum_red", rgba=(0.90, 0.04, 0.02, 1.0))
    white = model.material("etched_white", rgba=(0.96, 0.96, 0.90, 1.0))
    brass = model.material("hardened_brass", rgba=(0.86, 0.62, 0.22, 1.0))

    pedestal = model.part("pedestal")
    pedestal.visual(
        Cylinder(radius=0.285, length=0.035),
        origin=Origin(xyz=(0.0, 0.0, 0.0175)),
        material=brushed,
        name="floor_base",
    )
    pedestal.visual(
        Cylinder(radius=0.043, length=0.550),
        origin=Origin(xyz=(0.0, 0.0, 0.310)),
        material=brushed,
        name="pedestal_column",
    )

    # Foot rail is tied back to the central column with four welded spokes so it
    # reads as a connected pedestal, not as a floating hoop.
    pedestal.visual(
        mesh_from_geometry(TorusGeometry(radius=0.205, tube=0.011), "foot_rail"),
        origin=Origin(xyz=(0.0, 0.0, 0.310)),
        material=brushed,
        name="foot_rail",
    )
    for i in range(4):
        angle = i * math.pi * 0.5
        pedestal.visual(
            Box((0.185, 0.018, 0.014)),
            origin=Origin(
                xyz=(0.115 * math.cos(angle), 0.115 * math.sin(angle), 0.310),
                rpy=(0.0, 0.0, angle),
            ),
            material=brushed,
            name=f"foot_spoke_{i}",
        )

    # Hollow fixed bearing sleeve and index ring form the supported reference
    # stack for the centered swivel axis.
    pedestal.visual(
        _annular_mesh(0.034, 0.070, 0.065, "bearing_sleeve"),
        origin=Origin(xyz=(0.0, 0.0, 0.6175)),
        material=brushed,
        name="bearing_sleeve",
    )
    pedestal.visual(
        _annular_mesh(0.068, 0.170, 0.010, "index_ring"),
        origin=Origin(xyz=(0.0, 0.0, 0.645)),
        material=brushed,
        name="index_ring",
    )
    pedestal.visual(
        mesh_from_geometry(TorusGeometry(radius=0.058, tube=0.005), "lower_race"),
        origin=Origin(xyz=(0.0, 0.0, 0.654)),
        material=brushed,
        name="lower_race",
    )
    for i in range(16):
        angle = i * math.tau / 16.0
        pedestal.visual(
            Sphere(radius=0.0055),
            origin=Origin(
                xyz=(0.058 * math.cos(angle), 0.058 * math.sin(angle), 0.6595)
            ),
            material=brushed,
            name=f"bearing_ball_{i:02d}",
        )

    # Painted/engraved index ticks sit on the fixed dial face.
    for i in range(36):
        angle = i * math.tau / 36.0
        major = i % 3 == 0
        length = 0.038 if major else 0.025
        width = 0.0048 if major else 0.0026
        radius = 0.145 if major else 0.151
        pedestal.visual(
            Box((length, width, 0.0008)),
            origin=Origin(
                xyz=(radius * math.cos(angle), radius * math.sin(angle), 0.6504),
                rpy=(0.0, 0.0, angle),
            ),
            material=black,
            name=f"index_tick_{i:02d}",
        )

    # Side clamp bracket and guide collar for a repeatable index lock knob.
    pedestal.visual(
        Box((0.055, 0.070, 0.045)),
        origin=Origin(xyz=(0.1975, 0.0, 0.6575)),
        material=brushed,
        name="clamp_bracket",
    )
    pedestal.visual(
        Cylinder(radius=0.018, length=0.028),
        origin=Origin(xyz=(0.230, 0.0, 0.657), rpy=(0.0, math.pi / 2.0, 0.0)),
        material=brushed,
        name="guide_collar",
    )

    seat_stage = model.part("seat_stage")
    seat_stage.visual(
        Cylinder(radius=0.028, length=0.080),
        origin=Origin(xyz=(0.0, 0.0, -0.040)),
        material=brushed,
        name="spindle",
    )
    seat_stage.visual(
        Cylinder(radius=0.045, length=0.030),
        origin=Origin(xyz=(0.0, 0.0, 0.003)),
        material=brushed,
        name="rotating_hub",
    )
    seat_stage.visual(
        mesh_from_geometry(TorusGeometry(radius=0.058, tube=0.0055), "upper_race"),
        origin=Origin(xyz=(0.0, 0.0, 0.0045)),
        material=brushed,
        name="upper_race",
    )
    seat_stage.visual(
        _annular_mesh(0.090, 0.166, 0.012, "gap_skirt"),
        origin=Origin(xyz=(0.0, 0.0, -0.006)),
        material=brushed,
        name="gap_skirt",
    )
    for i in range(4):
        angle = i * math.pi * 0.5
        seat_stage.visual(
            Box((0.115, 0.014, 0.008)),
            origin=Origin(
                xyz=(0.072 * math.cos(angle), 0.072 * math.sin(angle), 0.004),
                rpy=(0.0, 0.0, angle),
            ),
            material=brushed,
            name=f"stage_spoke_{i}",
        )

    pointer_profile = [(0.102, -0.012), (0.162, 0.0), (0.102, 0.012)]
    seat_stage.visual(
        mesh_from_geometry(
            ExtrudeGeometry.centered(pointer_profile, 0.003), "datum_pointer"
        ),
        origin=Origin(xyz=(0.0, 0.0, 0.0015)),
        material=red,
        name="pointer",
    )
    seat_stage.visual(
        Cylinder(radius=0.070, length=0.068),
        origin=Origin(xyz=(0.0, 0.0, 0.049)),
        material=brushed,
        name="seat_neck",
    )
    seat_stage.visual(
        Cylinder(radius=0.205, length=0.018),
        origin=Origin(xyz=(0.0, 0.0, 0.081)),
        material=brushed,
        name="seat_plate",
    )
    seat_stage.visual(
        Cylinder(radius=0.230, length=0.055),
        origin=Origin(xyz=(0.0, 0.0, 0.115)),
        material=dark,
        name="seat_pad",
    )
    seat_stage.visual(
        Box((0.360, 0.006, 0.0012)),
        origin=Origin(xyz=(0.0, 0.0, 0.1431)),
        material=white,
        name="seat_x_datum",
    )
    seat_stage.visual(
        Box((0.006, 0.360, 0.0012)),
        origin=Origin(xyz=(0.0, 0.0, 0.1431)),
        material=white,
        name="seat_y_datum",
    )
    seat_stage.visual(
        Box((0.095, 0.035, 0.0020)),
        origin=Origin(xyz=(0.145, 0.0, 0.1435)),
        material=brass,
        name="front_datum_pad",
    )
    for i, (x, y) in enumerate(((-0.080, 0.130), (-0.080, -0.130))):
        seat_stage.visual(
            Cylinder(radius=0.020, length=0.0020),
            origin=Origin(xyz=(x, y, 0.1435)),
            material=brass,
            name=f"datum_pad_{i}",
        )

    clamp_knob = model.part("clamp_knob")
    clamp_knob.visual(
        Cylinder(radius=0.007, length=0.075),
        origin=Origin(xyz=(0.0375, 0.0, 0.0), rpy=(0.0, math.pi / 2.0, 0.0)),
        material=brushed,
        name="knob_shaft",
    )
    clamp_knob.visual(
        Cylinder(radius=0.032, length=0.030),
        origin=Origin(xyz=(0.090, 0.0, 0.0), rpy=(0.0, math.pi / 2.0, 0.0)),
        material=dark,
        name="knob_grip",
    )
    clamp_knob.visual(
        Box((0.002, 0.006, 0.045)),
        origin=Origin(xyz=(0.106, 0.0, 0.008)),
        material=white,
        name="knob_witness",
    )

    model.articulation(
        "seat_swivel",
        ArticulationType.CONTINUOUS,
        parent=pedestal,
        child=seat_stage,
        origin=Origin(xyz=(0.0, 0.0, 0.666)),
        axis=(0.0, 0.0, 1.0),
        motion_limits=MotionLimits(effort=30.0, velocity=2.0),
        motion_properties=MotionProperties(damping=0.18, friction=0.05),
    )
    model.articulation(
        "clamp_knob_turn",
        ArticulationType.CONTINUOUS,
        parent=pedestal,
        child=clamp_knob,
        origin=Origin(xyz=(0.244, 0.0, 0.657)),
        axis=(1.0, 0.0, 0.0),
        motion_limits=MotionLimits(effort=4.0, velocity=5.0),
        motion_properties=MotionProperties(damping=0.04, friction=0.02),
    )

    return model


def run_tests() -> TestReport:
    ctx = TestContext(object_model)
    pedestal = object_model.get_part("pedestal")
    seat_stage = object_model.get_part("seat_stage")
    clamp_knob = object_model.get_part("clamp_knob")
    swivel = object_model.get_articulation("seat_swivel")
    knob_turn = object_model.get_articulation("clamp_knob_turn")

    def _center(aabb):
        if aabb is None:
            return None
        return tuple((aabb[0][i] + aabb[1][i]) * 0.5 for i in range(3))

    ctx.expect_within(
        seat_stage,
        pedestal,
        axes="xy",
        inner_elem="spindle",
        outer_elem="bearing_sleeve",
        margin=0.001,
        name="spindle is centered inside the bearing sleeve",
    )
    ctx.expect_overlap(
        seat_stage,
        pedestal,
        axes="z",
        elem_a="spindle",
        elem_b="bearing_sleeve",
        min_overlap=0.060,
        name="spindle remains retained in the supported sleeve",
    )
    ctx.expect_gap(
        seat_stage,
        pedestal,
        axis="z",
        positive_elem="gap_skirt",
        negative_elem="index_ring",
        min_gap=0.003,
        max_gap=0.006,
        name="controlled datum gap at the swivel dial",
    )
    ctx.expect_gap(
        seat_stage,
        pedestal,
        axis="z",
        positive_elem="upper_race",
        negative_elem="bearing_ball_00",
        min_gap=0.0,
        max_gap=0.002,
        name="upper race is supported just above the bearing balls",
    )
    ctx.expect_gap(
        clamp_knob,
        pedestal,
        axis="x",
        positive_elem="knob_shaft",
        negative_elem="guide_collar",
        max_gap=0.001,
        max_penetration=0.0002,
        name="lock knob shaft seats against its guide collar",
    )

    rest_pos = ctx.part_world_position(seat_stage)
    pointer_0 = _center(ctx.part_element_world_aabb(seat_stage, elem="pointer"))
    with ctx.pose({swivel: math.pi / 2.0}):
        turned_pos = ctx.part_world_position(seat_stage)
        pointer_90 = _center(ctx.part_element_world_aabb(seat_stage, elem="pointer"))

    ctx.check(
        "seat swivel stays centered on the pedestal axis",
        rest_pos is not None
        and turned_pos is not None
        and abs(rest_pos[0] - turned_pos[0]) < 1e-6
        and abs(rest_pos[1] - turned_pos[1]) < 1e-6,
        details=f"rest={rest_pos}, turned={turned_pos}",
    )
    ctx.check(
        "datum pointer rotates around the index ring",
        pointer_0 is not None
        and pointer_90 is not None
        and pointer_0[0] > 0.120
        and abs(pointer_0[1]) < 0.025
        and pointer_90[1] > 0.120
        and abs(pointer_90[0]) < 0.025,
        details=f"pointer_0={pointer_0}, pointer_90={pointer_90}",
    )

    witness_0 = ctx.part_element_world_aabb(clamp_knob, elem="knob_witness")
    with ctx.pose({knob_turn: math.pi / 2.0}):
        witness_90 = ctx.part_element_world_aabb(clamp_knob, elem="knob_witness")
    if witness_0 is not None and witness_90 is not None:
        dy_0 = witness_0[1][1] - witness_0[0][1]
        dz_0 = witness_0[1][2] - witness_0[0][2]
        dy_90 = witness_90[1][1] - witness_90[0][1]
        dz_90 = witness_90[1][2] - witness_90[0][2]
    else:
        dy_0 = dz_0 = dy_90 = dz_90 = 0.0
    ctx.check(
        "clamp knob witness mark rotates about the guide axis",
        dz_0 > dy_0 * 3.0 and dy_90 > dz_90 * 3.0,
        details=f"rest dy/dz=({dy_0:.4f},{dz_0:.4f}), turned dy/dz=({dy_90:.4f},{dz_90:.4f})",
    )

    return ctx.report()


object_model = build_object_model()
