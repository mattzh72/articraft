from __future__ import annotations

import math

from sdk import (
    ArticulatedObject,
    ArticulationType,
    Box,
    Cylinder,
    LatheGeometry,
    MotionLimits,
    Origin,
    TestContext,
    TestReport,
    TorusGeometry,
    mesh_from_geometry,
    superellipse_side_loft,
)


def _mesh(name: str, geometry):
    return mesh_from_geometry(geometry, name)


def _center_from_aabb(aabb):
    if aabb is None:
        return None
    lower, upper = aabb
    return tuple((lower[i] + upper[i]) * 0.5 for i in range(3))


def build_object_model() -> ArticulatedObject:
    model = ArticulatedObject(name="swivel_bar_stool")

    chrome = model.material("chrome", rgba=(0.78, 0.80, 0.83, 1.0))
    dark_steel = model.material("dark_steel", rgba=(0.30, 0.31, 0.34, 1.0))
    satin_black = model.material("satin_black", rgba=(0.12, 0.12, 0.13, 1.0))
    vinyl_black = model.material("vinyl_black", rgba=(0.10, 0.10, 0.11, 1.0))
    charcoal = model.material("charcoal", rgba=(0.17, 0.17, 0.18, 1.0))

    pedestal = model.part("pedestal")
    base_profile = [
        (0.0, 0.0),
        (0.110, 0.0),
        (0.185, 0.003),
        (0.226, 0.010),
        (0.238, 0.026),
        (0.225, 0.040),
        (0.168, 0.050),
        (0.078, 0.054),
        (0.045, 0.055),
        (0.0, 0.055),
    ]
    pedestal.visual(
        _mesh("trumpet_base", LatheGeometry(base_profile, segments=88)),
        material=chrome,
        name="trumpet_base",
    )
    pedestal.visual(
        Cylinder(radius=0.038, length=0.490),
        origin=Origin(xyz=(0.0, 0.0, 0.300)),
        material=chrome,
        name="lower_column",
    )
    pedestal.visual(
        Cylinder(radius=0.043, length=0.137),
        origin=Origin(xyz=(0.0, 0.0, 0.6135)),
        material=chrome,
        name="upper_column",
    )
    pedestal.visual(
        Cylinder(radius=0.058, length=0.030),
        origin=Origin(xyz=(0.0, 0.0, 0.332)),
        material=dark_steel,
        name="footrest_collar",
    )
    pedestal.visual(
        _mesh("footrest_ring", TorusGeometry(radius=0.180, tube=0.013, radial_segments=20, tubular_segments=72)),
        origin=Origin(xyz=(0.0, 0.0, 0.332)),
        material=dark_steel,
        name="footrest_ring",
    )
    spoke_len = 0.114
    spoke_center = 0.113
    for name, xyz, rpy in (
        ("footrest_spoke_front", (spoke_center, 0.0, 0.332), (0.0, math.pi / 2.0, 0.0)),
        ("footrest_spoke_rear", (-spoke_center, 0.0, 0.332), (0.0, math.pi / 2.0, 0.0)),
        ("footrest_spoke_right", (0.0, spoke_center, 0.332), (-math.pi / 2.0, 0.0, 0.0)),
        ("footrest_spoke_left", (0.0, -spoke_center, 0.332), (-math.pi / 2.0, 0.0, 0.0)),
    ):
        pedestal.visual(
            Cylinder(radius=0.008, length=spoke_len),
            origin=Origin(xyz=xyz, rpy=rpy),
            material=dark_steel,
            name=name,
        )
    pedestal.visual(
        Cylinder(radius=0.070, length=0.018),
        origin=Origin(xyz=(0.0, 0.0, 0.691)),
        material=dark_steel,
        name="top_cap",
    )

    seat = model.part("seat")
    seat_profile = [
        (0.0, 0.012),
        (0.095, 0.012),
        (0.152, 0.016),
        (0.198, 0.027),
        (0.209, 0.046),
        (0.204, 0.070),
        (0.172, 0.084),
        (0.0, 0.086),
    ]
    seat.visual(
        _mesh("seat_cushion", LatheGeometry(seat_profile, segments=88)),
        material=vinyl_black,
        name="seat_cushion",
    )
    seat.visual(
        Cylinder(radius=0.082, length=0.012),
        origin=Origin(xyz=(0.0, 0.0, 0.006)),
        material=dark_steel,
        name="seat_plate",
    )
    for side_sign, name in ((1.0, "hinge_cheek_0"), (-1.0, "hinge_cheek_1")):
        seat.visual(
            Box((0.028, 0.030, 0.048)),
            origin=Origin(xyz=(-0.206, 0.071 * side_sign, 0.030)),
            material=dark_steel,
            name=name,
        )
    seat.visual(
        Cylinder(radius=0.012, length=0.024),
        origin=Origin(xyz=(-0.110, 0.186, 0.040), rpy=(-math.pi / 2.0, 0.0, 0.0)),
        material=dark_steel,
        name="knob_boss",
    )

    backrest = model.part("backrest")
    backrest.visual(
        Cylinder(radius=0.010, length=0.112),
        origin=Origin(rpy=(-math.pi / 2.0, 0.0, 0.0)),
        material=dark_steel,
        name="hinge_barrel",
    )
    backrest.visual(
        Box((0.036, 0.100, 0.064)),
        origin=Origin(xyz=(-0.024, 0.0, 0.032)),
        material=dark_steel,
        name="back_bracket",
    )
    backrest.visual(
        Box((0.038, 0.132, 0.084)),
        origin=Origin(xyz=(-0.042, 0.0, 0.090)),
        material=dark_steel,
        name="back_spine",
    )
    back_pad = superellipse_side_loft(
        [
            (-0.160, 0.126, 0.235, 0.032),
            (-0.090, 0.120, 0.252, 0.050),
            (0.000, 0.116, 0.262, 0.058),
            (0.090, 0.120, 0.252, 0.050),
            (0.160, 0.126, 0.235, 0.032),
        ],
        exponents=2.2,
        segments=44,
    ).translate(-0.048, 0.0, 0.0)
    backrest.visual(
        _mesh("back_pad", back_pad),
        material=vinyl_black,
        name="back_pad",
    )

    tension_knob = model.part("tension_knob")
    tension_knob.visual(
        Cylinder(radius=0.005, length=0.016),
        origin=Origin(xyz=(0.0, 0.008, 0.0), rpy=(-math.pi / 2.0, 0.0, 0.0)),
        material=chrome,
        name="shaft",
    )
    tension_knob.visual(
        Cylinder(radius=0.021, length=0.012),
        origin=Origin(xyz=(0.0, 0.016, 0.0), rpy=(-math.pi / 2.0, 0.0, 0.0)),
        material=charcoal,
        name="knob_body",
    )
    tension_knob.visual(
        Cylinder(radius=0.018, length=0.010),
        origin=Origin(xyz=(0.0, 0.027, 0.0), rpy=(-math.pi / 2.0, 0.0, 0.0)),
        material=satin_black,
        name="knob_cap",
    )
    tension_knob.visual(
        Box((0.008, 0.006, 0.006)),
        origin=Origin(xyz=(0.016, 0.022, 0.0)),
        material=chrome,
        name="grip_nub",
    )

    model.articulation(
        "pedestal_to_seat",
        ArticulationType.CONTINUOUS,
        parent=pedestal,
        child=seat,
        origin=Origin(xyz=(0.0, 0.0, 0.700)),
        axis=(0.0, 0.0, 1.0),
        motion_limits=MotionLimits(effort=40.0, velocity=8.0),
    )
    model.articulation(
        "seat_to_backrest",
        ArticulationType.REVOLUTE,
        parent=seat,
        child=backrest,
        origin=Origin(xyz=(-0.218, 0.0, 0.040)),
        axis=(0.0, -1.0, 0.0),
        motion_limits=MotionLimits(
            effort=12.0,
            velocity=1.4,
            lower=0.0,
            upper=0.38,
        ),
    )
    model.articulation(
        "seat_to_tension_knob",
        ArticulationType.CONTINUOUS,
        parent=seat,
        child=tension_knob,
        origin=Origin(xyz=(-0.110, 0.197, 0.040)),
        axis=(0.0, 1.0, 0.0),
        motion_limits=MotionLimits(effort=0.8, velocity=8.0),
    )

    return model


def run_tests() -> TestReport:
    ctx = TestContext(object_model)

    pedestal = object_model.get_part("pedestal")
    seat = object_model.get_part("seat")
    backrest = object_model.get_part("backrest")
    tension_knob = object_model.get_part("tension_knob")

    seat_spin = object_model.get_articulation("pedestal_to_seat")
    back_tilt = object_model.get_articulation("seat_to_backrest")
    knob_turn = object_model.get_articulation("seat_to_tension_knob")

    ctx.expect_gap(
        seat,
        pedestal,
        axis="z",
        positive_elem="seat_plate",
        negative_elem="top_cap",
        max_gap=0.001,
        max_penetration=0.0,
        name="seat plate rests on pedestal cap",
    )
    ctx.expect_overlap(
        seat,
        pedestal,
        axes="xy",
        elem_a="seat_plate",
        elem_b="top_cap",
        min_overlap=0.12,
        name="seat remains centered over pedestal cap",
    )
    ctx.expect_gap(
        backrest,
        seat,
        axis="z",
        positive_elem="back_pad",
        negative_elem="seat_cushion",
        min_gap=0.050,
        name="low back sits above the seat cushion",
    )
    ctx.expect_contact(
        tension_knob,
        seat,
        elem_a="shaft",
        elem_b="knob_boss",
        name="tension knob shaft mounts into the seat boss",
    )
    ctx.expect_origin_gap(
        tension_knob,
        seat,
        axis="y",
        min_gap=0.180,
        max_gap=0.220,
        name="tension knob sits on the right side of the seat",
    )

    knob_rest = ctx.part_world_position(tension_knob)
    with ctx.pose({seat_spin: math.pi / 2.0}):
        knob_spun = ctx.part_world_position(tension_knob)
    ctx.check(
        "seat spins around pedestal axis",
        knob_rest is not None
        and knob_spun is not None
        and abs(knob_rest[0] - knob_spun[0]) > 0.07
        and abs(knob_rest[1] - knob_spun[1]) > 0.12,
        details=f"rest={knob_rest}, spun={knob_spun}",
    )

    back_rest_aabb = ctx.part_element_world_aabb(backrest, elem="back_pad")
    with ctx.pose({back_tilt: back_tilt.motion_limits.upper}):
        back_tilted_aabb = ctx.part_element_world_aabb(backrest, elem="back_pad")
    back_rest_center = _center_from_aabb(back_rest_aabb)
    back_tilted_center = _center_from_aabb(back_tilted_aabb)
    ctx.check(
        "backrest tilts rearward about the hinge",
        back_rest_center is not None
        and back_tilted_center is not None
        and back_tilted_center[0] < back_rest_center[0] - 0.030
        and back_tilted_center[2] < back_rest_center[2] - 0.015,
        details=f"rest={back_rest_center}, tilted={back_tilted_center}",
    )

    knob_rest_aabb = ctx.part_element_world_aabb(tension_knob, elem="grip_nub")
    with ctx.pose({knob_turn: math.pi / 2.0}):
        knob_turned_aabb = ctx.part_element_world_aabb(tension_knob, elem="grip_nub")
    knob_rest_center = _center_from_aabb(knob_rest_aabb)
    knob_turned_center = _center_from_aabb(knob_turned_aabb)
    ctx.check(
        "tension knob rotation moves the visible grip nub",
        knob_rest_center is not None
        and knob_turned_center is not None
        and abs(knob_rest_center[0] - knob_turned_center[0]) > 0.010
        and abs(knob_rest_center[2] - knob_turned_center[2]) > 0.010,
        details=f"rest={knob_rest_center}, turned={knob_turned_center}",
    )

    return ctx.report()


object_model = build_object_model()
