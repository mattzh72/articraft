from __future__ import annotations

import math

from sdk import (
    ArticulatedObject,
    ArticulationType,
    Box,
    Cylinder,
    ExtrudeGeometry,
    KnobGeometry,
    KnobGrip,
    Material,
    MotionLimits,
    Origin,
    TestContext,
    TestReport,
    mesh_from_geometry,
    rounded_rect_profile,
)


def _rounded_pad_mesh(length: float, width: float, thickness: float, radius: float, name: str):
    profile = rounded_rect_profile(length, width, radius, corner_segments=8)
    return mesh_from_geometry(
        ExtrudeGeometry(profile, thickness, center=True),
        name,
    )


def build_object_model() -> ArticulatedObject:
    model = ArticulatedObject(name="flat_incline_utility_bench")

    steel = model.material("satin_black_powdercoat", rgba=(0.02, 0.022, 0.024, 1.0))
    rubber = model.material("black_rubber_feet", rgba=(0.01, 0.01, 0.009, 1.0))
    vinyl = model.material("slightly_sheened_black_vinyl", rgba=(0.015, 0.015, 0.017, 1.0))
    seam = model.material("dark_gray_seams", rgba=(0.10, 0.10, 0.105, 1.0))
    pin_metal = model.material("brushed_pin_steel", rgba=(0.56, 0.56, 0.52, 1.0))
    selector_black = model.material("matte_selector_plate", rgba=(0.035, 0.037, 0.04, 1.0))
    knob_red = model.material("red_pop_pin_knob", rgba=(0.62, 0.02, 0.02, 1.0))

    # The world frame is garage-bench scale: X is bench length, Y is width, Z is up.
    base = model.part("base_frame")
    # Floor rectangle.
    base.visual(Box((1.54, 0.045, 0.050)), origin=Origin(xyz=(0.08, 0.255, 0.035)), material=steel, name="side_rail_0")
    base.visual(Box((1.54, 0.045, 0.050)), origin=Origin(xyz=(0.08, -0.255, 0.035)), material=steel, name="side_rail_1")
    base.visual(Box((1.54, 0.050, 0.050)), origin=Origin(xyz=(0.08, 0.0, 0.035)), material=steel, name="center_floor_rail")
    base.visual(Box((0.060, 0.555, 0.050)), origin=Origin(xyz=(-0.665, 0.0, 0.035)), material=steel, name="front_crossbar")
    base.visual(Box((0.060, 0.555, 0.050)), origin=Origin(xyz=(0.825, 0.0, 0.035)), material=steel, name="rear_crossbar")
    base.visual(Box((0.100, 0.090, 0.018)), origin=Origin(xyz=(-0.665, 0.255, 0.009)), material=rubber, name="foot_0")
    base.visual(Box((0.100, 0.090, 0.018)), origin=Origin(xyz=(-0.665, -0.255, 0.009)), material=rubber, name="foot_1")
    base.visual(Box((0.100, 0.090, 0.018)), origin=Origin(xyz=(0.825, 0.255, 0.009)), material=rubber, name="foot_2")
    base.visual(Box((0.100, 0.090, 0.018)), origin=Origin(xyz=(0.825, -0.255, 0.009)), material=rubber, name="foot_3")
    # Supported central tube under the pads.
    base.visual(Box((1.02, 0.060, 0.055)), origin=Origin(xyz=(-0.060, 0.0, 0.470)), material=steel, name="top_spine")
    base.visual(Box((0.055, 0.060, 0.420)), origin=Origin(xyz=(-0.470, 0.0, 0.245)), material=steel, name="front_upright")
    base.visual(Box((0.055, 0.060, 0.420)), origin=Origin(xyz=(0.150, 0.0, 0.245)), material=steel, name="middle_upright")
    base.visual(Box((0.680, 0.045, 0.045)), origin=Origin(xyz=(-0.160, 0.0, 0.255), rpy=(0.0, -0.42, 0.0)), material=steel, name="diagonal_brace")
    # Separate front seat hinge yoke.
    base.visual(Box((0.100, 0.460, 0.050)), origin=Origin(xyz=(-0.470, 0.0, 0.445)), material=steel, name="front_hinge_bridge")
    base.visual(Box((0.055, 0.026, 0.130)), origin=Origin(xyz=(-0.470, 0.225, 0.520)), material=steel, name="front_hinge_lug_0")
    base.visual(Box((0.055, 0.026, 0.130)), origin=Origin(xyz=(-0.470, -0.225, 0.520)), material=steel, name="front_hinge_lug_1")
    # Seat/backrest junction hinge yoke.
    base.visual(Box((0.110, 0.460, 0.050)), origin=Origin(xyz=(0.060, 0.0, 0.445)), material=steel, name="back_hinge_bridge")
    base.visual(Box((0.055, 0.026, 0.130)), origin=Origin(xyz=(0.060, 0.225, 0.520)), material=steel, name="back_hinge_lug_0")
    base.visual(Box((0.055, 0.026, 0.130)), origin=Origin(xyz=(0.060, -0.225, 0.520)), material=steel, name="back_hinge_lug_1")
    # Rear lower pivot yoke for the ladder support, set behind the backrest bracket.
    base.visual(Box((0.060, 0.060, 0.270)), origin=Origin(xyz=(0.790, 0.255, 0.155)), material=steel, name="rear_pivot_stand_0")
    base.visual(Box((0.060, 0.060, 0.270)), origin=Origin(xyz=(0.790, -0.255, 0.155)), material=steel, name="rear_pivot_stand_1")
    base.visual(Box((0.060, 0.070, 0.060)), origin=Origin(xyz=(0.790, 0.215, 0.180)), material=steel, name="rear_pivot_yoke_0")
    base.visual(Box((0.060, 0.070, 0.060)), origin=Origin(xyz=(0.790, -0.215, 0.180)), material=steel, name="rear_pivot_yoke_1")
    base.visual(Box((0.065, 0.026, 0.130)), origin=Origin(xyz=(0.790, 0.190, 0.180)), material=steel, name="rear_pivot_lug_0")
    base.visual(Box((0.065, 0.026, 0.130)), origin=Origin(xyz=(0.790, -0.190, 0.180)), material=steel, name="rear_pivot_lug_1")
    base.visual(Box((0.060, 0.120, 0.040)), origin=Origin(xyz=(0.790, 0.255, 0.060)), material=steel, name="rear_lug_foot_0")
    base.visual(Box((0.060, 0.120, 0.040)), origin=Origin(xyz=(0.790, -0.255, 0.060)), material=steel, name="rear_lug_foot_1")

    seat = model.part("seat")
    seat.visual(
        _rounded_pad_mesh(0.530, 0.390, 0.075, 0.045, "seat_rounded_pad"),
        origin=Origin(xyz=(0.265, 0.0, 0.058)),
        material=vinyl,
        name="seat_pad",
    )
    seat.visual(Box((0.480, 0.014, 0.012)), origin=Origin(xyz=(0.265, 0.202, 0.095)), material=seam, name="seat_side_piping_0")
    seat.visual(Box((0.480, 0.014, 0.012)), origin=Origin(xyz=(0.265, -0.202, 0.095)), material=seam, name="seat_side_piping_1")
    seat.visual(Cylinder(radius=0.018, length=0.424), origin=Origin(rpy=(-math.pi / 2, 0.0, 0.0)), material=pin_metal, name="front_hinge_tube")
    seat.visual(Box((0.145, 0.330, 0.026)), origin=Origin(xyz=(0.070, 0.0, 0.025)), material=steel, name="front_hinge_leaf")

    backrest = model.part("backrest")
    backrest.visual(
        _rounded_pad_mesh(0.940, 0.365, 0.078, 0.052, "backrest_rounded_pad"),
        origin=Origin(xyz=(0.510, 0.0, 0.059)),
        material=vinyl,
        name="backrest_pad",
    )
    backrest.visual(Box((0.860, 0.014, 0.012)), origin=Origin(xyz=(0.515, 0.178, 0.097)), material=seam, name="backrest_side_piping_0")
    backrest.visual(Box((0.860, 0.014, 0.012)), origin=Origin(xyz=(0.515, -0.178, 0.097)), material=seam, name="backrest_side_piping_1")
    backrest.visual(Cylinder(radius=0.018, length=0.424), origin=Origin(rpy=(-math.pi / 2, 0.0, 0.0)), material=pin_metal, name="back_hinge_tube")
    backrest.visual(Box((0.830, 0.030, 0.040)), origin=Origin(xyz=(0.470, 0.145, 0.000)), material=steel, name="under_rail_0")
    backrest.visual(Box((0.830, 0.030, 0.040)), origin=Origin(xyz=(0.470, -0.145, 0.000)), material=steel, name="under_rail_1")
    backrest.visual(Box((0.120, 0.330, 0.026)), origin=Origin(xyz=(0.060, 0.0, 0.026)), material=steel, name="back_hinge_leaf")
    # Side selector plate with visible hole marks. It is mounted under and just
    # outside the backrest rail so the support ladder remains behind it, not hidden
    # inside the frame.
    backrest.visual(Box((0.390, 0.020, 0.070)), origin=Origin(xyz=(0.370, -0.168, -0.050)), material=selector_black, name="selector_plate")
    for hole_name, x in (
        ("selector_hole_0", 0.255),
        ("selector_hole_1", 0.335),
        ("selector_hole_2", 0.415),
        ("selector_hole_3", 0.495),
    ):
        backrest.visual(
            Cylinder(radius=0.013, length=0.006),
            origin=Origin(xyz=(x, -0.181, -0.050), rpy=(math.pi / 2, 0.0, 0.0)),
            material=pin_metal,
            name=hole_name,
        )

    ladder = model.part("ladder_support")
    dx, dz = -0.260, 0.305
    ladder_len = math.hypot(dx, dz)
    ladder_pitch = math.atan2(dx, dz)
    ladder.visual(Cylinder(radius=0.018, length=0.354), origin=Origin(rpy=(-math.pi / 2, 0.0, 0.0)), material=pin_metal, name="pivot_tube")
    for y, rail_name in ((0.115, "side_rail_0"), (-0.115, "side_rail_1")):
        ladder.visual(
            Box((0.026, 0.026, ladder_len)),
            origin=Origin(xyz=(dx * 0.50, y, dz * 0.50), rpy=(0.0, ladder_pitch, 0.0)),
            material=steel,
            name=rail_name,
        )
    for idx, frac in enumerate((0.32, 0.58, 0.84)):
        ladder.visual(
            Box((0.038, 0.275, 0.018)),
            origin=Origin(xyz=(dx * frac, 0.0, dz * frac)),
            material=pin_metal if idx == 2 else steel,
            name=f"ladder_rung_{idx}",
        )
    ladder.visual(
        Cylinder(radius=0.016, length=0.285),
        origin=Origin(xyz=(dx, 0.0, dz), rpy=(-math.pi / 2, 0.0, 0.0)),
        material=pin_metal,
        name="upper_roller",
    )

    pop_pin = model.part("pop_pin")
    pop_pin.visual(
        Cylinder(radius=0.008, length=0.060),
        origin=Origin(xyz=(0.0, -0.030, 0.0), rpy=(math.pi / 2, 0.0, 0.0)),
        material=pin_metal,
        name="pin_stem",
    )
    pop_pin.visual(
        mesh_from_geometry(
            KnobGeometry(
                0.060,
                0.026,
                body_style="lobed",
                grip=KnobGrip(style="scalloped", count=6, depth=0.004),
            ),
            "pop_pin_lobed_knob",
        ),
        origin=Origin(xyz=(0.0, -0.073, 0.0), rpy=(math.pi / 2, 0.0, 0.0)),
        material=knob_red,
        name="knob_cap",
    )

    model.articulation(
        "base_to_backrest",
        ArticulationType.REVOLUTE,
        parent=base,
        child=backrest,
        origin=Origin(xyz=(0.060, 0.0, 0.520)),
        axis=(0.0, -1.0, 0.0),
        motion_limits=MotionLimits(effort=120.0, velocity=1.2, lower=0.0, upper=1.25),
    )
    model.articulation(
        "base_to_seat",
        ArticulationType.REVOLUTE,
        parent=base,
        child=seat,
        origin=Origin(xyz=(-0.470, 0.0, 0.520)),
        axis=(0.0, -1.0, 0.0),
        motion_limits=MotionLimits(effort=70.0, velocity=1.0, lower=0.0, upper=0.35),
    )
    model.articulation(
        "base_to_ladder",
        ArticulationType.REVOLUTE,
        parent=base,
        child=ladder,
        origin=Origin(xyz=(0.790, 0.0, 0.180)),
        axis=(0.0, 1.0, 0.0),
        motion_limits=MotionLimits(effort=45.0, velocity=1.5, lower=-0.35, upper=0.85),
    )
    model.articulation(
        "backrest_to_pop_pin",
        ArticulationType.PRISMATIC,
        parent=backrest,
        child=pop_pin,
        origin=Origin(xyz=(0.335, -0.178, -0.050)),
        axis=(0.0, -1.0, 0.0),
        motion_limits=MotionLimits(effort=25.0, velocity=0.20, lower=0.0, upper=0.035),
    )
    return model


def run_tests() -> TestReport:
    ctx = TestContext(object_model)
    backrest = object_model.get_part("backrest")
    seat = object_model.get_part("seat")
    ladder = object_model.get_part("ladder_support")
    pop_pin = object_model.get_part("pop_pin")
    base = object_model.get_part("base_frame")

    backrest_hinge = object_model.get_articulation("base_to_backrest")
    seat_hinge = object_model.get_articulation("base_to_seat")
    pop_pin_slide = object_model.get_articulation("backrest_to_pop_pin")

    ctx.allow_overlap(
        backrest,
        pop_pin,
        elem_a="selector_hole_1",
        elem_b="pin_stem",
        reason="The pop-pin stem is intentionally shown seated in one selector hole under the backrest.",
    )
    ctx.expect_overlap(
        backrest,
        pop_pin,
        axes="xz",
        elem_a="selector_hole_1",
        elem_b="pin_stem",
        min_overlap=0.010,
        name="pop pin stem is seated through a selected hole",
    )
    ctx.expect_gap(
        backrest,
        pop_pin,
        axis="y",
        positive_elem="selector_hole_1",
        negative_elem="pin_stem",
        max_penetration=0.007,
        name="pop pin insertion is a shallow local fit",
    )

    ctx.expect_gap(
        ladder,
        backrest,
        axis="x",
        positive_elem="pivot_tube",
        negative_elem="selector_plate",
        min_gap=0.08,
        name="rear ladder lower pivot sits behind selector bracket",
    )
    ctx.expect_gap(
        backrest,
        pop_pin,
        axis="y",
        positive_elem="selector_plate",
        negative_elem="knob_cap",
        min_gap=0.025,
        name="pop pin knob remains visibly separate from selector plate",
    )
    ctx.expect_gap(
        seat,
        base,
        axis="z",
        positive_elem="seat_pad",
        negative_elem="top_spine",
        min_gap=0.025,
        name="seat pad rides above the steel frame",
    )

    rest_backrest = ctx.part_world_aabb(backrest)
    rest_seat = ctx.part_world_aabb(seat)
    rest_pin = ctx.part_world_position(pop_pin)
    with ctx.pose({backrest_hinge: 1.10, seat_hinge: 0.30, pop_pin_slide: 0.030}):
        raised_backrest = ctx.part_world_aabb(backrest)
        raised_seat = ctx.part_world_aabb(seat)
        pulled_pin = ctx.part_world_position(pop_pin)

    ctx.check(
        "backrest hinge raises the rear pad upward",
        rest_backrest is not None
        and raised_backrest is not None
        and raised_backrest[1][2] > rest_backrest[1][2] + 0.25,
        details=f"rest={rest_backrest}, raised={raised_backrest}",
    )
    ctx.check(
        "seat front hinge lifts the rear of the seat",
        rest_seat is not None
        and raised_seat is not None
        and raised_seat[1][2] > rest_seat[1][2] + 0.07,
        details=f"rest={rest_seat}, raised={raised_seat}",
    )
    ctx.check(
        "pop pin translates outward along its short axis",
        rest_pin is not None and pulled_pin is not None and pulled_pin[1] < rest_pin[1] - 0.020,
        details=f"rest={rest_pin}, pulled={pulled_pin}",
    )

    return ctx.report()


object_model = build_object_model()
