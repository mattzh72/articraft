from __future__ import annotations

import math

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


def _box(part, name, size, xyz, material):
    part.visual(Box(size), origin=Origin(xyz=xyz), material=material, name=name)


def _cylinder_y(part, name, radius, length, xyz, material):
    part.visual(
        Cylinder(radius=radius, length=length),
        origin=Origin(xyz=xyz, rpy=(-math.pi / 2.0, 0.0, 0.0)),
        material=material,
        name=name,
    )


def _cylinder_x(part, name, radius, length, xyz, material):
    part.visual(
        Cylinder(radius=radius, length=length),
        origin=Origin(xyz=xyz, rpy=(0.0, math.pi / 2.0, 0.0)),
        material=material,
        name=name,
    )


def build_object_model() -> ArticulatedObject:
    model = ArticulatedObject(name="precision_drying_rack")

    satin = model.material("satin_anodized_aluminum", rgba=(0.78, 0.82, 0.84, 1.0))
    dark = model.material("black_etched_marks", rgba=(0.02, 0.025, 0.025, 1.0))
    blue = model.material("blue_gap_shims", rgba=(0.08, 0.20, 0.72, 1.0))
    brass = model.material("brass_datum_faces", rgba=(0.95, 0.68, 0.26, 1.0))
    rubber = model.material("matte_black_rubber", rgba=(0.03, 0.03, 0.03, 1.0))

    central = model.part("central_frame")

    # Stable floor stack and square datum-friendly upper frame.
    for y, suffix in ((0.17, "0"), (-0.17, "1")):
        _box(central, f"foot_bar_{suffix}", (1.16, 0.040, 0.030), (0.0, y, 0.045), satin)
        for x in (-0.55, 0.55):
            _box(central, f"foot_pad_{suffix}_{'a' if x < 0 else 'b'}", (0.095, 0.072, 0.030), (x, y, 0.015), rubber)
            _box(central, f"leg_{suffix}_{'a' if x < 0 else 'b'}", (0.026, 0.026, 0.850), (x, y, 0.475), satin)

    _box(central, "side_rail_0", (1.20, 0.030, 0.024), (0.0, 0.208, 0.900), satin)
    _box(central, "side_rail_1", (1.20, 0.030, 0.024), (0.0, -0.208, 0.900), satin)
    for x, suffix in ((-0.55, "0"), (0.0, "1"), (0.55, "2")):
        _box(central, f"cross_rail_{suffix}", (0.024, 0.440, 0.024), (x, 0.0, 0.900), satin)

    for y, suffix in ((-0.108, "0"), (0.0, "1"), (0.108, "2")):
        _box(central, f"center_hanging_rail_{suffix}", (1.15, 0.014, 0.014), (0.0, y, 0.912), satin)

    _box(central, "datum_spine", (1.14, 0.030, 0.010), (0.0, 0.0, 0.918), brass)
    for x, suffix in ((-0.32, "0"), (0.0, "1"), (0.32, "2")):
        _box(central, f"datum_pad_{suffix}", (0.085, 0.064, 0.006), (x, 0.0, 0.926), brass)
        _box(central, f"centerline_mark_{suffix}", (0.006, 0.070, 0.002), (x, 0.0, 0.930), dark)

    # Hinge carrier rails keep the fold axes exposed but separated by a controlled gap.
    _box(central, "hinge_carrier_0", (1.18, 0.030, 0.040), (0.0, 0.237, 0.900), satin)
    _box(central, "hinge_carrier_1", (1.18, 0.030, 0.040), (0.0, -0.237, 0.900), satin)
    _cylinder_x(central, "hinge_pin_0", 0.0040, 1.16, (0.0, 0.265, 0.900), satin)
    _cylinder_x(central, "hinge_pin_1", 0.0040, 1.16, (0.0, -0.265, 0.900), satin)
    for x, suffix in ((-0.572, "a"), (0.572, "b")):
        _box(central, f"pin_boss_0_{suffix}", (0.018, 0.020, 0.026), (x, 0.256, 0.900), satin)
        _box(central, f"pin_boss_1_{suffix}", (0.018, 0.020, 0.026), (x, -0.256, 0.900), satin)
    _box(central, "index_plate_0", (0.180, 0.018, 0.145), (0.0, 0.238, 0.825), satin)
    _box(central, "index_plate_1", (0.180, 0.018, 0.145), (0.0, -0.238, 0.825), satin)

    for side, y, face_y in (("0", 0.248, 0.248), ("1", -0.248, -0.248)):
        for i, z in enumerate((0.785, 0.810, 0.835, 0.860, 0.885)):
            _box(central, f"angle_tick_{side}_{i}", (0.032 if i % 2 == 0 else 0.020, 0.003, 0.003), (0.055 - i * 0.026, face_y, z), dark)
        _box(central, f"fold_stop_block_{side}", (0.070, 0.024, 0.036), (0.50, y * (1 if side == "0" else 1), 0.865), blue)
        _box(central, f"open_stop_block_{side}", (0.070, 0.024, 0.036), (-0.50, y * (1 if side == "0" else 1), 0.865), blue)

    # Rotating index lock knobs: user controls, deliberately articulated.
    knob_0 = model.part("index_knob_0")
    _cylinder_y(knob_0, "knob_body", 0.028, 0.034, (0.0, 0.017, 0.0), brass)
    _cylinder_y(knob_0, "knob_face", 0.031, 0.006, (0.0, 0.037, 0.0), brass)
    _box(knob_0, "pointer_mark", (0.006, 0.003, 0.024), (0.0, 0.041, 0.010), dark)
    model.articulation(
        "index_knob_0_spin",
        ArticulationType.CONTINUOUS,
        parent=central,
        child=knob_0,
        origin=Origin(xyz=(0.0, 0.247, 0.825)),
        axis=(0.0, 1.0, 0.0),
        motion_limits=MotionLimits(effort=2.0, velocity=4.0),
    )

    knob_1 = model.part("index_knob_1")
    _cylinder_y(knob_1, "knob_body", 0.028, 0.034, (0.0, -0.017, 0.0), brass)
    _cylinder_y(knob_1, "knob_face", 0.031, 0.006, (0.0, -0.037, 0.0), brass)
    _box(knob_1, "pointer_mark", (0.006, 0.003, 0.024), (0.0, -0.041, 0.010), dark)
    model.articulation(
        "index_knob_1_spin",
        ArticulationType.CONTINUOUS,
        parent=central,
        child=knob_1,
        origin=Origin(xyz=(0.0, -0.247, 0.825)),
        axis=(0.0, -1.0, 0.0),
        motion_limits=MotionLimits(effort=2.0, velocity=4.0),
    )

    def make_hinge_link(name: str, sign: float):
        link = model.part(name)
        _cylinder_x(link, "hinge_barrel", 0.0075, 1.12, (0.0, 0.0, 0.0), satin)
        _box(link, "leaf_bar", (1.10, 0.024, 0.018), (0.0, sign * 0.018, 0.0), satin)
        _box(link, "gap_shim_0", (0.048, 0.006, 0.024), (-0.47, sign * 0.006, 0.0), blue)
        _box(link, "gap_shim_1", (0.048, 0.006, 0.024), (0.47, sign * 0.006, 0.0), blue)
        _box(link, "stop_tab_open", (0.070, 0.016, 0.020), (-0.50, sign * 0.014, -0.010), blue)
        _box(link, "stop_tab_fold", (0.070, 0.016, 0.020), (0.50, sign * 0.014, -0.010), blue)
        return link

    link_0 = make_hinge_link("hinge_link_0", 1.0)
    link_1 = make_hinge_link("hinge_link_1", -1.0)

    fold_limits = MotionLimits(effort=18.0, velocity=1.1, lower=0.0, upper=1.55)
    model.articulation(
        "fold_hinge_0",
        ArticulationType.REVOLUTE,
        parent=central,
        child=link_0,
        origin=Origin(xyz=(0.0, 0.265, 0.900)),
        axis=(1.0, 0.0, 0.0),
        motion_limits=fold_limits,
    )
    model.articulation(
        "fold_hinge_1",
        ArticulationType.REVOLUTE,
        parent=central,
        child=link_1,
        origin=Origin(xyz=(0.0, -0.265, 0.900)),
        axis=(-1.0, 0.0, 0.0),
        motion_limits=fold_limits,
    )

    def make_wing(name: str, sign: float):
        wing = model.part(name)
        _box(wing, "root_rail", (1.10, 0.024, 0.024), (0.0, sign * 0.042, 0.0), satin)
        _box(wing, "outer_rail", (1.10, 0.024, 0.024), (0.0, sign * 0.520, 0.0), satin)
        for x, suffix in ((-0.55, "0"), (0.55, "1")):
            _box(wing, f"side_bar_{suffix}", (0.024, 0.500, 0.024), (x, sign * 0.280, 0.0), satin)
        for i, y in enumerate((0.130, 0.220, 0.310, 0.400)):
            _box(wing, f"hanging_rail_{i}", (1.08, 0.012, 0.012), (0.0, sign * y, 0.015), satin)
        _box(wing, "datum_edge", (0.360, 0.010, 0.006), (0.0, sign * 0.520, 0.015), brass)
        _box(wing, "datum_corner_0", (0.090, 0.030, 0.006), (-0.50, sign * 0.515, 0.015), brass)
        _box(wing, "datum_corner_1", (0.090, 0.030, 0.006), (0.50, sign * 0.515, 0.015), brass)
        for i, x in enumerate((-0.36, -0.18, 0.0, 0.18, 0.36)):
            _box(wing, f"index_mark_{i}", (0.006, 0.030, 0.002), (x, sign * 0.043, 0.013), dark)
        return wing

    wing_0 = make_wing("wing_0", 1.0)
    wing_1 = make_wing("wing_1", -1.0)
    model.articulation(
        "link_to_wing_0",
        ArticulationType.FIXED,
        parent=link_0,
        child=wing_0,
        origin=Origin(),
    )
    model.articulation(
        "link_to_wing_1",
        ArticulationType.FIXED,
        parent=link_1,
        child=wing_1,
        origin=Origin(),
    )

    return model


def run_tests() -> TestReport:
    ctx = TestContext(object_model)
    central = object_model.get_part("central_frame")
    link_0 = object_model.get_part("hinge_link_0")
    link_1 = object_model.get_part("hinge_link_1")
    wing_0 = object_model.get_part("wing_0")
    wing_1 = object_model.get_part("wing_1")
    knob_0 = object_model.get_part("index_knob_0")
    knob_1 = object_model.get_part("index_knob_1")
    fold_0 = object_model.get_articulation("fold_hinge_0")
    fold_1 = object_model.get_articulation("fold_hinge_1")

    ctx.allow_overlap(
        central,
        link_0,
        elem_a="hinge_pin_0",
        elem_b="hinge_barrel",
        reason="The precision hinge pin is intentionally captured inside the rotating hinge barrel proxy.",
    )
    ctx.allow_overlap(
        central,
        link_1,
        elem_a="hinge_pin_1",
        elem_b="hinge_barrel",
        reason="The precision hinge pin is intentionally captured inside the rotating hinge barrel proxy.",
    )

    ctx.check(
        "wing hinges expose calibrated stops",
        fold_0.motion_limits is not None
        and fold_1.motion_limits is not None
        and abs(fold_0.motion_limits.lower - 0.0) < 1e-6
        and abs(fold_1.motion_limits.lower - 0.0) < 1e-6
        and abs(fold_0.motion_limits.upper - 1.55) < 1e-6
        and abs(fold_1.motion_limits.upper - 1.55) < 1e-6,
        details=f"fold_0={fold_0.motion_limits}, fold_1={fold_1.motion_limits}",
    )

    ctx.expect_gap(
        link_0,
        central,
        axis="y",
        min_gap=0.004,
        max_gap=0.012,
        positive_elem="hinge_barrel",
        negative_elem="hinge_carrier_0",
        name="positive hinge has controlled carrier gap",
    )
    ctx.expect_within(
        central,
        link_0,
        axes="yz",
        inner_elem="hinge_pin_0",
        outer_elem="hinge_barrel",
        margin=0.0005,
        name="positive hinge pin is coaxially captured",
    )
    ctx.expect_overlap(
        central,
        link_0,
        axes="x",
        min_overlap=1.05,
        elem_a="hinge_pin_0",
        elem_b="hinge_barrel",
        name="positive hinge pin spans barrel length",
    )
    ctx.expect_gap(
        central,
        link_1,
        axis="y",
        min_gap=0.004,
        max_gap=0.012,
        positive_elem="hinge_carrier_1",
        negative_elem="hinge_barrel",
        name="negative hinge has controlled carrier gap",
    )
    ctx.expect_within(
        central,
        link_1,
        axes="yz",
        inner_elem="hinge_pin_1",
        outer_elem="hinge_barrel",
        margin=0.0005,
        name="negative hinge pin is coaxially captured",
    )
    ctx.expect_overlap(
        central,
        link_1,
        axes="x",
        min_overlap=1.05,
        elem_a="hinge_pin_1",
        elem_b="hinge_barrel",
        name="negative hinge pin spans barrel length",
    )
    ctx.expect_contact(
        wing_0,
        link_0,
        elem_a="root_rail",
        elem_b="leaf_bar",
        contact_tol=0.0015,
        name="positive wing is bolted to hinge link",
    )
    ctx.expect_contact(
        wing_1,
        link_1,
        elem_a="root_rail",
        elem_b="leaf_bar",
        contact_tol=0.0015,
        name="negative wing is bolted to hinge link",
    )
    ctx.expect_gap(
        knob_0,
        central,
        axis="y",
        max_gap=0.001,
        max_penetration=0.0,
        positive_elem="knob_body",
        negative_elem="index_plate_0",
        name="positive index knob seats on index plate",
    )
    ctx.expect_gap(
        central,
        knob_1,
        axis="y",
        max_gap=0.001,
        max_penetration=0.0,
        positive_elem="index_plate_1",
        negative_elem="knob_body",
        name="negative index knob seats on index plate",
    )

    rest_0 = ctx.part_world_aabb(wing_0)
    rest_1 = ctx.part_world_aabb(wing_1)
    with ctx.pose({fold_0: 1.55, fold_1: 1.55}):
        folded_0 = ctx.part_world_aabb(wing_0)
        folded_1 = ctx.part_world_aabb(wing_1)

    ctx.check(
        "wings fold upward about their hinge lines",
        rest_0 is not None
        and rest_1 is not None
        and folded_0 is not None
        and folded_1 is not None
        and folded_0[1][2] > rest_0[1][2] + 0.40
        and folded_1[1][2] > rest_1[1][2] + 0.40
        and folded_0[1][1] < rest_0[1][1] - 0.38
        and folded_1[0][1] > rest_1[0][1] + 0.38,
        details=f"rest_0={rest_0}, folded_0={folded_0}, rest_1={rest_1}, folded_1={folded_1}",
    )

    return ctx.report()


object_model = build_object_model()
