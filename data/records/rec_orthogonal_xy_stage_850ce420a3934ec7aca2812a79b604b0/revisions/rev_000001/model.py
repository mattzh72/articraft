from __future__ import annotations

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


def _box(part, name: str, size, xyz, material):
    part.visual(Box(size), origin=Origin(xyz=xyz), material=material, name=name)


def _cylinder(part, name: str, radius: float, length: float, xyz, material):
    part.visual(Cylinder(radius=radius, length=length), origin=Origin(xyz=xyz), material=material, name=name)


def build_object_model() -> ArticulatedObject:
    model = ArticulatedObject(name="precision_slide_table")

    cast = model.material("satin_blackened_cast_iron", rgba=(0.10, 0.12, 0.13, 1.0))
    blue_gray = model.material("blue_gray_machined_body", rgba=(0.18, 0.22, 0.25, 1.0))
    machined = model.material("ground_machined_steel", rgba=(0.62, 0.64, 0.62, 1.0))
    bright = model.material("polished_bearing_land", rgba=(0.82, 0.84, 0.80, 1.0))
    black = model.material("black_oxide_hardware", rgba=(0.015, 0.016, 0.018, 1.0))
    brass = model.material("bronze_gib_strips", rgba=(0.72, 0.56, 0.30, 1.0))
    pad_mat = model.material("matte_fixture_pad", rgba=(0.27, 0.30, 0.32, 1.0))

    base = model.part("base_plate")
    _box(base, "base_plate", (0.90, 0.44, 0.055), (0.0, 0.0, 0.0275), cast)
    _box(base, "center_oil_groove", (0.74, 0.014, 0.003), (0.0, 0.0, 0.0565), black)

    _box(base, "x_rail_rib_0", (0.80, 0.048, 0.025), (0.0, -0.130, 0.0675), machined)
    base.visual(Box((0.78, 0.022, 0.006)), origin=Origin(xyz=(0.0, -0.130, 0.083)), material=bright, name="x_land_0")
    _box(base, "x_rail_rib_1", (0.80, 0.048, 0.025), (0.0, 0.130, 0.0675), machined)
    base.visual(Box((0.78, 0.022, 0.006)), origin=Origin(xyz=(0.0, 0.130, 0.083)), material=bright, name="x_land_1")
    for i, y in enumerate((-0.130, 0.130)):
        for j, x in enumerate((-0.30, -0.15, 0.0, 0.15, 0.30)):
            _cylinder(base, f"x_rail_screw_{i}_{j}", 0.006, 0.003, (x, y, 0.0870), black)

    _box(base, "x_stop_neg", (0.028, 0.36, 0.060), (-0.425, 0.0, 0.085), machined)
    _box(base, "x_stop_pos", (0.028, 0.36, 0.060), (0.425, 0.0, 0.085), machined)
    base.visual(Box((0.006, 0.11, 0.020)), origin=Origin(xyz=(-0.409, 0.0, 0.092)), material=black, name="x_bumper_neg")
    base.visual(Box((0.006, 0.11, 0.020)), origin=Origin(xyz=(0.409, 0.0, 0.092)), material=black, name="x_bumper_pos")

    for j, x in enumerate((-0.34, -0.20, 0.20, 0.34)):
        for k, y in enumerate((-0.190, 0.190)):
            _cylinder(base, f"base_screw_{j}_{k}", 0.008, 0.004, (x, y, 0.057), black)

    lower = model.part("lower_carriage")
    lower.visual(Box((0.54, 0.30, 0.050)), origin=Origin(xyz=(0.0, 0.0, 0.043)), material=blue_gray, name="lower_body")
    _box(lower, "lower_cover", (0.50, 0.26, 0.008), (0.0, 0.0, 0.072), machined)
    lower.visual(Box((0.50, 0.052, 0.018)), origin=Origin(xyz=(0.0, -0.130, 0.009)), material=machined, name="lower_bearing_0")
    _box(lower, "gib_strip_0", (0.45, 0.012, 0.020), (0.0, -0.1534, 0.033), brass)
    lower.visual(Box((0.50, 0.052, 0.018)), origin=Origin(xyz=(0.0, 0.130, 0.009)), material=machined, name="lower_bearing_1")
    _box(lower, "gib_strip_1", (0.45, 0.012, 0.020), (0.0, 0.1534, 0.033), brass)

    lower.visual(Box((0.032, 0.42, 0.022)), origin=Origin(xyz=(-0.095, 0.0, 0.079)), material=bright, name="cross_way_0")
    lower.visual(Box((0.032, 0.42, 0.022)), origin=Origin(xyz=(0.095, 0.0, 0.079)), material=bright, name="cross_way_1")

    _box(lower, "y_stop_neg", (0.29, 0.026, 0.055), (0.0, -0.221, 0.0955), machined)
    _box(lower, "y_stop_pos", (0.29, 0.026, 0.055), (0.0, 0.221, 0.0955), machined)
    lower.visual(Box((0.10, 0.006, 0.018)), origin=Origin(xyz=(0.0, -0.207, 0.094)), material=black, name="y_bumper_neg")
    lower.visual(Box((0.10, 0.006, 0.018)), origin=Origin(xyz=(0.0, 0.207, 0.094)), material=black, name="y_bumper_pos")

    for j, x in enumerate((-0.22, -0.08, 0.08, 0.22)):
        for k, y in enumerate((-0.100, 0.100)):
            _cylinder(lower, f"lower_cap_screw_{j}_{k}", 0.0055, 0.004, (x, y, 0.070), black)

    cross = model.part("cross_slide")
    cross.visual(Box((0.36, 0.26, 0.045)), origin=Origin(xyz=(0.0, 0.0, 0.0405)), material=cast, name="cross_body")
    cross.visual(Box((0.32, 0.22, 0.008)), origin=Origin(xyz=(0.0, 0.0, 0.067)), material=machined, name="cross_cover")
    cross.visual(Box((0.052, 0.24, 0.018)), origin=Origin(xyz=(-0.095, 0.0, 0.009)), material=machined, name="upper_bearing_0")
    cross.visual(Box((0.052, 0.24, 0.018)), origin=Origin(xyz=(0.095, 0.0, 0.009)), material=machined, name="upper_bearing_1")
    for i, x in enumerate((-0.185, 0.185)):
        _box(cross, f"side_keeper_{i}", (0.010, 0.22, 0.025), (x, 0.0, 0.032), brass)
    for j, x in enumerate((-0.160, 0.160)):
        for k, y in enumerate((-0.105, 0.105)):
            _cylinder(cross, f"cross_screw_{j}_{k}", 0.005, 0.003, (x, y, 0.0725), black)

    pad = model.part("instrument_pad")
    pad.visual(Box((0.28, 0.20, 0.026)), origin=Origin(xyz=(0.0, 0.0, 0.013)), material=pad_mat, name="pad_plate")
    _box(pad, "pad_x_slot", (0.22, 0.012, 0.003), (0.0, 0.0, 0.0275), black)
    _box(pad, "pad_y_slot", (0.012, 0.15, 0.003), (0.0, 0.0, 0.0280), black)
    _cylinder(pad, "center_locator", 0.028, 0.006, (0.0, 0.0, 0.029), bright)
    for j, x in enumerate((-0.105, 0.105)):
        for k, y in enumerate((-0.065, 0.065)):
            _cylinder(pad, f"pad_socket_{j}_{k}", 0.006, 0.004, (x, y, 0.027), black)

    model.articulation(
        "base_to_lower",
        ArticulationType.PRISMATIC,
        parent=base,
        child=lower,
        origin=Origin(xyz=(0.0, 0.0, 0.086)),
        axis=(1.0, 0.0, 0.0),
        motion_limits=MotionLimits(effort=300.0, velocity=0.08, lower=-0.13, upper=0.13),
    )
    model.articulation(
        "lower_to_cross",
        ArticulationType.PRISMATIC,
        parent=lower,
        child=cross,
        origin=Origin(xyz=(0.0, 0.0, 0.090)),
        axis=(0.0, 1.0, 0.0),
        motion_limits=MotionLimits(effort=180.0, velocity=0.06, lower=-0.065, upper=0.065),
    )
    model.articulation(
        "cross_to_pad",
        ArticulationType.FIXED,
        parent=cross,
        child=pad,
        origin=Origin(xyz=(0.0, 0.0, 0.071)),
    )

    return model


def run_tests() -> TestReport:
    ctx = TestContext(object_model)
    base = object_model.get_part("base_plate")
    lower = object_model.get_part("lower_carriage")
    cross = object_model.get_part("cross_slide")
    pad = object_model.get_part("instrument_pad")
    lower_joint = object_model.get_articulation("base_to_lower")
    cross_joint = object_model.get_articulation("lower_to_cross")

    ctx.check("lower carriage uses an X prismatic way", lower_joint.axis == (1.0, 0.0, 0.0))
    ctx.check("cross slide uses a Y prismatic way", cross_joint.axis == (0.0, 1.0, 0.0))

    for i in (0, 1):
        ctx.expect_contact(
            lower,
            base,
            elem_a=f"lower_bearing_{i}",
            elem_b=f"x_land_{i}",
            contact_tol=0.0005,
            name=f"lower bearing {i} rides on base land",
        )
        ctx.expect_overlap(
            lower,
            base,
            axes="xy",
            elem_a=f"lower_bearing_{i}",
            elem_b=f"x_land_{i}",
            min_overlap=0.015,
            name=f"lower bearing {i} remains over base way",
        )
        ctx.expect_contact(
            cross,
            lower,
            elem_a=f"upper_bearing_{i}",
            elem_b=f"cross_way_{i}",
            contact_tol=0.0005,
            name=f"upper bearing {i} rides on cross way",
        )
        ctx.expect_overlap(
            cross,
            lower,
            axes="xy",
            elem_a=f"upper_bearing_{i}",
            elem_b=f"cross_way_{i}",
            min_overlap=0.015,
            name=f"upper bearing {i} remains over cross way",
        )

    ctx.expect_contact(
        pad,
        cross,
        elem_a="pad_plate",
        elem_b="cross_cover",
        contact_tol=0.0005,
        name="instrument pad is seated on the cross slide cover",
    )

    rest_lower = ctx.part_world_position(lower)
    with ctx.pose({lower_joint: lower_joint.motion_limits.upper}):
        upper_lower = ctx.part_world_position(lower)
        ctx.expect_gap(
            base,
            lower,
            axis="x",
            positive_elem="x_bumper_pos",
            negative_elem="lower_body",
            min_gap=0.003,
            max_gap=0.020,
            name="positive lower travel stops before the end bumper",
        )
        ctx.expect_overlap(
            lower,
            base,
            axes="xy",
            elem_a="lower_bearing_0",
            elem_b="x_land_0",
            min_overlap=0.015,
            name="lower carriage remains supported at positive X travel",
        )
    with ctx.pose({lower_joint: lower_joint.motion_limits.lower}):
        lower_lower = ctx.part_world_position(lower)
        ctx.expect_gap(
            lower,
            base,
            axis="x",
            positive_elem="lower_body",
            negative_elem="x_bumper_neg",
            min_gap=0.003,
            max_gap=0.020,
            name="negative lower travel stops before the end bumper",
        )
        ctx.expect_overlap(
            lower,
            base,
            axes="xy",
            elem_a="lower_bearing_1",
            elem_b="x_land_1",
            min_overlap=0.015,
            name="lower carriage remains supported at negative X travel",
        )

    ctx.check(
        "lower carriage motion is along X only",
        rest_lower is not None
        and upper_lower is not None
        and lower_lower is not None
        and upper_lower[0] > rest_lower[0] + 0.12
        and lower_lower[0] < rest_lower[0] - 0.12
        and abs(upper_lower[1] - rest_lower[1]) < 1e-6
        and abs(lower_lower[1] - rest_lower[1]) < 1e-6,
        details=f"rest={rest_lower}, upper={upper_lower}, lower={lower_lower}",
    )

    rest_cross = ctx.part_world_position(cross)
    with ctx.pose({cross_joint: cross_joint.motion_limits.upper}):
        upper_cross = ctx.part_world_position(cross)
        ctx.expect_gap(
            lower,
            cross,
            axis="y",
            positive_elem="y_bumper_pos",
            negative_elem="cross_body",
            min_gap=0.003,
            max_gap=0.020,
            name="positive cross travel stops before the bumper",
        )
        ctx.expect_overlap(
            cross,
            lower,
            axes="xy",
            elem_a="upper_bearing_0",
            elem_b="cross_way_0",
            min_overlap=0.015,
            name="cross slide remains supported at positive Y travel",
        )
    with ctx.pose({cross_joint: cross_joint.motion_limits.lower}):
        lower_cross = ctx.part_world_position(cross)
        ctx.expect_gap(
            cross,
            lower,
            axis="y",
            positive_elem="cross_body",
            negative_elem="y_bumper_neg",
            min_gap=0.003,
            max_gap=0.020,
            name="negative cross travel stops before the bumper",
        )
        ctx.expect_overlap(
            cross,
            lower,
            axes="xy",
            elem_a="upper_bearing_1",
            elem_b="cross_way_1",
            min_overlap=0.015,
            name="cross slide remains supported at negative Y travel",
        )

    ctx.check(
        "cross slide motion is along Y only",
        rest_cross is not None
        and upper_cross is not None
        and lower_cross is not None
        and upper_cross[1] > rest_cross[1] + 0.06
        and lower_cross[1] < rest_cross[1] - 0.06
        and abs(upper_cross[0] - rest_cross[0]) < 1e-6
        and abs(lower_cross[0] - rest_cross[0]) < 1e-6,
        details=f"rest={rest_cross}, upper={upper_cross}, lower={lower_cross}",
    )

    return ctx.report()


object_model = build_object_model()
