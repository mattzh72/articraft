from __future__ import annotations

from math import pi

from sdk import (
    ArticulatedObject,
    ArticulationType,
    Box,
    Cylinder,
    MotionLimits,
    MotionProperties,
    Origin,
    TestContext,
    TestReport,
)


def _box(part, name, size, xyz, material):
    part.visual(Box(size), origin=Origin(xyz=xyz), material=material, name=name)


def _cylinder(part, name, radius, length, xyz, axis, material):
    if axis == "x":
        rpy = (0.0, pi / 2.0, 0.0)
    elif axis == "y":
        rpy = (pi / 2.0, 0.0, 0.0)
    else:
        rpy = (0.0, 0.0, 0.0)
    part.visual(
        Cylinder(radius=radius, length=length),
        origin=Origin(xyz=xyz, rpy=rpy),
        material=material,
        name=name,
    )


def _bolt_heads(part, prefix, centers, radius, height, z_base, material):
    for index, (x, y) in enumerate(centers):
        _cylinder(
            part,
            f"{prefix}_{index}",
            radius,
            height,
            (x, y, z_base + height / 2.0),
            "z",
            material,
        )


def _thread_rings(part, prefix, axis, coordinates, radius, length, fixed_xyz, material):
    for index, value in enumerate(coordinates):
        if axis == "x":
            xyz = (value, fixed_xyz[1], fixed_xyz[2])
        elif axis == "y":
            xyz = (fixed_xyz[0], value, fixed_xyz[2])
        else:
            xyz = (fixed_xyz[0], fixed_xyz[1], value)
        _cylinder(part, f"{prefix}_{index}", radius, length, xyz, axis, material)


def build_object_model() -> ArticulatedObject:
    model = ArticulatedObject(name="xyz_cartesian_stage_study")

    cast_iron = model.material("blued_cast_iron", color=(0.12, 0.135, 0.145, 1.0))
    hard_black = model.material("black_oxide", color=(0.02, 0.022, 0.024, 1.0))
    rail_steel = model.material("ground_rail_steel", color=(0.72, 0.74, 0.72, 1.0))
    brushed = model.material("brushed_aluminum", color=(0.48, 0.51, 0.52, 1.0))
    cover_blue = model.material("removable_cover_blue", color=(0.08, 0.17, 0.24, 1.0))
    datum_green = model.material("datum_ground_faces", color=(0.24, 0.38, 0.28, 1.0))
    brass = model.material("oiled_bronze_bushings", color=(0.62, 0.45, 0.18, 1.0))

    # Root: a low, wide fabricated bed with outriggers, two Y rails, guards,
    # screw supports, and small machined datum faces.
    base = model.part("base")
    _box(base, "bed_plate", (0.86, 1.02, 0.060), (0.0, 0.0, 0.065), cast_iron)
    _box(base, "underside_rib", (0.72, 0.84, 0.035), (0.0, 0.0, 0.025), cast_iron)
    for i, y in enumerate((-0.43, 0.43)):
        _box(base, f"outrigger_{i}", (1.04, 0.075, 0.045), (0.0, y, 0.036), cast_iron)
    for i, (x, y) in enumerate(((-0.46, -0.43), (0.46, -0.43), (-0.46, 0.43), (0.46, 0.43))):
        _cylinder(base, f"leveling_foot_{i}", 0.036, 0.025, (x, y, 0.0125), "z", hard_black)
        _cylinder(base, f"foot_pad_{i}", 0.046, 0.006, (x, y, 0.003), "z", hard_black)

    for i, x in enumerate((-0.24, 0.24)):
        _box(base, f"y_rail_pad_{i}", (0.076, 0.86, 0.012), (x, 0.0, 0.101), brushed)
        _box(base, f"y_rail_{i}", (0.045, 0.84, 0.035), (x, 0.0, 0.1245), rail_steel)
    for i, x in enumerate((-0.325, 0.325)):
        _box(base, f"y_guard_{i}", (0.030, 0.91, 0.050), (x, 0.0, 0.120), hard_black)

    _cylinder(base, "y_ballscrew", 0.014, 0.84, (0.0, 0.0, 0.152), "y", rail_steel)
    _thread_rings(
        base,
        "y_thread",
        "y",
        [v * 0.052 for v in range(-7, 8)],
        0.0155,
        0.005,
        (0.0, 0.0, 0.152),
        hard_black,
    )
    for i, y in enumerate((-0.44, 0.44)):
        _box(base, f"y_screw_support_{i}", (0.070, 0.045, 0.062), (0.0, y, 0.126), cast_iron)
        _cylinder(base, f"y_bearing_cap_{i}", 0.026, 0.014, (0.0, y, 0.152), "y", brass)
    _box(base, "y_motor_mount", (0.145, 0.080, 0.076), (0.0, -0.555, 0.133), hard_black)
    _cylinder(base, "y_motor_coupler", 0.025, 0.040, (0.0, -0.506, 0.152), "y", brass)

    _box(base, "x_datum_face", (0.008, 0.46, 0.046), (-0.434, 0.08, 0.118), datum_green)
    _box(base, "y_datum_face", (0.36, 0.008, 0.046), (-0.12, -0.514, 0.118), datum_green)
    _bolt_heads(
        base,
        "base_socket",
        [(-0.35, -0.34), (0.35, -0.34), (-0.35, 0.34), (0.35, 0.34)],
        0.012,
        0.008,
        0.095,
        hard_black,
    )

    # Y moving saddle.  Its lower trucks hover just above the Y rails; the top
    # face carries the full X guide set, screw, guard strips, and datum keys.
    y_carrier = model.part("y_carrier")
    _box(y_carrier, "y_plate", (0.54, 0.36, 0.040), (0.0, 0.0, 0.020), brushed)
    for i, (x, y) in enumerate(((-0.24, -0.12), (-0.24, 0.12), (0.24, -0.12), (0.24, 0.12))):
        _box(y_carrier, f"y_bearing_{i}", (0.090, 0.095, 0.048), (x, y, -0.022), hard_black)
        _cylinder(y_carrier, f"y_bearing_bolt_{i}", 0.008, 0.006, (x, y, 0.043), "z", hard_black)
    for i, y in enumerate((-0.12, 0.12)):
        _box(y_carrier, f"x_rail_pad_{i}", (0.48, 0.045, 0.015), (0.0, y, 0.0475), brushed)
        _box(y_carrier, f"x_rail_{i}", (0.46, 0.032, 0.030), (0.0, y, 0.070), rail_steel)
        _box(y_carrier, f"x_guard_{i}", (0.50, 0.020, 0.046), (0.0, y * 1.25, 0.063), hard_black)
    _cylinder(y_carrier, "x_ballscrew", 0.011, 0.47, (0.0, 0.0, 0.080), "x", rail_steel)
    _thread_rings(
        y_carrier,
        "x_thread",
        "x",
        [v * 0.041 for v in range(-5, 6)],
        0.0125,
        0.004,
        (0.0, 0.0, 0.080),
        hard_black,
    )
    for i, x in enumerate((-0.245, 0.245)):
        _box(y_carrier, f"x_screw_support_{i}", (0.035, 0.060, 0.054), (x, 0.0, 0.060), cast_iron)
        _cylinder(y_carrier, f"x_bearing_cap_{i}", 0.020, 0.010, (x, 0.0, 0.080), "x", brass)
    _box(y_carrier, "y_nut_block", (0.105, 0.080, 0.074), (0.0, 0.0, -0.037), brass)
    _box(y_carrier, "x_datum_key", (0.180, 0.012, 0.012), (-0.10, -0.184, 0.046), datum_green)

    # X carriage with a rigid vertical Z-column frame bolted to it.  The Z rails
    # and vertical screw are exposed on the front face.
    x_carrier = model.part("x_carrier")
    _box(x_carrier, "x_plate", (0.32, 0.32, 0.035), (0.0, 0.0, 0.0175), brushed)
    for i, (x, y) in enumerate(((-0.070, -0.12), (0.070, -0.12), (-0.070, 0.12), (0.070, 0.12))):
        _box(x_carrier, f"x_bearing_{i}", (0.090, 0.070, 0.040), (x, y, -0.008), hard_black)
        _cylinder(x_carrier, f"x_bearing_bolt_{i}", 0.007, 0.006, (x, y, 0.038), "z", hard_black)
    _box(x_carrier, "column_backplate", (0.22, 0.035, 0.640), (0.0, 0.095, 0.345), cast_iron)
    for i, x in enumerate((-0.105, 0.105)):
        _box(x_carrier, f"column_side_{i}", (0.035, 0.145, 0.620), (x, 0.030, 0.335), cast_iron)
    _box(x_carrier, "front_web", (0.22, 0.030, 0.560), (0.0, -0.0485, 0.345), cast_iron)
    _box(x_carrier, "top_crosshead", (0.29, 0.17, 0.050), (0.0, 0.035, 0.665), cast_iron)
    _box(x_carrier, "lower_crosshead", (0.29, 0.17, 0.045), (0.0, 0.030, 0.0575), cast_iron)
    for i, x in enumerate((-0.060, 0.060)):
        _box(x_carrier, f"z_rail_pad_{i}", (0.040, 0.018, 0.540), (x, -0.072, 0.340), brushed)
        _box(x_carrier, f"z_rail_{i}", (0.026, 0.026, 0.520), (x, -0.091, 0.340), rail_steel)
        _box(x_carrier, f"z_guard_{i}", (0.016, 0.020, 0.560), (x * 2.0, -0.048, 0.345), hard_black)
    _cylinder(x_carrier, "z_ballscrew", 0.010, 0.53, (0.0, -0.086, 0.345), "z", rail_steel)
    _thread_rings(
        x_carrier,
        "z_thread",
        "z",
        [0.095 + v * 0.043 for v in range(12)],
        0.0115,
        0.004,
        (0.0, -0.086, 0.0),
        hard_black,
    )
    for i, z in enumerate((0.075, 0.615)):
        _box(x_carrier, f"z_screw_support_{i}", (0.072, 0.042, 0.032), (0.0, -0.084, z), cast_iron)
        _cylinder(x_carrier, f"z_bearing_cap_{i}", 0.018, 0.010, (0.0, -0.086, z), "z", brass)
    _box(x_carrier, "z_motor_mount", (0.105, 0.080, 0.060), (0.0, 0.040, 0.720), hard_black)
    _box(x_carrier, "z_column_datum", (0.035, 0.008, 0.210), (0.125, -0.046, 0.285), datum_green)

    # Z moving ram.  It is a compact carriage plate with four bearing shoes and
    # a bolted front datum/tooling face, not a product-like housing.
    z_carrier = model.part("z_carrier")
    _box(z_carrier, "z_ram", (0.160, 0.030, 0.360), (0.0, 0.0, 0.180), brushed)
    for i, (x, z) in enumerate(((-0.060, 0.105), (0.060, 0.105), (-0.060, 0.285), (0.060, 0.285))):
        _box(z_carrier, f"z_bearing_{i}", (0.052, 0.024, 0.065), (x, 0.006, z), hard_black)
        _cylinder(z_carrier, f"z_bearing_bolt_{i}", 0.006, 0.006, (x, -0.016, z), "y", hard_black)
    _box(z_carrier, "front_datum_plate", (0.205, 0.024, 0.165), (0.0, -0.028, 0.165), datum_green)
    _box(z_carrier, "tooling_flange", (0.240, 0.030, 0.045), (0.0, -0.032, 0.075), hard_black)
    _cylinder(z_carrier, "z_nut_boss", 0.030, 0.060, (0.0, 0.030, 0.190), "y", brass)
    _bolt_heads(
        z_carrier,
        "tooling_socket",
        [(-0.075, -0.044), (0.075, -0.044), (-0.075, -0.044), (0.075, -0.044)],
        0.006,
        0.004,
        0.200,
        hard_black,
    )

    # Thin removable access covers are separate fixed links so the study reads
    # like a maintainable mechanical assembly.
    y_cover = model.part("y_cover")
    _box(y_cover, "cover_plate", (0.080, 0.620, 0.012), (0.0, 0.0, 0.006), cover_blue)
    _box(y_cover, "cover_pull_lip", (0.014, 0.620, 0.018), (0.047, 0.0, 0.009), cover_blue)
    _bolt_heads(
        y_cover,
        "cover_screw",
        [(-0.025, -0.270), (-0.025, 0.270), (0.025, -0.270), (0.025, 0.270)],
        0.006,
        0.004,
        0.012,
        hard_black,
    )

    x_cover = model.part("x_cover")
    _box(x_cover, "cover_flange", (0.340, 0.020, 0.014), (0.0, -0.010, 0.007), cover_blue)
    _box(x_cover, "cover_panel", (0.340, 0.012, 0.075), (0.0, 0.006, 0.0375), cover_blue)
    _bolt_heads(
        x_cover,
        "cover_screw",
        [(-0.145, 0.012), (0.145, 0.012), (-0.045, 0.012), (0.045, 0.012)],
        0.005,
        0.004,
        0.075,
        hard_black,
    )

    z_cover = model.part("z_cover")
    _box(z_cover, "cover_panel", (0.165, 0.010, 0.320), (0.0, 0.005, 0.160), cover_blue)
    _box(z_cover, "cover_bottom_lip", (0.165, 0.020, 0.016), (0.0, 0.020, 0.008), cover_blue)
    _bolt_heads(
        z_cover,
        "cover_screw",
        [(-0.060, 0.012), (0.060, 0.012), (-0.060, 0.012), (0.060, 0.012)],
        0.005,
        0.004,
        0.275,
        hard_black,
    )

    model.articulation(
        "y_axis",
        ArticulationType.PRISMATIC,
        parent=base,
        child=y_carrier,
        origin=Origin(xyz=(0.0, -0.180, 0.188)),
        axis=(0.0, 1.0, 0.0),
        motion_limits=MotionLimits(effort=650.0, velocity=0.18, lower=0.0, upper=0.360),
        motion_properties=MotionProperties(damping=40.0, friction=6.0),
    )
    model.articulation(
        "x_axis",
        ArticulationType.PRISMATIC,
        parent=y_carrier,
        child=x_carrier,
        origin=Origin(xyz=(-0.120, 0.0, 0.113)),
        axis=(1.0, 0.0, 0.0),
        motion_limits=MotionLimits(effort=420.0, velocity=0.16, lower=0.0, upper=0.240),
        motion_properties=MotionProperties(damping=35.0, friction=5.0),
    )
    model.articulation(
        "z_axis",
        ArticulationType.PRISMATIC,
        parent=x_carrier,
        child=z_carrier,
        origin=Origin(xyz=(0.0, -0.122, 0.095)),
        axis=(0.0, 0.0, 1.0),
        motion_limits=MotionLimits(effort=700.0, velocity=0.10, lower=0.0, upper=0.180),
        motion_properties=MotionProperties(damping=55.0, friction=8.0),
    )
    model.articulation(
        "base_to_y_cover",
        ArticulationType.FIXED,
        parent=base,
        child=y_cover,
        origin=Origin(xyz=(0.385, 0.0, 0.095)),
    )
    model.articulation(
        "y_to_x_cover",
        ArticulationType.FIXED,
        parent=y_carrier,
        child=x_cover,
        origin=Origin(xyz=(0.0, 0.180, 0.040)),
    )
    model.articulation(
        "x_to_z_cover",
        ArticulationType.FIXED,
        parent=x_carrier,
        child=z_cover,
        origin=Origin(xyz=(0.0, 0.1125, 0.170)),
    )

    return model


def run_tests() -> TestReport:
    ctx = TestContext(object_model)
    base = object_model.get_part("base")
    y_carrier = object_model.get_part("y_carrier")
    x_carrier = object_model.get_part("x_carrier")
    z_carrier = object_model.get_part("z_carrier")
    y_axis = object_model.get_articulation("y_axis")
    x_axis = object_model.get_articulation("x_axis")
    z_axis = object_model.get_articulation("z_axis")

    ctx.allow_overlap(
        base,
        y_carrier,
        elem_a="y_ballscrew",
        elem_b="y_nut_block",
        reason="The exposed Y ballscrew is intentionally captured through the bronze nut block on the moving saddle.",
    )
    ctx.allow_overlap(
        base,
        y_carrier,
        elem_a="y_thread_3",
        elem_b="y_nut_block",
        reason="The modeled screw thread ring intentionally enters the bronze saddle nut block.",
    )
    ctx.allow_overlap(
        base,
        y_carrier,
        elem_a="y_thread_4",
        elem_b="y_nut_block",
        reason="The modeled screw thread ring intentionally enters the bronze saddle nut block.",
    )
    ctx.allow_overlap(
        x_carrier,
        z_carrier,
        elem_a="z_ballscrew",
        elem_b="z_nut_boss",
        reason="The vertical ballscrew is intentionally represented as passing through the ram nut boss.",
    )
    ctx.expect_within(
        base,
        y_carrier,
        axes="xz",
        inner_elem="y_ballscrew",
        outer_elem="y_nut_block",
        margin=0.002,
        name="Y ballscrew is centered through the saddle nut",
    )
    ctx.expect_overlap(
        base,
        y_carrier,
        axes="y",
        elem_a="y_ballscrew",
        elem_b="y_nut_block",
        min_overlap=0.060,
        name="Y nut remains engaged with the ballscrew",
    )
    ctx.expect_overlap(
        base,
        y_carrier,
        axes="y",
        elem_a="y_thread_3",
        elem_b="y_nut_block",
        min_overlap=0.004,
        name="Y screw thread ring is inside the nut block",
    )
    ctx.expect_overlap(
        base,
        y_carrier,
        axes="y",
        elem_a="y_thread_4",
        elem_b="y_nut_block",
        min_overlap=0.004,
        name="Adjacent Y screw thread ring is inside the nut block",
    )
    ctx.expect_within(
        x_carrier,
        z_carrier,
        axes="xy",
        inner_elem="z_ballscrew",
        outer_elem="z_nut_boss",
        margin=0.002,
        name="Z ballscrew is centered through the ram nut boss",
    )
    ctx.expect_overlap(
        x_carrier,
        z_carrier,
        axes="z",
        elem_a="z_ballscrew",
        elem_b="z_nut_boss",
        min_overlap=0.040,
        name="Z nut boss remains engaged with the ballscrew",
    )

    ctx.expect_gap(
        y_carrier,
        base,
        axis="z",
        positive_elem="y_bearing_0",
        negative_elem="y_rail_0",
        max_gap=0.001,
        max_penetration=0.00001,
        name="Y bearing truck rides just above the exposed rail",
    )
    ctx.expect_gap(
        x_carrier,
        y_carrier,
        axis="z",
        positive_elem="x_bearing_0",
        negative_elem="x_rail_0",
        max_gap=0.001,
        max_penetration=0.00001,
        name="X bearing truck rides just above the exposed rail",
    )
    ctx.expect_gap(
        x_carrier,
        z_carrier,
        axis="y",
        positive_elem="z_rail_0",
        negative_elem="z_bearing_0",
        max_gap=0.001,
        max_penetration=0.00001,
        name="Z bearing shoe is clear of the front guide rail",
    )

    ctx.expect_contact(
        "y_cover",
        base,
        elem_a="cover_plate",
        elem_b="bed_plate",
        contact_tol=0.001,
        name="Y access cover seats on the base plate",
    )
    ctx.expect_contact(
        "x_cover",
        y_carrier,
        elem_a="cover_flange",
        elem_b="y_plate",
        contact_tol=0.001,
        name="X access cover flange seats on the Y saddle",
    )
    ctx.expect_contact(
        "z_cover",
        x_carrier,
        elem_a="cover_panel",
        elem_b="column_backplate",
        contact_tol=0.001,
        name="Z access cover mounts to the column backplate",
    )

    y_rest = ctx.part_world_position(y_carrier)
    with ctx.pose({y_axis: 0.300}):
        y_extended = ctx.part_world_position(y_carrier)
    ctx.check(
        "Y stage translates along the machine Y axis",
        y_rest is not None and y_extended is not None and y_extended[1] > y_rest[1] + 0.25,
        details=f"rest={y_rest}, extended={y_extended}",
    )

    x_rest = ctx.part_world_position(x_carrier)
    with ctx.pose({x_axis: 0.200}):
        x_extended = ctx.part_world_position(x_carrier)
    ctx.check(
        "X stage translates along the cross-slide X axis",
        x_rest is not None and x_extended is not None and x_extended[0] > x_rest[0] + 0.16,
        details=f"rest={x_rest}, extended={x_extended}",
    )

    z_rest = ctx.part_world_position(z_carrier)
    with ctx.pose({z_axis: 0.150}):
        z_extended = ctx.part_world_position(z_carrier)
    ctx.check(
        "Z ram translates vertically",
        z_rest is not None and z_extended is not None and z_extended[2] > z_rest[2] + 0.12,
        details=f"rest={z_rest}, extended={z_extended}",
    )

    return ctx.report()


object_model = build_object_model()
