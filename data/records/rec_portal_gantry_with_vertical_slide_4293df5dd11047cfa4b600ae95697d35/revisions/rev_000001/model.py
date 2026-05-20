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


def build_object_model() -> ArticulatedObject:
    model = ArticulatedObject(name="tabletop_portal_gantry")

    aluminum = model.material("clear_anodized_aluminum", rgba=(0.58, 0.61, 0.62, 1.0))
    dark = model.material("black_powder_coat", rgba=(0.04, 0.045, 0.05, 1.0))
    rail = model.material("ground_steel_rails", rgba=(0.82, 0.84, 0.82, 1.0))
    blue = model.material("blue_carriage_covers", rgba=(0.05, 0.15, 0.38, 1.0))
    bolt = model.material("dark_socket_bolts", rgba=(0.015, 0.015, 0.014, 1.0))
    rubber = model.material("black_rubber_bumpers", rgba=(0.01, 0.01, 0.01, 1.0))

    frame = model.part("frame")

    def add_box(part, name, size, xyz, material):
        part.visual(Box(size), origin=Origin(xyz=xyz), material=material, name=name)

    def add_y_bolt(part, name, xyz, radius=0.010, length=0.006):
        # Cylinders are local-Z; rotate so the screw head axis points out of the front face.
        part.visual(
            Cylinder(radius=radius, length=length),
            origin=Origin(xyz=xyz, rpy=(pi / 2.0, 0.0, 0.0)),
            material=bolt,
            name=name,
        )

    # One rooted, physically connected portal frame: base plates, two uprights,
    # a spanning extrusion, gussets, front linear rails and hard stops.
    add_box(frame, "top_beam", (1.68, 0.13, 0.13), (0.0, 0.0, 0.72), aluminum)
    for i, x in enumerate((-0.76, 0.76)):
        add_box(frame, f"upright_{i}", (0.10, 0.18, 0.68), (x, 0.0, 0.34), dark)
        add_box(frame, f"base_plate_{i}", (0.34, 0.26, 0.025), (x, 0.0, 0.0125), dark)
        add_box(frame, f"leg_face_plate_{i}", (0.135, 0.014, 0.52), (x, -0.097, 0.33), aluminum)
        add_box(frame, f"rear_leg_face_{i}", (0.135, 0.014, 0.52), (x, 0.097, 0.33), aluminum)
        # Tabletop rubber leveling feet, represented as threaded pads seated under the base plates.
        for j, y in enumerate((-0.085, 0.085)):
            for k, dx in enumerate((-0.105, 0.105)):
                frame.visual(
                    Cylinder(radius=0.022, length=0.014),
                    origin=Origin(xyz=(x + dx, y, -0.007)),
                    material=rubber,
                    name=f"foot_{i}_{j}_{k}",
                )
        # Diagonal web braces make the uprights read as stiff machine side plates.
        for j, y in enumerate((-0.081, 0.081)):
            add_box(
                frame,
                f"diagonal_brace_{i}_{j}",
                (0.030, 0.018, 0.36),
                (x + (-0.040 if x > 0 else 0.040), y, 0.205),
                aluminum,
            )
            frame.visuals[-1].origin = Origin(
                xyz=(x + (-0.040 if x > 0 else 0.040), y, 0.205),
                rpy=(0.0, 0.48 if x > 0 else -0.48, 0.0),
            )

    # Front guide paths along the beam.  The rails sit proud of the beam front face.
    frame.visual(
        Box((1.25, 0.013, 0.018)),
        origin=Origin(xyz=(0.0, -0.0715, 0.750)),
        material=rail,
        name="beam_rail_upper",
    )
    frame.visual(
        Box((1.25, 0.013, 0.018)),
        origin=Origin(xyz=(0.0, -0.0715, 0.690)),
        material=rail,
        name="beam_rail_lower",
    )
    frame.visual(
        Box((0.040, 0.028, 0.094)),
        origin=Origin(xyz=(-0.625, -0.079, 0.720)),
        material=dark,
        name="x_stop_0",
    )
    frame.visual(
        Box((0.040, 0.028, 0.094)),
        origin=Origin(xyz=(0.625, -0.079, 0.720)),
        material=dark,
        name="x_stop_1",
    )
    for i, x in enumerate((-0.48, -0.32, -0.16, 0.0, 0.16, 0.32, 0.48)):
        add_y_bolt(frame, f"upper_rail_bolt_{i}", (x, -0.080, 0.750), radius=0.007, length=0.006)
        add_y_bolt(frame, f"lower_rail_bolt_{i}", (x, -0.080, 0.690), radius=0.007, length=0.006)
    for i, x in enumerate((-0.625, 0.625)):
        add_y_bolt(frame, f"x_stop_bolt_{i}_0", (x, -0.096, 0.744), radius=0.007, length=0.006)
        add_y_bolt(frame, f"x_stop_bolt_{i}_1", (x, -0.096, 0.696), radius=0.007, length=0.006)

    # Moving X carriage.  Its frame origin is on the beam guide datum, not at an
    # arbitrary world origin, so the prismatic axis follows the guide path.
    carriage = model.part("carriage")
    carriage.visual(
        Box((0.22, 0.036, 0.18)),
        origin=Origin(xyz=(0.0, -0.045, 0.0)),
        material=blue,
        name="carriage_cover",
    )
    carriage.visual(
        Box((0.160, 0.020, 0.026)),
        origin=Origin(xyz=(0.0, -0.019, 0.030)),
        material=dark,
        name="upper_bearing",
    )
    carriage.visual(
        Box((0.160, 0.020, 0.026)),
        origin=Origin(xyz=(0.0, -0.019, -0.030)),
        material=dark,
        name="lower_bearing",
    )
    add_box(carriage, "carriage_side_cap_0", (0.018, 0.042, 0.165), (-0.110, -0.044, 0.0), dark)
    add_box(carriage, "carriage_side_cap_1", (0.018, 0.042, 0.165), (0.110, -0.044, 0.0), dark)
    add_box(carriage, "z_backbone", (0.180, 0.040, 0.240), (0.0, -0.083, -0.210), blue)
    carriage.visual(
        Box((0.014, 0.011, 0.220)),
        origin=Origin(xyz=(-0.052, -0.1085, -0.210)),
        material=rail,
        name="z_fixed_rail_0",
    )
    carriage.visual(
        Box((0.014, 0.011, 0.220)),
        origin=Origin(xyz=(0.052, -0.1085, -0.210)),
        material=rail,
        name="z_fixed_rail_1",
    )
    add_box(carriage, "z_bearing_upper_0", (0.040, 0.012, 0.045), (-0.052, -0.110, -0.135), dark)
    carriage.visual(
        Box((0.040, 0.012, 0.045)),
        origin=Origin(xyz=(0.052, -0.110, -0.135)),
        material=dark,
        name="z_bearing_upper_1",
    )
    carriage.visual(
        Box((0.040, 0.012, 0.045)),
        origin=Origin(xyz=(-0.052, -0.110, -0.285)),
        material=dark,
        name="z_bearing_lower_0",
    )
    add_box(carriage, "z_bearing_lower_1", (0.040, 0.012, 0.045), (0.052, -0.110, -0.285), dark)
    carriage.visual(
        Box((0.034, 0.018, 0.050)),
        origin=Origin(xyz=(-0.052, -0.111, -0.135)),
        material=dark,
        name="ram_preload_pad_0",
    )
    add_box(carriage, "ram_preload_pad_1", (0.034, 0.018, 0.050), (0.052, -0.111, -0.135), dark)
    add_box(carriage, "ram_preload_pad_2", (0.034, 0.018, 0.050), (-0.052, -0.111, -0.285), dark)
    add_box(carriage, "ram_preload_pad_3", (0.034, 0.018, 0.050), (0.052, -0.111, -0.285), dark)
    add_box(carriage, "z_stop_upper", (0.034, 0.026, 0.024), (0.088, -0.101, -0.083), rubber)
    add_box(carriage, "z_stop_lower", (0.034, 0.026, 0.024), (0.088, -0.101, -0.336), rubber)
    for i, x in enumerate((-0.070, 0.070)):
        add_y_bolt(carriage, f"cover_bolt_{i}_0", (x, -0.064, 0.060), radius=0.006, length=0.005)
        add_y_bolt(carriage, f"cover_bolt_{i}_1", (x, -0.064, -0.060), radius=0.006, length=0.005)
    for side, x in enumerate((-0.052, 0.052)):
        for j, z in enumerate((-0.285, -0.210, -0.135)):
            add_y_bolt(carriage, f"z_rail_bolt_{side}_{j}", (x, -0.115, z), radius=0.005, length=0.005)

    x_axis = model.articulation(
        "x_axis",
        ArticulationType.PRISMATIC,
        parent=frame,
        child=carriage,
        origin=Origin(xyz=(-0.460, -0.074, 0.720)),
        axis=(1.0, 0.0, 0.0),
        motion_limits=MotionLimits(effort=180.0, velocity=0.55, lower=0.0, upper=0.920),
    )

    # Hanging Z ram.  It is a separate moving link, with its own visible guide
    # paths and a tool plate, carried by the carriage's vertical bearing stack.
    ram = model.part("z_ram")
    ram.visual(
        Box((0.120, 0.030, 0.560)),
        origin=Origin(xyz=(0.0, -0.010, -0.110)),
        material=dark,
        name="ram_plate",
    )
    ram.visual(
        Box((0.012, 0.012, 0.580)),
        origin=Origin(xyz=(-0.038, -0.031, -0.090)),
        material=rail,
        name="ram_rail_0",
    )
    ram.visual(
        Box((0.012, 0.012, 0.580)),
        origin=Origin(xyz=(0.038, -0.031, -0.090)),
        material=rail,
        name="ram_rail_1",
    )
    add_box(ram, "tool_mount", (0.135, 0.036, 0.060), (0.0, -0.014, -0.315), blue)
    ram.visual(
        Cylinder(radius=0.032, length=0.040),
        origin=Origin(xyz=(0.0, -0.014, -0.365), rpy=(pi / 2.0, 0.0, 0.0)),
        material=aluminum,
        name="tool_flange",
    )
    add_box(ram, "z_stop_tab", (0.026, 0.018, 0.034), (0.067, -0.024, -0.030), aluminum)
    for side, x in enumerate((-0.038, 0.038)):
        for j, z in enumerate((-0.230, -0.130, -0.030)):
            add_y_bolt(ram, f"ram_rail_bolt_{side}_{j}", (x, -0.038, z), radius=0.0045, length=0.004)

    model.articulation(
        "z_axis",
        ArticulationType.PRISMATIC,
        parent=carriage,
        child=ram,
        origin=Origin(xyz=(0.0, -0.125, -0.200)),
        axis=(0.0, 0.0, -1.0),
        motion_limits=MotionLimits(effort=120.0, velocity=0.30, lower=0.0, upper=0.240),
    )

    # Keep the local variables clearly used for readability in IDEs.
    _ = x_axis
    return model


def run_tests() -> TestReport:
    ctx = TestContext(object_model)
    frame = object_model.get_part("frame")
    carriage = object_model.get_part("carriage")
    ram = object_model.get_part("z_ram")
    x_axis = object_model.get_articulation("x_axis")
    z_axis = object_model.get_articulation("z_axis")

    # The X carriage rides just in front of the rail faces and stays between hard stops.
    with ctx.pose({x_axis: 0.0, z_axis: 0.0}):
        ctx.expect_gap(
            frame,
            carriage,
            axis="y",
            positive_elem="beam_rail_lower",
            negative_elem="lower_bearing",
            min_gap=0.003,
            max_gap=0.009,
            name="lower bearing clears lower beam rail",
        )
        ctx.expect_gap(
            carriage,
            frame,
            axis="x",
            positive_elem="carriage_cover",
            negative_elem="x_stop_0",
            min_gap=0.025,
            name="carriage clears left end stop",
        )
        ctx.expect_overlap(
            carriage,
            frame,
            axes="z",
            elem_a="upper_bearing",
            elem_b="beam_rail_upper",
            min_overlap=0.012,
            name="upper bearing is aligned with upper beam rail",
        )

    rest_x = ctx.part_world_position(carriage)
    with ctx.pose({x_axis: 0.920, z_axis: 0.0}):
        ctx.expect_gap(
            frame,
            carriage,
            axis="x",
            positive_elem="x_stop_1",
            negative_elem="carriage_cover",
            min_gap=0.025,
            name="carriage clears right end stop",
        )
        ctx.expect_gap(
            frame,
            carriage,
            axis="y",
            positive_elem="beam_rail_upper",
            negative_elem="upper_bearing",
            min_gap=0.003,
            max_gap=0.009,
            name="upper bearing clears upper beam rail at far end",
        )
        far_x = ctx.part_world_position(carriage)

    ctx.check(
        "x carriage translates along beam",
        rest_x is not None and far_x is not None and far_x[0] > rest_x[0] + 0.85,
        details=f"rest={rest_x}, far={far_x}",
    )

    # The ram hangs in front of the carriage guide stack and travels downward without clipping it.
    with ctx.pose({x_axis: 0.460, z_axis: 0.0}):
        ctx.expect_gap(
            carriage,
            ram,
            axis="y",
            positive_elem="z_bearing_lower_0",
            negative_elem="ram_plate",
            min_gap=0.003,
            max_gap=0.010,
            name="z ram clears fixed bearing stack",
        )
        ctx.expect_overlap(
            ram,
            carriage,
            axes="z",
            elem_a="ram_rail_0",
            elem_b="z_fixed_rail_0",
            min_overlap=0.12,
            name="z guide paths overlap while retracted",
        )
        ctx.expect_contact(
            carriage,
            ram,
            elem_a="ram_preload_pad_0",
            elem_b="ram_plate",
            contact_tol=0.00001,
            name="z ram is captured by preload pad",
        )
        rest_z = ctx.part_world_position(ram)

    with ctx.pose({x_axis: 0.460, z_axis: 0.240}):
        ctx.expect_gap(
            carriage,
            ram,
            axis="y",
            positive_elem="z_bearing_upper_1",
            negative_elem="ram_plate",
            min_gap=0.003,
            max_gap=0.010,
            name="z ram clears fixed bearings when extended",
        )
        ctx.expect_overlap(
            ram,
            carriage,
            axes="z",
            elem_a="ram_rail_1",
            elem_b="z_fixed_rail_1",
            min_overlap=0.05,
            name="z guide paths remain engaged at full travel",
        )
        low_z = ctx.part_world_position(ram)

    ctx.check(
        "z ram travels downward",
        rest_z is not None and low_z is not None and low_z[2] < rest_z[2] - 0.20,
        details=f"rest={rest_z}, low={low_z}",
    )

    return ctx.report()


object_model = build_object_model()
