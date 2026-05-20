from __future__ import annotations

import math

from sdk import (
    ArticulatedObject,
    ArticulationType,
    Box,
    Cylinder,
    Material,
    MotionLimits,
    Origin,
    TestContext,
    TestReport,
)


def _box(part, name: str, size, xyz, material):
    part.visual(Box(size), origin=Origin(xyz=xyz), material=material, name=name)


def _cylinder(part, name: str, radius: float, length: float, xyz, rpy, material):
    part.visual(
        Cylinder(radius=radius, length=length),
        origin=Origin(xyz=xyz, rpy=rpy),
        material=material,
        name=name,
    )


def build_object_model() -> ArticulatedObject:
    model = ArticulatedObject(name="compact_cartesian_manipulator")

    cast = Material("mat_cast_iron", rgba=(0.18, 0.20, 0.22, 1.0))
    dark = Material("mat_dark_covers", rgba=(0.05, 0.055, 0.06, 1.0))
    rail = Material("mat_ground_steel", rgba=(0.72, 0.75, 0.76, 1.0))
    moving = Material("mat_moving_blue", rgba=(0.10, 0.23, 0.42, 1.0))
    upper = Material("mat_upper_orange", rgba=(0.88, 0.42, 0.10, 1.0))
    stop = Material("mat_stop_red", rgba=(0.75, 0.05, 0.04, 1.0))
    bolt = Material("mat_black_bolt", rgba=(0.015, 0.015, 0.018, 1.0))

    # Grounded X-axis slide: a heavy bed with raised rail seats, exposed round
    # guide rails, accordion screw cover, and hard-stop posts kept outside travel.
    base = model.part("base")
    _box(base, "base_plate", (0.96, 0.42, 0.035), (0.0, 0.0, 0.0175), cast)
    _box(base, "foot_rail_0", (0.88, 0.045, 0.026), (0.0, -0.165, -0.008), dark)
    _box(base, "foot_rail_1", (0.88, 0.045, 0.026), (0.0, 0.165, -0.008), dark)
    for i, y in enumerate((-0.13, 0.13)):
        _box(base, f"x_rail_bed_{i}", (0.82, 0.055, 0.055), (0.0, y, 0.0625), cast)
        _cylinder(
            base,
            f"x_rail_{i}",
            0.014,
            0.82,
            (0.0, y, 0.105),
            (0.0, math.pi / 2.0, 0.0),
            rail,
        )
    for i, x in enumerate((-0.43, 0.43)):
        _box(base, f"x_end_block_{i}", (0.050, 0.34, 0.090), (x, 0.0, 0.087), cast)
    _box(base, "x_screw_cover", (0.66, 0.050, 0.050), (0.0, 0.0, 0.060), dark)
    for i, x in enumerate((-0.30, -0.24, -0.18, -0.12, -0.06, 0.0, 0.06, 0.12, 0.18, 0.24, 0.30)):
        _box(base, f"x_cover_rib_{i}", (0.016, 0.060, 0.014), (x, 0.0, 0.092), cast)
    for i, x in enumerate((-0.37, 0.37)):
        _box(base, f"x_stop_post_{i}", (0.022, 0.090, 0.090), (x, 0.0, 0.080), cast)
        _box(base, f"x_stop_bumper_{i}", (0.034, 0.100, 0.030), (x, 0.0, 0.140), stop)

    # X carriage/saddle: the blue saddle rides over the lower rails and carries
    # the complete Y-axis guide system, making the stacked motion order visible.
    x_saddle = model.part("x_saddle")
    _box(x_saddle, "x_carriage_block", (0.32, 0.34, 0.045), (0.0, 0.0, 0.024), moving)
    for i, y in enumerate((-0.13, 0.13)):
        _box(x_saddle, f"x_bearing_shoe_{i}", (0.26, 0.040, 0.024), (0.0, y, -0.004), dark)
    for i, x in enumerate((-0.12, -0.06, 0.0, 0.06, 0.12)):
        _box(x_saddle, f"x_saddle_rib_{i}", (0.018, 0.30, 0.018), (x, 0.0, 0.053), moving)
    _box(x_saddle, "y_track_riser", (0.24, 0.28, 0.030), (0.0, 0.0, 0.058), moving)
    _box(x_saddle, "y_track_plate", (0.28, 0.54, 0.032), (0.0, 0.0, 0.086), moving)
    for i, x in enumerate((-0.085, 0.085)):
        _box(x_saddle, f"y_rail_seat_{i}", (0.035, 0.43, 0.012), (x, 0.0, 0.107), cast)
        _cylinder(
            x_saddle,
            f"y_rail_{i}",
            0.0105,
            0.47,
            (x, 0.0, 0.121),
            (math.pi / 2.0, 0.0, 0.0),
            rail,
        )
    for i, y in enumerate((-0.25, 0.25)):
        _box(x_saddle, f"y_end_block_{i}", (0.26, 0.035, 0.070), (0.0, y, 0.111), cast)
        _box(x_saddle, f"y_stop_bumper_{i}", (0.19, 0.018, 0.030), (0.0, y * 0.93, 0.150), stop)

    # Y cross-slide: a carried block with side skirts that clear the Y rails,
    # plus the vertical Z guide column bolted to its top.
    y_cross_slide = model.part("y_cross_slide")
    _box(y_cross_slide, "y_carriage_block", (0.23, 0.19, 0.036), (0.0, 0.0, -0.0005), moving)
    for i, x in enumerate((-0.13, 0.13)):
        _box(y_cross_slide, f"y_side_skirt_{i}", (0.035, 0.16, 0.060), (x, 0.0, -0.004), dark)
    _box(y_cross_slide, "cross_cover_plate", (0.27, 0.23, 0.028), (0.0, 0.0, 0.034), moving)
    for i, x in enumerate((-0.09, -0.03, 0.03, 0.09)):
        _box(y_cross_slide, f"cross_plate_rib_{i}", (0.015, 0.20, 0.015), (x, 0.0, 0.052), moving)
    _box(y_cross_slide, "column_pedestal", (0.19, 0.16, 0.070), (0.0, 0.020, 0.080), moving)
    _box(y_cross_slide, "z_column_spine", (0.13, 0.060, 0.440), (0.0, -0.015, 0.330), moving)
    _box(y_cross_slide, "z_bottom_cap", (0.18, 0.080, 0.035), (0.0, 0.025, 0.130), cast)
    _box(y_cross_slide, "z_top_cap", (0.18, 0.080, 0.035), (0.0, 0.025, 0.555), cast)
    for i, x in enumerate((-0.045, 0.045)):
        _cylinder(
            y_cross_slide,
            f"z_rail_{i}",
            0.009,
            0.420,
            (x, 0.045, 0.330),
            (0.0, 0.0, 0.0),
            rail,
        )
        for j, z in enumerate((0.180, 0.320, 0.460)):
            _box(y_cross_slide, f"z_rail_standoff_{i}_{j}", (0.025, 0.030, 0.025), (x, 0.030, z), cast)
    _box(y_cross_slide, "z_lower_bumper", (0.11, 0.022, 0.018), (0.0, 0.067, 0.150), stop)
    _box(y_cross_slide, "z_upper_bumper", (0.11, 0.022, 0.018), (0.0, 0.067, 0.535), stop)

    # Upper Z carriage: narrow orange sliding block and its tool plate are a
    # single uppermost moving link, clearly separated from the fixed Z column.
    z_carriage = model.part("z_carriage")
    _box(z_carriage, "z_carriage_plate", (0.16, 0.030, 0.160), (0.0, 0.018, 0.045), upper)
    for i, x in enumerate((-0.045, 0.045)):
        _box(z_carriage, f"z_bearing_shoe_{i}_0", (0.050, 0.018, 0.050), (x, -0.006, 0.005), dark)
        _box(z_carriage, f"z_bearing_shoe_{i}_1", (0.050, 0.018, 0.050), (x, -0.006, 0.085), dark)
    for i, z in enumerate((-0.015, 0.045, 0.105)):
        _box(z_carriage, f"z_carriage_rib_{i}", (0.13, 0.020, 0.012), (0.0, 0.041, z), upper)
    _box(z_carriage, "tool_plate_neck", (0.080, 0.040, 0.035), (0.0, 0.043, 0.129), upper)
    _box(z_carriage, "tool_plate", (0.20, 0.11, 0.018), (0.0, 0.065, 0.145), upper)
    for i, x in enumerate((-0.065, 0.065)):
        for j, y in enumerate((0.030, 0.095)):
            _cylinder(
                z_carriage,
                f"tool_bolt_{i}_{j}",
                0.008,
                0.006,
                (x, y, 0.157),
                (0.0, 0.0, 0.0),
                bolt,
            )

    x_joint = model.articulation(
        "base_to_x_saddle",
        ArticulationType.PRISMATIC,
        parent=base,
        child=x_saddle,
        origin=Origin(xyz=(0.0, 0.0, 0.135)),
        axis=(1.0, 0.0, 0.0),
        motion_limits=MotionLimits(effort=220.0, velocity=0.35, lower=-0.16, upper=0.16),
    )
    y_joint = model.articulation(
        "x_saddle_to_y_cross_slide",
        ArticulationType.PRISMATIC,
        parent=x_saddle,
        child=y_cross_slide,
        origin=Origin(xyz=(0.0, 0.0, 0.150)),
        axis=(0.0, 1.0, 0.0),
        motion_limits=MotionLimits(effort=140.0, velocity=0.30, lower=-0.10, upper=0.10),
    )
    z_joint = model.articulation(
        "y_cross_slide_to_z_carriage",
        ArticulationType.PRISMATIC,
        parent=y_cross_slide,
        child=z_carriage,
        origin=Origin(xyz=(0.0, 0.069, 0.190)),
        axis=(0.0, 0.0, 1.0),
        motion_limits=MotionLimits(effort=90.0, velocity=0.22, lower=0.0, upper=0.18),
    )
    # Keep handles alive for clarity in generated metadata and tests by name.
    _ = (x_joint, y_joint, z_joint)
    return model


def run_tests() -> TestReport:
    ctx = TestContext(object_model)
    base = object_model.get_part("base")
    x_saddle = object_model.get_part("x_saddle")
    y_cross_slide = object_model.get_part("y_cross_slide")
    z_carriage = object_model.get_part("z_carriage")
    x_joint = object_model.get_articulation("base_to_x_saddle")
    y_joint = object_model.get_articulation("x_saddle_to_y_cross_slide")
    z_joint = object_model.get_articulation("y_cross_slide_to_z_carriage")

    # Rest pose checks: each slide is visibly stacked above the guide system it
    # rides on, without needing collision waivers.
    ctx.expect_contact(
        x_saddle,
        base,
        elem_a="x_bearing_shoe_0",
        elem_b="x_rail_0",
        contact_tol=0.0005,
        name="x bearing shoe rides on lower rail",
    )
    ctx.expect_contact(
        y_cross_slide,
        x_saddle,
        elem_a="y_carriage_block",
        elem_b="y_rail_0",
        contact_tol=0.0005,
        name="y carriage rides on carried rail",
    )
    ctx.expect_contact(
        z_carriage,
        y_cross_slide,
        elem_a="z_bearing_shoe_0_0",
        elem_b="z_rail_0",
        contact_tol=0.0005,
        name="z shoe rides on front rail",
    )
    ctx.expect_overlap(
        z_carriage,
        y_cross_slide,
        axes="z",
        elem_a="z_carriage_plate",
        elem_b="z_rail_0",
        min_overlap=0.12,
        name="z carriage remains engaged on vertical rail",
    )

    # Decisive end-of-travel checks: the articulated paths have clearance to the
    # modeled stops, covers, and rail end blocks.
    with ctx.pose({x_joint: 0.16}):
        ctx.expect_gap(
            base,
            x_saddle,
            axis="x",
            positive_elem="x_stop_post_1",
            negative_elem="x_carriage_block",
            min_gap=0.025,
            name="x positive stop remains clear",
        )
    with ctx.pose({x_joint: -0.16}):
        ctx.expect_gap(
            x_saddle,
            base,
            axis="x",
            positive_elem="x_carriage_block",
            negative_elem="x_stop_post_0",
            min_gap=0.025,
            name="x negative stop remains clear",
        )
    with ctx.pose({y_joint: 0.10}):
        ctx.expect_gap(
            x_saddle,
            y_cross_slide,
            axis="y",
            positive_elem="y_end_block_1",
            negative_elem="y_carriage_block",
            min_gap=0.015,
            name="y positive stop remains clear",
        )
    with ctx.pose({y_joint: -0.10}):
        ctx.expect_gap(
            y_cross_slide,
            x_saddle,
            axis="y",
            positive_elem="y_carriage_block",
            negative_elem="y_end_block_0",
            min_gap=0.015,
            name="y negative stop remains clear",
        )
    with ctx.pose({z_joint: 0.18}):
        ctx.expect_gap(
            y_cross_slide,
            z_carriage,
            axis="z",
            positive_elem="z_top_cap",
            negative_elem="tool_plate",
            min_gap=0.004,
            name="z upper travel clears top cap",
        )

    return ctx.report()


object_model = build_object_model()
