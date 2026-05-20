from __future__ import annotations

from math import pi

import cadquery as cq

from sdk import (
    ArticulatedObject,
    ArticulationType,
    Box,
    Cylinder,
    MotionLimits,
    Origin,
    TestContext,
    TestReport,
    mesh_from_cadquery,
)


def _dovetail_rail_x(
    *,
    length: float,
    y_center: float,
    z_base: float,
    bottom_width: float,
    top_width: float,
    height: float,
) -> cq.Workplane:
    """Trapezoidal rail extruded along X, used for visible dovetail ways."""
    half_bottom = bottom_width / 2.0
    half_top = top_width / 2.0
    profile = [
        (y_center - half_bottom, z_base),
        (y_center + half_bottom, z_base),
        (y_center + half_top, z_base + height),
        (y_center - half_top, z_base + height),
    ]
    return (
        cq.Workplane("YZ")
        .polyline(profile)
        .close()
        .extrude(length)
        .translate((-length / 2.0, 0.0, 0.0))
    )


def _dovetail_rail_y(
    *,
    length: float,
    x_center: float,
    z_base: float,
    bottom_width: float,
    top_width: float,
    height: float,
) -> cq.Workplane:
    """Trapezoidal rail extruded along Y, used for the cross-slide ways."""
    half_bottom = bottom_width / 2.0
    half_top = top_width / 2.0
    profile = [
        (x_center - half_bottom, z_base),
        (x_center + half_bottom, z_base),
        (x_center + half_top, z_base + height),
        (x_center - half_top, z_base + height),
    ]
    return (
        cq.Workplane("XZ")
        .polyline(profile)
        .close()
        .extrude(length)
        # CadQuery's XZ workplane extrudes along local -Y, so translate back
        # by +length/2 to center the rail on the saddle frame.
        .translate((0.0, length / 2.0, 0.0))
    )


def build_object_model() -> ArticulatedObject:
    model = ArticulatedObject(name="cross_slide_drill_press_vise")

    cast_iron = model.material("dark_blued_cast_iron", rgba=(0.055, 0.065, 0.075, 1.0))
    machined = model.material("bright_machined_steel", rgba=(0.63, 0.64, 0.60, 1.0))
    black = model.material("black_oxide", rgba=(0.012, 0.012, 0.011, 1.0))
    jaw_plate = model.material("hardened_jaw_plates", rgba=(0.42, 0.43, 0.40, 1.0))
    slot_shadow = model.material("slot_shadow", rgba=(0.004, 0.004, 0.004, 1.0))

    base = model.part("base_plate")
    base.visual(
        Box((0.42, 0.28, 0.035)),
        origin=Origin(xyz=(0.0, 0.0, 0.0175)),
        material=cast_iron,
        name="base_casting",
    )
    for i, x in enumerate((-0.145, 0.145)):
        for j, y in enumerate((-0.095, 0.095)):
            base.visual(
                Box((0.070, 0.018, 0.003)),
                origin=Origin(xyz=(x, y, 0.0365)),
                material=slot_shadow,
                name=f"mount_slot_{i}_{j}",
            )
    for i, y in enumerate((-0.060, 0.060)):
        base.visual(
            mesh_from_cadquery(
                _dovetail_rail_x(
                    length=0.360,
                    y_center=y,
                    z_base=0.035,
                    bottom_width=0.046,
                    top_width=0.027,
                    height=0.020,
                ),
                f"base_x_way_{i}",
                tolerance=0.0007,
            ),
            material=machined,
            name=f"x_way_{i}",
        )
    for i, x in enumerate((-0.165, 0.165)):
        base.visual(
            Box((0.038, 0.034, 0.025)),
            origin=Origin(xyz=(x, -0.142, 0.0475)),
            material=cast_iron,
            name=f"x_screw_pedestal_{i}",
        )

    saddle = model.part("saddle")
    saddle.visual(
        Box((0.280, 0.200, 0.035)),
        origin=Origin(xyz=(0.0, 0.0, 0.0175)),
        material=cast_iron,
        name="saddle_plate",
    )
    saddle.visual(
        Box((0.088, 0.036, 0.026)),
        origin=Origin(xyz=(0.0, -0.112, 0.022)),
        material=cast_iron,
        name="x_screw_nut_boss",
    )
    for i, x in enumerate((-0.075, 0.075)):
        saddle.visual(
            mesh_from_cadquery(
                _dovetail_rail_y(
                    length=0.190,
                    x_center=x,
                    z_base=0.035,
                    bottom_width=0.040,
                    top_width=0.024,
                    height=0.020,
                ),
                f"saddle_y_way_{i}",
                tolerance=0.0007,
            ),
            material=machined,
            name=f"y_way_{i}",
        )
    saddle.visual(
        Box((0.030, 0.034, 0.027)),
        origin=Origin(xyz=(0.155, -0.115, 0.0485)),
        material=cast_iron,
        name="y_screw_pedestal",
    )
    saddle.visual(
        Box((0.260, 0.010, 0.012)),
        origin=Origin(xyz=(0.0, 0.105, 0.026)),
        material=black,
        name="x_gib_strip",
    )

    jaw_body = model.part("jaw_body")
    jaw_body.visual(
        Box((0.240, 0.180, 0.025)),
        origin=Origin(xyz=(0.0, 0.0, 0.0125)),
        material=cast_iron,
        name="jaw_slide",
    )
    jaw_body.visual(
        Box((0.250, 0.135, 0.020)),
        origin=Origin(xyz=(0.0, 0.0, 0.035)),
        material=cast_iron,
        name="vise_bed",
    )
    jaw_body.visual(
        Box((0.205, 0.032, 0.074)),
        origin=Origin(xyz=(0.0, 0.065, 0.082)),
        material=cast_iron,
        name="fixed_jaw",
    )
    jaw_body.visual(
        Box((0.190, 0.030, 0.064)),
        origin=Origin(xyz=(0.0, -0.058, 0.077)),
        material=cast_iron,
        name="movable_jaw_block",
    )
    jaw_body.visual(
        Box((0.178, 0.006, 0.045)),
        origin=Origin(xyz=(0.0, 0.046, 0.078)),
        material=jaw_plate,
        name="fixed_jaw_plate",
    )
    jaw_body.visual(
        Box((0.168, 0.006, 0.042)),
        origin=Origin(xyz=(0.0, -0.040, 0.076)),
        material=jaw_plate,
        name="movable_jaw_plate",
    )
    for i, x in enumerate((-0.064, -0.032, 0.0, 0.032, 0.064)):
        jaw_body.visual(
            Box((0.004, 0.007, 0.040)),
            origin=Origin(xyz=(x, 0.042, 0.078)),
            material=black,
            name=f"fixed_tooth_{i}",
        )
        jaw_body.visual(
            Box((0.004, 0.007, 0.038)),
            origin=Origin(xyz=(x, -0.036, 0.076)),
            material=black,
            name=f"movable_tooth_{i}",
        )

    x_crank = model.part("x_crank")
    x_crank.visual(
        Cylinder(radius=0.008, length=0.440),
        origin=Origin(xyz=(-0.220, 0.0, 0.0), rpy=(0.0, pi / 2.0, 0.0)),
        material=machined,
        name="lead_screw",
    )
    x_crank.visual(
        Cylinder(radius=0.030, length=0.012),
        origin=Origin(rpy=(0.0, pi / 2.0, 0.0)),
        material=machined,
        name="handwheel",
    )
    x_crank.visual(
        Cylinder(radius=0.012, length=0.020),
        origin=Origin(rpy=(0.0, pi / 2.0, 0.0)),
        material=black,
        name="hub",
    )
    x_crank.visual(
        Box((0.012, 0.008, 0.060)),
        origin=Origin(xyz=(0.010, 0.0, 0.030)),
        material=black,
        name="crank_arm",
    )
    x_crank.visual(
        Cylinder(radius=0.007, length=0.040),
        origin=Origin(xyz=(0.032, 0.0, 0.065), rpy=(0.0, pi / 2.0, 0.0)),
        material=black,
        name="grip",
    )

    y_crank = model.part("y_crank")
    y_crank.visual(
        Cylinder(radius=0.006, length=0.260),
        origin=Origin(xyz=(0.0, 0.130, 0.0), rpy=(-pi / 2.0, 0.0, 0.0)),
        material=machined,
        name="lead_screw",
    )
    y_crank.visual(
        Cylinder(radius=0.026, length=0.010),
        origin=Origin(rpy=(-pi / 2.0, 0.0, 0.0)),
        material=machined,
        name="handwheel",
    )
    y_crank.visual(
        Cylinder(radius=0.010, length=0.018),
        origin=Origin(rpy=(-pi / 2.0, 0.0, 0.0)),
        material=black,
        name="hub",
    )
    y_crank.visual(
        Box((0.008, 0.010, 0.052)),
        origin=Origin(xyz=(0.0, -0.010, 0.026)),
        material=black,
        name="crank_arm",
    )
    y_crank.visual(
        Cylinder(radius=0.006, length=0.034),
        origin=Origin(xyz=(0.0, -0.030, 0.058), rpy=(-pi / 2.0, 0.0, 0.0)),
        material=black,
        name="grip",
    )

    model.articulation(
        "x_slide",
        ArticulationType.PRISMATIC,
        parent=base,
        child=saddle,
        origin=Origin(xyz=(0.0, 0.0, 0.055)),
        axis=(1.0, 0.0, 0.0),
        motion_limits=MotionLimits(effort=220.0, velocity=0.10, lower=-0.055, upper=0.055),
    )
    model.articulation(
        "y_slide",
        ArticulationType.PRISMATIC,
        parent=saddle,
        child=jaw_body,
        origin=Origin(xyz=(0.0, 0.0, 0.055)),
        axis=(0.0, 1.0, 0.0),
        motion_limits=MotionLimits(effort=180.0, velocity=0.10, lower=-0.045, upper=0.045),
    )
    model.articulation(
        "x_crank_axis",
        ArticulationType.CONTINUOUS,
        parent=base,
        child=x_crank,
        origin=Origin(xyz=(0.225, -0.142, 0.068)),
        axis=(1.0, 0.0, 0.0),
        motion_limits=MotionLimits(effort=6.0, velocity=12.0),
    )
    model.articulation(
        "y_crank_axis",
        ArticulationType.CONTINUOUS,
        parent=saddle,
        child=y_crank,
        origin=Origin(xyz=(0.155, -0.145, 0.068)),
        axis=(0.0, 1.0, 0.0),
        motion_limits=MotionLimits(effort=5.0, velocity=12.0),
    )

    return model


def run_tests() -> TestReport:
    ctx = TestContext(object_model)

    base = object_model.get_part("base_plate")
    saddle = object_model.get_part("saddle")
    jaw_body = object_model.get_part("jaw_body")
    x_crank = object_model.get_part("x_crank")
    y_crank = object_model.get_part("y_crank")
    x_slide = object_model.get_articulation("x_slide")
    y_slide = object_model.get_articulation("y_slide")
    x_axis = object_model.get_articulation("x_crank_axis")
    y_axis = object_model.get_articulation("y_crank_axis")

    ctx.check("saddle uses X prismatic travel", tuple(x_slide.axis) == (1.0, 0.0, 0.0))
    ctx.check("jaw body uses Y prismatic travel", tuple(y_slide.axis) == (0.0, 1.0, 0.0))
    ctx.check("x crank is continuous", str(x_axis.articulation_type).lower().endswith("continuous"))
    ctx.check("y crank is continuous", str(y_axis.articulation_type).lower().endswith("continuous"))

    with ctx.pose({x_slide: 0.0, y_slide: 0.0}):
        ctx.expect_gap(
            saddle,
            base,
            axis="z",
            positive_elem="saddle_plate",
            negative_elem="x_way_0",
            max_gap=0.001,
            max_penetration=0.0,
            name="saddle bears on base dovetail ways",
        )
        ctx.expect_gap(
            jaw_body,
            saddle,
            axis="z",
            positive_elem="jaw_slide",
            negative_elem="y_way_0",
            max_gap=0.001,
            max_penetration=0.0,
            name="jaw body bears on saddle ways",
        )
        ctx.expect_overlap(
            saddle,
            base,
            axes="x",
            elem_a="saddle_plate",
            elem_b="x_way_0",
            min_overlap=0.220,
            name="saddle retained along X ways at center",
        )
        ctx.expect_overlap(
            jaw_body,
            saddle,
            axes="y",
            elem_a="jaw_slide",
            elem_b="y_way_0",
            min_overlap=0.140,
            name="jaw body retained along Y ways at center",
        )

    saddle_rest = ctx.part_world_position(saddle)
    jaw_rest = ctx.part_world_position(jaw_body)
    with ctx.pose({x_slide: 0.055, y_slide: 0.045, x_axis: pi / 2.0, y_axis: -pi / 2.0}):
        saddle_shifted = ctx.part_world_position(saddle)
        jaw_shifted = ctx.part_world_position(jaw_body)
        ctx.expect_overlap(
            saddle,
            base,
            axes="x",
            elem_a="saddle_plate",
            elem_b="x_way_0",
            min_overlap=0.165,
            name="saddle remains engaged at X travel limit",
        )
        ctx.expect_overlap(
            jaw_body,
            saddle,
            axes="y",
            elem_a="jaw_slide",
            elem_b="y_way_0",
            min_overlap=0.095,
            name="jaw body remains engaged at Y travel limit",
        )
        ctx.expect_origin_distance(x_crank, base, axes="yz", min_dist=0.13, max_dist=0.17)
        ctx.expect_origin_distance(y_crank, saddle, axes="xz", min_dist=0.16, max_dist=0.18)

    ctx.check(
        "X slide moves saddle in positive X",
        saddle_rest is not None and saddle_shifted is not None and saddle_shifted[0] > saddle_rest[0] + 0.050,
        details=f"rest={saddle_rest}, shifted={saddle_shifted}",
    )
    ctx.check(
        "Y slide moves jaw body in positive Y",
        jaw_rest is not None and jaw_shifted is not None and jaw_shifted[1] > jaw_rest[1] + 0.040,
        details=f"rest={jaw_rest}, shifted={jaw_shifted}",
    )

    return ctx.report()


object_model = build_object_model()
