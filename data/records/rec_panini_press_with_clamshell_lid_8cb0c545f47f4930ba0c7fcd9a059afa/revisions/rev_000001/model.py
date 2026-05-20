from __future__ import annotations

import math

import cadquery as cq
from sdk import (
    ArticulatedObject,
    ArticulationType,
    Box,
    Cylinder,
    KnobGeometry,
    KnobGrip,
    KnobIndicator,
    Material,
    MotionLimits,
    Origin,
    TestContext,
    TestReport,
    mesh_from_cadquery,
    mesh_from_geometry,
    wire_from_points,
)


def _rounded_box_mesh(size: tuple[float, float, float], name: str, *, fillet: float):
    shape = cq.Workplane("XY").box(*size)
    if fillet > 0.0:
        shape = shape.edges().fillet(fillet)
    return mesh_from_cadquery(shape, name, tolerance=0.0008, angular_tolerance=0.08)


def _lower_grill_mesh():
    length, width = 0.405, 0.275
    plate_thickness = 0.014
    ridge_height = 0.010
    ridge_width = 0.010
    shape = cq.Workplane("XY").box(length, width, plate_thickness)
    for i in range(11):
        y = -0.115 + i * 0.023
        ridge = (
            cq.Workplane("XY")
            .box(length - 0.035, ridge_width, ridge_height)
            .translate((0.0, y, plate_thickness / 2.0 + ridge_height / 2.0 - 0.001))
        )
        shape = shape.union(ridge)
    return mesh_from_cadquery(shape, "lower_grill_plate", tolerance=0.0007, angular_tolerance=0.08)


def _upper_cooking_mesh():
    length, width = 0.365, 0.255
    plate_thickness = 0.010
    ridge_height = 0.008
    ridge_width = 0.009
    # Local +Z is upward in the upper-platen frame.  This plate is mounted on the
    # underside, so the raised ridges point downward toward the food.
    shape = cq.Workplane("XY").box(length, width, plate_thickness)
    for i in range(10):
        y = -0.105 + i * 0.023
        ridge = (
            cq.Workplane("XY")
            .box(length - 0.030, ridge_width, ridge_height)
            .translate((0.0, y, -plate_thickness / 2.0 - ridge_height / 2.0 + 0.001))
        )
        shape = shape.union(ridge)
    return mesh_from_cadquery(shape, "upper_cooking_plate", tolerance=0.0007, angular_tolerance=0.08)


def _base_body_mesh():
    length, width, height = 0.480, 0.360, 0.080
    front_x = 0.260
    center_x = 0.020
    center_z = 0.050
    recess_depth = 0.018
    recess_width = 0.118
    recess_height = 0.072
    recess_z = 0.055

    body = cq.Workplane("XY").box(length, width, height)
    body = body.edges("|Z").fillet(0.012).edges(">Z").fillet(0.006)
    body = body.translate((center_x, 0.0, center_z))

    recess_cut = (
        cq.Workplane("XY")
        .box(recess_depth + 0.004, recess_width, recess_height)
        .translate((front_x - recess_depth / 2.0 + 0.002, 0.0, recess_z))
    )
    body = body.cut(recess_cut)
    return mesh_from_cadquery(body, "base_body_recessed", tolerance=0.0008, angular_tolerance=0.08)


def build_object_model() -> ArticulatedObject:
    model = ArticulatedObject(name="cafe_panini_press")

    stainless = model.material("brushed_stainless", rgba=(0.66, 0.64, 0.59, 1.0))
    dark_metal = model.material("seasoned_cast_iron", rgba=(0.045, 0.047, 0.044, 1.0))
    black_plastic = model.material("black_plastic", rgba=(0.012, 0.012, 0.014, 1.0))
    chrome = model.material("polished_tube", rgba=(0.86, 0.84, 0.78, 1.0))
    rubber = model.material("matte_rubber", rgba=(0.018, 0.018, 0.018, 1.0))

    base = model.part("lower_base")
    base.visual(_base_body_mesh(), material=stainless, name="base_body")
    base.visual(
        _lower_grill_mesh(),
        origin=Origin(xyz=(0.025, 0.0, 0.097)),
        material=dark_metal,
        name="lower_grill",
    )
    base.visual(
        Box((0.002, 0.108, 0.066)),
        origin=Origin(xyz=(0.241, 0.0, 0.055)),
        material=black_plastic,
        name="recess_floor",
    )

    # Rear hinge cheeks rise from the lower shell and visually carry the upper
    # platen.  They sit just outside the upper-platen width, with pin caps on
    # the outside faces.
    for y, name in [(-0.169, "hinge_cheek_0"), (0.169, "hinge_cheek_1")]:
        base.visual(
            Box((0.055, 0.018, 0.092)),
            origin=Origin(xyz=(-0.180, y, 0.134)),
            material=stainless,
            name=name,
        )
        base.visual(
            Cylinder(radius=0.018, length=0.008),
            origin=Origin(
                xyz=(-0.180, y + (0.013 if y > 0 else -0.013), 0.175),
                rpy=(math.pi / 2.0, 0.0, 0.0),
            ),
            material=chrome,
            name=f"hinge_pin_cap_{0 if y < 0 else 1}",
        )

    for x, y, name in [
        (-0.145, -0.125, "foot_0"),
        (-0.145, 0.125, "foot_1"),
        (0.205, -0.125, "foot_2"),
        (0.205, 0.125, "foot_3"),
    ]:
        base.visual(
            Cylinder(radius=0.025, length=0.018),
            origin=Origin(xyz=(x, y, 0.0015)),
            material=rubber,
            name=name,
        )

    upper = model.part("upper_platen")
    upper.visual(
        _rounded_box_mesh((0.395, 0.300, 0.086), "upper_deep_housing", fillet=0.014),
        origin=Origin(xyz=(0.202, 0.0, -0.006)),
        material=stainless,
        name="upper_housing",
    )
    upper.visual(
        _upper_cooking_mesh(),
        origin=Origin(xyz=(0.204, 0.0, -0.043)),
        material=dark_metal,
        name="upper_cooking_plate",
    )
    upper.visual(
        Cylinder(radius=0.013, length=0.320),
        origin=Origin(xyz=(0.0, 0.0, 0.0), rpy=(math.pi / 2.0, 0.0, 0.0)),
        material=chrome,
        name="hinge_barrel",
    )
    for y, name in [(-0.112, "handle_socket_0"), (0.112, "handle_socket_1")]:
        upper.visual(
            Box((0.034, 0.036, 0.026)),
            origin=Origin(xyz=(0.393, y, -0.008)),
            material=stainless,
            name=name,
        )

    handle_geom = wire_from_points(
        [
            (0.392, -0.112, -0.007),
            (0.455, -0.112, -0.021),
            (0.478, -0.088, -0.026),
            (0.478, 0.088, -0.026),
            (0.455, 0.112, -0.021),
            (0.392, 0.112, -0.007),
        ],
        radius=0.0075,
        radial_segments=20,
        cap_ends=True,
        corner_mode="fillet",
        corner_radius=0.020,
        corner_segments=12,
    )
    upper.visual(
        mesh_from_geometry(handle_geom, "bent_front_handle"),
        material=chrome,
        name="front_handle",
    )

    knob = model.part("timer_knob")
    knob.visual(
        Cylinder(radius=0.008, length=0.030),
        origin=Origin(xyz=(0.015, 0.0, 0.0), rpy=(0.0, math.pi / 2.0, 0.0)),
        material=chrome,
        name="shaft",
    )
    timer_knob_mesh = mesh_from_geometry(
        KnobGeometry(
            0.052,
            0.026,
            body_style="skirted",
            base_diameter=0.056,
            top_diameter=0.043,
            edge_radius=0.0015,
            grip=KnobGrip(style="fluted", count=20, depth=0.0014),
            indicator=KnobIndicator(style="line", mode="raised", angle_deg=90.0),
        ),
        "fluted_timer_knob",
    )
    knob.visual(
        timer_knob_mesh,
        origin=Origin(xyz=(0.043, 0.0, 0.0), rpy=(0.0, math.pi / 2.0, 0.0)),
        material=black_plastic,
        name="knob_cap",
    )

    hinge = model.articulation(
        "rear_hinge",
        ArticulationType.REVOLUTE,
        parent=base,
        child=upper,
        origin=Origin(xyz=(-0.180, 0.0, 0.175)),
        axis=(0.0, -1.0, 0.0),
        motion_limits=MotionLimits(effort=80.0, velocity=1.2, lower=0.0, upper=1.15),
    )
    hinge.meta["description"] = "Rear horizontal hinge: positive motion lifts the front platen upward."

    model.articulation(
        "timer_spindle",
        ArticulationType.CONTINUOUS,
        parent=base,
        child=knob,
        origin=Origin(xyz=(0.242, 0.0, 0.055)),
        axis=(1.0, 0.0, 0.0),
        motion_limits=MotionLimits(effort=1.5, velocity=8.0),
    )

    return model


def run_tests() -> TestReport:
    ctx = TestContext(object_model)
    base = object_model.get_part("lower_base")
    upper = object_model.get_part("upper_platen")
    knob = object_model.get_part("timer_knob")
    hinge = object_model.get_articulation("rear_hinge")
    spindle = object_model.get_articulation("timer_spindle")

    ctx.allow_overlap(
        base,
        knob,
        elem_a="base_body",
        elem_b="shaft",
        reason=(
            "The short timer shaft is intentionally captured through the simplified "
            "timer bushing behind the recessed front panel."
        ),
    )

    with ctx.pose({hinge: 0.0}):
        ctx.expect_gap(
            upper,
            base,
            axis="z",
            positive_elem="upper_cooking_plate",
            negative_elem="lower_grill",
            min_gap=0.002,
            max_gap=0.018,
            name="closed cooking plates have a shallow food gap",
        )
        ctx.expect_overlap(
            upper,
            base,
            axes="xy",
            elem_a="upper_cooking_plate",
            elem_b="lower_grill",
            min_overlap=0.220,
            name="upper platen covers the lower grill footprint",
        )

    closed_housing = ctx.part_element_world_aabb(upper, elem="upper_housing")
    with ctx.pose({hinge: 1.05}):
        opened_housing = ctx.part_element_world_aabb(upper, elem="upper_housing")
    ctx.check(
        "upper platen opens upward on rear hinge",
        closed_housing is not None
        and opened_housing is not None
        and opened_housing[1][2] > closed_housing[1][2] + 0.14,
        details=f"closed={closed_housing}, opened={opened_housing}",
    )

    ctx.expect_within(
        knob,
        base,
        axes="yz",
        inner_elem="knob_cap",
        outer_elem="recess_floor",
        margin=0.003,
        name="timer knob sits inside the real front recess",
    )
    ctx.expect_gap(
        knob,
        base,
        axis="x",
        positive_elem="shaft",
        negative_elem="recess_floor",
        min_gap=-0.0005,
        max_gap=0.002,
        name="short timer shaft emerges from recessed back panel",
    )
    ctx.check(
        "timer control is a continuous rotary spindle",
        spindle.articulation_type == ArticulationType.CONTINUOUS and spindle.axis == (1.0, 0.0, 0.0),
        details=f"type={spindle.articulation_type}, axis={spindle.axis}",
    )

    return ctx.report()


object_model = build_object_model()
