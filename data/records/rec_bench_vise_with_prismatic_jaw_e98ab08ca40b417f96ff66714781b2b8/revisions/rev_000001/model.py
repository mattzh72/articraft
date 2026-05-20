from __future__ import annotations

import cadquery as cq
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
    mesh_from_cadquery,
)

BASE_FOOT_X = 0.260
BASE_FOOT_Y = 0.180
BASE_FOOT_Z = 0.014
BASE_PEDESTAL_X = 0.220
BASE_PEDESTAL_Y = 0.128
BASE_PEDESTAL_Z = 0.016
BASE_RAIL_X = 0.205
BASE_RAIL_Y = 0.024
BASE_RAIL_H = 0.014
BASE_RAIL_OFFSET_Y = 0.034
BASE_SEAT_Z = BASE_FOOT_Z + BASE_PEDESTAL_Z + BASE_RAIL_H

X_SLIDE_X = 0.180
X_SLIDE_Y = 0.115
X_SLIDE_Z = 0.024
X_TOP_RAIL_X = 0.020
X_TOP_RAIL_Y = 0.105
X_TOP_RAIL_H = 0.010
X_TOP_RAIL_OFFSET_X = 0.030
X_TOP_SEAT_Z = X_SLIDE_Z + X_TOP_RAIL_H
X_TRAVEL = 0.040

Y_SLIDE_X = 0.145
Y_SLIDE_Y = 0.100
Y_SLIDE_Z = 0.020
Y_BED_Y = 0.072
Y_BED_Z = 0.014
JAW_X = 0.124
JAW_Y = 0.012
JAW_Z = 0.028
JAW_FACE_T = 0.003
Y_TRAVEL = 0.035

X_HOUSING_R = 0.011
X_HOUSING_L = 0.028
X_HOUSING_Z = 0.030
Y_HOUSING_R = 0.010
Y_HOUSING_L = 0.026
Y_HOUSING_Z = 0.021


def _base_casting_shape():
    foot = cq.Workplane("XY").box(BASE_FOOT_X, BASE_FOOT_Y, BASE_FOOT_Z).translate((0.0, 0.0, BASE_FOOT_Z / 2.0))
    pedestal = (
        cq.Workplane("XY")
        .box(BASE_PEDESTAL_X, BASE_PEDESTAL_Y, BASE_PEDESTAL_Z)
        .translate((0.0, 0.0, BASE_FOOT_Z + BASE_PEDESTAL_Z / 2.0))
    )
    rail_z = BASE_FOOT_Z + BASE_PEDESTAL_Z + BASE_RAIL_H / 2.0
    left_rail = cq.Workplane("XY").box(BASE_RAIL_X, BASE_RAIL_Y, BASE_RAIL_H).translate((0.0, BASE_RAIL_OFFSET_Y, rail_z))
    right_rail = cq.Workplane("XY").box(BASE_RAIL_X, BASE_RAIL_Y, BASE_RAIL_H).translate((0.0, -BASE_RAIL_OFFSET_Y, rail_z))
    front_apron = (
        cq.Workplane("XY")
        .box(BASE_PEDESTAL_X * 0.90, 0.022, BASE_PEDESTAL_Z * 1.15)
        .translate((0.0, -(BASE_PEDESTAL_Y / 2.0 + 0.011), BASE_FOOT_Z + BASE_PEDESTAL_Z * 0.575))
    )
    rear_rib = (
        cq.Workplane("XY")
        .box(BASE_PEDESTAL_X * 0.84, 0.016, BASE_PEDESTAL_Z * 0.95)
        .translate((0.0, BASE_PEDESTAL_Y / 2.0 + 0.008, BASE_FOOT_Z + BASE_PEDESTAL_Z * 0.475))
    )
    cheek_left = (
        cq.Workplane("XY")
        .box(0.030, BASE_PEDESTAL_Y * 0.88, BASE_PEDESTAL_Z * 0.95)
        .translate((BASE_PEDESTAL_X / 2.0 - 0.015, 0.0, BASE_FOOT_Z + BASE_PEDESTAL_Z * 0.475))
    )
    cheek_right = (
        cq.Workplane("XY")
        .box(0.030, BASE_PEDESTAL_Y * 0.88, BASE_PEDESTAL_Z * 0.95)
        .translate((-(BASE_PEDESTAL_X / 2.0 - 0.015), 0.0, BASE_FOOT_Z + BASE_PEDESTAL_Z * 0.475))
    )
    return foot.union(pedestal).union(left_rail).union(right_rail).union(front_apron).union(rear_rib).union(cheek_left).union(cheek_right)


def _x_slide_shape():
    carriage = cq.Workplane("XY").box(X_SLIDE_X, X_SLIDE_Y, X_SLIDE_Z).translate((0.0, 0.0, X_SLIDE_Z / 2.0))
    left_rail = (
        cq.Workplane("XY")
        .box(X_TOP_RAIL_X, X_TOP_RAIL_Y, X_TOP_RAIL_H)
        .translate((X_TOP_RAIL_OFFSET_X, 0.0, X_SLIDE_Z + X_TOP_RAIL_H / 2.0))
    )
    right_rail = (
        cq.Workplane("XY")
        .box(X_TOP_RAIL_X, X_TOP_RAIL_Y, X_TOP_RAIL_H)
        .translate((-X_TOP_RAIL_OFFSET_X, 0.0, X_SLIDE_Z + X_TOP_RAIL_H / 2.0))
    )
    front_apron = (
        cq.Workplane("XY")
        .box(X_SLIDE_X * 0.88, 0.020, 0.018)
        .translate((0.0, -(X_SLIDE_Y / 2.0 + 0.010), 0.009))
    )
    rear_lip = (
        cq.Workplane("XY")
        .box(X_SLIDE_X * 0.72, 0.012, 0.010)
        .translate((0.0, X_SLIDE_Y / 2.0 + 0.006, 0.017))
    )
    return carriage.union(left_rail).union(right_rail).union(front_apron).union(rear_lip)


def _y_slide_shape():
    table = cq.Workplane("XY").box(Y_SLIDE_X, Y_SLIDE_Y, Y_SLIDE_Z).translate((0.0, 0.0, Y_SLIDE_Z / 2.0))
    vise_bed = cq.Workplane("XY").box(Y_SLIDE_X, Y_BED_Y, Y_BED_Z).translate((0.0, 0.0, Y_SLIDE_Z + Y_BED_Z / 2.0))
    jaw_z = Y_SLIDE_Z + Y_BED_Z + JAW_Z / 2.0
    fixed_jaw = cq.Workplane("XY").box(JAW_X, JAW_Y, JAW_Z).translate((0.0, 0.030, jaw_z))
    movable_jaw = cq.Workplane("XY").box(JAW_X, JAW_Y, JAW_Z).translate((0.0, -0.012, jaw_z))
    screw_cover = (
        cq.Workplane("XY")
        .box(0.020, 0.038, 0.012)
        .translate((0.0, 0.009, Y_SLIDE_Z + Y_BED_Z + 0.006))
    )
    return table.union(vise_bed).union(fixed_jaw).union(movable_jaw).union(screw_cover)


def _steel_jaw_face(y_center: float) -> Origin:
    return Origin(xyz=(0.0, y_center, Y_SLIDE_Z + Y_BED_Z + JAW_Z / 2.0))


def _handwheel_shape():
    shaft_len = 0.016
    shaft_r = 0.0045
    hub_r = 0.010
    hub_len = 0.010
    rim_outer = 0.028
    rim_inner = 0.023
    rim_thickness = 0.004
    wheel_z = shaft_len + 0.004

    shaft = cq.Workplane("XY").circle(shaft_r).extrude(shaft_len)
    hub = cq.Workplane("XY").circle(hub_r).extrude(shaft_len + hub_len)
    rim = (
        cq.Workplane("XY")
        .workplane(offset=wheel_z - rim_thickness / 2.0)
        .circle(rim_outer)
        .circle(rim_inner)
        .extrude(rim_thickness)
    )

    body = shaft.union(hub).union(rim)
    spoke = cq.Workplane("XY").box(0.022, 0.004, rim_thickness).translate((0.015, 0.0, wheel_z))
    for angle_deg in (0.0, 120.0, 240.0):
        body = body.union(spoke.rotate((0.0, 0.0, 0.0), (0.0, 0.0, 1.0), angle_deg))

    crank_arm = cq.Workplane("XY").box(0.014, 0.005, 0.006).translate((0.022, 0.0, wheel_z + 0.003))
    grip = cq.Workplane("XY").workplane(offset=wheel_z + 0.006).center(0.022, 0.0).circle(0.0048).extrude(0.018)
    return body.union(crank_arm).union(grip)


def build_object_model() -> ArticulatedObject:
    model = ArticulatedObject(name="cross_slide_vise")

    cast_iron = model.material("cast_iron", rgba=(0.24, 0.26, 0.29, 1.0))
    machined_steel = model.material("machined_steel", rgba=(0.72, 0.74, 0.77, 1.0))
    handwheel_black = model.material("handwheel_black", rgba=(0.10, 0.10, 0.11, 1.0))
    dark_steel = model.material("dark_steel", rgba=(0.34, 0.36, 0.39, 1.0))

    base = model.part("base")
    base.visual(
        mesh_from_cadquery(_base_casting_shape(), "base_casting"),
        material=cast_iron,
        name="base_casting",
    )
    base.visual(
        Box((0.032, 0.030, 0.018)),
        origin=Origin(xyz=(0.0, -0.089, X_HOUSING_Z)),
        material=dark_steel,
        name="x_nut_block",
    )
    base.visual(
        Cylinder(radius=X_HOUSING_R, length=X_HOUSING_L),
        origin=Origin(
            xyz=(0.0, -(BASE_FOOT_Y / 2.0 + X_HOUSING_L / 2.0) + 0.002, X_HOUSING_Z),
            rpy=(pi / 2.0, 0.0, 0.0),
        ),
        material=machined_steel,
        name="x_screw_housing",
    )

    x_slide = model.part("x_slide")
    x_slide.visual(
        mesh_from_cadquery(_x_slide_shape(), "x_carriage"),
        material=cast_iron,
        name="x_carriage",
    )
    x_slide.visual(
        Cylinder(radius=Y_HOUSING_R, length=Y_HOUSING_L),
        origin=Origin(xyz=(X_SLIDE_X / 2.0 + Y_HOUSING_L / 2.0, 0.0, Y_HOUSING_Z), rpy=(0.0, pi / 2.0, 0.0)),
        material=machined_steel,
        name="y_screw_housing",
    )

    y_slide = model.part("y_slide")
    y_slide.visual(
        mesh_from_cadquery(_y_slide_shape(), "y_table"),
        material=cast_iron,
        name="y_table",
    )
    y_slide.visual(
        Box((JAW_X * 0.96, JAW_FACE_T, JAW_Z * 0.84)),
        origin=_steel_jaw_face(0.0245),
        material=machined_steel,
        name="fixed_jaw_face",
    )
    y_slide.visual(
        Box((JAW_X * 0.96, JAW_FACE_T, JAW_Z * 0.84)),
        origin=_steel_jaw_face(-0.0055),
        material=machined_steel,
        name="movable_jaw_face",
    )

    x_handle = model.part("x_handle")
    x_handle.visual(
        mesh_from_cadquery(_handwheel_shape(), "x_handwheel"),
        origin=Origin(rpy=(pi / 2.0, 0.0, 0.0)),
        material=handwheel_black,
        name="x_handwheel",
    )

    y_handle = model.part("y_handle")
    y_handle.visual(
        mesh_from_cadquery(_handwheel_shape(), "y_handwheel"),
        origin=Origin(rpy=(0.0, pi / 2.0, 0.0)),
        material=handwheel_black,
        name="y_handwheel",
    )

    model.articulation(
        "base_to_x_slide",
        ArticulationType.PRISMATIC,
        parent=base,
        child=x_slide,
        origin=Origin(xyz=(0.0, 0.0, BASE_SEAT_Z)),
        axis=(1.0, 0.0, 0.0),
        motion_limits=MotionLimits(effort=850.0, velocity=0.08, lower=-X_TRAVEL, upper=X_TRAVEL),
    )
    model.articulation(
        "x_slide_to_y_slide",
        ArticulationType.PRISMATIC,
        parent=x_slide,
        child=y_slide,
        origin=Origin(xyz=(0.0, 0.0, X_TOP_SEAT_Z)),
        axis=(0.0, 1.0, 0.0),
        motion_limits=MotionLimits(effort=850.0, velocity=0.08, lower=-Y_TRAVEL, upper=Y_TRAVEL),
    )
    model.articulation(
        "base_to_x_handle",
        ArticulationType.CONTINUOUS,
        parent=base,
        child=x_handle,
        origin=Origin(xyz=(0.0, -(BASE_FOOT_Y / 2.0 + X_HOUSING_L) + 0.002, X_HOUSING_Z)),
        axis=(0.0, 1.0, 0.0),
        motion_limits=MotionLimits(effort=40.0, velocity=10.0),
    )
    model.articulation(
        "x_slide_to_y_handle",
        ArticulationType.CONTINUOUS,
        parent=x_slide,
        child=y_handle,
        origin=Origin(xyz=(X_SLIDE_X / 2.0 + Y_HOUSING_L, 0.0, Y_HOUSING_Z)),
        axis=(1.0, 0.0, 0.0),
        motion_limits=MotionLimits(effort=40.0, velocity=10.0),
    )

    return model


def run_tests() -> TestReport:
    ctx = TestContext(object_model)
    base = object_model.get_part("base")
    x_slide = object_model.get_part("x_slide")
    y_slide = object_model.get_part("y_slide")
    x_handle = object_model.get_part("x_handle")
    y_handle = object_model.get_part("y_handle")
    base_to_x = object_model.get_articulation("base_to_x_slide")
    x_to_y = object_model.get_articulation("x_slide_to_y_slide")
    x_spin = object_model.get_articulation("base_to_x_handle")
    y_spin = object_model.get_articulation("x_slide_to_y_handle")

    ctx.expect_gap(
        x_slide,
        base,
        axis="z",
        positive_elem="x_carriage",
        negative_elem="base_casting",
        max_gap=0.0005,
        max_penetration=0.0,
        name="x slide sits on the base ways",
    )
    ctx.expect_overlap(
        x_slide,
        base,
        axes="xy",
        elem_a="x_carriage",
        elem_b="base_casting",
        min_overlap=0.08,
        name="x slide remains broadly supported by the base",
    )

    ctx.expect_gap(
        y_slide,
        x_slide,
        axis="z",
        positive_elem="y_table",
        negative_elem="x_carriage",
        max_gap=0.0005,
        max_penetration=0.0,
        name="y slide sits on the x slide ways",
    )
    ctx.expect_within(
        y_slide,
        x_slide,
        axes="x",
        elem_a="y_table",
        elem_b="x_carriage",
        margin=0.001,
        name="y slide stays centered between the x slide rails",
    )

    x_rest = ctx.part_world_position(x_slide)
    y_rest = ctx.part_world_position(y_slide)
    with ctx.pose({base_to_x: X_TRAVEL, x_to_y: Y_TRAVEL}):
        ctx.expect_overlap(
            x_slide,
            base,
            axes="x",
            elem_a="x_carriage",
            elem_b="base_casting",
            min_overlap=0.11,
            name="x slide retains insertion at max right travel",
        )
        ctx.expect_overlap(
            y_slide,
            x_slide,
            axes="y",
            elem_a="y_table",
            elem_b="x_carriage",
            min_overlap=0.03,
            name="y slide retains insertion at max forward travel",
        )
        x_extended = ctx.part_world_position(x_slide)
        y_extended = ctx.part_world_position(y_slide)

    ctx.check(
        "x slide moves toward positive x",
        x_rest is not None and x_extended is not None and x_extended[0] > x_rest[0] + 0.03,
        details=f"rest={x_rest}, extended={x_extended}",
    )
    ctx.check(
        "y slide moves toward positive y",
        y_rest is not None and y_extended is not None and y_extended[1] > y_rest[1] + 0.025,
        details=f"rest={y_rest}, extended={y_extended}",
    )
    ctx.check(
        "x handle uses a continuous joint",
        x_spin.motion_limits is not None and x_spin.motion_limits.lower is None and x_spin.motion_limits.upper is None,
        details=f"limits={x_spin.motion_limits}",
    )
    ctx.check(
        "y handle uses a continuous joint",
        y_spin.motion_limits is not None and y_spin.motion_limits.lower is None and y_spin.motion_limits.upper is None,
        details=f"limits={y_spin.motion_limits}",
    )
    with ctx.pose({x_spin: 1.1, y_spin: -0.8}):
        ctx.expect_contact(
            x_handle,
            base,
            elem_b="x_screw_housing",
            name="x handwheel remains seated on its screw housing",
        )
        ctx.expect_contact(
            y_handle,
            x_slide,
            elem_b="y_screw_housing",
            name="y handwheel remains seated on its screw housing",
        )

    return ctx.report()


object_model = build_object_model()
