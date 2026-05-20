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
    mesh_from_geometry,
    tube_from_spline_points,
)


BASE_L = 0.42
BASE_W = 0.38
BODY_H = 0.085
FEET_H = 0.03
BODY_TOP_Z = FEET_H + BODY_H

GRILL_RECESS_D = 0.016
LOWER_GRILL_L = 0.332
LOWER_GRILL_W = 0.282
LOWER_GRILL_T = 0.010
LOWER_RIB_H = 0.006

HINGE_X = -0.175
HINGE_Z = 0.188
BRACKET_Y = 0.162

LID_L = 0.355
LID_W = 0.36
LID_H = 0.10

DRAWER_L = 0.25
DRAWER_W = 0.272
DRAWER_H = 0.014
DRAWER_CENTER_X = 0.08
DRAWER_CENTER_Z = 0.021
DRAWER_TRAVEL = 0.12

RUNNER_L = 0.16
RUNNER_W = 0.018
RUNNER_H = 0.016
RUNNER_X = 0.06
RUNNER_Y = 0.135
RUNNER_Z = BODY_TOP_Z - BODY_H + (RUNNER_H / 2.0)


def _make_lower_frame_shape() -> cq.Workplane:
    housing = cq.Workplane("XY").box(BASE_L, BASE_W, BODY_H).translate((0.0, 0.0, FEET_H + (BODY_H / 2.0)))
    drawer_cavity = cq.Workplane("XY").box(0.24, 0.308, 0.032).translate((0.09, 0.0, 0.046))
    housing = housing.cut(drawer_cavity)
    recess = (
        cq.Workplane("XY")
        .box(LOWER_GRILL_L + 0.026, LOWER_GRILL_W + 0.026, GRILL_RECESS_D)
        .translate((0.0, 0.0, BODY_TOP_Z - (GRILL_RECESS_D / 2.0)))
    )
    housing = housing.cut(recess)

    grill = (
        cq.Workplane("XY")
        .box(LOWER_GRILL_L, LOWER_GRILL_W, LOWER_GRILL_T)
        .translate((0.0, 0.0, BODY_TOP_Z - GRILL_RECESS_D + (LOWER_GRILL_T / 2.0)))
    )
    housing = housing.union(grill)

    rib_z = BODY_TOP_Z - GRILL_RECESS_D + LOWER_GRILL_T + (LOWER_RIB_H / 2.0)
    for x_pos in (-0.12, -0.08, -0.04, 0.0, 0.04, 0.08, 0.12):
        rib = cq.Workplane("XY").box(0.016, LOWER_GRILL_W - 0.028, LOWER_RIB_H).translate((x_pos, 0.0, rib_z))
        housing = housing.union(rib)

    return housing


def _make_rear_bracket_shape() -> cq.Workplane:
    side_height = HINGE_Z - BODY_TOP_Z
    side_left = (
        cq.Workplane("XY")
        .box(0.018, 0.018, side_height)
        .translate((-0.022, BRACKET_Y + 0.008, -(side_height / 2.0)))
    )
    side_right = (
        cq.Workplane("XY")
        .box(0.018, 0.018, side_height)
        .translate((-0.022, -(BRACKET_Y + 0.008), -(side_height / 2.0)))
    )
    upper_bridge = cq.Workplane("XY").box(0.018, 0.356, 0.018).translate((-0.028, 0.0, -0.014))
    rear_web = cq.Workplane("XY").box(0.008, 0.356, side_height - 0.010).translate((-0.036, 0.0, -(side_height / 2.0) - 0.006))
    return side_left.union(side_right).union(upper_bridge).union(rear_web)


def _make_upper_platen_shape() -> cq.Workplane:
    platen = cq.Workplane("XY").box(LID_L, LID_W, LID_H).translate((0.162, 0.0, -0.017))

    for x_pos in (0.034, 0.078, 0.122, 0.166, 0.210, 0.254, 0.298):
        groove = cq.Workplane("XY").box(0.018, 0.300, 0.010).translate((x_pos, 0.0, -0.061))
        platen = platen.cut(groove)

    for y_pos in (-0.145, 0.145):
        ear = cq.Workplane("XY").box(0.024, 0.018, 0.052).translate((-0.004, y_pos, -0.008))
        platen = platen.union(ear)

    for y_pos in (-0.120, 0.120):
        post = cq.Workplane("XY").box(0.032, 0.020, 0.028).translate((0.355, y_pos, -0.020))
        platen = platen.union(post)

    return platen


def _make_drawer_shape() -> cq.Workplane:
    outer = cq.Workplane("XY").box(DRAWER_L, DRAWER_W, DRAWER_H)
    inner = cq.Workplane("XY").box(DRAWER_L - 0.020, DRAWER_W - 0.020, DRAWER_H - 0.004).translate((0.0, 0.0, 0.004))
    tray = outer.cut(inner)
    front_lip = cq.Workplane("XY").box(0.012, DRAWER_W + 0.016, 0.020).translate(((DRAWER_L / 2.0) + 0.006, 0.0, 0.003))
    pull = cq.Workplane("XY").box(0.018, 0.090, 0.010).translate(((DRAWER_L / 2.0) + 0.016, 0.0, -0.002))
    return tray.union(front_lip).union(pull)


def _make_handle_geometry():
    return tube_from_spline_points(
        [
            (0.360, -0.120, -0.020),
            (0.382, -0.105, -0.036),
            (0.400, -0.040, -0.046),
            (0.404, 0.000, -0.048),
            (0.400, 0.040, -0.046),
            (0.382, 0.105, -0.036),
            (0.360, 0.120, -0.020),
        ],
        radius=0.007,
        samples_per_segment=18,
        radial_segments=20,
        cap_ends=True,
    )


def build_object_model() -> ArticulatedObject:
    model = ArticulatedObject(name="cafe_panini_press")

    model.material("stainless", rgba=(0.73, 0.75, 0.78, 1.0))
    model.material("dark_steel", rgba=(0.30, 0.32, 0.35, 1.0))
    model.material("cast_iron", rgba=(0.17, 0.18, 0.19, 1.0))
    model.material("rubber", rgba=(0.09, 0.09, 0.10, 1.0))
    model.material("handle_black", rgba=(0.10, 0.10, 0.11, 1.0))

    lower_frame = model.part("lower_frame")
    lower_frame.visual(
        mesh_from_cadquery(_make_lower_frame_shape(), "lower_frame_housing"),
        material="stainless",
        name="housing",
    )
    for y_pos, name in ((-RUNNER_Y, "runner_0"), (RUNNER_Y, "runner_1")):
        lower_frame.visual(
            Box((RUNNER_L, RUNNER_W, RUNNER_H)),
            origin=Origin(xyz=(RUNNER_X, y_pos, RUNNER_Z)),
            material="dark_steel",
            name=name,
        )
    for y_pos, name in ((-RUNNER_Y, "runner_hanger_0"), (RUNNER_Y, "runner_hanger_1")):
        lower_frame.visual(
            Box((0.040, 0.010, 0.040)),
            origin=Origin(xyz=(0.000, y_pos, 0.066)),
            material="dark_steel",
            name=name,
        )
    for name, x_pos, y_pos in (
        ("foot_0", -0.16, -0.145),
        ("foot_1", -0.16, 0.145),
        ("foot_2", 0.172, -0.172),
        ("foot_3", 0.172, 0.172),
    ):
        lower_frame.visual(
            Cylinder(radius=0.016, length=FEET_H),
            origin=Origin(xyz=(x_pos, y_pos, FEET_H / 2.0)),
            material="rubber",
            name=name,
        )

    rear_bracket = model.part("rear_bracket")
    rear_bracket.visual(
        mesh_from_cadquery(_make_rear_bracket_shape(), "rear_bracket"),
        material="dark_steel",
        name="bracket",
    )
    upper_platen = model.part("upper_platen")
    upper_platen.visual(
        Box((LID_L, LID_W, LID_H)),
        origin=Origin(xyz=(0.162, 0.0, -0.017)),
        material="stainless",
        name="platen",
    )
    for x_pos, name in (
        (0.034, "rib_0"),
        (0.078, "rib_1"),
        (0.122, "rib_2"),
        (0.166, "rib_3"),
        (0.210, "rib_4"),
        (0.254, "rib_5"),
        (0.298, "rib_6"),
    ):
        upper_platen.visual(
            Box((0.018, 0.300, 0.005)),
            origin=Origin(xyz=(x_pos, 0.0, -0.0685)),
            material="cast_iron",
            name=name,
        )
    for y_pos, name in ((-0.145, "ear_0"), (0.145, "ear_1")):
        upper_platen.visual(
            Box((0.024, 0.018, 0.052)),
            origin=Origin(xyz=(-0.004, y_pos, -0.008)),
            material="dark_steel",
            name=name,
        )
    for y_pos, name in ((-0.120, "post_0"), (0.120, "post_1")):
        upper_platen.visual(
            Box((0.032, 0.020, 0.028)),
            origin=Origin(xyz=(0.355, y_pos, -0.020)),
            material="dark_steel",
            name=name,
        )
    upper_platen.visual(
        mesh_from_geometry(_make_handle_geometry(), "front_handle"),
        material="handle_black",
        name="handle",
    )

    grease_drawer = model.part("grease_drawer")
    grease_drawer.visual(
        mesh_from_cadquery(_make_drawer_shape(), "grease_drawer"),
        material="stainless",
        name="tray",
    )
    for y_pos, name in ((-0.132, "rail_0"), (0.132, "rail_1")):
        grease_drawer.visual(
            Box((RUNNER_L, 0.010, 0.010)),
            origin=Origin(xyz=(-0.020, y_pos, 0.009)),
            material="dark_steel",
            name=name,
        )

    model.articulation(
        "bracket_mount",
        ArticulationType.FIXED,
        parent=lower_frame,
        child=rear_bracket,
        origin=Origin(xyz=(HINGE_X, 0.0, HINGE_Z)),
    )
    model.articulation(
        "upper_hinge",
        ArticulationType.REVOLUTE,
        parent=rear_bracket,
        child=upper_platen,
        origin=Origin(),
        axis=(0.0, -1.0, 0.0),
        motion_limits=MotionLimits(lower=0.0, upper=1.30, effort=60.0, velocity=1.8),
    )
    model.articulation(
        "drawer_slide",
        ArticulationType.PRISMATIC,
        parent=lower_frame,
        child=grease_drawer,
        origin=Origin(xyz=(DRAWER_CENTER_X, 0.0, DRAWER_CENTER_Z)),
        axis=(1.0, 0.0, 0.0),
        motion_limits=MotionLimits(lower=0.0, upper=DRAWER_TRAVEL, effort=80.0, velocity=0.20),
    )

    return model


def run_tests() -> TestReport:
    ctx = TestContext(object_model)
    lower_frame = object_model.get_part("lower_frame")
    upper_platen = object_model.get_part("upper_platen")
    grease_drawer = object_model.get_part("grease_drawer")
    upper_hinge = object_model.get_articulation("upper_hinge")
    drawer_slide = object_model.get_articulation("drawer_slide")

    upper_limits = upper_hinge.motion_limits
    drawer_limits = drawer_slide.motion_limits
    hinge_upper = upper_limits.upper if upper_limits is not None and upper_limits.upper is not None else 1.30
    drawer_upper = drawer_limits.upper if drawer_limits is not None and drawer_limits.upper is not None else DRAWER_TRAVEL

    with ctx.pose({upper_hinge: 0.0, drawer_slide: 0.0}):
        ctx.expect_gap(
            upper_platen,
            lower_frame,
            axis="z",
            positive_elem="platen",
            negative_elem="housing",
            min_gap=0.001,
            max_gap=0.010,
            name="closed platen stays just above the lower grill",
        )
        ctx.expect_overlap(
            upper_platen,
            lower_frame,
            axes="xy",
            elem_a="platen",
            elem_b="housing",
            min_overlap=0.24,
            name="closed platen covers the lower grill footprint",
        )
        ctx.expect_overlap(
            grease_drawer,
            lower_frame,
            axes="x",
            elem_a="rail_0",
            elem_b="runner_0",
            min_overlap=0.12,
            name="drawer remains deeply inserted on the left runner when closed",
        )
        ctx.expect_overlap(
            grease_drawer,
            lower_frame,
            axes="x",
            elem_a="rail_1",
            elem_b="runner_1",
            min_overlap=0.12,
            name="drawer remains deeply inserted on the right runner when closed",
        )

    rest_handle_aabb = ctx.part_element_world_aabb(upper_platen, elem="handle")
    with ctx.pose({upper_hinge: hinge_upper}):
        open_handle_aabb = ctx.part_element_world_aabb(upper_platen, elem="handle")
    ctx.check(
        "upper platen opens upward",
        rest_handle_aabb is not None
        and open_handle_aabb is not None
        and open_handle_aabb[1][2] > rest_handle_aabb[1][2] + 0.12,
        details=f"rest_handle_aabb={rest_handle_aabb}, open_handle_aabb={open_handle_aabb}",
    )

    rest_drawer_pos = ctx.part_world_position(grease_drawer)
    with ctx.pose({drawer_slide: drawer_upper}):
        open_drawer_pos = ctx.part_world_position(grease_drawer)
        ctx.expect_overlap(
            grease_drawer,
            lower_frame,
            axes="yz",
            elem_a="rail_0",
            elem_b="runner_0",
            min_overlap=0.004,
            name="left drawer rail stays centered on its runner",
        )
        ctx.expect_overlap(
            grease_drawer,
            lower_frame,
            axes="yz",
            elem_a="rail_1",
            elem_b="runner_1",
            min_overlap=0.004,
            name="right drawer rail stays centered on its runner",
        )
        ctx.expect_overlap(
            grease_drawer,
            lower_frame,
            axes="x",
            elem_a="rail_0",
            elem_b="runner_0",
            min_overlap=0.035,
            name="left drawer rail keeps retained insertion at full extension",
        )
        ctx.expect_overlap(
            grease_drawer,
            lower_frame,
            axes="x",
            elem_a="rail_1",
            elem_b="runner_1",
            min_overlap=0.035,
            name="right drawer rail keeps retained insertion at full extension",
        )

    ctx.check(
        "grease drawer slides forward",
        rest_drawer_pos is not None
        and open_drawer_pos is not None
        and open_drawer_pos[0] > rest_drawer_pos[0] + 0.10,
        details=f"rest_drawer_pos={rest_drawer_pos}, open_drawer_pos={open_drawer_pos}",
    )

    return ctx.report()


object_model = build_object_model()
