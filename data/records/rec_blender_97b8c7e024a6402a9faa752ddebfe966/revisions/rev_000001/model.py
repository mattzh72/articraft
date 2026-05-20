from __future__ import annotations

import math

import cadquery as cq

from sdk import (
    ArticulatedObject,
    ArticulationType,
    Box,
    MotionLimits,
    Origin,
    TestContext,
    TestReport,
    mesh_from_cadquery,
)


BODY_W = 0.24
BODY_D = 0.29
BODY_H = 0.072
TOWER_D = 0.115
TOWER_H = 0.135

CUP_PIVOT_Y = -0.022
CUP_PIVOT_Z = 0.118
CUP_CENTER_Y = 0.034
CUP_BOTTOM_Z = -0.028
CUP_TOP_Z = 0.120
CUP_BOTTOM_R = 0.042
CUP_TOP_R = 0.056
CUP_WALL = 0.0035
CUP_FLOOR = 0.006

CLAMP_HINGE_Y = 0.088
CLAMP_HINGE_Z = 0.253
CLAMP_REACH = 0.094

BLADE_AXIS_REL = (0.0, CUP_CENTER_Y, CUP_BOTTOM_Z + CUP_FLOOR)


def _x_cylinder(radius: float, length: float, center: tuple[float, float, float]) -> cq.Workplane:
    return (
        cq.Workplane("YZ")
        .circle(radius)
        .extrude(length)
        .translate((center[0] - length / 2.0, center[1], center[2]))
    )


def _make_housing_shape() -> cq.Workplane:
    rear_box = cq.Workplane("XY").box(BODY_W * 0.92, 0.110, BODY_H + TOWER_H).translate((0.0, 0.125, (BODY_H + TOWER_H) / 2.0))
    front_foot = cq.Workplane("XY").box(0.140, 0.042, 0.040).translate((0.0, -0.096, 0.020))
    center_spine = cq.Workplane("XY").box(0.040, 0.235, 0.040).translate((0.0, 0.0075, 0.020))
    return rear_box.union(front_foot).union(center_spine)


def _make_support_shape(sign: float) -> cq.Workplane:
    yoke_x = sign * 0.068
    yoke = cq.Workplane("XY").box(0.016, 0.068, 0.108).translate((yoke_x, -0.028, 0.126))
    lower_arm = cq.Workplane("XY").box(0.040, 0.080, 0.040).translate((sign * 0.0922, -0.030, 0.082))
    rear_rail = cq.Workplane("XY").box(0.024, 0.110, 0.130).translate((sign * 0.1224, 0.055, 0.095))
    return yoke.union(lower_arm).union(rear_rail)


def _make_hinge_mount_shape() -> cq.Workplane:
    pedestal = cq.Workplane("XY").box(0.152, 0.016, 0.040).translate((0.0, 0.106, CLAMP_HINGE_Z - 0.028))
    posts = cq.Workplane("XY").box(0.020, 0.018, 0.040).translate((0.050, 0.106, CLAMP_HINGE_Z - 0.020))
    posts = posts.union(cq.Workplane("XY").box(0.020, 0.018, 0.040).translate((-0.050, 0.106, CLAMP_HINGE_Z - 0.020)))
    return pedestal.union(posts)


def _make_cup_shape() -> cq.Workplane:
    outer = (
        cq.Workplane("XY")
        .workplane(offset=CUP_BOTTOM_Z)
        .circle(CUP_BOTTOM_R)
        .workplane(offset=CUP_TOP_Z - CUP_BOTTOM_Z)
        .circle(CUP_TOP_R)
        .loft(combine=True)
        .translate((0.0, CUP_CENTER_Y, 0.0))
    )
    inner = (
        cq.Workplane("XY")
        .workplane(offset=CUP_BOTTOM_Z + CUP_FLOOR)
        .circle(CUP_BOTTOM_R - CUP_WALL - 0.001)
        .workplane(offset=CUP_TOP_Z - CUP_BOTTOM_Z - CUP_FLOOR)
        .circle(CUP_TOP_R - CUP_WALL)
        .loft(combine=True)
        .translate((0.0, CUP_CENTER_Y, 0.0))
    )
    body = outer.cut(inner)

    pour_notch = cq.Workplane("XY").box(0.026, 0.018, 0.020).translate((0.0, CUP_CENTER_Y - CUP_TOP_R + 0.004, CUP_TOP_Z - 0.004))
    body = body.cut(pour_notch)

    lug_x = 0.049
    lug_bridge = cq.Workplane("XY").box(0.020, 0.034, 0.020).translate((lug_x, 0.014, 0.001))
    lug_bridge = lug_bridge.union(cq.Workplane("XY").box(0.020, 0.034, 0.020).translate((-lug_x, 0.014, 0.001)))

    pin_collars = _x_cylinder(0.009, 0.016, (0.052, 0.000, 0.000))
    pin_collars = pin_collars.union(_x_cylinder(0.009, 0.016, (-0.052, 0.000, 0.000)))
    drive_boss = (
        cq.Workplane("XY")
        .circle(0.015)
        .extrude(0.008)
        .translate((0.0, CUP_CENTER_Y, CUP_BOTTOM_Z - 0.008))
    )

    return body.union(lug_bridge).union(pin_collars).union(drive_boss)


def _make_clamp_frame_shape() -> cq.Workplane:
    arm_x = 0.054
    arm = cq.Workplane("XY").box(0.016, CLAMP_REACH, 0.014).translate((arm_x, -CLAMP_REACH / 2.0, 0.008))
    arm = arm.union(
        cq.Workplane("XY").box(0.016, CLAMP_REACH, 0.014).translate((-arm_x, -CLAMP_REACH / 2.0, 0.008))
    )
    front_bar = cq.Workplane("XY").box(0.118, 0.018, 0.014).translate((0.0, -CLAMP_REACH, 0.008))
    rear_cross = cq.Workplane("XY").box(0.118, 0.014, 0.012).translate((0.0, -0.010, 0.008))
    barrels = _x_cylinder(0.009, 0.022, (0.054, 0.000, 0.000))
    barrels = barrels.union(_x_cylinder(0.009, 0.022, (-0.054, 0.000, 0.000)))
    return arm.union(front_bar).union(rear_cross).union(barrels)


def _make_blade_shape() -> cq.Workplane:
    shaft = cq.Workplane("XY").circle(0.004).extrude(0.024)
    hub = cq.Workplane("XY").circle(0.011).extrude(0.008).translate((0.0, 0.0, 0.008))
    blade_a = cq.Workplane("XY").box(0.050, 0.010, 0.002).translate((0.0, 0.0, 0.017))
    blade_a = blade_a.rotate((0.0, 0.0, 0.017), (0.0, 1.0, 0.017), 12.0).rotate((0.0, 0.0, 0.017), (0.0, 0.0, 1.0), 18.0)
    blade_b = cq.Workplane("XY").box(0.046, 0.010, 0.002).translate((0.0, 0.0, 0.015))
    blade_b = blade_b.rotate((0.0, 0.0, 0.015), (0.0, 1.0, 0.015), -12.0).rotate((0.0, 0.0, 0.015), (0.0, 0.0, 1.0), 108.0)
    return shaft.union(hub).union(blade_a).union(blade_b)


def build_object_model() -> ArticulatedObject:
    model = ArticulatedObject(name="laboratory_blender")

    housing_finish = model.material("housing_finish", rgba=(0.84, 0.86, 0.88, 1.0))
    dark_trim = model.material("dark_trim", rgba=(0.18, 0.20, 0.22, 1.0))
    cup_finish = model.material("cup_finish", rgba=(0.70, 0.78, 0.82, 0.55))
    steel = model.material("steel", rgba=(0.74, 0.76, 0.79, 1.0))

    housing = model.part("housing")
    housing.visual(
        mesh_from_cadquery(_make_housing_shape(), "housing_shell"),
        material=housing_finish,
        name="housing_shell",
    )

    support_0 = model.part("support_0")
    support_0.visual(
        mesh_from_cadquery(_make_support_shape(1.0), "support_0_shell"),
        material=housing_finish,
        name="support_0_shell",
    )

    support_1 = model.part("support_1")
    support_1.visual(
        mesh_from_cadquery(_make_support_shape(-1.0), "support_1_shell"),
        material=housing_finish,
        name="support_1_shell",
    )

    hinge_mount = model.part("hinge_mount")
    hinge_mount.visual(
        mesh_from_cadquery(_make_hinge_mount_shape(), "hinge_mount_shell"),
        material=housing_finish,
        name="hinge_mount_shell",
    )

    cup = model.part("cup")
    cup.visual(
        mesh_from_cadquery(_make_cup_shape(), "cup_body"),
        material=cup_finish,
        name="cup_body",
    )

    clamp = model.part("clamp")
    clamp.visual(
        mesh_from_cadquery(_make_clamp_frame_shape(), "clamp_frame"),
        material=dark_trim,
        name="clamp_frame",
    )
    clamp.visual(
        Box((0.084, 0.018, 0.014)),
        origin=Origin(xyz=(0.0, -CLAMP_REACH, -0.005)),
        material=steel,
        name="clamp_pad",
    )

    blade = model.part("blade")
    blade.visual(
        mesh_from_cadquery(_make_blade_shape(), "blade_rotor"),
        material=steel,
        name="blade_rotor",
    )

    model.articulation(
        "housing_to_support_0",
        ArticulationType.FIXED,
        parent=housing,
        child=support_0,
        origin=Origin(),
    )
    model.articulation(
        "housing_to_support_1",
        ArticulationType.FIXED,
        parent=housing,
        child=support_1,
        origin=Origin(),
    )
    model.articulation(
        "housing_to_hinge_mount",
        ArticulationType.FIXED,
        parent=housing,
        child=hinge_mount,
        origin=Origin(),
    )
    model.articulation(
        "cup_pitch",
        ArticulationType.REVOLUTE,
        parent=housing,
        child=cup,
        origin=Origin(xyz=(0.0, CUP_PIVOT_Y, CUP_PIVOT_Z)),
        axis=(1.0, 0.0, 0.0),
        motion_limits=MotionLimits(lower=0.0, upper=0.72, effort=14.0, velocity=1.4),
    )
    model.articulation(
        "clamp_hinge",
        ArticulationType.REVOLUTE,
        parent=hinge_mount,
        child=clamp,
        origin=Origin(xyz=(0.0, CLAMP_HINGE_Y, CLAMP_HINGE_Z)),
        axis=(-1.0, 0.0, 0.0),
        motion_limits=MotionLimits(lower=0.0, upper=1.20, effort=8.0, velocity=1.8),
    )
    model.articulation(
        "blade_spin",
        ArticulationType.CONTINUOUS,
        parent=cup,
        child=blade,
        origin=Origin(xyz=BLADE_AXIS_REL),
        axis=(0.0, 0.0, 1.0),
        motion_limits=MotionLimits(effort=1.0, velocity=40.0),
    )

    return model


def _aabb_center(aabb: tuple[tuple[float, float, float], tuple[float, float, float]] | None) -> tuple[float, float, float] | None:
    if aabb is None:
        return None
    low, high = aabb
    return tuple((low[i] + high[i]) * 0.5 for i in range(3))


def run_tests() -> TestReport:
    ctx = TestContext(object_model)
    housing = object_model.get_part("housing")
    support_0 = object_model.get_part("support_0")
    support_1 = object_model.get_part("support_1")
    cup = object_model.get_part("cup")
    clamp = object_model.get_part("clamp")
    blade = object_model.get_part("blade")
    cup_pitch = object_model.get_articulation("cup_pitch")
    clamp_hinge = object_model.get_articulation("clamp_hinge")
    blade_spin = object_model.get_articulation("blade_spin")

    ctx.allow_overlap(
        housing,
        support_0,
        reason="The positive-side cup support is authored as a fixed welded extension of the housing shell.",
    )
    ctx.allow_overlap(
        housing,
        support_1,
        reason="The negative-side cup support is authored as a fixed welded extension of the housing shell.",
    )

    ctx.expect_overlap(
        clamp,
        cup,
        axes="xy",
        elem_a="clamp_pad",
        elem_b="cup_body",
        min_overlap=0.012,
        name="clamp pad covers the cup opening",
    )
    ctx.expect_gap(
        clamp,
        cup,
        axis="z",
        positive_elem="clamp_pad",
        negative_elem="cup_body",
        min_gap=0.001,
        max_gap=0.008,
        name="closed clamp pad sits just above the cup rim",
    )
    ctx.expect_within(
        blade,
        cup,
        axes="xy",
        inner_elem="blade_rotor",
        outer_elem="cup_body",
        margin=0.004,
        name="blade rotor stays centered inside the cup chamber",
    )

    rest_cup_box = ctx.part_element_world_aabb(cup, elem="cup_body")
    rest_clamp_pad = ctx.part_element_world_aabb(clamp, elem="clamp_pad")

    cup_upper = cup_pitch.motion_limits.upper if cup_pitch.motion_limits is not None else None
    clamp_upper = clamp_hinge.motion_limits.upper if clamp_hinge.motion_limits is not None else None

    tipped_cup_box = None
    open_clamp_pad = None
    if cup_upper is not None and clamp_upper is not None:
        with ctx.pose({cup_pitch: cup_upper, clamp_hinge: clamp_upper}):
            tipped_cup_box = ctx.part_element_world_aabb(cup, elem="cup_body")
            open_clamp_pad = ctx.part_element_world_aabb(clamp, elem="clamp_pad")
            ctx.expect_gap(
                clamp,
                cup,
                axis="z",
                positive_elem="clamp_pad",
                negative_elem="cup_body",
                min_gap=0.030,
                name="open clamp clears the tipped cup",
            )

    rest_cup_center = _aabb_center(rest_cup_box)
    tipped_cup_center = _aabb_center(tipped_cup_box)
    rest_pad_center = _aabb_center(rest_clamp_pad)
    open_pad_center = _aabb_center(open_clamp_pad)

    ctx.check(
        "cup tips forward from the front seat",
        rest_cup_box is not None
        and tipped_cup_box is not None
        and rest_cup_center is not None
        and tipped_cup_center is not None
        and tipped_cup_box[0][1] < rest_cup_box[0][1] - 0.030
        and tipped_cup_center[1] < rest_cup_center[1] - 0.020,
        details=f"rest={rest_cup_box}, tipped={tipped_cup_box}",
    )
    ctx.check(
        "clamp opens upward on the rear hinge",
        rest_pad_center is not None
        and open_pad_center is not None
        and open_pad_center[2] > rest_pad_center[2] + 0.045,
        details=f"rest={rest_clamp_pad}, open={open_clamp_pad}",
    )
    ctx.check(
        "blade articulation is continuous",
        blade_spin.articulation_type == ArticulationType.CONTINUOUS
        and blade_spin.motion_limits is not None
        and blade_spin.motion_limits.lower is None
        and blade_spin.motion_limits.upper is None
        and blade_spin.axis == (0.0, 0.0, 1.0),
        details=(
            f"type={blade_spin.articulation_type}, axis={blade_spin.axis}, "
            f"limits={blade_spin.motion_limits}"
        ),
    )
    return ctx.report()


object_model = build_object_model()
