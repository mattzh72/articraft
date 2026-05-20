from __future__ import annotations

from math import pi

import cadquery as cq

from sdk import (
    ArticulatedObject,
    ArticulationType,
    Box,
    Cylinder,
    Inertial,
    MotionLimits,
    Origin,
    TestContext,
    TestReport,
    mesh_from_cadquery,
)


BODY_W = 0.64
BODY_D = 0.42
BODY_H = 0.335
BODY_WALL = 0.008
BODY_Z = 0.03

LID_W = 0.652
LID_D = 0.432
LID_H = 0.082
LID_WALL = 0.006

HINGE_Y = -0.205
HINGE_Z = BODY_Z + BODY_H

HANDLE_RAIL_X = 0.115
HANDLE_Y = -(BODY_D * 0.5 + 0.020)
HANDLE_JOINT_Z = 0.175
HANDLE_TRAVEL = 0.30

WHEEL_RADIUS = 0.072
WHEEL_WIDTH = 0.045
WHEEL_Y = -0.160
WHEEL_Z = WHEEL_RADIUS
WHEEL_JOINT_X = BODY_W * 0.5 + 0.045


def _body_shell() -> cq.Workplane:
    outer = cq.Workplane("XY").box(BODY_W, BODY_D, BODY_H, centered=(True, True, False))
    inner = (
        cq.Workplane("XY")
        .box(BODY_W - 2.0 * BODY_WALL, BODY_D - 2.0 * BODY_WALL, BODY_H - BODY_WALL, centered=(True, True, False))
        .translate((0.0, 0.0, BODY_WALL))
    )
    shell = outer.cut(inner)
    upper_band = (
        cq.Workplane("XY")
        .box(BODY_W + 0.026, BODY_D + 0.026, 0.060, centered=(True, True, False))
        .translate((0.0, 0.0, BODY_H - 0.120))
        .cut(
            cq.Workplane("XY")
            .box(BODY_W - 0.018, BODY_D - 0.018, 0.070, centered=(True, True, False))
            .translate((0.0, 0.0, BODY_H - 0.126))
        )
    )
    lower_skirt = (
        cq.Workplane("XY")
        .box(BODY_W + 0.020, BODY_D + 0.016, 0.045, centered=(True, True, False))
        .cut(
            cq.Workplane("XY")
            .box(BODY_W - 0.020, BODY_D - 0.020, 0.055, centered=(True, True, False))
            .translate((0.0, 0.0, -0.004))
        )
    )
    front_recess = (
        cq.Workplane("XY")
        .box(0.170, 0.034, 0.040, centered=(True, True, False))
        .translate((0.0, BODY_D * 0.5 - 0.017, BODY_H - 0.108))
    )
    side_recess_left = (
        cq.Workplane("XY")
        .box(0.020, 0.150, 0.110, centered=(True, True, False))
        .translate((-BODY_W * 0.5 + 0.010, 0.020, 0.130))
    )
    side_recess_right = (
        cq.Workplane("XY")
        .box(0.020, 0.150, 0.110, centered=(True, True, False))
        .translate((BODY_W * 0.5 - 0.010, 0.020, 0.130))
    )
    return shell.union(upper_band).union(lower_skirt).cut(front_recess).cut(side_recess_left).cut(side_recess_right)


def _lid_shell() -> cq.Workplane:
    outer = cq.Workplane("XY").box(LID_W, LID_D, LID_H, centered=(True, True, False))
    inner = cq.Workplane("XY").box(
        LID_W - 2.0 * LID_WALL,
        LID_D - 2.0 * LID_WALL,
        LID_H - LID_WALL,
        centered=(True, True, False),
    )
    shell = outer.cut(inner)
    stack_recess = (
        cq.Workplane("XY")
        .box(0.460, 0.240, 0.020, centered=(True, True, False))
        .translate((0.0, 0.015, LID_H - 0.010))
    )
    front_nose = (
        cq.Workplane("XY")
        .box(0.140, 0.030, 0.042, centered=(True, True, False))
        .translate((0.0, LID_D * 0.5 - 0.015, 0.012))
    )
    button_pocket = (
        cq.Workplane("XY")
        .box(0.038, 0.040, 0.022, centered=(True, True, False))
        .translate((0.0, LID_D * 0.5 - 0.022, 0.010))
    )
    return shell.cut(stack_recess).union(front_nose).cut(button_pocket)


def _add_wheel_visuals(part, *, side_sign: float, rubber, rim, hub) -> None:
    spin_origin = Origin(xyz=(side_sign * (WHEEL_WIDTH * 0.5), 0.0, 0.0), rpy=(0.0, pi / 2.0, 0.0))
    part.visual(Cylinder(radius=WHEEL_RADIUS, length=WHEEL_WIDTH), origin=spin_origin, material=rubber, name="tire")
    part.visual(
        Cylinder(radius=WHEEL_RADIUS * 0.76, length=WHEEL_WIDTH * 0.74),
        origin=Origin(
            xyz=(side_sign * (WHEEL_WIDTH * 0.5), 0.0, 0.0),
            rpy=(0.0, pi / 2.0, 0.0),
        ),
        material=rim,
        name="rim",
    )
    part.visual(
        Cylinder(radius=WHEEL_RADIUS * 0.43, length=0.010),
        origin=Origin(
            xyz=(side_sign * (WHEEL_WIDTH - 0.010) * 0.5, 0.0, 0.0),
            rpy=(0.0, pi / 2.0, 0.0),
        ),
        material=hub,
        name="hub_cap",
    )
    part.visual(
        Cylinder(radius=WHEEL_RADIUS * 0.28, length=0.008),
        origin=Origin(
            xyz=(side_sign * 0.004, 0.0, 0.0),
            rpy=(0.0, pi / 2.0, 0.0),
        ),
        material=hub,
        name="hub_inner",
    )


def build_object_model() -> ArticulatedObject:
    model = ArticulatedObject(name="rolling_toolbox_base")

    body_plastic = model.material("body_plastic", rgba=(0.16, 0.17, 0.18, 1.0))
    lid_plastic = model.material("lid_plastic", rgba=(0.22, 0.23, 0.25, 1.0))
    trim_black = model.material("trim_black", rgba=(0.07, 0.07, 0.08, 1.0))
    rail_aluminum = model.material("rail_aluminum", rgba=(0.77, 0.79, 0.81, 1.0))
    wheel_rubber = model.material("wheel_rubber", rgba=(0.05, 0.05, 0.05, 1.0))
    wheel_core = model.material("wheel_core", rgba=(0.42, 0.44, 0.47, 1.0))

    body = model.part("body")
    body.inertial = Inertial.from_geometry(
        Box((BODY_W + 0.10, BODY_D + 0.08, BODY_H + 0.12)),
        mass=18.0,
        origin=Origin(xyz=(0.0, 0.0, BODY_Z + BODY_H * 0.55)),
    )
    body.visual(
        mesh_from_cadquery(_body_shell(), "toolbox_body_shell"),
        origin=Origin(xyz=(0.0, 0.0, BODY_Z)),
        material=body_plastic,
        name="shell",
    )
    body.visual(Box((0.420, 0.018, 0.020)), origin=Origin(xyz=(0.0, HINGE_Y - 0.006, HINGE_Z - 0.012)), material=trim_black, name="hinge_backstrap")
    for x_pos, length, name in [(-0.220, 0.120, "hinge_barrel_0"), (0.0, 0.100, "hinge_barrel_1"), (0.220, 0.120, "hinge_barrel_2")]:
        body.visual(
            Cylinder(radius=0.008, length=length),
            origin=Origin(xyz=(x_pos, HINGE_Y - 0.013, HINGE_Z + 0.002), rpy=(0.0, pi / 2.0, 0.0)),
            material=trim_black,
            name=name,
        )
    for side_name, side_sign in [("left", -1.0), ("right", 1.0)]:
        sleeve_x = side_sign * HANDLE_RAIL_X
        body.visual(
            Box((0.028, 0.008, 0.210)),
            origin=Origin(xyz=(sleeve_x, HANDLE_Y - 0.008, 0.230)),
            material=trim_black,
            name=f"{side_name}_sleeve_back",
        )
        body.visual(
            Box((0.006, 0.040, 0.210)),
            origin=Origin(xyz=(sleeve_x - side_sign * 0.010, HANDLE_Y + 0.009, 0.230)),
            material=trim_black,
            name=f"{side_name}_sleeve_inner",
        )
        body.visual(
            Box((0.006, 0.040, 0.210)),
            origin=Origin(xyz=(sleeve_x + side_sign * 0.010, HANDLE_Y + 0.009, 0.230)),
            material=trim_black,
            name=f"{side_name}_sleeve_outer",
        )
        body.visual(
            Box((0.052, 0.110, 0.115)),
            origin=Origin(xyz=(side_sign * 0.308, WHEEL_Y + 0.020, 0.100)),
            material=trim_black,
            name=f"{side_name}_wheel_pod",
        )
        body.visual(
            Box((0.034, 0.072, 0.050)),
            origin=Origin(xyz=(side_sign * 0.324, WHEEL_Y - 0.030, 0.050)),
            material=trim_black,
            name=f"{side_name}_wheel_brace",
        )
        body.visual(
            Cylinder(radius=0.012, length=0.030),
            origin=Origin(
                xyz=(side_sign * (WHEEL_JOINT_X - 0.015), WHEEL_Y, WHEEL_Z),
                rpy=(0.0, pi / 2.0, 0.0),
            ),
            material=wheel_core,
            name=f"{side_name}_axle_pin",
        )
        body.visual(
            Box((0.014, 0.018, 0.032)),
            origin=Origin(
                xyz=(side_sign * 0.332, 0.058, 0.283),
            ),
            material=trim_black,
            name=f"{side_name}_latch_pin",
        )
    body.visual(
        Box((0.180, 0.022, 0.030)),
        origin=Origin(xyz=(0.0, BODY_D * 0.5 - 0.012, HINGE_Z - 0.035)),
        material=trim_black,
        name="front_striker",
    )
    body.visual(
        Box((0.080, 0.080, 0.040)),
        origin=Origin(xyz=(-0.190, BODY_D * 0.5 - 0.010, 0.020)),
        material=trim_black,
        name="left_foot",
    )
    body.visual(
        Box((0.080, 0.080, 0.040)),
        origin=Origin(xyz=(0.190, BODY_D * 0.5 - 0.010, 0.020)),
        material=trim_black,
        name="right_foot",
    )

    lid = model.part("lid")
    lid.inertial = Inertial.from_geometry(
        Box((LID_W, LID_D, LID_H)),
        mass=4.0,
        origin=Origin(xyz=(0.0, 0.212, 0.042)),
    )
    lid.visual(
        mesh_from_cadquery(_lid_shell(), "toolbox_lid_shell"),
        origin=Origin(xyz=(0.0, LID_D * 0.5 - 0.004, 0.002)),
        material=lid_plastic,
        name="shell",
    )
    lid.visual(
        Box((0.090, 0.016, 0.020)),
        origin=Origin(xyz=(-0.110, -0.008, 0.010)),
        material=trim_black,
        name="hinge_bridge_0",
    )
    lid.visual(
        Box((0.090, 0.016, 0.020)),
        origin=Origin(xyz=(0.110, -0.008, 0.010)),
        material=trim_black,
        name="hinge_bridge_1",
    )
    for x_pos, name in [(-0.110, "hinge_barrel_0"), (0.110, "hinge_barrel_1")]:
        lid.visual(
            Cylinder(radius=0.0075, length=0.100),
            origin=Origin(xyz=(x_pos, -0.013, 0.002), rpy=(0.0, pi / 2.0, 0.0)),
            material=trim_black,
            name=name,
        )
    for side_name, side_sign in [("left", -1.0), ("right", 1.0)]:
        lid.visual(
            Box((0.018, 0.040, 0.026)),
            origin=Origin(xyz=(side_sign * 0.315, 0.295, 0.014)),
            material=trim_black,
            name=f"{side_name}_keeper",
        )

    button = model.part("button")
    button.inertial = Inertial.from_geometry(
        Box((0.060, 0.030, 0.020)),
        mass=0.12,
        origin=Origin(xyz=(0.0, -0.006, 0.0)),
    )
    button.visual(Box((0.056, 0.014, 0.020)), origin=Origin(xyz=(0.0, 0.002, 0.0)), material=trim_black, name="cap")
    button.visual(Box((0.030, 0.024, 0.016)), origin=Origin(xyz=(0.0, -0.014, 0.0)), material=trim_black, name="stem")

    handle = model.part("handle")
    handle.inertial = Inertial.from_geometry(
        Box((0.30, 0.04, 0.56)),
        mass=1.8,
        origin=Origin(xyz=(0.0, 0.0, 0.10)),
    )
    for side_name, side_sign in [("left", -1.0), ("right", 1.0)]:
        handle.visual(
            Cylinder(radius=0.0065, length=0.480),
            origin=Origin(xyz=(side_sign * HANDLE_RAIL_X, 0.0, 0.055)),
            material=rail_aluminum,
            name=f"{side_name}_rail",
        )
        handle.visual(
            Box((0.036, 0.032, 0.066)),
            origin=Origin(xyz=(side_sign * HANDLE_RAIL_X, -0.018, 0.275)),
            material=trim_black,
            name=f"{side_name}_corner",
        )
        handle.visual(
            Box((0.022, 0.020, 0.026)),
            origin=Origin(xyz=(side_sign * HANDLE_RAIL_X, -0.008, 0.175)),
            material=trim_black,
            name=f"{side_name}_clamp",
        )
    handle.visual(
        Box((0.290, 0.032, 0.028)),
        origin=Origin(xyz=(0.0, -0.018, 0.313)),
        material=trim_black,
        name="grip",
    )
    handle.visual(
        Box((0.250, 0.020, 0.014)),
        origin=Origin(xyz=(0.0, -0.007, 0.175)),
        material=trim_black,
        name="cross_brace",
    )

    left_latch = model.part("left_latch")
    left_latch.inertial = Inertial.from_geometry(
        Box((0.024, 0.045, 0.100)),
        mass=0.22,
        origin=Origin(xyz=(-0.010, 0.012, 0.050)),
    )
    left_latch.visual(
        Cylinder(radius=0.009, length=0.018),
        origin=Origin(xyz=(-0.009, 0.0, 0.0), rpy=(0.0, pi / 2.0, 0.0)),
        material=trim_black,
        name="pivot",
    )
    left_latch.visual(
        Box((0.010, 0.018, 0.074)),
        origin=Origin(xyz=(0.0, 0.010, 0.037)),
        material=trim_black,
        name="strap",
    )
    left_latch.visual(
        Box((0.016, 0.028, 0.014)),
        origin=Origin(xyz=(0.004, 0.020, 0.078)),
        material=trim_black,
        name="hook",
    )

    right_latch = model.part("right_latch")
    right_latch.inertial = Inertial.from_geometry(
        Box((0.024, 0.045, 0.100)),
        mass=0.22,
        origin=Origin(xyz=(0.010, 0.012, 0.050)),
    )
    right_latch.visual(
        Cylinder(radius=0.009, length=0.018),
        origin=Origin(xyz=(0.009, 0.0, 0.0), rpy=(0.0, pi / 2.0, 0.0)),
        material=trim_black,
        name="pivot",
    )
    right_latch.visual(
        Box((0.010, 0.018, 0.074)),
        origin=Origin(xyz=(0.0, 0.010, 0.037)),
        material=trim_black,
        name="strap",
    )
    right_latch.visual(
        Box((0.016, 0.028, 0.014)),
        origin=Origin(xyz=(-0.004, 0.020, 0.078)),
        material=trim_black,
        name="hook",
    )

    left_wheel = model.part("left_wheel")
    left_wheel.inertial = Inertial.from_geometry(
        Cylinder(radius=WHEEL_RADIUS, length=WHEEL_WIDTH),
        mass=1.4,
        origin=Origin(xyz=(-WHEEL_WIDTH * 0.5, 0.0, 0.0), rpy=(0.0, pi / 2.0, 0.0)),
    )
    _add_wheel_visuals(left_wheel, side_sign=-1.0, rubber=wheel_rubber, rim=wheel_core, hub=trim_black)

    right_wheel = model.part("right_wheel")
    right_wheel.inertial = Inertial.from_geometry(
        Cylinder(radius=WHEEL_RADIUS, length=WHEEL_WIDTH),
        mass=1.4,
        origin=Origin(xyz=(WHEEL_WIDTH * 0.5, 0.0, 0.0), rpy=(0.0, pi / 2.0, 0.0)),
    )
    _add_wheel_visuals(right_wheel, side_sign=1.0, rubber=wheel_rubber, rim=wheel_core, hub=trim_black)

    model.articulation(
        "body_to_lid",
        ArticulationType.REVOLUTE,
        parent=body,
        child=lid,
        origin=Origin(xyz=(0.0, HINGE_Y, HINGE_Z)),
        axis=(1.0, 0.0, 0.0),
        motion_limits=MotionLimits(effort=25.0, velocity=1.8, lower=0.0, upper=1.55),
    )
    model.articulation(
        "body_to_handle",
        ArticulationType.PRISMATIC,
        parent=body,
        child=handle,
        origin=Origin(xyz=(0.0, HANDLE_Y, HANDLE_JOINT_Z)),
        axis=(0.0, 0.0, 1.0),
        motion_limits=MotionLimits(effort=40.0, velocity=0.35, lower=0.0, upper=HANDLE_TRAVEL),
    )
    model.articulation(
        "lid_to_button",
        ArticulationType.PRISMATIC,
        parent=lid,
        child=button,
        origin=Origin(xyz=(0.0, 0.422, 0.026)),
        axis=(0.0, -1.0, 0.0),
        motion_limits=MotionLimits(effort=8.0, velocity=0.12, lower=0.0, upper=0.012),
    )
    model.articulation(
        "left_latch_pivot",
        ArticulationType.REVOLUTE,
        parent=body,
        child=left_latch,
        origin=Origin(xyz=(-0.338, 0.070, 0.283)),
        axis=(1.0, 0.0, 0.0),
        motion_limits=MotionLimits(effort=6.0, velocity=2.2, lower=0.0, upper=1.10),
    )
    model.articulation(
        "right_latch_pivot",
        ArticulationType.REVOLUTE,
        parent=body,
        child=right_latch,
        origin=Origin(xyz=(0.338, 0.070, 0.283)),
        axis=(1.0, 0.0, 0.0),
        motion_limits=MotionLimits(effort=6.0, velocity=2.2, lower=0.0, upper=1.10),
    )
    model.articulation(
        "left_wheel_spin",
        ArticulationType.CONTINUOUS,
        parent=body,
        child=left_wheel,
        origin=Origin(xyz=(-WHEEL_JOINT_X, WHEEL_Y, WHEEL_Z)),
        axis=(1.0, 0.0, 0.0),
        motion_limits=MotionLimits(effort=12.0, velocity=20.0),
    )
    model.articulation(
        "right_wheel_spin",
        ArticulationType.CONTINUOUS,
        parent=body,
        child=right_wheel,
        origin=Origin(xyz=(WHEEL_JOINT_X, WHEEL_Y, WHEEL_Z)),
        axis=(1.0, 0.0, 0.0),
        motion_limits=MotionLimits(effort=12.0, velocity=20.0),
    )

    return model


def run_tests() -> TestReport:
    ctx = TestContext(object_model)

    body = object_model.get_part("body")
    lid = object_model.get_part("lid")
    handle = object_model.get_part("handle")
    button = object_model.get_part("button")
    left_latch = object_model.get_part("left_latch")
    right_latch = object_model.get_part("right_latch")
    lid_hinge = object_model.get_articulation("body_to_lid")
    handle_slide = object_model.get_articulation("body_to_handle")
    button_slide = object_model.get_articulation("lid_to_button")
    left_latch_pivot = object_model.get_articulation("left_latch_pivot")

    ctx.allow_overlap(
        button,
        lid,
        elem_a="stem",
        elem_b="shell",
        reason="The push button retracts into a simplified latch cavity inside the lid nose.",
    )
    ctx.allow_overlap(
        button,
        lid,
        elem_a="cap",
        elem_b="shell",
        reason="The button cap is intentionally seated into a shallow recessed bezel on the molded lid nose.",
    )

    with ctx.pose({lid_hinge: 0.0}):
        ctx.expect_gap(
            lid,
            body,
            axis="z",
            positive_elem="shell",
            negative_elem="shell",
            min_gap=0.001,
            max_gap=0.010,
            name="lid sits just above the bin rim",
        )
        ctx.expect_overlap(
            lid,
            body,
            axes="xy",
            elem_a="shell",
            elem_b="shell",
            min_overlap=0.34,
            name="lid covers the lower bin opening",
        )
        ctx.expect_gap(
            lid,
            left_latch,
            axis="x",
            positive_elem="left_keeper",
            negative_elem="hook",
            min_gap=0.0,
            max_gap=0.004,
            name="left latch hook sits close to the keeper",
        )
        ctx.expect_gap(
            right_latch,
            lid,
            axis="x",
            positive_elem="hook",
            negative_elem="right_keeper",
            min_gap=0.0,
            max_gap=0.004,
            name="right latch hook sits close to the keeper",
        )

    closed_lid_aabb = ctx.part_element_world_aabb(lid, elem="shell")
    open_lid_aabb = None
    if lid_hinge.motion_limits is not None and lid_hinge.motion_limits.upper is not None:
        with ctx.pose({lid_hinge: lid_hinge.motion_limits.upper}):
            open_lid_aabb = ctx.part_element_world_aabb(lid, elem="shell")
    ctx.check(
        "lid opens upward",
        closed_lid_aabb is not None
        and open_lid_aabb is not None
        and open_lid_aabb[1][2] > closed_lid_aabb[1][2] + 0.12,
        details=f"closed={closed_lid_aabb}, open={open_lid_aabb}",
    )

    collapsed_handle_pos = ctx.part_world_position(handle)
    extended_handle_pos = None
    if handle_slide.motion_limits is not None and handle_slide.motion_limits.upper is not None:
        with ctx.pose({handle_slide: handle_slide.motion_limits.upper}):
            extended_handle_pos = ctx.part_world_position(handle)
    ctx.check(
        "handle extends upward",
        collapsed_handle_pos is not None
        and extended_handle_pos is not None
        and extended_handle_pos[2] > collapsed_handle_pos[2] + 0.20,
        details=f"collapsed={collapsed_handle_pos}, extended={extended_handle_pos}",
    )

    closed_button_aabb = ctx.part_element_world_aabb(button, elem="cap")
    pressed_button_aabb = None
    if button_slide.motion_limits is not None and button_slide.motion_limits.upper is not None:
        with ctx.pose({button_slide: button_slide.motion_limits.upper}):
            pressed_button_aabb = ctx.part_element_world_aabb(button, elem="cap")
    ctx.check(
        "button presses inward",
        closed_button_aabb is not None
        and pressed_button_aabb is not None
        and pressed_button_aabb[1][1] < closed_button_aabb[1][1] - 0.008,
        details=f"closed={closed_button_aabb}, pressed={pressed_button_aabb}",
    )

    closed_latch_aabb = ctx.part_element_world_aabb(left_latch, elem="hook")
    open_latch_aabb = None
    if left_latch_pivot.motion_limits is not None and left_latch_pivot.motion_limits.upper is not None:
        with ctx.pose({left_latch_pivot: left_latch_pivot.motion_limits.upper}):
            open_latch_aabb = ctx.part_element_world_aabb(left_latch, elem="hook")
    ctx.check(
        "side latch rotates clear",
        closed_latch_aabb is not None
        and open_latch_aabb is not None
        and open_latch_aabb[0][1] < closed_latch_aabb[0][1] - 0.015,
        details=f"closed={closed_latch_aabb}, open={open_latch_aabb}",
    )

    return ctx.report()


object_model = build_object_model()
