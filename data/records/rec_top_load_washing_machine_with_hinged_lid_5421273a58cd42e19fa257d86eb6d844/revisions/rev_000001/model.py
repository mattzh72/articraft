from __future__ import annotations

import math

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


CABINET_W = 0.54
CABINET_D = 0.56
CABINET_H = 0.74
OPENING_W = 0.42
OPENING_D = 0.39
OPENING_Y = -0.035
LID_W = 0.455
LID_D = 0.445
LID_T = 0.026
POD_W = 0.44
POD_D = 0.11
POD_H = 0.12
TUB_OUTER_R = 0.168
TUB_INNER_R = 0.148
TUB_H = 0.39
TUB_BASE_Z = 0.28
WHEEL_R = 0.022
WHEEL_W = 0.016


def _body_mesh():
    opening = (
        cq.Workplane("XY")
        .box(OPENING_W, OPENING_D, CABINET_H + 0.04, centered=(True, True, False))
        .edges("|Z")
        .fillet(0.028)
        .translate((0.0, OPENING_Y, 0.05))
    )
    return (
        cq.Workplane("XY")
        .box(CABINET_W, CABINET_D, CABINET_H, centered=(True, True, False))
        .edges("|Z")
        .fillet(0.028)
        .cut(opening)
    )


def _control_pod_mesh():
    wall = 0.015
    front_wall = 0.018
    back_wall = 0.012
    top_wall = 0.015

    pod = (
        cq.Workplane("XY")
        .box(POD_W, POD_D, POD_H, centered=(True, False, False))
        .edges("<Y and >Z")
        .chamfer(0.028)
    )

    inner = (
        cq.Workplane("XY")
        .box(
            POD_W - 2.0 * wall,
            POD_D - front_wall - back_wall,
            POD_H - top_wall,
            centered=(True, False, False),
        )
        .translate((0.0, front_wall, 0.0))
    )

    dial_hole = (
        cq.Workplane("XY")
        .box(0.022, front_wall + 0.006, 0.022, centered=(True, False, True))
        .translate((-0.105, -0.003, 0.072))
    )
    button_hole_0 = (
        cq.Workplane("XY")
        .box(0.020, front_wall + 0.006, 0.012, centered=(True, False, True))
        .translate((0.075, -0.003, 0.046))
    )
    button_hole_1 = (
        cq.Workplane("XY")
        .box(0.020, front_wall + 0.006, 0.012, centered=(True, False, True))
        .translate((0.125, -0.003, 0.046))
    )

    return pod.cut(inner).cut(dial_hole).cut(button_hole_0).cut(button_hole_1)


def _lid_frame_mesh():
    frame = (
        cq.Workplane("XY")
        .box(LID_W, LID_D, LID_T, centered=(True, False, False))
        .translate((0.0, -LID_D, 0.0))
        .edges("|Z")
        .fillet(0.008)
    )
    window_cut = (
        cq.Workplane("XY")
        .box(0.325, 0.305, LID_T + 0.01, centered=(True, False, False))
        .translate((0.0, -0.355, 0.006))
        .edges("|Z")
        .fillet(0.016)
    )
    grip = (
        cq.Workplane("XY")
        .box(0.15, 0.02, 0.010, centered=(True, False, False))
        .translate((0.0, -LID_D + 0.008, 0.004))
    )
    return frame.cut(window_cut).union(grip)


def _tub_mesh():
    shell = (
        cq.Workplane("XY")
        .circle(TUB_OUTER_R)
        .extrude(TUB_H)
        .cut(
            cq.Workplane("XY")
            .circle(TUB_INNER_R)
            .extrude(TUB_H - 0.03)
            .translate((0.0, 0.0, 0.03))
        )
    )
    rim = (
        cq.Workplane("XY")
        .circle(TUB_OUTER_R + 0.010)
        .circle(TUB_INNER_R)
        .extrude(0.015)
        .translate((0.0, 0.0, TUB_H - 0.015))
    )
    return shell.union(rim).edges(">Z").fillet(0.006)


def _visual_center(aabb):
    if aabb is None:
        return None
    minimum, maximum = aabb
    return tuple((minimum[i] + maximum[i]) * 0.5 for i in range(3))


def build_object_model() -> ArticulatedObject:
    model = ArticulatedObject(name="portable_top_load_washer")

    body_white = model.material("body_white", rgba=(0.94, 0.95, 0.96, 1.0))
    pod_grey = model.material("pod_grey", rgba=(0.70, 0.73, 0.76, 1.0))
    trim_grey = model.material("trim_grey", rgba=(0.56, 0.60, 0.63, 1.0))
    smoked_lid = model.material("smoked_lid", rgba=(0.58, 0.68, 0.76, 0.45))
    stainless = model.material("stainless", rgba=(0.80, 0.82, 0.84, 1.0))
    button_white = model.material("button_white", rgba=(0.95, 0.96, 0.97, 1.0))
    dial_grey = model.material("dial_grey", rgba=(0.82, 0.84, 0.86, 1.0))
    pointer_orange = model.material("pointer_orange", rgba=(0.95, 0.50, 0.20, 1.0))
    rubber_black = model.material("rubber_black", rgba=(0.10, 0.10, 0.11, 1.0))
    wheel_grey = model.material("wheel_grey", rgba=(0.72, 0.74, 0.76, 1.0))

    cabinet = model.part("cabinet")
    cabinet.visual(
        mesh_from_cadquery(_body_mesh(), "washer_cabinet_shell"),
        material=body_white,
        name="cabinet_shell",
    )
    cabinet.visual(
        Cylinder(radius=0.026, length=TUB_BASE_Z - 0.05),
        origin=Origin(
            xyz=(0.0, OPENING_Y, 0.05 + (TUB_BASE_Z - 0.05) * 0.5),
        ),
        material=trim_grey,
        name="drive_post",
    )

    control_pod = model.part("control_pod")
    control_pod.visual(
        mesh_from_cadquery(_control_pod_mesh(), "washer_control_pod"),
        material=pod_grey,
        name="pod_shell",
    )
    model.articulation(
        "cabinet_to_control_pod",
        ArticulationType.FIXED,
        parent=cabinet,
        child=control_pod,
        origin=Origin(xyz=(0.0, 0.17, CABINET_H)),
    )

    lid = model.part("lid")
    lid.visual(
        mesh_from_cadquery(_lid_frame_mesh(), "washer_lid_frame"),
        material=body_white,
        name="lid_frame",
    )
    lid.visual(
        Box((0.335, 0.315, 0.012)),
        origin=Origin(xyz=(0.0, -0.205, 0.014)),
        material=smoked_lid,
        name="lid_window",
    )
    model.articulation(
        "cabinet_to_lid",
        ArticulationType.REVOLUTE,
        parent=cabinet,
        child=lid,
        origin=Origin(xyz=(0.0, 0.17, CABINET_H + 0.001)),
        axis=(-1.0, 0.0, 0.0),
        motion_limits=MotionLimits(
            effort=8.0,
            velocity=1.5,
            lower=0.0,
            upper=1.75,
        ),
    )

    tub = model.part("tub")
    tub.visual(
        mesh_from_cadquery(_tub_mesh(), "washer_tub_shell"),
        material=stainless,
        name="tub_shell",
    )
    tub.visual(
        Box((0.018, 0.085, 0.20)),
        origin=Origin(xyz=(0.141, 0.0, 0.17)),
        material=trim_grey,
        name="tub_lifter",
    )
    model.articulation(
        "cabinet_to_tub",
        ArticulationType.CONTINUOUS,
        parent=cabinet,
        child=tub,
        origin=Origin(xyz=(0.0, OPENING_Y, TUB_BASE_Z)),
        axis=(0.0, 0.0, 1.0),
        motion_limits=MotionLimits(
            effort=18.0,
            velocity=8.0,
        ),
    )

    timer_dial = model.part("timer_dial")
    timer_dial.visual(
        Cylinder(radius=0.026, length=0.015),
        origin=Origin(xyz=(0.0, -0.0075, 0.0), rpy=(math.pi / 2.0, 0.0, 0.0)),
        material=dial_grey,
        name="dial_cap",
    )
    timer_dial.visual(
        Cylinder(radius=0.008, length=0.024),
        origin=Origin(xyz=(0.0, 0.012, 0.0), rpy=(math.pi / 2.0, 0.0, 0.0)),
        material=trim_grey,
        name="dial_shaft",
    )
    timer_dial.visual(
        Box((0.006, 0.014, 0.010)),
        origin=Origin(xyz=(0.019, -0.014, 0.0)),
        material=pointer_orange,
        name="dial_pointer",
    )
    model.articulation(
        "control_pod_to_timer_dial",
        ArticulationType.CONTINUOUS,
        parent=control_pod,
        child=timer_dial,
        origin=Origin(xyz=(-0.105, 0.0, 0.072)),
        axis=(0.0, 1.0, 0.0),
        motion_limits=MotionLimits(
            effort=0.2,
            velocity=4.0,
        ),
    )

    button_x_positions = (0.075, 0.125)
    for index, button_x in enumerate(button_x_positions):
        button = model.part(f"mode_button_{index}")
        button.visual(
            Box((0.028, 0.010, 0.016)),
            origin=Origin(xyz=(0.0, -0.005, 0.0)),
            material=button_white,
            name="button_cap",
        )
        button.visual(
            Box((0.018, 0.020, 0.010)),
            origin=Origin(xyz=(0.0, 0.010, 0.0)),
            material=trim_grey,
            name="button_stem",
        )
        model.articulation(
            f"control_pod_to_mode_button_{index}",
            ArticulationType.PRISMATIC,
            parent=control_pod,
            child=button,
            origin=Origin(xyz=(button_x, 0.0, 0.046)),
            axis=(0.0, 1.0, 0.0),
            motion_limits=MotionLimits(
                effort=5.0,
                velocity=0.08,
                lower=0.0,
                upper=0.006,
            ),
        )

    caster_specs = (
        ("front_left", -0.20, -0.20),
        ("front_right", 0.20, -0.20),
        ("rear_left", -0.20, 0.20),
        ("rear_right", 0.20, 0.20),
    )
    for prefix, pos_x, pos_y in caster_specs:
        fork = model.part(f"{prefix}_fork")
        fork.visual(
            Box((0.042, 0.028, 0.006)),
            origin=Origin(xyz=(0.0, 0.0, -0.003)),
            material=trim_grey,
            name="fork_plate",
        )
        fork.visual(
            Box((0.026, 0.004, 0.030)),
            origin=Origin(xyz=(0.0, -0.011, -0.021)),
            material=trim_grey,
            name="fork_cheek_0",
        )
        fork.visual(
            Box((0.026, 0.004, 0.030)),
            origin=Origin(xyz=(0.0, 0.011, -0.021)),
            material=trim_grey,
            name="fork_cheek_1",
        )
        model.articulation(
            f"cabinet_to_{prefix}_fork",
            ArticulationType.FIXED,
            parent=cabinet,
            child=fork,
            origin=Origin(xyz=(pos_x, pos_y, 0.0)),
        )

        wheel = model.part(f"{prefix}_wheel")
        wheel.visual(
            Cylinder(radius=WHEEL_R, length=WHEEL_W),
            origin=Origin(xyz=(0.0, 0.0, 0.0), rpy=(math.pi / 2.0, 0.0, 0.0)),
            material=rubber_black,
            name="tire",
        )
        wheel.visual(
            Cylinder(radius=0.013, length=WHEEL_W + 0.002),
            origin=Origin(xyz=(0.0, 0.0, 0.0), rpy=(math.pi / 2.0, 0.0, 0.0)),
            material=wheel_grey,
            name="hub",
        )
        wheel.visual(
            Box((0.008, 0.004, 0.012)),
            origin=Origin(xyz=(0.014, 0.011, 0.0)),
            material=wheel_grey,
            name="spoke",
        )
        model.articulation(
            f"{prefix}_fork_to_wheel",
            ArticulationType.CONTINUOUS,
            parent=fork,
            child=wheel,
            origin=Origin(xyz=(0.0, 0.0, -0.035)),
            axis=(0.0, 1.0, 0.0),
            motion_limits=MotionLimits(
                effort=1.0,
                velocity=18.0,
            ),
        )

    return model


def run_tests() -> TestReport:
    ctx = TestContext(object_model)

    cabinet = object_model.get_part("cabinet")
    lid = object_model.get_part("lid")
    tub = object_model.get_part("tub")
    timer_dial = object_model.get_part("timer_dial")
    mode_button_0 = object_model.get_part("mode_button_0")
    front_left_wheel = object_model.get_part("front_left_wheel")

    lid_hinge = object_model.get_articulation("cabinet_to_lid")
    tub_spin = object_model.get_articulation("cabinet_to_tub")
    dial_spin = object_model.get_articulation("control_pod_to_timer_dial")
    button_press = object_model.get_articulation("control_pod_to_mode_button_0")
    wheel_spin = object_model.get_articulation("front_left_fork_to_wheel")

    ctx.expect_within(
        tub,
        cabinet,
        axes="xy",
        margin=0.09,
        inner_elem="tub_shell",
        name="wash tub stays within cabinet footprint",
    )

    closed_lid_center = None
    opened_lid_center = None
    with ctx.pose({lid_hinge: 0.0}):
        closed_lid_center = _visual_center(ctx.part_element_world_aabb(lid, elem="lid_frame"))
    with ctx.pose({lid_hinge: 1.6}):
        opened_lid_center = _visual_center(ctx.part_element_world_aabb(lid, elem="lid_frame"))
    ctx.check(
        "lid opens upward",
        closed_lid_center is not None
        and opened_lid_center is not None
        and opened_lid_center[2] > closed_lid_center[2] + 0.10,
        details=f"closed={closed_lid_center}, open={opened_lid_center}",
    )

    rest_lifter_center = _visual_center(ctx.part_element_world_aabb(tub, elem="tub_lifter"))
    with ctx.pose({tub_spin: math.pi / 2.0}):
        spun_lifter_center = _visual_center(ctx.part_element_world_aabb(tub, elem="tub_lifter"))
    ctx.check(
        "tub rotates around the vertical axis",
        rest_lifter_center is not None
        and spun_lifter_center is not None
        and abs(spun_lifter_center[1] - rest_lifter_center[1]) > 0.10,
        details=f"rest={rest_lifter_center}, spun={spun_lifter_center}",
    )

    rest_pointer_center = _visual_center(ctx.part_element_world_aabb(timer_dial, elem="dial_pointer"))
    with ctx.pose({dial_spin: 1.0}):
        turned_pointer_center = _visual_center(ctx.part_element_world_aabb(timer_dial, elem="dial_pointer"))
    ctx.check(
        "timer dial rotates on the control pod",
        rest_pointer_center is not None
        and turned_pointer_center is not None
        and abs(turned_pointer_center[2] - rest_pointer_center[2]) > 0.010,
        details=f"rest={rest_pointer_center}, turned={turned_pointer_center}",
    )

    rest_button_pos = ctx.part_world_position(mode_button_0)
    with ctx.pose({button_press: button_press.motion_limits.upper}):
        pressed_button_pos = ctx.part_world_position(mode_button_0)
    ctx.check(
        "mode button presses inward",
        rest_button_pos is not None
        and pressed_button_pos is not None
        and pressed_button_pos[1] > rest_button_pos[1] + 0.004,
        details=f"rest={rest_button_pos}, pressed={pressed_button_pos}",
    )

    rest_spoke_center = _visual_center(ctx.part_element_world_aabb(front_left_wheel, elem="spoke"))
    with ctx.pose({wheel_spin: math.pi / 2.0}):
        turned_spoke_center = _visual_center(ctx.part_element_world_aabb(front_left_wheel, elem="spoke"))
    ctx.check(
        "caster wheel rotates on its axle",
        rest_spoke_center is not None
        and turned_spoke_center is not None
        and abs(turned_spoke_center[2] - rest_spoke_center[2]) > 0.010,
        details=f"rest={rest_spoke_center}, turned={turned_spoke_center}",
    )

    return ctx.report()


object_model = build_object_model()
