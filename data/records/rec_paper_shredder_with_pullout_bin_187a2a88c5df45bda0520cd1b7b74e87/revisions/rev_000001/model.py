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
    MotionLimits,
    Origin,
    TestContext,
    TestReport,
    mesh_from_geometry,
    mesh_from_cadquery,
)

BODY_W = 0.260
BODY_D = 0.170
BODY_H = 0.430
BODY_WALL = 0.0035
BODY_BOTTOM = 0.008
BODY_TOP = 0.020

DRAWER_W = 0.228
DRAWER_D = 0.128
DRAWER_H = 0.168
DRAWER_WALL = 0.003
DRAWER_FACE_T = 0.010

POD_W = 0.042
POD_D = 0.070
POD_H = 0.064


def _body_shape() -> cq.Workplane:
    shell = (
        cq.Workplane("XY")
        .box(BODY_W, BODY_D, BODY_H)
        .translate((0.0, 0.0, BODY_H / 2.0))
        .edges("|Z")
        .fillet(0.012)
    )
    shell = shell.edges(">Z").fillet(0.006)

    inner_height = BODY_H - BODY_TOP - BODY_BOTTOM
    inner_cavity = (
        cq.Workplane("XY")
        .box(BODY_W - 2.0 * BODY_WALL, BODY_D - 2.0 * BODY_WALL, inner_height)
        .translate((0.0, 0.0, BODY_BOTTOM + inner_height / 2.0))
    )
    shell = shell.cut(inner_cavity)

    drawer_opening = (
        cq.Workplane("XY")
        .box(BODY_W - 0.026, 0.026, 0.188)
        .translate((0.0, -BODY_D / 2.0 + 0.013, 0.114))
    )
    shell = shell.cut(drawer_opening)

    throat_slot = (
        cq.Workplane("XY")
        .box(0.220, 0.006, 0.028)
        .translate((0.0, -0.006, BODY_H - 0.007))
    )
    shell = shell.cut(throat_slot)

    throat_relief = (
        cq.Workplane("XY")
        .box(0.222, 0.026, 0.012)
        .translate((0.0, -0.006, BODY_H - 0.018))
    )
    return shell.cut(throat_relief)


def _drawer_shape() -> cq.Workplane:
    drawer = (
        cq.Workplane("XY")
        .box(DRAWER_W, DRAWER_D, DRAWER_H)
        .translate((0.0, -DRAWER_D / 2.0, DRAWER_H / 2.0))
        .faces(">Z")
        .shell(-DRAWER_WALL)
    )

    front_panel = (
        cq.Workplane("XY")
        .box(DRAWER_W + 0.014, DRAWER_FACE_T, DRAWER_H + 0.008)
        .translate((0.0, -DRAWER_D - DRAWER_FACE_T / 2.0, DRAWER_H / 2.0))
    )
    drawer = drawer.union(front_panel)

    pull_recess = (
        cq.Workplane("XY")
        .box(0.094, 0.004, 0.022)
        .translate((0.0, -DRAWER_D - 0.008, 0.112))
    )
    return drawer.cut(pull_recess)


def _control_pod_shape() -> cq.Workplane:
    profile = [(-0.034, -0.030), (0.034, -0.030), (0.030, 0.020), (0.014, 0.032), (-0.024, 0.032), (-0.034, 0.012)]
    return (
        cq.Workplane("YZ")
        .polyline(profile)
        .close()
        .extrude(POD_W)
        .edges("|X")
        .fillet(0.004)
    )


def build_object_model() -> ArticulatedObject:
    model = ArticulatedObject(name="desk_shredder")

    shell_finish = model.material("shell_finish", rgba=(0.18, 0.19, 0.20, 1.0))
    drawer_finish = model.material("drawer_finish", rgba=(0.12, 0.13, 0.14, 1.0))
    cutter_metal = model.material("cutter_metal", rgba=(0.58, 0.60, 0.62, 1.0))
    control_finish = model.material("control_finish", rgba=(0.14, 0.15, 0.16, 1.0))
    control_dark = model.material("control_dark", rgba=(0.08, 0.09, 0.10, 1.0))
    control_grey = model.material("control_grey", rgba=(0.34, 0.36, 0.38, 1.0))

    body = model.part("body")
    body.visual(
        Box((0.006, BODY_D, BODY_H)),
        origin=Origin(xyz=(-BODY_W / 2.0 + 0.003, 0.0, BODY_H / 2.0)),
        material=shell_finish,
        name="left_wall",
    )
    body.visual(
        Box((0.006, BODY_D, BODY_H)),
        origin=Origin(xyz=(BODY_W / 2.0 - 0.003, 0.0, BODY_H / 2.0)),
        material=shell_finish,
        name="right_wall",
    )
    body.visual(
        Box((BODY_W - 0.012, 0.006, BODY_H)),
        origin=Origin(xyz=(0.0, BODY_D / 2.0 - 0.003, BODY_H / 2.0)),
        material=shell_finish,
        name="rear_wall",
    )
    body.visual(
        Box((BODY_W - 0.012, BODY_D - 0.006, 0.008)),
        origin=Origin(xyz=(0.0, 0.003, 0.004)),
        material=shell_finish,
        name="floor",
    )
    body.visual(
        Box((BODY_W - 0.012, 0.006, 0.202)),
        origin=Origin(xyz=(0.0, -BODY_D / 2.0 + 0.003, 0.309)),
        material=shell_finish,
        name="front_upper",
    )
    body.visual(
        Box((0.010, 0.112, 0.024)),
        origin=Origin(xyz=(-0.119, -0.004, 0.040)),
        material=shell_finish,
        name="left_runner",
    )
    body.visual(
        Box((0.010, 0.112, 0.024)),
        origin=Origin(xyz=(0.119, -0.004, 0.040)),
        material=shell_finish,
        name="right_runner",
    )
    body.visual(
        Box((BODY_W - 0.012, 0.082, BODY_TOP)),
        origin=Origin(xyz=(0.0, -0.044, BODY_H - BODY_TOP / 2.0)),
        material=shell_finish,
        name="top_front",
    )
    body.visual(
        Box((BODY_W - 0.012, 0.082, BODY_TOP)),
        origin=Origin(xyz=(0.0, 0.044, BODY_H - BODY_TOP / 2.0)),
        material=shell_finish,
        name="top_rear",
    )

    drawer = model.part("drawer")
    drawer.visual(
        Box((DRAWER_W, DRAWER_D, DRAWER_WALL)),
        origin=Origin(xyz=(0.0, -DRAWER_D / 2.0, DRAWER_WALL / 2.0)),
        material=drawer_finish,
        name="bottom",
    )
    drawer.visual(
        Box((DRAWER_WALL, DRAWER_D, DRAWER_H)),
        origin=Origin(
            xyz=(-DRAWER_W / 2.0 + DRAWER_WALL / 2.0, -DRAWER_D / 2.0, DRAWER_H / 2.0)
        ),
        material=drawer_finish,
        name="left_side",
    )
    drawer.visual(
        Box((DRAWER_WALL, DRAWER_D, DRAWER_H)),
        origin=Origin(
            xyz=(DRAWER_W / 2.0 - DRAWER_WALL / 2.0, -DRAWER_D / 2.0, DRAWER_H / 2.0)
        ),
        material=drawer_finish,
        name="right_side",
    )
    drawer.visual(
        Box((DRAWER_W - 2.0 * DRAWER_WALL, DRAWER_WALL, DRAWER_H)),
        origin=Origin(xyz=(0.0, -DRAWER_WALL / 2.0, DRAWER_H / 2.0)),
        material=drawer_finish,
        name="rear_wall",
    )
    drawer.visual(
        Box((DRAWER_W - 2.0 * DRAWER_WALL, DRAWER_WALL, DRAWER_H)),
        origin=Origin(xyz=(0.0, -DRAWER_D + DRAWER_WALL / 2.0, DRAWER_H / 2.0)),
        material=drawer_finish,
        name="front_wall",
    )
    drawer.visual(
        Box((DRAWER_W + 0.014, DRAWER_FACE_T, DRAWER_H + 0.008)),
        origin=Origin(xyz=(0.0, -DRAWER_D - DRAWER_FACE_T / 2.0, (DRAWER_H + 0.008) / 2.0)),
        material=drawer_finish,
        name="front_panel",
    )

    model.articulation(
        "body_to_drawer",
        ArticulationType.PRISMATIC,
        parent=body,
        child=drawer,
        origin=Origin(xyz=(0.0, 0.053, 0.024)),
        axis=(0.0, -1.0, 0.0),
        motion_limits=MotionLimits(
            effort=25.0,
            velocity=0.22,
            lower=0.0,
            upper=0.086,
        ),
    )

    cutter_pitch = 0.021
    cutter_centers = [(-5 + index) * cutter_pitch for index in range(11)]
    for part_name, joint_name, center_y, center_z, x_offset in (
        ("drum_front", "body_to_drum_front", -0.010, 0.374, 0.0),
        ("drum_rear", "body_to_drum_rear", 0.011, 0.381, cutter_pitch / 2.0),
    ):
        drum = model.part(part_name)
        drum.visual(
            Cylinder(radius=0.0065, length=BODY_W - 0.012),
            origin=Origin(rpy=(0.0, math.pi / 2.0, 0.0)),
            material=cutter_metal,
            name="shaft",
        )
        for index, center_x in enumerate(cutter_centers):
            shifted_x = center_x + x_offset
            if abs(shifted_x) > 0.108:
                continue
            drum.visual(
                Cylinder(radius=0.0115, length=0.011),
                origin=Origin(xyz=(shifted_x, 0.0, 0.0), rpy=(0.0, math.pi / 2.0, 0.0)),
                material=cutter_metal,
                name=f"cutter_{index}",
            )

        model.articulation(
            joint_name,
            ArticulationType.CONTINUOUS,
            parent=body,
            child=drum,
            origin=Origin(xyz=(0.0, center_y, center_z)),
            axis=(1.0, 0.0, 0.0),
            motion_limits=MotionLimits(
                effort=6.0,
                velocity=22.0,
            ),
        )

    control_pod = model.part("control_pod")
    control_pod.visual(
        mesh_from_cadquery(_control_pod_shape(), "control_pod"),
        material=control_finish,
        name="housing",
    )
    model.articulation(
        "body_to_control_pod",
        ArticulationType.FIXED,
        parent=body,
        child=control_pod,
        origin=Origin(xyz=(BODY_W / 2.0, 0.008, 0.336)),
    )

    power_rocker = model.part("power_rocker")
    power_rocker.visual(
        Cylinder(radius=0.0025, length=0.018),
        origin=Origin(rpy=(math.pi / 2.0, 0.0, 0.0)),
        material=control_dark,
        name="pivot",
    )
    power_rocker.visual(
        Box((0.012, 0.018, 0.024)),
        origin=Origin(xyz=(0.006, 0.0, 0.0)),
        material=control_dark,
        name="cap",
    )
    model.articulation(
        "control_pod_to_power_rocker",
        ArticulationType.REVOLUTE,
        parent=control_pod,
        child=power_rocker,
        origin=Origin(xyz=(POD_W, 0.005, 0.014)),
        axis=(0.0, 1.0, 0.0),
        motion_limits=MotionLimits(
            effort=0.3,
            velocity=4.0,
            lower=-0.32,
            upper=0.32,
        ),
    )

    reverse_button = model.part("reverse_button")
    reverse_button.visual(
        Box((0.005, 0.014, 0.012)),
        origin=Origin(xyz=(0.0025, 0.0, 0.0)),
        material=control_grey,
        name="cap",
    )
    model.articulation(
        "control_pod_to_reverse_button",
        ArticulationType.PRISMATIC,
        parent=control_pod,
        child=reverse_button,
        origin=Origin(xyz=(POD_W, 0.004, -0.006)),
        axis=(-1.0, 0.0, 0.0),
        motion_limits=MotionLimits(
            effort=8.0,
            velocity=0.05,
            lower=0.0,
            upper=0.003,
        ),
    )

    timer_knob = model.part("timer_knob")
    timer_knob.visual(
        mesh_from_geometry(
            KnobGeometry(
                0.024,
                0.014,
                body_style="tapered",
                top_diameter=0.020,
                edge_radius=0.001,
                grip=KnobGrip(style="fluted", count=14, depth=0.0009),
                indicator=KnobIndicator(style="line", mode="raised", angle_deg=0.0),
                center=False,
            ),
            "timer_knob",
        ),
        origin=Origin(rpy=(0.0, math.pi / 2.0, 0.0)),
        material=control_dark,
        name="knob",
    )
    model.articulation(
        "control_pod_to_timer_knob",
        ArticulationType.CONTINUOUS,
        parent=control_pod,
        child=timer_knob,
        origin=Origin(xyz=(POD_W, 0.006, -0.026)),
        axis=(1.0, 0.0, 0.0),
        motion_limits=MotionLimits(
            effort=0.2,
            velocity=8.0,
        ),
    )

    return model


def run_tests() -> TestReport:
    ctx = TestContext(object_model)
    body = object_model.get_part("body")
    control_pod = object_model.get_part("control_pod")
    drawer = object_model.get_part("drawer")
    drum_front = object_model.get_part("drum_front")
    drum_rear = object_model.get_part("drum_rear")
    power_rocker = object_model.get_part("power_rocker")
    reverse_button = object_model.get_part("reverse_button")
    timer_knob = object_model.get_part("timer_knob")
    drawer_slide = object_model.get_articulation("body_to_drawer")
    reverse_joint = object_model.get_articulation("control_pod_to_reverse_button")

    ctx.expect_within(
        drawer,
        body,
        axes="xz",
        margin=0.008,
        name="drawer stays centered in lower body opening",
    )
    ctx.expect_overlap(
        drawer,
        body,
        axes="x",
        min_overlap=0.200,
        name="drawer spans most of shredder width",
    )

    body_aabb = ctx.part_world_aabb(body)
    drawer_aabb = ctx.part_world_aabb(drawer)
    closed_front_ok = False
    if body_aabb is not None and drawer_aabb is not None:
        body_min, _ = body_aabb
        drawer_min, _ = drawer_aabb
        front_delta = float(drawer_min[1] - body_min[1])
        closed_front_ok = -0.004 <= front_delta <= 0.004
        ctx.check(
            "drawer front sits flush when closed",
            closed_front_ok,
            details=f"front_delta={front_delta:.5f}",
        )
    else:
        ctx.fail("drawer front sits flush when closed", "Missing AABB for body or drawer.")

    slide_upper = drawer_slide.motion_limits.upper if drawer_slide.motion_limits is not None else None
    rest_pos = ctx.part_world_position(drawer)
    with ctx.pose({drawer_slide: slide_upper if slide_upper is not None else 0.086}):
        ctx.expect_overlap(
            drawer,
            body,
            axes="y",
            min_overlap=0.035,
            name="opened drawer remains retained by the cabinet",
        )
        extended_pos = ctx.part_world_position(drawer)
        ctx.check(
            "drawer pulls forward when opened",
            rest_pos is not None
            and extended_pos is not None
            and extended_pos[1] < rest_pos[1] - 0.05,
            details=f"rest={rest_pos}, extended={extended_pos}",
        )

    body_aabb = ctx.part_world_aabb(body)
    pod_aabb = ctx.part_world_aabb(control_pod)
    ctx.check(
        "control pod protrudes from the right side",
        body_aabb is not None and pod_aabb is not None and pod_aabb[1][0] > body_aabb[1][0] + 0.030,
        details=f"body_aabb={body_aabb}, pod_aabb={pod_aabb}",
    )

    front_pos = ctx.part_world_position(drum_front)
    rear_pos = ctx.part_world_position(drum_rear)
    ctx.check(
        "cutter drums sit directly beneath the feed slot",
        front_pos is not None
        and rear_pos is not None
        and 0.36 <= front_pos[2] <= 0.39
        and 0.36 <= rear_pos[2] <= 0.39
        and front_pos[1] < 0.0 < rear_pos[1],
        details=f"front={front_pos}, rear={rear_pos}",
    )

    rocker_pos = ctx.part_world_position(power_rocker)
    button_pos = ctx.part_world_position(reverse_button)
    knob_pos = ctx.part_world_position(timer_knob)
    ctx.check(
        "control stack is ordered rocker button knob",
        rocker_pos is not None
        and button_pos is not None
        and knob_pos is not None
        and rocker_pos[2] > button_pos[2] > knob_pos[2],
        details=f"rocker={rocker_pos}, button={button_pos}, knob={knob_pos}",
    )

    reverse_rest = button_pos
    reverse_upper = reverse_joint.motion_limits.upper if reverse_joint.motion_limits is not None else None
    with ctx.pose({reverse_joint: reverse_upper if reverse_upper is not None else 0.003}):
        reverse_pressed = ctx.part_world_position(reverse_button)
        ctx.check(
            "reverse button presses inward",
            reverse_rest is not None
            and reverse_pressed is not None
            and reverse_pressed[0] < reverse_rest[0] - 0.002,
            details=f"rest={reverse_rest}, pressed={reverse_pressed}",
        )

    return ctx.report()


object_model = build_object_model()
