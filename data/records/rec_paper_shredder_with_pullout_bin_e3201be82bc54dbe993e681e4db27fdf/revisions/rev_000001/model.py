from __future__ import annotations

import math

import cadquery as cq

from sdk import (
    ArticulatedObject,
    ArticulationType,
    Box,
    Cylinder,
    KnobGeometry,
    KnobIndicator,
    KnobSkirt,
    MotionLimits,
    Origin,
    TestContext,
    TestReport,
    mesh_from_cadquery,
    mesh_from_geometry,
)


BODY_W = 0.365
BODY_D = 0.285
LOWER_W = 0.345
LOWER_D = 0.268
LOWER_H = 0.425
LOWER_Z0 = 0.045
HEAD_H = 0.150
HEAD_Z0 = 0.425
BODY_TOP_Z = HEAD_Z0 + HEAD_H

BIN_OPENING_W = 0.331
BIN_OPENING_H = 0.344
BIN_OPENING_Z0 = 0.055
BIN_TRAVEL = 0.155

BUTTON_X = (0.071, 0.095)
BUTTON_Y = 0.050
DIAL_X = 0.128
DIAL_Y = 0.050
SLIDER_HOME_X = -0.072
SLIDER_Y = 0.062


def _world_box(size: tuple[float, float, float], center: tuple[float, float, float]) -> cq.Workplane:
    return cq.Workplane("XY").box(*size).translate(center)


def _world_cylinder(radius: float, height: float, center: tuple[float, float, float]) -> cq.Workplane:
    return cq.Workplane("XY").circle(radius).extrude(height).translate(
        (center[0], center[1], center[2] - height * 0.5)
    )


def _body_shape() -> cq.Workplane:
    lower = cq.Workplane("XY").box(LOWER_W, LOWER_D, LOWER_H)
    lower = lower.edges("|Z").fillet(0.018)
    lower = lower.translate((0.0, 0.0, LOWER_Z0 + LOWER_H * 0.5))

    bin_cavity = _world_box(
        (BIN_OPENING_W, 0.250, BIN_OPENING_H),
        (
            0.0,
            LOWER_D * 0.5 - 0.250 * 0.5 + 0.012,
            BIN_OPENING_Z0 + BIN_OPENING_H * 0.5,
        ),
    )
    lower = lower.cut(bin_cavity)

    head = cq.Workplane("XY").box(BODY_W, BODY_D, HEAD_H)
    head = head.edges("|Z").fillet(0.020)
    head = head.translate((0.0, 0.0, HEAD_Z0 + HEAD_H * 0.5))

    head_inner = _world_box(
        (BODY_W - 0.032, BODY_D - 0.034, 0.136),
        (0.0, 0.0, 0.488),
    )
    head = head.cut(head_inner)

    feed_slot = _world_box((0.236, 0.010, 0.030), (0.0, -0.004, BODY_TOP_Z - 0.015))
    slider_slot = _world_box((0.058, 0.0065, 0.028), (SLIDER_HOME_X, SLIDER_Y, BODY_TOP_Z - 0.014))
    button_a_hole = _world_cylinder(BUTTON_X[0] * 0.0 + 0.0048, 0.030, (BUTTON_X[0], BUTTON_Y, BODY_TOP_Z - 0.015))
    button_b_hole = _world_cylinder(BUTTON_X[0] * 0.0 + 0.0048, 0.030, (BUTTON_X[1], BUTTON_Y, BODY_TOP_Z - 0.015))
    dial_hole = _world_cylinder(0.0052, 0.032, (DIAL_X, DIAL_Y, BODY_TOP_Z - 0.016))

    body = lower.union(head)
    body = body.cut(feed_slot)
    body = body.cut(slider_slot)
    body = body.cut(button_a_hole)
    body = body.cut(button_b_hole)
    body = body.cut(dial_hole)
    return body


def _head_shape() -> cq.Workplane:
    head = cq.Workplane("XY").box(BODY_W, BODY_D, HEAD_H)
    head = head.edges("|Z").fillet(0.020)
    head = head.translate((0.0, 0.0, HEAD_Z0 + HEAD_H * 0.5))

    head_inner = _world_box(
        (BODY_W - 0.032, BODY_D - 0.034, 0.136),
        (0.0, 0.0, 0.488),
    )
    head = head.cut(head_inner)

    feed_slot = _world_box((0.236, 0.010, 0.030), (0.0, -0.004, BODY_TOP_Z - 0.015))
    slider_slot = _world_box((0.058, 0.0065, 0.028), (SLIDER_HOME_X, SLIDER_Y, BODY_TOP_Z - 0.014))
    button_a_hole = _world_cylinder(0.0048, 0.030, (BUTTON_X[0], BUTTON_Y, BODY_TOP_Z - 0.015))
    button_b_hole = _world_cylinder(0.0048, 0.030, (BUTTON_X[1], BUTTON_Y, BODY_TOP_Z - 0.015))
    dial_hole = _world_cylinder(0.0052, 0.032, (DIAL_X, DIAL_Y, BODY_TOP_Z - 0.016))

    head = head.cut(feed_slot)
    head = head.cut(slider_slot)
    head = head.cut(button_a_hole)
    head = head.cut(button_b_hole)
    head = head.cut(dial_hole)
    return head


def _bin_shape() -> cq.Workplane:
    tub = cq.Workplane("XY").box(0.303, 0.210, 0.302)
    tub = tub.edges("|Z").fillet(0.010)
    tub = tub.translate((0.0, -0.117, -0.015))

    inner = cq.Workplane("XY").box(0.295, 0.202, 0.308).translate((0.0, -0.117, -0.010))
    tub = tub.cut(inner)

    front_panel = cq.Workplane("XY").box(0.314, 0.012, 0.332).translate((0.0, -0.006, 0.0))
    bin_shape = tub.union(front_panel)

    handle_recess = cq.Workplane("XY").box(0.086, 0.008, 0.026).translate((0.0, -0.004, 0.070))
    return bin_shape.cut(handle_recess)


def _drum_shape() -> cq.Workplane:
    teeth = 18
    inner_radius = 0.011
    outer_radius = 0.0135
    profile: list[tuple[float, float]] = []
    for index in range(teeth * 2):
        angle = math.tau * index / (teeth * 2)
        radius = outer_radius if index % 2 == 0 else inner_radius
        profile.append((radius * math.cos(angle), radius * math.sin(angle)))

    drum_body = cq.Workplane("YZ").polyline(profile).close().extrude(0.292).translate((-0.146, 0.0, 0.0))
    shaft = cq.Workplane("YZ").circle(0.0045).extrude(0.323).translate((-0.1615, 0.0, 0.0))
    return shaft.union(drum_body)


def build_object_model() -> ArticulatedObject:
    model = ArticulatedObject(name="micro_cut_office_shredder")

    body_mat = model.material("body_black", rgba=(0.15, 0.16, 0.17, 1.0))
    trim_mat = model.material("trim_dark", rgba=(0.10, 0.11, 0.12, 1.0))
    steel_mat = model.material("steel_dark", rgba=(0.42, 0.44, 0.46, 1.0))
    bin_mat = model.material("bin_black", rgba=(0.13, 0.13, 0.14, 1.0))
    control_mat = model.material("control_grey", rgba=(0.30, 0.31, 0.33, 1.0))
    tire_mat = model.material("tire_black", rgba=(0.08, 0.08, 0.09, 1.0))
    wheel_mat = model.material("wheel_grey", rgba=(0.60, 0.62, 0.64, 1.0))

    head_mesh = mesh_from_cadquery(_head_shape(), "shredder_head")
    bin_mesh = mesh_from_cadquery(_bin_shape(), "shredder_bin")
    drum_mesh = mesh_from_cadquery(_drum_shape(), "shredder_drum")
    dial_mesh = mesh_from_geometry(
        KnobGeometry(
            0.032,
            0.015,
            body_style="skirted",
            top_diameter=0.026,
            skirt=KnobSkirt(0.036, 0.004, flare=0.05),
            indicator=KnobIndicator(style="line", mode="raised", angle_deg=0.0),
            center=False,
        ),
        "jam_clear_dial",
    )
    body = model.part("body")
    body.visual(head_mesh, material=body_mat, name="head_shell")
    body.visual(
        Box((0.009, LOWER_D, LOWER_H)),
        origin=Origin(xyz=(-0.168, 0.0, LOWER_Z0 + LOWER_H * 0.5)),
        material=body_mat,
        name="left_side",
    )
    body.visual(
        Box((0.009, LOWER_D, LOWER_H)),
        origin=Origin(xyz=(0.168, 0.0, LOWER_Z0 + LOWER_H * 0.5)),
        material=body_mat,
        name="right_side",
    )
    body.visual(
        Box((LOWER_W - 0.018, 0.009, LOWER_H)),
        origin=Origin(xyz=(0.0, -0.1295, LOWER_Z0 + LOWER_H * 0.5)),
        material=body_mat,
        name="back_wall",
    )
    body.visual(
        Box((LOWER_W - 0.018, 0.225, 0.010)),
        origin=Origin(xyz=(0.0, -0.005, 0.050)),
        material=body_mat,
        name="bin_floor",
    )
    body.visual(
        Box((LOWER_W - 0.018, 0.092, 0.028)),
        origin=Origin(xyz=(0.0, -0.006, 0.416)),
        material=body_mat,
        name="lower_header",
    )
    body.visual(
        Box((0.005, 0.050, 0.050)),
        origin=Origin(xyz=(-0.164, 0.0, 0.536)),
        material=trim_mat,
        name="left_bearing_wall",
    )
    body.visual(
        Box((0.005, 0.050, 0.050)),
        origin=Origin(xyz=(0.164, 0.0, 0.536)),
        material=trim_mat,
        name="right_bearing_wall",
    )
    body.visual(
        Box((0.323, 0.018, 0.008)),
        origin=Origin(xyz=(0.0, 0.0, 0.513)),
        material=trim_mat,
        name="lower_bridge",
    )

    bin_part = model.part("bin")
    bin_part.visual(bin_mesh, material=bin_mat, name="bin_shell")
    model.articulation(
        "body_to_bin",
        ArticulationType.PRISMATIC,
        parent=body,
        child=bin_part,
        origin=Origin(xyz=(0.0, LOWER_D * 0.5 + 0.003, BIN_OPENING_Z0 + BIN_OPENING_H * 0.5 - 0.006)),
        axis=(0.0, 1.0, 0.0),
        motion_limits=MotionLimits(effort=80.0, velocity=0.35, lower=0.0, upper=BIN_TRAVEL),
    )

    front_drum = model.part("front_drum")
    front_drum.visual(drum_mesh, material=steel_mat, name="drum")
    model.articulation(
        "body_to_front_drum",
        ArticulationType.CONTINUOUS,
        parent=body,
        child=front_drum,
        origin=Origin(xyz=(0.0, 0.0138, 0.536)),
        axis=(1.0, 0.0, 0.0),
        motion_limits=MotionLimits(effort=1.0, velocity=18.0),
    )

    rear_drum = model.part("rear_drum")
    rear_drum.visual(drum_mesh, material=steel_mat, name="drum")
    model.articulation(
        "body_to_rear_drum",
        ArticulationType.CONTINUOUS,
        parent=body,
        child=rear_drum,
        origin=Origin(xyz=(0.0, -0.0138, 0.536)),
        axis=(-1.0, 0.0, 0.0),
        motion_limits=MotionLimits(effort=1.0, velocity=18.0),
    )

    slider = model.part("slider")
    slider.visual(
        Box((0.024, 0.012, 0.007)),
        origin=Origin(xyz=(0.0, 0.0, 0.0035)),
        material=control_mat,
        name="slider_cap",
    )
    slider.visual(
        Box((0.010, 0.004, 0.012)),
        origin=Origin(xyz=(0.0, 0.0, -0.003)),
        material=trim_mat,
        name="slider_stem",
    )
    model.articulation(
        "body_to_slider",
        ArticulationType.PRISMATIC,
        parent=body,
        child=slider,
        origin=Origin(xyz=(SLIDER_HOME_X, SLIDER_Y, BODY_TOP_Z)),
        axis=(1.0, 0.0, 0.0),
        motion_limits=MotionLimits(effort=12.0, velocity=0.12, lower=-0.018, upper=0.018),
    )

    dial = model.part("dial")
    dial.visual(dial_mesh, material=control_mat, name="dial_cap")
    dial.visual(
        Cylinder(radius=0.0045, length=0.010),
        origin=Origin(xyz=(0.0, 0.0, -0.003)),
        material=trim_mat,
        name="dial_shaft",
    )
    model.articulation(
        "body_to_dial",
        ArticulationType.CONTINUOUS,
        parent=body,
        child=dial,
        origin=Origin(xyz=(DIAL_X, DIAL_Y, BODY_TOP_Z)),
        axis=(0.0, 0.0, 1.0),
        motion_limits=MotionLimits(effort=0.3, velocity=6.0),
    )

    for index, x_pos in enumerate(BUTTON_X):
        button = model.part(f"button_{index}")
        button.visual(
            Cylinder(radius=0.0062, length=0.0045),
            origin=Origin(xyz=(0.0, 0.0, 0.00225)),
            material=control_mat,
            name="button_cap",
        )
        button.visual(
            Cylinder(radius=0.0042, length=0.011),
            origin=Origin(xyz=(0.0, 0.0, -0.0045)),
            material=trim_mat,
            name="button_stem",
        )
        model.articulation(
            f"body_to_button_{index}",
            ArticulationType.PRISMATIC,
            parent=body,
            child=button,
            origin=Origin(xyz=(x_pos, BUTTON_Y, BODY_TOP_Z)),
            axis=(0.0, 0.0, -1.0),
            motion_limits=MotionLimits(effort=16.0, velocity=0.08, lower=0.0, upper=0.004),
        )

    caster_mounts = {
        "front_left": (-0.120, 0.086),
        "front_right": (0.120, 0.086),
        "rear_left": (-0.120, -0.086),
        "rear_right": (0.120, -0.086),
    }
    for name, (x_pos, y_pos) in caster_mounts.items():
        fork = model.part(f"{name}_fork")
        fork.visual(
            Cylinder(radius=0.010, length=0.003),
            origin=Origin(xyz=(0.0, 0.0, -0.002)),
            material=trim_mat,
            name="top_plate",
        )
        fork.visual(
            Cylinder(radius=0.004, length=0.012),
            origin=Origin(xyz=(0.0, 0.0, -0.009)),
            material=trim_mat,
            name="stem",
        )
        fork.visual(
            Box((0.018, 0.011, 0.004)),
            origin=Origin(xyz=(0.0, 0.0, -0.0165)),
            material=trim_mat,
            name="yoke_bridge",
        )
        fork.visual(
            Box((0.0025, 0.011, 0.020)),
            origin=Origin(xyz=(-0.007, 0.0, -0.028)),
            material=trim_mat,
            name="left_cheek",
        )
        fork.visual(
            Box((0.0025, 0.011, 0.020)),
            origin=Origin(xyz=(0.007, 0.0, -0.028)),
            material=trim_mat,
            name="right_cheek",
        )
        model.articulation(
            f"body_to_{name}_fork",
            ArticulationType.FIXED,
            parent=body,
            child=fork,
            origin=Origin(xyz=(x_pos, y_pos, LOWER_Z0 + 0.0005)),
        )

        wheel = model.part(f"{name}_wheel")
        wheel.visual(
            Cylinder(radius=0.018, length=0.010),
            origin=Origin(xyz=(0.0, 0.0, 0.0), rpy=(0.0, math.pi * 0.5, 0.0)),
            material=tire_mat,
            name="tire",
        )
        wheel.visual(
            Cylinder(radius=0.0105, length=0.011),
            origin=Origin(xyz=(0.0, 0.0, 0.0), rpy=(0.0, math.pi * 0.5, 0.0)),
            material=wheel_mat,
            name="wheel_core",
        )
        model.articulation(
            f"{name}_fork_to_{name}_wheel",
            ArticulationType.CONTINUOUS,
            parent=fork,
            child=wheel,
            origin=Origin(xyz=(0.0, 0.0, -0.028)),
            axis=(1.0, 0.0, 0.0),
            motion_limits=MotionLimits(effort=0.4, velocity=14.0),
        )

    return model


def run_tests() -> TestReport:
    ctx = TestContext(object_model)

    body = object_model.get_part("body")
    bin_part = object_model.get_part("bin")
    slider = object_model.get_part("slider")
    dial = object_model.get_part("dial")
    front_drum = object_model.get_part("front_drum")
    rear_drum = object_model.get_part("rear_drum")

    bin_joint = object_model.get_articulation("body_to_bin")
    slider_joint = object_model.get_articulation("body_to_slider")
    dial_joint = object_model.get_articulation("body_to_dial")

    ctx.expect_overlap(
        bin_part,
        body,
        axes="xz",
        min_overlap=0.28,
        name="bin stays aligned with the shredder body footprint",
    )

    with ctx.pose({bin_joint: 0.0}):
        ctx.expect_overlap(
            bin_part,
            body,
            axes="y",
            min_overlap=0.20,
            name="closed bin sits deeply inside the cabinet",
        )
        closed_bin_pos = ctx.part_world_position(bin_part)

    bin_limits = bin_joint.motion_limits
    open_bin_pos = None
    if bin_limits is not None and bin_limits.upper is not None:
        with ctx.pose({bin_joint: bin_limits.upper}):
            ctx.expect_overlap(
                bin_part,
                body,
                axes="y",
                min_overlap=0.055,
                name="pulled bin still retains insertion in the cabinet",
            )
            open_bin_pos = ctx.part_world_position(bin_part)

    ctx.check(
        "bin pulls forward from the lower front",
        closed_bin_pos is not None and open_bin_pos is not None and open_bin_pos[1] > closed_bin_pos[1] + 0.12,
        details=f"closed={closed_bin_pos}, open={open_bin_pos}",
    )

    slider_limits = slider_joint.motion_limits
    slider_min_pos = None
    slider_max_pos = None
    if slider_limits is not None and slider_limits.lower is not None and slider_limits.upper is not None:
        with ctx.pose({slider_joint: slider_limits.lower}):
            slider_min_pos = ctx.part_world_position(slider)
        with ctx.pose({slider_joint: slider_limits.upper}):
            slider_max_pos = ctx.part_world_position(slider)

    ctx.check(
        "slider travels across the top guide slot",
        slider_min_pos is not None and slider_max_pos is not None and slider_max_pos[0] > slider_min_pos[0] + 0.03,
        details=f"min={slider_min_pos}, max={slider_max_pos}",
    )

    for index in range(2):
        button = object_model.get_part(f"button_{index}")
        button_joint = object_model.get_articulation(f"body_to_button_{index}")
        rest_pos = ctx.part_world_position(button)
        pressed_pos = None
        limits = button_joint.motion_limits
        if limits is not None and limits.upper is not None:
            with ctx.pose({button_joint: limits.upper}):
                pressed_pos = ctx.part_world_position(button)
        ctx.check(
            f"button_{index} depresses downward",
            rest_pos is not None and pressed_pos is not None and pressed_pos[2] < rest_pos[2] - 0.0025,
            details=f"rest={rest_pos}, pressed={pressed_pos}",
        )

    dial_rest = ctx.part_world_position(dial)
    with ctx.pose({dial_joint: 1.2}):
        dial_turn = ctx.part_world_position(dial)
    ctx.check(
        "dial rotates in place on the top deck",
        dial_rest is not None
        and dial_turn is not None
        and abs(dial_rest[0] - dial_turn[0]) < 1e-6
        and abs(dial_rest[1] - dial_turn[1]) < 1e-6
        and abs(dial_rest[2] - dial_turn[2]) < 1e-6,
        details=f"rest={dial_rest}, turned={dial_turn}",
    )

    front_joint = object_model.get_articulation("body_to_front_drum")
    rear_joint = object_model.get_articulation("body_to_rear_drum")
    ctx.check(
        "cutter drums use horizontal continuous shafts",
        front_joint.axis == (1.0, 0.0, 0.0)
        and rear_joint.axis == (-1.0, 0.0, 0.0)
        and front_joint.motion_limits is not None
        and rear_joint.motion_limits is not None
        and front_joint.motion_limits.lower is None
        and rear_joint.motion_limits.lower is None,
        details=f"front_axis={front_joint.axis}, rear_axis={rear_joint.axis}",
    )

    front_drum_pos = ctx.part_world_position(front_drum)
    rear_drum_pos = ctx.part_world_position(rear_drum)
    ctx.check(
        "cutter drums sit as a front rear pair below the feed slot",
        front_drum_pos is not None
        and rear_drum_pos is not None
        and front_drum_pos[2] > 0.50
        and rear_drum_pos[2] > 0.50
        and front_drum_pos[1] > rear_drum_pos[1] + 0.02,
        details=f"front={front_drum_pos}, rear={rear_drum_pos}",
    )

    wheel_joint_names = (
        "front_left_fork_to_front_left_wheel",
        "front_right_fork_to_front_right_wheel",
        "rear_left_fork_to_rear_left_wheel",
        "rear_right_fork_to_rear_right_wheel",
    )
    wheel_joint_ok = True
    wheel_details: list[str] = []
    for joint_name in wheel_joint_names:
        joint = object_model.get_articulation(joint_name)
        limits = joint.motion_limits
        joint_ok = joint.axis == (1.0, 0.0, 0.0) and limits is not None and limits.lower is None and limits.upper is None
        wheel_joint_ok = wheel_joint_ok and joint_ok
        wheel_details.append(f"{joint_name}:{joint.axis}:{None if limits is None else (limits.lower, limits.upper)}")
    ctx.check(
        "caster wheels use continuous axle rotation",
        wheel_joint_ok,
        details="; ".join(wheel_details),
    )

    return ctx.report()


object_model = build_object_model()
