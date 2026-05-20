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


def _base_shell_shape() -> cq.Workplane:
    heel = (
        cq.Workplane("XY")
        .box(0.37, 0.25, 0.03, centered=(True, True, False))
        .edges("|Z")
        .fillet(0.022)
    )
    deck = (
        cq.Workplane("XY")
        .box(0.24, 0.18, 0.018, centered=(True, True, False))
        .edges("|Z")
        .fillet(0.012)
        .translate((0.055, 0.0, 0.03))
    )
    rear_column = (
        cq.Workplane("XY")
        .box(0.11, 0.118, 0.192, centered=(True, True, False))
        .edges("|Z")
        .fillet(0.018)
        .translate((-0.095, 0.0, 0.03))
    )
    shoulder = (
        cq.Workplane("XZ")
        .polyline(
            [
                (-0.135, 0.105),
                (-0.11, 0.145),
                (-0.075, 0.228),
                (-0.025, 0.246),
                (0.005, 0.236),
                (-0.01, 0.175),
                (-0.082, 0.108),
            ]
        )
        .close()
        .extrude(0.11, both=True)
    )
    left_cheek = (
        cq.Workplane("XY")
        .box(0.03, 0.028, 0.058, centered=(True, True, False))
        .edges("|Z")
        .fillet(0.006)
        .translate((-0.048, -0.043, 0.225))
    )
    right_cheek = (
        cq.Workplane("XY")
        .box(0.03, 0.028, 0.058, centered=(True, True, False))
        .edges("|Z")
        .fillet(0.006)
        .translate((-0.048, 0.043, 0.225))
    )
    return heel.union(deck).union(rear_column).union(shoulder).union(left_cheek).union(right_cheek)


def _seat_ring_shape() -> cq.Workplane:
    ring = (
        cq.Workplane("XY")
        .circle(0.084)
        .circle(0.055)
        .extrude(0.01)
        .translate((0.13, 0.0, 0.048))
    )
    core = cq.Workplane("XY").circle(0.058).extrude(0.008).translate((0.13, 0.0, 0.04))
    seat = ring.union(core)

    for angle_deg in (-18.0, 102.0, 222.0):
        angle = math.radians(angle_deg)
        lug = (
            cq.Workplane("XY")
            .box(0.02, 0.014, 0.004, centered=(True, True, False))
            .translate((0.13 + 0.067 * math.cos(angle), 0.067 * math.sin(angle), 0.054))
            .rotate((0.13, 0.0, 0.054), (0.13, 0.0, 1.054), angle_deg + 90.0)
        )
        seat = seat.union(lug)

    return seat


def _head_shell_shape() -> cq.Workplane:
    body = (
        cq.Workplane("XZ")
        .polyline(
            [
                (0.03, -0.015),
                (0.04, 0.018),
                (0.06, 0.05),
                (0.085, 0.066),
                (0.185, 0.071),
                (0.27, 0.063),
                (0.312, 0.045),
                (0.326, 0.018),
                (0.321, -0.006),
                (0.295, -0.027),
                (0.235, -0.038),
                (0.14, -0.044),
                (0.075, -0.04),
                (0.042, -0.03),
            ]
        )
        .close()
        .extrude(0.15, both=True)
        .edges("|Y")
        .fillet(0.018)
    )
    hinge_barrel = cq.Workplane("XZ").circle(0.021).extrude(0.029, both=True)
    rear_bridge = (
        cq.Workplane("XY")
        .box(0.022, 0.052, 0.038, centered=(True, True, True))
        .translate((0.021, 0.0, 0.006))
    )
    drive_boss = cq.Workplane("XY").circle(0.021).extrude(0.016).translate((0.19, 0.0, -0.055))
    slot_mount = (
        cq.Workplane("XY")
        .box(0.05, 0.04, 0.014, centered=(True, True, True))
        .translate((0.09, 0.045, 0.034))
    )
    return body.union(hinge_barrel).union(rear_bridge).union(drive_boss).union(slot_mount)


def _bowl_shell_shape() -> cq.Workplane:
    bowl = (
        cq.Workplane("XY")
        .circle(0.07)
        .workplane(offset=0.035)
        .circle(0.092)
        .workplane(offset=0.06)
        .circle(0.108)
        .workplane(offset=0.07)
        .circle(0.112)
        .loft(combine=True)
        .faces(">Z")
        .shell(-0.006)
    )

    for angle_deg in (20.0, 140.0, 260.0):
        angle = math.radians(angle_deg)
        tab = (
            cq.Workplane("XY")
            .box(0.022, 0.013, 0.008, centered=(True, True, False))
            .translate((0.076 * math.cos(angle), 0.076 * math.sin(angle), 0.015))
            .rotate((0.0, 0.0, 0.012), (0.0, 0.0, 1.012), angle_deg + 90.0)
        )
        bowl = bowl.union(tab)

    return bowl


def _bowl_handle_shape() -> cq.Workplane:
    outer = (
        cq.Workplane("YZ")
        .polyline(
            [
                (0.082, 0.084),
                (0.14, 0.084),
                (0.152, 0.102),
                (0.152, 0.13),
                (0.135, 0.149),
                (0.088, 0.149),
                (0.074, 0.128),
                (0.074, 0.101),
            ]
        )
        .close()
        .extrude(0.022, both=True)
    )
    inner = (
        cq.Workplane("YZ")
        .polyline(
            [
                (0.094, 0.095),
                (0.128, 0.096),
                (0.136, 0.108),
                (0.136, 0.122),
                (0.126, 0.136),
                (0.096, 0.136),
                (0.088, 0.123),
                (0.088, 0.108),
            ]
        )
        .close()
        .extrude(0.026, both=True)
    )
    return outer.cut(inner)


def _paddle_blade_shape() -> cq.Workplane:
    blade = (
        cq.Workplane("XZ")
        .polyline(
            [
                (-0.013, -0.014),
                (-0.029, -0.032),
                (-0.04, -0.068),
                (-0.04, -0.113),
                (-0.024, -0.143),
                (0.0, -0.156),
                (0.024, -0.143),
                (0.04, -0.113),
                (0.04, -0.068),
                (0.029, -0.032),
                (0.013, -0.014),
            ]
        )
        .close()
        .extrude(0.01, both=True)
    )
    window = (
        cq.Workplane("XZ")
        .polyline(
            [
                (-0.016, -0.048),
                (-0.018, -0.109),
                (0.0, -0.139),
                (0.018, -0.109),
                (0.016, -0.048),
                (0.006, -0.036),
                (-0.006, -0.036),
            ]
        )
        .close()
        .extrude(0.016, both=True)
    )
    return blade.cut(window)


def build_object_model() -> ArticulatedObject:
    model = ArticulatedObject(name="stand_mixer")

    body_paint = model.material("body_paint", rgba=(0.92, 0.88, 0.80, 1.0))
    steel = model.material("steel", rgba=(0.82, 0.84, 0.86, 1.0))
    trim = model.material("trim", rgba=(0.12, 0.12, 0.13, 1.0))
    attachment = model.material("attachment", rgba=(0.72, 0.74, 0.76, 1.0))

    base = model.part("base")
    base.visual(
        mesh_from_cadquery(_base_shell_shape(), "mixer_base_shell"),
        material=body_paint,
        name="base_shell",
    )
    base.visual(
        mesh_from_cadquery(_seat_ring_shape(), "mixer_seat_ring"),
        material=trim,
        name="seat_ring",
    )

    bowl = model.part("bowl")
    bowl.visual(
        mesh_from_cadquery(_bowl_shell_shape(), "mixer_bowl_shell"),
        material=steel,
        name="bowl_shell",
    )
    bowl.visual(
        mesh_from_cadquery(_bowl_handle_shape(), "mixer_bowl_handle"),
        material=steel,
        name="bowl_handle",
    )

    head = model.part("head")
    head.visual(
        mesh_from_cadquery(_head_shell_shape(), "mixer_head_shell"),
        material=body_paint,
        name="head_shell",
    )

    lever = model.part("speed_lever")
    lever.visual(
        Box((0.05, 0.07, 0.04)),
        origin=Origin(xyz=(0.0, -0.035, 0.0)),
        material=trim,
        name="lever_guide",
    )
    lever.visual(
        Box((0.018, 0.016, 0.014)),
        origin=Origin(xyz=(0.006, 0.007, 0.0)),
        material=trim,
        name="lever_knob",
    )

    paddle = model.part("paddle")
    paddle.visual(
        Cylinder(radius=0.007, length=0.058),
        origin=Origin(xyz=(0.0, 0.0, -0.029)),
        material=attachment,
        name="shaft",
    )
    paddle.visual(
        Cylinder(radius=0.0115, length=0.014),
        origin=Origin(xyz=(0.0, 0.0, -0.009)),
        material=attachment,
        name="hub_collar",
    )
    paddle.visual(
        mesh_from_cadquery(_paddle_blade_shape(), "mixer_paddle_blade"),
        material=attachment,
        name="blade",
    )

    bowl_mount = model.articulation(
        "base_to_bowl",
        ArticulationType.REVOLUTE,
        parent=base,
        child=bowl,
        origin=Origin(xyz=(0.13, 0.0, 0.058)),
        axis=(0.0, 0.0, 1.0),
        motion_limits=MotionLimits(
            effort=8.0,
            velocity=1.2,
            lower=-0.35,
            upper=0.35,
        ),
    )

    head_hinge = model.articulation(
        "base_to_head",
        ArticulationType.REVOLUTE,
        parent=base,
        child=head,
        origin=Origin(xyz=(-0.048, 0.0, 0.283)),
        axis=(0.0, -1.0, 0.0),
        motion_limits=MotionLimits(
            effort=20.0,
            velocity=1.0,
            lower=0.0,
            upper=math.radians(62.0),
        ),
    )

    speed_slot = model.articulation(
        "head_to_speed_lever",
        ArticulationType.PRISMATIC,
        parent=head,
        child=lever,
        origin=Origin(xyz=(0.09, 0.06, 0.034)),
        axis=(1.0, 0.0, 0.0),
        motion_limits=MotionLimits(
            effort=2.0,
            velocity=0.08,
            lower=-0.012,
            upper=0.012,
        ),
    )

    paddle_spin = model.articulation(
        "head_to_paddle",
        ArticulationType.CONTINUOUS,
        parent=head,
        child=paddle,
        origin=Origin(xyz=(0.19, 0.0, -0.052)),
        axis=(0.0, 0.0, 1.0),
        motion_limits=MotionLimits(effort=8.0, velocity=18.0),
    )

    model.meta["key_articulations"] = (
        bowl_mount.name,
        head_hinge.name,
        speed_slot.name,
        paddle_spin.name,
    )
    return model


def _aabb_center(aabb):
    if aabb is None:
        return None
    lower, upper = aabb
    return tuple((lower[i] + upper[i]) * 0.5 for i in range(3))


def run_tests() -> TestReport:
    ctx = TestContext(object_model)

    base = object_model.get_part("base")
    bowl = object_model.get_part("bowl")
    head = object_model.get_part("head")
    lever = object_model.get_part("speed_lever")
    paddle = object_model.get_part("paddle")

    bowl_mount = object_model.get_articulation("base_to_bowl")
    head_hinge = object_model.get_articulation("base_to_head")
    speed_slot = object_model.get_articulation("head_to_speed_lever")

    ctx.allow_overlap(
        base,
        head,
        elem_a="base_shell",
        elem_b="head_shell",
        reason="The rear tilt hinge is represented with a simplified solid barrel nesting into the base cheek supports.",
    )
    ctx.allow_overlap(
        head,
        lever,
        elem_a="head_shell",
        elem_b="lever_guide",
        reason="The side speed lever uses a hidden guide tongue simplified as sliding within the short slot track on the head shell.",
    )

    ctx.expect_gap(
        bowl,
        base,
        axis="z",
        positive_elem="bowl_shell",
        negative_elem="seat_ring",
        max_gap=0.004,
        max_penetration=0.0,
        name="bowl seats on the bayonet ring",
    )

    with ctx.pose({head_hinge: 0.0}):
        ctx.expect_within(
            paddle,
            bowl,
            axes="xy",
            inner_elem="blade",
            outer_elem="bowl_shell",
            margin=0.018,
            name="paddle footprint stays inside the bowl opening",
        )
        ctx.expect_gap(
            head,
            bowl,
            axis="z",
            positive_elem="head_shell",
            negative_elem="bowl_shell",
            min_gap=0.002,
            max_gap=0.05,
            name="closed head shell clears the bowl rim",
        )

    lever_lower = speed_slot.motion_limits.lower if speed_slot.motion_limits is not None else None
    lever_upper = speed_slot.motion_limits.upper if speed_slot.motion_limits is not None else None
    lever_low_pos = None
    lever_high_pos = None
    if lever_lower is not None and lever_upper is not None:
        with ctx.pose({speed_slot: lever_lower}):
            lever_low_pos = ctx.part_world_position(lever)
        with ctx.pose({speed_slot: lever_upper}):
            lever_high_pos = ctx.part_world_position(lever)

    ctx.check(
        "speed lever slides along the side slot",
        lever_low_pos is not None
        and lever_high_pos is not None
        and lever_high_pos[0] > lever_low_pos[0] + 0.018
        and abs(lever_high_pos[1] - lever_low_pos[1]) < 0.003
        and abs(lever_high_pos[2] - lever_low_pos[2]) < 0.003,
        details=f"low={lever_low_pos}, high={lever_high_pos}",
    )

    hinge_upper = head_hinge.motion_limits.upper if head_hinge.motion_limits is not None else None
    paddle_rest = ctx.part_world_position(paddle)
    paddle_open = None
    if hinge_upper is not None:
        with ctx.pose({head_hinge: hinge_upper}):
            paddle_open = ctx.part_world_position(paddle)

    ctx.check(
        "tilt head lifts the paddle upward",
        paddle_rest is not None and paddle_open is not None and paddle_open[2] > paddle_rest[2] + 0.09,
        details=f"rest={paddle_rest}, open={paddle_open}",
    )

    bowl_lower = bowl_mount.motion_limits.lower if bowl_mount.motion_limits is not None else None
    bowl_upper = bowl_mount.motion_limits.upper if bowl_mount.motion_limits is not None else None
    handle_low = None
    handle_high = None
    if bowl_lower is not None and bowl_upper is not None:
        with ctx.pose({bowl_mount: bowl_lower}):
            handle_low = _aabb_center(ctx.part_element_world_aabb(bowl, elem="bowl_handle"))
        with ctx.pose({bowl_mount: bowl_upper}):
            handle_high = _aabb_center(ctx.part_element_world_aabb(bowl, elem="bowl_handle"))

    ctx.check(
        "bowl twists on the bayonet seat",
        handle_low is not None
        and handle_high is not None
        and abs(handle_high[0] - handle_low[0]) > 0.03,
        details=f"low={handle_low}, high={handle_high}",
    )

    return ctx.report()


object_model = build_object_model()
