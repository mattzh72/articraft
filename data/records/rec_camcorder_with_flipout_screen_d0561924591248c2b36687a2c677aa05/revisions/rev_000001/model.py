from __future__ import annotations

import math

import cadquery as cq

from sdk import (
    ArticulatedObject,
    ArticulationType,
    Box,
    Cylinder,
    LatheGeometry,
    MotionLimits,
    Origin,
    TestContext,
    TestReport,
    mesh_from_cadquery,
    mesh_from_geometry,
    rounded_rect_profile,
    section_loft,
)


def _yz_section(
    width: float,
    height: float,
    radius: float,
    x_pos: float,
    *,
    z_shift: float = 0.0,
) -> list[tuple[float, float, float]]:
    return [(x_pos, y, z + z_shift) for z, y in rounded_rect_profile(height, width, radius)]


def _build_body_shell():
    return section_loft(
        [
            _yz_section(0.094, 0.112, 0.016, -0.105, z_shift=0.000),
            _yz_section(0.098, 0.118, 0.018, -0.050, z_shift=0.004),
            _yz_section(0.094, 0.116, 0.018, 0.010, z_shift=0.002),
            _yz_section(0.086, 0.108, 0.016, 0.060, z_shift=-0.003),
            _yz_section(0.064, 0.088, 0.013, 0.102, z_shift=-0.010),
        ]
    )


def _build_lens_hood():
    outer = (
        cq.Workplane("XY")
        .rect(0.074, 0.062)
        .workplane(offset=0.055)
        .rect(0.118, 0.094)
        .loft(combine=True)
    )
    inner = (
        cq.Workplane("XY")
        .workplane(offset=0.003)
        .rect(0.060, 0.050)
        .workplane(offset=0.054)
        .rect(0.104, 0.080)
        .loft(combine=True)
    )
    return outer.cut(inner)


def _build_control_ring_shell():
    outer_profile = [
        (0.044, -0.013),
        (0.046, -0.009),
        (0.046, -0.003),
        (0.047, 0.000),
        (0.046, 0.003),
        (0.046, 0.009),
        (0.044, 0.013),
    ]
    inner_profile = [
        (0.040, -0.013),
        (0.040, 0.013),
    ]
    return LatheGeometry.from_shell_profiles(
        outer_profile,
        inner_profile,
        segments=64,
        start_cap="flat",
        end_cap="flat",
    )


def build_object_model() -> ArticulatedObject:
    model = ArticulatedObject(name="documentary_camcorder")

    body_finish = model.material("body_finish", rgba=(0.10, 0.10, 0.11, 1.0))
    trim_finish = model.material("trim_finish", rgba=(0.18, 0.18, 0.19, 1.0))
    rubber_finish = model.material("rubber_finish", rgba=(0.08, 0.08, 0.08, 1.0))
    hinge_finish = model.material("hinge_finish", rgba=(0.26, 0.27, 0.29, 1.0))
    glass_finish = model.material("glass_finish", rgba=(0.18, 0.24, 0.30, 0.92))

    body = model.part("body")
    body.visual(
        mesh_from_geometry(_build_body_shell(), "camcorder_body_shell"),
        material=body_finish,
        name="body_shell",
    )
    body.visual(
        Box((0.052, 0.050, 0.018)),
        origin=Origin(xyz=(-0.010, 0.000, 0.051)),
        material=trim_finish,
        name="top_block",
    )
    body.visual(
        Box((0.060, 0.008, 0.048)),
        origin=Origin(xyz=(0.006, 0.048, 0.004)),
        material=trim_finish,
        name="side_pad",
    )
    body.visual(
        Box((0.020, 0.054, 0.060)),
        origin=Origin(xyz=(0.103, 0.000, -0.006)),
        material=trim_finish,
        name="lens_mount",
    )
    body.visual(
        Box((0.110, 0.028, 0.052)),
        origin=Origin(xyz=(0.030, -0.055, -0.010)),
        material=rubber_finish,
        name="hand_grip",
    )
    body.visual(
        Box((0.040, 0.074, 0.086)),
        origin=Origin(xyz=(-0.120, 0.000, 0.002)),
        material=trim_finish,
        name="battery_pack",
    )
    body.visual(
        Cylinder(radius=0.013, length=0.030),
        origin=Origin(xyz=(-0.122, 0.000, 0.040), rpy=(0.0, math.pi / 2.0, 0.0)),
        material=trim_finish,
        name="eyecup",
    )

    lens = model.part("lens")
    lens.visual(
        Cylinder(radius=0.044, length=0.018),
        origin=Origin(xyz=(0.009, 0.000, 0.000), rpy=(0.0, math.pi / 2.0, 0.0)),
        material=trim_finish,
        name="mount_collar",
    )
    lens.visual(
        Cylinder(radius=0.034, length=0.052),
        origin=Origin(xyz=(0.033, 0.000, 0.000), rpy=(0.0, math.pi / 2.0, 0.0)),
        material=trim_finish,
        name="rear_barrel",
    )
    lens.visual(
        Cylinder(radius=0.043, length=0.006),
        origin=Origin(xyz=(0.041, 0.000, 0.000), rpy=(0.0, math.pi / 2.0, 0.0)),
        material=trim_finish,
        name="ring_shoulder",
    )
    lens.visual(
        Cylinder(radius=0.038, length=0.050),
        origin=Origin(xyz=(0.084, 0.000, 0.000), rpy=(0.0, math.pi / 2.0, 0.0)),
        material=trim_finish,
        name="front_barrel",
    )
    lens.visual(
        mesh_from_cadquery(_build_lens_hood(), "camcorder_lens_hood"),
        origin=Origin(xyz=(0.109, 0.000, 0.000), rpy=(0.0, math.pi / 2.0, 0.0)),
        material=body_finish,
        name="lens_hood",
    )
    lens.visual(
        Cylinder(radius=0.028, length=0.060),
        origin=Origin(xyz=(0.137, 0.000, 0.000), rpy=(0.0, math.pi / 2.0, 0.0)),
        material=body_finish,
        name="lens_core",
    )
    lens.visual(
        Cylinder(radius=0.031, length=0.003),
        origin=Origin(xyz=(0.144, 0.000, 0.000), rpy=(0.0, math.pi / 2.0, 0.0)),
        material=glass_finish,
        name="front_glass",
    )

    control_ring = model.part("control_ring")
    control_ring.visual(
        mesh_from_geometry(_build_control_ring_shell(), "camcorder_control_ring"),
        origin=Origin(rpy=(0.0, math.pi / 2.0, 0.0)),
        material=rubber_finish,
        name="ring_shell",
    )

    top_handle = model.part("top_handle")
    top_handle.visual(
        Box((0.024, 0.022, 0.030)),
        origin=Origin(xyz=(-0.047, 0.000, 0.015)),
        material=trim_finish,
        name="rear_foot",
    )
    top_handle.visual(
        Box((0.024, 0.022, 0.034)),
        origin=Origin(xyz=(0.048, 0.000, 0.017)),
        material=trim_finish,
        name="front_foot",
    )
    top_handle.visual(
        Box((0.116, 0.022, 0.014)),
        origin=Origin(xyz=(0.002, 0.000, 0.033)),
        material=trim_finish,
        name="underbar",
    )
    top_handle.visual(
        Box((0.138, 0.030, 0.020)),
        origin=Origin(xyz=(0.002, 0.000, 0.046)),
        material=trim_finish,
        name="handle_bridge",
    )
    top_handle.visual(
        Box((0.028, 0.026, 0.018)),
        origin=Origin(xyz=(0.056, 0.000, 0.052)),
        material=trim_finish,
        name="mic_block",
    )

    side_arm = model.part("side_arm")
    side_arm.visual(
        Cylinder(radius=0.007, length=0.050),
        material=hinge_finish,
        name="root_barrel",
    )
    side_arm.visual(
        Box((0.018, 0.046, 0.014)),
        origin=Origin(xyz=(0.000, 0.021, 0.000)),
        material=trim_finish,
        name="arm_beam",
    )
    side_arm.visual(
        Box((0.012, 0.022, 0.038)),
        origin=Origin(xyz=(0.000, 0.023, 0.000)),
        material=hinge_finish,
        name="arm_rib",
    )
    side_arm.visual(
        Box((0.022, 0.020, 0.048)),
        origin=Origin(xyz=(0.000, 0.040, 0.000)),
        material=trim_finish,
        name="screen_block",
    )

    screen = model.part("screen")
    screen.visual(
        Cylinder(radius=0.004, length=0.068),
        material=hinge_finish,
        name="screen_hinge",
    )
    screen.visual(
        Box((0.096, 0.011, 0.068)),
        origin=Origin(xyz=(0.046, 0.000, 0.000)),
        material=trim_finish,
        name="screen_shell",
    )
    screen.visual(
        Box((0.090, 0.003, 0.062)),
        origin=Origin(xyz=(0.046, -0.004, 0.000)),
        material=body_finish,
        name="screen_back",
    )
    screen.visual(
        Box((0.078, 0.002, 0.050)),
        origin=Origin(xyz=(0.047, 0.005, 0.000)),
        material=glass_finish,
        name="display_panel",
    )

    model.articulation(
        "body_to_lens",
        ArticulationType.FIXED,
        parent=body,
        child=lens,
        origin=Origin(xyz=(0.113, 0.000, -0.004)),
    )
    model.articulation(
        "lens_to_control_ring",
        ArticulationType.CONTINUOUS,
        parent=lens,
        child=control_ring,
        origin=Origin(xyz=(0.057, 0.000, 0.000)),
        axis=(1.0, 0.0, 0.0),
        motion_limits=MotionLimits(effort=2.0, velocity=8.0),
    )
    model.articulation(
        "body_to_top_handle",
        ArticulationType.FIXED,
        parent=body,
        child=top_handle,
        origin=Origin(xyz=(0.000, 0.000, 0.060)),
    )
    model.articulation(
        "body_to_side_arm",
        ArticulationType.REVOLUTE,
        parent=body,
        child=side_arm,
        origin=Origin(xyz=(0.010, 0.059, 0.006)),
        axis=(0.0, 0.0, -1.0),
        motion_limits=MotionLimits(
            effort=2.0,
            velocity=2.5,
            lower=-0.35,
            upper=1.10,
        ),
    )
    model.articulation(
        "arm_to_screen",
        ArticulationType.REVOLUTE,
        parent=side_arm,
        child=screen,
        origin=Origin(xyz=(0.000, 0.054, 0.000)),
        axis=(0.0, 0.0, 1.0),
        motion_limits=MotionLimits(
            effort=1.5,
            velocity=3.0,
            lower=0.0,
            upper=math.radians(175.0),
        ),
    )

    return model


def run_tests() -> TestReport:
    ctx = TestContext(object_model)
    body = object_model.get_part("body")
    lens = object_model.get_part("lens")
    control_ring = object_model.get_part("control_ring")
    screen = object_model.get_part("screen")
    side_arm = object_model.get_part("side_arm")
    handle = object_model.get_part("top_handle")

    arm_hinge = object_model.get_articulation("body_to_side_arm")
    screen_hinge = object_model.get_articulation("arm_to_screen")
    ring_spin = object_model.get_articulation("lens_to_control_ring")

    def _span(aabb, axis_index: int) -> float | None:
        if aabb is None:
            return None
        return aabb[1][axis_index] - aabb[0][axis_index]

    ctx.expect_contact(
        handle,
        body,
        elem_a="rear_foot",
        elem_b="top_block",
        name="top handle is mounted to the camera roof",
    )
    ctx.expect_origin_distance(
        control_ring,
        lens,
        axes="yz",
        max_dist=0.001,
        name="control ring stays concentric with lens barrel",
    )
    ctx.expect_overlap(
        control_ring,
        lens,
        axes="x",
        elem_a="ring_shell",
        elem_b="front_barrel",
        min_overlap=0.010,
        name="control ring covers the lens barrel",
    )
    ctx.expect_gap(
        screen,
        body,
        axis="y",
        positive_elem="screen_shell",
        negative_elem="body_shell",
        min_gap=0.040,
        max_gap=0.080,
        name="screen stands off from the body on the hinge arm",
    )
    ctx.expect_contact(
        screen,
        side_arm,
        elem_a="screen_hinge",
        elem_b="screen_block",
        name="screen hinge sits on the side arm block",
    )

    rest_screen_pos = ctx.part_world_position(screen)
    with ctx.pose({arm_hinge: 0.90}):
        swung_screen_pos = ctx.part_world_position(screen)
    ctx.check(
        "side arm swings the screen assembly forward",
        rest_screen_pos is not None
        and swung_screen_pos is not None
        and swung_screen_pos[0] > rest_screen_pos[0] + 0.020,
        details=f"rest={rest_screen_pos}, swung={swung_screen_pos}",
    )

    closed_aabb = ctx.part_element_world_aabb(screen, elem="screen_shell")
    with ctx.pose({screen_hinge: math.pi / 2.0}):
        opened_aabb = ctx.part_element_world_aabb(screen, elem="screen_shell")
    closed_x_span = _span(closed_aabb, 0)
    closed_y_span = _span(closed_aabb, 1)
    opened_x_span = _span(opened_aabb, 0)
    opened_y_span = _span(opened_aabb, 1)
    ctx.check(
        "viewing screen rotates outward on its vertical hinge",
        closed_x_span is not None
        and closed_y_span is not None
        and opened_x_span is not None
        and opened_y_span is not None
        and opened_y_span > closed_y_span + 0.050
        and opened_x_span < closed_x_span - 0.050,
        details=(
            f"closed_spans={(closed_x_span, closed_y_span)}, "
            f"opened_spans={(opened_x_span, opened_y_span)}"
        ),
    )

    ring_rest_aabb = ctx.part_world_aabb(control_ring)
    with ctx.pose({ring_spin: math.pi / 2.0}):
        ring_spun_aabb = ctx.part_world_aabb(control_ring)
    ring_rest_x = _span(ring_rest_aabb, 0)
    ring_rest_y = _span(ring_rest_aabb, 1)
    ring_spun_x = _span(ring_spun_aabb, 0)
    ring_spun_y = _span(ring_spun_aabb, 1)
    ctx.check(
        "control ring spins about the lens axis without changing its stance",
        ring_rest_x is not None
        and ring_rest_y is not None
        and ring_spun_x is not None
        and ring_spun_y is not None
        and abs(ring_rest_x - ring_spun_x) < 0.002
        and abs(ring_rest_y - ring_spun_y) < 0.002,
        details=(
            f"rest_spans={(ring_rest_x, ring_rest_y)}, "
            f"spun_spans={(ring_spun_x, ring_spun_y)}"
        ),
    )

    return ctx.report()


object_model = build_object_model()
