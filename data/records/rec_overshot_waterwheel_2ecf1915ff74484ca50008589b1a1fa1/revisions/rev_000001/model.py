from __future__ import annotations

# User code should import every SDK/stdlib symbol it uses instead of relying on
# hidden scaffold imports.
# >>> USER_CODE_START
import math

from sdk import (
    ArticulatedObject,
    ArticulationType,
    Box,
    BoxGeometry,
    Cylinder,
    CylinderGeometry,
    Inertial,
    MotionLimits,
    Origin,
    TestContext,
    TestReport,
    TorusGeometry,
    mesh_from_geometry,
)


def _midpoint(
    a: tuple[float, float, float], b: tuple[float, float, float]
) -> tuple[float, float, float]:
    return ((a[0] + b[0]) * 0.5, (a[1] + b[1]) * 0.5, (a[2] + b[2]) * 0.5)


def _distance(
    a: tuple[float, float, float], b: tuple[float, float, float]
) -> float:
    return math.sqrt((b[0] - a[0]) ** 2 + (b[1] - a[1]) ** 2 + (b[2] - a[2]) ** 2)


def _rpy_for_segment(
    a: tuple[float, float, float], b: tuple[float, float, float]
) -> tuple[float, float, float]:
    dx = b[0] - a[0]
    dy = b[1] - a[1]
    dz = b[2] - a[2]
    length_xy = math.hypot(dx, dy)
    yaw = math.atan2(dy, dx)
    pitch = math.atan2(length_xy, dz)
    return (0.0, pitch, yaw)


def _add_box_beam(
    part,
    a: tuple[float, float, float],
    b: tuple[float, float, float],
    *,
    cross_x: float,
    cross_y: float,
    material,
    name: str | None = None,
) -> None:
    part.visual(
        Box((cross_x, cross_y, _distance(a, b))),
        origin=Origin(xyz=_midpoint(a, b), rpy=_rpy_for_segment(a, b)),
        material=material,
        name=name,
    )


def _build_waterwheel_geometry() -> object:
    wheel_radius = 1.05
    wheel_width = 0.34
    ring_offset = 0.12
    geom = CylinderGeometry(radius=0.05, height=0.84, radial_segments=28).rotate_x(
        math.pi / 2.0
    )
    geom.merge(
        CylinderGeometry(radius=0.16, height=0.30, radial_segments=28).rotate_x(
            math.pi / 2.0
        )
    )
    geom.merge(
        CylinderGeometry(radius=0.22, height=0.035, radial_segments=28)
        .rotate_x(math.pi / 2.0)
        .translate(0.0, ring_offset, 0.0)
    )
    geom.merge(
        CylinderGeometry(radius=0.22, height=0.035, radial_segments=28)
        .rotate_x(math.pi / 2.0)
        .translate(0.0, -ring_offset, 0.0)
    )
    geom.merge(TorusGeometry(radius=wheel_radius, tube=0.045).rotate_x(math.pi / 2.0).translate(0.0, ring_offset, 0.0))
    geom.merge(TorusGeometry(radius=wheel_radius, tube=0.045).rotate_x(math.pi / 2.0).translate(0.0, -ring_offset, 0.0))

    bucket_count = 12
    angle_step = 2.0 * math.pi / bucket_count
    divider_inner_radius = 0.28
    divider_mid_radius = (wheel_radius + divider_inner_radius) * 0.5
    divider_length = wheel_radius - divider_inner_radius
    for index in range(bucket_count):
        angle = index * angle_step
        rotation = math.pi / 2.0 - angle
        divider = BoxGeometry((0.026, wheel_width - 0.05, divider_length))
        divider.rotate_y(rotation).translate(
            math.cos(angle) * divider_mid_radius,
            0.0,
            math.sin(angle) * divider_mid_radius,
        )
        geom.merge(divider)

        floor_angle = angle + angle_step * 0.5
        floor = BoxGeometry((0.30, wheel_width - 0.06, 0.022))
        floor.rotate_y(math.pi / 2.0 - floor_angle).translate(
            math.cos(floor_angle) * (wheel_radius - 0.12),
            0.0,
            math.sin(floor_angle) * (wheel_radius - 0.12),
        )
        geom.merge(floor)

    return geom


def build_object_model() -> ArticulatedObject:
    model = ArticulatedObject(name="overshot_waterwheel")

    timber = model.material("timber", rgba=(0.47, 0.34, 0.21, 1.0))
    dark_timber = model.material("dark_timber", rgba=(0.34, 0.24, 0.15, 1.0))
    iron = model.material("iron", rgba=(0.28, 0.29, 0.30, 1.0))
    wet_wood = model.material("wet_wood", rgba=(0.39, 0.30, 0.20, 1.0))

    frame = model.part("frame")
    frame.inertial = Inertial.from_geometry(
        Box((2.00, 1.02, 3.35)),
        mass=520.0,
        origin=Origin(xyz=(-0.10, 0.0, 1.65)),
    )

    # Ground runners and lower ties.
    frame.visual(
        Box((1.78, 0.12, 0.12)),
        origin=Origin(xyz=(-0.12, -0.44, 0.06)),
        material=dark_timber,
    )
    frame.visual(
        Box((1.78, 0.12, 0.12)),
        origin=Origin(xyz=(-0.12, 0.44, 0.06)),
        material=dark_timber,
    )
    frame.visual(
        Box((0.16, 1.00, 0.12)),
        origin=Origin(xyz=(-0.62, 0.0, 0.06)),
        material=dark_timber,
    )
    frame.visual(
        Box((0.16, 1.00, 0.12)),
        origin=Origin(xyz=(0.42, 0.0, 0.06)),
        material=dark_timber,
    )
    # Side support frames.
    for support_y in (-0.38, 0.38):
        _add_box_beam(
            frame,
            (-0.26, support_y, 0.12),
            (-0.26, support_y, 2.10),
            cross_x=0.12,
            cross_y=0.10,
            material=timber,
        )
        _add_box_beam(
            frame,
            (0.30, support_y, 0.12),
            (0.30, support_y, 1.96),
            cross_x=0.12,
            cross_y=0.10,
            material=timber,
        )
        _add_box_beam(
            frame,
            (-0.26, support_y, 2.10),
            (0.30, support_y, 1.96),
            cross_x=0.10,
            cross_y=0.10,
            material=timber,
        )
        _add_box_beam(
            frame,
            (-0.58, support_y, 0.12),
            (-0.34, support_y, 1.28),
            cross_x=0.10,
            cross_y=0.10,
            material=timber,
        )
        frame.visual(
            Box((0.12, 0.08, 0.12)),
            origin=Origin(xyz=(0.0, -0.46 if support_y < 0 else 0.46, 1.55)),
            material=iron,
            name="left_bearing_block" if support_y < 0 else "right_bearing_block",
        )
        _add_box_beam(
            frame,
            (-0.06, -0.46 if support_y < 0 else 0.46, 1.55),
            (-0.20, support_y, 1.55),
            cross_x=0.035,
            cross_y=0.035,
            material=iron,
        )
        _add_box_beam(
            frame,
            (0.06, -0.46 if support_y < 0 else 0.46, 1.55),
            (0.24, support_y, 1.55),
            cross_x=0.035,
            cross_y=0.035,
            material=iron,
        )

    # Upstream trestle carrying the feed chute back to the ground runners.
    for trestle_y in (-0.34, 0.34):
        frame.visual(
            Box((0.12, 0.12, 2.92)),
            origin=Origin(xyz=(-0.86, trestle_y, 1.46)),
            material=dark_timber,
        )
    frame.visual(
        Box((0.12, 0.86, 0.12)),
        origin=Origin(xyz=(-0.86, 0.0, 2.96)),
        material=dark_timber,
    )
    for brace_y in (-0.18, 0.18):
        _add_box_beam(
            frame,
            (-0.86, brace_y * 1.7, 2.96),
            (-0.60, brace_y, 2.90),
            cross_x=0.08,
            cross_y=0.08,
            material=dark_timber,
        )

    # Outboard chute braces running outside the wheel cheeks.
    for start_y, end_y in ((-0.30, -0.18), (0.30, 0.18)):
        _add_box_beam(
            frame,
            (-0.18, start_y, 2.18),
            (-0.80, end_y, 2.92),
            cross_x=0.08,
            cross_y=0.08,
            material=dark_timber,
        )

    # Upstream timber flume.
    chute_start = (-1.05, 0.0, 3.02)
    chute_end = (-0.68, 0.0, 2.93)
    _add_box_beam(
        frame,
        chute_start,
        chute_end,
        cross_x=0.05,
        cross_y=0.36,
        material=wet_wood,
        name="chute_floor",
    )
    for wall_y in (-0.175, 0.175):
        _add_box_beam(
            frame,
            (chute_start[0], wall_y, chute_start[2] + 0.11),
            (chute_end[0], wall_y, chute_end[2] + 0.11),
            cross_x=0.18,
            cross_y=0.035,
            material=timber,
        )
    frame.visual(
        Box((0.18, 0.36, 0.04)),
        origin=Origin(xyz=(-0.59, 0.0, 2.88)),
        material=wet_wood,
        name="headbox_floor",
    )
    frame.visual(
        Box((0.18, 0.03, 0.26)),
        origin=Origin(xyz=(-0.59, -0.185, 3.01)),
        material=timber,
    )
    frame.visual(
        Box((0.18, 0.03, 0.26)),
        origin=Origin(xyz=(-0.59, 0.185, 3.01)),
        material=timber,
    )
    frame.visual(
        Box((0.03, 0.36, 0.26)),
        origin=Origin(xyz=(-0.675, 0.0, 3.01)),
        material=timber,
    )

    # Headbox and outlet over the wheel.
    frame.visual(
        Box((0.06, 0.34, 0.16)),
        origin=Origin(xyz=(-0.62, 0.0, 2.96)),
        material=dark_timber,
    )
    frame.visual(
        Box((0.10, 0.34, 0.04)),
        origin=Origin(xyz=(-0.48, 0.0, 2.70)),
        material=dark_timber,
        name="gate_sill",
    )
    frame.visual(
        Box((0.12, 0.34, 0.07)),
        origin=Origin(xyz=(-0.48, 0.0, 3.485)),
        material=dark_timber,
        name="gate_lintel",
    )

    outlet_start = (-0.24, 0.0, 2.77)
    outlet_end = (-0.12, 0.0, 2.71)
    _add_box_beam(
        frame,
        outlet_start,
        outlet_end,
        cross_x=0.05,
        cross_y=0.30,
        material=wet_wood,
    )
    for wall_y in (-0.165, 0.165):
        _add_box_beam(
            frame,
            (outlet_start[0], wall_y, outlet_start[2] + 0.08),
            (outlet_end[0], wall_y, outlet_end[2] + 0.08),
            cross_x=0.14,
            cross_y=0.03,
            material=timber,
        )
    frame.visual(
        Box((0.28, 0.03, 0.06)),
        origin=Origin(xyz=(-0.32, -0.195, 2.82)),
        material=timber,
    )
    frame.visual(
        Box((0.28, 0.03, 0.06)),
        origin=Origin(xyz=(-0.32, 0.195, 2.82)),
        material=timber,
    )

    guide_z = 3.06
    guide_height = 0.78
    guide_y = 0.182
    guide_thickness_x = 0.028
    guide_thickness_y = 0.024
    frame.visual(
        Box((guide_thickness_x, guide_thickness_y, guide_height)),
        origin=Origin(xyz=(-0.516, -guide_y, guide_z)),
        material=dark_timber,
        name="left_guide_upstream",
    )
    frame.visual(
        Box((guide_thickness_x, guide_thickness_y, guide_height)),
        origin=Origin(xyz=(-0.444, -guide_y, guide_z)),
        material=dark_timber,
        name="left_guide_downstream",
    )
    frame.visual(
        Box((guide_thickness_x, guide_thickness_y, guide_height)),
        origin=Origin(xyz=(-0.516, guide_y, guide_z)),
        material=dark_timber,
        name="right_guide_upstream",
    )
    frame.visual(
        Box((guide_thickness_x, guide_thickness_y, guide_height)),
        origin=Origin(xyz=(-0.444, guide_y, guide_z)),
        material=dark_timber,
        name="right_guide_downstream",
    )

    waterwheel = model.part("waterwheel")
    waterwheel.visual(
        mesh_from_geometry(_build_waterwheel_geometry(), "overshot_wheel_body"),
        material=wet_wood,
        name="wheel_body",
    )
    waterwheel.visual(
        Box((0.10, 0.05, 0.06)),
        origin=Origin(xyz=(0.08, 0.36, 0.0)),
        material=iron,
        name="wheel_key",
    )
    waterwheel.inertial = Inertial.from_geometry(
        Cylinder(radius=1.10, length=0.84),
        mass=180.0,
        origin=Origin(rpy=(math.pi / 2.0, 0.0, 0.0)),
    )

    sluice_gate = model.part("sluice_gate")
    sluice_gate.visual(
        Box((0.032, 0.290, 0.52)),
        origin=Origin(xyz=(0.0, 0.0, 0.26)),
        material=dark_timber,
        name="gate_panel",
    )
    sluice_gate.visual(
        Box((0.048, 0.18, 0.04)),
        origin=Origin(xyz=(0.0, 0.0, 0.54)),
        material=iron,
        name="gate_handle",
    )
    sluice_gate.visual(
        Box((0.006, 0.05, 0.44)),
        origin=Origin(xyz=(-0.019, -0.155, 0.22)),
        material=timber,
        name="runner_left_upstream",
    )
    sluice_gate.visual(
        Box((0.006, 0.05, 0.44)),
        origin=Origin(xyz=(0.019, -0.155, 0.22)),
        material=timber,
        name="runner_left_downstream",
    )
    sluice_gate.visual(
        Box((0.006, 0.05, 0.44)),
        origin=Origin(xyz=(-0.019, 0.155, 0.22)),
        material=timber,
        name="runner_right_upstream",
    )
    sluice_gate.visual(
        Box((0.006, 0.05, 0.44)),
        origin=Origin(xyz=(0.019, 0.155, 0.22)),
        material=timber,
        name="runner_right_downstream",
    )
    sluice_gate.inertial = Inertial.from_geometry(
        Box((0.05, 0.32, 0.58)),
        mass=18.0,
        origin=Origin(xyz=(0.0, 0.0, 0.29)),
    )

    model.articulation(
        "wheel_spin",
        ArticulationType.CONTINUOUS,
        parent=frame,
        child=waterwheel,
        origin=Origin(xyz=(0.0, 0.0, 1.55)),
        axis=(0.0, 1.0, 0.0),
        motion_limits=MotionLimits(effort=220.0, velocity=1.2),
    )
    model.articulation(
        "gate_slide",
        ArticulationType.PRISMATIC,
        parent=frame,
        child=sluice_gate,
        origin=Origin(xyz=(-0.48, 0.0, 2.75)),
        axis=(0.0, 0.0, 1.0),
        motion_limits=MotionLimits(
            effort=90.0,
            velocity=0.20,
            lower=0.0,
            upper=0.16,
        ),
    )

    return model


def run_tests() -> TestReport:
    ctx = TestContext(object_model)
    frame = object_model.get_part("frame")
    waterwheel = object_model.get_part("waterwheel")
    sluice_gate = object_model.get_part("sluice_gate")
    wheel_spin = object_model.get_articulation("wheel_spin")
    gate_slide = object_model.get_articulation("gate_slide")

    ctx.check_model_valid()
    ctx.check_mesh_assets_ready()
    ctx.fail_if_isolated_parts()
    ctx.warn_if_part_contains_disconnected_geometry_islands()
    ctx.fail_if_parts_overlap_in_current_pose()

    ctx.check(
        "wheel_joint_axis_is_horizontal",
        wheel_spin.axis == (0.0, 1.0, 0.0),
        f"expected wheel spin axis (0, 1, 0), got {wheel_spin.axis}",
    )
    ctx.check(
        "gate_joint_axis_is_vertical",
        gate_slide.axis == (0.0, 0.0, 1.0),
        f"expected gate slide axis (0, 0, 1), got {gate_slide.axis}",
    )

    ctx.expect_gap(
        sluice_gate,
        frame,
        axis="x",
        positive_elem="gate_panel",
        negative_elem="left_guide_upstream",
        min_gap=0.004,
        max_gap=0.012,
        name="gate_clear_of_upstream_guide_rest",
    )
    ctx.expect_gap(
        frame,
        sluice_gate,
        axis="x",
        positive_elem="left_guide_downstream",
        negative_elem="gate_panel",
        min_gap=0.004,
        max_gap=0.012,
        name="gate_clear_of_downstream_guide_rest",
    )
    ctx.expect_gap(
        sluice_gate,
        frame,
        axis="z",
        positive_elem="gate_panel",
        negative_elem="gate_sill",
        min_gap=0.015,
        max_gap=0.035,
        name="gate_seats_just_above_sill_when_closed",
    )
    ctx.expect_contact(
        frame,
        sluice_gate,
        elem_a="left_guide_upstream",
        elem_b="runner_left_upstream",
        name="left_runner_contacts_guide_rest",
    )
    ctx.expect_contact(
        frame,
        sluice_gate,
        elem_a="right_guide_downstream",
        elem_b="runner_right_downstream",
        name="right_runner_contacts_guide_rest",
    )

    wheel_key_rest = ctx.part_element_world_aabb(waterwheel, elem="wheel_key")
    if wheel_key_rest is None:
        ctx.fail("wheel_key_aabb_rest", "wheel_key visual AABB was unavailable")
    else:
        key_rest_center = (
            (wheel_key_rest[0][0] + wheel_key_rest[1][0]) * 0.5,
            (wheel_key_rest[0][1] + wheel_key_rest[1][1]) * 0.5,
            (wheel_key_rest[0][2] + wheel_key_rest[1][2]) * 0.5,
        )
        with ctx.pose({wheel_spin: math.pi / 2.0}):
            wheel_key_turned = ctx.part_element_world_aabb(waterwheel, elem="wheel_key")
            if wheel_key_turned is None:
                ctx.fail(
                    "wheel_key_aabb_turned",
                    "wheel_key visual AABB was unavailable in a turned pose",
                )
            else:
                key_turned_center = (
                    (wheel_key_turned[0][0] + wheel_key_turned[1][0]) * 0.5,
                    (wheel_key_turned[0][1] + wheel_key_turned[1][1]) * 0.5,
                    (wheel_key_turned[0][2] + wheel_key_turned[1][2]) * 0.5,
                )
                ctx.check(
                    "wheel_spin_moves_asymmetric_feature",
                    abs(key_turned_center[0] - key_rest_center[0]) > 0.05
                    or abs(key_turned_center[2] - key_rest_center[2]) > 0.05,
                    (
                        "expected the keyed axle block to move when the wheel rotates; "
                        f"rest={key_rest_center}, turned={key_turned_center}"
                    ),
                )

    gate_rest = ctx.part_element_world_aabb(sluice_gate, elem="gate_panel")
    if gate_rest is None:
        ctx.fail("gate_panel_aabb_rest", "gate_panel visual AABB was unavailable")
    else:
        gate_rest_center = (
            (gate_rest[0][0] + gate_rest[1][0]) * 0.5,
            (gate_rest[0][1] + gate_rest[1][1]) * 0.5,
            (gate_rest[0][2] + gate_rest[1][2]) * 0.5,
        )
        with ctx.pose({gate_slide: 0.16}):
            gate_open = ctx.part_element_world_aabb(sluice_gate, elem="gate_panel")
            if gate_open is None:
                ctx.fail("gate_panel_aabb_open", "gate_panel AABB was unavailable at max lift")
            else:
                gate_open_center = (
                    (gate_open[0][0] + gate_open[1][0]) * 0.5,
                    (gate_open[0][1] + gate_open[1][1]) * 0.5,
                    (gate_open[0][2] + gate_open[1][2]) * 0.5,
                )
                ctx.check(
                    "gate_rises_without_side_drift",
                    gate_open_center[2] > gate_rest_center[2] + 0.14
                    and abs(gate_open_center[0] - gate_rest_center[0]) < 1e-6
                    and abs(gate_open_center[1] - gate_rest_center[1]) < 1e-6,
                    (
                        "expected a pure vertical lift; "
                        f"rest={gate_rest_center}, open={gate_open_center}"
                    ),
                )

            ctx.expect_gap(
                sluice_gate,
                frame,
                axis="x",
                positive_elem="gate_panel",
                negative_elem="left_guide_upstream",
                min_gap=0.004,
                max_gap=0.012,
                name="gate_clear_of_upstream_guide_open",
            )
            ctx.expect_gap(
                frame,
                sluice_gate,
                axis="x",
                positive_elem="left_guide_downstream",
                negative_elem="gate_panel",
                min_gap=0.004,
                max_gap=0.012,
                name="gate_clear_of_downstream_guide_open",
            )
            ctx.expect_gap(
                frame,
                sluice_gate,
                axis="z",
                positive_elem="gate_lintel",
                negative_elem="gate_panel",
                min_gap=0.010,
                max_gap=0.070,
                name="gate_remains_captured_below_lintel",
            )
            ctx.expect_contact(
                frame,
                sluice_gate,
                elem_a="left_guide_upstream",
                elem_b="runner_left_upstream",
                name="left_runner_contacts_guide_open",
            )
            ctx.expect_contact(
                frame,
                sluice_gate,
                elem_a="right_guide_downstream",
                elem_b="runner_right_downstream",
                name="right_runner_contacts_guide_open",
            )

    return ctx.report()


# >>> USER_CODE_END

object_model = build_object_model()
