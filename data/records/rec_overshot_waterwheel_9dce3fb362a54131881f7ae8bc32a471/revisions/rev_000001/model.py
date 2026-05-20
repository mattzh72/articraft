from __future__ import annotations

# User code should import every SDK/stdlib symbol it uses instead of relying on
# hidden scaffold imports.
# >>> USER_CODE_START
import math

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
    mesh_from_geometry,
    wire_from_points,
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
    thickness: float,
    material,
    extra: float = 0.0,
    name: str | None = None,
) -> None:
    part.visual(
        Box((thickness, thickness, _distance(a, b) + extra)),
        origin=Origin(xyz=_midpoint(a, b), rpy=_rpy_for_segment(a, b)),
        material=material,
        name=name,
    )


def _add_cylinder_member(
    part,
    a: tuple[float, float, float],
    b: tuple[float, float, float],
    *,
    radius: float,
    material,
    extra: float = 0.0,
    name: str | None = None,
) -> None:
    part.visual(
        Cylinder(radius=radius, length=_distance(a, b) + extra),
        origin=Origin(xyz=_midpoint(a, b), rpy=_rpy_for_segment(a, b)),
        material=material,
        name=name,
    )


def _circle_points(radius: float, segments: int) -> list[tuple[float, float, float]]:
    return [
        (
            radius * math.cos(2.0 * math.pi * index / segments),
            0.0,
            radius * math.sin(2.0 * math.pi * index / segments),
        )
        for index in range(segments)
    ]


def build_object_model() -> ArticulatedObject:
    model = ArticulatedObject(name="overshot_waterwheel")

    painted_steel = model.material("painted_steel", rgba=(0.22, 0.24, 0.26, 1.0))
    wheel_steel = model.material("wheel_steel", rgba=(0.56, 0.59, 0.62, 1.0))
    weathered_steel = model.material("weathered_steel", rgba=(0.43, 0.45, 0.46, 1.0))
    darker_steel = model.material("darker_steel", rgba=(0.16, 0.17, 0.18, 1.0))

    wheel_radius = 0.62
    wheel_center_z = 0.94
    wheel_width = 0.14
    rim_center_y = 0.055

    frame = model.part("frame")
    frame.inertial = Inertial.from_geometry(
        Box((1.34, 0.42, 1.86)),
        mass=42.0,
        origin=Origin(xyz=(0.0, 0.0, 0.93)),
    )

    support_side_y = 0.195

    for side_y in (-0.18, 0.18):
        frame.visual(
            Box((1.30, 0.06, 0.10)),
            origin=Origin(xyz=(0.0, side_y, 0.05)),
            material=painted_steel,
        )

    for x in (-0.48, 0.0, 0.48):
        frame.visual(
            Box((0.10, 0.36, 0.08)),
            origin=Origin(xyz=(x, 0.0, 0.06)),
            material=painted_steel,
        )

    for side_y in (-support_side_y, support_side_y):
        _add_box_beam(
            frame,
            (0.0, side_y, 0.08),
            (0.0, side_y, 1.28),
            thickness=0.05,
            material=painted_steel,
            extra=0.03,
        )
        _add_box_beam(
            frame,
            (-0.35, side_y, 0.09),
            (0.0, side_y, 1.20),
            thickness=0.04,
            material=painted_steel,
            extra=0.03,
        )
        _add_box_beam(
            frame,
            (0.35, side_y, 0.09),
            (0.0, side_y, 1.20),
            thickness=0.04,
            material=painted_steel,
            extra=0.03,
        )
        _add_box_beam(
            frame,
            (0.0, side_y, 1.20),
            (-0.55, side_y, 1.61),
            thickness=0.04,
            material=painted_steel,
            extra=0.03,
        )
        frame.visual(
            Box((0.08, 0.06, 0.10)),
            origin=Origin(
                xyz=(0.0, side_y + (-0.005 if side_y < 0.0 else 0.005), wheel_center_z)
            ),
            material=darker_steel,
            name="left_bearing" if side_y < 0.0 else "right_bearing",
        )

    frame.visual(
        Box((0.08, 0.36, 0.04)),
        origin=Origin(xyz=(0.0, 0.0, 1.20)),
        material=painted_steel,
    )
    frame.visual(
        Box((0.10, 0.40, 0.04)),
        origin=Origin(xyz=(-0.48, 0.0, 1.58)),
        material=painted_steel,
    )
    frame.visual(
        Box((0.10, 0.40, 0.04)),
        origin=Origin(xyz=(-0.20, 0.0, 1.58)),
        material=painted_steel,
    )
    for side_y in (-support_side_y, support_side_y):
        _add_box_beam(
            frame,
            (-0.48, side_y, 1.58),
            (-0.20, side_y, 1.58),
            thickness=0.04,
            material=painted_steel,
        )

    rim_ring_mesh = mesh_from_geometry(
        wire_from_points(
            _circle_points(wheel_radius, 56),
            radius=0.017,
            radial_segments=18,
            closed_path=True,
            cap_ends=False,
            corner_mode="miter",
            up_hint=(0.0, 1.0, 0.0),
        ),
        "waterwheel_rim_ring",
    )

    wheel = model.part("wheel")
    wheel.inertial = Inertial.from_geometry(
        Cylinder(radius=0.65, length=0.18),
        mass=18.0,
        origin=Origin(rpy=(math.pi / 2.0, 0.0, 0.0)),
    )

    wheel.visual(
        rim_ring_mesh,
        origin=Origin(xyz=(0.0, -rim_center_y, 0.0)),
        material=wheel_steel,
        name="left_rim",
    )
    wheel.visual(
        rim_ring_mesh,
        origin=Origin(xyz=(0.0, rim_center_y, 0.0)),
        material=wheel_steel,
        name="right_rim",
    )
    _add_cylinder_member(
        wheel,
        (0.0, -0.17, 0.0),
        (0.0, 0.17, 0.0),
        radius=0.028,
        material=darker_steel,
        name="axle",
    )
    _add_cylinder_member(
        wheel,
        (0.0, -0.06, 0.0),
        (0.0, 0.06, 0.0),
        radius=0.09,
        material=weathered_steel,
    )
    _add_cylinder_member(
        wheel,
        (0.0, -0.060, 0.0),
        (0.0, -0.050, 0.0),
        radius=0.15,
        material=weathered_steel,
    )
    _add_cylinder_member(
        wheel,
        (0.0, 0.050, 0.0),
        (0.0, 0.060, 0.0),
        radius=0.15,
        material=weathered_steel,
    )

    for angle_index in range(8):
        theta = angle_index * math.pi / 4.0 + math.pi / 8.0
        inner_radius = 0.14
        outer_radius = 0.59
        for side_y in (-rim_center_y, rim_center_y):
            _add_box_beam(
                wheel,
                (
                    inner_radius * math.cos(theta),
                    side_y,
                    inner_radius * math.sin(theta),
                ),
                (
                    outer_radius * math.cos(theta),
                    side_y,
                    outer_radius * math.sin(theta),
                ),
                thickness=0.012,
                material=weathered_steel,
                extra=0.04,
            )

    for bucket_index in range(12):
        theta = math.pi / 2.0 + bucket_index * (2.0 * math.pi / 12.0)
        radial_center = 0.54
        tangential_center = 0.58
        wheel.visual(
            Box((0.18, 0.112, 0.010)),
            origin=Origin(
                xyz=(
                    radial_center * math.cos(theta),
                    0.0,
                    radial_center * math.sin(theta),
                ),
                rpy=(0.0, -theta, 0.0),
            ),
            material=weathered_steel,
        )
        wheel.visual(
            Box((0.020, 0.112, 0.140)),
            origin=Origin(
                xyz=(
                    tangential_center * math.cos(theta),
                    0.0,
                    tangential_center * math.sin(theta),
                ),
                rpy=(0.0, -theta, 0.0),
            ),
            material=wheel_steel,
        )

    chute = model.part("chute")
    chute.inertial = Inertial.from_geometry(
        Box((0.64, 0.104, 0.40)),
        mass=8.0,
        origin=Origin(xyz=(-0.31, 0.0, 0.12)),
    )
    chute.visual(
        Box((0.62, 0.104, 0.008)),
        origin=Origin(xyz=(-0.31, 0.0, 0.004)),
        material=painted_steel,
        name="chute_floor",
    )
    chute.visual(
        Box((0.62, 0.008, 0.10)),
        origin=Origin(xyz=(-0.31, -0.048, 0.054)),
        material=painted_steel,
    )
    chute.visual(
        Box((0.62, 0.008, 0.10)),
        origin=Origin(xyz=(-0.31, 0.048, 0.054)),
        material=painted_steel,
    )
    chute.visual(
        Box((0.010, 0.104, 0.14)),
        origin=Origin(xyz=(-0.615, 0.0, 0.070)),
        material=painted_steel,
    )
    chute.visual(
        Box((0.022, 0.060, 0.018)),
        origin=Origin(xyz=(0.006, 0.0, 0.301)),
        material=darker_steel,
        name="guide_header",
    )
    chute.visual(
        Box((0.006, 0.012, 0.382)),
        origin=Origin(xyz=(0.011, -0.022, 0.101)),
        material=darker_steel,
        name="left_front_guide",
    )
    chute.visual(
        Box((0.006, 0.012, 0.382)),
        origin=Origin(xyz=(0.011, 0.022, 0.101)),
        material=darker_steel,
        name="right_front_guide",
    )
    chute.visual(
        Box((0.006, 0.012, 0.382)),
        origin=Origin(xyz=(0.001, -0.022, 0.101)),
        material=darker_steel,
        name="left_rear_guide",
    )
    chute.visual(
        Box((0.006, 0.012, 0.382)),
        origin=Origin(xyz=(0.001, 0.022, 0.101)),
        material=darker_steel,
        name="right_rear_guide",
    )

    gate = model.part("gate")
    gate.inertial = Inertial.from_geometry(
        Box((0.024, 0.060, 0.24)),
        mass=2.2,
        origin=Origin(xyz=(0.006, 0.0, -0.09)),
    )
    gate.visual(
        Box((0.004, 0.052, 0.180)),
        origin=Origin(xyz=(0.006, 0.0, -0.115)),
        material=wheel_steel,
        name="gate_plate",
    )
    gate.visual(
        Box((0.004, 0.010, 0.140)),
        origin=Origin(xyz=(0.006, -0.022, -0.075)),
        material=wheel_steel,
        name="left_clip",
    )
    gate.visual(
        Box((0.004, 0.010, 0.140)),
        origin=Origin(xyz=(0.006, 0.022, -0.075)),
        material=wheel_steel,
        name="right_clip",
    )
    gate.visual(
        Box((0.018, 0.026, 0.040)),
        origin=Origin(xyz=(0.014, 0.0, -0.005)),
        material=weathered_steel,
        name="gate_handle",
    )

    model.articulation(
        "wheel_spin",
        ArticulationType.CONTINUOUS,
        parent=frame,
        child=wheel,
        origin=Origin(xyz=(0.0, 0.0, wheel_center_z)),
        axis=(0.0, 1.0, 0.0),
        motion_limits=MotionLimits(effort=40.0, velocity=1.5),
    )
    model.articulation(
        "frame_to_chute",
        ArticulationType.FIXED,
        parent=frame,
        child=chute,
        origin=Origin(xyz=(-0.10, 0.0, 1.60)),
    )
    model.articulation(
        "gate_slide",
        ArticulationType.PRISMATIC,
        parent=chute,
        child=gate,
        origin=Origin(xyz=(0.0, 0.0, 0.0)),
        axis=(0.0, 0.0, 1.0),
        motion_limits=MotionLimits(
            effort=10.0,
            velocity=0.12,
            lower=0.0,
            upper=0.14,
        ),
    )

    return model


def run_tests() -> TestReport:
    ctx = TestContext(object_model)
    frame = object_model.get_part("frame")
    wheel = object_model.get_part("wheel")
    chute = object_model.get_part("chute")
    gate = object_model.get_part("gate")
    wheel_spin = object_model.get_articulation("wheel_spin")
    gate_slide = object_model.get_articulation("gate_slide")

    ctx.check_model_valid()
    ctx.check_mesh_assets_ready()
    ctx.fail_if_isolated_parts()
    ctx.warn_if_part_contains_disconnected_geometry_islands()
    ctx.fail_if_parts_overlap_in_current_pose()

    ctx.expect_contact(wheel, frame, elem_a="axle", elem_b="left_bearing")
    ctx.expect_contact(wheel, frame, elem_a="axle", elem_b="right_bearing")
    ctx.expect_contact(chute, frame, elem_a="chute_floor")
    ctx.expect_contact(gate, chute, elem_a="left_clip", elem_b="left_rear_guide")
    ctx.expect_contact(gate, chute, elem_a="right_clip", elem_b="right_rear_guide")

    ctx.expect_overlap(
        gate,
        chute,
        axes="yz",
        elem_a="left_clip",
        elem_b="left_front_guide",
        min_overlap=0.008,
        name="left_clip_captured_by_front_guide",
    )
    ctx.expect_overlap(
        gate,
        chute,
        axes="yz",
        elem_a="right_clip",
        elem_b="right_front_guide",
        min_overlap=0.008,
        name="right_clip_captured_by_front_guide",
    )
    ctx.expect_gap(
        chute,
        wheel,
        axis="z",
        positive_elem="chute_floor",
        min_gap=0.015,
        max_gap=0.08,
        name="chute_sits_just_above_top_buckets",
    )
    ctx.expect_overlap(chute, wheel, axes="y", min_overlap=0.10, name="chute_aligned_over_wheel_width")

    ctx.check(
        "wheel_spin_axis_is_horizontal",
        tuple(round(value, 6) for value in wheel_spin.axis) == (0.0, 1.0, 0.0),
        details=f"axis={wheel_spin.axis}",
    )
    ctx.check(
        "wheel_joint_is_continuous",
        wheel_spin.joint_type == ArticulationType.CONTINUOUS,
        details=f"type={wheel_spin.joint_type}",
    )
    ctx.check(
        "gate_slide_axis_is_vertical",
        tuple(round(value, 6) for value in gate_slide.axis) == (0.0, 0.0, 1.0),
        details=f"axis={gate_slide.axis}",
    )

    gate_rest = ctx.part_world_position(gate)
    assert gate_rest is not None
    with ctx.pose({gate_slide: 0.14}):
        gate_open = ctx.part_world_position(gate)
        assert gate_open is not None
        ctx.check(
            "gate_rises_when_opened",
            gate_open[2] > gate_rest[2] + 0.13,
            details=f"rest={gate_rest}, open={gate_open}",
        )
        ctx.expect_contact(
            gate,
            chute,
            elem_a="left_clip",
            elem_b="left_rear_guide",
            name="left_clip_stays_attached_when_open",
        )
        ctx.expect_contact(
            gate,
            chute,
            elem_a="right_clip",
            elem_b="right_rear_guide",
            name="right_clip_stays_attached_when_open",
        )
        ctx.expect_overlap(
            gate,
            chute,
            axes="yz",
            elem_a="left_clip",
            elem_b="left_front_guide",
            min_overlap=0.008,
            name="left_clip_stays_captured_when_open",
        )
        ctx.expect_overlap(
            gate,
            chute,
            axes="yz",
            elem_a="right_clip",
            elem_b="right_front_guide",
            min_overlap=0.008,
            name="right_clip_stays_captured_when_open",
        )

    with ctx.pose({wheel_spin: math.pi / 3.0}):
        ctx.expect_contact(
            wheel,
            frame,
            elem_a="axle",
            elem_b="left_bearing",
            name="left_bearing_stays_on_axle_when_wheel_turns",
        )
        ctx.expect_contact(
            wheel,
            frame,
            elem_a="axle",
            elem_b="right_bearing",
            name="right_bearing_stays_on_axle_when_wheel_turns",
        )

    return ctx.report()


# >>> USER_CODE_END

object_model = build_object_model()
