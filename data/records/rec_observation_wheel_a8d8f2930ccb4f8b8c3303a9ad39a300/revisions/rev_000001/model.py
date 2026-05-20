from __future__ import annotations

# User code should import every SDK/stdlib symbol it uses instead of relying on
# hidden scaffold imports.
# >>> USER_CODE_START
from math import atan2, cos, hypot, pi, sin, sqrt

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
    tube_from_spline_points,
)


WHEEL_RADIUS = 1.18
AXLE_HEIGHT = 1.68
WHEEL_HALF_WIDTH = 0.18
SEAT_COUNT = 8


def _save_mesh(name: str, geometry):
    return mesh_from_geometry(geometry, name)


def _midpoint(
    a: tuple[float, float, float], b: tuple[float, float, float]
) -> tuple[float, float, float]:
    return ((a[0] + b[0]) * 0.5, (a[1] + b[1]) * 0.5, (a[2] + b[2]) * 0.5)


def _distance(
    a: tuple[float, float, float], b: tuple[float, float, float]
) -> float:
    return sqrt((b[0] - a[0]) ** 2 + (b[1] - a[1]) ** 2 + (b[2] - a[2]) ** 2)


def _rpy_for_cylinder(
    a: tuple[float, float, float], b: tuple[float, float, float]
) -> tuple[float, float, float]:
    dx = b[0] - a[0]
    dy = b[1] - a[1]
    dz = b[2] - a[2]
    length_xy = hypot(dx, dy)
    yaw = atan2(dy, dx)
    pitch = atan2(length_xy, dz)
    return (0.0, pitch, yaw)


def _add_member(part, a, b, *, radius: float, material, name: str | None = None) -> None:
    part.visual(
        Cylinder(radius=radius, length=_distance(a, b)),
        origin=Origin(xyz=_midpoint(a, b), rpy=_rpy_for_cylinder(a, b)),
        material=material,
        name=name,
    )


def _circle_points(radius: float, *, samples: int = 32) -> list[tuple[float, float, float]]:
    return [
        (0.0, radius * cos((2.0 * pi * i) / samples), radius * sin((2.0 * pi * i) / samples))
        for i in range(samples)
    ]


def _build_bucket_part(model: ArticulatedObject, name: str, *, body_material, frame_material):
    bucket = model.part(name)
    bucket.visual(
        Cylinder(radius=0.009, length=0.122),
        origin=Origin(xyz=(-0.12, 0.0, -0.075)),
        material=frame_material,
        name="hanger_left",
    )
    bucket.visual(
        Cylinder(radius=0.009, length=0.122),
        origin=Origin(xyz=(0.12, 0.0, -0.075)),
        material=frame_material,
        name="hanger_right",
    )
    bucket.visual(
        Box((0.030, 0.180, 0.172)),
        origin=Origin(xyz=(-0.115, 0.0, -0.222)),
        material=body_material,
        name="side_left",
    )
    bucket.visual(
        Box((0.030, 0.180, 0.172)),
        origin=Origin(xyz=(0.115, 0.0, -0.222)),
        material=body_material,
        name="side_right",
    )
    bucket.visual(
        Box((0.220, 0.190, 0.028)),
        origin=Origin(xyz=(0.0, 0.0, -0.321)),
        material=body_material,
        name="seat_floor",
    )
    bucket.visual(
        Box((0.220, 0.024, 0.188)),
        origin=Origin(xyz=(0.0, -0.082, -0.227)),
        material=body_material,
        name="seat_back",
    )
    bucket.visual(
        Box((0.200, 0.084, 0.018)),
        origin=Origin(xyz=(0.0, 0.012, -0.268)),
        material=frame_material,
        name="seat_bench",
    )
    bucket.visual(
        Cylinder(radius=0.012, length=0.220),
        origin=Origin(xyz=(0.0, 0.086, -0.225), rpy=(0.0, pi / 2.0, 0.0)),
        material=frame_material,
        name="front_bar",
    )
    bucket.inertial = Inertial.from_geometry(
        Box((0.250, 0.200, 0.335)),
        mass=3.8,
        origin=Origin(xyz=(0.0, 0.0, -0.168)),
    )
    return bucket


def build_object_model() -> ArticulatedObject:
    model = ArticulatedObject(name="compact_fairground_wheel")

    frame_blue = model.material("frame_blue", rgba=(0.18, 0.28, 0.48, 1.0))
    wheel_white = model.material("wheel_white", rgba=(0.92, 0.92, 0.90, 1.0))
    dark_steel = model.material("dark_steel", rgba=(0.22, 0.24, 0.26, 1.0))
    steel = model.material("steel", rgba=(0.61, 0.64, 0.67, 1.0))
    seat_red = model.material("seat_red", rgba=(0.78, 0.18, 0.16, 1.0))
    seat_yellow = model.material("seat_yellow", rgba=(0.92, 0.72, 0.18, 1.0))
    seat_teal = model.material("seat_teal", rgba=(0.18, 0.55, 0.56, 1.0))

    frame = model.part("frame")
    frame.visual(
        Box((0.18, 1.72, 0.12)),
        origin=Origin(xyz=(0.45, 0.0, 0.06)),
        material=frame_blue,
        name="front_skid",
    )
    frame.visual(
        Box((0.18, 1.72, 0.12)),
        origin=Origin(xyz=(-0.45, 0.0, 0.06)),
        material=frame_blue,
        name="rear_skid",
    )
    _add_member(
        frame,
        (-0.45, -0.62, 0.12),
        (0.45, -0.62, 0.12),
        radius=0.038,
        material=frame_blue,
        name="base_tie_left",
    )
    _add_member(
        frame,
        (-0.45, 0.62, 0.12),
        (0.45, 0.62, 0.12),
        radius=0.038,
        material=frame_blue,
        name="base_tie_right",
    )
    _add_member(
        frame,
        (-0.45, 0.0, 0.12),
        (0.45, 0.0, 0.12),
        radius=0.032,
        material=frame_blue,
        name="base_tie_center",
    )

    front_apex = (0.45, 0.0, AXLE_HEIGHT)
    rear_apex = (-0.45, 0.0, AXLE_HEIGHT)
    for label, x in (("front", 0.45), ("rear", -0.45)):
        _add_member(
            frame,
            (x, -0.62, 0.12),
            (x, 0.0, AXLE_HEIGHT),
            radius=0.045,
            material=frame_blue,
            name=f"{label}_leg_left",
        )
        _add_member(
            frame,
            (x, 0.62, 0.12),
            (x, 0.0, AXLE_HEIGHT),
            radius=0.045,
            material=frame_blue,
            name=f"{label}_leg_right",
        )
        _add_member(
            frame,
            (x, -0.34, 0.72),
            (x, 0.34, 0.72),
            radius=0.025,
            material=frame_blue,
            name=f"{label}_tower_tie",
        )
    _add_member(
        frame,
        (-0.45, -0.62, 0.12),
        (0.45, 0.62, 0.12),
        radius=0.020,
        material=steel,
        name="base_cross_brace_a",
    )
    _add_member(
        frame,
        (-0.45, 0.62, 0.12),
        (0.45, -0.62, 0.12),
        radius=0.020,
        material=steel,
        name="base_cross_brace_b",
    )
    frame.visual(
        Box((0.10, 0.12, 0.12)),
        origin=Origin(xyz=(0.40, 0.0, AXLE_HEIGHT)),
        material=dark_steel,
        name="front_bearing",
    )
    frame.visual(
        Box((0.10, 0.12, 0.12)),
        origin=Origin(xyz=(-0.40, 0.0, AXLE_HEIGHT)),
        material=dark_steel,
        name="rear_bearing",
    )
    frame.inertial = Inertial.from_geometry(
        Box((0.90, 1.72, 1.80)),
        mass=85.0,
        origin=Origin(xyz=(0.0, 0.0, 0.90)),
    )

    wheel = model.part("wheel")
    ring_mesh = _save_mesh(
        "fairground_wheel_ring",
        tube_from_spline_points(
            _circle_points(WHEEL_RADIUS, samples=36),
            radius=0.028,
            samples_per_segment=6,
            radial_segments=18,
            closed_spline=True,
            cap_ends=False,
            up_hint=(1.0, 0.0, 0.0),
        ),
    )
    wheel.visual(
        ring_mesh,
        origin=Origin(xyz=(WHEEL_HALF_WIDTH, 0.0, 0.0)),
        material=wheel_white,
        name="front_ring",
    )
    wheel.visual(
        ring_mesh,
        origin=Origin(xyz=(-WHEEL_HALF_WIDTH, 0.0, 0.0)),
        material=wheel_white,
        name="rear_ring",
    )
    wheel.visual(
        Cylinder(radius=0.145, length=0.46),
        origin=Origin(rpy=(0.0, pi / 2.0, 0.0)),
        material=wheel_white,
        name="hub_barrel",
    )
    wheel.visual(
        Cylinder(radius=0.20, length=0.036),
        origin=Origin(xyz=(WHEEL_HALF_WIDTH - 0.012, 0.0, 0.0), rpy=(0.0, pi / 2.0, 0.0)),
        material=wheel_white,
        name="front_hub_plate",
    )
    wheel.visual(
        Cylinder(radius=0.20, length=0.036),
        origin=Origin(xyz=(-WHEEL_HALF_WIDTH + 0.012, 0.0, 0.0), rpy=(0.0, pi / 2.0, 0.0)),
        material=wheel_white,
        name="rear_hub_plate",
    )
    wheel.visual(
        Cylinder(radius=0.034, length=0.164),
        origin=Origin(xyz=(0.268, 0.0, 0.0), rpy=(0.0, pi / 2.0, 0.0)),
        material=dark_steel,
        name="front_axle_shaft",
    )
    wheel.visual(
        Cylinder(radius=0.034, length=0.164),
        origin=Origin(xyz=(-0.268, 0.0, 0.0), rpy=(0.0, pi / 2.0, 0.0)),
        material=dark_steel,
        name="rear_axle_shaft",
    )

    spoke_start = 0.17
    for index in range(SEAT_COUNT):
        angle = (2.0 * pi * index) / SEAT_COUNT
        rim_point_front = (
            WHEEL_HALF_WIDTH,
            WHEEL_RADIUS * cos(angle),
            WHEEL_RADIUS * sin(angle),
        )
        rim_point_rear = (
            -WHEEL_HALF_WIDTH,
            WHEEL_RADIUS * cos(angle),
            WHEEL_RADIUS * sin(angle),
        )
        hub_point_front = (
            WHEEL_HALF_WIDTH,
            spoke_start * cos(angle),
            spoke_start * sin(angle),
        )
        hub_point_rear = (
            -WHEEL_HALF_WIDTH,
            spoke_start * cos(angle),
            spoke_start * sin(angle),
        )
        _add_member(
            wheel,
            hub_point_front,
            rim_point_front,
            radius=0.016,
            material=steel,
            name=f"front_spoke_{index:02d}",
        )
        _add_member(
            wheel,
            hub_point_rear,
            rim_point_rear,
            radius=0.016,
            material=steel,
            name=f"rear_spoke_{index:02d}",
        )
        _add_member(
            wheel,
            rim_point_rear,
            rim_point_front,
            radius=0.015,
            material=wheel_white,
            name=f"arm_{index:02d}",
        )
    wheel.inertial = Inertial.from_geometry(
        Cylinder(radius=WHEEL_RADIUS, length=0.40),
        mass=42.0,
        origin=Origin(rpy=(0.0, pi / 2.0, 0.0)),
    )

    model.articulation(
        "frame_to_wheel",
        ArticulationType.CONTINUOUS,
        parent=frame,
        child=wheel,
        origin=Origin(xyz=(0.0, 0.0, AXLE_HEIGHT)),
        axis=(1.0, 0.0, 0.0),
        motion_limits=MotionLimits(effort=180.0, velocity=0.9),
    )

    seat_materials = (seat_red, seat_yellow, seat_teal)
    for index in range(SEAT_COUNT):
        angle = (2.0 * pi * index) / SEAT_COUNT
        bucket = _build_bucket_part(
            model,
            f"bucket_{index:02d}",
            body_material=seat_materials[index % len(seat_materials)],
            frame_material=dark_steel,
        )
        model.articulation(
            f"wheel_to_bucket_{index:02d}",
            ArticulationType.CONTINUOUS,
            parent=wheel,
            child=bucket,
            origin=Origin(
                xyz=(
                    0.0,
                    WHEEL_RADIUS * cos(angle),
                    WHEEL_RADIUS * sin(angle),
                )
            ),
            axis=(1.0, 0.0, 0.0),
            motion_limits=MotionLimits(effort=12.0, velocity=2.4),
        )

    return model


def run_tests() -> TestReport:
    ctx = TestContext(object_model)
    # `compile_model` automatically runs baseline sanity/QC:
    # - `check_model_valid()`
    # - exactly one root part
    # - `check_mesh_assets_ready()`
    # - disconnected floating-part-group detection
    # - disconnected within-part geometry-island detection
    # - current-pose real 3D overlap detection
    # Use `run_tests()` only for prompt-specific exact checks, targeted poses,
    # and explicit allowances such as `ctx.allow_overlap(...)`.

    frame = object_model.get_part("frame")
    wheel = object_model.get_part("wheel")
    wheel_spin = object_model.get_articulation("frame_to_wheel")
    bucket_00 = object_model.get_part("bucket_00")
    bucket_joint_00 = object_model.get_articulation("wheel_to_bucket_00")

    ctx.check(
        "wheel uses a continuous horizontal axle joint",
        wheel_spin.joint_type == ArticulationType.CONTINUOUS
        and tuple(wheel_spin.axis) == (1.0, 0.0, 0.0)
        and wheel_spin.motion_limits is not None
        and wheel_spin.motion_limits.lower is None
        and wheel_spin.motion_limits.upper is None,
        details=f"type={wheel_spin.joint_type}, axis={wheel_spin.axis}, limits={wheel_spin.motion_limits}",
    )

    bucket_axes_ok = True
    bucket_joint_names: list[str] = []
    for index in range(SEAT_COUNT):
        bucket_name = f"bucket_{index:02d}"
        joint_name = f"wheel_to_bucket_{index:02d}"
        bucket_joint_names.append(joint_name)
        bucket = object_model.get_part(bucket_name)
        joint = object_model.get_articulation(joint_name)
        bucket_axes_ok = (
            bucket_axes_ok
            and bucket is not None
            and joint.joint_type == ArticulationType.CONTINUOUS
            and tuple(joint.axis) == (1.0, 0.0, 0.0)
        )
    ctx.check(
        "each rim arm carries a bucket on a hanger pivot",
        bucket_axes_ok,
        details=f"checked joints={bucket_joint_names}",
    )

    ctx.expect_contact(
        wheel,
        frame,
        elem_a="front_axle_shaft",
        elem_b="front_bearing",
        contact_tol=0.001,
        name="front axle shaft seats in the front bearing block",
    )
    ctx.expect_contact(
        wheel,
        frame,
        elem_a="rear_axle_shaft",
        elem_b="rear_bearing",
        contact_tol=0.001,
        name="rear axle shaft seats in the rear bearing block",
    )
    ctx.expect_contact(
        bucket_00,
        wheel,
        elem_a="hanger_left",
        elem_b="arm_00",
        contact_tol=0.001,
        name="reference bucket hanger meets its rim arm on the left side",
    )
    ctx.expect_contact(
        bucket_00,
        wheel,
        elem_a="hanger_right",
        elem_b="arm_00",
        contact_tol=0.001,
        name="reference bucket hanger meets its rim arm on the right side",
    )

    ctx.expect_origin_gap(
        bucket_00,
        wheel,
        axis="y",
        min_gap=WHEEL_RADIUS - 0.001,
        max_gap=WHEEL_RADIUS + 0.001,
        name="reference bucket starts on the side rim position",
    )

    rest_pos = ctx.part_world_position(bucket_00)
    quarter_turn_pose = {wheel_spin: pi / 2.0}
    for index in range(SEAT_COUNT):
        quarter_turn_pose[object_model.get_articulation(f"wheel_to_bucket_{index:02d}")] = -pi / 2.0

    with ctx.pose(quarter_turn_pose):
        turned_pos = ctx.part_world_position(bucket_00)
        seat_back_aabb = ctx.part_element_world_aabb(bucket_00, elem="seat_back")
        upright_ok = False
        if seat_back_aabb is not None:
            y_span = seat_back_aabb[1][1] - seat_back_aabb[0][1]
            z_span = seat_back_aabb[1][2] - seat_back_aabb[0][2]
            upright_ok = z_span > y_span * 3.0
        else:
            y_span = None
            z_span = None
        ctx.check(
            "positive wheel rotation lifts a side bucket upward",
            rest_pos is not None
            and turned_pos is not None
            and turned_pos[2] > rest_pos[2] + (WHEEL_RADIUS * 0.75),
            details=f"rest={rest_pos}, turned={turned_pos}",
        )
        ctx.check(
            "counter-rotated bucket remains upright after a quarter turn",
            upright_ok,
            details=f"seat_back_y_span={y_span}, seat_back_z_span={z_span}",
        )

    frame_aabb = ctx.part_world_aabb(frame)
    wheel_aabb = ctx.part_world_aabb(wheel)
    frame_clearance_ok = False
    if frame_aabb is not None and wheel_aabb is not None:
        frame_clearance_ok = wheel_aabb[0][2] > frame_aabb[0][2] + 0.20
    ctx.check(
        "wheel sits elevated within the support frame",
        frame_clearance_ok,
        details=f"frame_aabb={frame_aabb}, wheel_aabb={wheel_aabb}",
    )

    return ctx.report()


# >>> USER_CODE_END

object_model = build_object_model()
