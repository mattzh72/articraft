from __future__ import annotations

# User code should import every SDK/stdlib symbol it uses instead of relying on
# hidden scaffold imports.
# >>> USER_CODE_START
from math import atan2, hypot, pi, sqrt

from sdk import (
    ArticulatedObject,
    ArticulationType,
    Box,
    Cylinder,
    MotionLimits,
    Origin,
    TestContext,
    TestReport,
)


Point3 = tuple[float, float, float]


def _midpoint(a: Point3, b: Point3) -> Point3:
    return ((a[0] + b[0]) * 0.5, (a[1] + b[1]) * 0.5, (a[2] + b[2]) * 0.5)


def _distance(a: Point3, b: Point3) -> float:
    return sqrt((b[0] - a[0]) ** 2 + (b[1] - a[1]) ** 2 + (b[2] - a[2]) ** 2)


def _rpy_for_cylinder(a: Point3, b: Point3) -> tuple[float, float, float]:
    dx = b[0] - a[0]
    dy = b[1] - a[1]
    dz = b[2] - a[2]
    length_xy = hypot(dx, dy)
    yaw = atan2(dy, dx)
    pitch = atan2(length_xy, dz)
    return (0.0, pitch, yaw)


def _add_tube(
    part,
    a: Point3,
    b: Point3,
    *,
    radius: float,
    material,
    name: str | None = None,
) -> None:
    part.visual(
        Cylinder(radius=radius, length=_distance(a, b)),
        origin=Origin(xyz=_midpoint(a, b), rpy=_rpy_for_cylinder(a, b)),
        material=material,
        name=name,
    )


def build_object_model() -> ArticulatedObject:
    model = ArticulatedObject(name="bench_playground_swing")

    powder_coat = model.material("powder_coat", rgba=(0.21, 0.28, 0.31, 1.0))
    steel_dark = model.material("steel_dark", rgba=(0.15, 0.17, 0.19, 1.0))
    wood = model.material("wood", rgba=(0.63, 0.45, 0.26, 1.0))
    bushing = model.material("bushing", rgba=(0.56, 0.58, 0.60, 1.0))

    frame = model.part("frame")

    crossbeam_z = 2.05
    crossbeam_radius = 0.065
    crossbeam_underside = crossbeam_z - crossbeam_radius
    upper_pivot_drop = 0.03
    upper_pivot_z = crossbeam_underside - upper_pivot_drop
    support_half_span = 1.14
    support_half_depth = 0.84
    apex_inset = 1.03

    left_front_foot = (-support_half_span, support_half_depth, 0.05)
    left_rear_foot = (-support_half_span, -support_half_depth, 0.05)
    right_front_foot = (support_half_span, support_half_depth, 0.05)
    right_rear_foot = (support_half_span, -support_half_depth, 0.05)
    left_apex = (-apex_inset, 0.0, crossbeam_underside)
    right_apex = (apex_inset, 0.0, crossbeam_underside)

    frame.visual(
        Cylinder(radius=crossbeam_radius, length=2.60),
        origin=Origin(xyz=(0.0, 0.0, crossbeam_z), rpy=(0.0, pi / 2.0, 0.0)),
        material=powder_coat,
        name="crossbeam",
    )

    for a, b, name in (
        (left_front_foot, left_apex, "left_front_leg"),
        (left_rear_foot, left_apex, "left_rear_leg"),
        (right_front_foot, right_apex, "right_front_leg"),
        (right_rear_foot, right_apex, "right_rear_leg"),
        (left_front_foot, left_rear_foot, "left_ground_spreader"),
        (right_front_foot, right_rear_foot, "right_ground_spreader"),
    ):
        _add_tube(frame, a, b, radius=0.045, material=powder_coat, name=name)

    for x in (-0.69, 0.69):
        for y in (-0.045, 0.045):
            frame.visual(
                Box((0.03, 0.012, 0.14)),
                origin=Origin(xyz=(x, y, crossbeam_underside - 0.045)),
                material=steel_dark,
            )

    hanger = model.part("hanger_yoke")

    upper_pivot_left = (-0.69, 0.0, 0.0)
    upper_pivot_right = (0.69, 0.0, 0.0)
    lower_pivot_left = (-0.69, -0.03, -1.23)
    lower_pivot_right = (0.69, -0.03, -1.23)

    hanger.visual(
        Cylinder(radius=0.022, length=0.08),
        origin=Origin(xyz=upper_pivot_left, rpy=(0.0, pi / 2.0, 0.0)),
        material=bushing,
        name="upper_left_hanger",
    )
    hanger.visual(
        Cylinder(radius=0.022, length=0.08),
        origin=Origin(xyz=upper_pivot_right, rpy=(0.0, pi / 2.0, 0.0)),
        material=bushing,
        name="upper_right_hanger",
    )
    hanger.visual(
        Cylinder(radius=0.024, length=0.08),
        origin=Origin(xyz=lower_pivot_left, rpy=(0.0, pi / 2.0, 0.0)),
        material=bushing,
        name="lower_left_hanger",
    )
    hanger.visual(
        Cylinder(radius=0.024, length=0.08),
        origin=Origin(xyz=lower_pivot_right, rpy=(0.0, pi / 2.0, 0.0)),
        material=bushing,
        name="lower_right_hanger",
    )
    _add_tube(hanger, upper_pivot_left, lower_pivot_left, radius=0.020, material=steel_dark, name="left_side_arm")
    _add_tube(hanger, upper_pivot_right, lower_pivot_right, radius=0.020, material=steel_dark, name="right_side_arm")
    hanger.visual(
        Box((1.42, 0.04, 0.04)),
        origin=Origin(xyz=(0.0, -0.02, -0.10)),
        material=steel_dark,
        name="upper_tie_bar",
    )
    hanger.visual(
        Box((1.42, 0.04, 0.04)),
        origin=Origin(xyz=(0.0, -0.05, -1.10)),
        material=steel_dark,
        name="lower_tie_bar",
    )

    bench = model.part("bench")

    bench.visual(
        Cylinder(radius=0.018, length=0.13),
        origin=Origin(xyz=(-0.585, -0.03, 0.0), rpy=(0.0, pi / 2.0, 0.0)),
        material=bushing,
        name="left_lower_pivot",
    )
    bench.visual(
        Cylinder(radius=0.018, length=0.13),
        origin=Origin(xyz=(0.585, -0.03, 0.0), rpy=(0.0, pi / 2.0, 0.0)),
        material=bushing,
        name="right_lower_pivot",
    )

    for x in (-0.64, 0.64):
        bench.visual(
            Box((0.04, 0.05, 0.62)),
            origin=Origin(xyz=(x, -0.17, 0.03)),
            material=powder_coat,
        )
        bench.visual(
            Box((0.04, 0.05, 0.30)),
            origin=Origin(xyz=(x, 0.21, -0.10)),
            material=powder_coat,
        )
        bench.visual(
            Box((0.12, 0.06, 0.08)),
            origin=Origin(xyz=(0.56 if x > 0.0 else -0.56, -0.03, 0.0)),
            material=powder_coat,
        )
        bench.visual(
            Box((0.04, 0.39, 0.03)),
            origin=Origin(xyz=(x, 0.02, 0.04)),
            material=powder_coat,
        )

    bench.visual(
        Box((1.30, 0.04, 0.04)),
        origin=Origin(xyz=(0.0, -0.15, -0.17)),
        material=powder_coat,
    )
    bench.visual(
        Box((1.30, 0.04, 0.04)),
        origin=Origin(xyz=(0.0, 0.22, -0.12)),
        material=powder_coat,
    )
    bench.visual(
        Box((1.30, 0.04, 0.04)),
        origin=Origin(xyz=(0.0, -0.18, 0.03), rpy=(0.28, 0.0, 0.0)),
        material=powder_coat,
    )
    bench.visual(
        Box((1.30, 0.04, 0.04)),
        origin=Origin(xyz=(0.0, -0.29, 0.32), rpy=(0.28, 0.0, 0.0)),
        material=powder_coat,
    )
    for x in (-0.50, 0.50):
        bench.visual(
            Box((0.04, 0.36, 0.04)),
            origin=Origin(xyz=(x, 0.03, -0.145)),
            material=powder_coat,
        )
        bench.visual(
            Box((0.04, 0.06, 0.36)),
            origin=Origin(xyz=(x, -0.235, 0.18), rpy=(0.28, 0.0, 0.0)),
            material=powder_coat,
        )

    seat_slat_data = (
        ("seat_slat_rear", -0.13, -0.150),
        ("seat_slat_mid_rear", -0.05, -0.145),
        ("seat_slat_mid", 0.03, -0.140),
        ("seat_slat_mid_front", 0.11, -0.133),
        ("seat_slat_front", 0.19, -0.125),
    )
    for name, y, z in seat_slat_data:
        bench.visual(
            Box((1.30, 0.07, 0.018)),
            origin=Origin(xyz=(0.0, y, z)),
            material=wood,
            name=name,
        )

    back_slat_data = (
        ("back_slat_low", -0.18, 0.07),
        ("back_slat_mid_low", -0.205, 0.145),
        ("back_slat_mid_high", -0.23, 0.22),
        ("back_slat_top", -0.255, 0.295),
    )
    for name, y, z in back_slat_data:
        bench.visual(
            Box((1.30, 0.018, 0.075)),
            origin=Origin(xyz=(0.0, y, z), rpy=(0.28, 0.0, 0.0)),
            material=wood,
            name=name,
        )

    model.articulation(
        "frame_to_hanger",
        ArticulationType.REVOLUTE,
        parent=frame,
        child=hanger,
        origin=Origin(xyz=(0.0, 0.0, upper_pivot_z)),
        axis=(1.0, 0.0, 0.0),
        motion_limits=MotionLimits(effort=120.0, velocity=1.2, lower=-0.55, upper=0.55),
    )
    model.articulation(
        "hanger_to_bench",
        ArticulationType.REVOLUTE,
        parent=hanger,
        child=bench,
        origin=Origin(xyz=(0.0, -0.03, -1.23)),
        axis=(1.0, 0.0, 0.0),
        motion_limits=MotionLimits(effort=60.0, velocity=1.5, lower=-0.35, upper=0.35),
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
    hanger = object_model.get_part("hanger_yoke")
    bench = object_model.get_part("bench")
    upper_joint = object_model.get_articulation("frame_to_hanger")
    lower_joint = object_model.get_articulation("hanger_to_bench")

    upper_limits = upper_joint.motion_limits
    lower_limits = lower_joint.motion_limits
    ctx.check(
        "upper hanger pivots provide wide swing arc",
        upper_limits is not None
        and upper_limits.lower is not None
        and upper_limits.upper is not None
        and upper_limits.lower <= -0.5
        and upper_limits.upper >= 0.5,
        details=f"upper_limits={upper_limits}",
    )
    ctx.check(
        "lower seat pivots provide modest trim arc",
        lower_limits is not None
        and lower_limits.lower is not None
        and lower_limits.upper is not None
        and lower_limits.lower <= -0.3
        and lower_limits.upper >= 0.3,
        details=f"lower_limits={lower_limits}",
    )

    ctx.expect_gap(
        frame,
        bench,
        axis="z",
        positive_elem="crossbeam",
        min_gap=0.82,
        name="bench hangs well below the crossbeam",
    )
    ctx.expect_within(
        bench,
        hanger,
        axes="x",
        margin=0.12,
        name="bench stays laterally between the side suspension arms",
    )

    rest_pos = ctx.part_world_position(bench)
    with ctx.pose({upper_joint: 0.45}):
        swung_pos = ctx.part_world_position(bench)
    ctx.check(
        "positive upper pivot swings the bench forward",
        rest_pos is not None and swung_pos is not None and swung_pos[1] > rest_pos[1] + 0.45,
        details=f"rest={rest_pos}, swung={swung_pos}",
    )

    rest_front = ctx.part_element_world_aabb(bench, elem="seat_slat_front")
    rest_rear = ctx.part_element_world_aabb(bench, elem="seat_slat_rear")
    with ctx.pose({lower_joint: 0.22}):
        pitched_front = ctx.part_element_world_aabb(bench, elem="seat_slat_front")
        pitched_rear = ctx.part_element_world_aabb(bench, elem="seat_slat_rear")

    def _center_z(aabb):
        if aabb is None:
            return None
        return (aabb[0][2] + aabb[1][2]) * 0.5

    rest_pitch = None
    pitched_pitch = None
    if rest_front is not None and rest_rear is not None:
        rest_pitch = _center_z(rest_front) - _center_z(rest_rear)
    if pitched_front is not None and pitched_rear is not None:
        pitched_pitch = _center_z(pitched_front) - _center_z(pitched_rear)

    ctx.check(
        "positive lower pivots raise the bench front edge",
        rest_pitch is not None and pitched_pitch is not None and pitched_pitch > rest_pitch + 0.03,
        details=f"rest_pitch_delta={rest_pitch}, pitched_delta={pitched_pitch}",
    )

    return ctx.report()


# >>> USER_CODE_END

object_model = build_object_model()
