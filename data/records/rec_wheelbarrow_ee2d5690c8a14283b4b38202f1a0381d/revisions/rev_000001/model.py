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
    MotionLimits,
    Origin,
    TestContext,
    TestReport,
    mesh_from_geometry,
    tube_from_spline_points,
)


HALF_PI = math.pi / 2.0


def _mesh(name: str, geometry):
    return mesh_from_geometry(geometry, name)


def _midpoint(
    a: tuple[float, float, float], b: tuple[float, float, float]
) -> tuple[float, float, float]:
    return ((a[0] + b[0]) * 0.5, (a[1] + b[1]) * 0.5, (a[2] + b[2]) * 0.5)


def _distance(a: tuple[float, float, float], b: tuple[float, float, float]) -> float:
    return math.sqrt((b[0] - a[0]) ** 2 + (b[1] - a[1]) ** 2 + (b[2] - a[2]) ** 2)


def _rpy_for_cylinder(
    a: tuple[float, float, float], b: tuple[float, float, float]
) -> tuple[float, float, float]:
    dx = b[0] - a[0]
    dy = b[1] - a[1]
    dz = b[2] - a[2]
    length_xy = math.hypot(dx, dy)
    yaw = math.atan2(dy, dx)
    pitch = math.atan2(length_xy, dz)
    return (0.0, pitch, yaw)


def _add_member(part, a, b, *, radius: float, material, name: str | None = None) -> None:
    part.visual(
        Cylinder(radius=radius, length=_distance(a, b)),
        origin=Origin(xyz=_midpoint(a, b), rpy=_rpy_for_cylinder(a, b)),
        material=material,
        name=name,
    )


def _add_tube_path(
    part,
    name: str,
    points: list[tuple[float, float, float]],
    *,
    radius: float,
    material,
    samples_per_segment: int = 12,
    radial_segments: int = 18,
) -> None:
    part.visual(
        _mesh(
            name,
            tube_from_spline_points(
                points,
                radius=radius,
                samples_per_segment=samples_per_segment,
                radial_segments=radial_segments,
            ),
        ),
        material=material,
        name=name,
    )


def _bolt_rpy(axis: str) -> tuple[float, float, float]:
    if axis == "x":
        return (0.0, HALF_PI, 0.0)
    if axis == "y":
        return (HALF_PI, 0.0, 0.0)
    return (0.0, 0.0, 0.0)


def _add_bolt_head(
    part,
    center: tuple[float, float, float],
    *,
    axis: str = "z",
    radius: float = 0.008,
    length: float = 0.006,
    material,
    name: str | None = None,
) -> None:
    part.visual(
        Cylinder(radius=radius, length=length),
        origin=Origin(xyz=center, rpy=_bolt_rpy(axis)),
        material=material,
        name=name,
    )


def _add_mount_bolt_pattern(
    part,
    *,
    xs: tuple[float, float],
    ys: tuple[float, float],
    z: float,
    axis: str,
    material,
    prefix: str,
) -> None:
    index = 0
    for x in xs:
        for y in ys:
            _add_bolt_head(
                part,
                (x, y, z),
                axis=axis,
                radius=0.0075,
                length=0.006,
                material=material,
                name=f"{prefix}_{index}",
            )
            index += 1


def build_object_model() -> ArticulatedObject:
    model = ArticulatedObject(name="industrial_safety_wheelbarrow")

    tray_steel = model.material("tray_steel", rgba=(0.34, 0.37, 0.40, 1.0))
    frame_steel = model.material("frame_steel", rgba=(0.20, 0.21, 0.22, 1.0))
    zinc = model.material("zinc", rgba=(0.74, 0.76, 0.78, 1.0))
    rubber = model.material("rubber", rgba=(0.06, 0.06, 0.06, 1.0))
    safety_yellow = model.material("safety_yellow", rgba=(0.88, 0.77, 0.16, 1.0))
    safety_red = model.material("safety_red", rgba=(0.75, 0.14, 0.10, 1.0))
    grip_black = model.material("grip_black", rgba=(0.08, 0.08, 0.08, 1.0))

    tray = model.part("tray")
    tray.visual(
        Box((0.76, 0.54, 0.008)),
        origin=Origin(xyz=(-0.06, 0.0, 0.460)),
        material=tray_steel,
        name="floor",
    )
    tray.visual(
        Box((0.80, 0.008, 0.19)),
        origin=Origin(xyz=(-0.06, 0.255, 0.555), rpy=(0.0, 0.0, -0.10)),
        material=tray_steel,
        name="left_wall",
    )
    tray.visual(
        Box((0.80, 0.008, 0.19)),
        origin=Origin(xyz=(-0.06, -0.255, 0.555), rpy=(0.0, 0.0, 0.10)),
        material=tray_steel,
        name="right_wall",
    )
    tray.visual(
        Box((0.010, 0.58, 0.18)),
        origin=Origin(xyz=(-0.445, 0.0, 0.550)),
        material=tray_steel,
        name="rear_wall",
    )
    tray.visual(
        Box((0.010, 0.42, 0.19)),
        origin=Origin(xyz=(0.325, 0.0, 0.555)),
        material=tray_steel,
        name="front_wall",
    )
    tray.visual(
        Box((0.82, 0.028, 0.028)),
        origin=Origin(xyz=(-0.05, 0.258, 0.650), rpy=(0.0, 0.0, -0.10)),
        material=tray_steel,
        name="left_rim",
    )
    tray.visual(
        Box((0.82, 0.028, 0.028)),
        origin=Origin(xyz=(-0.05, -0.258, 0.650), rpy=(0.0, 0.0, 0.10)),
        material=tray_steel,
        name="right_rim",
    )
    tray.visual(
        Box((0.028, 0.60, 0.028)),
        origin=Origin(xyz=(-0.445, 0.0, 0.646)),
        material=tray_steel,
        name="rear_rim",
    )
    tray.visual(
        Box((0.028, 0.44, 0.028)),
        origin=Origin(xyz=(0.325, 0.0, 0.646)),
        material=tray_steel,
        name="front_rim",
    )
    tray.visual(
        Box((0.09, 0.44, 0.06)),
        origin=Origin(xyz=(0.290, 0.0, 0.620)),
        material=tray_steel,
        name="front_lip",
    )
    tray.visual(
        Box((0.36, 0.09, 0.05)),
        origin=Origin(xyz=(-0.18, 0.0, 0.431)),
        material=tray_steel,
        name="center_keel",
    )
    tray.visual(
        Box((0.22, 0.18, 0.012)),
        origin=Origin(xyz=(0.20, 0.0, 0.470)),
        material=tray_steel,
        name="front_inner_plate",
    )
    tray.visual(
        Box((0.26, 0.08, 0.012)),
        origin=Origin(xyz=(-0.12, 0.18, 0.470)),
        material=tray_steel,
        name="left_inner_plate",
    )
    tray.visual(
        Box((0.26, 0.08, 0.012)),
        origin=Origin(xyz=(-0.12, -0.18, 0.470)),
        material=tray_steel,
        name="right_inner_plate",
    )
    tray.visual(
        Box((0.14, 0.06, 0.012)),
        origin=Origin(xyz=(-0.28, 0.17, 0.470)),
        material=tray_steel,
        name="left_rear_inner_plate",
    )
    tray.visual(
        Box((0.14, 0.06, 0.012)),
        origin=Origin(xyz=(-0.28, -0.17, 0.470)),
        material=tray_steel,
        name="right_rear_inner_plate",
    )
    _add_mount_bolt_pattern(
        tray,
        xs=(-0.02, 0.10),
        ys=(0.21, 0.25),
        z=0.467,
        axis="z",
        material=zinc,
        prefix="left_handle_bolt",
    )
    _add_mount_bolt_pattern(
        tray,
        xs=(-0.02, 0.10),
        ys=(-0.25, -0.21),
        z=0.467,
        axis="z",
        material=zinc,
        prefix="right_handle_bolt",
    )
    _add_mount_bolt_pattern(
        tray,
        xs=(0.12, 0.28),
        ys=(-0.06, 0.06),
        z=0.467,
        axis="z",
        material=zinc,
        prefix="fork_bolt",
    )
    _add_mount_bolt_pattern(
        tray,
        xs=(-0.31, -0.25),
        ys=(0.15, 0.19),
        z=0.467,
        axis="z",
        material=zinc,
        prefix="left_leg_bolt",
    )
    _add_mount_bolt_pattern(
        tray,
        xs=(-0.31, -0.25),
        ys=(-0.19, -0.15),
        z=0.467,
        axis="z",
        material=zinc,
        prefix="right_leg_bolt",
    )

    handle_frame = model.part("handle_frame")
    _add_tube_path(
        handle_frame,
        "left_handle_tube",
        [
            (-0.66, 0.33, 0.094),
            (-0.52, 0.32, 0.030),
            (-0.34, 0.30, -0.050),
            (-0.12, 0.27, -0.105),
            (0.10, 0.245, -0.085),
            (0.24, 0.23, -0.036),
        ],
        radius=0.023,
        material=frame_steel,
        samples_per_segment=14,
    )
    _add_tube_path(
        handle_frame,
        "right_handle_tube",
        [
            (-0.66, -0.33, 0.094),
            (-0.52, -0.32, 0.030),
            (-0.34, -0.30, -0.050),
            (-0.12, -0.27, -0.105),
            (0.10, -0.245, -0.085),
            (0.24, -0.23, -0.036),
        ],
        radius=0.023,
        material=frame_steel,
        samples_per_segment=14,
    )
    handle_frame.visual(
        Box((0.22, 0.08, 0.064)),
        origin=Origin(xyz=(0.12, 0.23, -0.032)),
        material=frame_steel,
        name="left_mount_pad",
    )
    handle_frame.visual(
        Box((0.22, 0.08, 0.064)),
        origin=Origin(xyz=(0.12, -0.23, -0.032)),
        material=frame_steel,
        name="right_mount_pad",
    )
    handle_frame.visual(
        Box((0.05, 0.62, 0.05)),
        origin=Origin(xyz=(-0.54, 0.0, 0.072)),
        material=frame_steel,
        name="rear_crossbar",
    )
    handle_frame.visual(
        Box((0.05, 0.48, 0.05)),
        origin=Origin(xyz=(0.12, 0.0, -0.100)),
        material=frame_steel,
        name="front_crossbar",
    )
    _add_tube_path(
        handle_frame,
        "left_guard_loop",
        [
            (-0.66, 0.33, 0.094),
            (-0.72, 0.40, 0.132),
            (-0.62, 0.44, 0.172),
            (-0.50, 0.40, 0.132),
            (-0.46, 0.32, 0.060),
        ],
        radius=0.014,
        material=frame_steel,
        samples_per_segment=10,
        radial_segments=16,
    )
    _add_tube_path(
        handle_frame,
        "right_guard_loop",
        [
            (-0.66, -0.33, 0.094),
            (-0.72, -0.40, 0.132),
            (-0.62, -0.44, 0.172),
            (-0.50, -0.40, 0.132),
            (-0.46, -0.32, 0.060),
        ],
        radius=0.014,
        material=frame_steel,
        samples_per_segment=10,
        radial_segments=16,
    )
    handle_frame.visual(
        Cylinder(radius=0.028, length=0.14),
        origin=Origin(xyz=(-0.70, 0.33, 0.094), rpy=(0.0, HALF_PI, 0.0)),
        material=grip_black,
        name="left_grip",
    )
    handle_frame.visual(
        Cylinder(radius=0.028, length=0.14),
        origin=Origin(xyz=(-0.70, -0.33, 0.094), rpy=(0.0, HALF_PI, 0.0)),
        material=grip_black,
        name="right_grip",
    )
    _add_mount_bolt_pattern(
        handle_frame,
        xs=(0.04, 0.16),
        ys=(0.21, 0.25),
        z=-0.003,
        axis="z",
        material=zinc,
        prefix="left_handle_nut",
    )
    _add_mount_bolt_pattern(
        handle_frame,
        xs=(0.04, 0.16),
        ys=(-0.25, -0.21),
        z=-0.003,
        axis="z",
        material=zinc,
        prefix="right_handle_nut",
    )

    rear_leg_frame = model.part("rear_leg_frame")
    rear_leg_frame.visual(
        Box((0.12, 0.05, 0.060)),
        origin=Origin(xyz=(0.00, 0.17, -0.030)),
        material=frame_steel,
        name="left_bracket",
    )
    rear_leg_frame.visual(
        Box((0.12, 0.05, 0.060)),
        origin=Origin(xyz=(0.00, -0.17, -0.030)),
        material=frame_steel,
        name="right_bracket",
    )
    _add_member(
        rear_leg_frame,
        (-0.02, 0.17, -0.035),
        (-0.22, 0.18, -0.410),
        radius=0.020,
        material=frame_steel,
        name="left_leg",
    )
    _add_member(
        rear_leg_frame,
        (-0.02, -0.17, -0.035),
        (-0.22, -0.18, -0.410),
        radius=0.020,
        material=frame_steel,
        name="right_leg",
    )
    _add_member(
        rear_leg_frame,
        (0.05, 0.13, -0.090),
        (-0.15, 0.18, -0.320),
        radius=0.016,
        material=frame_steel,
        name="left_brace",
    )
    _add_member(
        rear_leg_frame,
        (0.05, -0.13, -0.090),
        (-0.15, -0.18, -0.320),
        radius=0.016,
        material=frame_steel,
        name="right_brace",
    )
    rear_leg_frame.visual(
        Box((0.12, 0.05, 0.040)),
        origin=Origin(xyz=(-0.22, 0.18, -0.436)),
        material=frame_steel,
        name="left_foot",
    )
    rear_leg_frame.visual(
        Box((0.12, 0.05, 0.040)),
        origin=Origin(xyz=(-0.22, -0.18, -0.436)),
        material=frame_steel,
        name="right_foot",
    )
    rear_leg_frame.visual(
        Box((0.05, 0.38, 0.04)),
        origin=Origin(xyz=(-0.19, 0.0, -0.340)),
        material=frame_steel,
        name="cross_brace",
    )
    _add_member(
        rear_leg_frame,
        (-0.22, -0.20, -0.340),
        (-0.22, 0.20, -0.340),
        radius=0.016,
        material=safety_red,
        name="lockout_bar",
    )
    rear_leg_frame.visual(
        Box((0.06, 0.12, 0.03)),
        origin=Origin(xyz=(-0.24, 0.0, -0.340)),
        material=safety_red,
        name="pedal_plate",
    )
    rear_leg_frame.visual(
        Box((0.12, 0.42, 0.03)),
        origin=Origin(xyz=(-0.24, 0.0, -0.410)),
        material=frame_steel,
        name="anti_tip_bar",
    )
    rear_leg_frame.visual(
        Box((0.06, 0.04, 0.020)),
        origin=Origin(xyz=(0.02, 0.16, -0.010)),
        material=safety_yellow,
        name="left_stop",
    )
    rear_leg_frame.visual(
        Box((0.06, 0.04, 0.020)),
        origin=Origin(xyz=(0.02, -0.16, -0.010)),
        material=safety_yellow,
        name="right_stop",
    )
    _add_mount_bolt_pattern(
        rear_leg_frame,
        xs=(-0.03, 0.03),
        ys=(0.15, 0.19),
        z=-0.003,
        axis="z",
        material=zinc,
        prefix="left_leg_nut",
    )
    _add_mount_bolt_pattern(
        rear_leg_frame,
        xs=(-0.03, 0.03),
        ys=(-0.19, -0.15),
        z=-0.003,
        axis="z",
        material=zinc,
        prefix="right_leg_nut",
    )

    fork_assembly = model.part("fork_assembly")
    fork_assembly.visual(
        Box((0.30, 0.18, 0.032)),
        origin=Origin(xyz=(0.02, 0.0, -0.016)),
        material=frame_steel,
        name="mount_plate",
    )
    fork_assembly.visual(
        Box((0.12, 0.16, 0.080)),
        origin=Origin(xyz=(0.05, 0.0, -0.072)),
        material=frame_steel,
        name="rear_web",
    )
    fork_assembly.visual(
        Box((0.18, 0.22, 0.024)),
        origin=Origin(xyz=(0.10, 0.0, -0.050)),
        material=frame_steel,
        name="crown",
    )
    fork_assembly.visual(
        Box((0.18, 0.012, 0.24)),
        origin=Origin(xyz=(0.35, 0.095, -0.208)),
        material=frame_steel,
        name="left_plate",
    )
    fork_assembly.visual(
        Box((0.18, 0.012, 0.24)),
        origin=Origin(xyz=(0.35, -0.095, -0.208)),
        material=frame_steel,
        name="right_plate",
    )
    _add_member(
        fork_assembly,
        (0.14, 0.080, -0.048),
        (0.28, 0.094, -0.106),
        radius=0.012,
        material=frame_steel,
        name="left_guard_strap",
    )
    _add_member(
        fork_assembly,
        (0.14, -0.080, -0.048),
        (0.28, -0.094, -0.106),
        radius=0.012,
        material=frame_steel,
        name="right_guard_strap",
    )
    fork_assembly.visual(
        Box((0.22, 0.24, 0.020)),
        origin=Origin(xyz=(0.34, 0.0, 0.032)),
        material=safety_yellow,
        name="guard_top",
    )
    fork_assembly.visual(
        Box((0.18, 0.012, 0.10)),
        origin=Origin(xyz=(0.34, 0.110, -0.010)),
        material=safety_yellow,
        name="left_guard_skirt",
    )
    fork_assembly.visual(
        Box((0.18, 0.012, 0.10)),
        origin=Origin(xyz=(0.34, -0.110, -0.010)),
        material=safety_yellow,
        name="right_guard_skirt",
    )
    _add_member(
        fork_assembly,
        (0.19, 0.103, -0.038),
        (0.23, 0.103, 0.022),
        radius=0.008,
        material=frame_steel,
        name="left_guard_post",
    )
    _add_member(
        fork_assembly,
        (0.19, -0.103, -0.038),
        (0.23, -0.103, 0.022),
        radius=0.008,
        material=frame_steel,
        name="right_guard_post",
    )
    fork_assembly.visual(
        Cylinder(radius=0.030, length=0.018),
        origin=Origin(xyz=(0.44, 0.108, -0.236), rpy=(HALF_PI, 0.0, 0.0)),
        material=zinc,
        name="left_axle_cap",
    )
    fork_assembly.visual(
        Cylinder(radius=0.030, length=0.018),
        origin=Origin(xyz=(0.44, -0.108, -0.236), rpy=(HALF_PI, 0.0, 0.0)),
        material=zinc,
        name="right_axle_cap",
    )
    fork_assembly.visual(
        Box((0.04, 0.010, 0.08)),
        origin=Origin(xyz=(0.44, 0.104, -0.180)),
        material=safety_red,
        name="left_lock_plate",
    )
    fork_assembly.visual(
        Box((0.04, 0.010, 0.08)),
        origin=Origin(xyz=(0.44, -0.104, -0.180)),
        material=safety_red,
        name="right_lock_plate",
    )
    fork_assembly.visual(
        Box((0.06, 0.05, 0.04)),
        origin=Origin(xyz=(0.00, 0.05, -0.020)),
        material=safety_yellow,
        name="left_bump_stop",
    )
    fork_assembly.visual(
        Box((0.06, 0.05, 0.04)),
        origin=Origin(xyz=(0.00, -0.05, -0.020)),
        material=safety_yellow,
        name="right_bump_stop",
    )
    _add_mount_bolt_pattern(
        fork_assembly,
        xs=(-0.06, 0.06),
        ys=(-0.05, 0.05),
        z=-0.003,
        axis="z",
        material=zinc,
        prefix="fork_nut",
    )

    wheel = model.part("wheel")
    wheel.visual(
        Cylinder(radius=0.22, length=0.132),
        origin=Origin(rpy=(HALF_PI, 0.0, 0.0)),
        material=rubber,
        name="tire",
    )
    wheel.visual(
        Cylinder(radius=0.175, length=0.008),
        origin=Origin(xyz=(0.0, 0.032, 0.0), rpy=(HALF_PI, 0.0, 0.0)),
        material=zinc,
        name="left_disc",
    )
    wheel.visual(
        Cylinder(radius=0.175, length=0.008),
        origin=Origin(xyz=(0.0, -0.032, 0.0), rpy=(HALF_PI, 0.0, 0.0)),
        material=zinc,
        name="right_disc",
    )
    wheel.visual(
        Cylinder(radius=0.14, length=0.072),
        origin=Origin(rpy=(HALF_PI, 0.0, 0.0)),
        material=zinc,
        name="rim",
    )
    wheel.visual(
        Cylinder(radius=0.068, length=0.178),
        origin=Origin(rpy=(HALF_PI, 0.0, 0.0)),
        material=frame_steel,
        name="hub",
    )
    wheel.visual(
        Cylinder(radius=0.038, length=0.050),
        origin=Origin(rpy=(HALF_PI, 0.0, 0.0)),
        material=zinc,
        name="center_cap",
    )

    model.articulation(
        "tray_to_handles",
        ArticulationType.FIXED,
        parent=tray,
        child=handle_frame,
        origin=Origin(xyz=(-0.12, 0.0, 0.456)),
    )
    model.articulation(
        "tray_to_rear_legs",
        ArticulationType.FIXED,
        parent=tray,
        child=rear_leg_frame,
        origin=Origin(xyz=(-0.28, 0.0, 0.456)),
    )
    model.articulation(
        "tray_to_fork",
        ArticulationType.FIXED,
        parent=tray,
        child=fork_assembly,
        origin=Origin(xyz=(0.22, 0.0, 0.456)),
    )
    model.articulation(
        "fork_to_wheel",
        ArticulationType.CONTINUOUS,
        parent=fork_assembly,
        child=wheel,
        origin=Origin(xyz=(0.44, 0.0, -0.236)),
        axis=(0.0, 1.0, 0.0),
        motion_limits=MotionLimits(effort=45.0, velocity=18.0),
    )

    return model


def run_tests() -> TestReport:
    ctx = TestContext(object_model)
    tray = object_model.get_part("tray")
    handle_frame = object_model.get_part("handle_frame")
    rear_leg_frame = object_model.get_part("rear_leg_frame")
    fork_assembly = object_model.get_part("fork_assembly")
    wheel = object_model.get_part("wheel")
    wheel_spin = object_model.get_articulation("fork_to_wheel")

    ctx.check_model_valid()
    ctx.check_mesh_assets_ready()

    # Preferred default QC stack:
    # 1) likely-failure grounded-component floating check for disconnected part groups
    ctx.fail_if_isolated_parts()
    # 2) noisier warning-tier sensor for same-part disconnected geometry islands
    ctx.warn_if_part_contains_disconnected_geometry_islands()
    # 3) likely-failure rest-pose part-to-part overlap backstop for real 3D interpenetration
    # This is not an "inside / nested / footprint overlap" check.
    # Investigate all three. Warning-tier signals are not free passes.
    # Use `ctx.allow_overlap(...)` only for true intended penetration.
    # If parts are nested but should remain clear, prove that with exact
    # `expect_contact(...)`, `expect_gap(...)`, `expect_overlap(...)`, or
    # `expect_within(...)` checks instead.
    ctx.fail_if_parts_overlap_in_current_pose()

    ctx.expect_contact(tray, handle_frame, elem_a="floor", elem_b="left_mount_pad")
    ctx.expect_contact(tray, handle_frame, elem_a="floor", elem_b="right_mount_pad")
    ctx.expect_contact(tray, rear_leg_frame, elem_a="floor", elem_b="left_bracket")
    ctx.expect_contact(tray, rear_leg_frame, elem_a="floor", elem_b="right_bracket")
    ctx.expect_contact(tray, fork_assembly, elem_a="floor", elem_b="mount_plate")
    ctx.expect_contact(fork_assembly, wheel, elem_a="left_plate", elem_b="hub")
    ctx.expect_contact(fork_assembly, wheel, elem_a="right_plate", elem_b="hub")

    ctx.expect_gap(
        fork_assembly,
        wheel,
        axis="y",
        positive_elem="left_plate",
        negative_elem="tire",
        min_gap=0.018,
        max_gap=0.030,
        name="left_fork_tire_clearance",
    )
    ctx.expect_gap(
        wheel,
        fork_assembly,
        axis="y",
        positive_elem="tire",
        negative_elem="right_plate",
        min_gap=0.018,
        max_gap=0.030,
        name="right_fork_tire_clearance",
    )
    ctx.expect_gap(
        tray,
        wheel,
        axis="z",
        positive_elem="floor",
        negative_elem="tire",
        min_gap=0.012,
        max_gap=0.030,
        name="wheel_to_tray_clearance",
    )
    ctx.expect_gap(
        fork_assembly,
        wheel,
        axis="z",
        positive_elem="guard_top",
        negative_elem="tire",
        min_gap=0.030,
        max_gap=0.050,
        name="guard_to_tire_clearance",
    )
    ctx.expect_origin_gap(
        wheel,
        rear_leg_frame,
        axis="x",
        min_gap=0.75,
        name="wheelbase_is_stable",
    )

    limits = wheel_spin.motion_limits
    ctx.check(
        "wheel_joint_axis",
        tuple(round(value, 6) for value in wheel_spin.axis) == (0.0, 1.0, 0.0),
        details=f"unexpected wheel axis: {wheel_spin.axis}",
    )
    ctx.check(
        "wheel_joint_is_continuous",
        wheel_spin.articulation_type == ArticulationType.CONTINUOUS,
        details=f"unexpected wheel joint type: {wheel_spin.articulation_type}",
    )
    ctx.check(
        "wheel_joint_limits_are_continuous",
        limits is not None and limits.lower is None and limits.upper is None,
        details=f"wheel joint limits should be continuous-style, got {limits}",
    )

    return ctx.report()


# >>> USER_CODE_END

object_model = build_object_model()
