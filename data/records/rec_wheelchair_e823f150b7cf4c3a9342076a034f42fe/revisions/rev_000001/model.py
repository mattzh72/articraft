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
    TorusGeometry,
    mesh_from_geometry,
    wire_from_points,
)


def _save_mesh(name: str, geometry):
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


def _add_member(
    part,
    a: tuple[float, float, float],
    b: tuple[float, float, float],
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


def _build_drive_wheel(part, *, prefix: str, outboard_sign: float, rim_material, tire_material, handrim_material) -> None:
    wheel_spin = Origin(rpy=(math.pi / 2.0, 0.0, 0.0))
    part.visual(
        _save_mesh(
            f"{prefix}_tire",
            TorusGeometry(radius=0.282, tube=0.024, radial_segments=18, tubular_segments=72),
        ),
        origin=wheel_spin,
        material=tire_material,
        name="tire",
    )
    part.visual(
        _save_mesh(
            f"{prefix}_rim",
            TorusGeometry(radius=0.255, tube=0.012, radial_segments=16, tubular_segments=64),
        ),
        origin=wheel_spin,
        material=rim_material,
        name="rim",
    )
    part.visual(
        Cylinder(radius=0.032, length=0.020),
        origin=Origin(rpy=(math.pi / 2.0, 0.0, 0.0)),
        material=rim_material,
        name="hub_barrel",
    )
    part.visual(
        Cylinder(radius=0.050, length=0.006),
        origin=Origin(xyz=(0.0, outboard_sign * 0.012, 0.0), rpy=(math.pi / 2.0, 0.0, 0.0)),
        material=rim_material,
        name="outboard_flange",
    )
    part.visual(
        Cylinder(radius=0.022, length=0.008),
        origin=Origin(xyz=(0.0, -outboard_sign * 0.014, 0.0), rpy=(math.pi / 2.0, 0.0, 0.0)),
        material=rim_material,
        name="inboard_cap",
    )
    part.visual(
        _save_mesh(
            f"{prefix}_handrim",
            TorusGeometry(radius=0.324, tube=0.005, radial_segments=12, tubular_segments=64),
        ),
        origin=Origin(xyz=(0.0, outboard_sign * 0.032, 0.0), rpy=(math.pi / 2.0, 0.0, 0.0)),
        material=handrim_material,
        name="handrim",
    )

    for index in range(8):
        angle = (2.0 * math.pi * index) / 8.0
        inner = (0.032 * math.cos(angle), 0.0, 0.032 * math.sin(angle))
        outer = (0.244 * math.cos(angle), 0.0, 0.244 * math.sin(angle))
        _add_member(part, inner, outer, radius=0.0045, material=rim_material)

    for angle in (math.pi / 4.0, 3.0 * math.pi / 4.0, 5.0 * math.pi / 4.0, 7.0 * math.pi / 4.0):
        rim_pickup = (0.262 * math.cos(angle), 0.0, 0.262 * math.sin(angle))
        handrim_pickup = (
            0.319 * math.cos(angle),
            outboard_sign * 0.032,
            0.319 * math.sin(angle),
        )
        _add_member(part, rim_pickup, handrim_pickup, radius=0.003, material=handrim_material)


def _build_caster_wheel(part, *, prefix: str, rim_material, tire_material) -> None:
    wheel_spin = Origin(rpy=(math.pi / 2.0, 0.0, 0.0))
    part.visual(
        Cylinder(radius=0.062, length=0.022),
        origin=wheel_spin,
        material=tire_material,
        name="tire",
    )
    part.visual(
        Cylinder(radius=0.049, length=0.020),
        origin=Origin(rpy=(math.pi / 2.0, 0.0, 0.0)),
        material=rim_material,
        name="rim",
    )
    part.visual(
        Cylinder(radius=0.020, length=0.028),
        origin=Origin(rpy=(math.pi / 2.0, 0.0, 0.0)),
        material=rim_material,
        name="hub",
    )
    part.visual(
        Cylinder(radius=0.028, length=0.004),
        origin=Origin(xyz=(0.0, 0.009, 0.0), rpy=(math.pi / 2.0, 0.0, 0.0)),
        material=rim_material,
        name="hub_cap_pos",
    )
    part.visual(
        Cylinder(radius=0.028, length=0.004),
        origin=Origin(xyz=(0.0, -0.009, 0.0), rpy=(math.pi / 2.0, 0.0, 0.0)),
        material=rim_material,
        name="hub_cap_neg",
    )


def _build_caster_fork(part, *, material) -> None:
    part.visual(
        Cylinder(radius=0.011, length=0.050),
        origin=Origin(xyz=(0.0, 0.0, -0.025)),
        material=material,
        name="stem",
    )
    part.visual(
        Cylinder(radius=0.020, length=0.010),
        origin=Origin(xyz=(0.0, 0.0, -0.005)),
        material=material,
        name="top_washer",
    )
    part.visual(
        Box((0.032, 0.040, 0.018)),
        origin=Origin(xyz=(-0.018, 0.0, -0.045)),
        material=material,
        name="fork_crown",
    )
    for label, side_y in (("pos", 0.014), ("neg", -0.014)):
        part.visual(
            Box((0.022, 0.006, 0.024)),
            origin=Origin(xyz=(-0.036, side_y, -0.060)),
            material=material,
            name=f"upper_gusset_{label}",
        )
        part.visual(
            Box((0.068, 0.006, 0.072)),
            origin=Origin(xyz=(-0.074, side_y, -0.091)),
            material=material,
            name=f"cheek_{label}",
        )


def build_object_model() -> ArticulatedObject:
    model = ArticulatedObject(name="mass_market_wheelchair")

    frame_metal = model.material("frame_metal", rgba=(0.68, 0.71, 0.74, 1.0))
    painted_steel = model.material("painted_steel", rgba=(0.24, 0.26, 0.29, 1.0))
    molded_black = model.material("molded_black", rgba=(0.11, 0.11, 0.12, 1.0))
    rubber = model.material("rubber", rgba=(0.04, 0.04, 0.04, 1.0))
    handrim_metal = model.material("handrim_metal", rgba=(0.82, 0.84, 0.86, 1.0))

    frame = model.part("frame")
    frame.inertial = Inertial.from_geometry(
        Box((0.94, 0.66, 0.96)),
        mass=16.0,
        origin=Origin(xyz=(0.08, 0.0, 0.48)),
    )

    rail_radius = 0.014
    for side_name, side_y in (("left", 0.22), ("right", -0.22)):
        side_rail = wire_from_points(
            [
                (-0.185, side_y, 0.92),
                (-0.185, side_y, 0.50),
                (0.160, side_y, 0.50),
                (0.240, side_y, 0.385),
                (0.285, side_y, 0.195),
            ],
            radius=rail_radius,
            radial_segments=16,
            cap_ends=True,
            corner_mode="fillet",
            corner_radius=0.055,
            corner_segments=10,
        )
        frame.visual(
            _save_mesh(f"frame_{side_name}_rail", side_rail),
            material=frame_metal,
            name=f"{side_name}_side_rail",
        )

    _add_member(
        frame,
        (-0.180, -0.22, 0.785),
        (-0.180, 0.22, 0.785),
        radius=0.012,
        material=frame_metal,
        name="back_crossbar",
    )
    _add_member(
        frame,
        (0.120, -0.22, 0.500),
        (0.120, 0.22, 0.500),
        radius=0.012,
        material=frame_metal,
        name="seat_crossbar",
    )
    _add_member(
        frame,
        (-0.145, -0.235, 0.305),
        (-0.145, 0.235, 0.305),
        radius=0.010,
        material=painted_steel,
        name="axle_spreader",
    )

    for side_name, side_sign in (("left", 1.0), ("right", -1.0)):
        side_y = 0.233 * side_sign
        boss_y = 0.291 * side_sign
        rail_y = 0.220 * side_sign
        frame.visual(
            Box((0.060, 0.020, 0.240)),
            origin=Origin(xyz=(-0.160, side_y, 0.310)),
            material=painted_steel,
            name=f"{side_name}_axle_plate",
        )
        frame.visual(
            Box((0.022, 0.058, 0.052)),
            origin=Origin(xyz=(-0.145, 0.262 * side_sign, 0.305)),
            material=painted_steel,
            name=f"{side_name}_axle_bridge",
        )
        frame.visual(
            Cylinder(radius=0.012, length=0.012),
            origin=Origin(xyz=(-0.145, boss_y, 0.305), rpy=(math.pi / 2.0, 0.0, 0.0)),
            material=painted_steel,
            name=f"{side_name}_axle_boss",
        )
        _add_member(
            frame,
            (-0.185, rail_y, 0.500),
            (-0.160, side_y, 0.430),
            radius=0.011,
            material=frame_metal,
            name=f"{side_name}_axle_strut",
        )
        frame.visual(
            Cylinder(radius=0.018, length=0.050),
            origin=Origin(xyz=(0.285, 0.22 * side_sign, 0.170)),
            material=painted_steel,
            name=f"{side_name}_caster_head",
        )
        frame.visual(
            Box((0.040, 0.030, 0.065)),
            origin=Origin(xyz=(0.268, 0.22 * side_sign, 0.228)),
            material=painted_steel,
            name=f"{side_name}_caster_gusset",
        )
        frame.visual(
            Box((0.240, 0.040, 0.018)),
            origin=Origin(xyz=(-0.030, 0.22 * side_sign, 0.635)),
            material=molded_black,
            name=f"{side_name}_armrest_pad",
        )
        for post_x, post_name in ((-0.115, "rear"), (0.045, "front")):
            _add_member(
                frame,
                (post_x, rail_y, 0.500),
                (post_x, rail_y, 0.626),
                radius=0.008,
                material=painted_steel,
                name=f"{side_name}_armrest_{post_name}_post",
            )
        _add_member(
            frame,
            (0.180, rail_y, 0.500),
            (0.240, rail_y, 0.385),
            radius=0.008,
            material=frame_metal,
            name=f"{side_name}_front_mount_brace",
        )

    frame.visual(
        Box((0.430, 0.412, 0.008)),
        origin=Origin(xyz=(0.095, 0.0, 0.504)),
        material=molded_black,
        name="seat_sling",
    )
    frame.visual(
        Box((0.038, 0.412, 0.020)),
        origin=Origin(xyz=(-0.086, 0.0, 0.508)),
        material=molded_black,
        name="seat_back_junction",
    )
    frame.visual(
        Box((0.012, 0.412, 0.330)),
        origin=Origin(xyz=(-0.179, 0.0, 0.665)),
        material=molded_black,
        name="back_sling",
    )

    _add_member(
        frame,
        (0.240, -0.22, 0.385),
        (0.240, 0.22, 0.385),
        radius=0.012,
        material=frame_metal,
        name="footrest_mount_bar",
    )

    for side_name, side_y in (("left", 0.140), ("right", -0.140)):
        hanger = wire_from_points(
            [
                (0.240, side_y, 0.385),
                (0.225, side_y, 0.275),
                (0.240, side_y, 0.188),
            ],
            radius=0.010,
            radial_segments=14,
            cap_ends=True,
            corner_mode="fillet",
            corner_radius=0.035,
            corner_segments=8,
        )
        frame.visual(
            _save_mesh(f"{side_name}_footrest_hanger", hanger),
            material=frame_metal,
            name=f"{side_name}_footrest_hanger",
        )
        _add_member(
            frame,
            (0.240, side_y, 0.188),
            (0.240, side_y, 0.170),
            radius=0.010,
            material=frame_metal,
            name=f"{side_name}_footplate_stem",
        )
        frame.visual(
            Box((0.170, 0.110, 0.012)),
            origin=Origin(xyz=(0.325, side_y, 0.164)),
            material=molded_black,
            name=f"{side_name}_footplate",
        )

    _add_member(
        frame,
        (0.240, -0.140, 0.188),
        (0.240, 0.140, 0.188),
        radius=0.008,
        material=frame_metal,
        name="footplate_toe_bar",
    )

    left_drive_wheel = model.part("left_drive_wheel")
    left_drive_wheel.inertial = Inertial.from_geometry(
        Cylinder(radius=0.306, length=0.032),
        mass=2.4,
        origin=Origin(rpy=(math.pi / 2.0, 0.0, 0.0)),
    )
    _build_drive_wheel(
        left_drive_wheel,
        prefix="left_drive_wheel",
        outboard_sign=1.0,
        rim_material=painted_steel,
        tire_material=rubber,
        handrim_material=handrim_metal,
    )

    right_drive_wheel = model.part("right_drive_wheel")
    right_drive_wheel.inertial = Inertial.from_geometry(
        Cylinder(radius=0.306, length=0.032),
        mass=2.4,
        origin=Origin(rpy=(math.pi / 2.0, 0.0, 0.0)),
    )
    _build_drive_wheel(
        right_drive_wheel,
        prefix="right_drive_wheel",
        outboard_sign=-1.0,
        rim_material=painted_steel,
        tire_material=rubber,
        handrim_material=handrim_metal,
    )

    left_caster_fork = model.part("left_caster_fork")
    left_caster_fork.inertial = Inertial.from_geometry(
        Box((0.110, 0.055, 0.180)),
        mass=0.45,
        origin=Origin(xyz=(-0.028, 0.0, -0.070)),
    )
    left_caster_fork.visual(
        Cylinder(radius=0.011, length=0.055),
        origin=Origin(xyz=(0.0, 0.0, -0.0275)),
        material=painted_steel,
        name="stem",
    )
    left_caster_fork.visual(
        Cylinder(radius=0.020, length=0.010),
        origin=Origin(xyz=(0.0, 0.0, -0.005)),
        material=painted_steel,
        name="top_washer",
    )
    left_caster_fork.visual(
        Box((0.042, 0.050, 0.022)),
        origin=Origin(xyz=(-0.014, 0.0, -0.055)),
        material=painted_steel,
        name="fork_crown",
    )
    for lug_sign in (-1.0, 1.0):
        _add_member(
            left_caster_fork,
            (0.0, 0.018 * lug_sign, -0.055),
            (-0.110, 0.018 * lug_sign, -0.080),
            radius=0.006,
            material=painted_steel,
        )
        left_caster_fork.visual(
            Cylinder(radius=0.006, length=0.006),
            origin=Origin(xyz=(-0.110, 0.017 * lug_sign, -0.080), rpy=(math.pi / 2.0, 0.0, 0.0)),
            material=painted_steel,
            name="axle_lug" if lug_sign > 0.0 else None,
        )

    right_caster_fork = model.part("right_caster_fork")
    right_caster_fork.inertial = Inertial.from_geometry(
        Box((0.110, 0.055, 0.180)),
        mass=0.45,
        origin=Origin(xyz=(-0.028, 0.0, -0.070)),
    )
    right_caster_fork.visual(
        Cylinder(radius=0.011, length=0.055),
        origin=Origin(xyz=(0.0, 0.0, -0.0275)),
        material=painted_steel,
        name="stem",
    )
    right_caster_fork.visual(
        Cylinder(radius=0.020, length=0.010),
        origin=Origin(xyz=(0.0, 0.0, -0.005)),
        material=painted_steel,
        name="top_washer",
    )
    right_caster_fork.visual(
        Box((0.042, 0.050, 0.022)),
        origin=Origin(xyz=(-0.014, 0.0, -0.055)),
        material=painted_steel,
        name="fork_crown",
    )
    for lug_sign in (-1.0, 1.0):
        _add_member(
            right_caster_fork,
            (0.0, 0.018 * lug_sign, -0.055),
            (-0.110, 0.018 * lug_sign, -0.080),
            radius=0.006,
            material=painted_steel,
        )
        right_caster_fork.visual(
            Cylinder(radius=0.006, length=0.006),
            origin=Origin(xyz=(-0.110, 0.017 * lug_sign, -0.080), rpy=(math.pi / 2.0, 0.0, 0.0)),
            material=painted_steel,
            name="axle_lug" if lug_sign > 0.0 else None,
        )

    left_caster_wheel = model.part("left_caster_wheel")
    left_caster_wheel.inertial = Inertial.from_geometry(
        Cylinder(radius=0.065, length=0.028),
        mass=0.35,
        origin=Origin(rpy=(math.pi / 2.0, 0.0, 0.0)),
    )
    _build_caster_wheel(
        left_caster_wheel,
        prefix="left_caster_wheel",
        rim_material=painted_steel,
        tire_material=rubber,
    )

    right_caster_wheel = model.part("right_caster_wheel")
    right_caster_wheel.inertial = Inertial.from_geometry(
        Cylinder(radius=0.065, length=0.028),
        mass=0.35,
        origin=Origin(rpy=(math.pi / 2.0, 0.0, 0.0)),
    )
    _build_caster_wheel(
        right_caster_wheel,
        prefix="right_caster_wheel",
        rim_material=painted_steel,
        tire_material=rubber,
    )

    model.articulation(
        "left_drive_spin",
        ArticulationType.CONTINUOUS,
        parent=frame,
        child=left_drive_wheel,
        origin=Origin(xyz=(-0.145, 0.315, 0.305)),
        axis=(0.0, 1.0, 0.0),
        motion_limits=MotionLimits(effort=20.0, velocity=18.0),
    )
    model.articulation(
        "right_drive_spin",
        ArticulationType.CONTINUOUS,
        parent=frame,
        child=right_drive_wheel,
        origin=Origin(xyz=(-0.145, -0.315, 0.305)),
        axis=(0.0, 1.0, 0.0),
        motion_limits=MotionLimits(effort=20.0, velocity=18.0),
    )
    model.articulation(
        "left_caster_swivel",
        ArticulationType.CONTINUOUS,
        parent=frame,
        child=left_caster_fork,
        origin=Origin(xyz=(0.285, 0.220, 0.145)),
        axis=(0.0, 0.0, 1.0),
        motion_limits=MotionLimits(effort=6.0, velocity=8.0),
    )
    model.articulation(
        "right_caster_swivel",
        ArticulationType.CONTINUOUS,
        parent=frame,
        child=right_caster_fork,
        origin=Origin(xyz=(0.285, -0.220, 0.145)),
        axis=(0.0, 0.0, 1.0),
        motion_limits=MotionLimits(effort=6.0, velocity=8.0),
    )
    model.articulation(
        "left_caster_roll",
        ArticulationType.CONTINUOUS,
        parent=left_caster_fork,
        child=left_caster_wheel,
        origin=Origin(xyz=(-0.110, 0.0, -0.080)),
        axis=(0.0, 1.0, 0.0),
        motion_limits=MotionLimits(effort=3.0, velocity=18.0),
    )
    model.articulation(
        "right_caster_roll",
        ArticulationType.CONTINUOUS,
        parent=right_caster_fork,
        child=right_caster_wheel,
        origin=Origin(xyz=(-0.110, 0.0, -0.080)),
        axis=(0.0, 1.0, 0.0),
        motion_limits=MotionLimits(effort=3.0, velocity=18.0),
    )

    return model


def run_tests() -> TestReport:
    ctx = TestContext(object_model)
    frame = object_model.get_part("frame")
    left_drive_wheel = object_model.get_part("left_drive_wheel")
    right_drive_wheel = object_model.get_part("right_drive_wheel")
    left_caster_fork = object_model.get_part("left_caster_fork")
    right_caster_fork = object_model.get_part("right_caster_fork")
    left_caster_wheel = object_model.get_part("left_caster_wheel")
    right_caster_wheel = object_model.get_part("right_caster_wheel")
    left_drive_spin = object_model.get_articulation("left_drive_spin")
    right_drive_spin = object_model.get_articulation("right_drive_spin")
    left_caster_swivel = object_model.get_articulation("left_caster_swivel")
    right_caster_swivel = object_model.get_articulation("right_caster_swivel")
    left_caster_roll = object_model.get_articulation("left_caster_roll")
    right_caster_roll = object_model.get_articulation("right_caster_roll")

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

    for part_name in (
        "frame",
        "left_drive_wheel",
        "right_drive_wheel",
        "left_caster_fork",
        "right_caster_fork",
        "left_caster_wheel",
        "right_caster_wheel",
    ):
        ctx.check(
            f"part_present_{part_name}",
            object_model.get_part(part_name) is not None,
            f"Missing required part: {part_name}",
        )

    ctx.expect_contact(frame, left_drive_wheel, elem_a="left_axle_boss", elem_b="inboard_cap")
    ctx.expect_contact(frame, right_drive_wheel, elem_a="right_axle_boss", elem_b="inboard_cap")
    ctx.expect_contact(frame, left_caster_fork, elem_a="left_caster_head", elem_b="top_washer")
    ctx.expect_contact(frame, right_caster_fork, elem_a="right_caster_head", elem_b="top_washer")
    ctx.expect_gap(
        left_caster_fork,
        left_caster_wheel,
        axis="y",
        positive_elem="axle_lug",
        negative_elem="hub_cap_pos",
        min_gap=0.0,
        max_gap=0.004,
    )
    ctx.expect_gap(
        right_caster_fork,
        right_caster_wheel,
        axis="y",
        positive_elem="axle_lug",
        negative_elem="hub_cap_pos",
        min_gap=0.0,
        max_gap=0.004,
    )

    ctx.expect_origin_gap(left_drive_wheel, frame, axis="y", min_gap=0.30)
    ctx.expect_origin_gap(frame, right_drive_wheel, axis="y", min_gap=0.30)
    ctx.expect_origin_gap(left_caster_wheel, frame, axis="x", min_gap=0.17)
    ctx.expect_origin_gap(right_caster_wheel, frame, axis="x", min_gap=0.17)
    ctx.expect_gap(frame, left_caster_wheel, axis="z", min_gap=0.0, max_gap=0.030)
    ctx.expect_gap(frame, right_caster_wheel, axis="z", min_gap=0.0, max_gap=0.030)

    ctx.check(
        "drive_wheel_axes",
        left_drive_spin.axis == (0.0, 1.0, 0.0) and right_drive_spin.axis == (0.0, 1.0, 0.0),
        f"Unexpected drive wheel axes: left={left_drive_spin.axis}, right={right_drive_spin.axis}",
    )
    ctx.check(
        "caster_axes",
        left_caster_swivel.axis == (0.0, 0.0, 1.0)
        and right_caster_swivel.axis == (0.0, 0.0, 1.0)
        and left_caster_roll.axis == (0.0, 1.0, 0.0)
        and right_caster_roll.axis == (0.0, 1.0, 0.0),
        "Caster swivel and roll axes must remain vertical and transverse respectively.",
    )

    return ctx.report()


# >>> USER_CODE_END

object_model = build_object_model()
