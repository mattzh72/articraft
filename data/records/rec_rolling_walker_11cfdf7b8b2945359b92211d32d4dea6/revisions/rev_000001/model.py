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
    tube_from_spline_points,
)


def _save_mesh(name: str, geometry):
    return mesh_from_geometry(geometry, name)


def _midpoint(
    a: tuple[float, float, float], b: tuple[float, float, float]
) -> tuple[float, float, float]:
    return ((a[0] + b[0]) * 0.5, (a[1] + b[1]) * 0.5, (a[2] + b[2]) * 0.5)


def _distance(
    a: tuple[float, float, float], b: tuple[float, float, float]
) -> float:
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


def _add_member(part, a, b, radius: float, material, name: str | None = None) -> None:
    part.visual(
        Cylinder(radius=radius, length=_distance(a, b)),
        origin=Origin(xyz=_midpoint(a, b), rpy=_rpy_for_cylinder(a, b)),
        material=material,
        name=name,
    )


def _build_side_frame(
    part,
    *,
    mesh_prefix: str,
    side_sign: float,
    frame_material,
    grip_material,
    seal_material,
    hardware_material,
    rubber_material,
) -> None:
    p0 = (side_sign * 0.230, -0.020, -0.290)
    p1 = (side_sign * 0.230, -0.032, -0.100)
    p2 = (side_sign * 0.230, -0.048, 0.100)
    p3 = (side_sign * 0.212, -0.120, 0.270)
    p4 = (side_sign * 0.190, -0.200, 0.370)
    p5 = (side_sign * 0.190, -0.290, 0.370)
    p6 = (side_sign * 0.200, -0.350, 0.210)
    p7 = (side_sign * 0.210, -0.380, -0.120)
    p8 = (side_sign * 0.220, -0.420, -0.470)

    main_tube = tube_from_spline_points(
        [p0, p1, p2, p3, p4, p5, p6, p7, p8],
        radius=0.016,
        samples_per_segment=16,
        radial_segments=18,
        cap_ends=True,
        up_hint=(0.0, 0.0, 1.0),
    )
    part.visual(
        _save_mesh(f"{mesh_prefix}_main_tube", main_tube),
        material=frame_material,
        name="main_tube",
    )

    lower_brace = tube_from_spline_points(
        [p1, (side_sign * 0.222, -0.015, -0.180), p7],
        radius=0.013,
        samples_per_segment=12,
        radial_segments=16,
        cap_ends=True,
        up_hint=(0.0, 0.0, 1.0),
    )
    part.visual(
        _save_mesh(f"{mesh_prefix}_lower_brace", lower_brace),
        material=frame_material,
        name="lower_brace",
    )

    _add_member(
        part,
        (side_sign * 0.058, -0.014, 0.018),
        p4,
        0.013,
        frame_material,
        name="upper_fold_arm",
    )
    _add_member(
        part,
        (side_sign * 0.058, -0.014, -0.058),
        p3,
        0.013,
        frame_material,
        name="lower_fold_arm",
    )

    part.visual(
        Box((0.040, 0.060, 0.110)),
        origin=Origin(xyz=(side_sign * 0.078, -0.018, 0.000)),
        material=seal_material,
        name="hinge_gusset",
    )
    part.visual(
        Box((0.022, 0.046, 0.086)),
        origin=Origin(xyz=(side_sign * 0.046, -0.016, 0.000)),
        material=hardware_material,
        name="hinge_leaf",
    )

    grip_center = (side_sign * 0.190, -0.245, 0.370)
    part.visual(
        Cylinder(radius=0.018, length=0.120),
        origin=Origin(
            xyz=grip_center,
            rpy=(math.pi / 2.0, 0.0, 0.0),
        ),
        material=grip_material,
        name="hand_grip",
    )

    _add_member(
        part,
        p4,
        (side_sign * 0.206, -0.118, 0.422),
        0.010,
        frame_material,
        name="brake_mount_post",
    )
    part.visual(
        Box((0.022, 0.024, 0.020)),
        origin=Origin(xyz=(side_sign * 0.206, -0.118, 0.422)),
        material=frame_material,
        name="brake_mount_bracket",
    )
    part.visual(
        Cylinder(radius=0.011, length=0.050),
        origin=Origin(
            xyz=(side_sign * 0.210, -0.116, 0.422),
            rpy=(0.0, math.pi / 2.0, 0.0),
        ),
        material=hardware_material,
        name="brake_ready_boss",
    )
    part.visual(
        Box((0.026, 0.020, 0.016)),
        origin=Origin(xyz=(side_sign * 0.214, -0.116, 0.434)),
        material=seal_material,
        name="cable_port_cap",
    )

    caster_origin = (side_sign * 0.230, -0.020, -0.335)
    part.visual(
        Cylinder(radius=0.018, length=0.060),
        origin=Origin(xyz=(caster_origin[0], caster_origin[1], caster_origin[2] + 0.030)),
        material=frame_material,
        name="caster_receiver",
    )
    part.visual(
        Cylinder(radius=0.024, length=0.008),
        origin=Origin(xyz=(caster_origin[0], caster_origin[1], caster_origin[2] + 0.064)),
        material=seal_material,
        name="caster_rain_cap",
    )

    part.visual(
        Box((0.040, 0.055, 0.050)),
        origin=Origin(xyz=(side_sign * 0.220, -0.420, -0.495)),
        material=rubber_material,
        name="rear_ferrule",
    )

    part.inertial = Inertial.from_geometry(
        Box((0.28, 0.48, 0.96)),
        mass=2.8,
        origin=Origin(xyz=(side_sign * 0.190, -0.220, -0.040)),
    )


def _build_center_body(part, *, frame_material, seal_material, hardware_material) -> None:
    part.visual(
        Box((0.180, 0.100, 0.110)),
        origin=Origin(xyz=(0.0, 0.080, 0.520)),
        material=seal_material,
        name="sealed_hinge_housing",
    )
    part.visual(
        Box((0.220, 0.130, 0.012)),
        origin=Origin(xyz=(0.0, 0.092, 0.582)),
        material=frame_material,
        name="drip_cap",
    )
    part.visual(
        Cylinder(radius=0.018, length=0.180),
        origin=Origin(xyz=(0.0, 0.078, 0.480), rpy=(0.0, math.pi / 2.0, 0.0)),
        material=frame_material,
        name="lower_spreader_tube",
    )
    part.visual(
        Box((0.150, 0.044, 0.036)),
        origin=Origin(xyz=(0.0, 0.072, 0.444)),
        material=frame_material,
        name="underslung_crossbar",
    )
    for x_pos in (-0.078, 0.078):
        part.visual(
            Box((0.016, 0.026, 0.032)),
            origin=Origin(xyz=(x_pos, 0.092, 0.566)),
            material=frame_material,
            name=f"{'left' if x_pos < 0 else 'right'}_drip_post",
        )
    for x_pos in (-0.052, 0.052):
        part.visual(
            Box((0.024, 0.028, 0.040)),
            origin=Origin(xyz=(x_pos, 0.075, 0.460)),
            material=frame_material,
            name=f"{'left' if x_pos < 0 else 'right'}_crossbar_web",
        )
    for side_sign in (-1.0, 1.0):
        hinge_x = side_sign * 0.055
        part.visual(
            Cylinder(radius=0.021, length=0.032),
            origin=Origin(xyz=(hinge_x, 0.080, 0.553)),
            material=hardware_material,
            name=f"{'left' if side_sign < 0 else 'right'}_hinge_barrel_top",
        )
        part.visual(
            Cylinder(radius=0.021, length=0.032),
            origin=Origin(xyz=(hinge_x, 0.080, 0.487)),
            material=hardware_material,
            name=f"{'left' if side_sign < 0 else 'right'}_hinge_barrel_bottom",
        )
        part.visual(
            Box((0.034, 0.060, 0.020)),
            origin=Origin(xyz=(hinge_x, 0.080, 0.520)),
            material=seal_material,
            name=f"{'left' if side_sign < 0 else 'right'}_hinge_seal_block",
        )

    part.inertial = Inertial.from_geometry(
        Box((0.24, 0.14, 0.17)),
        mass=1.6,
        origin=Origin(xyz=(0.0, 0.080, 0.510)),
    )


def _build_caster_fork(
    part,
    *,
    hardware_material,
    seal_material,
    wheel_center_z: float,
) -> None:
    part.visual(
        Cylinder(radius=0.015, length=0.028),
        origin=Origin(xyz=(0.0, 0.0, -0.014)),
        material=hardware_material,
        name="caster_stem",
    )
    part.visual(
        Cylinder(radius=0.020, length=0.010),
        origin=Origin(xyz=(0.0, 0.0, -0.004)),
        material=seal_material,
        name="stem_seal",
    )
    part.visual(
        Box((0.052, 0.034, 0.010)),
        origin=Origin(xyz=(0.0, 0.0, -0.010)),
        material=hardware_material,
        name="fork_crown",
    )
    part.visual(
        Box((0.050, 0.045, 0.006)),
        origin=Origin(xyz=(0.0, 0.004, -0.003)),
        material=seal_material,
        name="splash_hood",
    )
    for side_sign in (-1.0, 1.0):
        part.visual(
            Box((0.006, 0.020, 0.084)),
            origin=Origin(xyz=(side_sign * 0.020, 0.0, wheel_center_z + 0.050)),
            material=hardware_material,
            name=f"{'left' if side_sign < 0 else 'right'}_fork_tine",
        )
    part.inertial = Inertial.from_geometry(
        Box((0.060, 0.050, 0.120)),
        mass=0.45,
        origin=Origin(xyz=(0.0, 0.0, -0.050)),
    )


def _build_front_wheel(part, *, tire_material, hub_material) -> None:
    spin_origin = Origin(rpy=(0.0, math.pi / 2.0, 0.0))
    part.visual(
        Cylinder(radius=0.080, length=0.028),
        origin=spin_origin,
        material=tire_material,
        name="tread",
    )
    part.visual(
        Cylinder(radius=0.050, length=0.032),
        origin=spin_origin,
        material=tire_material,
        name="sidewall_band",
    )
    part.visual(
        Cylinder(radius=0.022, length=0.034),
        origin=spin_origin,
        material=hub_material,
        name="hub_barrel",
    )
    part.visual(
        Cylinder(radius=0.010, length=0.040),
        origin=spin_origin,
        material=hub_material,
        name="axle_boss",
    )
    part.inertial = Inertial.from_geometry(
        Cylinder(radius=0.080, length=0.028),
        mass=0.55,
        origin=spin_origin,
    )


def build_object_model() -> ArticulatedObject:
    model = ArticulatedObject(name="weatherproof_rolling_walker")

    frame_aluminum = model.material("frame_aluminum", rgba=(0.79, 0.81, 0.82, 1.0))
    seal_gray = model.material("seal_gray", rgba=(0.28, 0.30, 0.32, 1.0))
    hardware_stainless = model.material(
        "hardware_stainless", rgba=(0.67, 0.70, 0.72, 1.0)
    )
    grip_black = model.material("grip_black", rgba=(0.08, 0.08, 0.09, 1.0))
    tire_rubber = model.material("tire_rubber", rgba=(0.06, 0.06, 0.06, 1.0))

    center_body = model.part("center_body")
    _build_center_body(
        center_body,
        frame_material=frame_aluminum,
        seal_material=seal_gray,
        hardware_material=hardware_stainless,
    )

    left_frame = model.part("left_frame")
    _build_side_frame(
        left_frame,
        mesh_prefix="left_frame",
        side_sign=-1.0,
        frame_material=frame_aluminum,
        grip_material=grip_black,
        seal_material=seal_gray,
        hardware_material=hardware_stainless,
        rubber_material=tire_rubber,
    )

    right_frame = model.part("right_frame")
    _build_side_frame(
        right_frame,
        mesh_prefix="right_frame",
        side_sign=1.0,
        frame_material=frame_aluminum,
        grip_material=grip_black,
        seal_material=seal_gray,
        hardware_material=hardware_stainless,
        rubber_material=tire_rubber,
    )

    left_caster_fork = model.part("left_caster_fork")
    _build_caster_fork(
        left_caster_fork,
        hardware_material=hardware_stainless,
        seal_material=seal_gray,
        wheel_center_z=-0.105,
    )

    right_caster_fork = model.part("right_caster_fork")
    _build_caster_fork(
        right_caster_fork,
        hardware_material=hardware_stainless,
        seal_material=seal_gray,
        wheel_center_z=-0.105,
    )

    left_front_wheel = model.part("left_front_wheel")
    _build_front_wheel(
        left_front_wheel,
        tire_material=tire_rubber,
        hub_material=hardware_stainless,
    )

    right_front_wheel = model.part("right_front_wheel")
    _build_front_wheel(
        right_front_wheel,
        tire_material=tire_rubber,
        hub_material=hardware_stainless,
    )

    model.articulation(
        "center_to_left_frame",
        ArticulationType.REVOLUTE,
        parent=center_body,
        child=left_frame,
        origin=Origin(xyz=(-0.055, 0.080, 0.520)),
        axis=(0.0, 0.0, 1.0),
        motion_limits=MotionLimits(
            effort=18.0,
            velocity=1.2,
            lower=0.0,
            upper=0.95,
        ),
    )
    model.articulation(
        "center_to_right_frame",
        ArticulationType.REVOLUTE,
        parent=center_body,
        child=right_frame,
        origin=Origin(xyz=(0.055, 0.080, 0.520)),
        axis=(0.0, 0.0, -1.0),
        motion_limits=MotionLimits(
            effort=18.0,
            velocity=1.2,
            lower=0.0,
            upper=0.95,
        ),
    )

    model.articulation(
        "left_caster_swivel",
        ArticulationType.CONTINUOUS,
        parent=left_frame,
        child=left_caster_fork,
        origin=Origin(xyz=(-0.230, -0.020, -0.335)),
        axis=(0.0, 0.0, 1.0),
        motion_limits=MotionLimits(effort=2.5, velocity=6.0),
    )
    model.articulation(
        "right_caster_swivel",
        ArticulationType.CONTINUOUS,
        parent=right_frame,
        child=right_caster_fork,
        origin=Origin(xyz=(0.230, -0.020, -0.335)),
        axis=(0.0, 0.0, 1.0),
        motion_limits=MotionLimits(effort=2.5, velocity=6.0),
    )
    model.articulation(
        "left_wheel_spin",
        ArticulationType.CONTINUOUS,
        parent=left_caster_fork,
        child=left_front_wheel,
        origin=Origin(xyz=(0.0, 0.0, -0.105)),
        axis=(1.0, 0.0, 0.0),
        motion_limits=MotionLimits(effort=2.0, velocity=18.0),
    )
    model.articulation(
        "right_wheel_spin",
        ArticulationType.CONTINUOUS,
        parent=right_caster_fork,
        child=right_front_wheel,
        origin=Origin(xyz=(0.0, 0.0, -0.105)),
        axis=(1.0, 0.0, 0.0),
        motion_limits=MotionLimits(effort=2.0, velocity=18.0),
    )

    return model


def run_tests() -> TestReport:
    ctx = TestContext(object_model)
    center_body = object_model.get_part("center_body")
    left_frame = object_model.get_part("left_frame")
    right_frame = object_model.get_part("right_frame")
    left_caster_fork = object_model.get_part("left_caster_fork")
    right_caster_fork = object_model.get_part("right_caster_fork")
    left_front_wheel = object_model.get_part("left_front_wheel")
    right_front_wheel = object_model.get_part("right_front_wheel")

    left_fold = object_model.get_articulation("center_to_left_frame")
    right_fold = object_model.get_articulation("center_to_right_frame")
    left_swivel = object_model.get_articulation("left_caster_swivel")
    right_swivel = object_model.get_articulation("right_caster_swivel")
    left_spin = object_model.get_articulation("left_wheel_spin")
    right_spin = object_model.get_articulation("right_wheel_spin")

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

    with ctx.pose(
        {
            left_fold: 0.0,
            right_fold: 0.0,
            left_swivel: 0.0,
            right_swivel: 0.0,
            left_spin: 0.0,
            right_spin: 0.0,
        }
    ):
        ctx.expect_contact(center_body, left_frame, name="left_frame_hinge_seated")
        ctx.expect_contact(center_body, right_frame, name="right_frame_hinge_seated")
        ctx.expect_contact(left_frame, left_caster_fork, name="left_caster_stem_supported")
        ctx.expect_contact(right_frame, right_caster_fork, name="right_caster_stem_supported")
        ctx.expect_contact(left_caster_fork, left_front_wheel, name="left_wheel_captured")
        ctx.expect_contact(right_caster_fork, right_front_wheel, name="right_wheel_captured")

        open_aabbs = [
            ctx.part_world_aabb(part)
            for part in (
                center_body,
                left_frame,
                right_frame,
                left_caster_fork,
                right_caster_fork,
                left_front_wheel,
                right_front_wheel,
            )
        ]
        open_aabbs = [aabb for aabb in open_aabbs if aabb is not None]
        min_x = min(aabb[0][0] for aabb in open_aabbs)
        max_x = max(aabb[1][0] for aabb in open_aabbs)
        min_y = min(aabb[0][1] for aabb in open_aabbs)
        max_y = max(aabb[1][1] for aabb in open_aabbs)
        min_z = min(aabb[0][2] for aabb in open_aabbs)
        max_z = max(aabb[1][2] for aabb in open_aabbs)
        ctx.check(
            "walker_overall_proportions",
            0.50 <= (max_x - min_x) <= 0.72
            and 0.44 <= (max_y - min_y) <= 0.62
            and 0.84 <= (max_z - min_z) <= 0.98,
            details=(
                f"width={max_x - min_x:.3f}, depth={max_y - min_y:.3f}, "
                f"height={max_z - min_z:.3f}"
            ),
        )
        ctx.check(
            "walker_stance_near_ground",
            -0.005 <= min_z <= 0.020,
            details=f"lowest point z={min_z:.4f}",
        )

    with ctx.pose(
        {
            left_fold: 0.95,
            right_fold: 0.95,
            left_swivel: 0.0,
            right_swivel: 0.0,
        }
    ):
        folded_aabbs = [
            ctx.part_world_aabb(part)
            for part in (
                center_body,
                left_frame,
                right_frame,
                left_caster_fork,
                right_caster_fork,
                left_front_wheel,
                right_front_wheel,
            )
        ]
        folded_aabbs = [aabb for aabb in folded_aabbs if aabb is not None]
        folded_width = max(aabb[1][0] for aabb in folded_aabbs) - min(
            aabb[0][0] for aabb in folded_aabbs
        )

    with ctx.pose(
        {
            left_fold: 0.0,
            right_fold: 0.0,
            left_swivel: 0.0,
            right_swivel: 0.0,
        }
    ):
        open_aabbs = [
            ctx.part_world_aabb(part)
            for part in (
                center_body,
                left_frame,
                right_frame,
                left_caster_fork,
                right_caster_fork,
                left_front_wheel,
                right_front_wheel,
            )
        ]
        open_aabbs = [aabb for aabb in open_aabbs if aabb is not None]
        open_width = max(aabb[1][0] for aabb in open_aabbs) - min(
            aabb[0][0] for aabb in open_aabbs
        )

    ctx.check(
        "frame_folds_narrower",
        folded_width < open_width - 0.10,
        details=f"open_width={open_width:.3f}, folded_width={folded_width:.3f}",
    )
    ctx.check(
        "fold_axes_opposed_for_symmetric_closing",
        left_fold.axis == (0.0, 0.0, 1.0) and right_fold.axis == (0.0, 0.0, -1.0),
        details=f"left_axis={left_fold.axis}, right_axis={right_fold.axis}",
    )
    ctx.check(
        "caster_axes_match_mechanics",
        left_swivel.axis == (0.0, 0.0, 1.0)
        and right_swivel.axis == (0.0, 0.0, 1.0)
        and left_spin.axis == (1.0, 0.0, 0.0)
        and right_spin.axis == (1.0, 0.0, 0.0),
        details=(
            f"left_swivel={left_swivel.axis}, right_swivel={right_swivel.axis}, "
            f"left_spin={left_spin.axis}, right_spin={right_spin.axis}"
        ),
    )

    return ctx.report()


# >>> USER_CODE_END

object_model = build_object_model()
