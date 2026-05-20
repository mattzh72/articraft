from __future__ import annotations

# User code should import every SDK/stdlib symbol it uses instead of relying on
# hidden scaffold imports.
# >>> USER_CODE_START
from math import pi

from sdk import (
    ArticulatedObject,
    ArticulationType,
    Box,
    BoxGeometry,
    Cylinder,
    CylinderGeometry,
    Inertial,
    LatheGeometry,
    MeshGeometry,
    MotionLimits,
    Origin,
    TestContext,
    TestReport,
    mesh_from_geometry,
    tube_from_spline_points,
)


def _save_mesh(name: str, geometry: MeshGeometry):
    return mesh_from_geometry(geometry, name)


def _merge_geometries(*geometries: MeshGeometry) -> MeshGeometry:
    merged = MeshGeometry()
    for geometry in geometries:
        merged.merge(geometry)
    return merged


def _mirror_y(points: list[tuple[float, float, float]]) -> list[tuple[float, float, float]]:
    return [(x, -y, z) for x, y, z in points]


def build_object_model() -> ArticulatedObject:
    model = ArticulatedObject(name="field_service_wheelbarrow")

    frame_paint = model.material("frame_paint", rgba=(0.28, 0.35, 0.18, 1.0))
    tray_steel = model.material("tray_steel", rgba=(0.62, 0.64, 0.60, 1.0))
    wear_steel = model.material("wear_steel", rgba=(0.44, 0.45, 0.47, 1.0))
    machined_steel = model.material("machined_steel", rgba=(0.68, 0.70, 0.73, 1.0))
    rubber = model.material("rubber", rgba=(0.06, 0.06, 0.07, 1.0))
    grip_black = model.material("grip_black", rgba=(0.10, 0.10, 0.10, 1.0))

    frame = model.part("frame")
    frame.inertial = Inertial.from_geometry(
        Box((1.72, 0.66, 0.78)),
        mass=18.0,
        origin=Origin(xyz=(-0.16, 0.0, 0.39)),
    )

    tube_radius = 0.023
    left_handle_path = [
        (-0.96, 0.31, 0.72),
        (-0.82, 0.30, 0.69),
        (-0.66, 0.28, 0.54),
        (-0.56, 0.24, 0.10),
        (-0.52, 0.23, 0.03),
        (-0.40, 0.22, 0.18),
        (-0.16, 0.21, 0.30),
        (0.10, 0.20, 0.35),
        (0.38, 0.18, 0.39),
    ]
    right_handle_path = _mirror_y(left_handle_path)

    left_handle = tube_from_spline_points(
        left_handle_path,
        radius=tube_radius,
        samples_per_segment=16,
        radial_segments=18,
    )
    right_handle = tube_from_spline_points(
        right_handle_path,
        radius=tube_radius,
        samples_per_segment=16,
        radial_segments=18,
    )
    rear_brace = tube_from_spline_points(
        [(-0.58, 0.24, 0.16), (-0.58, -0.24, 0.16)],
        radius=0.024,
        samples_per_segment=2,
        radial_segments=16,
    )
    cross_bridge = BoxGeometry((0.10, 0.54, 0.08)).translate(-0.18, 0.0, 0.31)
    left_side_rail = BoxGeometry((0.82, 0.08, 0.050)).translate(-0.01, 0.19, 0.387)
    right_side_rail = BoxGeometry((0.82, 0.08, 0.050)).translate(-0.01, -0.19, 0.387)

    frame.visual(
        _save_mesh(
            "wheelbarrow_frame_weldment",
            _merge_geometries(
                left_handle,
                right_handle,
                rear_brace,
                cross_bridge,
                left_side_rail,
                right_side_rail,
            ),
        ),
        material=frame_paint,
        name="frame_weldment",
    )
    frame.visual(
        Cylinder(radius=0.029, length=0.15),
        origin=Origin(xyz=(-0.93, 0.28, 0.72), rpy=(0.0, pi / 2.0, 0.0)),
        material=grip_black,
        name="left_grip",
    )
    frame.visual(
        Cylinder(radius=0.029, length=0.15),
        origin=Origin(xyz=(-0.93, -0.28, 0.72), rpy=(0.0, pi / 2.0, 0.0)),
        material=grip_black,
        name="right_grip",
    )
    frame.visual(
        Box((0.09, 0.05, 0.028)),
        origin=Origin(xyz=(-0.52, 0.21, 0.014)),
        material=wear_steel,
        name="left_shoe",
    )
    frame.visual(
        Box((0.09, 0.05, 0.028)),
        origin=Origin(xyz=(-0.52, -0.21, 0.014)),
        material=wear_steel,
        name="right_shoe",
    )

    tray = model.part("tray")
    tray.inertial = Inertial.from_geometry(
        Box((0.94, 0.66, 0.30)),
        mass=11.0,
        origin=Origin(xyz=(-0.02, 0.0, 0.54)),
    )

    tray_floor = BoxGeometry((0.80, 0.46, 0.022)).translate(-0.05, 0.0, 0.443)
    left_wall = BoxGeometry((0.86, 0.022, 0.18)).translate(-0.02, 0.241, 0.533)
    right_wall = BoxGeometry((0.86, 0.022, 0.18)).translate(-0.02, -0.241, 0.533)
    rear_wall = BoxGeometry((0.06, 0.46, 0.15)).translate(-0.42, 0.0, 0.518)
    front_wall = BoxGeometry((0.18, 0.26, 0.18)).rotate_y(-0.55).translate(0.33, 0.0, 0.515)
    left_rear_pad = BoxGeometry((0.22, 0.08, 0.020)).translate(-0.12, 0.17, 0.422)
    right_rear_pad = BoxGeometry((0.22, 0.08, 0.020)).translate(-0.12, -0.17, 0.422)
    left_front_pad = BoxGeometry((0.18, 0.08, 0.020)).translate(0.17, 0.16, 0.422)
    right_front_pad = BoxGeometry((0.18, 0.08, 0.020)).translate(0.17, -0.16, 0.422)

    tray.visual(
        _save_mesh(
            "wheelbarrow_tray_shell",
            _merge_geometries(
                tray_floor,
                left_wall,
                right_wall,
                rear_wall,
                front_wall,
                left_rear_pad,
                right_rear_pad,
                left_front_pad,
                right_front_pad,
            ),
        ),
        material=tray_steel,
        name="tray_shell",
    )

    fork = model.part("fork_axle")
    fork.inertial = Inertial.from_geometry(
        Box((0.34, 0.30, 0.28)),
        mass=7.0,
        origin=Origin(xyz=(0.52, 0.0, 0.42)),
    )

    left_arm_geom = tube_from_spline_points(
        [(0.44, 0.18, 0.392), (0.56, 0.16, 0.37), (0.72, 0.12, 0.32), (0.86, 0.10, 0.25)],
        radius=0.020,
        samples_per_segment=12,
        radial_segments=16,
    )
    right_arm_geom = tube_from_spline_points(
        _mirror_y([(0.44, 0.18, 0.392), (0.56, 0.16, 0.37), (0.72, 0.12, 0.32), (0.86, 0.10, 0.25)]),
        radius=0.020,
        samples_per_segment=12,
        radial_segments=16,
    )
    fork.visual(
        Box((0.04, 0.07, 0.05)),
        origin=Origin(xyz=(0.42, 0.18, 0.392)),
        material=wear_steel,
        name="fork_mount_left",
    )
    fork.visual(
        Box((0.04, 0.07, 0.05)),
        origin=Origin(xyz=(0.42, -0.18, 0.392)),
        material=wear_steel,
        name="fork_mount_right",
    )
    fork.visual(_save_mesh("wheelbarrow_left_fork_arm", left_arm_geom), material=frame_paint, name="left_arm")
    fork.visual(_save_mesh("wheelbarrow_right_fork_arm", right_arm_geom), material=frame_paint, name="right_arm")
    fork.visual(
        Box((0.05, 0.03, 0.05)),
        origin=Origin(xyz=(0.86, 0.10, 0.25)),
        material=wear_steel,
        name="left_boss",
    )
    fork.visual(
        Box((0.05, 0.03, 0.05)),
        origin=Origin(xyz=(0.86, -0.10, 0.25)),
        material=wear_steel,
        name="right_boss",
    )
    fork.visual(
        Cylinder(radius=0.015, length=0.27),
        origin=Origin(xyz=(0.86, 0.0, 0.25), rpy=(pi / 2.0, 0.0, 0.0)),
        material=machined_steel,
        name="axle_shaft",
    )
    fork.visual(
        Cylinder(radius=0.050, length=0.020),
        origin=Origin(xyz=(0.86, 0.059, 0.25), rpy=(pi / 2.0, 0.0, 0.0)),
        material=machined_steel,
        name="left_spacer",
    )
    fork.visual(
        Cylinder(radius=0.050, length=0.020),
        origin=Origin(xyz=(0.86, -0.059, 0.25), rpy=(pi / 2.0, 0.0, 0.0)),
        material=machined_steel,
        name="right_spacer",
    )
    fork.visual(
        Cylinder(radius=0.030, length=0.010),
        origin=Origin(xyz=(0.86, 0.115, 0.25), rpy=(pi / 2.0, 0.0, 0.0)),
        material=machined_steel,
        name="left_nut",
    )
    fork.visual(
        Cylinder(radius=0.030, length=0.010),
        origin=Origin(xyz=(0.86, -0.115, 0.25), rpy=(pi / 2.0, 0.0, 0.0)),
        material=machined_steel,
        name="right_nut",
    )

    wheel = model.part("wheel")
    wheel.inertial = Inertial.from_geometry(
        Cylinder(radius=0.25, length=0.12),
        mass=4.0,
        origin=Origin(rpy=(pi / 2.0, 0.0, 0.0)),
    )

    tire_profile = [
        (0.165, -0.055),
        (0.205, -0.060),
        (0.238, -0.050),
        (0.248, -0.028),
        (0.250, -0.010),
        (0.250, 0.010),
        (0.248, 0.028),
        (0.238, 0.050),
        (0.205, 0.060),
        (0.165, 0.055),
        (0.170, 0.020),
        (0.170, -0.020),
        (0.165, -0.055),
    ]
    hub_outer = [(0.058, -0.060), (0.058, 0.060)]
    hub_inner = [(0.024, -0.054), (0.024, 0.054)]
    rim_outer = [
        (0.058, -0.054),
        (0.130, -0.056),
        (0.155, -0.046),
        (0.172, -0.026),
        (0.172, 0.026),
        (0.155, 0.046),
        (0.130, 0.056),
        (0.058, 0.054),
    ]
    rim_inner = [
        (0.052, -0.048),
        (0.122, -0.048),
        (0.146, -0.040),
        (0.162, -0.022),
        (0.162, 0.022),
        (0.146, 0.040),
        (0.122, 0.048),
        (0.052, 0.048),
    ]

    wheel.visual(
        _save_mesh("wheelbarrow_tire", LatheGeometry(tire_profile, segments=72).rotate_x(pi / 2.0)),
        material=rubber,
        name="tire",
    )
    wheel.visual(
        _save_mesh(
            "wheelbarrow_rim_shell",
            LatheGeometry.from_shell_profiles(rim_outer, rim_inner, segments=72).rotate_x(pi / 2.0),
        ),
        material=tray_steel,
        name="rim_shell",
    )
    wheel.visual(
        _save_mesh(
            "wheelbarrow_hub_shell",
            LatheGeometry.from_shell_profiles(hub_outer, hub_inner, segments=72).rotate_x(pi / 2.0),
        ),
        material=machined_steel,
        name="hub_shell",
    )

    model.articulation(
        "frame_to_tray",
        ArticulationType.FIXED,
        parent=frame,
        child=tray,
        origin=Origin(),
    )
    model.articulation(
        "frame_to_fork",
        ArticulationType.FIXED,
        parent=frame,
        child=fork,
        origin=Origin(),
    )
    model.articulation(
        "wheel_spin",
        ArticulationType.CONTINUOUS,
        parent=fork,
        child=wheel,
        origin=Origin(xyz=(0.86, 0.0, 0.25)),
        axis=(0.0, 1.0, 0.0),
        motion_limits=MotionLimits(effort=30.0, velocity=20.0),
    )

    return model


def run_tests() -> TestReport:
    ctx = TestContext(object_model)
    frame = object_model.get_part("frame")
    tray = object_model.get_part("tray")
    fork = object_model.get_part("fork_axle")
    wheel = object_model.get_part("wheel")
    wheel_spin = object_model.get_articulation("wheel_spin")

    ctx.check_model_valid()
    ctx.check_mesh_assets_ready()
    ctx.allow_overlap(
        fork,
        wheel,
        elem_a="left_spacer",
        elem_b="hub_shell",
        reason="replaceable wheel hub bears against the left service spacer on the axle stack",
    )
    ctx.allow_overlap(
        fork,
        wheel,
        elem_a="right_spacer",
        elem_b="hub_shell",
        reason="replaceable wheel hub bears against the right service spacer on the axle stack",
    )

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

    ctx.expect_contact(tray, frame, name="tray_is_supported_on_frame")
    ctx.expect_contact(fork, frame, name="fork_is_bolted_to_frame")
    ctx.expect_contact(wheel, fork, name="wheel_is_carried_by_axle_spacers")
    ctx.expect_gap(wheel, tray, axis="x", min_gap=0.04, name="wheel_sits_forward_of_tray_nose")
    ctx.expect_gap(
        fork,
        wheel,
        axis="y",
        positive_elem="left_arm",
        negative_elem="tire",
        min_gap=0.01,
        name="left_fork_arm_clears_tire",
    )
    ctx.expect_gap(
        wheel,
        fork,
        axis="y",
        positive_elem="tire",
        negative_elem="right_arm",
        min_gap=0.01,
        name="right_fork_arm_clears_tire",
    )
    ctx.check(
        "wheel_joint_axis_matches_axle",
        tuple(round(v, 3) for v in wheel_spin.axis) == (0.0, 1.0, 0.0),
        details=f"wheel_spin axis was {wheel_spin.axis!r}, expected axle-aligned +Y",
    )

    left_shoe_aabb = ctx.part_element_world_aabb(frame, elem="left_shoe")
    right_shoe_aabb = ctx.part_element_world_aabb(frame, elem="right_shoe")
    wheel_aabb = ctx.part_world_aabb(wheel)
    stance_ok = False
    stance_details = "missing stance geometry"
    if left_shoe_aabb and right_shoe_aabb and wheel_aabb:
        left_bottom = left_shoe_aabb[0][2]
        right_bottom = right_shoe_aabb[0][2]
        wheel_bottom = wheel_aabb[0][2]
        left_y = 0.5 * (left_shoe_aabb[0][1] + left_shoe_aabb[1][1])
        right_y = 0.5 * (right_shoe_aabb[0][1] + right_shoe_aabb[1][1])
        wheel_front = wheel_aabb[1][0]
        left_rear = left_shoe_aabb[0][0]
        stance_ok = (
            -0.01 <= left_bottom <= 0.02
            and -0.01 <= right_bottom <= 0.02
            and -0.01 <= wheel_bottom <= 0.02
            and left_y > 0.12
            and right_y < -0.12
            and (wheel_front - left_rear) > 0.95
        )
        stance_details = (
            f"left_bottom={left_bottom:.3f}, right_bottom={right_bottom:.3f}, "
            f"wheel_bottom={wheel_bottom:.3f}, left_y={left_y:.3f}, "
            f"right_y={right_y:.3f}, wheel_front_minus_rear={wheel_front - left_rear:.3f}"
        )
    ctx.check("tripod_stance_is_obvious", stance_ok, details=stance_details)

    with ctx.pose({wheel_spin: 1.3}):
        ctx.expect_contact(wheel, fork, name="wheel_keeps_axle_contact_when_spun")

    return ctx.report()


# >>> USER_CODE_END

object_model = build_object_model()
