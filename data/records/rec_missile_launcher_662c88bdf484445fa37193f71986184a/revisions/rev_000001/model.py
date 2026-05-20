from __future__ import annotations

# User code should import every SDK/stdlib symbol it uses instead of relying on
# hidden scaffold imports.
# >>> USER_CODE_START
from math import pi

from sdk import (
    ArticulatedObject,
    ArticulationType,
    Box,
    Cylinder,
    CylinderGeometry,
    ExtrudeWithHolesGeometry,
    Inertial,
    MotionLimits,
    Origin,
    TestContext,
    TestReport,
    TorusGeometry,
    mesh_from_geometry,
    rounded_rect_profile,
    superellipse_profile,
    tube_from_spline_points,
)


def _save_mesh(name: str, geometry):
    return mesh_from_geometry(geometry, name)


def _offset_profile(profile: list[tuple[float, float]], *, dx: float = 0.0, dy: float = 0.0) -> list[tuple[float, float]]:
    return [(x + dx, y + dy) for x, y in profile]


def build_object_model() -> ArticulatedObject:
    model = ArticulatedObject(name="light_missile_launcher")

    olive = model.material("olive", rgba=(0.39, 0.43, 0.28, 1.0))
    dark_olive = model.material("dark_olive", rgba=(0.24, 0.28, 0.18, 1.0))
    gunmetal = model.material("gunmetal", rgba=(0.24, 0.26, 0.29, 1.0))
    steel = model.material("steel", rgba=(0.61, 0.64, 0.67, 1.0))
    rubber = model.material("rubber", rgba=(0.07, 0.07, 0.07, 1.0))

    tube_radius = 0.09
    tube_length = 1.28
    tube_spacing = 0.26
    tube_center_z = 0.07
    front_x = 1.34
    rear_x = front_x - tube_length

    open_tube_mesh = _save_mesh(
        "launcher_tube_shell",
        CylinderGeometry(radius=tube_radius, height=tube_length, radial_segments=40, closed=False).rotate_y(pi / 2.0),
    )
    rim_mesh = _save_mesh(
        "launcher_tube_rim",
        TorusGeometry(radius=tube_radius, tube=0.006, radial_segments=18, tubular_segments=48).rotate_y(pi / 2.0),
    )
    wheel_ring_mesh = _save_mesh(
        "handwheel_ring",
        TorusGeometry(radius=0.055, tube=0.006, radial_segments=16, tubular_segments=48).rotate_x(pi / 2.0),
    )

    bracket_profile = rounded_rect_profile(0.40, 0.31, 0.03, corner_segments=8)
    tube_hole = superellipse_profile(tube_radius * 2.02, tube_radius * 2.02, exponent=2.0, segments=32)
    bracket_mesh = _save_mesh(
        "launcher_bracket_web",
        ExtrudeWithHolesGeometry(
            bracket_profile,
            [
                _offset_profile(tube_hole, dx=tube_spacing * 0.5, dy=tube_center_z),
                _offset_profile(tube_hole, dx=-tube_spacing * 0.5, dy=tube_center_z),
            ],
            height=0.052,
            center=True,
        ).rotate_y(pi / 2.0),
    )

    pedestal = model.part("pedestal")
    pedestal.visual(Box((0.72, 0.06, 0.07)), origin=Origin(xyz=(0.0, 0.23, 0.035)), material=dark_olive, name="left_skid")
    pedestal.visual(Box((0.72, 0.06, 0.07)), origin=Origin(xyz=(0.0, -0.23, 0.035)), material=dark_olive, name="right_skid")
    pedestal.visual(Box((0.12, 0.52, 0.06)), origin=Origin(xyz=(0.27, 0.0, 0.035)), material=dark_olive, name="front_crossbar")
    pedestal.visual(Box((0.12, 0.52, 0.06)), origin=Origin(xyz=(-0.27, 0.0, 0.035)), material=dark_olive, name="rear_crossbar")
    pedestal.visual(Box((0.18, 0.26, 0.05)), origin=Origin(xyz=(0.0, 0.0, 0.085)), material=dark_olive, name="mast_boot")
    pedestal.visual(
        Cylinder(radius=0.075, length=0.53),
        origin=Origin(xyz=(0.0, 0.0, 0.375)),
        material=olive,
        name="center_mast",
    )
    pedestal.visual(
        Cylinder(radius=0.145, length=0.10),
        origin=Origin(xyz=(0.0, 0.0, 0.69)),
        material=gunmetal,
        name="yaw_bearing_housing",
    )
    pedestal.visual(
        Box((0.22, 0.22, 0.02)),
        origin=Origin(xyz=(0.0, 0.0, 0.75)),
        material=gunmetal,
        name="yaw_mount_plate",
    )
    for name, points in (
        ("front_left_leg", [(-0.21, 0.19, 0.07), (-0.09, 0.10, 0.28), (-0.04, 0.04, 0.60)]),
        ("front_right_leg", [(-0.21, -0.19, 0.07), (-0.09, -0.10, 0.28), (-0.04, -0.04, 0.60)]),
        ("rear_left_leg", [(0.21, 0.19, 0.07), (0.09, 0.10, 0.28), (0.04, 0.04, 0.60)]),
        ("rear_right_leg", [(0.21, -0.19, 0.07), (0.09, -0.10, 0.28), (0.04, -0.04, 0.60)]),
        ("left_brace", [(-0.26, 0.23, 0.06), (0.0, 0.15, 0.18), (0.0, 0.07, 0.60)]),
        ("right_brace", [(-0.26, -0.23, 0.06), (0.0, -0.15, 0.18), (0.0, -0.07, 0.60)]),
    ):
        pedestal.visual(
            _save_mesh(
                f"{name}_mesh",
                tube_from_spline_points(points, radius=0.018, samples_per_segment=10, radial_segments=14),
            ),
            material=olive,
            name=name,
        )
    pedestal.inertial = Inertial.from_geometry(
        Box((0.72, 0.58, 0.78)),
        mass=62.0,
        origin=Origin(xyz=(0.0, 0.0, 0.39)),
    )

    yaw_head = model.part("yaw_head")
    yaw_head.visual(
        Cylinder(radius=0.13, length=0.055),
        origin=Origin(xyz=(0.0, 0.0, 0.0275)),
        material=gunmetal,
        name="turntable_base",
    )
    yaw_head.visual(
        Cylinder(radius=0.072, length=0.15),
        origin=Origin(xyz=(0.0, 0.0, 0.1125)),
        material=olive,
        name="rotating_drum",
    )
    yaw_head.visual(
        Box((0.12, 0.38, 0.03)),
        origin=Origin(xyz=(-0.12, 0.0, 0.17)),
        material=dark_olive,
        name="yoke_saddle",
    )
    yaw_head.visual(
        Box((0.16, 0.42, 0.08)),
        origin=Origin(xyz=(-0.07, 0.0, 0.13)),
        material=dark_olive,
        name="lower_spreader",
    )
    yaw_head.visual(
        Box((0.08, 0.04, 0.34)),
        origin=Origin(xyz=(-0.04, 0.22, 0.29)),
        material=olive,
        name="left_yoke_arm",
    )
    yaw_head.visual(
        Box((0.08, 0.04, 0.34)),
        origin=Origin(xyz=(-0.04, -0.22, 0.29)),
        material=olive,
        name="right_yoke_arm",
    )
    yaw_head.visual(
        Box((0.10, 0.40, 0.04)),
        origin=Origin(xyz=(-0.03, 0.0, 0.44)),
        material=dark_olive,
        name="arm_tie_bar",
    )
    yaw_head.visual(
        Cylinder(radius=0.048, length=0.010),
        origin=Origin(xyz=(0.0, 0.20, 0.30), rpy=(pi / 2.0, 0.0, 0.0)),
        material=gunmetal,
        name="left_trunnion_bearing",
    )
    yaw_head.visual(
        Cylinder(radius=0.048, length=0.010),
        origin=Origin(xyz=(0.0, -0.20, 0.30), rpy=(pi / 2.0, 0.0, 0.0)),
        material=gunmetal,
        name="right_trunnion_bearing",
    )
    yaw_head.inertial = Inertial.from_geometry(
        Box((0.24, 0.42, 0.50)),
        mass=18.0,
        origin=Origin(xyz=(-0.02, 0.0, 0.25)),
    )

    launcher = model.part("launcher_carriage")
    launcher.visual(
        bracket_mesh,
        origin=Origin(xyz=(0.12, 0.0, 0.025)),
        material=olive,
        name="bracket_web",
    )
    launcher.visual(
        Box((0.10, 0.10, 0.03)),
        origin=Origin(xyz=(0.18, 0.0, -0.055)),
        material=dark_olive,
        name="lower_bridge",
    )
    launcher.visual(
        Box((0.16, 0.05, 0.08)),
        origin=Origin(xyz=(0.02, 0.135, -0.05)),
        material=dark_olive,
        name="left_lower_brace",
    )
    launcher.visual(
        Box((0.16, 0.05, 0.08)),
        origin=Origin(xyz=(0.02, -0.135, -0.05)),
        material=dark_olive,
        name="right_lower_brace",
    )
    launcher.visual(
        Box((0.18, 0.32, 0.03)),
        origin=Origin(xyz=(0.08, 0.0, 0.17)),
        material=dark_olive,
        name="top_clip_bar",
    )
    launcher.visual(
        Box((0.14, 0.04, 0.22)),
        origin=Origin(xyz=(0.01, 0.17, 0.01)),
        material=olive,
        name="left_side_plate",
    )
    launcher.visual(
        Box((0.14, 0.04, 0.22)),
        origin=Origin(xyz=(0.01, -0.17, 0.01)),
        material=olive,
        name="right_side_plate",
    )
    launcher.visual(
        Box((0.26, 0.16, 0.07)),
        origin=Origin(xyz=(-0.03, 0.0, 0.055)),
        material=dark_olive,
        name="rear_saddle_block",
    )
    launcher.visual(
        Box((0.06, 0.10, 0.08)),
        origin=Origin(xyz=(0.33, 0.0, 0.04)),
        material=dark_olive,
        name="mid_spacer",
    )
    launcher.visual(
        Box((0.06, 0.10, 0.08)),
        origin=Origin(xyz=(0.82, 0.0, 0.04)),
        material=dark_olive,
        name="front_spacer",
    )
    launcher.visual(
        Cylinder(radius=0.040, length=0.040),
        origin=Origin(xyz=(0.0, 0.185, 0.0), rpy=(pi / 2.0, 0.0, 0.0)),
        material=steel,
        name="left_trunnion_cap",
    )
    launcher.visual(
        Cylinder(radius=0.040, length=0.040),
        origin=Origin(xyz=(0.0, -0.185, 0.0), rpy=(pi / 2.0, 0.0, 0.0)),
        material=steel,
        name="right_trunnion_cap",
    )
    launcher.visual(
        Cylinder(radius=0.018, length=0.030),
        origin=Origin(xyz=(-0.145, 0.295, 0.025), rpy=(pi / 2.0, 0.0, 0.0)),
        material=gunmetal,
        name="wheel_mount_stub",
    )
    launcher.visual(
        Box((0.068, 0.25, 0.04)),
        origin=Origin(xyz=(-0.115, 0.1625, 0.025)),
        material=dark_olive,
        name="wheel_support_arm",
    )
    for side_name, side_y in (("left", tube_spacing * 0.5), ("right", -tube_spacing * 0.5)):
        launcher.visual(
            open_tube_mesh,
            origin=Origin(xyz=(rear_x + tube_length * 0.5, side_y, tube_center_z)),
            material=olive,
            name=f"{side_name}_tube_shell",
        )
        launcher.visual(
            rim_mesh,
            origin=Origin(xyz=(front_x, side_y, tube_center_z)),
            material=steel,
            name=f"{side_name}_tube_front_rim",
        )
        launcher.visual(
            rim_mesh,
            origin=Origin(xyz=(rear_x, side_y, tube_center_z)),
            material=steel,
            name=f"{side_name}_tube_rear_rim",
        )
    launcher.inertial = Inertial.from_geometry(
        Box((1.34, 0.42, 0.34)),
        mass=26.0,
        origin=Origin(xyz=(0.46, 0.0, 0.06)),
    )

    handwheel = model.part("handwheel")
    handwheel.visual(
        Cylinder(radius=0.008, length=0.030),
        origin=Origin(xyz=(0.0, -0.015, 0.0), rpy=(pi / 2.0, 0.0, 0.0)),
        material=gunmetal,
        name="wheel_axle",
    )
    handwheel.visual(
        Cylinder(radius=0.014, length=0.028),
        origin=Origin(xyz=(0.0, 0.012, 0.0), rpy=(pi / 2.0, 0.0, 0.0)),
        material=steel,
        name="wheel_hub",
    )
    handwheel.visual(wheel_ring_mesh, origin=Origin(xyz=(0.0, 0.015, 0.0)), material=rubber, name="wheel_ring")
    handwheel.visual(
        Box((0.082, 0.008, 0.008)),
        origin=Origin(xyz=(0.0, 0.015, 0.0)),
        material=steel,
        name="spoke_horizontal",
    )
    handwheel.visual(
        Box((0.008, 0.008, 0.104)),
        origin=Origin(xyz=(0.0, 0.015, 0.0)),
        material=steel,
        name="spoke_vertical",
    )
    handwheel.visual(
        Box((0.078, 0.008, 0.008)),
        origin=Origin(xyz=(0.0, 0.015, 0.0), rpy=(0.0, 0.0, pi / 4.0)),
        material=steel,
        name="spoke_diagonal",
    )
    handwheel.inertial = Inertial.from_geometry(
        Box((0.13, 0.05, 0.13)),
        mass=0.5,
        origin=Origin(xyz=(0.0, 0.0, 0.0)),
    )

    model.articulation(
        "pedestal_to_yaw_head",
        ArticulationType.CONTINUOUS,
        parent=pedestal,
        child=yaw_head,
        origin=Origin(xyz=(0.0, 0.0, 0.76)),
        axis=(0.0, 0.0, 1.0),
        motion_limits=MotionLimits(effort=70.0, velocity=1.5),
    )
    model.articulation(
        "yaw_head_to_launcher",
        ArticulationType.REVOLUTE,
        parent=yaw_head,
        child=launcher,
        origin=Origin(xyz=(0.0, 0.0, 0.30)),
        axis=(0.0, -1.0, 0.0),
        motion_limits=MotionLimits(effort=55.0, velocity=1.1, lower=-0.20, upper=0.95),
    )
    model.articulation(
        "launcher_to_handwheel",
        ArticulationType.CONTINUOUS,
        parent=launcher,
        child=handwheel,
        origin=Origin(xyz=(-0.145, 0.34, 0.025)),
        axis=(0.0, 1.0, 0.0),
        motion_limits=MotionLimits(effort=3.0, velocity=8.0),
    )

    return model


def run_tests() -> TestReport:
    ctx = TestContext(object_model)
    pedestal = object_model.get_part("pedestal")
    yaw_head = object_model.get_part("yaw_head")
    launcher = object_model.get_part("launcher_carriage")
    handwheel = object_model.get_part("handwheel")

    yaw_joint = object_model.get_articulation("pedestal_to_yaw_head")
    elevation_joint = object_model.get_articulation("yaw_head_to_launcher")
    wheel_joint = object_model.get_articulation("launcher_to_handwheel")

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
    ctx.allow_overlap(
        launcher,
        yaw_head,
        elem_a="left_trunnion_cap",
        elem_b="left_trunnion_bearing",
        reason="Launcher trunnion cap is intentionally seated into the yaw-head bearing collar.",
    )
    ctx.allow_overlap(
        launcher,
        yaw_head,
        elem_a="right_trunnion_cap",
        elem_b="right_trunnion_bearing",
        reason="Launcher trunnion cap is intentionally seated into the yaw-head bearing collar.",
    )
    ctx.allow_overlap(
        launcher,
        yaw_head,
        elem_a="left_trunnion_cap",
        elem_b="left_yoke_arm",
        reason="The left trunnion cap passes through an implied bored opening in the yoke arm that is not cut out in this simplified visual plate.",
    )
    ctx.allow_overlap(
        launcher,
        yaw_head,
        elem_a="right_trunnion_cap",
        elem_b="right_yoke_arm",
        reason="The right trunnion cap passes through an implied bored opening in the yoke arm that is not cut out in this simplified visual plate.",
    )
    ctx.fail_if_parts_overlap_in_current_pose()

    ctx.check("yaw_axis_vertical", tuple(yaw_joint.axis) == (0.0, 0.0, 1.0), details=f"axis={yaw_joint.axis}")
    ctx.check(
        "elevation_axis_horizontal",
        abs(elevation_joint.axis[1]) > 0.99 and abs(elevation_joint.axis[0]) < 1e-6 and abs(elevation_joint.axis[2]) < 1e-6,
        details=f"axis={elevation_joint.axis}",
    )
    ctx.check(
        "handwheel_axis_local",
        tuple(wheel_joint.axis) == (0.0, 1.0, 0.0),
        details=f"axis={wheel_joint.axis}",
    )

    ctx.expect_origin_distance(yaw_head, pedestal, axes="xy", max_dist=0.001, name="yaw_head_centered_on_pedestal")
    ctx.expect_gap(yaw_head, pedestal, axis="z", max_gap=0.001, max_penetration=0.0, name="yaw_head_seats_on_bearing")
    ctx.expect_overlap(launcher, yaw_head, axes="y", min_overlap=0.22, name="launcher_carried_between_yoke_arms")
    ctx.expect_contact(
        yaw_head,
        launcher,
        elem_a="left_trunnion_bearing",
        elem_b="left_trunnion_cap",
        name="left_trunnion_is_seated",
    )
    ctx.expect_contact(
        yaw_head,
        launcher,
        elem_a="right_trunnion_bearing",
        elem_b="right_trunnion_cap",
        name="right_trunnion_is_seated",
    )
    ctx.expect_contact(handwheel, launcher, contact_tol=0.0005, name="handwheel_is_mounted")

    left_front_rest = ctx.part_element_world_aabb(launcher, elem="left_tube_front_rim")
    right_front_rest = ctx.part_element_world_aabb(launcher, elem="right_tube_front_rim")
    spoke_rest = ctx.part_element_world_aabb(handwheel, elem="spoke_horizontal")

    if left_front_rest is not None:
        rest_top_z = left_front_rest[1][2]
        with ctx.pose({elevation_joint: 0.80}):
            left_front_up = ctx.part_element_world_aabb(launcher, elem="left_tube_front_rim")
            if left_front_up is not None:
                ctx.check(
                    "launcher_front_rises_with_elevation",
                    left_front_up[1][2] > rest_top_z + 0.35,
                    details=f"rest_top_z={rest_top_z:.3f}, elevated_top_z={left_front_up[1][2]:.3f}",
                )
            ctx.expect_gap(
                launcher,
                pedestal,
                axis="z",
                min_gap=0.12,
                name="elevated_launcher_clears_pedestal",
            )
            ctx.expect_contact(handwheel, launcher, contact_tol=0.0005, name="handwheel_stays_mounted_elevated")

    if right_front_rest is not None:
        rest_center_y = 0.5 * (right_front_rest[0][1] + right_front_rest[1][1])
        with ctx.pose({yaw_joint: 0.75}):
            right_front_yawed = ctx.part_element_world_aabb(launcher, elem="right_tube_front_rim")
            if right_front_yawed is not None:
                yawed_center_y = 0.5 * (right_front_yawed[0][1] + right_front_yawed[1][1])
                ctx.check(
                    "launcher_swings_in_yaw",
                    abs(yawed_center_y - rest_center_y) > 0.55,
                    details=f"rest_center_y={rest_center_y:.3f}, yawed_center_y={yawed_center_y:.3f}",
                )

    if spoke_rest is not None:
        rest_x_span = spoke_rest[1][0] - spoke_rest[0][0]
        with ctx.pose({wheel_joint: pi / 2.0}):
            spoke_rotated = ctx.part_element_world_aabb(handwheel, elem="spoke_horizontal")
            if spoke_rotated is not None:
                rotated_z_span = spoke_rotated[1][2] - spoke_rotated[0][2]
                ctx.check(
                    "handwheel_rotates_about_local_axis",
                    rotated_z_span > rest_x_span * 0.75,
                    details=f"rest_x_span={rest_x_span:.3f}, rotated_z_span={rotated_z_span:.3f}",
                )
            ctx.expect_contact(handwheel, launcher, contact_tol=0.0005, name="handwheel_contact_after_spin")

    return ctx.report()


# >>> USER_CODE_END

object_model = build_object_model()
