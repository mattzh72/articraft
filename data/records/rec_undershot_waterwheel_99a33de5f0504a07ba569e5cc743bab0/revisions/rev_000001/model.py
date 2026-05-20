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
    LatheGeometry,
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


def _add_bolt_y(
    part,
    *,
    x: float,
    y: float,
    z: float,
    radius: float,
    length: float,
    material,
    name: str | None = None,
) -> None:
    part.visual(
        Cylinder(radius=radius, length=length),
        origin=Origin(xyz=(x, y, z), rpy=(math.pi / 2.0, 0.0, 0.0)),
        material=material,
        name=name,
    )


def build_object_model() -> ArticulatedObject:
    model = ArticulatedObject(name="industrial_undershot_waterwheel")

    frame_gray = model.material("frame_gray", rgba=(0.33, 0.36, 0.40, 1.0))
    dark_steel = model.material("dark_steel", rgba=(0.18, 0.19, 0.21, 1.0))
    guard_yellow = model.material("guard_yellow", rgba=(0.78, 0.67, 0.17, 1.0))
    worn_steel = model.material("worn_steel", rgba=(0.54, 0.57, 0.60, 1.0))
    oxide_red = model.material("oxide_red", rgba=(0.58, 0.18, 0.13, 1.0))
    fastener_steel = model.material("fastener_steel", rgba=(0.70, 0.72, 0.75, 1.0))

    bearing_shell_mesh = mesh_from_geometry(
        LatheGeometry.from_shell_profiles(
            outer_profile=[
                (0.132, -0.095),
                (0.155, -0.075),
                (0.155, 0.075),
                (0.132, 0.095),
            ],
            inner_profile=[
                (0.082, -0.070),
                (0.082, 0.070),
            ],
            segments=56,
            start_cap="flat",
            end_cap="flat",
        ),
        "bearing_shell",
    )
    rim_mesh = mesh_from_geometry(
        LatheGeometry.from_shell_profiles(
            outer_profile=[
                (1.18, -0.05),
                (1.18, 0.05),
            ],
            inner_profile=[
                (1.08, -0.05),
                (1.08, 0.05),
            ],
            segments=72,
            start_cap="flat",
            end_cap="flat",
        ),
        "wheel_rim",
    )

    support_frame = model.part("support_frame")
    for y_sign, y in ((1, 1.02), (-1, -1.02)):
        support_frame.visual(
            Box((3.45, 0.20, 0.16)),
            origin=Origin(xyz=(0.0, y, 0.08)),
            material=frame_gray,
            name=f"base_rail_{'left' if y_sign > 0 else 'right'}",
        )
    for x_sign, x in ((1, 1.38), (-1, -1.38)):
        support_frame.visual(
            Box((0.24, 2.24, 0.20)),
            origin=Origin(xyz=(x, 0.0, 0.10)),
            material=frame_gray,
            name=f"cross_tie_{'front' if x_sign > 0 else 'rear'}",
        )
    support_frame.visual(
        Box((0.22, 2.02, 0.20)),
        origin=Origin(xyz=(0.0, 0.0, 0.20)),
        material=frame_gray,
        name="center_tie",
    )
    for side_name, y in (("left", 1.02), ("right", -1.02)):
        for x_name, x in (("front", 0.88), ("rear", -0.88)):
            support_frame.visual(
                Box((0.20, 0.18, 0.68)),
                origin=Origin(xyz=(x, y, 0.42)),
                material=frame_gray,
                name=f"{side_name}_{x_name}_leg",
            )
        support_frame.visual(
            Box((1.82, 0.14, 0.14)),
            origin=Origin(xyz=(0.0, y, 0.77)),
            material=frame_gray,
            name=f"{side_name}_side_stringer",
        )
        support_frame.visual(
            Box((0.48, 0.10, 0.94)),
            origin=Origin(xyz=(0.0, 0.97 if side_name == "left" else -0.97, 1.09)),
            material=frame_gray,
            name=f"{side_name}_pedestal_plate",
        )
        support_frame.visual(
            Box((0.62, 0.20, 0.18)),
            origin=Origin(xyz=(0.0, y, 1.58)),
            material=frame_gray,
            name=f"{side_name}_saddle_beam",
        )
        _add_member(
            support_frame,
            (0.88, y, 0.76),
            (0.18, y, 1.56),
            radius=0.060,
            material=frame_gray,
            name=f"{side_name}_front_brace",
        )
        _add_member(
            support_frame,
            (-0.88, y, 0.76),
            (-0.18, y, 1.56),
            radius=0.060,
            material=frame_gray,
            name=f"{side_name}_rear_brace",
        )
        support_frame.visual(
            Box((0.28, 0.10, 0.32)),
            origin=Origin(xyz=(0.0, 0.95 if side_name == "left" else -0.95, 1.42)),
            material=frame_gray,
            name=f"{side_name}_bearing_saddle",
        )

    support_frame.visual(
        bearing_shell_mesh,
        origin=Origin(xyz=(0.0, 0.84, 1.58), rpy=(math.pi / 2.0, 0.0, 0.0)),
        material=dark_steel,
        name="left_bearing_housing",
    )
    support_frame.visual(
        bearing_shell_mesh,
        origin=Origin(xyz=(0.0, -0.84, 1.58), rpy=(math.pi / 2.0, 0.0, 0.0)),
        material=dark_steel,
        name="right_bearing_housing",
    )
    for side_name, y in (("left", 0.75), ("right", -0.75)):
        for x in (-0.16, 0.16):
            support_frame.visual(
                Box((0.08, 0.10, 0.20)),
                origin=Origin(xyz=(x, y, 1.58)),
                material=frame_gray,
                name=f"{side_name}_bearing_ear_{'outer' if x > 0 else 'inner'}",
            )
        for idx, (x, z) in enumerate(
            ((-0.16, 1.68), (0.16, 1.68), (-0.16, 1.48), (0.16, 1.48))
        ):
            _add_bolt_y(
                support_frame,
                x=x,
                y=y,
                z=z,
                radius=0.018,
                length=0.12,
                material=fastener_steel,
                name=f"{side_name}_bearing_bolt_{idx}",
            )

    support_frame.visual(
        Box((0.05, 0.04, 0.08)),
        origin=Origin(xyz=(0.125, 0.54, 1.58)),
        material=dark_steel,
        name="left_thrust_stop",
    )
    support_frame.visual(
        Box((0.05, 0.47, 0.08)),
        origin=Origin(xyz=(0.125, 0.775, 1.58)),
        material=dark_steel,
        name="left_thrust_stop_carrier_outer",
    )
    support_frame.visual(
        Box((0.05, 0.04, 0.08)),
        origin=Origin(xyz=(-0.125, 0.54, 1.58)),
        material=dark_steel,
        name="left_thrust_stop_inner",
    )
    support_frame.visual(
        Box((0.05, 0.47, 0.08)),
        origin=Origin(xyz=(-0.125, 0.775, 1.58)),
        material=dark_steel,
        name="left_thrust_stop_carrier_inner",
    )
    support_frame.visual(
        Box((0.05, 0.04, 0.08)),
        origin=Origin(xyz=(0.125, -0.54, 1.58)),
        material=dark_steel,
        name="right_thrust_stop",
    )
    support_frame.visual(
        Box((0.05, 0.47, 0.08)),
        origin=Origin(xyz=(0.125, -0.775, 1.58)),
        material=dark_steel,
        name="right_thrust_stop_carrier_outer",
    )
    support_frame.visual(
        Box((0.05, 0.04, 0.08)),
        origin=Origin(xyz=(-0.125, -0.54, 1.58)),
        material=dark_steel,
        name="right_thrust_stop_inner",
    )
    support_frame.visual(
        Box((0.05, 0.47, 0.08)),
        origin=Origin(xyz=(-0.125, -0.775, 1.58)),
        material=dark_steel,
        name="right_thrust_stop_carrier_inner",
    )
    for y_name, y in (("left", 1.02), ("right", -1.02)):
        for x_name, x in (("front", 0.88), ("rear", -0.88)):
            for idx, dx in enumerate((-0.05, 0.05)):
                for jdx, dy in enumerate((-0.05, 0.05)):
                    support_frame.visual(
                        Cylinder(radius=0.014, length=0.08),
                        origin=Origin(xyz=(x + dx, y + dy, 0.20)),
                        material=fastener_steel,
                        name=f"{y_name}_{x_name}_anchor_{idx}_{jdx}",
                    )

    wheel = model.part("wheel")
    wheel.visual(
        Cylinder(radius=0.055, length=1.80),
        origin=Origin(rpy=(math.pi / 2.0, 0.0, 0.0)),
        material=dark_steel,
        name="axle",
    )
    wheel.visual(
        Cylinder(radius=0.18, length=1.02),
        origin=Origin(rpy=(math.pi / 2.0, 0.0, 0.0)),
        material=dark_steel,
        name="hub_barrel",
    )
    wheel.visual(
        Cylinder(radius=0.30, length=0.16),
        origin=Origin(xyz=(0.0, 0.43, 0.0), rpy=(math.pi / 2.0, 0.0, 0.0)),
        material=worn_steel,
        name="left_hub_plate",
    )
    wheel.visual(
        Cylinder(radius=0.30, length=0.16),
        origin=Origin(xyz=(0.0, -0.43, 0.0), rpy=(math.pi / 2.0, 0.0, 0.0)),
        material=worn_steel,
        name="right_hub_plate",
    )
    wheel.visual(
        Box((0.44, 0.86, 0.10)),
        origin=Origin(),
        material=dark_steel,
        name="hub_web_horizontal",
    )
    wheel.visual(
        Box((0.10, 0.86, 0.44)),
        origin=Origin(),
        material=dark_steel,
        name="hub_web_vertical",
    )
    wheel.visual(
        Box((0.18, 0.04, 0.18)),
        origin=Origin(xyz=(0.0, 0.63, 0.0)),
        material=dark_steel,
        name="left_collar",
    )
    wheel.visual(
        Box((0.18, 0.04, 0.18)),
        origin=Origin(xyz=(0.0, -0.63, 0.0)),
        material=dark_steel,
        name="right_collar",
    )
    wheel.visual(
        rim_mesh,
        origin=Origin(xyz=(0.0, 0.49, 0.0), rpy=(math.pi / 2.0, 0.0, 0.0)),
        material=worn_steel,
        name="left_rim",
    )
    wheel.visual(
        rim_mesh,
        origin=Origin(xyz=(0.0, -0.49, 0.0), rpy=(math.pi / 2.0, 0.0, 0.0)),
        material=worn_steel,
        name="right_rim",
    )
    for side_name, y in (("left", 0.49), ("right", -0.49)):
        for index in range(8):
            angle = (2.0 * math.pi * index) / 8.0
            _add_member(
                wheel,
                (math.sin(angle) * 0.18, y, math.cos(angle) * 0.18),
                (math.sin(angle) * 1.10, y, math.cos(angle) * 1.10),
                radius=0.055,
                material=dark_steel,
                name=f"{side_name}_spoke_{index}",
            )
    for index in range(12):
        angle = (2.0 * math.pi * index) / 12.0
        wheel.visual(
            Box((0.22, 1.02, 0.08)),
            origin=Origin(
                xyz=(math.sin(angle) * 1.18, 0.0, math.cos(angle) * 1.18),
                rpy=(0.0, angle, 0.0),
            ),
            material=worn_steel,
            name=f"paddle_{index:02d}",
        )
        wheel.visual(
            Box((0.12, 0.18, 0.14)),
            origin=Origin(
                xyz=(math.sin(angle) * 1.10, 0.45, math.cos(angle) * 1.10),
                rpy=(0.0, angle, 0.0),
            ),
            material=dark_steel,
            name=f"left_paddle_clamp_{index:02d}",
        )
        wheel.visual(
            Box((0.12, 0.18, 0.14)),
            origin=Origin(
                xyz=(math.sin(angle) * 1.10, -0.45, math.cos(angle) * 1.10),
                rpy=(0.0, angle, 0.0),
            ),
            material=dark_steel,
            name=f"right_paddle_clamp_{index:02d}",
        )

    top_guard = model.part("top_guard")
    top_guard.visual(
        Box((3.10, 2.30, 0.10)),
        origin=Origin(xyz=(0.0, 0.0, 3.10)),
        material=guard_yellow,
        name="roof_panel",
    )
    top_guard.visual(
        Box((0.08, 2.30, 1.18)),
        origin=Origin(xyz=(1.51, 0.0, 2.46)),
        material=guard_yellow,
        name="front_guard_panel",
    )
    top_guard.visual(
        Box((0.08, 2.30, 1.18)),
        origin=Origin(xyz=(-1.51, 0.0, 2.46)),
        material=guard_yellow,
        name="rear_guard_panel",
    )
    top_guard.visual(
        Box((3.02, 0.08, 1.02)),
        origin=Origin(xyz=(0.0, 1.14, 2.53)),
        material=guard_yellow,
        name="left_guard_panel",
    )
    top_guard.visual(
        Box((3.02, 0.08, 1.02)),
        origin=Origin(xyz=(0.0, -1.14, 2.53)),
        material=guard_yellow,
        name="right_guard_panel",
    )
    for side_name, y_outer, y_inner in (("left", 1.14, 1.02), ("right", -1.14, -1.02)):
        for x_name, x in (("front", 0.22), ("rear", -0.22)):
            top_guard.visual(
                Box((0.12, 0.16, 0.68)),
                origin=Origin(xyz=(x, (y_outer + y_inner) * 0.5, 2.06)),
                material=frame_gray,
                name=f"{side_name}_{x_name}_hanger",
            )
            top_guard.visual(
                Box((0.24, 0.12, 0.10)),
                origin=Origin(xyz=(x, y_inner, 1.72)),
                material=frame_gray,
                name=f"{side_name}_{x_name}_mount_tab",
            )
            _add_bolt_y(
                top_guard,
                x=x,
                y=y_inner,
                z=1.72,
                radius=0.016,
                length=0.12,
                material=fastener_steel,
                name=f"{side_name}_{x_name}_mount_bolt",
            )

    inlet_guard = model.part("inlet_guard")
    inlet_guard.visual(
        Box((0.08, 1.84, 1.08)),
        origin=Origin(xyz=(1.60, 0.0, 0.90)),
        material=guard_yellow,
        name="debris_screen",
    )
    inlet_guard.visual(
        Box((0.08, 1.84, 0.12)),
        origin=Origin(xyz=(1.56, 0.0, 0.24)),
        material=frame_gray,
        name="toe_plate",
    )
    inlet_guard.visual(
        Box((0.24, 1.84, 0.14)),
        origin=Origin(xyz=(1.50, 0.0, 1.53)),
        material=guard_yellow,
        name="screen_top_hood",
    )
    for side_name, y in (("left", 0.78), ("right", -0.78)):
        inlet_guard.visual(
            Box((0.36, 0.18, 0.18)),
            origin=Origin(xyz=(1.34, y, 0.29)),
            material=frame_gray,
            name=f"{side_name}_guard_foot",
        )
        inlet_guard.visual(
            Box((0.12, 0.12, 0.96)),
            origin=Origin(xyz=(1.50, y, 0.72)),
            material=frame_gray,
            name=f"{side_name}_guard_post",
        )
        _add_member(
            inlet_guard,
            (1.32, y, 0.34),
            (1.48, y, 1.48),
            radius=0.035,
            material=frame_gray,
            name=f"{side_name}_diagonal_brace",
        )

    service_lockout = model.part("service_lockout")
    service_lockout.visual(
        Box((0.12, 0.14, 0.28)),
        origin=Origin(xyz=(0.0, -1.19, 1.58)),
        material=oxide_red,
        name="mount_block",
    )
    service_lockout.visual(
        Box((0.56, 0.02, 0.64)),
        origin=Origin(xyz=(0.18, -1.24, 1.58)),
        material=oxide_red,
        name="lockout_plate",
    )
    service_lockout.visual(
        Box((0.10, 0.10, 0.12)),
        origin=Origin(xyz=(0.42, -1.19, 1.86)),
        material=oxide_red,
        name="upper_stop_lug",
    )
    service_lockout.visual(
        Box((0.10, 0.10, 0.12)),
        origin=Origin(xyz=(0.42, -1.19, 1.30)),
        material=oxide_red,
        name="lower_stop_lug",
    )
    _add_bolt_y(
        service_lockout,
        x=0.05,
        y=-1.19,
        z=1.69,
        radius=0.015,
        length=0.14,
        material=fastener_steel,
        name="upper_mount_bolt",
    )
    _add_bolt_y(
        service_lockout,
        x=0.05,
        y=-1.19,
        z=1.47,
        radius=0.015,
        length=0.14,
        material=fastener_steel,
        name="lower_mount_bolt",
    )
    service_lockout.visual(
        Cylinder(radius=0.028, length=0.16),
        origin=Origin(xyz=(0.42, -1.23, 1.90), rpy=(math.pi / 2.0, 0.0, 0.0)),
        material=dark_steel,
        name="pin_stow_tube",
    )
    service_lockout.visual(
        Cylinder(radius=0.010, length=0.18),
        origin=Origin(xyz=(0.42, -1.23, 1.90), rpy=(math.pi / 2.0, 0.0, 0.0)),
        material=fastener_steel,
        name="lockout_pin",
    )

    model.articulation(
        "wheel_rotation",
        ArticulationType.CONTINUOUS,
        parent=support_frame,
        child=wheel,
        origin=Origin(xyz=(0.0, 0.0, 1.58)),
        axis=(0.0, 1.0, 0.0),
        motion_limits=MotionLimits(effort=18000.0, velocity=1.2),
    )
    model.articulation(
        "support_to_top_guard",
        ArticulationType.FIXED,
        parent=support_frame,
        child=top_guard,
        origin=Origin(),
    )
    model.articulation(
        "support_to_inlet_guard",
        ArticulationType.FIXED,
        parent=support_frame,
        child=inlet_guard,
        origin=Origin(),
    )
    model.articulation(
        "support_to_service_lockout",
        ArticulationType.FIXED,
        parent=support_frame,
        child=service_lockout,
        origin=Origin(),
    )

    return model


def run_tests() -> TestReport:
    ctx = TestContext(object_model)
    support_frame = object_model.get_part("support_frame")
    wheel = object_model.get_part("wheel")
    top_guard = object_model.get_part("top_guard")
    inlet_guard = object_model.get_part("inlet_guard")
    service_lockout = object_model.get_part("service_lockout")
    wheel_rotation = object_model.get_articulation("wheel_rotation")

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

    ctx.check(
        "wheel_rotation_joint_is_continuous",
        wheel_rotation.articulation_type == ArticulationType.CONTINUOUS
        and wheel_rotation.axis == (0.0, 1.0, 0.0),
        details=f"Expected centered continuous axle rotation on +Y, got {wheel_rotation.articulation_type} axis={wheel_rotation.axis}",
    )

    ctx.expect_contact(top_guard, support_frame, name="top_guard_is_mounted")
    ctx.expect_contact(inlet_guard, support_frame, name="inlet_guard_is_mounted")
    ctx.expect_contact(service_lockout, support_frame, name="lockout_is_mounted")

    ctx.expect_overlap(
        wheel,
        support_frame,
        axes="xz",
        min_overlap=0.10,
        elem_a="axle",
        elem_b="left_bearing_housing",
        name="left_bearing_tracks_axle_centerline",
    )
    ctx.expect_overlap(
        wheel,
        support_frame,
        axes="xz",
        min_overlap=0.10,
        elem_a="axle",
        elem_b="right_bearing_housing",
        name="right_bearing_tracks_axle_centerline",
    )

    ctx.expect_gap(
        top_guard,
        wheel,
        axis="z",
        min_gap=0.12,
        max_gap=0.40,
        positive_elem="roof_panel",
        name="roof_clears_rotating_wheel",
    )
    ctx.expect_gap(
        inlet_guard,
        wheel,
        axis="x",
        min_gap=0.18,
        max_gap=0.40,
        positive_elem="debris_screen",
        name="inlet_guard_clears_front_quadrant",
    )
    ctx.expect_gap(
        wheel,
        support_frame,
        axis="y",
        min_gap=0.04,
        max_gap=0.12,
        positive_elem="left_collar",
        negative_elem="left_thrust_stop",
        name="left_axial_overtravel_gap",
    )
    ctx.expect_gap(
        support_frame,
        wheel,
        axis="y",
        min_gap=0.04,
        max_gap=0.12,
        positive_elem="right_thrust_stop",
        negative_elem="right_collar",
        name="right_axial_overtravel_gap",
    )

    wheel_pos = ctx.part_world_position(wheel)
    left_aabb = ctx.part_element_world_aabb(support_frame, elem="left_bearing_housing")
    right_aabb = ctx.part_element_world_aabb(support_frame, elem="right_bearing_housing")
    if wheel_pos is not None and left_aabb is not None and right_aabb is not None:
        left_inner_y = left_aabb[0][1]
        right_inner_y = right_aabb[1][1]
        left_center_x = (left_aabb[0][0] + left_aabb[1][0]) * 0.5
        right_center_x = (right_aabb[0][0] + right_aabb[1][0]) * 0.5
        left_center_z = (left_aabb[0][2] + left_aabb[1][2]) * 0.5
        right_center_z = (right_aabb[0][2] + right_aabb[1][2]) * 0.5
        clearance_balance = abs((left_inner_y - wheel_pos[1]) - (wheel_pos[1] - right_inner_y))
        ctx.check(
            "wheel_axle_centered_between_side_bearings",
            clearance_balance < 0.03
            and abs(left_center_x - wheel_pos[0]) < 0.02
            and abs(right_center_x - wheel_pos[0]) < 0.02
            and abs(left_center_z - wheel_pos[2]) < 0.02
            and abs(right_center_z - wheel_pos[2]) < 0.02,
            details=(
                "Wheel axle is not centered between side bearings: "
                f"balance={clearance_balance:.4f}, "
                f"left_center=({left_center_x:.3f},{left_center_z:.3f}), "
                f"right_center=({right_center_x:.3f},{right_center_z:.3f}), "
                f"wheel=({wheel_pos[0]:.3f},{wheel_pos[2]:.3f})"
            ),
        )
    else:
        ctx.fail(
            "wheel_axle_centered_between_side_bearings",
            "Could not evaluate bearing placement against wheel axle center.",
        )

    return ctx.report()


# >>> USER_CODE_END

object_model = build_object_model()
