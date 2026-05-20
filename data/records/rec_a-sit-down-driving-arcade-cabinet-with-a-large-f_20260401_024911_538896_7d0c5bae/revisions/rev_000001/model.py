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
    ExtrudeGeometry,
    Inertial,
    LoftGeometry,
    MotionLimits,
    Origin,
    TestContext,
    TestReport,
    TorusGeometry,
    mesh_from_geometry,
    rounded_rect_profile,
)


def _save_mesh(name: str, geometry):
    return mesh_from_geometry(geometry, name)


def _side_panel_mesh(profile: list[tuple[float, float]], *, thickness: float, side: str, outer_y: float):
    panel = ExtrudeGeometry.from_z0(profile, thickness, cap=True, closed=True).rotate_x(pi / 2.0)
    if side == "left":
        panel.translate(0.0, outer_y, 0.0)
    else:
        panel.scale(1.0, -1.0, 1.0).translate(0.0, -outer_y, 0.0)
    return panel


def _offset_profile(
    profile: list[tuple[float, float]],
    *,
    sx: float = 1.0,
    sy: float = 1.0,
    dx: float = 0.0,
    dy: float = 0.0,
) -> list[tuple[float, float]]:
    return [(sx * x + dx, sy * y + dy) for x, y in profile]


def build_object_model() -> ArticulatedObject:
    model = ArticulatedObject(name="sitdown_arcade_racer")

    cabinet_shell = model.material("cabinet_shell", rgba=(0.19, 0.23, 0.28, 1.0))
    cabinet_trim = model.material("cabinet_trim", rgba=(0.63, 0.67, 0.72, 1.0))
    dash_black = model.material("dash_black", rgba=(0.10, 0.11, 0.12, 1.0))
    seat_vinyl = model.material("seat_vinyl", rgba=(0.12, 0.12, 0.13, 1.0))
    screen_glass = model.material("screen_glass", rgba=(0.10, 0.21, 0.28, 0.55))
    wheel_rubber = model.material("wheel_rubber", rgba=(0.06, 0.06, 0.07, 1.0))
    wheel_core = model.material("wheel_core", rgba=(0.18, 0.19, 0.21, 1.0))

    cabinet_body = model.part("cabinet_body")
    cabinet_body.inertial = Inertial.from_geometry(
        Box((1.72, 0.98, 1.78)),
        mass=235.0,
        origin=Origin(xyz=(0.04, 0.0, 0.89)),
    )

    cabinet_body.visual(
        Box((1.62, 0.94, 0.10)),
        origin=Origin(xyz=(0.00, 0.00, 0.05)),
        material=cabinet_shell,
        name="base_plinth",
    )

    side_profile = [
        (-0.78, 0.00),
        (-0.78, 0.42),
        (-0.67, 0.78),
        (-0.36, 1.18),
        (-0.04, 1.49),
        (0.32, 1.70),
        (0.74, 1.62),
        (0.86, 1.34),
        (0.86, 0.40),
        (0.61, 0.35),
        (0.34, 0.29),
        (-0.06, 0.27),
        (-0.43, 0.28),
        (-0.65, 0.18),
    ]
    cabinet_body.visual(
        _save_mesh(
            "arcade_left_side_panel",
            _side_panel_mesh(side_profile, thickness=0.028, side="left", outer_y=0.47),
        ),
        material=cabinet_shell,
        name="left_side_panel",
    )
    cabinet_body.visual(
        _save_mesh(
            "arcade_right_side_panel",
            _side_panel_mesh(side_profile, thickness=0.028, side="right", outer_y=0.47),
        ),
        material=cabinet_shell,
        name="right_side_panel",
    )

    cabinet_body.visual(
        Box((0.88, 0.58, 0.05)),
        origin=Origin(xyz=(-0.08, 0.00, 0.125)),
        material=dash_black,
        name="cockpit_floor",
    )
    cabinet_body.visual(
        Box((0.36, 0.56, 0.08)),
        origin=Origin(xyz=(0.45, 0.00, 0.16)),
        material=dash_black,
        name="pedal_deck",
    )
    cabinet_body.visual(
        Box((0.30, 0.88, 0.34)),
        origin=Origin(xyz=(0.72, 0.00, 0.17)),
        material=cabinet_shell,
        name="front_lower_housing",
    )
    cabinet_body.visual(
        Box((0.28, 0.82, 0.40)),
        origin=Origin(xyz=(0.20, 0.00, 0.40)),
        material=cabinet_shell,
        name="dashboard_pedestal",
    )
    cabinet_body.visual(
        Box((0.42, 0.78, 0.10)),
        origin=Origin(xyz=(0.13, 0.00, 0.69), rpy=(0.0, -0.40, 0.0)),
        material=dash_black,
        name="dashboard_top",
    )
    cabinet_body.visual(
        Box((0.18, 0.70, 0.04)),
        origin=Origin(xyz=(0.11, 0.00, 0.585)),
        material=dash_black,
        name="dashboard_bridge",
    )
    cabinet_body.visual(
        Box((0.12, 0.18, 0.20)),
        origin=Origin(xyz=(0.06, 0.00, 0.70)),
        material=dash_black,
        name="column_mount",
    )
    cabinet_body.visual(
        Cylinder(radius=0.060, length=0.12),
        origin=Origin(xyz=(0.06, 0.00, 0.82), rpy=(0.0, pi / 2.0, 0.0)),
        material=dash_black,
        name="steering_column_sleeve",
    )
    cabinet_body.visual(
        Box((0.46, 0.88, 0.58)),
        origin=Origin(xyz=(0.47, 0.00, 1.13), rpy=(0.0, -0.10, 0.0)),
        material=cabinet_shell,
        name="monitor_housing",
    )
    cabinet_body.visual(
        Box((0.18, 0.82, 0.28)),
        origin=Origin(xyz=(0.31, 0.00, 0.74)),
        material=cabinet_shell,
        name="monitor_riser",
    )
    cabinet_body.visual(
        Box((0.24, 0.88, 0.20)),
        origin=Origin(xyz=(0.50, 0.00, 1.54)),
        material=cabinet_shell,
        name="marquee_header",
    )
    cabinet_body.visual(
        Box((0.18, 0.82, 0.03)),
        origin=Origin(xyz=(0.50, 0.00, 1.43)),
        material=cabinet_shell,
        name="marquee_brace",
    )
    cabinet_body.visual(
        Box((0.10, 0.84, 0.12)),
        origin=Origin(xyz=(0.74, 0.00, 1.48)),
        material=cabinet_trim,
        name="bezel_top",
    )
    cabinet_body.visual(
        Box((0.10, 0.84, 0.10)),
        origin=Origin(xyz=(0.74, 0.00, 0.96)),
        material=cabinet_trim,
        name="bezel_bottom",
    )
    cabinet_body.visual(
        Box((0.10, 0.10, 0.42)),
        origin=Origin(xyz=(0.74, 0.37, 1.22)),
        material=cabinet_trim,
        name="bezel_left",
    )
    cabinet_body.visual(
        Box((0.10, 0.10, 0.42)),
        origin=Origin(xyz=(0.74, -0.37, 1.22)),
        material=cabinet_trim,
        name="bezel_right",
    )
    cabinet_body.visual(
        Box((0.02, 0.68, 0.40)),
        origin=Origin(xyz=(0.68, 0.00, 1.22)),
        material=screen_glass,
        name="screen",
    )
    cabinet_body.visual(
        Box((0.24, 0.50, 0.32)),
        origin=Origin(xyz=(-0.44, 0.00, 0.26)),
        material=cabinet_shell,
        name="seat_pedestal",
    )
    seat_base_profile = rounded_rect_profile(0.52, 0.60, 0.09, corner_segments=8)
    seat_top_profile = _offset_profile(seat_base_profile, sx=0.82, sy=0.86, dx=-0.05)
    cabinet_body.visual(
        _save_mesh(
            "arcade_seat_cushion",
            LoftGeometry(
                [
                    [(x, y, 0.0) for x, y in seat_base_profile],
                    [(x, y, 0.12) for x, y in seat_top_profile],
                ],
                cap=True,
                closed=True,
            ),
        ),
        origin=Origin(xyz=(-0.33, 0.00, 0.42), rpy=(0.0, -0.10, 0.0)),
        material=seat_vinyl,
        name="seat_cushion",
    )
    seat_back_profile = rounded_rect_profile(0.58, 0.62, 0.08, corner_segments=8)
    cabinet_body.visual(
        _save_mesh(
            "arcade_seat_back",
            ExtrudeGeometry.centered(seat_back_profile, 0.16, cap=True, closed=True).rotate_x(pi / 2.0).rotate_z(pi / 2.0),
        ),
        origin=Origin(xyz=(-0.44, 0.00, 0.47), rpy=(0.0, 0.18, 0.0)),
        material=seat_vinyl,
        name="seat_back",
    )

    steering_wheel = model.part("steering_wheel")
    steering_wheel.inertial = Inertial.from_geometry(
        Cylinder(radius=0.19, length=0.10),
        mass=4.5,
        origin=Origin(rpy=(0.0, pi / 2.0, 0.0)),
    )
    steering_wheel.visual(
        _save_mesh(
            "arcade_wheel_rim",
            TorusGeometry(radius=0.17, tube=0.018, radial_segments=18, tubular_segments=40).rotate_y(pi / 2.0),
        ),
        material=wheel_rubber,
        name="rim",
    )
    steering_wheel.visual(
        Cylinder(radius=0.055, length=0.08),
        origin=Origin(rpy=(0.0, pi / 2.0, 0.0)),
        material=wheel_core,
        name="hub",
    )
    steering_wheel.visual(
        Cylinder(radius=0.026, length=0.10),
        origin=Origin(xyz=(0.07, 0.00, 0.00), rpy=(0.0, pi / 2.0, 0.0)),
        material=wheel_core,
        name="shaft_stub",
    )
    steering_wheel.visual(
        Box((0.032, 0.032, 0.32)),
        origin=Origin(),
        material=wheel_core,
        name="vertical_spoke",
    )
    steering_wheel.visual(
        Box((0.032, 0.32, 0.032)),
        origin=Origin(),
        material=wheel_core,
        name="horizontal_spoke",
    )

    model.articulation(
        "body_to_steering_wheel",
        ArticulationType.CONTINUOUS,
        parent=cabinet_body,
        child=steering_wheel,
        origin=Origin(xyz=(-0.12, 0.00, 0.82)),
        axis=(1.0, 0.0, 0.0),
        motion_limits=MotionLimits(effort=16.0, velocity=8.0),
    )

    return model


def run_tests() -> TestReport:
    ctx = TestContext(object_model)
    cabinet_body = object_model.get_part("cabinet_body")
    steering_wheel = object_model.get_part("steering_wheel")
    steering_joint = object_model.get_articulation("body_to_steering_wheel")

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

    body_aabb = ctx.part_world_aabb(cabinet_body)
    if body_aabb is None:
        ctx.fail("cabinet body has measurable extents", "cabinet_body AABB was None")
    else:
        mins, maxs = body_aabb
        dx = maxs[0] - mins[0]
        dy = maxs[1] - mins[1]
        dz = maxs[2] - mins[2]
        ctx.check(
            "cabinet proportions read as sit-down arcade machine",
            1.55 <= dx <= 1.85 and 0.90 <= dy <= 1.02 and 1.60 <= dz <= 1.82,
            details=f"dims=({dx:.3f}, {dy:.3f}, {dz:.3f})",
        )

    wheel_pos = ctx.part_world_position(steering_wheel)
    ctx.check(
        "steering articulation is longitudinal continuous rotation",
        steering_joint.articulation_type == ArticulationType.CONTINUOUS
        and steering_joint.axis == (1.0, 0.0, 0.0)
        and steering_joint.motion_limits is not None
        and steering_joint.motion_limits.lower is None
        and steering_joint.motion_limits.upper is None,
        details=(
            f"type={steering_joint.articulation_type}, axis={steering_joint.axis}, "
            f"limits={steering_joint.motion_limits}"
        ),
    )
    ctx.check(
        "steering wheel sits in front of the seat at driving height",
        wheel_pos is not None and -0.22 <= wheel_pos[0] <= -0.02 and abs(wheel_pos[1]) <= 0.01 and 0.74 <= wheel_pos[2] <= 0.92,
        details=f"wheel_pos={wheel_pos}",
    )
    ctx.expect_contact(
        steering_wheel,
        cabinet_body,
        contact_tol=0.002,
        name="steering wheel is mounted to the cabinet",
    )

    with ctx.pose({steering_joint: 1.3}):
        ctx.fail_if_parts_overlap_in_current_pose(name="turned steering wheel clears the cabinet")

    return ctx.report()


# >>> USER_CODE_END

object_model = build_object_model()
