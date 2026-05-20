from __future__ import annotations

# User code should import every SDK/stdlib symbol it uses instead of relying on
# hidden scaffold imports.
# >>> USER_CODE_START
import math

from sdk import (
    ArticulatedObject,
    ArticulationType,
    AssetContext,
    Box,
    Cylinder,
    Inertial,
    LatheGeometry,
    MotionLimits,
    Origin,
    TestContext,
    TestReport,
    mesh_from_geometry,
)

ASSETS = AssetContext.from_script(__file__)


def build_object_model() -> ArticulatedObject:
    def _save_mesh(name: str, geometry):
        return mesh_from_geometry(geometry, ASSETS.mesh_path(name))

    def _distance(
        a: tuple[float, float, float], b: tuple[float, float, float]
    ) -> float:
        return math.sqrt((b[0] - a[0]) ** 2 + (b[1] - a[1]) ** 2 + (b[2] - a[2]) ** 2)

    def _midpoint(
        a: tuple[float, float, float], b: tuple[float, float, float]
    ) -> tuple[float, float, float]:
        return ((a[0] + b[0]) * 0.5, (a[1] + b[1]) * 0.5, (a[2] + b[2]) * 0.5)

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

    def _shell_ring(name: str, *, outer_radius: float, inner_radius: float, width: float):
        half = width * 0.5
        return _save_mesh(
            name,
            LatheGeometry.from_shell_profiles(
                [(outer_radius, -half), (outer_radius, half)],
                [(inner_radius, -half), (inner_radius, half)],
                segments=72,
                start_cap="flat",
                end_cap="flat",
            ),
        )

    model = ArticulatedObject(name="undershot_waterwheel_utility", assets=ASSETS)

    frame_paint = model.material("frame_paint", rgba=(0.24, 0.31, 0.23, 1.0))
    wheel_paint = model.material("wheel_paint", rgba=(0.33, 0.36, 0.38, 1.0))
    dark_steel = model.material("dark_steel", rgba=(0.18, 0.19, 0.20, 1.0))
    fastener_steel = model.material("fastener_steel", rgba=(0.73, 0.75, 0.76, 1.0))
    bearing_iron = model.material("bearing_iron", rgba=(0.42, 0.45, 0.47, 1.0))
    paddle_composite = model.material("paddle_composite", rgba=(0.20, 0.17, 0.14, 1.0))

    wheel_radius = 0.95
    wheel_axle_height = 1.03
    wheel_width = 0.58
    rim_inner_radius = 0.82
    rim_width = 0.07
    side_ring_y = 0.29
    hub_bore_radius = 0.16
    rim_ring_mesh = _shell_ring(
        "waterwheel_rim_ring.obj",
        outer_radius=wheel_radius,
        inner_radius=rim_inner_radius,
        width=rim_width,
    )
    hub_sleeve_mesh = _shell_ring(
        "waterwheel_hub_sleeve.obj",
        outer_radius=0.24,
        inner_radius=hub_bore_radius,
        width=0.54,
    )
    hub_flange_mesh = _shell_ring(
        "waterwheel_hub_flange.obj",
        outer_radius=0.34,
        inner_radius=hub_bore_radius,
        width=0.06,
    )
    bearing_collar_mesh = _shell_ring(
        "waterwheel_bearing_collar.obj",
        outer_radius=0.28,
        inner_radius=hub_bore_radius,
        width=0.05,
    )
    bearing_housing_shell_mesh = _shell_ring(
        "waterwheel_bearing_housing_shell.obj",
        outer_radius=0.23,
        inner_radius=0.135,
        width=0.18,
    )

    frame = model.part("frame")
    frame.inertial = Inertial.from_geometry(
        Box((3.25, 1.60, 1.60)),
        mass=980.0,
        origin=Origin(xyz=(0.0, 0.0, 0.80)),
    )

    frame.visual(
        Box((3.20, 0.20, 0.18)),
        origin=Origin(xyz=(0.0, 0.66, 0.09)),
        material=frame_paint,
        name="left_base_skid",
    )
    frame.visual(
        Box((3.20, 0.20, 0.18)),
        origin=Origin(xyz=(0.0, -0.66, 0.09)),
        material=frame_paint,
        name="right_base_skid",
    )
    for x_pos, visual_name in [(-1.15, "cross_tie_front"), (1.15, "cross_tie_rear")]:
        frame.visual(
            Box((0.24, 1.22, 0.14)),
            origin=Origin(xyz=(x_pos, 0.0, 0.18)),
            material=frame_paint,
            name=visual_name,
        )

    for side_sign, side_name in [(1.0, "left"), (-1.0, "right")]:
        y_pos = 0.67 * side_sign
        frame.visual(
            Box((0.34, 0.14, 0.24)),
            origin=Origin(xyz=(0.0, y_pos, 0.78)),
            material=bearing_iron,
            name=f"{side_name}_pedestal_base",
        )
        frame.visual(
            Box((0.30, 0.14, 0.08)),
            origin=Origin(xyz=(0.0, y_pos, 1.23)),
            material=bearing_iron,
            name=f"{side_name}_bearing_cap",
        )
        frame.visual(
            Box((0.38, 0.10, 0.06)),
            origin=Origin(xyz=(0.0, 0.61 * side_sign, 1.03)),
            material=bearing_iron,
            name=f"{side_name}_bearing_saddle",
        )
        for bolt_x in (-0.09, 0.09):
            for bolt_y in (-0.035, 0.035):
                frame.visual(
                    Cylinder(radius=0.016, length=0.018),
                    origin=Origin(xyz=(bolt_x, y_pos + bolt_y, 1.279)),
                    material=fastener_steel,
                )

        _add_member(
            frame,
            (-0.68, 0.66 * side_sign, 0.18),
            (-0.20, 0.63 * side_sign, 1.02),
            radius=0.055,
            material=frame_paint,
        )
        _add_member(
            frame,
            (0.68, 0.66 * side_sign, 0.18),
            (0.20, 0.63 * side_sign, 1.02),
            radius=0.055,
            material=frame_paint,
        )
        _add_member(
            frame,
            (-0.68, 0.66 * side_sign, 0.18),
            (0.68, 0.66 * side_sign, 0.18),
            radius=0.040,
            material=frame_paint,
        )
        _add_member(
            frame,
            (-0.26, 0.63 * side_sign, 1.02),
            (0.26, 0.63 * side_sign, 1.02),
            radius=0.050,
            material=frame_paint,
        )
        _add_member(
            frame,
            (-0.48, 0.66 * side_sign, 0.30),
            (0.00, 0.63 * side_sign, 0.86),
            radius=0.038,
            material=frame_paint,
        )
        _add_member(
            frame,
            (0.48, 0.66 * side_sign, 0.30),
            (0.00, 0.63 * side_sign, 0.86),
            radius=0.038,
            material=frame_paint,
        )
        frame.visual(
            Box((0.18, 0.18, 0.14)),
            origin=Origin(xyz=(-0.68, 0.66 * side_sign, 0.22)),
            material=frame_paint,
        )
        frame.visual(
            Box((0.18, 0.18, 0.14)),
            origin=Origin(xyz=(0.68, 0.66 * side_sign, 0.22)),
            material=frame_paint,
        )

    frame.visual(
        bearing_housing_shell_mesh,
        origin=Origin(xyz=(0.0, 0.57, wheel_axle_height), rpy=(math.pi / 2.0, 0.0, 0.0)),
        material=bearing_iron,
        name="left_bearing_housing",
    )
    frame.visual(
        bearing_housing_shell_mesh,
        origin=Origin(xyz=(0.0, -0.57, wheel_axle_height), rpy=(math.pi / 2.0, 0.0, 0.0)),
        material=bearing_iron,
        name="right_bearing_housing",
    )
    frame.visual(
        Cylinder(radius=0.12, length=0.08),
        origin=Origin(xyz=(0.0, 0.58, wheel_axle_height), rpy=(math.pi / 2.0, 0.0, 0.0)),
        material=dark_steel,
        name="left_journal_stub",
    )
    frame.visual(
        Cylinder(radius=0.12, length=0.08),
        origin=Origin(xyz=(0.0, -0.58, wheel_axle_height), rpy=(math.pi / 2.0, 0.0, 0.0)),
        material=dark_steel,
        name="right_journal_stub",
    )

    for x_pos in (-1.43, -0.95, 0.0, 0.95, 1.43):
        for y_pos in (-0.73, -0.59, 0.59, 0.73):
            frame.visual(
                Cylinder(radius=0.016, length=0.018),
                origin=Origin(xyz=(x_pos, y_pos, 0.189)),
                material=fastener_steel,
            )

    wheel = model.part("wheel")
    wheel.inertial = Inertial.from_geometry(
        Cylinder(radius=wheel_radius, length=0.98),
        mass=620.0,
        origin=Origin(rpy=(math.pi / 2.0, 0.0, 0.0)),
    )

    wheel.visual(
        rim_ring_mesh,
        origin=Origin(xyz=(0.0, side_ring_y, 0.0), rpy=(math.pi / 2.0, 0.0, 0.0)),
        material=wheel_paint,
        name="rim_left",
    )
    wheel.visual(
        rim_ring_mesh,
        origin=Origin(xyz=(0.0, -side_ring_y, 0.0), rpy=(math.pi / 2.0, 0.0, 0.0)),
        material=wheel_paint,
        name="rim_right",
    )
    wheel.visual(
        Cylinder(radius=0.17, length=0.92),
        origin=Origin(rpy=(math.pi / 2.0, 0.0, 0.0)),
        material=dark_steel,
        name="hub_sleeve",
    )
    wheel.visual(
        Cylinder(radius=0.30, length=0.06),
        origin=Origin(xyz=(0.0, 0.31, 0.0), rpy=(math.pi / 2.0, 0.0, 0.0)),
        material=wheel_paint,
        name="left_hub_flange",
    )
    wheel.visual(
        Cylinder(radius=0.30, length=0.06),
        origin=Origin(xyz=(0.0, -0.31, 0.0), rpy=(math.pi / 2.0, 0.0, 0.0)),
        material=wheel_paint,
        name="right_hub_flange",
    )
    wheel.visual(
        Cylinder(radius=0.12, length=0.08),
        origin=Origin(xyz=(0.0, 0.50, 0.0), rpy=(math.pi / 2.0, 0.0, 0.0)),
        material=bearing_iron,
        name="left_bearing_collar",
    )
    wheel.visual(
        Cylinder(radius=0.12, length=0.08),
        origin=Origin(xyz=(0.0, -0.50, 0.0), rpy=(math.pi / 2.0, 0.0, 0.0)),
        material=bearing_iron,
        name="right_bearing_collar",
    )

    for spoke_index in range(10):
        angle = (math.tau * spoke_index) / 10.0
        inner_radius = 0.30
        outer_radius = 0.79
        for y_pos in (-0.25, 0.25):
            _add_member(
                wheel,
                (
                    inner_radius * math.sin(angle),
                    y_pos,
                    inner_radius * math.cos(angle),
                ),
                (
                    outer_radius * math.sin(angle),
                    y_pos,
                    outer_radius * math.cos(angle),
                ),
                radius=0.035,
                material=wheel_paint,
            )
        if spoke_index % 2 == 0:
            _add_member(
                wheel,
                (
                    inner_radius * math.sin(angle),
                    -0.22,
                    inner_radius * math.cos(angle),
                ),
                (
                    outer_radius * math.sin(angle + 0.20),
                    0.22,
                    outer_radius * math.cos(angle + 0.20),
                ),
                radius=0.020,
                material=dark_steel,
            )
            _add_member(
                wheel,
                (
                    inner_radius * math.sin(angle),
                    0.22,
                    inner_radius * math.cos(angle),
                ),
                (
                    outer_radius * math.sin(angle + 0.20),
                    -0.22,
                    outer_radius * math.cos(angle + 0.20),
                ),
                radius=0.020,
                material=dark_steel,
            )

    paddle_count = 12
    paddle_radius = 0.87
    for paddle_index in range(paddle_count):
        angle = math.pi + (math.tau * paddle_index) / paddle_count
        center = (
            paddle_radius * math.sin(angle),
            0.0,
            paddle_radius * math.cos(angle),
        )
        wheel.visual(
            Box((0.18, wheel_width, 0.14)),
            origin=Origin(xyz=center, rpy=(0.0, angle, 0.0)),
            material=paddle_composite,
            name=f"paddle_{paddle_index:02d}",
        )
        wheel.visual(
            Box((0.04, wheel_width, 0.18)),
            origin=Origin(
                xyz=(
                    center[0] + 0.065 * math.cos(angle),
                    0.0,
                    center[2] - 0.065 * math.sin(angle),
                ),
                rpy=(0.0, angle, 0.0),
            ),
            material=wheel_paint,
        )
        wheel.visual(
            Box((0.04, wheel_width, 0.18)),
            origin=Origin(
                xyz=(
                    center[0] - 0.065 * math.cos(angle),
                    0.0,
                    center[2] + 0.065 * math.sin(angle),
                ),
                rpy=(0.0, angle, 0.0),
            ),
            material=wheel_paint,
        )
    for flange_y in (-0.31, 0.31):
        for bolt_index in range(8):
            angle = (math.tau * bolt_index) / 8.0
            wheel.visual(
                Cylinder(radius=0.014, length=0.05),
                origin=Origin(
                    xyz=(
                        0.22 * math.sin(angle),
                        flange_y,
                        0.22 * math.cos(angle),
                    ),
                    rpy=(math.pi / 2.0, 0.0, 0.0),
                ),
                material=fastener_steel,
            )

    model.articulation(
        "wheel_spin",
        ArticulationType.CONTINUOUS,
        parent=frame,
        child=wheel,
        origin=Origin(xyz=(0.0, 0.0, wheel_axle_height)),
        axis=(0.0, 1.0, 0.0),
        motion_limits=MotionLimits(effort=1500.0, velocity=1.6),
    )

    return model


def run_tests() -> TestReport:
    ctx = TestContext(object_model, asset_root=ASSETS.asset_root)
    frame = object_model.get_part("frame")
    wheel = object_model.get_part("wheel")
    wheel_spin = object_model.get_articulation("wheel_spin")
    left_stub = frame.get_visual("left_journal_stub")
    right_stub = frame.get_visual("right_journal_stub")
    left_collar = wheel.get_visual("left_bearing_collar")
    right_collar = wheel.get_visual("right_bearing_collar")
    hub_sleeve = wheel.get_visual("hub_sleeve")
    low_paddle = wheel.get_visual("paddle_00")

    ctx.check_model_valid()
    ctx.check_mesh_files_exist()

    # Preferred default QC stack:
    # 1) likely-failure broad-part floating check for isolated parts
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
        "wheel_spin_axis_is_horizontal",
        tuple(wheel_spin.axis) == (0.0, 1.0, 0.0),
        f"expected wheel spin axis (0, 1, 0), got {wheel_spin.axis!r}",
    )
    limits = wheel_spin.motion_limits
    ctx.check(
        "wheel_spin_uses_continuous_limits",
        limits is not None and limits.lower is None and limits.upper is None,
        "undershot wheel should use a continuous rotation joint without finite bounds",
    )
    ctx.expect_origin_gap(
        wheel,
        frame,
        axis="z",
        min_gap=0.98,
        max_gap=1.08,
        name="wheel_axle_height_is_low_flow_ready",
    )
    ctx.expect_origin_distance(
        wheel,
        frame,
        axes="xy",
        max_dist=0.02,
        name="wheel_centered_between_side_frames",
    )
    ctx.fail_if_articulation_overlaps(
        max_pose_samples=16,
        name="wheel_rotation_clearance_sweep",
    )
    ctx.fail_if_isolated_parts(
        max_pose_samples=12,
        contact_tol=5e-4,
        name="wheel_supported_through_rotation",
    )

    for angle, label in [
        (0.0, "rest"),
        (math.pi / 4.0, "quarter_turn"),
        (math.pi / 2.0, "half_turn"),
    ]:
        with ctx.pose({wheel_spin: angle}):
            left_collar_aabb = ctx.part_element_world_aabb(wheel, elem=left_collar)
            right_collar_aabb = ctx.part_element_world_aabb(wheel, elem=right_collar)
            hub_aabb = ctx.part_element_world_aabb(wheel, elem=hub_sleeve)
            left_stub_aabb = ctx.part_element_world_aabb(frame, elem=left_stub)
            right_stub_aabb = ctx.part_element_world_aabb(frame, elem=right_stub)

            ctx.expect_contact(
                frame,
                wheel,
                elem_a=left_stub,
                elem_b=left_collar,
                contact_tol=5e-4,
                name=f"left_bearing_support_contact_at_{label}",
            )
            ctx.expect_contact(
                frame,
                wheel,
                elem_a=right_stub,
                elem_b=right_collar,
                contact_tol=5e-4,
                name=f"right_bearing_support_contact_at_{label}",
            )
            left_collar_gap = None
            right_collar_gap = None
            left_stub_gap = None
            right_stub_gap = None
            if left_collar_aabb and hub_aabb:
                left_collar_gap = left_collar_aabb[0][1] - hub_aabb[1][1]
            if right_collar_aabb and hub_aabb:
                right_collar_gap = hub_aabb[0][1] - right_collar_aabb[1][1]
            if left_stub_aabb and hub_aabb:
                left_stub_gap = left_stub_aabb[0][1] - hub_aabb[1][1]
            if right_stub_aabb and hub_aabb:
                right_stub_gap = hub_aabb[0][1] - right_stub_aabb[1][1]

            ctx.check(
                f"left_collar_sits_outboard_of_hub_at_{label}",
                left_collar_gap is not None and -1e-6 <= left_collar_gap <= 0.05,
                f"expected left collar to start just outboard of hub; gap={left_collar_gap!r}",
            )
            ctx.check(
                f"right_collar_sits_outboard_of_hub_at_{label}",
                right_collar_gap is not None and -1e-6 <= right_collar_gap <= 0.05,
                f"expected right collar to start just outboard of hub; gap={right_collar_gap!r}",
            )
            ctx.check(
                f"left_stub_clear_of_hub_body_at_{label}",
                left_stub_gap is not None and 0.07 <= left_stub_gap <= 0.13,
                f"expected left journal stub to sit outside hub barrel; gap={left_stub_gap!r}",
            )
            ctx.check(
                f"right_stub_clear_of_hub_body_at_{label}",
                right_stub_gap is not None and 0.07 <= right_stub_gap <= 0.13,
                f"expected right journal stub to sit outside hub barrel; gap={right_stub_gap!r}",
            )
            ctx.fail_if_parts_overlap_in_current_pose(name=f"no_part_overlap_at_{label}")
            ctx.fail_if_isolated_parts(contact_tol=5e-4, name=f"no_floating_at_{label}")

    with ctx.pose({wheel_spin: 0.0}):
        paddle_aabb = ctx.part_element_world_aabb(wheel, elem=low_paddle)
        ctx.check(
            "undershot_paddle_reaches_low_water_zone",
            paddle_aabb is not None and 0.0 <= paddle_aabb[0][2] <= 0.12,
            f"expected lowest paddle to approach the waterline; got aabb={paddle_aabb!r}",
        )

    return ctx.report()


# >>> USER_CODE_END

object_model = build_object_model()
