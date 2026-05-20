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
)


def _midpoint(
    a: tuple[float, float, float], b: tuple[float, float, float]
) -> tuple[float, float, float]:
    return ((a[0] + b[0]) * 0.5, (a[1] + b[1]) * 0.5, (a[2] + b[2]) * 0.5)


def _distance(a: tuple[float, float, float], b: tuple[float, float, float]) -> float:
    return math.sqrt((b[0] - a[0]) ** 2 + (b[1] - a[1]) ** 2 + (b[2] - a[2]) ** 2)


def _rpy_for_z_aligned_member(
    a: tuple[float, float, float], b: tuple[float, float, float]
) -> tuple[float, float, float]:
    dx = b[0] - a[0]
    dy = b[1] - a[1]
    dz = b[2] - a[2]
    length_xy = math.hypot(dx, dy)
    yaw = math.atan2(dy, dx)
    pitch = math.atan2(length_xy, dz)
    return (0.0, pitch, yaw)


def _add_round_beam(part, a, b, radius: float, material, *, name: str | None = None) -> None:
    part.visual(
        Cylinder(radius=radius, length=_distance(a, b)),
        origin=Origin(xyz=_midpoint(a, b), rpy=_rpy_for_z_aligned_member(a, b)),
        material=material,
        name=name,
    )


def _aabb_center(aabb) -> tuple[float, float, float]:
    return tuple((lo + hi) * 0.5 for lo, hi in zip(aabb[0], aabb[1]))


def build_object_model() -> ArticulatedObject:
    model = ArticulatedObject(name="overshot_waterwheel")

    stone = model.material("stone", rgba=(0.58, 0.58, 0.56, 1.0))
    timber = model.material("timber", rgba=(0.43, 0.29, 0.17, 1.0))
    weathered_timber = model.material("weathered_timber", rgba=(0.50, 0.36, 0.22, 1.0))
    dark_iron = model.material("dark_iron", rgba=(0.18, 0.18, 0.20, 1.0))

    rim_mesh = mesh_from_geometry(
        TorusGeometry(radius=0.40, tube=0.018, radial_segments=18, tubular_segments=72).rotate_x(
            math.pi / 2.0
        ),
        "waterwheel_rim_v2",
    )

    foundation = model.part("millwork_foundation")
    foundation.visual(
        Box((1.50, 0.12, 0.16)),
        origin=Origin(xyz=(-0.20, 0.28, 0.08)),
        material=stone,
        name="left_sill",
    )
    foundation.visual(
        Box((1.50, 0.12, 0.16)),
        origin=Origin(xyz=(-0.20, -0.28, 0.08)),
        material=stone,
        name="right_sill",
    )
    foundation.visual(
        Box((0.48, 0.56, 0.14)),
        origin=Origin(xyz=(0.08, 0.00, 0.07)),
        material=stone,
        name="wheel_cross_sill",
    )
    foundation.visual(
        Box((0.18, 0.60, 0.24)),
        origin=Origin(xyz=(-0.88, 0.00, 0.12)),
        material=stone,
        name="upstream_pier",
    )
    foundation.visual(
        Box((0.18, 0.60, 0.24)),
        origin=Origin(xyz=(-0.68, 0.00, 0.12)),
        material=stone,
        name="mid_pier",
    )

    for side_y in (-0.28, 0.28):
        for post_x in (-0.18, 0.18):
            foundation.visual(
                Box((0.08, 0.08, 0.96)),
                origin=Origin(xyz=(post_x, side_y, 0.64)),
                material=timber,
                name=f"support_post_{post_x:+.2f}_{side_y:+.2f}",
            )
        foundation.visual(
            Box((0.40, 0.08, 0.08)),
            origin=Origin(xyz=(0.00, side_y, 0.58)),
            material=timber,
            name=f"lower_transom_{side_y:+.2f}",
        )
        foundation.visual(
            Box((0.44, 0.08, 0.08)),
            origin=Origin(xyz=(0.00, side_y, 1.08)),
            material=timber,
            name=f"cap_beam_{side_y:+.2f}",
        )
        foundation.visual(
            Box((0.14, 0.05, 0.14)),
            origin=Origin(xyz=(0.00, side_y, 0.95)),
            material=dark_iron,
            name=f"bearing_block_{side_y:+.2f}",
        )
        _add_round_beam(
            foundation,
            (-0.18, side_y, 0.20),
            (0.12, side_y, 0.98),
            0.024,
            timber,
            name=f"brace_forward_{side_y:+.2f}",
        )
        _add_round_beam(
            foundation,
            (0.18, side_y, 0.20),
            (-0.12, side_y, 0.98),
            0.024,
            timber,
            name=f"brace_rear_{side_y:+.2f}",
        )

    for support_x, post_height, cap_z in ((-0.88, 1.20, 1.48), (-0.68, 1.15, 1.43)):
        for post_y in (-0.09, 0.09):
            foundation.visual(
                Box((0.08, 0.08, post_height)),
                origin=Origin(xyz=(support_x, post_y, 0.24 + post_height * 0.5)),
                material=timber,
                name=f"flume_post_{support_x:+.2f}_{post_y:+.2f}",
            )
        foundation.visual(
            Box((0.12, 0.24, 0.08)),
            origin=Origin(xyz=(support_x, 0.00, cap_z)),
            material=timber,
            name=f"flume_cap_{support_x:+.2f}",
        )

    chute_pitch = 0.14
    foundation.visual(
        Box((0.47, 0.24, 0.04)),
        origin=Origin(xyz=(-0.635, 0.00, 1.505), rpy=(0.0, chute_pitch, 0.0)),
        material=weathered_timber,
        name="chute_floor_rear",
    )
    foundation.visual(
        Box((0.08, 0.18, 0.04)),
        origin=Origin(xyz=(-0.46, 0.00, 1.484), rpy=(0.0, chute_pitch, 0.0)),
        material=weathered_timber,
        name="chute_floor_front",
    )
    foundation.visual(
        Box((0.47, 0.03, 0.14)),
        origin=Origin(xyz=(-0.635, 0.135, 1.565), rpy=(0.0, chute_pitch, 0.0)),
        material=weathered_timber,
        name="chute_left_wall",
    )
    foundation.visual(
        Box((0.47, 0.03, 0.14)),
        origin=Origin(xyz=(-0.635, -0.135, 1.565), rpy=(0.0, chute_pitch, 0.0)),
        material=weathered_timber,
        name="chute_right_wall",
    )
    foundation.visual(
        Box((0.03, 0.03, 0.40)),
        origin=Origin(xyz=(-0.39, 0.115, 1.62)),
        material=dark_iron,
        name="gate_guide_left",
    )
    foundation.visual(
        Box((0.03, 0.03, 0.40)),
        origin=Origin(xyz=(-0.39, -0.115, 1.62)),
        material=dark_iron,
        name="gate_guide_right",
    )
    foundation.visual(
        Box((0.10, 0.26, 0.04)),
        origin=Origin(xyz=(-0.39, 0.00, 1.84)),
        material=dark_iron,
        name="gate_lintel",
    )
    foundation.visual(
        Box((0.12, 0.03, 0.12)),
        origin=Origin(xyz=(-0.23, 0.105, 1.525), rpy=(0.0, chute_pitch, 0.0)),
        material=weathered_timber,
        name="chute_left_cheek",
    )
    foundation.visual(
        Box((0.12, 0.03, 0.12)),
        origin=Origin(xyz=(-0.23, -0.105, 1.525), rpy=(0.0, chute_pitch, 0.0)),
        material=weathered_timber,
        name="chute_right_cheek",
    )
    foundation.visual(
        Box((0.06, 0.21, 0.03)),
        origin=Origin(xyz=(-0.28, 0.00, 1.445), rpy=(0.0, chute_pitch, 0.0)),
        material=weathered_timber,
        name="chute_lip",
    )

    foundation.inertial = Inertial.from_geometry(
        Box((1.60, 0.66, 1.82)),
        mass=120.0,
        origin=Origin(xyz=(-0.20, 0.00, 0.91)),
    )

    waterwheel = model.part("waterwheel")
    waterwheel.visual(
        Cylinder(radius=0.035, length=0.51),
        origin=Origin(rpy=(math.pi / 2.0, 0.0, 0.0)),
        material=dark_iron,
        name="axle",
    )
    waterwheel.visual(
        Cylinder(radius=0.060, length=0.34),
        origin=Origin(rpy=(math.pi / 2.0, 0.0, 0.0)),
        material=dark_iron,
        name="hub_barrel",
    )
    waterwheel.visual(
        Cylinder(radius=0.12, length=0.04),
        origin=Origin(xyz=(0.0, -0.165, 0.0), rpy=(math.pi / 2.0, 0.0, 0.0)),
        material=timber,
        name="left_hub_disc",
    )
    waterwheel.visual(
        Cylinder(radius=0.12, length=0.04),
        origin=Origin(xyz=(0.0, 0.165, 0.0), rpy=(math.pi / 2.0, 0.0, 0.0)),
        material=timber,
        name="right_hub_disc",
    )
    waterwheel.visual(
        rim_mesh,
        origin=Origin(xyz=(0.0, -0.165, 0.0)),
        material=timber,
        name="left_rim",
    )
    waterwheel.visual(
        rim_mesh,
        origin=Origin(xyz=(0.0, 0.165, 0.0)),
        material=timber,
        name="right_rim",
    )

    spoke_count = 12
    bucket_offset = math.radians(15.0)
    for index in range(spoke_count):
        theta = bucket_offset + (2.0 * math.pi * index / spoke_count)
        radial_dir = (math.cos(theta), 0.0, math.sin(theta))
        tangent_dir = (-math.sin(theta), 0.0, math.cos(theta))

        for side_y, side_name in ((-0.165, "left"), (0.165, "right")):
            spoke_start = (0.11 * radial_dir[0], side_y, 0.11 * radial_dir[2])
            spoke_end = (0.41 * radial_dir[0], side_y, 0.41 * radial_dir[2])
            _add_round_beam(
                waterwheel,
                spoke_start,
                spoke_end,
                0.015,
                timber,
                name=f"{side_name}_spoke_{index:02d}",
            )

        bucket_center = (0.42 * radial_dir[0], 0.0, 0.42 * radial_dir[2])
        waterwheel.visual(
            Box((0.09, 0.31, 0.14)),
            origin=Origin(xyz=bucket_center, rpy=(0.0, -theta, 0.0)),
            material=weathered_timber,
            name=f"bucket_{index:02d}",
        )

        shelf_center = (
            0.34 * radial_dir[0] - 0.028 * tangent_dir[0],
            0.0,
            0.34 * radial_dir[2] - 0.028 * tangent_dir[2],
        )
        waterwheel.visual(
            Box((0.032, 0.31, 0.10)),
            origin=Origin(xyz=shelf_center, rpy=(0.0, -theta, 0.0)),
            material=weathered_timber,
            name=f"bucket_shelf_{index:02d}",
        )

    waterwheel.inertial = Inertial.from_geometry(
        Box((0.92, 0.51, 0.92)),
        mass=28.0,
        origin=Origin(),
    )

    sluice_gate = model.part("sluice_gate")
    sluice_gate.visual(
        Box((0.018, 0.20, 0.22)),
        origin=Origin(xyz=(0.0, 0.0, 0.11)),
        material=dark_iron,
        name="gate_panel",
    )
    sluice_gate.visual(
        Box((0.024, 0.12, 0.04)),
        origin=Origin(xyz=(0.0, 0.0, 0.24)),
        material=dark_iron,
        name="gate_cap",
    )
    sluice_gate.inertial = Inertial.from_geometry(
        Box((0.03, 0.20, 0.26)),
        mass=3.0,
        origin=Origin(xyz=(0.0, 0.0, 0.13)),
    )

    model.articulation(
        "waterwheel_spin",
        ArticulationType.CONTINUOUS,
        parent=foundation,
        child=waterwheel,
        origin=Origin(xyz=(0.0, 0.0, 0.95)),
        axis=(0.0, 1.0, 0.0),
        motion_limits=MotionLimits(effort=80.0, velocity=2.0),
    )
    model.articulation(
        "sluice_gate_slide",
        ArticulationType.PRISMATIC,
        parent=foundation,
        child=sluice_gate,
        origin=Origin(xyz=(-0.39, 0.0, 1.46)),
        axis=(0.0, 0.0, 1.0),
        motion_limits=MotionLimits(effort=12.0, velocity=0.12, lower=0.0, upper=0.10),
    )

    return model


def run_tests() -> TestReport:
    ctx = TestContext(object_model)
    foundation = object_model.get_part("millwork_foundation")
    waterwheel = object_model.get_part("waterwheel")
    sluice_gate = object_model.get_part("sluice_gate")
    wheel_spin = object_model.get_articulation("waterwheel_spin")
    gate_slide = object_model.get_articulation("sluice_gate_slide")

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
        "waterwheel_continuous_axis",
        tuple(round(value, 3) for value in wheel_spin.axis) == (0.0, 1.0, 0.0),
        details=f"axis={wheel_spin.axis}",
    )
    ctx.check(
        "sluice_gate_prismatic_axis",
        tuple(round(value, 3) for value in gate_slide.axis) == (0.0, 0.0, 1.0),
        details=f"axis={gate_slide.axis}",
    )

    ctx.expect_contact(waterwheel, foundation, name="waterwheel_supported_on_axle_bearings")
    ctx.expect_contact(sluice_gate, foundation, name="sluice_gate_captured_by_guides")

    bucket_rest_aabb = ctx.part_element_world_aabb(waterwheel, elem="bucket_00")
    gate_rest_pos = ctx.part_world_position(sluice_gate)
    assert bucket_rest_aabb is not None
    assert gate_rest_pos is not None
    bucket_rest_center = _aabb_center(bucket_rest_aabb)

    with ctx.pose({wheel_spin: math.pi / 2.0}):
        ctx.expect_contact(waterwheel, foundation, name="waterwheel_stays_supported_at_quarter_turn")
        ctx.fail_if_parts_overlap_in_current_pose(name="waterwheel_clear_at_quarter_turn")
        bucket_turn_aabb = ctx.part_element_world_aabb(waterwheel, elem="bucket_00")
        assert bucket_turn_aabb is not None
        bucket_turn_center = _aabb_center(bucket_turn_aabb)
        ctx.check(
            "waterwheel_bucket_moves_in_vertical_plane",
            abs(bucket_turn_center[1] - bucket_rest_center[1]) < 0.01
            and bucket_turn_center[2] < bucket_rest_center[2] - 0.40,
            details=f"rest={bucket_rest_center}, quarter={bucket_turn_center}",
        )

    with ctx.pose({gate_slide: 0.08}):
        ctx.expect_contact(sluice_gate, foundation, name="sluice_gate_remains_in_guides_when_open")
        gate_open_pos = ctx.part_world_position(sluice_gate)
        assert gate_open_pos is not None
        ctx.check(
            "sluice_gate_lifts_upward",
            gate_open_pos[2] > gate_rest_pos[2] + 0.075,
            details=f"rest={gate_rest_pos}, open={gate_open_pos}",
        )

    return ctx.report()


# >>> USER_CODE_END

object_model = build_object_model()
