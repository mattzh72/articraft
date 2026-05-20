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
)


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


def _aabb_center(aabb) -> tuple[float, float, float] | None:
    if aabb is None:
        return None
    lo, hi = aabb
    return (
        (lo[0] + hi[0]) * 0.5,
        (lo[1] + hi[1]) * 0.5,
        (lo[2] + hi[2]) * 0.5,
    )


def build_object_model() -> ArticulatedObject:
    model = ArticulatedObject(name="weatherproof_searchlight_tower")

    concrete = model.material("concrete", rgba=(0.68, 0.68, 0.66, 1.0))
    galvanized = model.material("galvanized", rgba=(0.57, 0.60, 0.62, 1.0))
    coated_dark = model.material("coated_dark", rgba=(0.20, 0.23, 0.26, 1.0))
    coated_light = model.material("coated_light", rgba=(0.82, 0.84, 0.82, 1.0))
    stainless = model.material("stainless", rgba=(0.76, 0.78, 0.80, 1.0))
    lens_glass = model.material("lens_glass", rgba=(0.74, 0.86, 0.94, 0.45))
    gasket_black = model.material("gasket_black", rgba=(0.08, 0.09, 0.10, 1.0))

    tower_support = model.part("tower_support")
    tower_support.visual(
        Box((1.50, 1.50, 0.30)),
        origin=Origin(xyz=(0.0, 0.0, 0.15)),
        material=concrete,
        name="foundation_block",
    )
    tower_support.visual(
        Box((0.88, 0.88, 0.14)),
        origin=Origin(xyz=(0.0, 0.0, 0.37)),
        material=coated_dark,
        name="base_plinth",
    )
    tower_support.visual(
        Cylinder(radius=0.28, length=0.04),
        origin=Origin(xyz=(0.0, 0.0, 0.46)),
        material=galvanized,
        name="mast_flange",
    )
    tower_support.visual(
        Cylinder(radius=0.19, length=0.50),
        origin=Origin(xyz=(0.0, 0.0, 0.73)),
        material=galvanized,
        name="mast_reinforcement_sleeve",
    )
    tower_support.visual(
        Cylinder(radius=0.16, length=3.40),
        origin=Origin(xyz=(0.0, 0.0, 2.18)),
        material=galvanized,
        name="mast_tube",
    )
    tower_support.visual(
        Cylinder(radius=0.12, length=0.10),
        origin=Origin(xyz=(0.0, 0.0, 3.83)),
        material=coated_dark,
        name="mast_top_neck",
    )
    tower_support.visual(
        Cylinder(radius=0.24, length=0.05),
        origin=Origin(xyz=(0.0, 0.0, 3.905)),
        material=coated_dark,
        name="top_turntable",
    )

    brace_base = 0.24
    brace_top = 0.14
    for sx in (-1.0, 1.0):
        for sy in (-1.0, 1.0):
            _add_member(
                tower_support,
                (sx * brace_base, sy * brace_base, 0.44),
                (sx * brace_top, sy * brace_top, 1.04),
                radius=0.034,
                material=galvanized,
            )

    for angle_deg in range(0, 360, 45):
        angle = math.radians(angle_deg)
        radius = 0.19
        tower_support.visual(
            Cylinder(radius=0.016, length=0.10),
            origin=Origin(
                xyz=(radius * math.cos(angle), radius * math.sin(angle), 0.49)
            ),
            material=stainless,
        )
        tower_support.visual(
            Cylinder(radius=0.028, length=0.014),
            origin=Origin(
                xyz=(radius * math.cos(angle), radius * math.sin(angle), 0.542)
            ),
            material=stainless,
        )

    tower_support.visual(
        Box((0.06, 0.12, 0.18)),
        origin=Origin(xyz=(0.17, 0.0, 1.55)),
        material=coated_dark,
        name="service_box_bracket",
    )
    tower_support.visual(
        Box((0.20, 0.14, 0.16)),
        origin=Origin(xyz=(0.28, 0.0, 1.55)),
        material=coated_light,
        name="service_box",
    )
    tower_support.visual(
        Box((0.24, 0.18, 0.03)),
        origin=Origin(xyz=(0.30, 0.0, 1.645)),
        material=coated_light,
        name="service_box_hood",
    )
    tower_support.visual(
        Cylinder(radius=0.024, length=0.54),
        origin=Origin(xyz=(0.15, 0.0, 1.20)),
        material=gasket_black,
        name="sealed_conduit",
    )

    tower_support.inertial = Inertial.from_geometry(
        Box((1.50, 1.50, 3.93)),
        mass=2800.0,
        origin=Origin(xyz=(0.0, 0.0, 1.965)),
    )

    pan_yoke = model.part("pan_yoke")
    pan_yoke.visual(
        Cylinder(radius=0.20, length=0.04),
        origin=Origin(xyz=(0.0, 0.0, 0.02)),
        material=coated_dark,
        name="turntable_ring",
    )
    pan_yoke.visual(
        Cylinder(radius=0.25, length=0.08),
        origin=Origin(xyz=(0.0, 0.0, 0.08)),
        material=coated_dark,
        name="slew_shroud",
    )
    pan_yoke.visual(
        Box((0.56, 0.46, 0.18)),
        origin=Origin(xyz=(0.02, 0.0, 0.18)),
        material=coated_dark,
        name="gearbox_housing",
    )
    pan_yoke.visual(
        Box((0.58, 0.22, 0.04)),
        origin=Origin(xyz=(0.12, 0.0, 0.29)),
        material=coated_light,
        name="front_drip_hood",
    )
    pan_yoke.visual(
        Box((0.22, 0.26, 0.34)),
        origin=Origin(xyz=(-0.10, 0.0, 0.45)),
        material=coated_dark,
        name="center_pedestal",
    )
    pan_yoke.visual(
        Box((0.26, 0.64, 0.14)),
        origin=Origin(xyz=(-0.04, 0.0, 0.45)),
        material=coated_dark,
        name="lower_yoke_tie",
    )
    pan_yoke.visual(
        Box((0.24, 0.70, 0.08)),
        origin=Origin(xyz=(0.18, 0.0, 0.96)),
        material=coated_dark,
        name="upper_yoke_tie",
    )
    pan_yoke.visual(
        Box((0.22, 0.08, 0.90)),
        origin=Origin(xyz=(0.19, 0.34, 0.67)),
        material=coated_light,
        name="left_yoke_arm",
    )
    pan_yoke.visual(
        Box((0.22, 0.08, 0.90)),
        origin=Origin(xyz=(0.19, -0.34, 0.67)),
        material=coated_light,
        name="right_yoke_arm",
    )
    pan_yoke.visual(
        Box((0.16, 0.12, 0.18)),
        origin=Origin(xyz=(0.28, 0.28, 0.62)),
        material=coated_light,
        name="left_bearing_block",
    )
    pan_yoke.visual(
        Box((0.16, 0.12, 0.18)),
        origin=Origin(xyz=(0.28, -0.28, 0.62)),
        material=coated_light,
        name="right_bearing_block",
    )
    pan_yoke.visual(
        Box((0.20, 0.18, 0.02)),
        origin=Origin(xyz=(0.28, 0.28, 0.72)),
        material=coated_light,
        name="left_bearing_rain_cap",
    )
    pan_yoke.visual(
        Box((0.20, 0.18, 0.02)),
        origin=Origin(xyz=(0.28, -0.28, 0.72)),
        material=coated_light,
        name="right_bearing_rain_cap",
    )
    pan_yoke.visual(
        Cylinder(radius=0.018, length=0.14),
        origin=Origin(xyz=(0.19, 0.28, 0.62), rpy=(0.0, math.pi / 2.0, 0.0)),
        material=stainless,
        name="left_hardware_cover",
    )
    pan_yoke.visual(
        Cylinder(radius=0.018, length=0.14),
        origin=Origin(xyz=(0.19, -0.28, 0.62), rpy=(0.0, math.pi / 2.0, 0.0)),
        material=stainless,
        name="right_hardware_cover",
    )
    _add_member(
        pan_yoke,
        (-0.02, 0.28, 0.52),
        (0.14, 0.28, 0.82),
        radius=0.024,
        material=coated_dark,
        name="left_diagonal_brace",
    )
    _add_member(
        pan_yoke,
        (-0.02, -0.28, 0.52),
        (0.14, -0.28, 0.82),
        radius=0.024,
        material=coated_dark,
        name="right_diagonal_brace",
    )
    pan_yoke.inertial = Inertial.from_geometry(
        Box((0.60, 0.78, 1.05)),
        mass=220.0,
        origin=Origin(xyz=(0.0, 0.0, 0.525)),
    )

    spotlight_head = model.part("spotlight_head")
    spotlight_head.visual(
        Box((0.12, 0.38, 0.16)),
        origin=Origin(xyz=(0.08, 0.0, 0.0)),
        material=coated_dark,
        name="trunnion_saddle",
    )
    spotlight_head.visual(
        Cylinder(radius=0.07, length=0.06),
        origin=Origin(xyz=(0.0, 0.19, 0.0), rpy=(-math.pi / 2.0, 0.0, 0.0)),
        material=coated_dark,
        name="left_trunnion",
    )
    spotlight_head.visual(
        Cylinder(radius=0.07, length=0.06),
        origin=Origin(xyz=(0.0, -0.19, 0.0), rpy=(-math.pi / 2.0, 0.0, 0.0)),
        material=coated_dark,
        name="right_trunnion",
    )
    spotlight_head.visual(
        Cylinder(radius=0.22, length=0.68),
        origin=Origin(xyz=(0.42, 0.0, 0.0), rpy=(0.0, math.pi / 2.0, 0.0)),
        material=coated_light,
        name="barrel_shell",
    )
    spotlight_head.visual(
        Cylinder(radius=0.17, length=0.12),
        origin=Origin(xyz=(0.08, 0.0, 0.0), rpy=(0.0, math.pi / 2.0, 0.0)),
        material=coated_light,
        name="rear_cap",
    )
    spotlight_head.visual(
        Box((0.24, 0.26, 0.14)),
        origin=Origin(xyz=(0.16, 0.0, 0.20)),
        material=coated_light,
        name="driver_box",
    )
    spotlight_head.visual(
        Box((0.30, 0.32, 0.03)),
        origin=Origin(xyz=(0.18, 0.0, 0.285)),
        material=coated_light,
        name="driver_box_hood",
    )
    spotlight_head.visual(
        Box((0.14, 0.18, 0.05)),
        origin=Origin(xyz=(0.30, 0.0, -0.14)),
        material=coated_dark,
        name="underslung_service_box",
    )
    spotlight_head.visual(
        Cylinder(radius=0.205, length=0.012),
        origin=Origin(xyz=(0.754, 0.0, 0.0), rpy=(0.0, math.pi / 2.0, 0.0)),
        material=lens_glass,
        name="lens_glass",
    )
    spotlight_head.visual(
        Cylinder(radius=0.24, length=0.056),
        origin=Origin(xyz=(0.784, 0.0, 0.0), rpy=(0.0, math.pi / 2.0, 0.0)),
        material=coated_dark,
        name="bezel_ring",
    )
    spotlight_head.visual(
        Cylinder(radius=0.26, length=0.18),
        origin=Origin(xyz=(0.90, 0.0, 0.0), rpy=(0.0, math.pi / 2.0, 0.0)),
        material=coated_light,
        name="hood_shell",
    )
    spotlight_head.visual(
        Box((0.18, 0.56, 0.04)),
        origin=Origin(xyz=(0.90, 0.0, 0.20)),
        material=coated_light,
        name="hood_brow",
    )
    spotlight_head.visual(
        Cylinder(radius=0.024, length=0.08),
        origin=Origin(xyz=(0.02, 0.0, 0.29)),
        material=gasket_black,
        name="rear_cable_gland",
    )
    spotlight_head.inertial = Inertial.from_geometry(
        Box((1.10, 0.66, 0.62)),
        mass=165.0,
        origin=Origin(xyz=(0.28, 0.0, 0.0)),
    )

    model.articulation(
        "pan_rotation",
        ArticulationType.CONTINUOUS,
        parent=tower_support,
        child=pan_yoke,
        origin=Origin(xyz=(0.0, 0.0, 3.93)),
        axis=(0.0, 0.0, 1.0),
        motion_limits=MotionLimits(effort=180.0, velocity=0.6),
    )
    model.articulation(
        "tilt_motion",
        ArticulationType.REVOLUTE,
        parent=pan_yoke,
        child=spotlight_head,
        origin=Origin(xyz=(0.35, 0.0, 0.62)),
        axis=(0.0, -1.0, 0.0),
        motion_limits=MotionLimits(
            effort=120.0,
            velocity=0.9,
            lower=math.radians(-45.0),
            upper=math.radians(70.0),
        ),
    )

    return model


def run_tests() -> TestReport:
    ctx = TestContext(object_model)
    tower_support = object_model.get_part("tower_support")
    pan_yoke = object_model.get_part("pan_yoke")
    spotlight_head = object_model.get_part("spotlight_head")
    pan_rotation = object_model.get_articulation("pan_rotation")
    tilt_motion = object_model.get_articulation("tilt_motion")

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
        "pan_axis_is_vertical",
        tuple(round(v, 6) for v in pan_rotation.axis) == (0.0, 0.0, 1.0),
        f"Expected vertical pan axis, got {pan_rotation.axis}.",
    )
    ctx.check(
        "tilt_axis_raises_beam",
        tuple(round(v, 6) for v in tilt_motion.axis) == (0.0, -1.0, 0.0),
        f"Expected tilt axis (0, -1, 0), got {tilt_motion.axis}.",
    )
    ctx.expect_contact(
        pan_yoke,
        tower_support,
        elem_a="turntable_ring",
        elem_b="top_turntable",
        name="pan_turntable_is_seated",
    )
    ctx.expect_contact(
        spotlight_head,
        pan_yoke,
        elem_a="left_trunnion",
        elem_b="left_bearing_block",
        name="left_tilt_bearing_is_supported",
    )
    ctx.expect_contact(
        spotlight_head,
        pan_yoke,
        elem_a="right_trunnion",
        elem_b="right_bearing_block",
        name="right_tilt_bearing_is_supported",
    )
    ctx.expect_gap(
        spotlight_head,
        pan_yoke,
        axis="z",
        positive_elem="barrel_shell",
        negative_elem="gearbox_housing",
        min_gap=0.06,
        name="head_clears_pan_housing",
    )

    rest_hood_center = _aabb_center(
        ctx.part_element_world_aabb(spotlight_head, elem="hood_shell")
    )
    raised_hood_center = None
    quarter_pan_center = None

    with ctx.pose({tilt_motion: math.radians(35.0)}):
        raised_hood_center = _aabb_center(
            ctx.part_element_world_aabb(spotlight_head, elem="hood_shell")
        )

    with ctx.pose({pan_rotation: math.pi / 2.0}):
        quarter_pan_center = _aabb_center(
            ctx.part_element_world_aabb(spotlight_head, elem="hood_shell")
        )

    ctx.check(
        "positive_tilt_lifts_front_hood",
        rest_hood_center is not None
        and raised_hood_center is not None
        and raised_hood_center[2] > rest_hood_center[2] + 0.12,
        (
            "Positive tilt should raise the front hood; "
            f"rest={rest_hood_center}, raised={raised_hood_center}."
        ),
    )
    ctx.check(
        "pan_motion_swings_head_around_mast",
        rest_hood_center is not None
        and quarter_pan_center is not None
        and abs(quarter_pan_center[0]) < 0.10
        and abs(quarter_pan_center[1]) > 0.70,
        (
            "Quarter-turn pan should move the head off the X axis onto the Y axis; "
            f"rest={rest_hood_center}, quarter_turn={quarter_pan_center}."
        ),
    )

    return ctx.report()


# >>> USER_CODE_END

object_model = build_object_model()
