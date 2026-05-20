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


def _y_cylinder_origin(
    *,
    xyz: tuple[float, float, float],
) -> Origin:
    return Origin(xyz=xyz, rpy=(math.pi / 2.0, 0.0, 0.0))


def _axis_matches(
    actual: tuple[float, float, float] | None,
    expected: tuple[float, float, float],
    tol: float = 1e-6,
) -> bool:
    if actual is None:
        return False
    return all(abs(a - b) <= tol for a, b in zip(actual, expected))


def build_object_model() -> ArticulatedObject:
    model = ArticulatedObject(name="retractable_dock_bridge")

    concrete = model.material("concrete", rgba=(0.66, 0.66, 0.67, 1.0))
    dock_steel = model.material("dock_steel", rgba=(0.28, 0.30, 0.33, 1.0))
    galvanized = model.material("galvanized", rgba=(0.71, 0.73, 0.75, 1.0))
    safety_yellow = model.material("safety_yellow", rgba=(0.86, 0.73, 0.14, 1.0))
    bumper_rubber = model.material("bumper_rubber", rgba=(0.10, 0.10, 0.11, 1.0))

    bridge_top_z = 1.36
    platform_length = 1.55
    platform_width = 1.86
    deck_thickness = 0.04

    dock_housing = model.part("dock_housing")
    dock_housing.visual(
        Box((2.60, 3.20, 0.18)),
        origin=Origin(xyz=(1.30, 0.0, 0.09)),
        material=concrete,
        name="apron",
    )
    dock_housing.visual(
        Box((0.70, 3.20, 1.02)),
        origin=Origin(xyz=(-0.35, 0.0, 0.51)),
        material=concrete,
        name="dock_face_lower",
    )
    dock_housing.visual(
        Box((0.70, 0.58, 0.62)),
        origin=Origin(xyz=(-0.35, -1.31, 1.13)),
        material=concrete,
        name="dock_face_left_cheek",
    )
    dock_housing.visual(
        Box((0.70, 0.58, 0.62)),
        origin=Origin(xyz=(-0.35, 1.31, 1.13)),
        material=concrete,
        name="dock_face_right_cheek",
    )
    dock_housing.visual(
        Box((0.80, 3.20, 0.10)),
        origin=Origin(xyz=(-0.10, 0.0, 1.46)),
        material=concrete,
        name="dock_cap",
    )
    dock_housing.visual(
        Box((1.00, 1.90, 0.05)),
        origin=Origin(xyz=(-0.05, 0.0, 1.56)),
        material=dock_steel,
        name="housing_roof",
    )
    dock_housing.visual(
        Box((1.00, 0.08, 0.34)),
        origin=Origin(xyz=(-0.05, -1.01, 1.39)),
        material=dock_steel,
        name="left_housing_wall",
    )
    dock_housing.visual(
        Box((1.00, 0.08, 0.34)),
        origin=Origin(xyz=(-0.05, 1.01, 1.39)),
        material=dock_steel,
        name="right_housing_wall",
    )
    dock_housing.visual(
        Box((0.10, 1.98, 0.34)),
        origin=Origin(xyz=(-0.55, 0.0, 1.39)),
        material=dock_steel,
        name="rear_bulkhead",
    )
    dock_housing.visual(
        Box((1.10, 0.70, 0.06)),
        origin=Origin(xyz=(-0.05, -0.60, 1.27)),
        material=dock_steel,
        name="left_floor_plate",
    )
    dock_housing.visual(
        Box((1.10, 0.70, 0.06)),
        origin=Origin(xyz=(-0.05, 0.60, 1.27)),
        material=dock_steel,
        name="right_floor_plate",
    )
    dock_housing.visual(
        Box((0.48, 0.22, 0.06)),
        origin=Origin(xyz=(-0.31, 0.0, 1.27)),
        material=dock_steel,
        name="rear_slot_bridge",
    )
    dock_housing.visual(
        Box((1.50, 0.14, 0.06)),
        origin=Origin(xyz=(0.15, -0.73, 1.27)),
        material=dock_steel,
        name="left_rail",
    )
    dock_housing.visual(
        Box((1.50, 0.14, 0.06)),
        origin=Origin(xyz=(0.15, 0.73, 1.27)),
        material=dock_steel,
        name="right_rail",
    )
    dock_housing.visual(
        Box((0.26, 0.18, 0.10)),
        origin=Origin(xyz=(0.02, 0.0, 1.064)),
        material=dock_steel,
        name="slot_base",
    )
    dock_housing.visual(
        Box((0.20, 0.05, 0.106)),
        origin=Origin(xyz=(0.02, -0.065, 1.167)),
        material=dock_steel,
        name="slot_left_guide",
    )
    dock_housing.visual(
        Box((0.20, 0.05, 0.106)),
        origin=Origin(xyz=(0.02, 0.065, 1.167)),
        material=dock_steel,
        name="slot_right_guide",
    )
    dock_housing.visual(
        Box((0.12, 0.25, 0.35)),
        origin=Origin(xyz=(0.06, -1.15, 1.08)),
        material=bumper_rubber,
        name="left_bumper",
    )
    dock_housing.visual(
        Box((0.12, 0.25, 0.35)),
        origin=Origin(xyz=(0.06, 1.15, 1.08)),
        material=bumper_rubber,
        name="right_bumper",
    )
    dock_housing.inertial = Inertial.from_geometry(
        Box((3.30, 3.20, 1.45)),
        mass=3800.0,
        origin=Origin(xyz=(0.95, 0.0, 0.725)),
    )

    platform = model.part("platform")
    platform.visual(
        Box((platform_length, platform_width, deck_thickness)),
        origin=Origin(xyz=(platform_length * 0.5, 0.0, -deck_thickness * 0.5)),
        material=galvanized,
        name="deck_plate",
    )
    platform.visual(
        Box((1.10, 0.06, 0.08)),
        origin=Origin(xyz=(1.00, -0.90, 0.04)),
        material=dock_steel,
        name="left_curb",
    )
    platform.visual(
        Box((1.10, 0.06, 0.08)),
        origin=Origin(xyz=(1.00, 0.90, 0.04)),
        material=dock_steel,
        name="right_curb",
    )
    platform.visual(
        Box((1.00, 0.12, 0.02)),
        origin=Origin(xyz=(0.40, -0.73, -0.05)),
        material=dock_steel,
        name="left_runner",
    )
    platform.visual(
        Box((1.00, 0.12, 0.02)),
        origin=Origin(xyz=(0.40, 0.73, -0.05)),
        material=dock_steel,
        name="right_runner",
    )
    platform.visual(
        Box((0.12, 1.68, 0.05)),
        origin=Origin(xyz=(1.38, 0.0, -0.065)),
        material=dock_steel,
        name="front_stiffener",
    )
    platform.visual(
        Box((0.10, 0.26, 0.044)),
        origin=Origin(xyz=(1.50, -0.67, -0.062)),
        material=dock_steel,
        name="left_hinge_web",
    )
    platform.visual(
        Box((0.10, 0.26, 0.044)),
        origin=Origin(xyz=(1.50, 0.0, -0.062)),
        material=dock_steel,
        name="center_hinge_web",
    )
    platform.visual(
        Box((0.10, 0.26, 0.044)),
        origin=Origin(xyz=(1.50, 0.67, -0.062)),
        material=dock_steel,
        name="right_hinge_web",
    )
    platform.visual(
        Box((0.04, 0.04, 0.08)),
        origin=Origin(xyz=(0.12, -0.08, -0.08)),
        material=dock_steel,
        name="brace_ear_left",
    )
    platform.visual(
        Box((0.04, 0.04, 0.08)),
        origin=Origin(xyz=(0.12, 0.08, -0.08)),
        material=dock_steel,
        name="brace_ear_right",
    )
    platform.inertial = Inertial.from_geometry(
        Box((platform_length, platform_width, 0.16)),
        mass=420.0,
        origin=Origin(xyz=(platform_length * 0.5, 0.0, -0.02)),
    )

    lip = model.part("lip")
    lip.visual(
        Box((0.32, 1.78, 0.028)),
        origin=Origin(xyz=(0.16, 0.0, 0.046)),
        material=galvanized,
        name="lip_plate",
    )
    lip.visual(
        Box((0.09, 1.78, 0.018)),
        origin=Origin(xyz=(0.275, 0.0, 0.023), rpy=(0.0, -0.18, 0.0)),
        material=galvanized,
        name="lip_bevel",
    )
    lip.visual(
        Box((0.08, 0.26, 0.04)),
        origin=Origin(xyz=(0.04, -0.67, 0.02)),
        material=dock_steel,
        name="lip_hinge_strap_left",
    )
    lip.visual(
        Box((0.08, 0.26, 0.04)),
        origin=Origin(xyz=(0.04, 0.67, 0.02)),
        material=dock_steel,
        name="lip_hinge_strap_right",
    )
    lip.inertial = Inertial.from_geometry(
        Box((0.32, 1.78, 0.06)),
        mass=55.0,
        origin=Origin(xyz=(0.16, 0.0, 0.03)),
    )

    safety_brace = model.part("safety_brace")
    safety_brace.visual(
        Box((0.18, 0.06, 0.018)),
        origin=Origin(xyz=(0.09, 0.0, -0.009)),
        material=safety_yellow,
        name="brace_bar",
    )
    safety_brace.visual(
        Cylinder(radius=0.014, length=0.12),
        origin=_y_cylinder_origin(xyz=(0.0, 0.0, 0.0)),
        material=dock_steel,
        name="brace_barrel",
    )
    safety_brace.visual(
        Cylinder(radius=0.02, length=0.08),
        origin=_y_cylinder_origin(xyz=(0.172, 0.0, -0.01)),
        material=safety_yellow,
        name="brace_tip",
    )
    safety_brace.inertial = Inertial.from_geometry(
        Box((0.20, 0.08, 0.05)),
        mass=8.0,
        origin=Origin(xyz=(0.09, 0.0, -0.01)),
    )

    model.articulation(
        "bridge_slide",
        ArticulationType.PRISMATIC,
        parent=dock_housing,
        child=platform,
        origin=Origin(xyz=(-0.10, 0.0, bridge_top_z)),
        axis=(1.0, 0.0, 0.0),
        motion_limits=MotionLimits(
            effort=12000.0,
            velocity=0.20,
            lower=-0.35,
            upper=0.40,
        ),
    )
    model.articulation(
        "platform_to_lip",
        ArticulationType.REVOLUTE,
        parent=platform,
        child=lip,
        origin=Origin(xyz=(platform_length, 0.0, -0.06)),
        axis=(0.0, 1.0, 0.0),
        motion_limits=MotionLimits(
            effort=4000.0,
            velocity=0.9,
            lower=-0.10,
            upper=0.75,
        ),
    )
    model.articulation(
        "platform_to_brace",
        ArticulationType.REVOLUTE,
        parent=platform,
        child=safety_brace,
        origin=Origin(xyz=(0.12, 0.0, -0.055)),
        axis=(0.0, 1.0, 0.0),
        motion_limits=MotionLimits(
            effort=300.0,
            velocity=1.2,
            lower=0.0,
            upper=1.45,
        ),
    )

    return model


def run_tests() -> TestReport:
    ctx = TestContext(object_model)
    ctx.check_model_valid()
    ctx.check_mesh_assets_ready()

    dock_housing = object_model.get_part("dock_housing")
    platform = object_model.get_part("platform")
    lip = object_model.get_part("lip")
    safety_brace = object_model.get_part("safety_brace")

    bridge_slide = object_model.get_articulation("bridge_slide")
    platform_to_lip = object_model.get_articulation("platform_to_lip")
    platform_to_brace = object_model.get_articulation("platform_to_brace")

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

    ctx.expect_contact(
        platform,
        dock_housing,
        elem_a="left_runner",
        elem_b="left_rail",
        name="left_runner_bears_on_left_rail",
    )
    ctx.expect_contact(
        platform,
        dock_housing,
        elem_a="right_runner",
        elem_b="right_rail",
        name="right_runner_bears_on_right_rail",
    )
    ctx.expect_contact(
        lip,
        platform,
        elem_a="lip_hinge_strap_left",
        elem_b="left_hinge_web",
        name="lip_left_strap_contacts_platform_hinge_web",
    )
    ctx.expect_contact(
        lip,
        platform,
        elem_a="lip_hinge_strap_right",
        elem_b="right_hinge_web",
        name="lip_right_strap_contacts_platform_hinge_web",
    )
    ctx.expect_contact(
        safety_brace,
        platform,
        elem_a="brace_barrel",
        elem_b="brace_ear_left",
        name="brace_barrel_contacts_left_ear",
    )
    ctx.expect_contact(
        safety_brace,
        platform,
        elem_a="brace_barrel",
        elem_b="brace_ear_right",
        name="brace_barrel_contacts_right_ear",
    )

    ctx.check(
        "bridge_slide_axis_is_x",
        _axis_matches(bridge_slide.axis, (1.0, 0.0, 0.0)),
        f"Expected bridge_slide axis (1,0,0), got {bridge_slide.axis}",
    )
    ctx.check(
        "lip_hinge_axis_is_y",
        _axis_matches(platform_to_lip.axis, (0.0, 1.0, 0.0)),
        f"Expected platform_to_lip axis (0,1,0), got {platform_to_lip.axis}",
    )
    ctx.check(
        "brace_hinge_axis_is_y",
        _axis_matches(platform_to_brace.axis, (0.0, 1.0, 0.0)),
        f"Expected platform_to_brace axis (0,1,0), got {platform_to_brace.axis}",
    )

    platform_rest = ctx.part_world_position(platform)
    with ctx.pose({bridge_slide: 0.35}):
        platform_extended = ctx.part_world_position(platform)
        ctx.fail_if_parts_overlap_in_current_pose(name="no_overlap_when_platform_extended")
    if platform_rest is not None and platform_extended is not None:
        ctx.check(
            "platform_translates_outward",
            platform_extended[0] - platform_rest[0] >= 0.34,
            (
                "Expected the bridge to slide outward by at least 0.34 m; "
                f"got {platform_extended[0] - platform_rest[0]:.3f} m"
            ),
        )
    else:
        ctx.fail("platform_position_probe_available", "Could not measure platform world position.")

    lip_rest_aabb = ctx.part_world_aabb(lip)
    platform_rest_aabb = ctx.part_world_aabb(platform)
    with ctx.pose({platform_to_lip: 0.60}):
        lip_lowered_aabb = ctx.part_world_aabb(lip)
        ctx.fail_if_parts_overlap_in_current_pose(name="no_overlap_with_lip_lowered")
    if lip_rest_aabb is not None and lip_lowered_aabb is not None and platform_rest_aabb is not None:
        ctx.check(
            "lip_rotates_down_below_platform_plane",
            lip_lowered_aabb[0][2] < platform_rest_aabb[0][2] - 0.08,
            (
                "Lowered lip should hang noticeably below the bridge deck; "
                f"lip min z={lip_lowered_aabb[0][2]:.3f}, platform min z={platform_rest_aabb[0][2]:.3f}"
            ),
        )
        ctx.check(
            "lip_motion_changes_pose",
            lip_lowered_aabb[0][2] < lip_rest_aabb[0][2] - 0.08,
            (
                "Lowering the lip should move its front edge downward; "
                f"rest min z={lip_rest_aabb[0][2]:.3f}, lowered min z={lip_lowered_aabb[0][2]:.3f}"
            ),
        )
    else:
        ctx.fail("lip_pose_probe_available", "Could not measure lip/platform AABBs.")

    with ctx.pose({platform_to_brace: 1.40}):
        ctx.fail_if_parts_overlap_in_current_pose(name="no_overlap_with_brace_dropped")
        ctx.expect_contact(
            safety_brace,
            dock_housing,
            elem_a="brace_tip",
            elem_b="slot_base",
            name="brace_tip_seats_on_slot_base",
        )
        ctx.expect_contact(
            safety_brace,
            dock_housing,
            elem_a="brace_tip",
            elem_b="slot_left_guide",
            name="brace_tip_contacts_left_slot_guide",
        )
        ctx.expect_contact(
            safety_brace,
            dock_housing,
            elem_a="brace_tip",
            elem_b="slot_right_guide",
            name="brace_tip_contacts_right_slot_guide",
        )

    return ctx.report()


# >>> USER_CODE_END

object_model = build_object_model()
