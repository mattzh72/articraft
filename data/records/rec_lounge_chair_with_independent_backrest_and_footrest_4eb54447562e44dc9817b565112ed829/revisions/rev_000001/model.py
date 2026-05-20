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
    rounded_rect_profile,
    section_loft,
)


def _save_mesh(name: str, geometry):
    return mesh_from_geometry(geometry, name)


def _rounded_section(width: float, depth: float, radius: float, z: float) -> list[tuple[float, float, float]]:
    safe_radius = min(radius, (width * 0.5) - 1e-4, (depth * 0.5) - 1e-4)
    return [(x, y, z) for x, y in rounded_rect_profile(width, depth, safe_radius, corner_segments=10)]


def _soft_block(
    width: float,
    depth: float,
    height: float,
    *,
    radius: float,
    belly: float = 0.012,
    top_inset: float = 0.006,
    bottom_inset: float = 0.010,
):
    lower_w = max(width - bottom_inset, 0.02)
    lower_d = max(depth - bottom_inset, 0.02)
    crown_w = width + belly
    crown_d = depth + belly * 0.55
    upper_w = max(width - top_inset, 0.02)
    upper_d = max(depth - top_inset * 0.8, 0.02)
    return section_loft(
        [
            _rounded_section(lower_w, lower_d, radius * 0.92, 0.0),
            _rounded_section(crown_w, crown_d, radius, height * 0.28),
            _rounded_section(crown_w * 0.995, crown_d * 0.99, radius * 0.98, height * 0.72),
            _rounded_section(upper_w, upper_d, radius * 0.9, height),
        ]
    )


def build_object_model() -> ArticulatedObject:
    model = ArticulatedObject(name="cinema_recliner")

    upholstery = model.material("upholstery", rgba=(0.23, 0.13, 0.08, 1.0))
    accent_upholstery = model.material("accent_upholstery", rgba=(0.28, 0.16, 0.10, 1.0))
    base_black = model.material("base_black", rgba=(0.09, 0.09, 0.10, 1.0))
    bracket_black = model.material("bracket_black", rgba=(0.14, 0.14, 0.15, 1.0))
    pin_metal = model.material("pin_metal", rgba=(0.52, 0.54, 0.56, 1.0))

    base = model.part("base")
    base.visual(
        Box((0.90, 0.68, 0.06)),
        origin=Origin(xyz=(0.0, 0.0, 0.03)),
        material=base_black,
        name="base_plinth",
    )
    base.visual(
        Box((0.70, 0.52, 0.10)),
        origin=Origin(xyz=(0.0, 0.0, 0.11)),
        material=base_black,
        name="base_pedestal",
    )
    base.inertial = Inertial.from_geometry(
        Box((0.90, 0.68, 0.16)),
        mass=26.0,
        origin=Origin(xyz=(0.0, 0.0, 0.08)),
    )

    seat_body = model.part("seat_body")
    seat_body.visual(
        Box((0.78, 0.64, 0.20)),
        origin=Origin(xyz=(0.0, 0.0, 0.10)),
        material=accent_upholstery,
        name="seat_core",
    )
    seat_body.visual(
        _save_mesh("seat_cushion", _soft_block(0.64, 0.58, 0.12, radius=0.07, belly=0.016)),
        origin=Origin(xyz=(0.0, 0.02, 0.20)),
        material=upholstery,
        name="seat_cushion",
    )
    seat_body.visual(
        _save_mesh("left_arm_pad", _soft_block(0.17, 0.74, 0.24, radius=0.055, belly=0.014)),
        origin=Origin(xyz=(-0.415, 0.04, 0.20)),
        material=upholstery,
        name="left_arm_pad",
    )
    seat_body.visual(
        _save_mesh("right_arm_pad", _soft_block(0.17, 0.74, 0.24, radius=0.055, belly=0.014)),
        origin=Origin(xyz=(0.415, 0.04, 0.20)),
        material=upholstery,
        name="right_arm_pad",
    )
    seat_body.visual(
        Box((0.05, 0.12, 0.20)),
        origin=Origin(xyz=(-0.325, 0.375, 0.10)),
        material=bracket_black,
        name="left_footrest_bracket",
    )
    seat_body.visual(
        Box((0.05, 0.12, 0.20)),
        origin=Origin(xyz=(0.325, 0.375, 0.10)),
        material=bracket_black,
        name="right_footrest_bracket",
    )
    seat_body.visual(
        Cylinder(radius=0.022, length=0.05),
        origin=Origin(xyz=(-0.325, 0.34, 0.20), rpy=(0.0, math.pi / 2.0, 0.0)),
        material=pin_metal,
        name="left_footrest_hinge_sleeve",
    )
    seat_body.visual(
        Cylinder(radius=0.022, length=0.05),
        origin=Origin(xyz=(0.325, 0.34, 0.20), rpy=(0.0, math.pi / 2.0, 0.0)),
        material=pin_metal,
        name="right_footrest_hinge_sleeve",
    )
    seat_body.visual(
        Box((0.06, 0.08, 0.18)),
        origin=Origin(xyz=(-0.355, -0.355, 0.31)),
        material=bracket_black,
        name="left_back_pivot_mount",
    )
    seat_body.visual(
        Box((0.06, 0.08, 0.18)),
        origin=Origin(xyz=(0.355, -0.355, 0.31)),
        material=bracket_black,
        name="right_back_pivot_mount",
    )
    seat_body.visual(
        Cylinder(radius=0.03, length=0.05),
        origin=Origin(xyz=(-0.355, -0.355, 0.31), rpy=(0.0, math.pi / 2.0, 0.0)),
        material=pin_metal,
        name="left_back_pivot_cap",
    )
    seat_body.visual(
        Cylinder(radius=0.03, length=0.05),
        origin=Origin(xyz=(0.355, -0.355, 0.31), rpy=(0.0, math.pi / 2.0, 0.0)),
        material=pin_metal,
        name="right_back_pivot_cap",
    )
    seat_body.inertial = Inertial.from_geometry(
        Box((0.98, 0.84, 0.44)),
        mass=38.0,
        origin=Origin(xyz=(0.0, 0.0, 0.22)),
    )

    backrest = model.part("backrest")
    backrest.visual(
        _save_mesh("backrest_shell", _soft_block(0.64, 0.12, 0.78, radius=0.075, belly=0.015)),
        origin=Origin(xyz=(0.0, -0.06, 0.0)),
        material=accent_upholstery,
        name="backrest_shell",
    )
    backrest.visual(
        _save_mesh("backrest_lower_pad", _soft_block(0.54, 0.08, 0.28, radius=0.05, belly=0.014)),
        origin=Origin(xyz=(0.0, -0.01, 0.10)),
        material=upholstery,
        name="backrest_lower_pad",
    )
    backrest.visual(
        _save_mesh("backrest_upper_pad", _soft_block(0.54, 0.08, 0.22, radius=0.05, belly=0.012)),
        origin=Origin(xyz=(0.0, -0.005, 0.50)),
        material=upholstery,
        name="backrest_upper_pad",
    )
    backrest.visual(
        Box((0.07, 0.06, 0.12)),
        origin=Origin(xyz=(-0.272, -0.01, 0.06)),
        material=accent_upholstery,
        name="left_back_pivot_cheek",
    )
    backrest.visual(
        Box((0.07, 0.06, 0.12)),
        origin=Origin(xyz=(0.272, -0.01, 0.06)),
        material=accent_upholstery,
        name="right_back_pivot_cheek",
    )
    backrest.visual(
        Cylinder(radius=0.03, length=0.04),
        origin=Origin(xyz=(-0.305, -0.005, 0.0), rpy=(0.0, math.pi / 2.0, 0.0)),
        material=pin_metal,
        name="left_back_pivot_pin",
    )
    backrest.visual(
        Cylinder(radius=0.03, length=0.04),
        origin=Origin(xyz=(0.305, -0.005, 0.0), rpy=(0.0, math.pi / 2.0, 0.0)),
        material=pin_metal,
        name="right_back_pivot_pin",
    )
    backrest.inertial = Inertial.from_geometry(
        Box((0.68, 0.18, 0.80)),
        mass=18.0,
        origin=Origin(xyz=(0.0, -0.04, 0.40)),
    )

    footrest = model.part("footrest")
    footrest.visual(
        _save_mesh("ottoman_panel", _soft_block(0.52, 0.07, 0.24, radius=0.05, belly=0.012)),
        origin=Origin(xyz=(0.0, 0.035, -0.24)),
        material=upholstery,
        name="ottoman_panel",
    )
    footrest.visual(
        Box((0.05, 0.05, 0.18)),
        origin=Origin(xyz=(-0.275, 0.035, -0.15)),
        material=bracket_black,
        name="left_ottoman_lug",
    )
    footrest.visual(
        Box((0.05, 0.05, 0.18)),
        origin=Origin(xyz=(0.275, 0.035, -0.15)),
        material=bracket_black,
        name="right_ottoman_lug",
    )
    footrest.inertial = Inertial.from_geometry(
        Box((0.60, 0.08, 0.24)),
        mass=7.0,
        origin=Origin(xyz=(0.0, 0.035, -0.12)),
    )

    model.articulation(
        "base_to_seat",
        ArticulationType.FIXED,
        parent=base,
        child=seat_body,
        origin=Origin(xyz=(0.0, 0.0, 0.16)),
    )
    model.articulation(
        "seat_to_backrest",
        ArticulationType.REVOLUTE,
        parent=seat_body,
        child=backrest,
        origin=Origin(xyz=(0.0, -0.355, 0.31)),
        axis=(1.0, 0.0, 0.0),
        motion_limits=MotionLimits(effort=18.0, velocity=1.2, lower=0.0, upper=0.55),
    )
    model.articulation(
        "seat_to_footrest",
        ArticulationType.REVOLUTE,
        parent=seat_body,
        child=footrest,
        origin=Origin(xyz=(0.0, 0.34, 0.20)),
        axis=(1.0, 0.0, 0.0),
        motion_limits=MotionLimits(effort=14.0, velocity=1.5, lower=0.0, upper=1.35),
    )

    return model


def run_tests() -> TestReport:
    ctx = TestContext(object_model)
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

    base = object_model.get_part("base")
    seat_body = object_model.get_part("seat_body")
    backrest = object_model.get_part("backrest")
    footrest = object_model.get_part("footrest")

    back_hinge = object_model.get_articulation("seat_to_backrest")
    foot_hinge = object_model.get_articulation("seat_to_footrest")

    ctx.expect_contact(seat_body, base, name="seat_body_is_supported_by_base")
    ctx.expect_contact(backrest, seat_body, name="backrest_touches_rear_pivots")
    ctx.expect_contact(footrest, seat_body, name="footrest_is_captured_by_front_brackets")

    ctx.expect_origin_gap(seat_body, base, axis="z", min_gap=0.15, max_gap=0.17, name="seat_rises_from_low_base")
    ctx.expect_origin_gap(seat_body, backrest, axis="y", min_gap=0.30, max_gap=0.36, name="backrest_pivot_sits_behind_seat")
    ctx.expect_origin_gap(footrest, seat_body, axis="y", min_gap=0.33, max_gap=0.35, name="footrest_hinge_sits_at_front_edge")

    ctx.check(
        "backrest_axis_is_horizontal",
        tuple(round(value, 6) for value in back_hinge.axis) == (1.0, 0.0, 0.0),
        f"Unexpected backrest axis: {back_hinge.axis}",
    )
    ctx.check(
        "footrest_axis_is_horizontal",
        tuple(round(value, 6) for value in foot_hinge.axis) == (1.0, 0.0, 0.0),
        f"Unexpected footrest axis: {foot_hinge.axis}",
    )

    back_limits = back_hinge.motion_limits
    foot_limits = foot_hinge.motion_limits
    ctx.check(
        "backrest_limits_match_recline_range",
        back_limits is not None
        and back_limits.lower == 0.0
        and back_limits.upper is not None
        and 0.45 <= back_limits.upper <= 0.65,
        f"Unexpected backrest limits: {back_limits}",
    )
    ctx.check(
        "footrest_limits_match_foldout_range",
        foot_limits is not None
        and foot_limits.lower == 0.0
        and foot_limits.upper is not None
        and 1.10 <= foot_limits.upper <= 1.45,
        f"Unexpected footrest limits: {foot_limits}",
    )

    rest_back_aabb = ctx.part_world_aabb(backrest)
    rest_foot_aabb = ctx.part_world_aabb(footrest)

    if rest_back_aabb is not None:
        with ctx.pose({back_hinge: 0.45}):
            reclined_back_aabb = ctx.part_world_aabb(backrest)
            if reclined_back_aabb is not None:
                ctx.check(
                    "backrest_reclines_rearward",
                    reclined_back_aabb[0][1] < rest_back_aabb[0][1] - 0.08,
                    f"Backrest did not swing rearward enough: rest={rest_back_aabb}, reclined={reclined_back_aabb}",
                )
            ctx.expect_contact(backrest, seat_body, name="backrest_stays_on_pivots_when_reclined")

    if rest_foot_aabb is not None:
        with ctx.pose({foot_hinge: 1.20}):
            extended_foot_aabb = ctx.part_world_aabb(footrest)
            if extended_foot_aabb is not None:
                ctx.check(
                    "footrest_swings_forward",
                    extended_foot_aabb[1][1] > rest_foot_aabb[1][1] + 0.16,
                    f"Footrest did not swing forward enough: rest={rest_foot_aabb}, extended={extended_foot_aabb}",
                )
                ctx.check(
                    "footrest_lifts_into_ottoman_position",
                    extended_foot_aabb[0][2] > rest_foot_aabb[0][2] + 0.10,
                    f"Footrest did not lift when extended: rest={rest_foot_aabb}, extended={extended_foot_aabb}",
                )
            ctx.expect_contact(footrest, seat_body, name="footrest_remains_supported_when_extended")

    return ctx.report()


# >>> USER_CODE_END

object_model = build_object_model()
