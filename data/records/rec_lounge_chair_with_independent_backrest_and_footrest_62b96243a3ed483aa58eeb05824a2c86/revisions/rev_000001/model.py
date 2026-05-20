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
    MotionLimits,
    Origin,
    TestContext,
    TestReport,
    mesh_from_geometry,
    rounded_rect_profile,
)


def rounded_slab_mesh(
    name: str,
    *,
    length: float,
    width: float,
    thickness: float,
    corner_radius: float,
):
    geom = ExtrudeGeometry.centered(
        rounded_rect_profile(length, width, corner_radius),
        thickness,
        cap=True,
        closed=True,
    )
    return mesh_from_geometry(geom, name)


def rounded_panel_mesh(
    name: str,
    *,
    height: float,
    width: float,
    thickness: float,
    corner_radius: float,
):
    geom = ExtrudeGeometry.centered(
        rounded_rect_profile(height, width, corner_radius),
        thickness,
        cap=True,
        closed=True,
    )
    geom.rotate_y(pi / 2.0)
    return mesh_from_geometry(geom, name)


def build_object_model() -> ArticulatedObject:
    model = ArticulatedObject(name="modern_lounge_chair")

    upholstery = model.material("upholstery", rgba=(0.77, 0.73, 0.67, 1.0))
    shell = model.material("shell", rgba=(0.22, 0.20, 0.18, 1.0))
    metal = model.material("metal", rgba=(0.24, 0.25, 0.27, 1.0))

    seat_base = model.part("seat_base")
    seat_base.visual(
        rounded_slab_mesh(
            "seat_plinth_mesh",
            length=0.70,
            width=0.82,
            thickness=0.20,
            corner_radius=0.05,
        ),
        origin=Origin(xyz=(-0.06, 0.0, 0.10)),
        material=shell,
        name="plinth",
    )
    seat_base.visual(
        rounded_slab_mesh(
            "seat_cushion_mesh",
            length=0.90,
            width=0.88,
            thickness=0.14,
            corner_radius=0.07,
        ),
        origin=Origin(xyz=(0.0, 0.0, 0.27)),
        material=upholstery,
        name="seat_cushion",
    )
    seat_base.visual(
        Box((0.06, 0.06, 0.14)),
        origin=Origin(xyz=(-0.38, 0.43, 0.29)),
        material=metal,
        name="left_pivot_support",
    )
    seat_base.visual(
        Box((0.06, 0.06, 0.14)),
        origin=Origin(xyz=(-0.38, -0.43, 0.29)),
        material=metal,
        name="right_pivot_support",
    )

    backrest = model.part("backrest")
    backrest.visual(
        rounded_panel_mesh(
            "back_shell_mesh",
            height=0.52,
            width=0.76,
            thickness=0.018,
            corner_radius=0.05,
        ),
        origin=Origin(xyz=(-0.014, 0.0, 0.245), rpy=(0.0, -0.35, 0.0)),
        material=shell,
        name="back_shell",
    )
    backrest.visual(
        rounded_panel_mesh(
            "backrest_cushion_mesh",
            height=0.50,
            width=0.74,
            thickness=0.08,
            corner_radius=0.06,
        ),
        origin=Origin(xyz=(0.03, 0.0, 0.24), rpy=(0.0, -0.35, 0.0)),
        material=upholstery,
        name="backrest_cushion",
    )
    backrest.visual(
        Box((0.075, 0.74, 0.09)),
        origin=Origin(xyz=(0.0, 0.0, 0.10), rpy=(0.0, -0.35, 0.0)),
        material=shell,
        name="lower_frame",
    )
    backrest.visual(
        Box((0.055, 0.05, 0.19)),
        origin=Origin(xyz=(0.0, 0.345, 0.10), rpy=(0.0, -0.35, 0.0)),
        material=metal,
        name="left_back_arm",
    )
    backrest.visual(
        Box((0.055, 0.05, 0.19)),
        origin=Origin(xyz=(0.0, -0.345, 0.10), rpy=(0.0, -0.35, 0.0)),
        material=metal,
        name="right_back_arm",
    )
    backrest.visual(
        Cylinder(radius=0.028, length=0.05),
        origin=Origin(xyz=(0.0, 0.375, 0.0), rpy=(pi / 2.0, 0.0, 0.0)),
        material=metal,
        name="left_pivot_cap",
    )
    backrest.visual(
        Cylinder(radius=0.028, length=0.05),
        origin=Origin(xyz=(0.0, -0.375, 0.0), rpy=(pi / 2.0, 0.0, 0.0)),
        material=metal,
        name="right_pivot_cap",
    )

    bracket = model.part("footrest_bracket")
    bracket.visual(
        Box((0.02, 0.515, 0.12)),
        origin=Origin(xyz=(0.30, 0.0, 0.14)),
        material=metal,
        name="mount_plate",
    )
    bracket.visual(
        Box((0.14, 0.025, 0.025)),
        origin=Origin(xyz=(0.38, 0.27, 0.14)),
        material=metal,
        name="left_link_arm",
    )
    bracket.visual(
        Box((0.14, 0.025, 0.025)),
        origin=Origin(xyz=(0.38, -0.27, 0.14)),
        material=metal,
        name="right_link_arm",
    )
    bracket.visual(
        Cylinder(radius=0.012, length=0.54),
        origin=Origin(xyz=(0.45, 0.0, 0.14), rpy=(pi / 2.0, 0.0, 0.0)),
        material=metal,
        name="hinge_spindle",
    )

    footrest = model.part("footrest_panel")
    footrest.visual(
        rounded_panel_mesh(
            "footrest_panel_mesh",
            height=0.15,
            width=0.48,
            thickness=0.045,
            corner_radius=0.03,
        ),
        origin=Origin(xyz=(-0.085, 0.0, -0.095)),
        material=upholstery,
        name="footrest_panel",
    )
    footrest.visual(
        Box((0.09, 0.16, 0.07)),
        origin=Origin(xyz=(-0.0225, 0.0, -0.025)),
        material=metal,
        name="hinge_cartridge",
    )
    footrest.visual(
        Box((0.11, 0.18, 0.02)),
        origin=Origin(xyz=(-0.055, 0.0, -0.055)),
        material=metal,
        name="clip_channel",
    )

    model.articulation(
        "seat_to_backrest",
        ArticulationType.REVOLUTE,
        parent=seat_base,
        child=backrest,
        origin=Origin(xyz=(-0.38, 0.0, 0.365)),
        axis=(0.0, 1.0, 0.0),
        motion_limits=MotionLimits(
            effort=20.0,
            velocity=1.5,
            lower=-0.12,
            upper=0.35,
        ),
    )
    model.articulation(
        "seat_to_footrest_bracket",
        ArticulationType.FIXED,
        parent=seat_base,
        child=bracket,
    )
    model.articulation(
        "bracket_to_footrest",
        ArticulationType.REVOLUTE,
        parent=bracket,
        child=footrest,
        origin=Origin(xyz=(0.45, 0.0, 0.14)),
        axis=(0.0, 1.0, 0.0),
        motion_limits=MotionLimits(
            effort=8.0,
            velocity=2.0,
            lower=-1.05,
            upper=0.0,
        ),
    )

    return model


def run_tests() -> TestReport:
    ctx = TestContext(object_model)
    seat_base = object_model.get_part("seat_base")
    backrest = object_model.get_part("backrest")
    bracket = object_model.get_part("footrest_bracket")
    footrest = object_model.get_part("footrest_panel")
    backrest_joint = object_model.get_articulation("seat_to_backrest")
    bracket_joint = object_model.get_articulation("seat_to_footrest_bracket")
    footrest_joint = object_model.get_articulation("bracket_to_footrest")

    ctx.check_model_valid()
    ctx.check_mesh_assets_ready()
    ctx.allow_overlap(
        bracket,
        footrest,
        elem_a="hinge_spindle",
        elem_b="hinge_cartridge",
        reason="The footrest pivots on a spindle captured inside its central clip cartridge.",
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

    ctx.check(
        "backrest_pivot_axis_is_horizontal",
        tuple(round(value, 6) for value in backrest_joint.axis) == (0.0, 1.0, 0.0),
        f"unexpected backrest axis: {backrest_joint.axis}",
    )
    ctx.check(
        "footrest_pivot_axis_is_horizontal",
        tuple(round(value, 6) for value in footrest_joint.axis) == (0.0, 1.0, 0.0),
        f"unexpected footrest axis: {footrest_joint.axis}",
    )
    ctx.check(
        "footrest_bracket_is_fixed_to_seat",
        bracket_joint.parent == seat_base.name and bracket_joint.child == bracket.name,
        f"unexpected bracket articulation: parent={bracket_joint.parent}, child={bracket_joint.child}",
    )

    ctx.expect_contact(bracket, seat_base, name="bracket_is_mounted_to_seat")
    ctx.expect_contact(
        backrest,
        seat_base,
        elem_a="left_pivot_cap",
        elem_b="left_pivot_support",
        name="left_backrest_pivot_is_seated",
    )
    ctx.expect_contact(
        backrest,
        seat_base,
        elem_a="right_pivot_cap",
        elem_b="right_pivot_support",
        name="right_backrest_pivot_is_seated",
    )
    ctx.expect_gap(
        backrest,
        seat_base,
        axis="z",
        positive_elem="backrest_cushion",
        negative_elem="seat_cushion",
        min_gap=0.008,
        max_gap=0.06,
        name="floating_backrest_gap_above_seat",
    )
    ctx.expect_gap(
        seat_base,
        footrest,
        axis="z",
        positive_elem="seat_cushion",
        negative_elem="footrest_panel",
        min_gap=0.0,
        max_gap=0.08,
        name="stowed_footrest_sits_below_seat_front",
    )
    ctx.expect_within(
        footrest,
        seat_base,
        axes="y",
        margin=0.04,
        name="footrest_stays_within_seat_width",
    )
    ctx.expect_contact(
        footrest,
        bracket,
        contact_tol=0.005,
        name="footrest_is_clipped_to_bracket_at_rest",
    )

    with ctx.pose({footrest_joint: -1.0}):
        ctx.expect_contact(
            footrest,
            bracket,
            contact_tol=0.005,
            name="footrest_stays_clipped_when_deployed",
        )
        footrest_panel_aabb = ctx.part_element_world_aabb(footrest, elem="footrest_panel")
        seat_cushion_aabb = ctx.part_element_world_aabb(seat_base, elem="seat_cushion")
        if footrest_panel_aabb is None or seat_cushion_aabb is None:
            ctx.fail(
                "deployed_footrest_projects_forward",
                "missing seat or footrest element bounds in deployed pose",
            )
        else:
            footrest_front = footrest_panel_aabb[1][0]
            seat_front = seat_cushion_aabb[1][0]
            ctx.check(
                "deployed_footrest_projects_forward",
                footrest_front > seat_front + 0.09,
                f"footrest front edge {footrest_front:.3f} did not project beyond seat front {seat_front:.3f}",
            )

    return ctx.report()


# >>> USER_CODE_END

object_model = build_object_model()
