from __future__ import annotations

# User code should import every SDK/stdlib symbol it uses instead of relying on
# hidden scaffold imports.
# >>> USER_CODE_START
from math import pi

from sdk import (
    ArticulatedObject,
    ArticulationType,
    AssetContext,
    Box,
    ExtrudeGeometry,
    Inertial,
    MotionLimits,
    Origin,
    TestContext,
    TestReport,
    mesh_from_geometry,
    rounded_rect_profile,
)

ASSETS = AssetContext.from_script(__file__)
BACKREST_RECLINE = 35.0 * pi / 180.0
FOOTREST_STOWED = -70.0 * pi / 180.0


def _save_mesh(name: str, geometry):
    return mesh_from_geometry(geometry, ASSETS.mesh_path(name))


def _seat_cushion_mesh(name: str, *, width: float, depth: float, thickness: float):
    radius = min(width, depth, thickness * 3.0) * 0.12
    geometry = ExtrudeGeometry.centered(
        rounded_rect_profile(width, depth, radius, corner_segments=8),
        thickness,
    )
    return _save_mesh(name, geometry)


def _upright_panel_mesh(name: str, *, width: float, height: float, thickness: float):
    radius = min(width, height, thickness * 4.0) * 0.12
    geometry = ExtrudeGeometry.centered(
        rounded_rect_profile(width, height, radius, corner_segments=8),
        thickness,
    ).rotate_x(pi / 2.0)
    return _save_mesh(name, geometry)


def _arm_mesh(name: str, *, thickness: float, length: float, height: float):
    radius = min(thickness, height) * 0.28
    geometry = ExtrudeGeometry.centered(
        rounded_rect_profile(thickness, height, radius, corner_segments=8),
        length,
    ).rotate_x(pi / 2.0)
    return _save_mesh(name, geometry)


def build_object_model() -> ArticulatedObject:
    model = ArticulatedObject(name="chaise_recliner", assets=ASSETS)

    upholstery = model.material("upholstery", rgba=(0.56, 0.58, 0.61, 1.0))
    accent_upholstery = model.material("accent_upholstery", rgba=(0.50, 0.52, 0.55, 1.0))
    frame_finish = model.material("frame_finish", rgba=(0.17, 0.16, 0.15, 1.0))
    foot_cap = model.material("foot_cap", rgba=(0.12, 0.12, 0.12, 1.0))

    seat_mesh = _seat_cushion_mesh(
        "recliner_seat_pad.obj",
        width=0.60,
        depth=0.66,
        thickness=0.12,
    )
    back_panel_mesh = _upright_panel_mesh(
        "recliner_back_panel.obj",
        width=0.60,
        height=0.66,
        thickness=0.10,
    )
    foot_panel_mesh = _seat_cushion_mesh(
        "recliner_foot_panel.obj",
        width=0.60,
        depth=0.52,
        thickness=0.09,
    )
    arm_mesh = _arm_mesh(
        "recliner_side_arm.obj",
        thickness=0.13,
        length=0.90,
        height=0.48,
    )

    base_frame = model.part("base_frame")
    base_frame.inertial = Inertial.from_geometry(
        Box((0.86, 0.96, 0.58)),
        mass=34.0,
        origin=Origin(xyz=(0.0, -0.02, 0.29)),
    )
    base_frame.visual(
        Box((0.74, 0.70, 0.09)),
        origin=Origin(xyz=(0.0, -0.15, 0.045)),
        material=frame_finish,
        name="plinth",
    )
    base_frame.visual(
        seat_mesh,
        origin=Origin(xyz=(0.0, -0.01, 0.15)),
        material=upholstery,
        name="seat_pad",
    )
    base_frame.visual(
        arm_mesh,
        origin=Origin(xyz=(0.365, -0.02, 0.33)),
        material=upholstery,
        name="left_arm",
    )
    base_frame.visual(
        arm_mesh,
        origin=Origin(xyz=(-0.365, -0.02, 0.33)),
        material=upholstery,
        name="right_arm",
    )
    base_frame.visual(
        Box((0.56, 0.06, 0.12)),
        origin=Origin(xyz=(0.0, -0.31, 0.15)),
        material=frame_finish,
        name="rear_bridge",
    )
    base_frame.visual(
        Box((0.56, 0.06, 0.10)),
        origin=Origin(xyz=(0.0, 0.29, 0.14)),
        material=frame_finish,
        name="front_bridge",
    )
    base_frame.visual(
        Box((0.10, 0.14, 0.03)),
        origin=Origin(xyz=(0.365, -0.36, 0.105)),
        material=foot_cap,
        name="left_rear_foot",
    )
    base_frame.visual(
        Box((0.10, 0.14, 0.03)),
        origin=Origin(xyz=(-0.365, -0.36, 0.105)),
        material=foot_cap,
        name="right_rear_foot",
    )
    base_frame.visual(
        Box((0.10, 0.14, 0.03)),
        origin=Origin(xyz=(0.365, 0.32, 0.105)),
        material=foot_cap,
        name="left_front_foot",
    )
    base_frame.visual(
        Box((0.10, 0.14, 0.03)),
        origin=Origin(xyz=(-0.365, 0.32, 0.105)),
        material=foot_cap,
        name="right_front_foot",
    )

    backrest = model.part("backrest")
    backrest.inertial = Inertial.from_geometry(
        Box((0.60, 0.10, 0.66)),
        mass=8.5,
        origin=Origin(xyz=(0.0, 0.05, 0.33)),
    )
    backrest.visual(
        back_panel_mesh,
        origin=Origin(xyz=(0.0, 0.05, 0.33)),
        material=upholstery,
        name="back_panel",
    )
    backrest.visual(
        Box((0.54, 0.05, 0.04)),
        origin=Origin(xyz=(0.0, 0.025, 0.02)),
        material=accent_upholstery,
        name="back_hinge_rail",
    )

    footrest = model.part("footrest")
    footrest.inertial = Inertial.from_geometry(
        Box((0.60, 0.52, 0.09)),
        mass=5.5,
        origin=Origin(xyz=(0.0, 0.10, -0.045)),
    )
    footrest.visual(
        foot_panel_mesh,
        origin=Origin(xyz=(0.0, 0.10, -0.045)),
        material=upholstery,
        name="foot_panel",
    )
    footrest.visual(
        Box((0.54, 0.05, 0.04)),
        origin=Origin(xyz=(0.0, -0.03, -0.02)),
        material=accent_upholstery,
        name="foot_hinge_rail",
    )

    model.articulation(
        "backrest_recline",
        ArticulationType.REVOLUTE,
        parent=base_frame,
        child=backrest,
        origin=Origin(xyz=(0.0, -0.34, 0.21)),
        axis=(1.0, 0.0, 0.0),
        motion_limits=MotionLimits(
            effort=30.0,
            velocity=1.2,
            lower=0.0,
            upper=BACKREST_RECLINE,
        ),
    )
    model.articulation(
        "footrest_pivot",
        ArticulationType.REVOLUTE,
        parent=base_frame,
        child=footrest,
        origin=Origin(xyz=(0.0, 0.48, 0.21)),
        axis=(1.0, 0.0, 0.0),
        motion_limits=MotionLimits(
            effort=25.0,
            velocity=1.6,
            lower=FOOTREST_STOWED,
            upper=0.0,
        ),
    )

    return model


def run_tests() -> TestReport:
    ctx = TestContext(object_model, asset_root=ASSETS.asset_root)
    base_frame = object_model.get_part("base_frame")
    backrest = object_model.get_part("backrest")
    footrest = object_model.get_part("footrest")
    backrest_recline = object_model.get_articulation("backrest_recline")
    footrest_pivot = object_model.get_articulation("footrest_pivot")

    seat_pad = base_frame.get_visual("seat_pad")
    back_panel = backrest.get_visual("back_panel")
    foot_panel = footrest.get_visual("foot_panel")
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
        "backrest_axis_is_transverse",
        tuple(backrest_recline.axis) == (1.0, 0.0, 0.0),
        details=f"expected transverse x-axis, got {backrest_recline.axis}",
    )
    ctx.check(
        "footrest_axis_is_transverse",
        tuple(footrest_pivot.axis) == (1.0, 0.0, 0.0),
        details=f"expected transverse x-axis, got {footrest_pivot.axis}",
    )
    ctx.check(
        "backrest_recline_range_is_about_35_deg",
        abs(backrest_recline.motion_limits.lower - 0.0) < 1e-6
        and abs(backrest_recline.motion_limits.upper - BACKREST_RECLINE) < 1e-6,
        details=(
            "backrest range should run from upright to about 35 degrees; "
            f"got {backrest_recline.motion_limits.lower}..{backrest_recline.motion_limits.upper}"
        ),
    )
    ctx.check(
        "footrest_range_is_about_70_deg",
        abs(footrest_pivot.motion_limits.lower - FOOTREST_STOWED) < 1e-6
        and abs(footrest_pivot.motion_limits.upper - 0.0) < 1e-6,
        details=(
            "footrest should sweep about 70 degrees from deployed to stowed; "
            f"got {footrest_pivot.motion_limits.lower}..{footrest_pivot.motion_limits.upper}"
        ),
    )

    ctx.expect_overlap(
        backrest,
        base_frame,
        axes="x",
        min_overlap=0.56,
        elem_a=back_panel,
        elem_b=seat_pad,
        name="backrest_is_centered_between_arms",
    )
    ctx.expect_overlap(
        footrest,
        base_frame,
        axes="x",
        min_overlap=0.56,
        elem_a=foot_panel,
        elem_b=seat_pad,
        name="footrest_matches_seat_width",
    )
    ctx.expect_gap(
        backrest,
        base_frame,
        axis="z",
        min_gap=0.0,
        max_gap=0.005,
        positive_elem=back_panel,
        negative_elem=seat_pad,
        name="backrest_bottom_is_flush_with_seat_top",
    )
    ctx.expect_gap(
        footrest,
        base_frame,
        axis="y",
        max_gap=0.005,
        max_penetration=1e-5,
        positive_elem=foot_panel,
        negative_elem=seat_pad,
        name="footrest_starts_at_seat_front_edge",
    )
    ctx.expect_contact(
        backrest,
        base_frame,
        elem_a=back_panel,
        elem_b=seat_pad,
        name="backrest_contacts_base_at_hinge_edge",
    )
    ctx.expect_contact(
        footrest,
        base_frame,
        elem_a=foot_panel,
        elem_b=seat_pad,
        name="footrest_contacts_base_at_front_edge",
    )

    with ctx.pose({backrest_recline: 0.0, footrest_pivot: 0.0}):
        upright_back_aabb = ctx.part_element_world_aabb(backrest, elem=back_panel)
        deployed_foot_aabb = ctx.part_element_world_aabb(footrest, elem=foot_panel)

    with ctx.pose({backrest_recline: BACKREST_RECLINE, footrest_pivot: 0.0}):
        reclined_back_aabb = ctx.part_element_world_aabb(backrest, elem=back_panel)
        ctx.fail_if_parts_overlap_in_current_pose(name="no_overlaps_with_backrest_reclined")

    with ctx.pose({backrest_recline: 0.0, footrest_pivot: FOOTREST_STOWED}):
        stowed_foot_aabb = ctx.part_element_world_aabb(footrest, elem=foot_panel)
        ctx.fail_if_parts_overlap_in_current_pose(name="no_overlaps_with_footrest_stowed")

    ctx.check(
        "backrest_moves_rearward_when_reclined",
        upright_back_aabb is not None
        and reclined_back_aabb is not None
        and reclined_back_aabb[0][1] < upright_back_aabb[0][1] - 0.14
        and reclined_back_aabb[1][2] < upright_back_aabb[1][2] - 0.02,
        details=(
            f"upright={upright_back_aabb}, reclined={reclined_back_aabb}; "
            "expected the back panel to swing rearward and lower at the top"
        ),
    )
    ctx.check(
        "footrest_stows_downward",
        deployed_foot_aabb is not None
        and stowed_foot_aabb is not None
        and stowed_foot_aabb[0][2] < deployed_foot_aabb[0][2] - 0.20
        and stowed_foot_aabb[1][1] < deployed_foot_aabb[1][1] - 0.12,
        details=(
            f"deployed={deployed_foot_aabb}, stowed={stowed_foot_aabb}; "
            "expected the long footrest panel to swing down and retract forward reach"
        ),
    )

    return ctx.report()


# >>> USER_CODE_END

object_model = build_object_model()
