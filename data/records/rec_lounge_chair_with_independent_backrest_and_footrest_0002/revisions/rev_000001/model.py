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
    MotionLimits,
    Origin,
    TestContext,
    TestReport,
    mesh_from_geometry,
    repair_loft,
    rounded_rect_profile,
    section_loft,
)

ASSETS = AssetContext.from_script(__file__)


def build_object_model() -> ArticulatedObject:
    model = ArticulatedObject(name="reclining_lounge_chair", assets=ASSETS)

    upholstery = model.material("upholstery", rgba=(0.56, 0.55, 0.53, 1.0))
    frame_metal = model.material("frame_metal", rgba=(0.16, 0.17, 0.19, 1.0))
    hinge_metal = model.material("hinge_metal", rgba=(0.58, 0.60, 0.62, 1.0))

    seat_length = 0.62
    seat_width = 0.68
    seat_thickness = 0.10
    seat_center_z = 0.24

    back_panel_width = 0.628
    back_panel_height = 0.72
    back_panel_thickness = 0.09

    foot_panel_length = 0.34
    foot_panel_width = 0.628
    foot_panel_thickness = 0.07

    pivot_radius = 0.014
    pivot_length = 0.026
    pivot_y = 0.327

    def add_box(part, *, name, size, xyz, material):
        part.visual(
            Box(size),
            origin=Origin(xyz=xyz),
            material=material,
            name=name,
        )

    def add_y_cylinder(part, *, name, radius, length, xyz, material):
        part.visual(
            Cylinder(radius=radius, length=length),
            origin=Origin(xyz=xyz, rpy=(math.pi / 2.0, 0.0, 0.0)),
            material=material,
            name=name,
        )

    def soft_panel_mesh(name: str, *, sx: float, sy: float, sz: float, corner_radius: float):
        def loop(width: float, height: float, z: float):
            profile = rounded_rect_profile(
                width,
                height,
                radius=min(corner_radius, width * 0.48, height * 0.48),
                corner_segments=8,
            )
            return [(x, y, z) for x, y in profile]

        sections = [
            loop(sx * 0.97, sy * 0.97, -sz * 0.50),
            loop(sx * 1.00, sy * 1.00, -sz * 0.34),
            loop(sx * 1.00, sy * 1.00, 0.00),
            loop(sx * 0.985, sy * 0.985, sz * 0.28),
            loop(sx * 0.93, sy * 0.93, sz * 0.50),
        ]
        return mesh_from_geometry(
            repair_loft(section_loft(sections)),
            ASSETS.mesh_path(f"{name}.obj"),
        )

    seat_base = model.part("seat_base")
    seat_base.visual(
        soft_panel_mesh(
            "seat_pan",
            sx=seat_length,
            sy=seat_width,
            sz=seat_thickness,
            corner_radius=0.10,
        ),
        origin=Origin(xyz=(0.0, 0.0, seat_center_z)),
        material=upholstery,
        name="seat_pan",
    )
    add_box(
        seat_base,
        name="left_side_rail",
        size=(0.54, 0.06, 0.04),
        xyz=(0.0, 0.29, 0.16),
        material=frame_metal,
    )
    add_box(
        seat_base,
        name="right_side_rail",
        size=(0.54, 0.06, 0.04),
        xyz=(0.0, -0.29, 0.16),
        material=frame_metal,
    )
    add_box(
        seat_base,
        name="front_cross_rail",
        size=(0.06, 0.52, 0.04),
        xyz=(0.24, 0.0, 0.16),
        material=frame_metal,
    )
    add_box(
        seat_base,
        name="rear_cross_rail",
        size=(0.06, 0.52, 0.04),
        xyz=(-0.24, 0.0, 0.16),
        material=frame_metal,
    )
    add_box(
        seat_base,
        name="left_support_beam",
        size=(0.50, 0.18, 0.02),
        xyz=(0.0, 0.17, 0.18),
        material=frame_metal,
    )
    add_box(
        seat_base,
        name="right_support_beam",
        size=(0.50, 0.18, 0.02),
        xyz=(0.0, -0.17, 0.18),
        material=frame_metal,
    )
    for name, x, y in (
        ("front_left_foot", 0.20, 0.29),
        ("front_right_foot", 0.20, -0.29),
        ("rear_left_foot", -0.20, 0.29),
        ("rear_right_foot", -0.20, -0.29),
    ):
        add_box(
            seat_base,
            name=name,
            size=(0.10, 0.06, 0.16),
            xyz=(x, y, 0.08),
            material=frame_metal,
        )
    add_box(
        seat_base,
        name="left_back_bracket",
        size=(0.06, 0.014, 0.10),
        xyz=(-0.31, 0.347, 0.305),
        material=hinge_metal,
    )
    add_box(
        seat_base,
        name="right_back_bracket",
        size=(0.06, 0.014, 0.10),
        xyz=(-0.31, -0.347, 0.305),
        material=hinge_metal,
    )
    add_box(
        seat_base,
        name="left_foot_bracket",
        size=(0.05, 0.01, 0.05),
        xyz=(0.335, 0.345, 0.165),
        material=hinge_metal,
    )
    add_box(
        seat_base,
        name="right_foot_bracket",
        size=(0.05, 0.01, 0.05),
        xyz=(0.335, -0.345, 0.165),
        material=hinge_metal,
    )
    add_box(
        seat_base,
        name="left_back_brace",
        size=(0.02, 0.04, 0.085),
        xyz=(-0.28, 0.33, 0.2175),
        material=frame_metal,
    )
    add_box(
        seat_base,
        name="right_back_brace",
        size=(0.02, 0.04, 0.085),
        xyz=(-0.28, -0.33, 0.2175),
        material=frame_metal,
    )
    add_box(
        seat_base,
        name="left_foot_brace",
        size=(0.04, 0.04, 0.02),
        xyz=(0.29, 0.33, 0.145),
        material=frame_metal,
    )
    add_box(
        seat_base,
        name="right_foot_brace",
        size=(0.04, 0.04, 0.02),
        xyz=(0.29, -0.33, 0.145),
        material=frame_metal,
    )
    seat_base.inertial = Inertial.from_geometry(
        Box((seat_length, seat_width, seat_thickness)),
        mass=14.0,
        origin=Origin(xyz=(0.0, 0.0, seat_center_z)),
    )

    backrest = model.part("backrest")
    backrest.visual(
        soft_panel_mesh(
            "back_panel",
            sx=back_panel_height,
            sy=back_panel_width,
            sz=back_panel_thickness,
            corner_radius=0.08,
        ),
        origin=Origin(xyz=(-0.055, 0.0, 0.35), rpy=(0.0, -math.pi / 2.0, 0.0)),
        material=upholstery,
        name="back_panel",
    )
    add_box(
        backrest,
        name="left_back_hinge_arm",
        size=(0.045, 0.04, 0.05),
        xyz=(-0.0325, 0.312, 0.015),
        material=hinge_metal,
    )
    add_box(
        backrest,
        name="right_back_hinge_arm",
        size=(0.045, 0.04, 0.05),
        xyz=(-0.0325, -0.312, 0.015),
        material=hinge_metal,
    )
    add_y_cylinder(
        backrest,
        name="left_back_pivot",
        radius=pivot_radius,
        length=pivot_length,
        xyz=(0.0, pivot_y, 0.0),
        material=hinge_metal,
    )
    add_y_cylinder(
        backrest,
        name="right_back_pivot",
        radius=pivot_radius,
        length=pivot_length,
        xyz=(0.0, -pivot_y, 0.0),
        material=hinge_metal,
    )
    backrest.inertial = Inertial.from_geometry(
        Box((back_panel_thickness, back_panel_width, back_panel_height)),
        mass=6.5,
        origin=Origin(xyz=(-0.055, 0.0, 0.35)),
    )

    footrest = model.part("footrest")
    footrest.visual(
        soft_panel_mesh(
            "foot_panel",
            sx=foot_panel_length,
            sy=foot_panel_width,
            sz=foot_panel_thickness,
            corner_radius=0.07,
        ),
        origin=Origin(xyz=(foot_panel_length / 2.0, 0.0, -foot_panel_thickness / 2.0)),
        material=upholstery,
        name="foot_panel",
    )
    add_box(
        footrest,
        name="left_foot_hinge_arm",
        size=(0.035, 0.04, 0.035),
        xyz=(0.0175, 0.312, -0.0175),
        material=hinge_metal,
    )
    add_box(
        footrest,
        name="right_foot_hinge_arm",
        size=(0.035, 0.04, 0.035),
        xyz=(0.0175, -0.312, -0.0175),
        material=hinge_metal,
    )
    add_y_cylinder(
        footrest,
        name="left_foot_pivot",
        radius=pivot_radius,
        length=pivot_length,
        xyz=(0.0, pivot_y, 0.0),
        material=hinge_metal,
    )
    add_y_cylinder(
        footrest,
        name="right_foot_pivot",
        radius=pivot_radius,
        length=pivot_length,
        xyz=(0.0, -pivot_y, 0.0),
        material=hinge_metal,
    )
    footrest.inertial = Inertial.from_geometry(
        Box((foot_panel_length, foot_panel_width, foot_panel_thickness)),
        mass=2.8,
        origin=Origin(xyz=(foot_panel_length / 2.0, 0.0, -foot_panel_thickness / 2.0)),
    )

    model.articulation(
        "seat_to_backrest",
        ArticulationType.REVOLUTE,
        parent=seat_base,
        child=backrest,
        origin=Origin(xyz=(-0.31, 0.0, 0.305), rpy=(0.0, math.radians(-5.0), 0.0)),
        axis=(0.0, -1.0, 0.0),
        motion_limits=MotionLimits(
            effort=24.0,
            velocity=1.4,
            lower=0.0,
            upper=math.radians(35.0),
        ),
    )
    model.articulation(
        "seat_to_footrest",
        ArticulationType.REVOLUTE,
        parent=seat_base,
        child=footrest,
        origin=Origin(xyz=(0.324, 0.0, 0.176)),
        axis=(0.0, -1.0, 0.0),
        motion_limits=MotionLimits(
            effort=18.0,
            velocity=1.8,
            lower=0.0,
            upper=math.radians(80.0),
        ),
    )

    return model


def run_tests() -> TestReport:
    ctx = TestContext(object_model, asset_root=ASSETS.asset_root)
    seat_base = object_model.get_part("seat_base")
    backrest = object_model.get_part("backrest")
    footrest = object_model.get_part("footrest")
    back_hinge = object_model.get_articulation("seat_to_backrest")
    foot_hinge = object_model.get_articulation("seat_to_footrest")

    seat_pan = seat_base.get_visual("seat_pan")
    left_support_beam = seat_base.get_visual("left_support_beam")
    right_support_beam = seat_base.get_visual("right_support_beam")
    left_back_bracket = seat_base.get_visual("left_back_bracket")
    right_back_bracket = seat_base.get_visual("right_back_bracket")
    left_foot_bracket = seat_base.get_visual("left_foot_bracket")
    right_foot_bracket = seat_base.get_visual("right_foot_bracket")
    back_panel = backrest.get_visual("back_panel")
    left_back_pivot = backrest.get_visual("left_back_pivot")
    right_back_pivot = backrest.get_visual("right_back_pivot")
    foot_panel = footrest.get_visual("foot_panel")
    left_foot_pivot = footrest.get_visual("left_foot_pivot")
    right_foot_pivot = footrest.get_visual("right_foot_pivot")

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
    ctx.fail_if_articulation_overlaps(max_pose_samples=24, name="articulation_clearance_sweep")

    ctx.expect_contact(
        seat_base,
        seat_base,
        elem_a=seat_pan,
        elem_b=left_support_beam,
        name="seat_pan_contacts_left_support_beam",
    )
    ctx.expect_contact(
        seat_base,
        seat_base,
        elem_a=seat_pan,
        elem_b=right_support_beam,
        name="seat_pan_contacts_right_support_beam",
    )
    ctx.expect_overlap(
        seat_base,
        seat_base,
        axes="xy",
        elem_a=seat_pan,
        elem_b=left_support_beam,
        min_overlap=0.18,
        name="seat_pan_over_left_support_beam",
    )
    ctx.expect_overlap(
        seat_base,
        seat_base,
        axes="xy",
        elem_a=seat_pan,
        elem_b=right_support_beam,
        min_overlap=0.18,
        name="seat_pan_over_right_support_beam",
    )

    ctx.expect_gap(
        seat_base,
        backrest,
        axis="y",
        max_gap=0.002,
        max_penetration=1e-5,
        positive_elem=left_back_bracket,
        negative_elem=left_back_pivot,
        name="left_back_pivot_seats_against_base_bracket",
    )
    ctx.expect_gap(
        backrest,
        seat_base,
        axis="y",
        max_gap=0.002,
        max_penetration=1e-5,
        positive_elem=right_back_pivot,
        negative_elem=right_back_bracket,
        name="right_back_pivot_seats_against_base_bracket",
    )
    ctx.expect_overlap(
        backrest,
        seat_base,
        axes="xz",
        elem_a=left_back_pivot,
        elem_b=left_back_bracket,
        min_overlap=0.02,
        name="left_back_pivot_supported_in_xz",
    )
    ctx.expect_overlap(
        backrest,
        seat_base,
        axes="xz",
        elem_a=right_back_pivot,
        elem_b=right_back_bracket,
        min_overlap=0.02,
        name="right_back_pivot_supported_in_xz",
    )

    ctx.expect_gap(
        seat_base,
        footrest,
        axis="y",
        max_gap=0.012,
        max_penetration=1e-5,
        positive_elem=left_foot_bracket,
        negative_elem=left_foot_pivot,
        name="left_foot_pivot_seats_against_base_bracket",
    )
    ctx.expect_gap(
        footrest,
        seat_base,
        axis="y",
        max_gap=0.012,
        max_penetration=1e-5,
        positive_elem=right_foot_pivot,
        negative_elem=right_foot_bracket,
        name="right_foot_pivot_seats_against_base_bracket",
    )
    ctx.expect_overlap(
        footrest,
        seat_base,
        axes="xz",
        elem_a=left_foot_pivot,
        elem_b=left_foot_bracket,
        min_overlap=0.02,
        name="left_foot_pivot_supported_in_xz",
    )
    ctx.expect_overlap(
        footrest,
        seat_base,
        axes="xz",
        elem_a=right_foot_pivot,
        elem_b=right_foot_bracket,
        min_overlap=0.02,
        name="right_foot_pivot_supported_in_xz",
    )

    back_limits = back_hinge.motion_limits
    foot_limits = foot_hinge.motion_limits
    ctx.check(
        "backrest_motion_limit_range",
        abs(back_limits.lower - 0.0) < 1e-9
        and abs(back_limits.upper - math.radians(35.0)) < 1e-9,
        details=(
            f"Expected 0 to 35 deg; got "
            f"{math.degrees(back_limits.lower):.1f} to {math.degrees(back_limits.upper):.1f}"
        ),
    )
    ctx.check(
        "footrest_motion_limit_range",
        abs(foot_limits.lower - 0.0) < 1e-9
        and abs(foot_limits.upper - math.radians(80.0)) < 1e-9,
        details=(
            f"Expected 0 to 80 deg; got "
            f"{math.degrees(foot_limits.lower):.1f} to {math.degrees(foot_limits.upper):.1f}"
        ),
    )

    def elem_center(part, elem):
        aabb = ctx.part_element_world_aabb(part, elem=elem)
        if aabb is None:
            return None
        lo, hi = aabb
        return tuple((lo[i] + hi[i]) * 0.5 for i in range(3))

    with ctx.pose({back_hinge: 0.0}):
        back_rest_center = elem_center(backrest, back_panel)
        back_rest_aabb = ctx.part_element_world_aabb(backrest, elem=back_panel)
    with ctx.pose({back_hinge: math.radians(30.0)}):
        back_reclined_center = elem_center(backrest, back_panel)
        back_reclined_aabb = ctx.part_element_world_aabb(backrest, elem=back_panel)

    ctx.check(
        "backrest_moves_rearward_when_reclined",
        back_rest_center is not None
        and back_reclined_center is not None
        and back_reclined_center[0] < back_rest_center[0] - 0.12,
        details=(
            f"Expected backrest center x to move rearward by > 0.12 m; "
            f"got {back_rest_center} to {back_reclined_center}"
        ),
    )
    ctx.check(
        "backrest_top_lowers_when_reclined",
        back_rest_aabb is not None
        and back_reclined_aabb is not None
        and back_reclined_aabb[1][2] < back_rest_aabb[1][2] - 0.08,
        details=(
            f"Expected backrest top z to lower by > 0.08 m; "
            f"got {back_rest_aabb} to {back_reclined_aabb}"
        ),
    )

    with ctx.pose({foot_hinge: 0.0}):
        foot_stowed_center = elem_center(footrest, foot_panel)
    with ctx.pose({foot_hinge: math.radians(80.0)}):
        foot_raised_center = elem_center(footrest, foot_panel)

    ctx.check(
        "footrest_swings_upward",
        foot_stowed_center is not None
        and foot_raised_center is not None
        and foot_raised_center[2] > foot_stowed_center[2] + 0.15,
        details=(
            f"Expected footrest center z to rise by > 0.15 m; "
            f"got {foot_stowed_center} to {foot_raised_center}"
        ),
    )
    ctx.check(
        "footrest_stays_in_front_zone_when_raised",
        foot_stowed_center is not None
        and foot_raised_center is not None
        and foot_stowed_center[0] > 0.45
        and foot_raised_center[0] > 0.35,
        details=(
            f"Expected footrest center to remain forward of the seat front zone; "
            f"got {foot_stowed_center} to {foot_raised_center}"
        ),
    )

    return ctx.report()


# >>> USER_CODE_END

object_model = build_object_model()
