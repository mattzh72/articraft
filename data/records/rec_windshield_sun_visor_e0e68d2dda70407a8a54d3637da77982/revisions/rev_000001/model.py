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
    LoftGeometry,
    MotionLimits,
    Origin,
    TestContext,
    TestReport,
    mesh_from_geometry,
    rounded_rect_profile,
    tube_from_spline_points,
)


def _panel_shell_mesh():
    lower = [
        (x, y, -0.005)
        for x, y in rounded_rect_profile(0.196, 0.100, 0.018, corner_segments=10)
    ]
    upper = [
        (x, y, 0.005)
        for x, y in rounded_rect_profile(0.192, 0.096, 0.016, corner_segments=10)
    ]
    return mesh_from_geometry(
        LoftGeometry([lower, upper], cap=True, closed=True),
        "visor_panel_shell",
    )


def _arm_spine_mesh():
    return mesh_from_geometry(
        tube_from_spline_points(
            [
                (0.0, 0.006, -0.001),
                (0.0, 0.030, -0.002),
                (0.0, 0.062, -0.004),
                (0.0, 0.086, 0.000),
            ],
            radius=0.0065,
            samples_per_segment=14,
            radial_segments=18,
            cap_ends=True,
        ),
        "visor_hinge_arm_spine",
    )


def build_object_model() -> ArticulatedObject:
    model = ArticulatedObject(name="desktop_windshield_sun_visor")

    stand_dark = model.material("stand_dark", rgba=(0.14, 0.15, 0.17, 1.0))
    stand_mid = model.material("stand_mid", rgba=(0.22, 0.24, 0.27, 1.0))
    arm_dark = model.material("arm_dark", rgba=(0.11, 0.12, 0.13, 1.0))
    hinge_metal = model.material("hinge_metal", rgba=(0.68, 0.70, 0.73, 1.0))
    visor_fabric = model.material("visor_fabric", rgba=(0.73, 0.72, 0.68, 1.0))

    stand_base = model.part("stand_base")
    stand_base.visual(
        Box((0.220, 0.110, 0.014)),
        origin=Origin(xyz=(0.000, 0.000, 0.007)),
        material=stand_dark,
        name="foot_base",
    )
    stand_base.visual(
        Box((0.100, 0.064, 0.040)),
        origin=Origin(xyz=(-0.010, -0.004, 0.027)),
        material=stand_mid,
        name="rear_ballast",
    )
    stand_base.visual(
        Box((0.012, 0.036, 0.194)),
        origin=Origin(xyz=(-0.101, 0.012, 0.111)),
        material=stand_mid,
        name="mast",
    )
    stand_base.visual(
        Box((0.214, 0.100, 0.010)),
        origin=Origin(xyz=(0.010, 0.000, 0.211)),
        material=stand_mid,
        name="roof_pad",
    )
    stand_base.visual(
        Box((0.010, 0.020, 0.034)),
        origin=Origin(xyz=(-0.093, -0.008, 0.191)),
        material=stand_dark,
        name="hinge_bracket_left",
    )
    stand_base.visual(
        Box((0.010, 0.020, 0.034)),
        origin=Origin(xyz=(-0.063, -0.008, 0.191)),
        material=stand_dark,
        name="hinge_bracket_right",
    )
    stand_base.inertial = Inertial.from_geometry(
        Box((0.224, 0.116, 0.216)),
        mass=1.9,
        origin=Origin(xyz=(0.000, 0.020, 0.108)),
    )

    hinge_arm = model.part("hinge_arm")
    hinge_arm.visual(
        Cylinder(radius=0.008, length=0.020),
        origin=Origin(rpy=(0.0, math.pi / 2.0, 0.0)),
        material=hinge_metal,
        name="hinge_barrel",
    )
    hinge_arm.visual(
        Box((0.012, 0.014, 0.014)),
        origin=Origin(xyz=(-0.004, 0.008, 0.000)),
        material=arm_dark,
        name="hinge_block",
    )
    hinge_arm.visual(
        Box((0.006, 0.072, 0.014)),
        origin=Origin(xyz=(-0.009, 0.034, -0.004)),
        material=arm_dark,
        name="arm_spine",
    )
    hinge_arm.visual(
        Box((0.024, 0.010, 0.006)),
        origin=Origin(xyz=(0.000, 0.072, -0.010)),
        material=hinge_metal,
        name="fork_backer",
    )
    hinge_arm.visual(
        Box((0.006, 0.022, 0.020)),
        origin=Origin(xyz=(-0.0095, 0.078, 0.000)),
        material=hinge_metal,
        name="fork_plate_left",
    )
    hinge_arm.visual(
        Box((0.006, 0.022, 0.020)),
        origin=Origin(xyz=(0.0095, 0.078, 0.000)),
        material=hinge_metal,
        name="fork_plate_right",
    )
    hinge_arm.inertial = Inertial.from_geometry(
        Box((0.036, 0.118, 0.040)),
        mass=0.22,
        origin=Origin(xyz=(0.000, 0.058, 0.006)),
    )

    visor_panel = model.part("visor_panel")
    visor_panel.visual(
        Cylinder(radius=0.0065, length=0.008),
        material=hinge_metal,
        name="pivot_hub",
    )
    visor_panel.visual(
        Box((0.004, 0.024, 0.010)),
        origin=Origin(xyz=(0.000, -0.018, 0.005)),
        material=arm_dark,
        name="hub_rib",
    )
    visor_panel.visual(
        Box((0.030, 0.008, 0.008)),
        origin=Origin(xyz=(0.015, -0.028, 0.006)),
        material=arm_dark,
        name="panel_root_tab",
    )
    visor_panel.visual(
        _panel_shell_mesh(),
        origin=Origin(xyz=(0.118, -0.020, 0.000)),
        material=visor_fabric,
        name="panel_shell",
    )
    visor_panel.inertial = Inertial.from_geometry(
        Box((0.206, 0.112, 0.016)),
        mass=0.34,
        origin=Origin(xyz=(0.118, -0.018, 0.000)),
    )

    model.articulation(
        "roof_hinge",
        ArticulationType.REVOLUTE,
        parent=stand_base,
        child=hinge_arm,
        origin=Origin(xyz=(-0.078, -0.008, 0.199)),
        axis=(-1.0, 0.0, 0.0),
        motion_limits=MotionLimits(
            effort=6.0,
            velocity=1.6,
            lower=0.0,
            upper=1.18,
        ),
    )
    model.articulation(
        "swing_pivot",
        ArticulationType.REVOLUTE,
        parent=hinge_arm,
        child=visor_panel,
        origin=Origin(xyz=(0.000, 0.082, 0.000)),
        axis=(0.0, 1.0, 0.0),
        motion_limits=MotionLimits(
            effort=4.0,
            velocity=1.8,
            lower=0.0,
            upper=1.40,
        ),
    )

    return model


def run_tests() -> TestReport:
    ctx = TestContext(object_model)
    stand_base = object_model.get_part("stand_base")
    hinge_arm = object_model.get_part("hinge_arm")
    visor_panel = object_model.get_part("visor_panel")
    roof_hinge = object_model.get_articulation("roof_hinge")
    swing_pivot = object_model.get_articulation("swing_pivot")

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

    ctx.expect_contact(
        hinge_arm,
        stand_base,
        elem_a="hinge_barrel",
        elem_b="hinge_bracket_left",
        name="hinge_barrel_contacts_left_bracket",
    )
    ctx.expect_contact(
        hinge_arm,
        stand_base,
        elem_a="hinge_barrel",
        elem_b="hinge_bracket_right",
        name="hinge_barrel_contacts_right_bracket",
    )
    ctx.expect_contact(
        visor_panel,
        hinge_arm,
        elem_a="pivot_hub",
        elem_b="fork_plate_left",
        name="pivot_hub_contacts_left_fork_plate",
    )
    ctx.expect_contact(
        visor_panel,
        hinge_arm,
        elem_a="pivot_hub",
        elem_b="fork_plate_right",
        name="pivot_hub_contacts_right_fork_plate",
    )

    with ctx.pose({roof_hinge: 0.0, swing_pivot: 0.0}):
        ctx.expect_gap(
            stand_base,
            visor_panel,
            axis="z",
            positive_elem="roof_pad",
            negative_elem="panel_shell",
            min_gap=0.001,
            max_gap=0.005,
            name="stowed_panel_tucks_close_under_roof",
        )
        ctx.expect_overlap(
            visor_panel,
            stand_base,
            axes="xy",
            elem_a="panel_shell",
            elem_b="roof_pad",
            min_overlap=0.040,
            name="stowed_panel_stays_under_roof_footprint",
        )

    with ctx.pose({roof_hinge: 1.18, swing_pivot: 0.0}):
        ctx.expect_gap(
            visor_panel,
            stand_base,
            axis="z",
            positive_elem="panel_shell",
            negative_elem="foot_base",
            min_gap=0.002,
            name="deployed_panel_clears_base_plate",
        )
        deployed_forward = ctx.part_element_world_aabb(visor_panel, elem="panel_shell")

    with ctx.pose({roof_hinge: 0.0, swing_pivot: 0.0}):
        stowed = ctx.part_element_world_aabb(visor_panel, elem="panel_shell")

    if stowed is not None and deployed_forward is not None:
        stowed_center_z = 0.5 * (stowed[0][2] + stowed[1][2])
        deployed_center_z = 0.5 * (deployed_forward[0][2] + deployed_forward[1][2])
        ctx.check(
            "roof_hinge_lowers_panel_from_stowed_pose",
            deployed_center_z < stowed_center_z - 0.050,
            details=(
                f"expected deployed panel center z to drop by at least 0.050 m; "
                f"stowed={stowed_center_z:.4f}, deployed={deployed_center_z:.4f}"
            ),
        )
    else:
        ctx.fail("roof_hinge_lowers_panel_from_stowed_pose", "panel_shell AABB unavailable")

    with ctx.pose({roof_hinge: 1.18, swing_pivot: 1.10}):
        ctx.expect_gap(
            visor_panel,
            stand_base,
            axis="z",
            positive_elem="panel_shell",
            negative_elem="foot_base",
            min_gap=0.002,
            name="side_swung_panel_still_clears_base_plate",
        )
        side_swung = ctx.part_element_world_aabb(visor_panel, elem="panel_shell")

    if deployed_forward is not None and side_swung is not None:
        forward_center = (
            0.5 * (deployed_forward[0][0] + deployed_forward[1][0]),
            0.5 * (deployed_forward[0][1] + deployed_forward[1][1]),
        )
        side_center = (
            0.5 * (side_swung[0][0] + side_swung[1][0]),
            0.5 * (side_swung[0][1] + side_swung[1][1]),
        )
        xy_shift = math.hypot(side_center[0] - forward_center[0], side_center[1] - forward_center[1])
        forward_dx = deployed_forward[1][0] - deployed_forward[0][0]
        side_dx = side_swung[1][0] - side_swung[0][0]
        ctx.check(
            "secondary_pivot_swings_panel_sideways",
            xy_shift > 0.040 and side_dx < forward_dx * 0.90,
            details=(
                f"expected clear side swing; xy_shift={xy_shift:.4f}, "
                f"forward_dx={forward_dx:.4f}, side_dx={side_dx:.4f}"
            ),
        )
    else:
        ctx.fail("secondary_pivot_swings_panel_sideways", "panel_shell AABB unavailable in deployed poses")

    return ctx.report()


# >>> USER_CODE_END

object_model = build_object_model()
