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
    ExtrudeGeometry,
    Inertial,
    MotionLimits,
    Origin,
    TestContext,
    TestReport,
    mesh_from_geometry,
    rounded_rect_profile,
)


def _panel_mesh():
    return mesh_from_geometry(
        ExtrudeGeometry(
            rounded_rect_profile(0.338, 0.148, 0.020, corner_segments=8),
            0.018,
            center=True,
            cap=True,
            closed=True,
        ),
        "sun_visor_panel_shell",
    )


def build_object_model() -> ArticulatedObject:
    model = ArticulatedObject(name="windshield_sun_visor")

    plastic = model.material("warm_gray_plastic", rgba=(0.77, 0.77, 0.74, 1.0))
    fabric = model.material("visor_fabric", rgba=(0.72, 0.71, 0.67, 1.0))
    steel = model.material("dark_steel", rgba=(0.24, 0.25, 0.27, 1.0))
    fastener = model.material("fastener_gray", rgba=(0.55, 0.56, 0.58, 1.0))

    roof_mount = model.part("roof_mount")
    roof_mount.visual(
        Box((0.118, 0.046, 0.008)),
        origin=Origin(xyz=(0.0, 0.0, 0.020)),
        material=plastic,
        name="base_plate",
    )
    roof_mount.visual(
        Box((0.054, 0.030, 0.016)),
        origin=Origin(xyz=(0.0, 0.0, 0.018)),
        material=plastic,
        name="support_block",
    )
    roof_mount.visual(
        Box((0.040, 0.012, 0.010)),
        origin=Origin(xyz=(0.0, -0.017, 0.014)),
        material=plastic,
        name="rear_clamp_web",
    )
    roof_mount.visual(
        Cylinder(radius=0.010, length=0.010),
        origin=Origin(xyz=(-0.019, 0.0, 0.000), rpy=(0.0, math.pi / 2.0, 0.0)),
        material=plastic,
        name="left_knuckle",
    )
    roof_mount.visual(
        Cylinder(radius=0.010, length=0.010),
        origin=Origin(xyz=(0.019, 0.0, 0.000), rpy=(0.0, math.pi / 2.0, 0.0)),
        material=plastic,
        name="right_knuckle",
    )
    roof_mount.visual(
        Cylinder(radius=0.0055, length=0.004),
        origin=Origin(xyz=(-0.032, 0.0, 0.026)),
        material=fastener,
        name="left_bolt_cap",
    )
    roof_mount.visual(
        Cylinder(radius=0.0055, length=0.004),
        origin=Origin(xyz=(0.032, 0.0, 0.026)),
        material=fastener,
        name="right_bolt_cap",
    )
    roof_mount.inertial = Inertial.from_geometry(
        Box((0.118, 0.046, 0.030)),
        mass=0.22,
        origin=Origin(xyz=(0.0, 0.0, 0.015)),
    )

    hinge_arm = model.part("hinge_arm")
    hinge_arm.visual(
        Cylinder(radius=0.009, length=0.028),
        origin=Origin(rpy=(0.0, math.pi / 2.0, 0.0)),
        material=steel,
        name="primary_barrel",
    )
    hinge_arm.visual(
        Box((0.026, 0.020, 0.014)),
        origin=Origin(xyz=(0.0, 0.012, -0.005)),
        material=steel,
        name="root_gusset",
    )
    hinge_arm.visual(
        Box((0.018, 0.064, 0.007)),
        origin=Origin(xyz=(0.0, 0.045, -0.011)),
        material=steel,
        name="arm_spine",
    )
    hinge_arm.visual(
        Box((0.005, 0.020, 0.026)),
        origin=Origin(xyz=(-0.0095, 0.082, -0.018)),
        material=steel,
        name="left_fork",
    )
    hinge_arm.visual(
        Box((0.005, 0.020, 0.026)),
        origin=Origin(xyz=(0.0095, 0.082, -0.018)),
        material=steel,
        name="right_fork",
    )
    hinge_arm.visual(
        Box((0.018, 0.008, 0.007)),
        origin=Origin(xyz=(0.0, 0.087, -0.028)),
        material=steel,
        name="fork_bridge",
    )
    hinge_arm.inertial = Inertial.from_geometry(
        Box((0.028, 0.095, 0.034)),
        mass=0.16,
        origin=Origin(xyz=(0.0, 0.047, -0.014)),
    )

    visor_panel = model.part("visor_panel")
    visor_panel.visual(
        _panel_mesh(),
        origin=Origin(xyz=(0.186, 0.073, 0.000)),
        material=fabric,
        name="panel_shell",
    )
    visor_panel.visual(
        Box((0.014, 0.022, 0.020)),
        origin=Origin(xyz=(0.0, 0.012, 0.000)),
        material=plastic,
        name="pivot_hub",
    )
    visor_panel.visual(
        Cylinder(radius=0.0055, length=0.030),
        origin=Origin(),
        material=steel,
        name="pivot_pin",
    )
    visor_panel.visual(
        Cylinder(radius=0.010, length=0.004),
        origin=Origin(xyz=(0.0, 0.000, 0.017)),
        material=plastic,
        name="top_retainer",
    )
    visor_panel.visual(
        Cylinder(radius=0.010, length=0.004),
        origin=Origin(xyz=(0.0, 0.000, -0.017)),
        material=plastic,
        name="bottom_retainer",
    )
    visor_panel.visual(
        Box((0.030, 0.018, 0.010)),
        origin=Origin(xyz=(0.014, 0.014, -0.004)),
        material=plastic,
        name="hub_reinforcement",
    )
    visor_panel.inertial = Inertial.from_geometry(
        Box((0.340, 0.150, 0.022)),
        mass=0.62,
        origin=Origin(xyz=(0.187, 0.074, 0.0)),
    )

    model.articulation(
        "roof_to_arm",
        ArticulationType.REVOLUTE,
        parent=roof_mount,
        child=hinge_arm,
        origin=Origin(),
        axis=(-1.0, 0.0, 0.0),
        motion_limits=MotionLimits(
            effort=8.0,
            velocity=2.0,
            lower=0.0,
            upper=1.35,
        ),
    )
    model.articulation(
        "arm_to_panel",
        ArticulationType.REVOLUTE,
        parent=hinge_arm,
        child=visor_panel,
        origin=Origin(xyz=(0.0, 0.082, -0.018)),
        axis=(0.0, 0.0, 1.0),
        motion_limits=MotionLimits(
            effort=3.0,
            velocity=2.5,
            lower=0.0,
            upper=1.45,
        ),
    )

    return model


def run_tests() -> TestReport:
    ctx = TestContext(object_model)
    roof_mount = object_model.get_part("roof_mount")
    hinge_arm = object_model.get_part("hinge_arm")
    visor_panel = object_model.get_part("visor_panel")
    primary = object_model.get_articulation("roof_to_arm")
    secondary = object_model.get_articulation("arm_to_panel")

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
        roof_mount,
        elem_a="primary_barrel",
        elem_b="left_knuckle",
        name="primary_barrel_supported_by_left_knuckle",
    )
    ctx.expect_contact(
        hinge_arm,
        roof_mount,
        elem_a="primary_barrel",
        elem_b="right_knuckle",
        name="primary_barrel_supported_by_right_knuckle",
    )
    ctx.expect_contact(
        visor_panel,
        hinge_arm,
        elem_a="pivot_hub",
        elem_b="left_fork",
        name="panel_hub_captured_by_left_fork",
    )
    ctx.expect_contact(
        visor_panel,
        hinge_arm,
        elem_a="pivot_hub",
        elem_b="right_fork",
        name="panel_hub_captured_by_right_fork",
    )

    with ctx.pose({primary: 0.0, secondary: 0.0}):
        ctx.expect_gap(
            roof_mount,
            visor_panel,
            axis="z",
            positive_elem="base_plate",
            negative_elem="panel_shell",
            min_gap=0.020,
            max_gap=0.040,
            name="stowed_panel_sits_below_roof_mount",
        )
        ctx.expect_overlap(
            visor_panel,
            roof_mount,
            axes="x",
            min_overlap=0.035,
            name="stowed_panel_stays_under_mount_span",
        )

    with ctx.pose({primary: 0.0, secondary: 0.0}):
        stowed_shell = ctx.part_element_world_aabb(visor_panel, elem="panel_shell")
    with ctx.pose({primary: 1.20, secondary: 0.0}):
        lowered_shell = ctx.part_element_world_aabb(visor_panel, elem="panel_shell")
        lowered_ok = (
            stowed_shell is not None
            and lowered_shell is not None
            and lowered_shell[0][2] < stowed_shell[0][2] - 0.10
        )
        ctx.check(
            "primary_pivot_lowers_panel",
            lowered_ok,
            details="primary hinge should carry the visor panel well below the roof mount when deployed",
        )
        ctx.expect_gap(
            roof_mount,
            visor_panel,
            axis="z",
            positive_elem="base_plate",
            negative_elem="panel_shell",
            min_gap=0.090,
            name="deployed_panel_clears_roof_mount",
        )

    with ctx.pose({primary: 1.20, secondary: 0.0}):
        unswung_shell = ctx.part_element_world_aabb(visor_panel, elem="panel_shell")
    with ctx.pose({primary: 1.20, secondary: 1.30}):
        swung_shell = ctx.part_element_world_aabb(visor_panel, elem="panel_shell")
        swung_ok = False
        if unswung_shell is not None and swung_shell is not None:
            unswung_center_x = 0.5 * (unswung_shell[0][0] + unswung_shell[1][0])
            swung_center_x = 0.5 * (swung_shell[0][0] + swung_shell[1][0])
            swung_ok = abs(swung_center_x - unswung_center_x) > 0.12
        ctx.check(
            "secondary_pivot_swings_panel_sideways",
            swung_ok,
            details="secondary pivot should visibly swing the visor panel away from its straight-ahead deployed orientation",
        )

    return ctx.report()


# >>> USER_CODE_END

object_model = build_object_model()
