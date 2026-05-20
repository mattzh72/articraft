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
    tube_from_spline_points,
)


def _add_x_cylinder(
    part,
    *,
    radius: float,
    length: float,
    xyz: tuple[float, float, float],
    material,
    name: str,
) -> None:
    part.visual(
        Cylinder(radius=radius, length=length),
        origin=Origin(xyz=xyz, rpy=(0.0, math.pi / 2.0, 0.0)),
        material=material,
        name=name,
    )


def _arm_side_mesh(side: str):
    sign = -1.0 if side == "left" else 1.0
    path = [
        (sign * 0.39, 0.000, 0.000),
        (sign * 0.39, 0.012, -0.045),
        (sign * 0.39, 0.025, -0.105),
        (sign * 0.39, 0.040, -0.190),
    ]
    return mesh_from_geometry(
        tube_from_spline_points(
            path,
            radius=0.014,
            samples_per_segment=16,
            radial_segments=18,
            cap_ends=True,
        ),
        f"{side}_visor_hinge_arm",
    )


def build_object_model() -> ArticulatedObject:
    model = ArticulatedObject(name="weatherproof_windshield_sun_visor")

    powder_coat = model.material("powder_coat_charcoal", rgba=(0.24, 0.25, 0.27, 1.0))
    silver_panel = model.material("silver_panel", rgba=(0.73, 0.76, 0.78, 1.0))
    rubber_seal = model.material("rubber_seal", rgba=(0.10, 0.11, 0.12, 1.0))
    stainless = model.material("stainless_hardware", rgba=(0.78, 0.80, 0.82, 1.0))

    roof_mount = model.part("roof_mount")
    roof_mount.visual(
        Box((1.12, 0.10, 0.04)),
        origin=Origin(xyz=(0.0, 0.000, 0.020)),
        material=powder_coat,
        name="mount_rail",
    )
    roof_mount.visual(
        Box((1.04, 0.06, 0.014)),
        origin=Origin(xyz=(0.0, 0.028, 0.047)),
        material=powder_coat,
        name="roof_backer_pad",
    )
    roof_mount.visual(
        Box((1.20, 0.05, 0.016)),
        origin=Origin(xyz=(0.0, -0.073, 0.040)),
        material=powder_coat,
        name="drip_hood",
    )
    roof_mount.visual(
        Box((1.06, 0.022, 0.008)),
        origin=Origin(xyz=(0.0, -0.034, -0.004)),
        material=rubber_seal,
        name="roof_seal_strip",
    )
    roof_mount.visual(
        Box((0.08, 0.07, 0.06)),
        origin=Origin(xyz=(-0.50, -0.072, -0.004)),
        material=powder_coat,
        name="left_primary_cheek",
    )
    roof_mount.visual(
        Box((0.08, 0.07, 0.06)),
        origin=Origin(xyz=(0.50, -0.072, -0.004)),
        material=powder_coat,
        name="right_primary_cheek",
    )
    _add_x_cylinder(
        roof_mount,
        radius=0.028,
        length=0.04,
        xyz=(-0.44, -0.072, -0.004),
        material=powder_coat,
        name="left_primary_bearing_cap",
    )
    _add_x_cylinder(
        roof_mount,
        radius=0.028,
        length=0.04,
        xyz=(0.44, -0.072, -0.004),
        material=powder_coat,
        name="right_primary_bearing_cap",
    )
    _add_x_cylinder(
        roof_mount,
        radius=0.010,
        length=0.008,
        xyz=(-0.544, -0.072, -0.004),
        material=stainless,
        name="left_mount_bolt_head",
    )
    _add_x_cylinder(
        roof_mount,
        radius=0.010,
        length=0.008,
        xyz=(0.544, -0.072, -0.004),
        material=stainless,
        name="right_mount_bolt_head",
    )
    roof_mount.inertial = Inertial.from_geometry(
        Box((1.20, 0.13, 0.07)),
        mass=5.8,
        origin=Origin(xyz=(0.0, -0.008, 0.022)),
    )

    hinge_arm = model.part("hinge_arm")
    _add_x_cylinder(
        hinge_arm,
        radius=0.018,
        length=0.72,
        xyz=(0.0, 0.0, 0.0),
        material=powder_coat,
        name="primary_torque_tube",
    )
    _add_x_cylinder(
        hinge_arm,
        radius=0.024,
        length=0.06,
        xyz=(-0.39, 0.0, 0.0),
        material=stainless,
        name="left_primary_collar",
    )
    _add_x_cylinder(
        hinge_arm,
        radius=0.024,
        length=0.06,
        xyz=(0.39, 0.0, 0.0),
        material=stainless,
        name="right_primary_collar",
    )
    hinge_arm.visual(
        _arm_side_mesh("left"),
        material=powder_coat,
        name="left_drop_arm",
    )
    hinge_arm.visual(
        _arm_side_mesh("right"),
        material=powder_coat,
        name="right_drop_arm",
    )
    _add_x_cylinder(
        hinge_arm,
        radius=0.013,
        length=0.78,
        xyz=(0.0, 0.025, -0.105),
        material=powder_coat,
        name="mid_cross_brace",
    )
    _add_x_cylinder(
        hinge_arm,
        radius=0.024,
        length=0.06,
        xyz=(-0.39, 0.040, -0.190),
        material=stainless,
        name="left_secondary_clevis_boss",
    )
    _add_x_cylinder(
        hinge_arm,
        radius=0.024,
        length=0.06,
        xyz=(0.39, 0.040, -0.190),
        material=stainless,
        name="right_secondary_clevis_boss",
    )
    hinge_arm.inertial = Inertial.from_geometry(
        Box((0.84, 0.10, 0.24)),
        mass=2.2,
        origin=Origin(xyz=(0.0, 0.020, -0.095)),
    )

    visor_panel = model.part("visor_panel")
    _add_x_cylinder(
        visor_panel,
        radius=0.017,
        length=0.56,
        xyz=(0.0, 0.0, 0.0),
        material=stainless,
        name="secondary_pivot_tube",
    )
    _add_x_cylinder(
        visor_panel,
        radius=0.022,
        length=0.08,
        xyz=(-0.32, 0.0, 0.0),
        material=stainless,
        name="left_secondary_collar",
    )
    _add_x_cylinder(
        visor_panel,
        radius=0.022,
        length=0.08,
        xyz=(0.32, 0.0, 0.0),
        material=stainless,
        name="right_secondary_collar",
    )
    visor_panel.visual(
        Box((0.68, 0.050, 0.025)),
        origin=Origin(xyz=(0.0, -0.022, -0.015)),
        material=powder_coat,
        name="sealed_clamp_rail",
    )
    visor_panel.visual(
        Box((1.28, 0.180, 0.012)),
        origin=Origin(xyz=(0.0, -0.118, -0.026)),
        material=silver_panel,
        name="visor_blade",
    )
    visor_panel.visual(
        Box((1.28, 0.018, 0.026)),
        origin=Origin(xyz=(0.0, -0.209, -0.034)),
        material=powder_coat,
        name="front_drip_edge",
    )
    visor_panel.visual(
        Box((0.040, 0.050, 0.020)),
        origin=Origin(xyz=(-0.34, -0.020, -0.018)),
        material=rubber_seal,
        name="left_side_seal_boot",
    )
    visor_panel.visual(
        Box((0.040, 0.050, 0.020)),
        origin=Origin(xyz=(0.34, -0.020, -0.018)),
        material=rubber_seal,
        name="right_side_seal_boot",
    )
    visor_panel.visual(
        Box((0.016, 0.200, 0.020)),
        origin=Origin(xyz=(-0.632, -0.118, -0.026)),
        material=powder_coat,
        name="left_end_cap",
    )
    visor_panel.visual(
        Box((0.016, 0.200, 0.020)),
        origin=Origin(xyz=(0.632, -0.118, -0.026)),
        material=powder_coat,
        name="right_end_cap",
    )
    visor_panel.inertial = Inertial.from_geometry(
        Box((1.28, 0.22, 0.04)),
        mass=3.0,
        origin=Origin(xyz=(0.0, -0.115, -0.022)),
    )

    model.articulation(
        "roof_to_hinge_arm",
        ArticulationType.REVOLUTE,
        parent=roof_mount,
        child=hinge_arm,
        origin=Origin(xyz=(0.0, -0.072, -0.004)),
        axis=(1.0, 0.0, 0.0),
        motion_limits=MotionLimits(
            effort=35.0,
            velocity=1.2,
            lower=-0.55,
            upper=0.45,
        ),
    )
    model.articulation(
        "hinge_arm_to_visor_panel",
        ArticulationType.REVOLUTE,
        parent=hinge_arm,
        child=visor_panel,
        origin=Origin(xyz=(0.0, 0.040, -0.190)),
        axis=(1.0, 0.0, 0.0),
        motion_limits=MotionLimits(
            effort=22.0,
            velocity=1.4,
            lower=-0.25,
            upper=0.55,
        ),
    )

    return model


def run_tests() -> TestReport:
    ctx = TestContext(object_model)
    roof_mount = object_model.get_part("roof_mount")
    hinge_arm = object_model.get_part("hinge_arm")
    visor_panel = object_model.get_part("visor_panel")
    primary = object_model.get_articulation("roof_to_hinge_arm")
    secondary = object_model.get_articulation("hinge_arm_to_visor_panel")

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

    with ctx.pose({primary: 0.0, secondary: 0.0}):
        ctx.expect_contact(
            hinge_arm,
            roof_mount,
            name="primary_bearing_stack_is_physically_supported",
        )
        ctx.expect_contact(
            visor_panel,
            hinge_arm,
            name="secondary_bearing_stack_is_physically_supported",
        )
        ctx.expect_gap(
            roof_mount,
            visor_panel,
            axis="z",
            min_gap=0.12,
            name="visor_panel_hangs_below_roof_mount",
        )
        ctx.expect_overlap(
            visor_panel,
            roof_mount,
            axes="x",
            min_overlap=1.00,
            name="visor_width_covers_windshield_span",
        )

    with ctx.pose({primary: 0.35, secondary: 0.0}):
        stowed_panel_pos = ctx.part_world_position(visor_panel)
    with ctx.pose({primary: -0.35, secondary: 0.0}):
        deployed_panel_pos = ctx.part_world_position(visor_panel)

    primary_ok = (
        stowed_panel_pos is not None
        and deployed_panel_pos is not None
        and stowed_panel_pos[2] > deployed_panel_pos[2] + 0.010
        and stowed_panel_pos[1] > deployed_panel_pos[1] + 0.040
    )
    ctx.check(
        "primary_pivot_stows_panel_upward_and_rearward",
        primary_ok,
        details=(
            f"stowed={stowed_panel_pos}, deployed={deployed_panel_pos}; "
            "expected positive primary rotation to lift the visor and pull it rearward."
        ),
    )

    def _aabb_center_z(aabb):
        if aabb is None:
            return None
        lower, upper = aabb
        return (lower[2] + upper[2]) * 0.5

    with ctx.pose({primary: 0.0, secondary: 0.45}):
        nose_down = ctx.part_element_world_aabb(visor_panel, elem="front_drip_edge")
    with ctx.pose({primary: 0.0, secondary: -0.20}):
        nose_up = ctx.part_element_world_aabb(visor_panel, elem="front_drip_edge")

    nose_down_z = _aabb_center_z(nose_down)
    nose_up_z = _aabb_center_z(nose_up)
    secondary_ok = (
        nose_down_z is not None
        and nose_up_z is not None
        and nose_down_z < nose_up_z - 0.010
    )
    ctx.check(
        "secondary_pivot_can_tip_drip_edge_downward",
        secondary_ok,
        details=(
            f"nose_down_z={nose_down_z}, nose_up_z={nose_up_z}; "
            "expected positive secondary rotation to drop the visor nose for glare and rain control."
        ),
    )

    return ctx.report()


# >>> USER_CODE_END

object_model = build_object_model()
