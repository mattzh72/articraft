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


def _add_header_bolts(part, material, *, z: float, xs: tuple[float, ...], ys: tuple[float, ...]) -> None:
    bolt_index = 0
    for x in xs:
        for y in ys:
            part.visual(
                Cylinder(radius=0.008, length=0.006),
                origin=Origin(xyz=(x, y, z)),
                material=material,
                name=f"header_bolt_{bolt_index}",
            )
            bolt_index += 1
def build_object_model() -> ArticulatedObject:
    model = ArticulatedObject(name="industrial_safety_visor")

    structure_gray = model.material("structure_gray", rgba=(0.38, 0.40, 0.42, 1.0))
    dark_oxide = model.material("dark_oxide", rgba=(0.16, 0.17, 0.18, 1.0))
    zinc = model.material("zinc", rgba=(0.73, 0.75, 0.77, 1.0))
    visor_smoke = model.material("visor_smoke", rgba=(0.33, 0.24, 0.12, 0.55))
    bumper_black = model.material("bumper_black", rgba=(0.08, 0.08, 0.09, 1.0))
    safety_yellow = model.material("safety_yellow", rgba=(0.89, 0.73, 0.12, 1.0))

    roof_mount = model.part("roof_mount")
    roof_mount.visual(
        Box((0.16, 0.62, 0.03)),
        origin=Origin(xyz=(0.0, 0.0, -0.015)),
        material=structure_gray,
        name="header_plate",
    )
    roof_mount.visual(
        Box((0.12, 0.42, 0.016)),
        origin=Origin(xyz=(-0.015, 0.0, -0.038)),
        material=dark_oxide,
        name="backer_plate",
    )
    roof_mount.visual(
        Box((0.120, 0.100, 0.018)),
        origin=Origin(xyz=(0.030, 0.0, -0.021)),
        material=structure_gray,
        name="hinge_bridge",
    )
    roof_mount.visual(
        Box((0.055, 0.018, 0.10)),
        origin=Origin(xyz=(0.08, 0.033, -0.08)),
        material=dark_oxide,
        name="primary_clevis_left",
    )
    roof_mount.visual(
        Box((0.055, 0.018, 0.10)),
        origin=Origin(xyz=(0.08, -0.033, -0.08)),
        material=dark_oxide,
        name="primary_clevis_right",
    )
    roof_mount.visual(
        Box((0.085, 0.010, 0.12)),
        origin=Origin(xyz=(0.092, 0.058, -0.08)),
        material=structure_gray,
        name="primary_guard_left",
    )
    roof_mount.visual(
        Box((0.085, 0.010, 0.12)),
        origin=Origin(xyz=(0.092, -0.058, -0.08)),
        material=structure_gray,
        name="primary_guard_right",
    )
    roof_mount.visual(
        Box((0.050, 0.080, 0.040)),
        origin=Origin(xyz=(0.130, 0.0, -0.041)),
        material=safety_yellow,
        name="stow_stop_block",
    )
    roof_mount.visual(
        Box((0.060, 0.008, 0.075)),
        origin=Origin(xyz=(0.068, 0.064, -0.083)),
        material=safety_yellow,
        name="primary_lock_sector",
    )
    roof_mount.visual(
        Box((0.028, 0.018, 0.028)),
        origin=Origin(xyz=(0.020, 0.033, -0.122)),
        material=safety_yellow,
        name="deploy_stop_left",
    )
    roof_mount.visual(
        Box((0.019, 0.018, 0.072)),
        origin=Origin(xyz=(0.043, 0.033, -0.096)),
        material=safety_yellow,
        name="deploy_stop_left_rib",
    )
    roof_mount.visual(
        Box((0.028, 0.018, 0.028)),
        origin=Origin(xyz=(0.020, -0.033, -0.122)),
        material=safety_yellow,
        name="deploy_stop_right",
    )
    roof_mount.visual(
        Box((0.019, 0.018, 0.072)),
        origin=Origin(xyz=(0.043, -0.033, -0.096)),
        material=safety_yellow,
        name="deploy_stop_right_rib",
    )
    _add_header_bolts(
        roof_mount,
        zinc,
        z=0.003,
        xs=(-0.045, 0.000, 0.045),
        ys=(-0.215, 0.215),
    )
    roof_mount.inertial = Inertial.from_geometry(
        Box((0.18, 0.62, 0.16)),
        mass=5.2,
        origin=Origin(xyz=(0.01, 0.0, -0.08)),
    )

    hinge_arm = model.part("hinge_arm")
    hinge_arm.visual(
        Cylinder(radius=0.020, length=0.048),
        origin=Origin(rpy=(math.pi / 2.0, 0.0, 0.0)),
        material=dark_oxide,
        name="primary_barrel",
    )
    hinge_arm.visual(
        Box((0.050, 0.040, 0.040)),
        origin=Origin(xyz=(0.025, 0.0, 0.0)),
        material=structure_gray,
        name="pivot_web",
    )
    hinge_arm.visual(
        Box((0.115, 0.012, 0.040)),
        origin=Origin(xyz=(0.1075, 0.016, -0.004)),
        material=structure_gray,
        name="arm_plate_left",
    )
    hinge_arm.visual(
        Box((0.115, 0.012, 0.040)),
        origin=Origin(xyz=(0.1075, -0.016, -0.004)),
        material=structure_gray,
        name="arm_plate_right",
    )
    hinge_arm.visual(
        Box((0.040, 0.060, 0.022)),
        origin=Origin(xyz=(0.060, 0.0, 0.008)),
        material=safety_yellow,
        name="stow_stop_pad",
    )
    hinge_arm.visual(
        Box((0.130, 0.060, 0.022)),
        origin=Origin(xyz=(0.125, 0.0, -0.032)),
        material=dark_oxide,
        name="arm_spine",
    )
    hinge_arm.visual(
        Box((0.050, 0.020, 0.026)),
        origin=Origin(xyz=(0.030, 0.0, -0.033)),
        material=structure_gray,
        name="primary_lock_plunger_housing",
    )
    hinge_arm.visual(
        Box((0.050, 0.012, 0.082)),
        origin=Origin(xyz=(0.210, 0.034, -0.006)),
        material=dark_oxide,
        name="secondary_yoke_left",
    )
    hinge_arm.visual(
        Box((0.050, 0.012, 0.082)),
        origin=Origin(xyz=(0.210, -0.034, -0.006)),
        material=dark_oxide,
        name="secondary_yoke_right",
    )
    hinge_arm.visual(
        Box((0.050, 0.056, 0.018)),
        origin=Origin(xyz=(0.210, 0.0, 0.032)),
        material=structure_gray,
        name="secondary_guard_bridge",
    )
    hinge_arm.visual(
        Box((0.036, 0.040, 0.018)),
        origin=Origin(xyz=(0.198, 0.0, 0.047)),
        material=safety_yellow,
        name="secondary_stop_block",
    )
    hinge_arm.inertial = Inertial.from_geometry(
        Box((0.27, 0.10, 0.12)),
        mass=2.6,
        origin=Origin(xyz=(0.135, 0.0, -0.012)),
    )

    visor_panel = model.part("visor_panel")
    visor_panel.visual(
        Cylinder(radius=0.020, length=0.056),
        origin=Origin(rpy=(math.pi / 2.0, 0.0, 0.0)),
        material=dark_oxide,
        name="secondary_barrel",
    )
    visor_panel.visual(
        Box((0.090, 0.680, 0.032)),
        origin=Origin(xyz=(0.155, 0.0, -0.018)),
        material=structure_gray,
        name="top_beam",
    )
    visor_panel.visual(
        Box((0.130, 0.040, 0.024)),
        origin=Origin(xyz=(0.055, 0.0, -0.022)),
        material=structure_gray,
        name="hinge_spine",
    )
    visor_panel.visual(
        Box((0.245, 0.030, 0.040)),
        origin=Origin(xyz=(0.1225, 0.318, -0.054)),
        material=structure_gray,
        name="left_side_rail",
    )
    visor_panel.visual(
        Box((0.245, 0.030, 0.040)),
        origin=Origin(xyz=(0.1225, -0.318, -0.054)),
        material=structure_gray,
        name="right_side_rail",
    )
    visor_panel.visual(
        Box((0.032, 0.680, 0.040)),
        origin=Origin(xyz=(0.228, 0.0, -0.054)),
        material=structure_gray,
        name="bottom_rail",
    )
    visor_panel.visual(
        Box((0.205, 0.618, 0.012)),
        origin=Origin(xyz=(0.132, 0.0, -0.058)),
        material=visor_smoke,
        name="shield_insert",
    )
    visor_panel.visual(
        Box((0.210, 0.032, 0.026)),
        origin=Origin(xyz=(0.118, 0.105, -0.048)),
        material=dark_oxide,
        name="center_rib_left",
    )
    visor_panel.visual(
        Box((0.210, 0.032, 0.026)),
        origin=Origin(xyz=(0.118, -0.105, -0.048)),
        material=dark_oxide,
        name="center_rib_right",
    )
    visor_panel.visual(
        Box((0.060, 0.060, 0.030)),
        origin=Origin(xyz=(0.005, 0.0, 0.0)),
        material=dark_oxide,
        name="hinge_boss_block",
    )
    visor_panel.visual(
        Box((0.040, 0.020, 0.018)),
        origin=Origin(xyz=(-0.030, 0.0, 0.018)),
        material=safety_yellow,
        name="secondary_stop_tongue",
    )
    visor_panel.visual(
        Box((0.012, 0.684, 0.018)),
        origin=Origin(xyz=(0.244, 0.0, -0.078)),
        material=bumper_black,
        name="edge_bumper",
    )
    visor_panel.inertial = Inertial.from_geometry(
        Box((0.26, 0.70, 0.13)),
        mass=4.9,
        origin=Origin(xyz=(0.130, 0.0, -0.052)),
    )

    model.articulation(
        "roof_to_arm",
        ArticulationType.REVOLUTE,
        parent=roof_mount,
        child=hinge_arm,
        origin=Origin(xyz=(0.080, 0.0, -0.080)),
        axis=(0.0, 1.0, 0.0),
        motion_limits=MotionLimits(
            effort=45.0,
            velocity=1.2,
            lower=0.0,
            upper=1.22,
        ),
    )
    model.articulation(
        "arm_to_panel",
        ArticulationType.REVOLUTE,
        parent=hinge_arm,
        child=visor_panel,
        origin=Origin(xyz=(0.210, 0.0, -0.006)),
        axis=(0.0, 1.0, 0.0),
        motion_limits=MotionLimits(
            effort=28.0,
            velocity=1.0,
            lower=-0.08,
            upper=0.82,
        ),
    )

    return model


def run_tests() -> TestReport:
    ctx = TestContext(object_model)
    roof_mount = object_model.get_part("roof_mount")
    hinge_arm = object_model.get_part("hinge_arm")
    visor_panel = object_model.get_part("visor_panel")
    primary_hinge = object_model.get_articulation("roof_to_arm")
    secondary_hinge = object_model.get_articulation("arm_to_panel")

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
        "all_prompt_parts_exist",
        all(part is not None for part in (roof_mount, hinge_arm, visor_panel)),
        "roof mount, hinge arm, and visor panel must all be present",
    )
    ctx.check(
        "primary_hinge_axis_is_lateral",
        tuple(primary_hinge.axis) == (0.0, 1.0, 0.0),
        f"expected primary hinge axis (0, 1, 0), got {primary_hinge.axis}",
    )
    ctx.check(
        "secondary_hinge_axis_is_lateral",
        tuple(secondary_hinge.axis) == (0.0, 1.0, 0.0),
        f"expected secondary hinge axis (0, 1, 0), got {secondary_hinge.axis}",
    )
    ctx.expect_contact(
        roof_mount,
        hinge_arm,
        elem_a="primary_clevis_left",
        elem_b="primary_barrel",
        name="primary_pivot_left_cheek_supports_arm",
    )
    ctx.expect_contact(
        roof_mount,
        hinge_arm,
        elem_a="primary_clevis_right",
        elem_b="primary_barrel",
        name="primary_pivot_right_cheek_supports_arm",
    )
    ctx.expect_contact(
        hinge_arm,
        visor_panel,
        elem_a="secondary_yoke_left",
        elem_b="secondary_barrel",
        name="secondary_pivot_left_cheek_supports_panel",
    )
    ctx.expect_contact(
        hinge_arm,
        visor_panel,
        elem_a="secondary_yoke_right",
        elem_b="secondary_barrel",
        name="secondary_pivot_right_cheek_supports_panel",
    )
    ctx.expect_contact(
        roof_mount,
        hinge_arm,
        elem_a="stow_stop_block",
        elem_b="stow_stop_pad",
        name="primary_stow_stop_is_seated",
    )

    with ctx.pose({primary_hinge: 0.0, secondary_hinge: 0.0}):
        stowed_panel_origin = ctx.part_world_position(visor_panel)
        stowed_bottom_rail = ctx.part_element_world_aabb(visor_panel, elem="bottom_rail")
        ctx.expect_gap(
            roof_mount,
            visor_panel,
            axis="z",
            min_gap=0.020,
            positive_elem="header_plate",
            negative_elem="top_beam",
            name="stowed_panel_hangs_below_header",
        )
        ctx.expect_overlap(
            visor_panel,
            roof_mount,
            axes="y",
            min_overlap=0.55,
            name="visor_panel_spans_header_width",
        )

    with ctx.pose({primary_hinge: 1.00, secondary_hinge: 0.0}):
        deployed_panel_origin = ctx.part_world_position(visor_panel)
        panel_drops = (
            stowed_panel_origin is not None
            and deployed_panel_origin is not None
            and deployed_panel_origin[2] < (stowed_panel_origin[2] - 0.16)
        )
        ctx.check(
            "primary_hinge_drops_panel_downward",
            panel_drops,
            f"stowed origin={stowed_panel_origin}, deployed origin={deployed_panel_origin}",
        )

    with ctx.pose({primary_hinge: 1.00, secondary_hinge: 0.48}):
        swung_bottom_rail = ctx.part_element_world_aabb(visor_panel, elem="bottom_rail")
        secondary_drop = False
        if stowed_bottom_rail is not None and swung_bottom_rail is not None:
            stowed_bottom_center_z = (stowed_bottom_rail[0][2] + stowed_bottom_rail[1][2]) * 0.5
            swung_bottom_center_z = (swung_bottom_rail[0][2] + swung_bottom_rail[1][2]) * 0.5
            secondary_drop = swung_bottom_center_z < (stowed_bottom_center_z - 0.20)
        ctx.check(
            "secondary_pivot_swings_free_edge_lower",
            secondary_drop,
            f"stowed bottom rail={stowed_bottom_rail}, swung bottom rail={swung_bottom_rail}",
        )
        ctx.expect_gap(
            roof_mount,
            visor_panel,
            axis="z",
            min_gap=0.150,
            positive_elem="header_plate",
            negative_elem="top_beam",
            name="deployed_panel_stays_clear_of_roof",
        )

    return ctx.report()


# >>> USER_CODE_END

object_model = build_object_model()
