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
    section_loft,
    sweep_profile_along_spline,
)


def _yz_section(
    y_size: float,
    z_size: float,
    radius: float,
    x_pos: float,
    *,
    z_shift: float = 0.0,
    corner_segments: int = 8,
) -> list[tuple[float, float, float]]:
    profile = rounded_rect_profile(z_size, y_size, radius, corner_segments=corner_segments)
    return [(x_pos, y_val, z_val + z_shift) for z_val, y_val in profile]


def _build_mount_housing_mesh():
    return ExtrudeGeometry(
        rounded_rect_profile(0.112, 0.056, 0.014, corner_segments=8),
        0.020,
        cap=True,
        center=True,
        closed=True,
    )


def _build_arm_mesh():
    arm_profile = rounded_rect_profile(0.016, 0.0052, 0.0022, corner_segments=6)
    arm_path = [
        (0.012, 0.0, -0.0005),
        (0.024, 0.0, -0.0015),
        (0.042, 0.0, -0.0050),
        (0.054, 0.0, -0.0078),
        (0.060, 0.0, -0.0085),
    ]
    return sweep_profile_along_spline(
        arm_path,
        profile=arm_profile,
        samples_per_segment=18,
        cap_profile=True,
        up_hint=(0.0, 0.0, 1.0),
    )


def _build_visor_body_mesh():
    sections = [
        _yz_section(0.146, 0.0185, 0.0088, 0.010, z_shift=0.0000),
        _yz_section(0.150, 0.0190, 0.0090, 0.120, z_shift=0.0005),
        _yz_section(0.148, 0.0182, 0.0088, 0.245, z_shift=0.0002),
        _yz_section(0.142, 0.0162, 0.0076, 0.330, z_shift=-0.0004),
    ]
    return section_loft(sections)


def build_object_model() -> ArticulatedObject:
    model = ArticulatedObject(name="premium_windshield_sun_visor")

    roof_polymer = model.material("roof_polymer", rgba=(0.18, 0.19, 0.20, 1.0))
    satin_metal = model.material("satin_metal", rgba=(0.67, 0.69, 0.72, 1.0))
    soft_polymer = model.material("soft_polymer", rgba=(0.48, 0.49, 0.46, 1.0))
    trim_elastomer = model.material("trim_elastomer", rgba=(0.11, 0.11, 0.12, 1.0))

    roof_mount = model.part("roof_mount")
    roof_mount.visual(
        mesh_from_geometry(_build_mount_housing_mesh(), "visor_mount_housing"),
        origin=Origin(xyz=(-0.068, 0.0, 0.004)),
        material=roof_polymer,
        name="mount_housing",
    )
    roof_mount.visual(
        Box((0.022, 0.028, 0.012)),
        origin=Origin(xyz=(-0.021, 0.0, -0.001)),
        material=roof_polymer,
        name="hinge_neck",
    )
    roof_mount.visual(
        Cylinder(radius=0.011, length=0.008),
        origin=Origin(xyz=(0.0, -0.010, 0.0), rpy=(math.pi / 2.0, 0.0, 0.0)),
        material=roof_polymer,
        name="left_primary_ear",
    )
    roof_mount.visual(
        Cylinder(radius=0.011, length=0.008),
        origin=Origin(xyz=(0.0, 0.010, 0.0), rpy=(math.pi / 2.0, 0.0, 0.0)),
        material=roof_polymer,
        name="right_primary_ear",
    )
    roof_mount.visual(
        Box((0.088, 0.032, 0.0016)),
        origin=Origin(xyz=(-0.070, 0.0, 0.0110)),
        material=satin_metal,
        name="roof_plate",
    )
    roof_mount.visual(
        Box((0.014, 0.014, 0.010)),
        origin=Origin(xyz=(-0.094, 0.0, 0.009)),
        material=roof_polymer,
        name="plate_post_left",
    )
    roof_mount.visual(
        Box((0.014, 0.014, 0.010)),
        origin=Origin(xyz=(-0.046, 0.0, 0.009)),
        material=roof_polymer,
        name="plate_post_right",
    )
    roof_mount.inertial = Inertial.from_geometry(
        Box((0.116, 0.058, 0.026)),
        mass=0.38,
        origin=Origin(xyz=(-0.060, 0.0, 0.003)),
    )

    hinge_arm = model.part("hinge_arm")
    hinge_arm.visual(
        Cylinder(radius=0.0088, length=0.012),
        origin=Origin(xyz=(0.0, 0.0, 0.0), rpy=(math.pi / 2.0, 0.0, 0.0)),
        material=satin_metal,
        name="primary_barrel",
    )
    hinge_arm.visual(
        mesh_from_geometry(_build_arm_mesh(), "visor_hinge_arm"),
        material=satin_metal,
        name="arm_blade",
    )
    hinge_arm.visual(
        Box((0.016, 0.012, 0.008)),
        origin=Origin(xyz=(0.010, 0.0, -0.004)),
        material=satin_metal,
        name="arm_shoulder",
    )
    hinge_arm.visual(
        Box((0.024, 0.024, 0.020)),
        origin=Origin(xyz=(0.068, 0.0, -0.010)),
        material=satin_metal,
        name="secondary_bridge",
    )
    hinge_arm.visual(
        Cylinder(radius=0.009, length=0.005),
        origin=Origin(xyz=(0.084, 0.0, -0.0155), rpy=(0.0, 0.0, 0.0)),
        material=trim_elastomer,
        name="lower_secondary_ear",
    )
    hinge_arm.visual(
        Cylinder(radius=0.009, length=0.005),
        origin=Origin(xyz=(0.084, 0.0, -0.0045), rpy=(0.0, 0.0, 0.0)),
        material=trim_elastomer,
        name="upper_secondary_ear",
    )
    hinge_arm.inertial = Inertial.from_geometry(
        Box((0.098, 0.022, 0.026)),
        mass=0.17,
        origin=Origin(xyz=(0.049, 0.0, -0.006)),
    )

    visor_panel = model.part("visor_panel")
    visor_panel.visual(
        Cylinder(radius=0.0075, length=0.006),
        material=trim_elastomer,
        name="secondary_boss",
    )
    visor_panel.visual(
        Box((0.022, 0.044, 0.012)),
        origin=Origin(xyz=(0.011, 0.0, 0.0)),
        material=trim_elastomer,
        name="pivot_gusset",
    )
    visor_panel.visual(
        mesh_from_geometry(_build_visor_body_mesh(), "visor_panel_body"),
        material=soft_polymer,
        name="visor_body",
    )
    visor_panel.visual(
        Box((0.228, 0.042, 0.0024)),
        origin=Origin(xyz=(0.192, -0.033, -0.0087)),
        material=trim_elastomer,
        name="grip_pad",
    )
    visor_panel.inertial = Inertial.from_geometry(
        Box((0.340, 0.152, 0.020)),
        mass=0.54,
        origin=Origin(xyz=(0.170, 0.0, 0.0)),
    )

    model.articulation(
        "roof_mount_to_hinge_arm",
        ArticulationType.REVOLUTE,
        parent=roof_mount,
        child=hinge_arm,
        origin=Origin(),
        axis=(0.0, 1.0, 0.0),
        motion_limits=MotionLimits(
            effort=12.0,
            velocity=2.5,
            lower=0.0,
            upper=math.radians(88.0),
        ),
    )
    model.articulation(
        "hinge_arm_to_visor_panel",
        ArticulationType.REVOLUTE,
        parent=hinge_arm,
        child=visor_panel,
        origin=Origin(xyz=(0.089, 0.0, -0.010)),
        axis=(0.0, 0.0, 1.0),
        motion_limits=MotionLimits(
            effort=6.0,
            velocity=2.8,
            lower=math.radians(-70.0),
            upper=math.radians(70.0),
        ),
    )

    return model


def run_tests() -> TestReport:
    ctx = TestContext(object_model)
    roof_mount = object_model.get_part("roof_mount")
    hinge_arm = object_model.get_part("hinge_arm")
    visor_panel = object_model.get_part("visor_panel")
    primary = object_model.get_articulation("roof_mount_to_hinge_arm")
    secondary = object_model.get_articulation("hinge_arm_to_visor_panel")

    def _aabb_center(aabb):
        if aabb is None:
            return None
        lower, upper = aabb
        return tuple((lower[i] + upper[i]) * 0.5 for i in range(3))

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
        "primary_axis_is_transverse",
        primary.axis == (0.0, 1.0, 0.0),
        f"expected primary axis (0, 1, 0), got {primary.axis}",
    )
    ctx.check(
        "secondary_axis_is_visible_swing_axis",
        secondary.axis == (0.0, 0.0, 1.0),
        f"expected secondary axis (0, 0, 1), got {secondary.axis}",
    )
    ctx.expect_contact(roof_mount, hinge_arm, name="mount_and_arm_touch_at_primary_pivot")
    ctx.expect_contact(hinge_arm, visor_panel, name="arm_and_panel_touch_at_secondary_pivot")
    ctx.expect_gap(
        roof_mount,
        visor_panel,
        axis="z",
        min_gap=0.001,
        positive_elem="roof_plate",
        negative_elem="visor_body",
        name="stowed_panel_sits_below_roof_mount",
    )

    visor_aabb = ctx.part_world_aabb(visor_panel)
    if visor_aabb is not None:
        lower, upper = visor_aabb
        visor_dims = tuple(upper[i] - lower[i] for i in range(3))
        ctx.check(
            "visor_has_consumer_scale_proportions",
            0.32 <= visor_dims[0] <= 0.36 and 0.13 <= visor_dims[1] <= 0.17 and 0.016 <= visor_dims[2] <= 0.028,
            f"unexpected visor extents {visor_dims}",
        )
    else:
        ctx.fail("visor_aabb_available", "visor panel world AABB could not be computed")

    with ctx.pose({primary: 0.0}):
        stowed_origin = ctx.part_world_position(visor_panel)
    with ctx.pose({primary: math.radians(78.0)}):
        lowered_origin = ctx.part_world_position(visor_panel)
    ctx.check(
        "primary_pivot_lowers_panel",
        stowed_origin is not None
        and lowered_origin is not None
        and lowered_origin[2] < stowed_origin[2] - 0.06,
        f"stowed={stowed_origin}, lowered={lowered_origin}",
    )

    with ctx.pose({secondary: 0.0}):
        straight_center = _aabb_center(ctx.part_element_world_aabb(visor_panel, elem="visor_body"))
    with ctx.pose({secondary: math.radians(55.0)}):
        swung_center = _aabb_center(ctx.part_element_world_aabb(visor_panel, elem="visor_body"))
    ctx.check(
        "secondary_pivot_swings_panel_sideways",
        straight_center is not None
        and swung_center is not None
        and swung_center[1] > straight_center[1] + 0.09,
        f"straight={straight_center}, swung={swung_center}",
    )

    return ctx.report()


# >>> USER_CODE_END

object_model = build_object_model()
