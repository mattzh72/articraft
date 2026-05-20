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
    MotionLimits,
    Origin,
    TestContext,
    TestReport,
    mesh_from_geometry,
    rounded_rect_profile,
)


def _rounded_panel_mesh(
    name: str,
    *,
    width: float,
    height: float,
    thickness: float,
    corner_radius: float,
    center: tuple[float, float, float],
):
    geom = ExtrudeGeometry.centered(
        rounded_rect_profile(
            width,
            height,
            corner_radius,
            corner_segments=8,
        ),
        thickness,
        cap=True,
        closed=True,
    )
    geom.translate(*center)
    return mesh_from_geometry(geom, name)


def build_object_model() -> ArticulatedObject:
    model = ArticulatedObject(name="windshield_sun_visor")

    powder_coat = model.material("powder_coat", rgba=(0.20, 0.22, 0.24, 1.0))
    mount_gray = model.material("mount_gray", rgba=(0.34, 0.36, 0.39, 1.0))
    service_gray = model.material("service_gray", rgba=(0.56, 0.58, 0.60, 1.0))
    bushing_bronze = model.material("bushing_bronze", rgba=(0.66, 0.54, 0.34, 1.0))
    fastener_steel = model.material("fastener_steel", rgba=(0.72, 0.74, 0.76, 1.0))
    visor_skin = model.material("visor_skin", rgba=(0.34, 0.37, 0.30, 1.0))
    rubber_black = model.material("rubber_black", rgba=(0.08, 0.08, 0.09, 1.0))

    roof_mount = model.part("roof_mount")
    roof_mount.visual(
        Box((0.170, 0.080, 0.012)),
        origin=Origin(xyz=(-0.035, 0.000, 0.026)),
        material=mount_gray,
        name="roof_plate",
    )
    roof_mount.visual(
        Box((0.030, 0.058, 0.020)),
        origin=Origin(xyz=(-0.030, 0.000, 0.010)),
        material=mount_gray,
        name="service_block",
    )
    roof_mount.visual(
        Box((0.060, 0.028, 0.010)),
        origin=Origin(xyz=(-0.065, 0.000, 0.015)),
        material=service_gray,
        name="access_cover",
    )
    roof_mount.visual(
        Box((0.030, 0.014, 0.040)),
        origin=Origin(xyz=(0.000, 0.030, 0.000)),
        material=mount_gray,
        name="left_bearing_tower",
    )
    roof_mount.visual(
        Box((0.030, 0.014, 0.040)),
        origin=Origin(xyz=(0.000, -0.030, 0.000)),
        material=mount_gray,
        name="right_bearing_tower",
    )
    for bolt_index, (bolt_x, bolt_y) in enumerate(
        ((-0.080, -0.025), (-0.080, 0.025), (0.005, -0.025), (0.005, 0.025))
    ):
        roof_mount.visual(
            Cylinder(radius=0.007, length=0.010),
            origin=Origin(xyz=(bolt_x, bolt_y, 0.037)),
            material=fastener_steel,
            name=f"mount_bolt_{bolt_index}",
        )

    primary_arm = model.part("primary_arm")
    primary_arm.visual(
        Cylinder(radius=0.010, length=0.034),
        origin=Origin(rpy=(math.pi / 2.0, 0.0, 0.0)),
        material=powder_coat,
        name="hinge_barrel",
    )
    primary_arm.visual(
        Cylinder(radius=0.014, length=0.006),
        origin=Origin(xyz=(0.000, 0.020, 0.000), rpy=(math.pi / 2.0, 0.0, 0.0)),
        material=bushing_bronze,
        name="left_thrust_flange",
    )
    primary_arm.visual(
        Cylinder(radius=0.014, length=0.006),
        origin=Origin(xyz=(0.000, -0.020, 0.000), rpy=(math.pi / 2.0, 0.0, 0.0)),
        material=bushing_bronze,
        name="right_thrust_flange",
    )
    primary_arm.visual(
        Box((0.088, 0.032, 0.014)),
        origin=Origin(xyz=(0.052, 0.000, -0.013)),
        material=powder_coat,
        name="arm_beam",
    )
    primary_arm.visual(
        Box((0.030, 0.028, 0.018)),
        origin=Origin(xyz=(0.022, 0.000, -0.006)),
        material=service_gray,
        name="root_gusset",
    )
    primary_arm.visual(
        Box((0.018, 0.032, 0.020)),
        origin=Origin(xyz=(0.082, 0.000, -0.010)),
        material=service_gray,
        name="service_lug",
    )
    primary_arm.visual(
        Box((0.008, 0.048, 0.024)),
        origin=Origin(xyz=(0.092, 0.000, -0.010)),
        material=powder_coat,
        name="front_bearing_ear",
    )
    primary_arm.visual(
        Box((0.008, 0.048, 0.024)),
        origin=Origin(xyz=(0.120, 0.000, -0.010)),
        material=powder_coat,
        name="rear_bearing_ear",
    )
    primary_arm.visual(
        Box((0.040, 0.048, 0.008)),
        origin=Origin(xyz=(0.106, 0.000, 0.002)),
        material=service_gray,
        name="bearing_bridge",
    )
    primary_arm.visual(
        Box((0.020, 0.048, 0.006)),
        origin=Origin(xyz=(0.106, 0.000, 0.007)),
        material=fastener_steel,
        name="bearing_cap",
    )

    visor_panel = model.part("visor_panel")
    visor_panel.visual(
        Cylinder(radius=0.009, length=0.012),
        origin=Origin(rpy=(0.0, math.pi / 2.0, 0.0)),
        material=powder_coat,
        name="spindle_core",
    )
    visor_panel.visual(
        Cylinder(radius=0.013, length=0.004),
        origin=Origin(xyz=(-0.008, 0.000, 0.000), rpy=(0.0, math.pi / 2.0, 0.0)),
        material=bushing_bronze,
        name="front_spindle_flange",
    )
    visor_panel.visual(
        Cylinder(radius=0.013, length=0.004),
        origin=Origin(xyz=(0.008, 0.000, 0.000), rpy=(0.0, math.pi / 2.0, 0.0)),
        material=bushing_bronze,
        name="rear_spindle_flange",
    )
    visor_panel.visual(
        Box((0.020, 0.054, 0.010)),
        origin=Origin(xyz=(0.000, 0.022, -0.006)),
        material=fastener_steel,
        name="pivot_clamp_cap",
    )
    visor_panel.visual(
        Box((0.016, 0.030, 0.022)),
        origin=Origin(xyz=(0.000, 0.038, -0.016)),
        material=service_gray,
        name="knuckle_body",
    )
    visor_panel.visual(
        Box((0.086, 0.042, 0.018)),
        origin=Origin(xyz=(0.070, 0.092, -0.030)),
        material=service_gray,
        name="service_bracket",
    )
    visor_panel.visual(
        Box((0.030, 0.040, 0.024)),
        origin=Origin(xyz=(0.020, 0.060, -0.022)),
        material=service_gray,
        name="service_boss",
    )
    visor_panel.visual(
        Box((0.168, 0.390, 0.024)),
        origin=Origin(xyz=(0.118, 0.214, -0.040)),
        material=visor_skin,
        name="visor_shell",
    )
    visor_panel.visual(
        Box((0.140, 0.338, 0.006)),
        origin=Origin(xyz=(0.120, 0.214, -0.026)),
        material=service_gray,
        name="service_cover",
    )
    visor_panel.visual(
        Box((0.132, 0.312, 0.006)),
        origin=Origin(xyz=(0.124, 0.214, -0.055)),
        material=rubber_black,
        name="rear_wear_pad",
    )
    visor_panel.visual(
        Box((0.040, 0.124, 0.010)),
        origin=Origin(xyz=(0.046, 0.272, -0.046)),
        material=rubber_black,
        name="bumper_strip",
    )
    visor_panel.visual(
        Cylinder(radius=0.005, length=0.006),
        origin=Origin(xyz=(0.070, 0.100, -0.022), rpy=(0.0, math.pi / 2.0, 0.0)),
        material=fastener_steel,
        name="cover_fastener_front",
    )
    visor_panel.visual(
        Cylinder(radius=0.005, length=0.006),
        origin=Origin(xyz=(0.070, 0.300, -0.022), rpy=(0.0, math.pi / 2.0, 0.0)),
        material=fastener_steel,
        name="cover_fastener_rear",
    )

    model.articulation(
        "roof_to_primary_hinge",
        ArticulationType.REVOLUTE,
        parent=roof_mount,
        child=primary_arm,
        origin=Origin(),
        axis=(0.0, 1.0, 0.0),
        motion_limits=MotionLimits(
            effort=25.0,
            velocity=1.6,
            lower=0.0,
            upper=1.35,
        ),
    )
    model.articulation(
        "primary_to_visor_swing",
        ArticulationType.REVOLUTE,
        parent=primary_arm,
        child=visor_panel,
        origin=Origin(xyz=(0.106, 0.000, -0.010)),
        axis=(1.0, 0.0, 0.0),
        motion_limits=MotionLimits(
            effort=12.0,
            velocity=1.5,
            lower=-1.05,
            upper=1.05,
        ),
    )

    return model


def run_tests() -> TestReport:
    ctx = TestContext(object_model)
    roof_mount = object_model.get_part("roof_mount")
    primary_arm = object_model.get_part("primary_arm")
    visor_panel = object_model.get_part("visor_panel")
    primary_hinge = object_model.get_articulation("roof_to_primary_hinge")
    secondary_pivot = object_model.get_articulation("primary_to_visor_swing")

    ctx.check_model_valid()
    ctx.check_mesh_assets_ready()

    # Preferred default QC stack:
    # 1) likely-failure grounded-component floating check for disconnected part groups
    ctx.fail_if_isolated_parts()
    # 2) noisier warning-tier sensor for same-part disconnected geometry islands
    ctx.warn_if_part_contains_disconnected_geometry_islands()
    ctx.fail_if_part_contains_disconnected_geometry_islands(
        name="all visor parts are internally connected"
    )
    # 3) likely-failure rest-pose part-to-part overlap backstop for real 3D interpenetration
    # This is not an "inside / nested / footprint overlap" check.
    # Investigate all three. Warning-tier signals are not free passes.
    # Use `ctx.allow_overlap(...)` only for true intended penetration.
    # If parts are nested but should remain clear, prove that with exact
    # `expect_contact(...)`, `expect_gap(...)`, `expect_overlap(...)`, or
    # `expect_within(...)` checks instead.
    ctx.fail_if_parts_overlap_in_current_pose()

    ctx.expect_contact(
        primary_arm,
        roof_mount,
        elem_a="left_thrust_flange",
        elem_b="left_bearing_tower",
        contact_tol=0.0005,
        name="left primary hinge flange seats on mount tower",
    )
    ctx.expect_contact(
        primary_arm,
        roof_mount,
        elem_a="right_thrust_flange",
        elem_b="right_bearing_tower",
        contact_tol=0.0005,
        name="right primary hinge flange seats on mount tower",
    )
    ctx.expect_contact(
        visor_panel,
        primary_arm,
        elem_a="front_spindle_flange",
        elem_b="front_bearing_ear",
        contact_tol=0.0005,
        name="front swing flange seats on bearing ear",
    )
    ctx.expect_contact(
        visor_panel,
        primary_arm,
        elem_a="rear_spindle_flange",
        elem_b="rear_bearing_ear",
        contact_tol=0.0005,
        name="rear swing flange seats on bearing ear",
    )

    with ctx.pose({primary_hinge: 0.0, secondary_pivot: 0.0}):
        ctx.expect_gap(
            roof_mount,
            visor_panel,
            axis="z",
            positive_elem="roof_plate",
            negative_elem="visor_shell",
            min_gap=0.045,
            max_gap=0.085,
            name="stowed visor shell clears the roof plate",
        )
        ctx.expect_gap(
            primary_arm,
            visor_panel,
            axis="z",
            positive_elem="arm_beam",
            negative_elem="visor_shell",
            min_gap=0.006,
            max_gap=0.030,
            name="stowed visor shell clears the arm beam",
        )

    with ctx.pose({primary_hinge: 1.30, secondary_pivot: 0.0}):
        ctx.expect_gap(
            roof_mount,
            visor_panel,
            axis="z",
            positive_elem="roof_plate",
            negative_elem="visor_shell",
            min_gap=0.145,
            name="deployed visor drops well below the roof mount",
        )

    stowed_shell = None
    dropped_shell = None
    centered_shell = None
    swung_shell = None
    with ctx.pose({primary_hinge: 0.0, secondary_pivot: 0.0}):
        stowed_shell = ctx.part_element_world_aabb(visor_panel, elem="visor_shell")
    with ctx.pose({primary_hinge: 1.30, secondary_pivot: 0.0}):
        dropped_shell = ctx.part_element_world_aabb(visor_panel, elem="visor_shell")
        centered_shell = ctx.part_element_world_aabb(visor_panel, elem="visor_shell")
    with ctx.pose({primary_hinge: 1.30, secondary_pivot: 0.85}):
        swung_shell = ctx.part_element_world_aabb(visor_panel, elem="visor_shell")

    if stowed_shell is not None and dropped_shell is not None:
        stowed_center_z = (stowed_shell[0][2] + stowed_shell[1][2]) * 0.5
        dropped_center_z = (dropped_shell[0][2] + dropped_shell[1][2]) * 0.5
        ctx.check(
            "primary hinge lowers the visor shell",
            dropped_center_z < stowed_center_z - 0.120,
            (
                f"expected deployed shell center to drop by at least 0.120 m, "
                f"got stowed z={stowed_center_z:.4f}, deployed z={dropped_center_z:.4f}"
            ),
        )

    if centered_shell is not None and swung_shell is not None:
        centered_x = (centered_shell[0][0] + centered_shell[1][0]) * 0.5
        centered_z = (centered_shell[0][2] + centered_shell[1][2]) * 0.5
        swung_x = (swung_shell[0][0] + swung_shell[1][0]) * 0.5
        swung_z = (swung_shell[0][2] + swung_shell[1][2]) * 0.5
        ctx.check(
            "secondary pivot repositions the visor sideways",
            abs(swung_x - centered_x) > 0.120,
            (
                f"expected at least 0.120 m sideways shell shift, "
                f"got centered x={centered_x:.4f}, swung x={swung_x:.4f}"
            ),
        )
        ctx.check(
            "secondary pivot stays on a near-vertical service spindle",
            abs(swung_z - centered_z) < 0.050,
            (
                f"expected swing to stay near the same deployment height, "
                f"got centered z={centered_z:.4f}, swung z={swung_z:.4f}"
            ),
        )

    return ctx.report()


# >>> USER_CODE_END

object_model = build_object_model()
