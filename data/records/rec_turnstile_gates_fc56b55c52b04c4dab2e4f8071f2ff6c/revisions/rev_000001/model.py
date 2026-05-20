from __future__ import annotations

# User code should import every SDK/stdlib symbol it uses instead of relying on
# hidden scaffold imports.
# >>> USER_CODE_START
from math import cos, pi, sin

from sdk import (
    ArticulatedObject,
    ArticulationType,
    Cylinder,
    LatheGeometry,
    MotionLimits,
    Origin,
    Sphere,
    TestContext,
    TestReport,
    mesh_from_geometry,
)


def build_object_model() -> ArticulatedObject:
    model = ArticulatedObject(name="premium_tripod_turnstile")

    painted_metal = model.material(
        "painted_metal",
        rgba=(0.23, 0.25, 0.28, 1.0),
    )
    satin_steel = model.material(
        "satin_steel",
        rgba=(0.78, 0.80, 0.82, 1.0),
    )
    polymer_black = model.material(
        "polymer_black",
        rgba=(0.11, 0.12, 0.13, 1.0),
    )
    elastomer = model.material(
        "elastomer",
        rgba=(0.06, 0.06, 0.07, 1.0),
    )

    pedestal = model.part("pedestal")
    bearing_core = model.part("bearing_core")
    rotor = model.part("rotor")

    pedestal_profile = [
        (0.0, 0.0),
        (0.145, 0.0),
        (0.145, 0.010),
        (0.118, 0.026),
        (0.098, 0.070),
        (0.086, 0.760),
        (0.094, 0.836),
        (0.076, 0.872),
        (0.076, 0.889),
        (0.0, 0.889),
    ]
    pedestal.visual(
        mesh_from_geometry(LatheGeometry(pedestal_profile, segments=72), "pedestal_shell"),
        material=painted_metal,
        name="pedestal_shell",
    )
    pedestal.visual(
        Cylinder(radius=0.079, length=0.010),
        origin=Origin(xyz=(0.0, 0.0, 0.884)),
        material=polymer_black,
        name="top_bezel",
    )
    pedestal.visual(
        mesh_from_geometry(
            LatheGeometry.from_shell_profiles(
                [
                    (0.100, 0.181),
                    (0.1015, 0.186),
                    (0.1015, 0.194),
                    (0.100, 0.199),
                ],
                [
                    (0.0950, 0.181),
                    (0.0944, 0.190),
                    (0.0950, 0.199),
                ],
                segments=72,
                start_cap="flat",
                end_cap="flat",
                lip_samples=6,
            ),
            "service_band_mesh",
        ),
        material=polymer_black,
        name="service_band",
    )
    pedestal.visual(
        mesh_from_geometry(
            LatheGeometry.from_shell_profiles(
                [
                    (0.146, 0.000),
                    (0.146, 0.006),
                    (0.143, 0.014),
                    (0.140, 0.020),
                ],
                [
                    (0.138, 0.000),
                    (0.138, 0.008),
                    (0.136, 0.014),
                    (0.134, 0.020),
                ],
                segments=72,
                start_cap="flat",
                end_cap="flat",
                lip_samples=6,
            ),
            "base_foot_ring_mesh",
        ),
        material=elastomer,
        name="base_foot_ring",
    )

    bearing_core.visual(
        Cylinder(radius=0.056, length=0.010),
        origin=Origin(xyz=(0.0, 0.0, -0.046)),
        material=satin_steel,
        name="thrust_washer",
    )
    bearing_core.visual(
        Cylinder(radius=0.037, length=0.070),
        origin=Origin(xyz=(0.0, 0.0, -0.006)),
        material=polymer_black,
        name="bearing_barrel",
    )
    bearing_core.visual(
        Cylinder(radius=0.012, length=0.100),
        origin=Origin(xyz=(0.0, 0.0, 0.0)),
        material=satin_steel,
        name="spindle",
    )
    bearing_core.visual(
        Sphere(radius=0.014),
        origin=Origin(xyz=(0.0, 0.0, 0.050)),
        material=satin_steel,
        name="spindle_cap",
    )

    hub_outer_profile = [
        (0.058, -0.041),
        (0.061, -0.032),
        (0.064, -0.006),
        (0.064, 0.006),
        (0.061, 0.032),
        (0.058, 0.041),
    ]
    hub_inner_profile = [
        (0.041, -0.041),
        (0.041, -0.010),
        (0.043, 0.020),
        (0.045, 0.041),
    ]
    rotor.visual(
        mesh_from_geometry(
            LatheGeometry.from_shell_profiles(
                hub_outer_profile,
                hub_inner_profile,
                segments=72,
                start_cap="flat",
                end_cap="flat",
                lip_samples=8,
            ),
            "rotor_hub",
        ),
        material=painted_metal,
        name="hub_shell",
    )

    arm_outer_radius = 0.064
    arm_length = 0.430
    arm_radius = 0.011
    root_collar_length = 0.050
    grip_length = 0.140
    grip_radius = 0.017
    tip_radius = 0.017
    root_center_radius = arm_outer_radius + (root_collar_length * 0.5) - 0.006
    arm_center_radius = arm_outer_radius + (arm_length * 0.5) - 0.006
    grip_center_radius = arm_outer_radius + arm_length - (grip_length * 0.5) - 0.012
    tip_center_radius = arm_outer_radius + arm_length + tip_radius - 0.008

    for index, angle in enumerate((0.0, 2.0 * pi / 3.0, 4.0 * pi / 3.0), start=1):
        ux = cos(angle)
        uy = sin(angle)
        arm_rpy = (0.0, pi / 2.0, angle)

        rotor.visual(
            Cylinder(radius=0.015, length=root_collar_length),
            origin=Origin(
                xyz=(ux * root_center_radius, uy * root_center_radius, 0.0),
                rpy=arm_rpy,
            ),
            material=painted_metal,
            name=f"arm_root_{index}",
        )
        rotor.visual(
            Cylinder(radius=arm_radius, length=arm_length),
            origin=Origin(
                xyz=(ux * arm_center_radius, uy * arm_center_radius, 0.0),
                rpy=arm_rpy,
            ),
            material=satin_steel,
            name=f"arm_tube_{index}",
        )
        rotor.visual(
            Cylinder(radius=grip_radius, length=grip_length),
            origin=Origin(
                xyz=(ux * grip_center_radius, uy * grip_center_radius, 0.0),
                rpy=arm_rpy,
            ),
            material=polymer_black,
            name=f"grip_{index}",
        )
        rotor.visual(
            Sphere(radius=tip_radius),
            origin=Origin(xyz=(ux * tip_center_radius, uy * tip_center_radius, 0.0)),
            material=elastomer,
            name=f"tip_{index}",
        )

    model.articulation(
        "pedestal_to_bearing_core",
        ArticulationType.FIXED,
        parent=pedestal,
        child=bearing_core,
        origin=Origin(xyz=(0.0, 0.0, 0.940)),
    )
    model.articulation(
        "bearing_core_to_rotor",
        ArticulationType.CONTINUOUS,
        parent=bearing_core,
        child=rotor,
        origin=Origin(),
        axis=(0.0, 0.0, 1.0),
        motion_limits=MotionLimits(effort=30.0, velocity=4.0),
    )

    return model


def run_tests() -> TestReport:
    ctx = TestContext(object_model)
    pedestal = object_model.get_part("pedestal")
    bearing_core = object_model.get_part("bearing_core")
    rotor = object_model.get_part("rotor")
    rotor_joint = object_model.get_articulation("bearing_core_to_rotor")

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
        pedestal,
        bearing_core,
        elem_a="pedestal_shell",
        elem_b="thrust_washer",
        name="bearing_core_is_seated_on_pedestal",
    )
    ctx.expect_contact(
        rotor,
        bearing_core,
        elem_a="hub_shell",
        elem_b="thrust_washer",
        name="rotor_is_supported_by_thrust_washer",
    )
    ctx.expect_gap(
        rotor,
        pedestal,
        axis="z",
        min_gap=0.008,
        name="rotor_clears_pedestal_top",
    )
    ctx.expect_origin_distance(
        rotor,
        bearing_core,
        axes="xy",
        min_dist=0.0,
        max_dist=0.0001,
        name="rotor_stays_concentric_with_spindle",
    )

    with ctx.pose({rotor_joint: 2.0 * pi / 3.0}):
        ctx.expect_origin_distance(
            rotor,
            bearing_core,
            axes="xy",
            min_dist=0.0,
            max_dist=0.0001,
            name="rotor_remains_concentric_when_indexed",
        )
        ctx.expect_gap(
            rotor,
            pedestal,
            axis="z",
            min_gap=0.008,
            name="indexed_rotor_still_clears_pedestal",
        )

    return ctx.report()


# >>> USER_CODE_END

object_model = build_object_model()
