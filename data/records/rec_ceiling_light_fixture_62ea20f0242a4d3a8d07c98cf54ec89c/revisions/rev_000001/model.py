from __future__ import annotations

# User code should import every SDK/stdlib symbol it uses instead of relying on
# hidden scaffold imports.
# >>> USER_CODE_START
from math import cos, pi, radians, sin

from sdk import (
    ArticulatedObject,
    ArticulationType,
    Box,
    Cylinder,
    Inertial,
    LatheGeometry,
    MotionLimits,
    Origin,
    Sphere,
    TestContext,
    TestReport,
    mesh_from_geometry,
    tube_from_spline_points,
)


CANOPY_RADIUS = 0.108
CANOPY_HINGE_RADIUS = 0.104
HINGE_Z = -0.039
ARM_TIP = (0.278, 0.0, -0.084)
ARM_TUBE_END = (0.244, 0.0, -0.074)


def _save_mesh(name: str, geometry):
    return mesh_from_geometry(geometry, name)


def _build_canopy_skirt():
    outer_profile = [
        (0.086, -0.040),
        (0.094, -0.039),
        (0.102, -0.030),
        (0.106, -0.016),
        (0.108, -0.004),
        (0.108, 0.0),
    ]
    inner_profile = [
        (0.074, -0.036),
        (0.084, -0.035),
        (0.094, -0.027),
        (0.099, -0.015),
        (0.101, -0.004),
    ]
    return LatheGeometry.from_shell_profiles(outer_profile, inner_profile, segments=72)


def _build_globe_shell():
    outer_profile = [
        (0.003, -0.073),
        (0.030, -0.070),
        (0.055, -0.047),
        (0.071, 0.0),
        (0.063, 0.047),
        (0.039, 0.073),
        (0.022, 0.088),
        (0.018, 0.094),
    ]
    inner_profile = [
        (0.0, -0.068),
        (0.024, -0.064),
        (0.047, -0.045),
        (0.064, 0.0),
        (0.056, 0.044),
        (0.034, 0.069),
        (0.019, 0.083),
        (0.014, 0.089),
    ]
    return LatheGeometry.from_shell_profiles(outer_profile, inner_profile, segments=64)


def _build_branch_arm():
    return tube_from_spline_points(
        [
            (0.010, 0.0, -0.002),
            (0.060, 0.0, -0.006),
            (0.138, 0.0, -0.026),
            (0.220, 0.0, -0.058),
            ARM_TUBE_END,
        ],
        radius=0.011,
        samples_per_segment=18,
        radial_segments=20,
        cap_ends=True,
    )


def _add_arm_geometry(part, arm_mesh, brass, accent) -> None:
    part.visual(
        Sphere(radius=0.0085),
        origin=Origin(xyz=(0.0085, 0.0, -0.004)),
        material=accent,
        name="hinge_knuckle",
    )
    part.visual(
        Box((0.016, 0.016, 0.014)),
        origin=Origin(xyz=(0.018, 0.0, -0.005)),
        material=accent,
        name="hinge_neck",
    )
    part.visual(arm_mesh, material=brass, name="arm_tube")
    part.visual(
        Cylinder(radius=0.012, length=0.018),
        origin=Origin(xyz=(0.244, 0.0, -0.078), rpy=(0.0, pi / 2.0, 0.0)),
        material=brass,
        name="tip_collar",
    )
    part.visual(
        Box((0.036, 0.022, 0.016)),
        origin=Origin(xyz=(0.260, 0.0, -0.082)),
        material=brass,
        name="shade_hinge_block",
    )
    part.inertial = Inertial.from_geometry(
        Box((0.31, 0.06, 0.15)),
        mass=1.2,
        origin=Origin(xyz=(0.150, 0.0, -0.045)),
    )


def _add_shade_geometry(part, globe_mesh, brass, glass, accent) -> None:
    part.visual(
        Box((0.012, 0.018, 0.024)),
        origin=Origin(xyz=(0.006, 0.0, -0.006)),
        material=accent,
        name="hinge_blade",
    )
    part.visual(
        Box((0.032, 0.020, 0.018)),
        origin=Origin(xyz=(0.018, 0.0, -0.016)),
        material=brass,
        name="top_cap",
    )
    part.visual(
        Cylinder(radius=0.022, length=0.065),
        origin=Origin(xyz=(0.026, 0.0, -0.056)),
        material=brass,
        name="shade_socket",
    )
    part.visual(
        globe_mesh,
        origin=Origin(xyz=(0.026, 0.0, -0.115)),
        material=glass,
        name="globe_shell",
    )
    part.inertial = Inertial.from_geometry(
        Box((0.16, 0.16, 0.19)),
        mass=0.8,
        origin=Origin(xyz=(0.026, 0.0, -0.094)),
    )


def _center_from_aabb(aabb):
    if aabb is None:
        return None
    mn, mx = aabb
    return tuple((mn[i] + mx[i]) * 0.5 for i in range(3))


def _radial_xyz(radius: float, angle: float, z: float) -> tuple[float, float, float]:
    return (radius * cos(angle), radius * sin(angle), z)


def _tangent_offset(angle: float, amount: float) -> tuple[float, float, float]:
    return (-sin(angle) * amount, cos(angle) * amount, 0.0)


def build_object_model() -> ArticulatedObject:
    model = ArticulatedObject(name="semi_flush_branching_ceiling_light")

    brass = model.material("brushed_brass", rgba=(0.71, 0.60, 0.36, 1.0))
    accent = model.material("satin_black", rgba=(0.17, 0.17, 0.18, 1.0))
    glass = model.material("opal_glass", rgba=(0.94, 0.93, 0.90, 0.92))

    canopy_skirt_mesh = _save_mesh("canopy_skirt", _build_canopy_skirt())
    branch_arm_mesh = _save_mesh("branch_arm", _build_branch_arm())
    globe_shell_mesh = _save_mesh("globe_shade_shell", _build_globe_shell())

    canopy = model.part("canopy")
    canopy.visual(
        Cylinder(radius=CANOPY_RADIUS, length=0.004),
        origin=Origin(xyz=(0.0, 0.0, -0.002)),
        material=brass,
        name="canopy_plate",
    )
    canopy.visual(canopy_skirt_mesh, material=brass, name="canopy_skirt")
    canopy.visual(
        Cylinder(radius=0.040, length=0.028),
        origin=Origin(xyz=(0.0, 0.0, -0.018)),
        material=brass,
        name="center_hub",
    )
    canopy.visual(
        Cylinder(radius=0.022, length=0.014),
        origin=Origin(xyz=(0.0, 0.0, -0.037)),
        material=accent,
        name="hub_boss",
    )
    for index, angle in enumerate((0.0, 2.0 * pi / 3.0, 4.0 * pi / 3.0), start=1):
        pedestal_xyz = _radial_xyz(CANOPY_HINGE_RADIUS - 0.006, angle, -0.039)
        canopy.visual(
            Box((0.012, 0.022, 0.018)),
            origin=Origin(
                xyz=pedestal_xyz,
                rpy=(0.0, 0.0, angle),
            ),
            material=brass,
            name=f"branch_{index}_pedestal",
        )
    canopy.inertial = Inertial.from_geometry(
        Box((0.26, 0.26, 0.08)),
        mass=4.5,
        origin=Origin(xyz=(0.0, 0.0, -0.024)),
    )

    for index, angle in enumerate((0.0, 2.0 * pi / 3.0, 4.0 * pi / 3.0), start=1):
        arm = model.part(f"arm_{index}")
        _add_arm_geometry(arm, branch_arm_mesh, brass, accent)

        shade = model.part(f"shade_{index}")
        _add_shade_geometry(shade, globe_shell_mesh, brass, glass, accent)

        model.articulation(
            f"canopy_to_arm_{index}",
            ArticulationType.REVOLUTE,
            parent=canopy,
            child=arm,
            origin=Origin(xyz=_radial_xyz(CANOPY_HINGE_RADIUS, angle, HINGE_Z), rpy=(0.0, 0.0, angle)),
            axis=(0.0, -1.0, 0.0),
            motion_limits=MotionLimits(
                effort=16.0,
                velocity=1.2,
                lower=radians(-22.0),
                upper=radians(38.0),
            ),
        )
        model.articulation(
            f"arm_{index}_to_shade_{index}",
            ArticulationType.REVOLUTE,
            parent=arm,
            child=shade,
            origin=Origin(xyz=ARM_TIP),
            axis=(0.0, 1.0, 0.0),
            motion_limits=MotionLimits(
                effort=6.0,
                velocity=1.6,
                lower=radians(-45.0),
                upper=radians(45.0),
            ),
        )

    return model


def run_tests() -> TestReport:
    ctx = TestContext(object_model)
    canopy = object_model.get_part("canopy")
    arms = [object_model.get_part(f"arm_{index}") for index in range(1, 4)]
    shades = [object_model.get_part(f"shade_{index}") for index in range(1, 4)]
    arm_joints = [object_model.get_articulation(f"canopy_to_arm_{index}") for index in range(1, 4)]
    shade_joints = [object_model.get_articulation(f"arm_{index}_to_shade_{index}") for index in range(1, 4)]

    ctx.check_model_valid()
    ctx.check_mesh_assets_ready()

    for index, arm in enumerate(arms, start=1):
        ctx.allow_overlap(
            arm,
            canopy,
            elem_a="hinge_knuckle",
            elem_b=f"branch_{index}_pedestal",
            reason="The branch hinge is represented as a simplified captured pivot, so the knuckle shares the pedestal pivot envelope.",
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

    ctx.check("three articulated branch arms are present", len(arms) == 3, details=f"arms={len(arms)}")
    ctx.check("three globe shades are present", len(shades) == 3, details=f"shades={len(shades)}")

    for arm_joint in arm_joints:
        ctx.check(
            f"{arm_joint.name} keeps a horizontal hinge axis",
            arm_joint.axis == (0.0, -1.0, 0.0),
            details=f"axis={arm_joint.axis}",
        )
    for shade_joint in shade_joints:
        ctx.check(
            f"{shade_joint.name} keeps a horizontal tilt axis",
            shade_joint.axis == (0.0, 1.0, 0.0),
            details=f"axis={shade_joint.axis}",
        )

    for index, arm in enumerate(arms, start=1):
        ctx.expect_contact(
            arm,
            canopy,
            elem_a="hinge_knuckle",
            elem_b=f"branch_{index}_pedestal",
            name=f"arm {index} hinge hardware contacts the canopy",
        )
    for index, (arm, shade) in enumerate(zip(arms, shades), start=1):
        ctx.expect_contact(
            shade,
            arm,
            elem_a="hinge_blade",
            elem_b="shade_hinge_block",
            name=f"shade {index} hinge blade seats in the arm block",
        )

    rest_shade_origin = ctx.part_world_position(shades[0])
    with ctx.pose({arm_joints[0]: radians(30.0)}):
        raised_shade_origin = ctx.part_world_position(shades[0])
        ctx.expect_origin_gap(
            shades[0],
            canopy,
            axis="z",
            min_gap=0.020,
            name="raised first branch keeps the globe elevated from the canopy",
        )
        ctx.expect_origin_gap(
            shades[0],
            canopy,
            axis="x",
            min_gap=0.22,
            name="raised first branch keeps the globe outboard of the canopy",
        )
    ctx.check(
        "positive arm motion raises the first branch",
        rest_shade_origin is not None
        and raised_shade_origin is not None
        and raised_shade_origin[2] > rest_shade_origin[2] + 0.03,
        details=f"rest={rest_shade_origin}, raised={raised_shade_origin}",
    )

    globe_center_rest = _center_from_aabb(ctx.part_element_world_aabb(shades[0], elem="globe_shell"))
    with ctx.pose({shade_joints[0]: radians(35.0)}):
        globe_center_tilted = _center_from_aabb(ctx.part_element_world_aabb(shades[0], elem="globe_shell"))
    ctx.check(
        "positive shade tilt changes the globe aim",
        globe_center_rest is not None
        and globe_center_tilted is not None
        and abs(globe_center_tilted[0] - globe_center_rest[0]) > 0.03,
        details=f"rest={globe_center_rest}, tilted={globe_center_tilted}",
    )

    return ctx.report()


# >>> USER_CODE_END

object_model = build_object_model()
