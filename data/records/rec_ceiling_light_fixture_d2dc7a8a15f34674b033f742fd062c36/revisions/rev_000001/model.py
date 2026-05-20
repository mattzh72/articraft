from __future__ import annotations

# User code should import every SDK/stdlib symbol it uses instead of relying on
# hidden scaffold imports.
# >>> USER_CODE_START
from math import cos, pi, sin

from sdk import (
    ArticulatedObject,
    ArticulationType,
    Box,
    Cylinder,
    Inertial,
    LatheGeometry,
    MotionLimits,
    Origin,
    TestContext,
    TestReport,
    TorusGeometry,
    mesh_from_geometry,
    tube_from_spline_points,
)


def _mesh(name: str, geometry):
    return mesh_from_geometry(geometry, name)


def _polar_xy(radius: float, angle: float) -> tuple[float, float]:
    return (radius * cos(angle), radius * sin(angle))


def build_object_model() -> ArticulatedObject:
    model = ArticulatedObject(name="pendant_chandelier")

    aged_brass = model.material("aged_brass", rgba=(0.66, 0.56, 0.34, 1.0))
    bronze_shadow = model.material("bronze_shadow", rgba=(0.24, 0.20, 0.16, 1.0))
    candle_ivory = model.material("candle_ivory", rgba=(0.95, 0.93, 0.84, 1.0))
    frosted_glass = model.material("frosted_glass", rgba=(0.95, 0.93, 0.88, 0.92))

    canopy_depth = 0.055
    stem_length = 0.555
    hub_z = -0.640
    hub_ring_radius = 0.095
    hub_ring_tube = 0.012
    hinge_radius = 0.116
    arm_tip_x = 0.332

    canopy_mesh = _mesh(
        "ceiling_canopy_shell",
        LatheGeometry.from_shell_profiles(
            outer_profile=[
                (0.018, 0.000),
                (0.055, -0.008),
                (0.075, -0.020),
                (0.064, -0.055),
            ],
            inner_profile=[
                (0.013, -0.003),
                (0.048, -0.011),
                (0.067, -0.021),
                (0.056, -0.053),
            ],
            segments=64,
        ),
    )
    hub_ring_mesh = _mesh(
        "hub_ring",
        TorusGeometry(radius=hub_ring_radius, tube=hub_ring_tube, radial_segments=18, tubular_segments=72),
    )
    arm_mesh = _mesh(
        "radial_arm_tube",
        tube_from_spline_points(
            [
                (0.012, 0.000, 0.000),
                (0.090, 0.000, 0.004),
                (0.195, 0.000, 0.018),
                (0.285, 0.000, 0.022),
                (0.322, 0.000, 0.016),
            ],
            radius=0.009,
            samples_per_segment=16,
            radial_segments=18,
            cap_ends=True,
        ),
    )
    shade_mesh = _mesh(
        "candle_socket_shade",
        LatheGeometry.from_shell_profiles(
            outer_profile=[
                (0.026, 0.018),
                (0.036, 0.005),
                (0.053, -0.030),
                (0.070, -0.074),
                (0.082, -0.110),
            ],
            inner_profile=[
                (0.021, 0.017),
                (0.031, 0.005),
                (0.049, -0.028),
                (0.066, -0.072),
                (0.080, -0.108),
            ],
            segments=56,
        ),
    )

    body = model.part("body")
    body.visual(canopy_mesh, material=aged_brass, name="canopy_shell")
    body.visual(
        Cylinder(radius=0.062, length=0.008),
        origin=Origin(xyz=(0.0, 0.0, -0.018)),
        material=bronze_shadow,
        name="canopy_mount_plate",
    )
    body.visual(
        Cylinder(radius=0.020, length=0.038),
        origin=Origin(xyz=(0.0, 0.0, -0.036)),
        material=aged_brass,
        name="stem_receiver",
    )
    body.visual(
        Cylinder(radius=0.015, length=0.022),
        origin=Origin(xyz=(0.0, 0.0, -0.054)),
        material=bronze_shadow,
        name="canopy_collar",
    )
    body.visual(
        Cylinder(radius=0.010, length=stem_length),
        origin=Origin(xyz=(0.0, 0.0, -0.3325)),
        material=aged_brass,
        name="down_stem",
    )
    body.visual(
        Cylinder(radius=0.023, length=0.060),
        origin=Origin(xyz=(0.0, 0.0, hub_z)),
        material=bronze_shadow,
        name="hub_collar",
    )
    body.visual(
        hub_ring_mesh,
        origin=Origin(xyz=(0.0, 0.0, hub_z)),
        material=aged_brass,
        name="hub_ring",
    )
    body.visual(
        Cylinder(radius=0.016, length=0.050),
        origin=Origin(xyz=(0.0, 0.0, hub_z - 0.045)),
        material=aged_brass,
        name="lower_finial",
    )

    for index in range(5):
        angle = 2.0 * pi * index / 5.0
        spoke_x, spoke_y = _polar_xy(0.060, angle)
        hanger_x, hanger_y = _polar_xy(hinge_radius, angle)
        body.visual(
            Box((0.074, 0.014, 0.020)),
            origin=Origin(xyz=(spoke_x, spoke_y, hub_z), rpy=(0.0, 0.0, angle)),
            material=aged_brass,
            name=f"hub_spoke_{index}",
        )
        body.visual(
            Box((0.018, 0.024, 0.028)),
            origin=Origin(xyz=(hanger_x, hanger_y, hub_z + 0.014), rpy=(0.0, 0.0, angle)),
            material=aged_brass,
            name=f"hub_hanger_{index}",
        )
        body.visual(
            Box((0.016, 0.014, 0.016)),
            origin=Origin(xyz=(_polar_xy(0.102, angle)[0], _polar_xy(0.102, angle)[1], hub_z + 0.010), rpy=(0.0, 0.0, angle)),
            material=bronze_shadow,
            name=f"hub_bracket_{index}",
        )

    body.inertial = Inertial.from_geometry(
        Box((0.34, 0.34, 0.74)),
        mass=6.5,
        origin=Origin(xyz=(0.0, 0.0, -0.370)),
    )

    for index in range(5):
        arm = model.part(f"arm_{index}")
        arm.visual(
            Box((0.020, 0.022, 0.020)),
            origin=Origin(xyz=(0.010, 0.0, -0.010)),
            material=aged_brass,
            name="root_mount",
        )
        arm.visual(arm_mesh, material=aged_brass, name="arm_tube")
        arm.visual(
            Cylinder(radius=0.031, length=0.004),
            origin=Origin(xyz=(arm_tip_x, 0.0, 0.016)),
            material=aged_brass,
            name="bobeche",
        )
        arm.visual(
            Cylinder(radius=0.018, length=0.024),
            origin=Origin(xyz=(arm_tip_x, 0.0, 0.030)),
            material=bronze_shadow,
            name="socket_body",
        )
        arm.visual(
            Cylinder(radius=0.012, length=0.080),
            origin=Origin(xyz=(arm_tip_x, 0.0, 0.082)),
            material=candle_ivory,
            name="candle_sleeve",
        )
        arm.visual(
            shade_mesh,
            origin=Origin(xyz=(arm_tip_x, 0.0, 0.024)),
            material=frosted_glass,
            name="shade_shell",
        )
        arm.inertial = Inertial.from_geometry(
            Box((0.40, 0.18, 0.24)),
            mass=0.85,
            origin=Origin(xyz=(0.210, 0.0, -0.010)),
        )

        angle = 2.0 * pi * index / 5.0
        hinge_x, hinge_y = _polar_xy(hinge_radius, angle)
        model.articulation(
            f"arm_{index}_hinge",
            ArticulationType.REVOLUTE,
            parent=body,
            child=arm,
            origin=Origin(xyz=(hinge_x, hinge_y, hub_z), rpy=(0.0, 0.0, angle)),
            axis=(0.0, 1.0, 0.0),
            motion_limits=MotionLimits(effort=8.0, velocity=1.2, lower=-0.45, upper=0.65),
        )

    return model


def run_tests() -> TestReport:
    ctx = TestContext(object_model)
    body = object_model.get_part("body")
    arms = [object_model.get_part(f"arm_{index}") for index in range(5)]
    hinges = [object_model.get_articulation(f"arm_{index}_hinge") for index in range(5)]
    hangers = [body.get_visual(f"hub_hanger_{index}") for index in range(5)]
    root_mounts = [arm.get_visual("root_mount") for arm in arms]

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

    for index, (arm, hinge, hanger, root_mount) in enumerate(zip(arms, hinges, hangers, root_mounts)):
        ctx.expect_contact(
            arm,
            body,
            elem_a=root_mount,
            elem_b=hanger,
            contact_tol=1e-6,
            name=f"arm_{index}_mounted_to_hub",
        )
        ctx.check(
            f"arm_{index}_hinge_axis",
            tuple(hinge.axis) == (0.0, 1.0, 0.0),
            f"expected local hinge axis (0, 1, 0), got {hinge.axis}",
        )
        limits = hinge.motion_limits
        ctx.check(
            f"arm_{index}_hinge_limits",
            limits is not None
            and limits.lower is not None
            and limits.upper is not None
            and limits.lower < 0.0 < limits.upper,
            f"expected hinge limits spanning rest pose, got {limits}",
        )

    with ctx.pose({hinges[0]: 0.45, hinges[2]: -0.25, hinges[4]: 0.30}):
        ctx.fail_if_parts_overlap_in_current_pose(name="adjusted_arm_pose_overlap_free")

    return ctx.report()


# >>> USER_CODE_END

object_model = build_object_model()
