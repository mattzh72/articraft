from __future__ import annotations

# User code should import every SDK/stdlib symbol it uses instead of relying on
# hidden scaffold imports.
# >>> USER_CODE_START
import math

import cadquery as cq

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
    mesh_from_cadquery,
)


LOWER_AXIS_Z = 0.095
PLATTER_RADIUS = 0.310
PLATTER_THICKNESS = 0.050
UPPER_AXIS_OFFSET = 0.180
UPPER_AXIS_Z = 0.298
ARM_PAD_TOP_Z = 0.076
ARM_SHOE_THICKNESS = 0.024


def polar_points(radius: float, count: int, phase: float = 0.0) -> list[tuple[float, float]]:
    return [
        (
            radius * math.cos(phase + (2.0 * math.pi * i / count)),
            radius * math.sin(phase + (2.0 * math.pi * i / count)),
        )
        for i in range(count)
    ]


def offset_polar_points(
    center_xy: tuple[float, float],
    radius: float,
    count: int,
    phase: float = 0.0,
) -> list[tuple[float, float]]:
    cx, cy = center_xy
    return [
        (
            cx + radius * math.cos(phase + (2.0 * math.pi * i / count)),
            cy + radius * math.sin(phase + (2.0 * math.pi * i / count)),
        )
        for i in range(count)
    ]


def disk(radius: float, z0: float, height: float) -> cq.Workplane:
    return cq.Workplane("XY").workplane(offset=z0).circle(radius).extrude(height)


def annulus(outer_radius: float, inner_radius: float, z0: float, height: float) -> cq.Workplane:
    return (
        cq.Workplane("XY")
        .workplane(offset=z0)
        .circle(outer_radius)
        .circle(inner_radius)
        .extrude(height)
    )


def rect_prism(
    size_x: float,
    size_y: float,
    z0: float,
    height: float,
    *,
    x: float = 0.0,
    y: float = 0.0,
) -> cq.Workplane:
    return (
        cq.Workplane("XY")
        .workplane(offset=z0)
        .rect(size_x, size_y)
        .extrude(height)
        .translate((x, y, 0.0))
    )


def xz_web(points: list[tuple[float, float]], y0: float, thickness: float) -> cq.Workplane:
    return cq.Workplane("XZ").workplane(offset=y0).polyline(points).close().extrude(thickness)


def hole_pattern(
    points: list[tuple[float, float]],
    radius: float,
    z0: float,
    depth: float,
) -> cq.Workplane:
    return cq.Workplane("XY").workplane(offset=z0).pushPoints(points).circle(radius).extrude(depth)


def annular_recess(
    outer_radius: float,
    inner_radius: float,
    z0: float,
    depth: float,
    *,
    x: float = 0.0,
    y: float = 0.0,
) -> cq.Workplane:
    return annulus(outer_radius, inner_radius, z0, depth).translate((x, y, 0.0))


def make_base_shape() -> cq.Workplane:
    base = disk(0.230, 0.000, 0.030)
    base = base.union(disk(0.190, 0.030, 0.025))
    base = base.union(disk(0.115, 0.055, 0.008))
    base = base.union(disk(0.082, 0.063, 0.024))
    base = base.union(disk(0.115, 0.087, 0.008))

    base = base.cut(hole_pattern(polar_points(0.175, 6, math.pi / 6.0), 0.006, 0.000, 0.110))
    base = base.cut(annular_recess(0.103, 0.072, 0.087, 0.004))

    return base


def make_lower_platter_shape() -> cq.Workplane:
    platter = disk(PLATTER_RADIUS, 0.000, 0.046)
    platter = platter.union(disk(0.135, 0.046, 0.014))
    platter = platter.union(disk(0.055, 0.060, 0.008))
    platter = platter.union(rect_prism(0.170, 0.172, 0.046, 0.022, x=0.235))
    platter = platter.union(rect_prism(0.105, 0.110, 0.046, 0.030, x=0.185))

    mount_web_profile = [
        (0.118, 0.046),
        (0.285, 0.046),
        (0.285, 0.068),
        (0.205, 0.068),
        (0.150, 0.060),
        (0.118, 0.052),
    ]
    platter = platter.union(xz_web(mount_web_profile, 0.070, 0.016))
    platter = platter.union(xz_web(mount_web_profile, -0.086, 0.016))

    platter = platter.cut(hole_pattern(polar_points(0.185, 8), 0.0065, 0.046, -0.012))
    platter = platter.cut(
        hole_pattern(polar_points(0.100, 6, math.pi / 6.0), 0.0055, 0.060, -0.014)
    )
    platter = platter.cut(
        hole_pattern(
            [(0.192, 0.050), (0.192, -0.050), (0.278, 0.050), (0.278, -0.050)],
            0.0045,
            0.068,
            -0.018,
        )
    )
    platter = platter.cut(annular_recess(0.118, 0.086, 0.000, 0.012))
    platter = platter.cut(annular_recess(0.120, 0.060, 0.060, -0.004))

    return platter


def make_bridge_arm_shape() -> cq.Workplane:
    shoe = rect_prism(0.150, 0.152, ARM_PAD_TOP_Z, ARM_SHOE_THICKNESS, x=0.235)
    column = rect_prism(0.064, 0.122, ARM_PAD_TOP_Z + ARM_SHOE_THICKNESS, 0.166, x=0.268)
    beam = rect_prism(0.140, 0.112, 0.214, 0.046, x=0.215)

    housing_collar = disk(0.090, 0.174, 0.016).translate((UPPER_AXIS_OFFSET, 0.0, 0.0))
    housing_body = disk(0.075, 0.190, 0.070).translate((UPPER_AXIS_OFFSET, 0.0, 0.0))
    housing_pilot = disk(0.042, 0.260, 0.032).translate((UPPER_AXIS_OFFSET, 0.0, 0.0))
    thrust_land = disk(0.070, 0.292, 0.006).translate((UPPER_AXIS_OFFSET, 0.0, 0.0))

    web_profile = [
        (0.160, ARM_PAD_TOP_Z + ARM_SHOE_THICKNESS),
        (0.306, ARM_PAD_TOP_Z + ARM_SHOE_THICKNESS),
        (0.306, 0.122),
        (0.268, 0.214),
        (0.192, 0.214),
        (0.160, 0.156),
    ]
    side_web_pos = xz_web(web_profile, 0.050, 0.016)
    side_web_neg = xz_web(web_profile, -0.066, 0.016)

    center_rib = (
        cq.Workplane("YZ")
        .workplane(offset=0.205)
        .polyline(
            [
                (-0.032, ARM_PAD_TOP_Z + ARM_SHOE_THICKNESS),
                (0.032, ARM_PAD_TOP_Z + ARM_SHOE_THICKNESS),
                (0.032, 0.210),
                (0.000, 0.252),
                (-0.032, 0.210),
            ]
        )
        .close()
        .extrude(0.020)
    )

    arm = shoe.union(column).union(beam)
    arm = arm.union(housing_collar).union(housing_body).union(housing_pilot).union(thrust_land)
    arm = arm.union(side_web_pos).union(side_web_neg).union(center_rib)

    shoe_holes = [
        (0.192, 0.050),
        (0.192, -0.050),
        (0.278, 0.050),
        (0.278, -0.050),
    ]
    housing_holes = offset_polar_points((UPPER_AXIS_OFFSET, 0.0), 0.052, 4, math.pi / 4.0)

    arm = arm.cut(hole_pattern(shoe_holes, 0.006, ARM_PAD_TOP_Z + ARM_SHOE_THICKNESS, -0.012))
    arm = arm.cut(hole_pattern(housing_holes, 0.0055, 0.298, -0.014))

    return arm


def make_upper_faceplate_shape() -> cq.Workplane:
    faceplate = disk(0.125, 0.000, 0.028)
    faceplate = faceplate.union(disk(0.050, 0.028, 0.010))
    faceplate = faceplate.union(disk(0.022, 0.038, 0.008))

    faceplate = faceplate.cut(annular_recess(0.078, 0.046, 0.000, 0.010))
    faceplate = faceplate.cut(annular_recess(0.095, 0.042, 0.028, -0.004))
    faceplate = faceplate.cut(
        hole_pattern(polar_points(0.075, 6, math.pi / 6.0), 0.0050, 0.028, -0.014)
    )

    return faceplate


def build_object_model() -> ArticulatedObject:
    model = ArticulatedObject(name="offset_rotary_fixture")

    model.material("base_gray", rgba=(0.29, 0.31, 0.33, 1.0))
    model.material("machined_steel", rgba=(0.58, 0.60, 0.63, 1.0))
    model.material("aluminum_face", rgba=(0.72, 0.74, 0.77, 1.0))

    base = model.part("base")
    base.visual(
        mesh_from_cadquery(make_base_shape(), "fixture_base"),
        material="base_gray",
        name="base_shell",
    )
    base.inertial = Inertial.from_geometry(
        Cylinder(radius=0.230, length=LOWER_AXIS_Z),
        mass=48.0,
        origin=Origin(xyz=(0.0, 0.0, LOWER_AXIS_Z * 0.5)),
    )

    platter = model.part("lower_platter")
    platter.visual(
        mesh_from_cadquery(make_lower_platter_shape(), "fixture_lower_platter"),
        material="machined_steel",
        name="platter_shell",
    )
    platter.inertial = Inertial.from_geometry(
        Cylinder(radius=PLATTER_RADIUS, length=PLATTER_THICKNESS),
        mass=32.0,
        origin=Origin(xyz=(0.0, 0.0, PLATTER_THICKNESS * 0.5)),
    )

    bridge_arm = model.part("bridge_arm")
    bridge_arm.visual(
        mesh_from_cadquery(make_bridge_arm_shape(), "fixture_bridge_arm"),
        material="base_gray",
        name="bridge_shell",
    )
    bridge_arm.inertial = Inertial.from_geometry(
        Box((0.180, 0.160, 0.248)),
        mass=18.0,
        origin=Origin(xyz=(0.215, 0.0, 0.174)),
    )

    upper_faceplate = model.part("upper_faceplate")
    upper_faceplate.visual(
        mesh_from_cadquery(make_upper_faceplate_shape(), "fixture_upper_faceplate"),
        material="aluminum_face",
        name="faceplate_shell",
    )
    upper_faceplate.inertial = Inertial.from_geometry(
        Cylinder(radius=0.125, length=0.046),
        mass=7.0,
        origin=Origin(xyz=(0.0, 0.0, 0.013)),
    )

    model.articulation(
        "base_to_lower_platter",
        ArticulationType.REVOLUTE,
        parent=base,
        child=platter,
        origin=Origin(xyz=(0.0, 0.0, LOWER_AXIS_Z)),
        axis=(0.0, 0.0, 1.0),
        motion_limits=MotionLimits(
            effort=120.0,
            velocity=1.2,
            lower=-math.pi,
            upper=math.pi,
        ),
    )
    model.articulation(
        "lower_platter_to_bridge_arm",
        ArticulationType.FIXED,
        parent=platter,
        child=bridge_arm,
        origin=Origin(),
    )
    model.articulation(
        "bridge_arm_to_upper_faceplate",
        ArticulationType.REVOLUTE,
        parent=bridge_arm,
        child=upper_faceplate,
        origin=Origin(xyz=(UPPER_AXIS_OFFSET, 0.0, UPPER_AXIS_Z)),
        axis=(0.0, 0.0, 1.0),
        motion_limits=MotionLimits(
            effort=60.0,
            velocity=1.8,
            lower=-math.pi,
            upper=math.pi,
        ),
    )

    return model


def run_tests() -> TestReport:
    ctx = TestContext(object_model)
    base = object_model.get_part("base")
    platter = object_model.get_part("lower_platter")
    bridge_arm = object_model.get_part("bridge_arm")
    upper_faceplate = object_model.get_part("upper_faceplate")
    lower_joint = object_model.get_articulation("base_to_lower_platter")
    upper_joint = object_model.get_articulation("bridge_arm_to_upper_faceplate")

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
        "parallel_vertical_rotary_axes",
        tuple(lower_joint.axis) == (0.0, 0.0, 1.0) and tuple(upper_joint.axis) == (0.0, 0.0, 1.0),
        details=f"lower axis={lower_joint.axis}, upper axis={upper_joint.axis}",
    )
    ctx.expect_contact(platter, base, name="lower_stage_bearing_contact")
    ctx.expect_origin_gap(
        platter,
        base,
        axis="z",
        min_gap=LOWER_AXIS_Z,
        max_gap=LOWER_AXIS_Z,
        name="lower_stage_axis_height",
    )
    ctx.expect_contact(bridge_arm, platter, name="bridge_arm_seated_on_platter")
    ctx.expect_overlap(
        bridge_arm,
        platter,
        axes="xy",
        min_overlap=0.120,
        name="bridge_arm_footprint_supported",
    )
    ctx.expect_contact(upper_faceplate, bridge_arm, name="upper_stage_bearing_contact")
    ctx.expect_origin_distance(
        upper_faceplate,
        platter,
        axes="xy",
        min_dist=0.160,
        max_dist=0.200,
        name="upper_stage_lateral_offset",
    )
    ctx.check(
        "upper_axis_elevated_above_platter",
        0.280 <= upper_joint.origin.xyz[2] <= 0.320,
        details=f"upper joint z offset={upper_joint.origin.xyz[2]:.4f}",
    )

    with ctx.pose({lower_joint: 1.35, upper_joint: -1.10}):
        ctx.fail_if_parts_overlap_in_current_pose(name="nonzero_pose_clearance")
        ctx.expect_contact(upper_faceplate, bridge_arm, name="upper_stage_contact_in_pose")

    return ctx.report()


# >>> USER_CODE_END

object_model = build_object_model()
