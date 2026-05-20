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
    Sphere,
    TestContext,
    TestReport,
    TorusGeometry,
    mesh_from_geometry,
)


def build_object_model() -> ArticulatedObject:
    model = ArticulatedObject(name="classroom_globe")

    stand_finish = model.material("stand_finish", rgba=(0.23, 0.19, 0.16, 1.0))
    brass = model.material("brass", rgba=(0.69, 0.56, 0.24, 1.0))
    ocean_blue = model.material("ocean_blue", rgba=(0.24, 0.48, 0.76, 1.0))
    ivory = model.material("ivory", rgba=(0.94, 0.91, 0.82, 1.0))

    globe_radius = 0.132
    trunnion_radius = 0.006
    trunnion_length = 0.017
    meridian_radius = 0.160
    meridian_tube_radius = 0.008
    ring_center_z = 0.310

    def add_cylinder_x(part, *, name, center_xyz, radius, length, material):
        part.visual(
            Cylinder(radius=radius, length=length),
            origin=Origin(xyz=center_xyz, rpy=(0.0, math.pi / 2.0, 0.0)),
            material=material,
            name=name,
        )

    def add_cylinder_z(part, *, name, center_xyz, radius, length, material):
        part.visual(
            Cylinder(radius=radius, length=length),
            origin=Origin(xyz=center_xyz),
            material=material,
            name=name,
        )

    stand = model.part("stand")
    stand.visual(
        Cylinder(radius=0.145, length=0.020),
        origin=Origin(xyz=(0.0, 0.0, 0.010)),
        material=stand_finish,
        name="base_foot",
    )
    stand.visual(
        Cylinder(radius=0.100, length=0.018),
        origin=Origin(xyz=(0.0, 0.0, 0.029)),
        material=stand_finish,
        name="pedestal_step",
    )
    stand.visual(
        Cylinder(radius=0.020, length=0.100),
        origin=Origin(xyz=(0.0, 0.0, 0.070)),
        material=stand_finish,
        name="center_column",
    )
    stand.visual(
        Box((0.360, 0.050, 0.012)),
        origin=Origin(xyz=(0.0, 0.0, 0.125)),
        material=stand_finish,
        name="fork_crown",
    )
    add_cylinder_z(
        stand,
        name="left_fork_arm",
        center_xyz=(-0.190, 0.0, 0.217),
        radius=0.012,
        length=0.186,
        material=stand_finish,
    )
    add_cylinder_z(
        stand,
        name="right_fork_arm",
        center_xyz=(0.190, 0.0, 0.217),
        radius=0.012,
        length=0.186,
        material=stand_finish,
    )

    add_cylinder_x(
        stand,
        name="left_fork_bearing",
        center_xyz=(-0.184, 0.0, ring_center_z),
        radius=0.012,
        length=0.016,
        material=ivory,
    )
    add_cylinder_x(
        stand,
        name="right_fork_bearing",
        center_xyz=(0.184, 0.0, ring_center_z),
        radius=0.012,
        length=0.016,
        material=ivory,
    )
    stand.inertial = Inertial.from_geometry(
        Box((0.400, 0.120, 0.322)),
        mass=3.8,
        origin=Origin(xyz=(0.0, 0.0, 0.161)),
    )

    meridian = model.part("meridian")
    meridian_ring = TorusGeometry(radius=meridian_radius, tube=meridian_tube_radius)
    meridian_ring.rotate_x(math.pi / 2.0)
    meridian.visual(
        mesh_from_geometry(meridian_ring, "meridian_ring"),
        material=brass,
        name="meridian_ring",
    )
    add_cylinder_x(
        meridian,
        name="left_pivot_boss",
        center_xyz=(-0.168, 0.0, 0.0),
        radius=0.011,
        length=0.016,
        material=brass,
    )
    add_cylinder_x(
        meridian,
        name="right_pivot_boss",
        center_xyz=(0.168, 0.0, 0.0),
        radius=0.011,
        length=0.016,
        material=brass,
    )
    add_cylinder_z(
        meridian,
        name="top_bearing",
        center_xyz=(0.0, 0.0, 0.156),
        radius=0.009,
        length=0.014,
        material=ivory,
    )
    add_cylinder_z(
        meridian,
        name="bottom_bearing",
        center_xyz=(0.0, 0.0, -0.156),
        radius=0.009,
        length=0.014,
        material=ivory,
    )
    meridian.inertial = Inertial.from_geometry(
        Cylinder(radius=0.170, length=0.022),
        mass=0.8,
        origin=Origin(),
    )

    globe = model.part("globe")
    globe.visual(
        Sphere(radius=globe_radius),
        material=ocean_blue,
        name="globe_shell",
    )
    globe_equator = TorusGeometry(radius=0.131, tube=0.003)
    globe.visual(
        mesh_from_geometry(globe_equator, "globe_equator_band_v1"),
        material=ivory,
        name="equator_band",
    )
    add_cylinder_z(
        globe,
        name="axis_spindle",
        center_xyz=(0.0, 0.0, 0.0),
        radius=0.003,
        length=2.0 * (globe_radius + trunnion_length),
        material=ivory,
    )
    add_cylinder_z(
        globe,
        name="north_trunnion",
        center_xyz=(0.0, 0.0, globe_radius + trunnion_length / 2.0),
        radius=trunnion_radius,
        length=trunnion_length,
        material=ivory,
    )
    add_cylinder_z(
        globe,
        name="south_trunnion",
        center_xyz=(0.0, 0.0, -(globe_radius + trunnion_length / 2.0)),
        radius=trunnion_radius,
        length=trunnion_length,
        material=ivory,
    )
    globe.inertial = Inertial.from_geometry(
        Sphere(radius=globe_radius),
        mass=1.2,
        origin=Origin(),
    )

    model.articulation(
        "stand_to_meridian",
        ArticulationType.REVOLUTE,
        parent=stand,
        child=meridian,
        origin=Origin(xyz=(0.0, 0.0, ring_center_z)),
        axis=(1.0, 0.0, 0.0),
        motion_limits=MotionLimits(
            effort=6.0,
            velocity=1.5,
            lower=-0.55,
            upper=0.80,
        ),
    )
    model.articulation(
        "meridian_to_globe",
        ArticulationType.CONTINUOUS,
        parent=meridian,
        child=globe,
        origin=Origin(),
        axis=(0.0, 0.0, 1.0),
        motion_limits=MotionLimits(effort=1.0, velocity=3.0),
    )

    return model


def run_tests() -> TestReport:
    ctx = TestContext(object_model)
    stand = object_model.get_part("stand")
    meridian = object_model.get_part("meridian")
    globe = object_model.get_part("globe")

    meridian_tilt = object_model.get_articulation("stand_to_meridian")
    globe_spin = object_model.get_articulation("meridian_to_globe")

    left_fork_bearing = stand.get_visual("left_fork_bearing")
    right_fork_bearing = stand.get_visual("right_fork_bearing")
    left_pivot_boss = meridian.get_visual("left_pivot_boss")
    right_pivot_boss = meridian.get_visual("right_pivot_boss")
    top_bearing = meridian.get_visual("top_bearing")
    bottom_bearing = meridian.get_visual("bottom_bearing")
    globe_shell = globe.get_visual("globe_shell")
    north_trunnion = globe.get_visual("north_trunnion")
    south_trunnion = globe.get_visual("south_trunnion")

    def extent(aabb, axis_index: int) -> float:
        return aabb[1][axis_index] - aabb[0][axis_index]

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
        "stand_to_meridian_axis",
        tuple(meridian_tilt.axis) == (1.0, 0.0, 0.0),
        f"expected horizontal x-axis tilt, got {meridian_tilt.axis}",
    )
    ctx.check(
        "meridian_to_globe_axis",
        tuple(globe_spin.axis) == (0.0, 0.0, 1.0),
        f"expected north-south z-axis globe spin, got {globe_spin.axis}",
    )
    ctx.check(
        "meridian_tilt_limits_realistic",
        meridian_tilt.motion_limits is not None
        and meridian_tilt.motion_limits.lower is not None
        and meridian_tilt.motion_limits.upper is not None
        and meridian_tilt.motion_limits.lower <= -0.40
        and meridian_tilt.motion_limits.upper >= 0.70,
        "meridian tilt should provide a substantial classroom-globe adjustment range",
    )
    ctx.check(
        "globe_spin_is_continuous",
        globe_spin.joint_type == ArticulationType.CONTINUOUS,
        f"expected continuous globe spin, got {globe_spin.joint_type}",
    )

    globe_shell_aabb = ctx.part_element_world_aabb(globe, elem=globe_shell)
    meridian_ring_aabb = ctx.part_element_world_aabb(meridian, elem="meridian_ring")
    stand_aabb = ctx.part_world_aabb(stand)
    if globe_shell_aabb is not None:
        globe_diameter = extent(globe_shell_aabb, 2)
        ctx.check(
            "globe_diameter_realistic",
            0.24 <= globe_diameter <= 0.28,
            f"expected classroom globe diameter near 0.26 m, got {globe_diameter:.3f} m",
        )
    if meridian_ring_aabb is not None:
        meridian_diameter = extent(meridian_ring_aabb, 2)
        ctx.check(
            "meridian_diameter_realistic",
            0.31 <= meridian_diameter <= 0.35,
            f"expected full meridian diameter near 0.34 m, got {meridian_diameter:.3f} m",
        )
    if stand_aabb is not None:
        stand_height = extent(stand_aabb, 2)
        ctx.check(
            "stand_height_realistic",
            0.30 <= stand_height <= 0.34,
            f"expected pedestal stand height near 0.32 m, got {stand_height:.3f} m",
        )

    with ctx.pose({meridian_tilt: 0.0, globe_spin: 0.0}):
        ctx.expect_contact(
            stand,
            meridian,
            elem_a=left_fork_bearing,
            elem_b=left_pivot_boss,
            name="left_fork_contacts_left_pivot",
        )
        ctx.expect_contact(
            stand,
            meridian,
            elem_a=right_fork_bearing,
            elem_b=right_pivot_boss,
            name="right_fork_contacts_right_pivot",
        )
        ctx.expect_contact(
            globe,
            meridian,
            elem_a=north_trunnion,
            elem_b=top_bearing,
            name="north_trunnion_contacts_top_bearing",
        )
        ctx.expect_contact(
            globe,
            meridian,
            elem_a=south_trunnion,
            elem_b=bottom_bearing,
            name="south_trunnion_contacts_bottom_bearing",
        )
        ctx.expect_overlap(
            globe,
            meridian,
            axes="xz",
            min_overlap=0.24,
            elem_a=globe_shell,
            elem_b="meridian_ring",
            name="globe_reads_inside_meridian",
        )

    with ctx.pose({globe_spin: 1.2}):
        ctx.expect_contact(
            globe,
            meridian,
            elem_a=north_trunnion,
            elem_b=top_bearing,
            name="north_trunnion_contacts_when_spun",
        )
        ctx.expect_contact(
            globe,
            meridian,
            elem_a=south_trunnion,
            elem_b=bottom_bearing,
            name="south_trunnion_contacts_when_spun",
        )

    limits = meridian_tilt.motion_limits
    if limits is not None and limits.lower is not None and limits.upper is not None:
        with ctx.pose({meridian_tilt: limits.lower, globe_spin: 0.7}):
            ctx.fail_if_parts_overlap_in_current_pose(name="meridian_lower_no_overlap")
            ctx.fail_if_isolated_parts(name="meridian_lower_no_floating")
            ctx.expect_contact(
                stand,
                meridian,
                elem_a=left_fork_bearing,
                elem_b=left_pivot_boss,
                name="left_pivot_contact_lower_tilt",
            )
            ctx.expect_contact(
                stand,
                meridian,
                elem_a=right_fork_bearing,
                elem_b=right_pivot_boss,
                name="right_pivot_contact_lower_tilt",
            )
        with ctx.pose({meridian_tilt: limits.upper, globe_spin: 2.2}):
            ctx.fail_if_parts_overlap_in_current_pose(name="meridian_upper_no_overlap")
            ctx.fail_if_isolated_parts(name="meridian_upper_no_floating")
            ctx.expect_contact(
                globe,
                meridian,
                elem_a=north_trunnion,
                elem_b=top_bearing,
                name="north_trunnion_contact_upper_tilt",
            )
            ctx.expect_contact(
                globe,
                meridian,
                elem_a=south_trunnion,
                elem_b=bottom_bearing,
                name="south_trunnion_contact_upper_tilt",
            )

    # For bounded REVOLUTE/PRISMATIC joints, also check at least the lower/upper
    # motion-limit poses for both no overlap and no floating. Example:
    # hinge = object_model.get_articulation("lid_hinge")
    # limits = hinge.motion_limits
    # if limits is not None and limits.lower is not None and limits.upper is not None:
    #     with ctx.pose({hinge: limits.lower}):
    #         ctx.fail_if_parts_overlap_in_current_pose(name="lid_hinge_lower_no_overlap")
    #         ctx.fail_if_isolated_parts(name="lid_hinge_lower_no_floating")
    #     with ctx.pose({hinge: limits.upper}):
    #         ctx.fail_if_parts_overlap_in_current_pose(name="lid_hinge_upper_no_overlap")
    #         ctx.fail_if_isolated_parts(name="lid_hinge_upper_no_floating")

    return ctx.report()


# >>> USER_CODE_END

object_model = build_object_model()
