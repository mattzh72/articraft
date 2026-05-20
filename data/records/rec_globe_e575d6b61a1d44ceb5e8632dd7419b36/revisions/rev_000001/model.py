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
    model = ArticulatedObject(name="minimalist_globe")

    matte_black = model.material("matte_black", rgba=(0.11, 0.11, 0.12, 1.0))
    graphite = model.material("graphite", rgba=(0.22, 0.23, 0.25, 1.0))
    brushed_steel = model.material("brushed_steel", rgba=(0.72, 0.74, 0.76, 1.0))
    ocean_blue = model.material("ocean_blue", rgba=(0.22, 0.44, 0.67, 1.0))
    land_sage = model.material("land_sage", rgba=(0.62, 0.70, 0.58, 1.0))

    tilt = math.radians(24.0)
    globe_radius = 0.10
    visible_spindle = 0.018

    base_ring = model.part("base_ring")
    base_ring.visual(
        mesh_from_geometry(
            TorusGeometry(
                radius=0.090,
                tube=0.012,
                radial_segments=18,
                tubular_segments=48,
            ),
            "globe_base_ring",
        ),
        origin=Origin(xyz=(0.0, 0.0, 0.012)),
        material=matte_black,
        name="outer_ring",
    )
    base_ring.visual(
        Cylinder(radius=0.028, length=0.014),
        origin=Origin(xyz=(0.0, 0.0, 0.007)),
        material=graphite,
        name="base_hub",
    )
    for index, angle in enumerate((0.0, math.pi / 2.0, math.pi, 3.0 * math.pi / 2.0)):
        base_ring.visual(
            Box((0.056, 0.012, 0.006)),
            origin=Origin(xyz=(0.055, 0.0, 0.008), rpy=(0.0, 0.0, angle)),
            material=graphite,
            name=f"bridge_{index}",
        )
    base_ring.inertial = Inertial.from_geometry(
        Box((0.22, 0.22, 0.03)),
        mass=1.6,
        origin=Origin(xyz=(0.0, 0.0, 0.015)),
    )

    pedestal_arm = model.part("pedestal_arm")
    pedestal_arm.visual(
        Cylinder(radius=0.052, length=0.018),
        origin=Origin(xyz=(0.0, 0.0, 0.009)),
        material=graphite,
        name="pedestal_disk",
    )
    pedestal_arm.visual(
        Cylinder(radius=0.030, length=0.010),
        origin=Origin(xyz=(0.0, 0.0, 0.023)),
        material=matte_black,
        name="pedestal_collar",
    )

    support_joint_xyz = (0.110, 0.0, 0.280)
    support_axis = (math.sin(tilt), 0.0, math.cos(tilt))
    support_tip_center = (
        support_joint_xyz[0] + 0.008 * support_axis[0],
        support_joint_xyz[1],
        support_joint_xyz[2] + 0.008 * support_axis[2],
    )
    arm_points = (
        (0.034, 0.0, 0.018),
        (0.156, 0.0, 0.045),
        (0.190, 0.0, 0.238),
        support_tip_center,
    )

    for name, start, end, radius in (
        ("support_foot", arm_points[0], arm_points[1], 0.0065),
        ("support_riser", arm_points[1], arm_points[2], 0.0060),
        ("support_head", arm_points[2], arm_points[3], 0.0055),
    ):
        dx = end[0] - start[0]
        dz = end[2] - start[2]
        pedestal_arm.visual(
            Cylinder(radius=radius, length=math.hypot(dx, dz)),
            origin=Origin(
                xyz=(
                    0.5 * (start[0] + end[0]),
                    0.0,
                    0.5 * (start[2] + end[2]),
                ),
                rpy=(0.0, math.atan2(dx, dz), 0.0),
            ),
            material=brushed_steel,
            name=name,
        )
    pedestal_arm.visual(
        Sphere(radius=0.008),
        origin=Origin(xyz=arm_points[1]),
        material=brushed_steel,
        name="arm_elbow_lower",
    )
    pedestal_arm.visual(
        Sphere(radius=0.0075),
        origin=Origin(xyz=arm_points[2]),
        material=brushed_steel,
        name="arm_elbow_upper",
    )
    pedestal_arm.visual(
        Sphere(radius=0.008),
        origin=Origin(xyz=support_tip_center),
        material=brushed_steel,
        name="support_tip",
    )
    pedestal_arm.inertial = Inertial.from_geometry(
        Box((0.14, 0.11, 0.31)),
        mass=0.9,
        origin=Origin(xyz=(0.055, 0.0, 0.155)),
    )

    globe = model.part("globe")
    globe.visual(
        Sphere(radius=globe_radius),
        origin=Origin(xyz=(0.0, 0.0, -(globe_radius + visible_spindle))),
        material=ocean_blue,
        name="globe_shell",
    )
    globe.visual(
        Cylinder(radius=0.007, length=0.032),
        origin=Origin(xyz=(0.0, 0.0, -0.016)),
        material=brushed_steel,
        name="polar_spindle",
    )
    globe.visual(
        Sphere(radius=0.012),
        origin=Origin(xyz=(0.066, 0.042, -0.066)),
        material=land_sage,
        name="continent_marker",
    )
    globe.inertial = Inertial.from_geometry(
        Sphere(radius=0.108),
        mass=0.7,
        origin=Origin(xyz=(0.0, 0.0, -(globe_radius + visible_spindle))),
    )

    model.articulation(
        "base_swivel",
        ArticulationType.CONTINUOUS,
        parent=base_ring,
        child=pedestal_arm,
        origin=Origin(xyz=(0.0, 0.0, 0.014)),
        axis=(0.0, 0.0, 1.0),
        motion_limits=MotionLimits(effort=12.0, velocity=2.8),
    )
    model.articulation(
        "globe_spin",
        ArticulationType.CONTINUOUS,
        parent=pedestal_arm,
        child=globe,
        origin=Origin(xyz=support_joint_xyz, rpy=(0.0, tilt, 0.0)),
        axis=(0.0, 0.0, 1.0),
        motion_limits=MotionLimits(effort=4.0, velocity=5.0),
    )

    return model


def run_tests() -> TestReport:
    ctx = TestContext(object_model)
    # `compile_model` automatically runs baseline sanity/QC:
    # - `check_model_valid()`
    # - exactly one root part
    # - `check_mesh_assets_ready()`
    # - disconnected floating-part-group detection
    # - disconnected within-part geometry-island detection
    # - current-pose real 3D overlap detection
    # Use `run_tests()` only for prompt-specific exact checks, targeted poses,
    # and explicit allowances such as `ctx.allow_overlap(...)`.
    base_ring = object_model.get_part("base_ring")
    pedestal_arm = object_model.get_part("pedestal_arm")
    globe = object_model.get_part("globe")
    base_swivel = object_model.get_articulation("base_swivel")
    globe_spin = object_model.get_articulation("globe_spin")

    ctx.check(
        "base swivel is a vertical continuous axis",
        base_swivel.joint_type == ArticulationType.CONTINUOUS
        and tuple(base_swivel.axis) == (0.0, 0.0, 1.0),
        details=f"type={base_swivel.joint_type}, axis={base_swivel.axis}",
    )
    ctx.check(
        "globe spin uses a tilted continuous polar axis",
        globe_spin.joint_type == ArticulationType.CONTINUOUS
        and tuple(globe_spin.axis) == (0.0, 0.0, 1.0)
        and abs(globe_spin.origin.rpy[1] - math.radians(24.0)) < 1e-6,
        details=f"type={globe_spin.joint_type}, axis={globe_spin.axis}, rpy={globe_spin.origin.rpy}",
    )

    with ctx.pose({base_swivel: 0.0, globe_spin: 0.0}):
        ctx.expect_contact(
            pedestal_arm,
            base_ring,
            elem_a="pedestal_disk",
            elem_b="base_hub",
            name="pedestal disk seats on base hub",
        )
        ctx.expect_contact(
            pedestal_arm,
            globe,
            elem_a="support_tip",
            elem_b="polar_spindle",
            name="support tip contacts the polar spindle",
        )
        ctx.expect_gap(
            globe,
            pedestal_arm,
            axis="z",
            min_gap=0.02,
            positive_elem="globe_shell",
            negative_elem="pedestal_disk",
            name="globe clears the pedestal assembly",
        )

    rest_pos = ctx.part_world_position(globe)
    with ctx.pose({base_swivel: math.pi / 2.0, globe_spin: 0.0}):
        quarter_turn_pos = ctx.part_world_position(globe)
    moved_on_swivel = (
        rest_pos is not None
        and quarter_turn_pos is not None
        and math.hypot(
            quarter_turn_pos[0] - rest_pos[0],
            quarter_turn_pos[1] - rest_pos[1],
        )
        > 0.14
        and abs(quarter_turn_pos[2] - rest_pos[2]) < 1e-6
    )
    ctx.check(
        "base swivel carries the globe around the vertical axis",
        moved_on_swivel,
        details=f"rest={rest_pos}, turned={quarter_turn_pos}",
    )

    def aabb_center(aabb):
        if aabb is None:
            return None
        return tuple((aabb[0][i] + aabb[1][i]) * 0.5 for i in range(3))

    with ctx.pose({base_swivel: 0.0, globe_spin: 0.0}):
        marker_rest = aabb_center(ctx.part_element_world_aabb(globe, elem="continent_marker"))
        globe_joint_rest = ctx.part_world_position(globe)
    with ctx.pose({base_swivel: 0.0, globe_spin: math.pi / 2.0}):
        marker_spun = aabb_center(ctx.part_element_world_aabb(globe, elem="continent_marker"))
        globe_joint_spun = ctx.part_world_position(globe)

    marker_moves = (
        marker_rest is not None
        and marker_spun is not None
        and globe_joint_rest is not None
        and globe_joint_spun is not None
        and math.dist(marker_rest, marker_spun) > 0.05
        and math.dist(globe_joint_rest, globe_joint_spun) < 1e-6
    )
    ctx.check(
        "globe spin rotates the globe about its own tilted polar axis",
        marker_moves,
        details=(
            f"marker_rest={marker_rest}, marker_spun={marker_spun}, "
            f"joint_rest={globe_joint_rest}, joint_spun={globe_joint_spun}"
        ),
    )

    return ctx.report()


# >>> USER_CODE_END

object_model = build_object_model()
