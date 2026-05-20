from __future__ import annotations

# User code should import every SDK/stdlib symbol it uses instead of relying on
# hidden scaffold imports.
# >>> USER_CODE_START
import math

from sdk import (
    ArticulatedObject,
    ArticulationType,
    Box,
    BoxGeometry,
    Cylinder,
    CylinderGeometry,
    ExtrudeGeometry,
    Inertial,
    LatheGeometry,
    MotionLimits,
    Origin,
    TestContext,
    TestReport,
    mesh_from_geometry,
    rounded_rect_profile,
    section_loft,
    tube_from_spline_points,
)


def build_object_model() -> ArticulatedObject:
    model = ArticulatedObject(name="premium_countertop_stand_mixer")

    enamel = model.material("enamel", rgba=(0.93, 0.93, 0.90, 1.0))
    steel = model.material("steel", rgba=(0.82, 0.84, 0.86, 1.0))
    dark_trim = model.material("dark_trim", rgba=(0.14, 0.14, 0.15, 1.0))
    satin = model.material("satin", rgba=(0.62, 0.64, 0.67, 1.0))

    def xy_section(
        width: float,
        depth: float,
        radius: float,
        z: float,
        *,
        x_center: float = 0.0,
        y_center: float = 0.0,
    ) -> list[tuple[float, float, float]]:
        return [
            (x_center + x, y_center + y, z)
            for x, y in rounded_rect_profile(width, depth, radius)
        ]

    def yz_section(
        x: float,
        width: float,
        height: float,
        radius: float,
        *,
        z_center: float = 0.0,
    ) -> list[tuple[float, float, float]]:
        return [
            (x, y, z_center + z)
            for y, z in rounded_rect_profile(width, height, radius)
        ]

    base = model.part("base")

    heel_mesh = mesh_from_geometry(
        ExtrudeGeometry(rounded_rect_profile(0.34, 0.23, 0.050), 0.050),
        "base_heel",
    )
    base.visual(
        heel_mesh,
        origin=Origin(xyz=(0.120, 0.0, 0.025)),
        material=enamel,
        name="heel_shell",
    )

    tower_mesh = mesh_from_geometry(
        section_loft(
            [
                xy_section(0.145, 0.120, 0.032, 0.050, x_center=-0.006),
                xy_section(0.132, 0.110, 0.031, 0.145, x_center=-0.004),
                xy_section(0.112, 0.096, 0.028, 0.235, x_center=0.002),
                xy_section(0.105, 0.092, 0.028, 0.293, x_center=0.012),
            ]
        ),
        "base_tower",
    )
    base.visual(tower_mesh, material=enamel, name="tower_shell")

    base.visual(
        Cylinder(radius=0.016, length=0.030),
        origin=Origin(xyz=(0.012, -0.044, 0.293), rpy=(math.pi / 2.0, 0.0, 0.0)),
        material=enamel,
        name="hinge_boss_left",
    )
    base.visual(
        Cylinder(radius=0.016, length=0.030),
        origin=Origin(xyz=(0.012, 0.044, 0.293), rpy=(math.pi / 2.0, 0.0, 0.0)),
        material=enamel,
        name="hinge_boss_right",
    )
    base.visual(
        Box((0.160, 0.018, 0.002)),
        origin=Origin(xyz=(0.218, -0.046, 0.051)),
        material=dark_trim,
        name="slide_rail_left",
    )
    base.visual(
        Box((0.160, 0.018, 0.002)),
        origin=Origin(xyz=(0.218, 0.046, 0.051)),
        material=dark_trim,
        name="slide_rail_right",
    )
    base.visual(
        Cylinder(radius=0.011, length=0.018),
        origin=Origin(xyz=(0.004, 0.051, 0.186), rpy=(math.pi / 2.0, 0.0, 0.0)),
        material=dark_trim,
        name="speed_pod",
    )
    base.visual(
        Box((0.024, 0.012, 0.016)),
        origin=Origin(xyz=(0.004, -0.044, 0.270)),
        material=dark_trim,
        name="lock_guide",
    )

    base.inertial = Inertial.from_geometry(
        Box((0.34, 0.23, 0.31)),
        mass=9.2,
        origin=Origin(xyz=(0.105, 0.0, 0.155)),
    )

    bowl = model.part("bowl")

    bowl.visual(
        Box((0.190, 0.130, 0.008)),
        origin=Origin(xyz=(0.0, 0.0, 0.004)),
        material=satin,
        name="carriage_plate",
    )
    bowl.visual(
        Box((0.160, 0.024, 0.004)),
        origin=Origin(xyz=(0.0, -0.046, -0.002)),
        material=dark_trim,
        name="carriage_runner_left",
    )
    bowl.visual(
        Box((0.160, 0.024, 0.004)),
        origin=Origin(xyz=(0.0, 0.046, -0.002)),
        material=dark_trim,
        name="carriage_runner_right",
    )
    bowl.visual(
        Cylinder(radius=0.038, length=0.030),
        origin=Origin(xyz=(0.0, 0.0, 0.015)),
        material=satin,
        name="bowl_pedestal",
    )
    bowl.visual(
        Cylinder(radius=0.050, length=0.010),
        origin=Origin(xyz=(0.0, 0.0, 0.033)),
        material=satin,
        name="bowl_saddle",
    )

    bowl_shell_mesh = mesh_from_geometry(
        LatheGeometry.from_shell_profiles(
            [
                (0.020, 0.000),
                (0.055, 0.010),
                (0.098, 0.048),
                (0.108, 0.102),
                (0.104, 0.138),
                (0.113, 0.144),
            ],
            [
                (0.000, 0.004),
                (0.047, 0.015),
                (0.088, 0.050),
                (0.097, 0.103),
                (0.094, 0.135),
            ],
            segments=48,
            lip_samples=8,
        ),
        "mixing_bowl",
    )
    bowl.visual(
        bowl_shell_mesh,
        origin=Origin(xyz=(0.0, 0.0, 0.028)),
        material=steel,
        name="bowl_shell",
    )

    bowl.inertial = Inertial.from_geometry(
        Cylinder(radius=0.113, length=0.172),
        mass=1.3,
        origin=Origin(xyz=(0.0, 0.0, 0.090)),
    )

    head = model.part("head")

    head_shell_mesh = mesh_from_geometry(
        section_loft(
            [
                yz_section(0.042, 0.104, 0.090, 0.026, z_center=0.060),
                yz_section(0.118, 0.146, 0.122, 0.042, z_center=0.076),
                yz_section(0.198, 0.138, 0.130, 0.038, z_center=0.058),
                yz_section(0.276, 0.104, 0.102, 0.026, z_center=0.028),
            ]
        ),
        "head_shell",
    )
    head.visual(head_shell_mesh, material=enamel, name="head_shell")
    head.visual(
        Box((0.040, 0.044, 0.018)),
        origin=Origin(xyz=(0.022, 0.0, 0.010)),
        material=enamel,
        name="head_hinge_cheek",
    )
    head.visual(
        Cylinder(radius=0.028, length=0.038),
        origin=Origin(xyz=(0.218, 0.0, -0.026)),
        material=enamel,
        name="nose_collar",
    )

    head.inertial = Inertial.from_geometry(
        Box((0.28, 0.15, 0.16)),
        mass=4.6,
        origin=Origin(xyz=(0.150, 0.0, 0.050)),
    )

    whisk = model.part("whisk")

    whisk_geom = CylinderGeometry(radius=0.0050, height=0.060).translate(0.0, 0.0, -0.030)
    whisk_geom.merge(CylinderGeometry(radius=0.011, height=0.030).translate(0.0, 0.0, -0.072))
    whisk_geom.merge(CylinderGeometry(radius=0.017, height=0.018).translate(0.0, 0.0, -0.095))

    wire_count = 6
    for i in range(wire_count):
        angle = (2.0 * math.pi * i) / wire_count
        c = math.cos(angle)
        s = math.sin(angle)
        wire = tube_from_spline_points(
            [
                (0.007 * c, 0.007 * s, -0.060),
                (0.024 * c, 0.024 * s, -0.094),
                (0.040 * c, 0.040 * s, -0.138),
                (0.0, 0.0, -0.171),
                (-0.040 * c, -0.040 * s, -0.138),
                (-0.024 * c, -0.024 * s, -0.094),
                (-0.007 * c, -0.007 * s, -0.060),
            ],
            radius=0.0014,
            samples_per_segment=10,
            radial_segments=8,
        )
        whisk_geom.merge(wire)

    whisk.visual(
        mesh_from_geometry(whisk_geom, "balloon_whisk"),
        material=steel,
        name="whisk_shell",
    )
    whisk.inertial = Inertial.from_geometry(
        Cylinder(radius=0.048, length=0.176),
        mass=0.24,
        origin=Origin(xyz=(0.0, 0.0, -0.090)),
    )

    speed_control = model.part("speed_control")

    speed_geom = (
        CylinderGeometry(radius=0.007, height=0.016)
        .rotate_x(math.pi / 2.0)
        .translate(0.0, 0.008, 0.0)
    )
    speed_geom.merge(BoxGeometry((0.034, 0.010, 0.008)).translate(0.018, 0.012, 0.006))
    speed_geom.merge(
        CylinderGeometry(radius=0.0045, height=0.020)
        .rotate_y(math.pi / 2.0)
        .translate(0.032, 0.012, 0.012)
    )
    speed_control.visual(
        mesh_from_geometry(speed_geom, "speed_lever"),
        material=dark_trim,
        name="speed_lever",
    )
    speed_control.inertial = Inertial.from_geometry(
        Box((0.040, 0.026, 0.022)),
        mass=0.08,
        origin=Origin(xyz=(0.020, 0.012, 0.009)),
    )

    head_lock = model.part("head_lock")
    head_lock.visual(
        Box((0.020, 0.012, 0.010)),
        origin=Origin(xyz=(0.0, -0.006, 0.0)),
        material=dark_trim,
        name="lock_button",
    )
    head_lock.visual(
        Box((0.010, 0.016, 0.006)),
        origin=Origin(xyz=(0.0, 0.008, 0.0)),
        material=dark_trim,
        name="lock_stem",
    )
    head_lock.inertial = Inertial.from_geometry(
        Box((0.020, 0.012, 0.010)),
        mass=0.03,
        origin=Origin(xyz=(0.0, -0.006, 0.0)),
    )

    model.articulation(
        "base_to_bowl_slide",
        ArticulationType.PRISMATIC,
        parent=base,
        child=bowl,
        origin=Origin(xyz=(0.218, 0.0, 0.056)),
        axis=(1.0, 0.0, 0.0),
        motion_limits=MotionLimits(
            effort=45.0,
            velocity=0.10,
            lower=0.0,
            upper=0.055,
        ),
    )

    model.articulation(
        "base_to_head_hinge",
        ArticulationType.REVOLUTE,
        parent=base,
        child=head,
        origin=Origin(xyz=(0.012, 0.0, 0.293)),
        axis=(0.0, -1.0, 0.0),
        motion_limits=MotionLimits(
            effort=18.0,
            velocity=1.0,
            lower=0.0,
            upper=math.radians(58.0),
        ),
    )

    model.articulation(
        "head_to_whisk_spin",
        ArticulationType.CONTINUOUS,
        parent=head,
        child=whisk,
        origin=Origin(xyz=(0.218, 0.0, -0.045)),
        axis=(0.0, 0.0, 1.0),
        motion_limits=MotionLimits(effort=6.0, velocity=18.0),
    )

    model.articulation(
        "base_to_speed_control",
        ArticulationType.REVOLUTE,
        parent=base,
        child=speed_control,
        origin=Origin(xyz=(0.004, 0.060, 0.186)),
        axis=(0.0, -1.0, 0.0),
        motion_limits=MotionLimits(
            effort=1.2,
            velocity=2.0,
            lower=-0.45,
            upper=0.35,
        ),
    )

    model.articulation(
        "base_to_head_lock",
        ArticulationType.PRISMATIC,
        parent=base,
        child=head_lock,
        origin=Origin(xyz=(0.004, -0.066, 0.270)),
        axis=(0.0, -1.0, 0.0),
        motion_limits=MotionLimits(
            effort=0.8,
            velocity=0.04,
            lower=0.0,
            upper=0.008,
        ),
    )

    return model


def run_tests() -> TestReport:
    ctx = TestContext(object_model)

    base = object_model.get_part("base")
    bowl = object_model.get_part("bowl")
    head = object_model.get_part("head")
    whisk = object_model.get_part("whisk")
    speed_control = object_model.get_part("speed_control")
    head_lock = object_model.get_part("head_lock")

    bowl_slide = object_model.get_articulation("base_to_bowl_slide")
    head_hinge = object_model.get_articulation("base_to_head_hinge")
    whisk_spin = object_model.get_articulation("head_to_whisk_spin")
    speed_joint = object_model.get_articulation("base_to_speed_control")
    lock_joint = object_model.get_articulation("base_to_head_lock")

    ctx.expect_gap(
        bowl,
        base,
        axis="z",
        positive_elem="carriage_runner_left",
        negative_elem="heel_shell",
        min_gap=0.001,
        max_gap=0.0065,
        name="bowl carriage rides just above the heel",
    )
    ctx.expect_overlap(
        whisk,
        bowl,
        axes="xy",
        elem_a="whisk_shell",
        elem_b="bowl_shell",
        min_overlap=0.070,
        name="whisk stays centered over the bowl mouth",
    )
    ctx.expect_gap(
        head,
        bowl,
        axis="z",
        positive_elem="nose_collar",
        negative_elem="bowl_shell",
        min_gap=0.018,
        max_gap=0.060,
        name="closed head clears the bowl rim",
    )

    bowl_rest = ctx.part_world_position(bowl)
    whisk_rest = ctx.part_world_position(whisk)
    speed_rest = ctx.part_element_world_aabb(speed_control, elem="speed_lever")
    lock_rest = ctx.part_element_world_aabb(head_lock, elem="lock_button")

    with ctx.pose({bowl_slide: 0.055}):
        bowl_extended = ctx.part_world_position(bowl)
        ctx.expect_overlap(
            bowl,
            base,
            axes="x",
            elem_a="carriage_plate",
            elem_b="heel_shell",
            min_overlap=0.110,
            name="extended bowl carriage remains captured by the heel",
        )
    ctx.check(
        "bowl slide moves forward",
        bowl_rest is not None
        and bowl_extended is not None
        and bowl_extended[0] > bowl_rest[0] + 0.045,
        details=f"rest={bowl_rest}, extended={bowl_extended}",
    )

    with ctx.pose({head_hinge: math.radians(58.0)}):
        whisk_open = ctx.part_world_position(whisk)
        ctx.expect_gap(
            head,
            bowl,
            axis="z",
            positive_elem="nose_collar",
            negative_elem="bowl_shell",
            min_gap=0.090,
            name="tilted head opens clearly above the bowl",
        )
    ctx.check(
        "head tilt raises the whisk",
        whisk_rest is not None
        and whisk_open is not None
        and whisk_open[2] > whisk_rest[2] + 0.080,
        details=f"rest={whisk_rest}, open={whisk_open}",
    )

    whisk_spin_rest = ctx.part_world_position(whisk)
    with ctx.pose({whisk_spin: 1.35}):
        whisk_spin_pose = ctx.part_world_position(whisk)
    ctx.check(
        "whisk spins about a fixed vertical axis",
        whisk_spin_rest is not None
        and whisk_spin_pose is not None
        and max(
            abs(whisk_spin_pose[i] - whisk_spin_rest[i])
            for i in range(3)
        )
        < 1e-6,
        details=f"rest={whisk_spin_rest}, spun={whisk_spin_pose}",
    )

    with ctx.pose({speed_joint: 0.30}):
        speed_raised = ctx.part_element_world_aabb(speed_control, elem="speed_lever")
    ctx.check(
        "speed control pivots upward on the base side",
        speed_rest is not None
        and speed_raised is not None
        and speed_raised[1][2] > speed_rest[1][2] + 0.004,
        details=f"rest={speed_rest}, raised={speed_raised}",
    )

    with ctx.pose({lock_joint: 0.008}):
        lock_extended = ctx.part_element_world_aabb(head_lock, elem="lock_button")
    ctx.check(
        "head lock button extends outward",
        lock_rest is not None
        and lock_extended is not None
        and lock_extended[0][1] < lock_rest[0][1] - 0.006,
        details=f"rest={lock_rest}, extended={lock_extended}",
    )

    return ctx.report()


# >>> USER_CODE_END

object_model = build_object_model()
