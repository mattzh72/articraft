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
)


def _xy_section(
    width_x: float,
    depth_y: float,
    radius: float,
    z: float,
    *,
    x_shift: float = 0.0,
) -> list[tuple[float, float, float]]:
    return [(x + x_shift, y, z) for x, y in rounded_rect_profile(width_x, depth_y, radius)]


def _yz_section(
    x: float,
    width_y: float,
    height_z: float,
    radius: float,
    *,
    z_shift: float = 0.0,
) -> list[tuple[float, float, float]]:
    return [(x, y, z + z_shift) for y, z in rounded_rect_profile(width_y, height_z, radius)]


def build_object_model() -> ArticulatedObject:
    model = ArticulatedObject(name="black_steel_stand_mixer")

    satin_black = model.material("satin_black", rgba=(0.08, 0.08, 0.09, 1.0))
    dark_steel = model.material("dark_steel", rgba=(0.34, 0.36, 0.39, 1.0))
    brushed_steel = model.material("brushed_steel", rgba=(0.82, 0.84, 0.86, 1.0))
    shadow_black = model.material("shadow_black", rgba=(0.14, 0.14, 0.15, 1.0))

    base = model.part("base")

    deck = ExtrudeGeometry(rounded_rect_profile(0.36, 0.24, 0.055), 0.05).translate(0.04, 0.0, 0.025)
    shoulder = BoxGeometry((0.10, 0.072, 0.040)).translate(0.020, 0.0, 0.070)
    pedestal = section_loft(
        [
            _xy_section(0.14, 0.16, 0.040, 0.05, x_shift=-0.025),
            _xy_section(0.11, 0.140, 0.034, 0.19, x_shift=-0.035),
            _xy_section(0.070, 0.108, 0.026, 0.345, x_shift=-0.045),
        ]
    )
    hinge_tower = BoxGeometry((0.032, 0.110, 0.040)).translate(-0.075, 0.0, 0.365)
    hinge_saddle = (
        CylinderGeometry(radius=0.015, height=0.110)
        .rotate_x(math.pi / 2.0)
        .translate(-0.075, 0.0, 0.378)
    )
    speed_boss = (
        CylinderGeometry(radius=0.016, height=0.012)
        .rotate_x(math.pi / 2.0)
        .translate(0.042, 0.114, 0.036)
    )
    lock_guide = BoxGeometry((0.048, 0.006, 0.024)).translate(-0.094, -0.057, 0.362)
    base_geom = (
        deck.merge(shoulder)
        .merge(pedestal)
        .merge(hinge_tower)
        .merge(hinge_saddle)
        .merge(speed_boss)
        .merge(lock_guide)
    )
    base.visual(mesh_from_geometry(base_geom, "base_shell"), material=satin_black, name="base_shell")
    base.inertial = Inertial.from_geometry(
        Box((0.36, 0.24, 0.35)),
        mass=8.5,
        origin=Origin(xyz=(0.02, 0.0, 0.175)),
    )

    carriage = model.part("carriage")
    carriage_geom = BoxGeometry((0.022, 0.080, 0.080)).translate(0.011, 0.0, 0.040)
    carriage_geom.merge(BoxGeometry((0.050, 0.040, 0.020)).translate(0.045, 0.0, 0.014))
    carriage_geom.merge(BoxGeometry((0.086, 0.014, 0.020)).translate(0.060, 0.028, 0.024))
    carriage_geom.merge(BoxGeometry((0.086, 0.014, 0.020)).translate(0.060, -0.028, 0.024))
    carriage_geom.merge(CylinderGeometry(radius=0.018, height=0.026).translate(0.092, 0.0, 0.013))
    carriage_geom.merge(CylinderGeometry(radius=0.058, height=0.012).translate(0.110, 0.0, 0.025))
    carriage.visual(
        mesh_from_geometry(carriage_geom, "carriage_shell"),
        material=dark_steel,
        name="carriage_shell",
    )
    carriage.inertial = Inertial.from_geometry(
        Box((0.11, 0.10, 0.10)),
        mass=1.3,
        origin=Origin(xyz=(0.060, 0.0, 0.030)),
    )

    bowl = model.part("bowl")
    bowl_geom = LatheGeometry.from_shell_profiles(
        [
            (0.038, 0.000),
            (0.060, 0.008),
            (0.090, 0.045),
            (0.110, 0.120),
            (0.118, 0.185),
            (0.114, 0.190),
        ],
        [
            (0.000, 0.012),
            (0.048, 0.020),
            (0.082, 0.048),
            (0.100, 0.120),
            (0.108, 0.182),
        ],
        segments=64,
    )
    bowl.visual(mesh_from_geometry(bowl_geom, "bowl_shell"), material=brushed_steel, name="bowl_shell")
    bowl.inertial = Inertial.from_geometry(
        Cylinder(radius=0.118, length=0.190),
        mass=1.1,
        origin=Origin(xyz=(0.0, 0.0, 0.095)),
    )

    head = model.part("head")
    head_geom = section_loft(
        [
            _yz_section(0.055, 0.090, 0.060, 0.018, z_shift=0.030),
            _yz_section(0.145, 0.150, 0.132, 0.038, z_shift=0.024),
            _yz_section(0.255, 0.132, 0.124, 0.032, z_shift=0.008),
            _yz_section(0.350, 0.088, 0.096, 0.022, z_shift=-0.012),
        ]
    )
    head.visual(mesh_from_geometry(head_geom, "head_shell"), material=satin_black, name="head_shell")
    head.visual(
        Cylinder(radius=0.028, length=0.100),
        origin=Origin(xyz=(0.030, 0.0, 0.026), rpy=(math.pi / 2.0, 0.0, 0.0)),
        material=brushed_steel,
        name="hinge_cap",
    )
    head.visual(
        Cylinder(radius=0.048, length=0.030),
        origin=Origin(xyz=(0.347, 0.0, -0.028), rpy=(0.0, math.pi / 2.0, 0.0)),
        material=brushed_steel,
        name="nose_collar",
    )
    head.visual(
        Cylinder(radius=0.030, length=0.040),
        origin=Origin(xyz=(0.270, 0.0, -0.060)),
        material=shadow_black,
        name="hub_housing",
    )
    head.inertial = Inertial.from_geometry(
        Box((0.34, 0.16, 0.18)),
        mass=4.3,
        origin=Origin(xyz=(0.190, 0.0, 0.000)),
    )

    beater = model.part("beater")
    beater_geom = CylinderGeometry(radius=0.010, height=0.040).translate(0.0, 0.0, -0.020)
    beater_geom.merge(CylinderGeometry(radius=0.013, height=0.014).translate(0.0, 0.0, -0.047))
    beater_geom.merge(BoxGeometry((0.016, 0.012, 0.034)).translate(0.0, 0.0, -0.071))
    beater_geom.merge(BoxGeometry((0.056, 0.012, 0.012)).translate(0.0, 0.0, -0.080))
    beater_geom.merge(BoxGeometry((0.012, 0.012, 0.070)).translate(-0.026, 0.0, -0.118))
    beater_geom.merge(BoxGeometry((0.012, 0.012, 0.110)).translate(0.028, 0.0, -0.110))
    beater_geom.merge(BoxGeometry((0.012, 0.012, 0.050)).translate(0.0, 0.0, -0.130))
    beater_geom.merge(BoxGeometry((0.060, 0.012, 0.012)).translate(0.0, 0.0, -0.159))
    beater.visual(
        mesh_from_geometry(beater_geom, "beater_shell"),
        material=dark_steel,
        name="beater_shell",
    )
    beater.inertial = Inertial.from_geometry(
        Box((0.06, 0.012, 0.17)),
        mass=0.35,
        origin=Origin(xyz=(0.0, 0.0, -0.085)),
    )

    speed_control = model.part("speed_control")
    speed_control_geom = (
        CylinderGeometry(radius=0.015, height=0.012)
        .rotate_x(math.pi / 2.0)
        .translate(0.0, 0.006, 0.0)
    )
    speed_control_geom.merge(BoxGeometry((0.028, 0.010, 0.008)).translate(0.012, 0.011, 0.0))
    speed_control.visual(
        mesh_from_geometry(speed_control_geom, "speed_control_shell"),
        material=brushed_steel,
        name="speed_control_shell",
    )
    speed_control.inertial = Inertial.from_geometry(
        Box((0.043, 0.016, 0.030)),
        mass=0.08,
        origin=Origin(xyz=(0.006, 0.008, 0.0)),
    )

    lock_release = model.part("lock_release")
    lock_release_geom = BoxGeometry((0.030, 0.008, 0.012)).translate(0.015, -0.004, 0.006)
    lock_release_geom.merge(BoxGeometry((0.010, 0.010, 0.016)).translate(0.026, -0.005, 0.008))
    lock_release.visual(
        mesh_from_geometry(lock_release_geom, "lock_release_shell"),
        material=brushed_steel,
        name="lock_release_shell",
    )
    lock_release.inertial = Inertial.from_geometry(
        Box((0.036, 0.010, 0.016)),
        mass=0.06,
        origin=Origin(xyz=(0.018, -0.005, 0.008)),
    )

    model.articulation(
        "base_to_carriage",
        ArticulationType.PRISMATIC,
        parent=base,
        child=carriage,
        origin=Origin(xyz=(0.070, 0.0, 0.065)),
        axis=(0.0, 0.0, 1.0),
        motion_limits=MotionLimits(effort=40.0, velocity=0.10, lower=0.0, upper=0.040),
    )

    model.articulation(
        "carriage_to_bowl",
        ArticulationType.FIXED,
        parent=carriage,
        child=bowl,
        origin=Origin(xyz=(0.132, 0.0, 0.031)),
    )

    model.articulation(
        "base_to_head",
        ArticulationType.REVOLUTE,
        parent=base,
        child=head,
        origin=Origin(xyz=(-0.105, 0.0, 0.395)),
        axis=(0.0, -1.0, 0.0),
        motion_limits=MotionLimits(
            effort=20.0,
            velocity=1.2,
            lower=0.0,
            upper=math.radians(63.0),
        ),
    )

    model.articulation(
        "head_to_beater",
        ArticulationType.CONTINUOUS,
        parent=head,
        child=beater,
        origin=Origin(xyz=(0.307, 0.0, -0.080)),
        axis=(0.0, 0.0, 1.0),
        motion_limits=MotionLimits(effort=4.0, velocity=18.0),
    )

    model.articulation(
        "base_to_speed_control",
        ArticulationType.REVOLUTE,
        parent=base,
        child=speed_control,
        origin=Origin(xyz=(0.042, 0.120, 0.036)),
        axis=(0.0, 1.0, 0.0),
        motion_limits=MotionLimits(
            effort=0.6,
            velocity=4.0,
            lower=-0.7,
            upper=0.7,
        ),
    )

    model.articulation(
        "base_to_lock_release",
        ArticulationType.PRISMATIC,
        parent=base,
        child=lock_release,
        origin=Origin(xyz=(-0.118, -0.060, 0.356)),
        axis=(1.0, 0.0, 0.0),
        motion_limits=MotionLimits(
            effort=8.0,
            velocity=0.05,
            lower=0.0,
            upper=0.010,
        ),
    )

    return model


def run_tests() -> TestReport:
    ctx = TestContext(object_model)
    base = object_model.get_part("base")
    carriage = object_model.get_part("carriage")
    bowl = object_model.get_part("bowl")
    head = object_model.get_part("head")
    beater = object_model.get_part("beater")
    speed_control = object_model.get_part("speed_control")
    lock_release = object_model.get_part("lock_release")

    carriage_slide = object_model.get_articulation("base_to_carriage")
    head_hinge = object_model.get_articulation("base_to_head")
    beater_spin = object_model.get_articulation("head_to_beater")
    speed_joint = object_model.get_articulation("base_to_speed_control")
    lock_joint = object_model.get_articulation("base_to_lock_release")

    ctx.check(
        "bowl carriage is prismatic from the base",
        carriage_slide.articulation_type == ArticulationType.PRISMATIC and carriage_slide.parent == base.name,
        details=f"type={carriage_slide.articulation_type}, parent={carriage_slide.parent}",
    )
    ctx.check(
        "head opens upward on a rear hinge",
        head_hinge.articulation_type == ArticulationType.REVOLUTE and head_hinge.parent == base.name,
        details=f"type={head_hinge.articulation_type}, parent={head_hinge.parent}",
    )
    ctx.check(
        "beater spins continuously beneath the head",
        beater_spin.articulation_type == ArticulationType.CONTINUOUS and beater_spin.parent == head.name,
        details=f"type={beater_spin.articulation_type}, parent={beater_spin.parent}",
    )
    ctx.check(
        "speed control rotates on the base",
        speed_joint.articulation_type == ArticulationType.REVOLUTE and speed_joint.parent == base.name,
        details=f"type={speed_joint.articulation_type}, parent={speed_joint.parent}",
    )
    ctx.check(
        "head lock release slides on the base",
        lock_joint.articulation_type == ArticulationType.PRISMATIC and lock_joint.parent == base.name,
        details=f"type={lock_joint.articulation_type}, parent={lock_joint.parent}",
    )
    ctx.expect_contact(bowl, carriage, contact_tol=0.0015, name="bowl seats on the carriage")
    ctx.expect_origin_gap(bowl, base, axis="x", min_gap=0.14, name="bowl sits forward of the base origin")
    ctx.expect_overlap(beater, bowl, axes="xy", min_overlap=0.02, name="beater is centered over the bowl")

    bowl_rest = ctx.part_world_position(bowl)
    head_rest = ctx.part_world_aabb(head)
    beater_rest = ctx.part_element_world_aabb(beater, elem="beater_shell")
    speed_rest = ctx.part_element_world_aabb(speed_control, elem="speed_control_shell")
    lock_rest = ctx.part_world_position(lock_release)
    with ctx.pose({carriage_slide: carriage_slide.motion_limits.upper}):
        bowl_raised = ctx.part_world_position(bowl)
    with ctx.pose({head_hinge: head_hinge.motion_limits.upper}):
        head_open = ctx.part_world_aabb(head)
    with ctx.pose({beater_spin: math.pi / 2.0}):
        beater_turned = ctx.part_element_world_aabb(beater, elem="beater_shell")
    with ctx.pose({speed_joint: 0.7}):
        speed_turned = ctx.part_element_world_aabb(speed_control, elem="speed_control_shell")
    with ctx.pose({lock_joint: lock_joint.motion_limits.upper}):
        lock_extended = ctx.part_world_position(lock_release)

    ctx.check(
        "carriage raises the bowl",
        bowl_rest is not None and bowl_raised is not None and bowl_raised[2] > bowl_rest[2] + 0.03,
        details=f"rest={bowl_rest}, raised={bowl_raised}",
    )
    ctx.check(
        "tilt head lifts when opened",
        head_rest is not None and head_open is not None and head_open[1][2] > head_rest[1][2] + 0.10,
        details=f"rest={head_rest}, open={head_open}",
    )
    ctx.check(
        "flat beater rotates around its spindle",
        (
            beater_rest is not None
            and beater_turned is not None
            and (beater_rest[1][0] - beater_rest[0][0]) > (beater_turned[1][0] - beater_turned[0][0]) + 0.02
            and (beater_turned[1][1] - beater_turned[0][1]) > (beater_rest[1][1] - beater_rest[0][1]) + 0.02
        ),
        details=f"rest={beater_rest}, turned={beater_turned}",
    )
    ctx.check(
        "speed control visibly rotates",
        (
            speed_rest is not None
            and speed_turned is not None
            and abs(speed_turned[1][2] - speed_rest[1][2]) > 0.006
        ),
        details=f"rest={speed_rest}, turned={speed_turned}",
    )
    ctx.check(
        "lock release slides forward",
        lock_rest is not None and lock_extended is not None and lock_extended[0] > lock_rest[0] + 0.008,
        details=f"rest={lock_rest}, extended={lock_extended}",
    )

    return ctx.report()


# >>> USER_CODE_END

object_model = build_object_model()
